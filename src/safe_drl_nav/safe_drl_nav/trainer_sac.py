from __future__ import annotations
import copy
import torch
import torch.nn.functional as F


class SACTrainer:
    """
    Soft Actor-Critic with:
      - Auto-entropy temperature (alpha tuned online → no manual alpha guessing)
      - Gradient clipping (max_norm=1.0) on both actor and critic
      - Frozen target critic (no gradients through targets)
    """

    def __init__(
        self,
        actor,
        critic,
        memory,
        *,
        gamma:  float = 0.99,
        tau:    float = 0.005,
        alpha:  float = 0.2,        # initial value; overridden by auto-alpha
        lr:     float = 3e-4,
        device: str | torch.device = "cpu",
        auto_alpha: bool = True,
        action_dim: int = 2,
    ) -> None:
        self.actor   = actor
        self.critic  = critic
        self.memory  = memory
        self.gamma   = gamma
        self.tau     = tau
        self.device  = torch.device(device) if isinstance(device, str) else device

        self.critic_target = copy.deepcopy(critic).to(self.device)
        for p in self.critic_target.parameters():
            p.requires_grad_(False)

        self.actor_opt  = torch.optim.Adam(actor.parameters(),  lr=lr)
        self.critic_opt = torch.optim.Adam(critic.parameters(), lr=lr)

        # Auto-entropy temperature -------------------------------------------
        self._auto_alpha     = auto_alpha
        self._target_entropy = float(-action_dim)   # standard SAC heuristic: -|A|
        if auto_alpha:
            self.log_alpha  = torch.zeros(1, requires_grad=True, device=self.device)
            self.alpha      = self.log_alpha.exp().item()
            self.alpha_opt  = torch.optim.Adam([self.log_alpha], lr=lr)
        else:
            self.alpha     = alpha
            self.log_alpha = None
            self.alpha_opt = None

    def train(self, batch_size: int = 256) -> None:
        state, action, reward, next_state, done = self.memory.sample(batch_size)
        state      = state.to(self.device)
        action     = action.to(self.device)
        reward     = reward.view(-1, 1).to(self.device)
        next_state = next_state.to(self.device)
        done       = done.view(-1, 1).to(self.device)

        # ── Critic update ─────────────────────────────────────────────────
        with torch.no_grad():
            next_action, next_log_prob, _ = self.actor.sample(next_state)
            q1_next, q2_next = self.critic_target(next_state, next_action)
            min_q_next = torch.min(q1_next, q2_next) - self.alpha * next_log_prob
            target_Q   = reward + (1.0 - done) * self.gamma * min_q_next

        q1, q2      = self.critic(state, action)
        critic_loss = F.mse_loss(q1, target_Q) + F.mse_loss(q2, target_Q)
        if not torch.isfinite(critic_loss):
            return  # skip corrupt batch — isolated Gazebo glitch must not update weights
        self.critic_opt.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=1.0)
        self.critic_opt.step()

        # ── Actor update ──────────────────────────────────────────────────
        pi, log_prob, _ = self.actor.sample(state)
        q1_pi, q2_pi    = self.critic(state, pi)
        actor_loss      = (self.alpha * log_prob - torch.min(q1_pi, q2_pi)).mean()
        if not torch.isfinite(actor_loss):
            return
        self.actor_opt.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=1.0)
        self.actor_opt.step()

        # ── Auto-alpha update ─────────────────────────────────────────────
        if self._auto_alpha and self.log_alpha is not None:
            alpha_loss = -(
                self.log_alpha * (log_prob + self._target_entropy).detach()
            ).mean()
            self.alpha_opt.zero_grad()
            alpha_loss.backward()
            self.alpha_opt.step()
            self.alpha = self.log_alpha.exp().clamp(1e-4, 1.0).item()

        # ── Soft target update ────────────────────────────────────────────
        for p, t_p in zip(self.critic.parameters(), self.critic_target.parameters()):
            t_p.data.copy_(self.tau * p.data + (1.0 - self.tau) * t_p.data)
