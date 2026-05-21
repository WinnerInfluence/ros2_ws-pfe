from __future__ import annotations
import copy
import torch
import torch.nn.functional as F


class TD3Trainer:
    """
    Twin Delayed DDPG (TD3) with:
      - Gradient clipping (max_norm=1.0) on both actor and critic
      - Frozen target networks (no gradient leakage through targets)
      - Configurable policy delay, target-policy smoothing noise
    """

    def __init__(
        self,
        actor,
        critic,
        memory,
        *,
        gamma:        float = 0.99,
        tau:          float = 0.005,
        lr:           float = 3e-4,
        policy_noise: float = 0.2,
        noise_clip:   float = 0.5,
        policy_delay: int   = 2,
        device: str | torch.device = "cpu",
    ) -> None:
        self.actor   = actor
        self.critic  = critic
        self.memory  = memory
        self.gamma        = gamma
        self.tau          = tau
        self.policy_noise = policy_noise
        self.noise_clip   = noise_clip
        self.policy_delay = policy_delay
        self.device = torch.device(device) if isinstance(device, str) else device

        self.actor_target  = copy.deepcopy(actor).to(self.device)
        self.critic_target = copy.deepcopy(critic).to(self.device)
        for p in (*self.actor_target.parameters(), *self.critic_target.parameters()):
            p.requires_grad_(False)

        self.actor_opt  = torch.optim.Adam(actor.parameters(),  lr=lr)
        self.critic_opt = torch.optim.Adam(critic.parameters(), lr=lr)
        self._step = 0

    def train(self, batch_size: int = 256) -> None:
        self._step += 1
        state, action, reward, next_state, done = self.memory.sample(batch_size)
        state      = state.to(self.device)
        action     = action.to(self.device)
        reward     = reward.view(-1, 1).to(self.device)
        next_state = next_state.to(self.device)
        done       = done.view(-1, 1).to(self.device)

        # ── Critic update ─────────────────────────────────────────────────
        with torch.no_grad():
            noise = (torch.randn_like(action) * self.policy_noise).clamp(
                -self.noise_clip, self.noise_clip
            )
            next_action = (self.actor_target(next_state) + noise).clamp(-1.0, 1.0)
            q1_next, q2_next = self.critic_target(next_state, next_action)
            target_Q = reward + (1.0 - done) * self.gamma * torch.min(q1_next, q2_next)

        q1, q2      = self.critic(state, action)
        critic_loss = F.mse_loss(q1, target_Q) + F.mse_loss(q2, target_Q)
        if not torch.isfinite(critic_loss):
            return  # skip corrupt batch — isolated Gazebo glitch must not update weights
        self.critic_opt.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=1.0)
        self.critic_opt.step()

        # ── Delayed actor + target update ─────────────────────────────────
        if self._step % self.policy_delay == 0:
            actor_loss = -self.critic.Q1(state, self.actor(state)).mean()
            if not torch.isfinite(actor_loss):
                return
            self.actor_opt.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=1.0)
            self.actor_opt.step()

            for p, t_p in zip(self.critic.parameters(), self.critic_target.parameters()):
                t_p.data.copy_(self.tau * p.data + (1.0 - self.tau) * t_p.data)
            for p, t_p in zip(self.actor.parameters(), self.actor_target.parameters()):
                t_p.data.copy_(self.tau * p.data + (1.0 - self.tau) * t_p.data)
