"""
trainer_custom.py — Training logic for the "Personal Touch" algorithm.

===========================================================================
INTEGRATION CONTRACT (do NOT change the class name or .train() signature)
===========================================================================

  * CustomTrainer(actor, critic, memory, *, device="cpu", **kwargs)
      - train(batch_size=128)  — one gradient update step, called every
        env step (after replay warmup) by main_agent.py's training loop.

    That is the entire interface main_agent.py requires.  Everything else
    is yours to design.

===========================================================================
DEFAULT IMPLEMENTATION: TD3-style deterministic actor-critic
===========================================================================

The default below is a clean TD3-like update rule (deterministic policy
gradient + twin-critic Bellman target + soft target update).  It is a
working, sensible baseline that you can:

  a) Use as-is if you want a TD3 variant with your custom network.
  b) Swap out the update equations for your research algorithm.

Common "Personal Touch" ideas to try:
  - Prioritised Experience Replay (PER) — change memory.sample() call
  - Distributional critic   (C51 / QR-DQN style Q distribution)
  - Model-based imagination — add a learned world model, plan ahead
  - Curiosity / intrinsic reward — add an ICM module alongside
  - Asymmetric actor-critic  — larger critic, smaller actor
  - Constrained RL (safety layer) — project actions onto a safe set
"""
from __future__ import annotations

import copy
import torch
import torch.nn.functional as F


class CustomTrainer:
    """
    Default: twin-critic TD3-style update with soft target tracking.
    Replace the internals of train() with your own algorithm.
    """

    def __init__(
        self,
        actor: torch.nn.Module,
        critic: torch.nn.Module,
        memory,
        *,
        gamma: float = 0.99,
        tau: float = 0.005,
        actor_lr: float = 3e-4,
        critic_lr: float = 3e-4,
        # TD3-style target policy smoothing — set to 0 to disable.
        policy_noise: float = 0.2,
        noise_clip: float = 0.5,
        # Delay actor update every N critic steps (TD3 trick).
        policy_delay: int = 2,
        device: str | torch.device = "cpu",
    ) -> None:
        self.actor  = actor
        self.critic = critic
        self.memory = memory
        self.gamma  = gamma
        self.tau    = tau
        self.policy_noise  = policy_noise
        self.noise_clip    = noise_clip
        self.policy_delay  = policy_delay
        self.device = torch.device(device) if isinstance(device, str) else device
        self._train_step   = 0

        self.actor_target  = copy.deepcopy(actor).to(self.device)
        self.critic_target = copy.deepcopy(critic).to(self.device)
        for p in (*self.actor_target.parameters(),
                  *self.critic_target.parameters()):
            p.requires_grad_(False)

        self.actor_opt  = torch.optim.Adam(actor.parameters(),  lr=actor_lr)
        self.critic_opt = torch.optim.Adam(critic.parameters(), lr=critic_lr)

    # ---------------------------------------------------------------------- #
    # Main update — called once per env step after replay warm-up             #
    # ---------------------------------------------------------------------- #

    def train(self, batch_size: int = 128) -> None:
        """One gradient step.  Swap this body for your own update rule."""
        self._train_step += 1

        state, action, reward, next_state, done = self.memory.sample(batch_size)
        state      = state.to(self.device)
        action     = action.to(self.device)
        reward     = reward.view(-1, 1).to(self.device)
        next_state = next_state.to(self.device)
        done       = done.view(-1, 1).to(self.device)

        # ---- Critic update ---- #
        with torch.no_grad():
            # Target policy smoothing (TD3): add clipped Gaussian noise to
            # the target action so the critic cannot exploit sharp Q peaks.
            noise = (
                torch.randn_like(action) * self.policy_noise
            ).clamp(-self.noise_clip, self.noise_clip)
            next_action = (
                self.actor_target.predict(next_state) + noise
            ).clamp(-1.0, 1.0)

            q1_next, q2_next = self.critic_target(next_state, next_action)
            target_q = reward + (1.0 - done) * self.gamma * torch.min(q1_next, q2_next)

        q1, q2 = self.critic(state, action)
        critic_loss = F.mse_loss(q1, target_q) + F.mse_loss(q2, target_q)

        self.critic_opt.zero_grad()
        critic_loss.backward()
        # Gradient clipping for stability in long maze episodes.
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=10.0)
        self.critic_opt.step()

        # ---- Actor update (delayed) ---- #
        if self._train_step % self.policy_delay == 0:
            q1_pi, _ = self.critic(state, self.actor.predict(state))
            actor_loss = -q1_pi.mean()

            self.actor_opt.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=10.0)
            self.actor_opt.step()

            # ---- Soft target update ---- #
            self._soft_update(self.actor,  self.actor_target)
            self._soft_update(self.critic, self.critic_target)

    # ---------------------------------------------------------------------- #
    # Helpers                                                                 #
    # ---------------------------------------------------------------------- #

    def _soft_update(
        self, source: torch.nn.Module, target: torch.nn.Module
    ) -> None:
        for p_s, p_t in zip(source.parameters(), target.parameters()):
            p_t.data.copy_(self.tau * p_s.data + (1.0 - self.tau) * p_t.data)
