"""
networks_custom.py — "Personal Touch" algorithm: actor and critic networks.

===========================================================================
INTEGRATION CONTRACT (do NOT change these — main_agent.py depends on them)
===========================================================================

  * ActorNetwork(state_dim, action_dim)
      - forward(state_tensor)  → implementation-defined (used internally)
      - predict(state_tensor)  → action_tensor  (deterministic, clipped to [-1, 1])
        Called by: main_agent training loop (--algo custom)
                   evaluate_agent.py benchmark harness

  * CriticNetwork(state_dim, action_dim)
      - forward(state_tensor, action_tensor) → Q-value tensor  shape (B, 1)

  * Observation / action dimensions MUST stay fixed:
      state_dim  = 38   (36 LiDAR bins normalised + delta_x + delta_y)
      action_dim = 2    ([linear_vel, angular_vel], both in [-1, 1])
    Changing these will cause a dimension mismatch when loading pre-trained
    weights from Phase 1 (SAC/TD3) via --load-pretrained.

===========================================================================
OPTIONAL SAC-STYLE STOCHASTIC INTERFACE
===========================================================================

If your algorithm uses stochastic exploration (like SAC), also implement:

    def sample(self, state):
        \"\"\"Returns (action, log_prob, mean_action).\"\"\"
        ...

main_agent will call .predict() for the 'custom' branch; .sample() is
only needed if you want to reuse the SAC trainer directly.

===========================================================================
HOW TO REPLACE WITH YOUR OWN ARCHITECTURE
===========================================================================

  1. Edit the network body inside ActorNetwork and CriticNetwork below.
  2. Keep the __init__ signature: (state_dim=38, action_dim=2).
  3. Keep predict() returning a (B, action_dim) tensor in [-1, 1].
  4. Run:  python3 main_agent.py --algo custom --force-restart
"""
from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F


# ---------------------------------------------------------------------------
# Actor
# ---------------------------------------------------------------------------

class ActorNetwork(nn.Module):
    """
    Default architecture: 3-layer MLP with LayerNorm stabilisation.

    Replace the body freely — just preserve the __init__ signature and
    the predict() output contract.
    """

    def __init__(self, state_dim: int = 38, action_dim: int = 2) -> None:
        super().__init__()
        # ----- YOUR ARCHITECTURE STARTS HERE -----
        hidden = 256
        self.fc1   = nn.Linear(state_dim, hidden)
        self.ln1   = nn.LayerNorm(hidden)
        self.fc2   = nn.Linear(hidden, hidden)
        self.ln2   = nn.LayerNorm(hidden)
        self.fc3   = nn.Linear(hidden, hidden // 2)
        self.out   = nn.Linear(hidden // 2, action_dim)
        # ----- YOUR ARCHITECTURE ENDS HERE -------

        self._init_weights()

    def _init_weights(self) -> None:
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=1.0)
                nn.init.zeros_(m.bias)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        x = F.silu(self.ln1(self.fc1(state)))
        x = F.silu(self.ln2(self.fc2(x)))
        x = F.silu(self.fc3(x))
        return torch.tanh(self.out(x))

    def predict(self, state: torch.Tensor) -> torch.Tensor:
        """
        Deterministic inference — called by the training loop and
        evaluate_agent.py.  Output is clipped to [-1, 1] via tanh.
        """
        return self.forward(state)


# ---------------------------------------------------------------------------
# Critic
# ---------------------------------------------------------------------------

class CriticNetwork(nn.Module):
    """
    Twin critic (like SAC/TD3) for stability.  Returns (Q1, Q2).

    If your algorithm uses a single critic, just return the same tensor
    twice: `return q, q`.
    """

    def __init__(self, state_dim: int = 38, action_dim: int = 2) -> None:
        super().__init__()
        in_dim = state_dim + action_dim
        hidden = 256

        # Q1
        self.q1_fc1 = nn.Linear(in_dim, hidden)
        self.q1_fc2 = nn.Linear(hidden, hidden)
        self.q1_out = nn.Linear(hidden, 1)

        # Q2
        self.q2_fc1 = nn.Linear(in_dim, hidden)
        self.q2_fc2 = nn.Linear(hidden, hidden)
        self.q2_out = nn.Linear(hidden, 1)

        self._init_weights()

    def _init_weights(self) -> None:
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=1.0)
                nn.init.zeros_(m.bias)

    def forward(
        self, state: torch.Tensor, action: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        sa = torch.cat([state, action], dim=1)
        q1 = self.q1_out(F.relu(self.q1_fc2(F.relu(self.q1_fc1(sa)))))
        q2 = self.q2_out(F.relu(self.q2_fc2(F.relu(self.q2_fc1(sa)))))
        return q1, q2
