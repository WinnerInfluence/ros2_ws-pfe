from __future__ import annotations
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

LOG_SIG_MAX =  2
LOG_SIG_MIN = -20
HIDDEN      = 256   # upgraded from 128; LayerNorm+SiLU stabilise training


class CriticNetwork(nn.Module):
    """Twin soft Q-network for SAC (256-wide, LayerNorm, SiLU)."""

    def __init__(self, state_dim: int = 38, action_dim: int = 2) -> None:
        super().__init__()
        in_dim = state_dim + action_dim

        def _block() -> nn.Sequential:
            return nn.Sequential(
                nn.Linear(in_dim, HIDDEN), nn.LayerNorm(HIDDEN), nn.SiLU(),
                nn.Linear(HIDDEN,  HIDDEN), nn.LayerNorm(HIDDEN), nn.SiLU(),
                nn.Linear(HIDDEN,  1),
            )

        self.net_q1 = _block()
        self.net_q2 = _block()

    def forward(
        self, state: torch.Tensor, action: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        sa = torch.cat([state, action], dim=1)
        return self.net_q1(sa), self.net_q2(sa)


class ActorNetwork(nn.Module):
    """Stochastic actor for SAC (256-wide, LayerNorm, SiLU, reparameterised tanh)."""

    def __init__(self, state_dim: int = 38, action_dim: int = 2) -> None:
        super().__init__()
        self.fc1 = nn.Linear(state_dim, HIDDEN)
        self.ln1 = nn.LayerNorm(HIDDEN)
        self.fc2 = nn.Linear(HIDDEN, HIDDEN)
        self.ln2 = nn.LayerNorm(HIDDEN)
        self.mean_linear    = nn.Linear(HIDDEN, action_dim)
        self.log_std_linear = nn.Linear(HIDDEN, action_dim)

    def forward(
        self, state: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        x = F.silu(self.ln1(self.fc1(state)))
        x = F.silu(self.ln2(self.fc2(x)))
        mean    = self.mean_linear(x)
        log_std = self.log_std_linear(x).clamp(LOG_SIG_MIN, LOG_SIG_MAX)
        return mean, log_std

    def sample(
        self, state: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        mean, log_std = self.forward(state)
        std    = log_std.exp()
        normal = Normal(mean, std)
        x_t    = normal.rsample()           # reparameterisation trick
        y_t    = torch.tanh(x_t)
        log_prob = (
            normal.log_prob(x_t)
            - torch.log(1.0 - y_t.pow(2) + 1e-6)
        ).sum(dim=1, keepdim=True)
        return y_t, log_prob, torch.tanh(mean)
