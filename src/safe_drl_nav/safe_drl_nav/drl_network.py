"""
Legacy actor/critic (128-wide ReLU) — superseded by networks_sac.py / networks_td3.py.

Kept for old scripts or checkpoints; main_agent uses networks_* by algo.
"""
from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

LOG_SIG_MAX = 2
LOG_SIG_MIN = -20
HIDDEN = 128


class CriticNetwork(nn.Module):
    def __init__(self, state_dim: int = 38, action_dim: int = 2) -> None:
        super().__init__()
        in_dim = state_dim + action_dim
        self.net_q1 = nn.Sequential(
            nn.Linear(in_dim, HIDDEN),
            nn.ReLU(),
            nn.Linear(HIDDEN, HIDDEN),
            nn.ReLU(),
            nn.Linear(HIDDEN, 1),
        )
        self.net_q2 = nn.Sequential(
            nn.Linear(in_dim, HIDDEN),
            nn.ReLU(),
            nn.Linear(HIDDEN, HIDDEN),
            nn.ReLU(),
            nn.Linear(HIDDEN, 1),
        )

    def forward(self, state: torch.Tensor, action: torch.Tensor):
        sa = torch.cat([state, action], dim=1)
        return self.net_q1(sa), self.net_q2(sa)


class ActorNetwork(nn.Module):
    def __init__(self, state_dim: int = 38, action_dim: int = 2) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, HIDDEN),
            nn.ReLU(),
            nn.Linear(HIDDEN, HIDDEN),
            nn.ReLU(),
        )
        self.mean = nn.Linear(HIDDEN, action_dim)
        self.log_std = nn.Linear(HIDDEN, action_dim)

    def forward(self, state: torch.Tensor):
        x = self.net(state)
        mean = self.mean(x)
        log_std = self.log_std(x).clamp(LOG_SIG_MIN, LOG_SIG_MAX)
        std = log_std.exp()
        return mean, std

    def sample(self, state: torch.Tensor):
        mean, std = self.forward(state)
        normal = Normal(mean, std)
        x_t = normal.rsample()
        action = torch.tanh(x_t)
        log_prob = normal.log_prob(x_t) - torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=1, keepdim=True)
        return action, log_prob
