from __future__ import annotations
import torch
import torch.nn as nn

HIDDEN = 256   # upgraded from 128; LayerNorm+SiLU stabilise training


class ActorNetwork(nn.Module):
    """Deterministic actor for TD3 (256-wide, LayerNorm, SiLU, tanh output)."""

    def __init__(self, state_dim: int = 38, action_dim: int = 2) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, HIDDEN), nn.LayerNorm(HIDDEN), nn.SiLU(),
            nn.Linear(HIDDEN,    HIDDEN), nn.LayerNorm(HIDDEN), nn.SiLU(),
            nn.Linear(HIDDEN, action_dim), nn.Tanh(),
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self.net(state)


class CriticNetwork(nn.Module):
    """Twin Q-network for TD3 (256-wide, LayerNorm, SiLU)."""

    def __init__(self, state_dim: int = 38, action_dim: int = 2) -> None:
        super().__init__()
        in_dim = state_dim + action_dim

        def _block() -> nn.Sequential:
            return nn.Sequential(
                nn.Linear(in_dim, HIDDEN), nn.LayerNorm(HIDDEN), nn.SiLU(),
                nn.Linear(HIDDEN,  HIDDEN), nn.LayerNorm(HIDDEN), nn.SiLU(),
                nn.Linear(HIDDEN,  1),
            )

        self.q1 = _block()
        self.q2 = _block()

    def forward(
        self, state: torch.Tensor, action: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        sa = torch.cat([state, action], dim=1)
        return self.q1(sa), self.q2(sa)

    def Q1(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        sa = torch.cat([state, action], dim=1)
        return self.q1(sa)
