"""
training_contract.py — load YAML/JSON contract and patch main_agent module globals.

Canonical file: training_contract.yaml next to this module.
Env: TRAINING_CONTRACT=/abs/path.yaml
"""
from __future__ import annotations

import hashlib
import json
import os
from typing import Any

try:
    import yaml  # type: ignore
except ImportError:
    yaml = None


def default_contract_path() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "training_contract.yaml")


def file_sha256(path: str) -> str | None:
    if not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_contract(path: str | None = None) -> dict[str, Any]:
    p = path or os.environ.get("TRAINING_CONTRACT", "").strip() or default_contract_path()
    p = os.path.abspath(os.path.expanduser(p))
    if not os.path.isfile(p):
        raise FileNotFoundError(f"Training contract not found: {p}")

    if p.endswith(".json"):
        with open(p, encoding="utf-8") as f:
            return json.load(f)

    if yaml is None:
        raise ImportError(
            "PyYAML required for .yaml contracts.  pip install pyyaml\n"
            "Or export a JSON copy of training_contract.yaml."
        )
    with open(p, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Contract root must be a mapping: {p}")
    return data


def apply_contract_to_main_agent(
    ma: Any, contract: dict[str, Any], *, contract_path: str
) -> None:
    """Patch safe_drl_nav.main_agent (or main_agent) module-level MDP constants."""
    obs = contract["observation"]
    mdp = contract["mdp"]
    wp = contract["waypoints"]

    ma.LIDAR_BINS = int(obs["lidar_bins"])
    # Derived — keep consistent
    exp_sd = int(obs.get("state_dim", ma.LIDAR_BINS + obs.get("goal_channels", 2)))
    if exp_sd != ma.LIDAR_BINS + 2:
        raise ValueError(
            f"contract state_dim {exp_sd} != lidar_bins+2 ({ma.LIDAR_BINS + 2})"
        )

    ma.DISTANCE_MULTIPLIER = float(mdp["distance_multiplier"])
    ma.STEP_PENALTY = float(mdp["step_penalty"])
    ma.GOAL_RADIUS = float(mdp["goal_radius_m"])
    ma.GOAL_REWARD = float(mdp["single_goal_reward"])
    ma.WAYPOINT_REWARD = float(mdp["waypoint_reward"])
    ma.FINAL_REWARD = float(mdp["final_waypoint_reward"])
    ma.COLLISION_TERMINAL_PENALTY = float(mdp["collision_terminal_penalty"])
    ma.SHIELD_CRITICAL_RANGE = float(mdp["shield_critical_range_m"])
    ma.SHIELD_STEP_PENALTY = float(mdp["shield_step_penalty"])
    ma.SHIELD_BACKUP_LIN = float(mdp["shield_backup_lin"])
    ma.SHIELD_TURN_ANG = float(mdp["shield_turn_ang"])

    # Proximity reward — optional fields; graceful default if absent so old
    # contracts that lack these keys still load without errors.
    ma.PROXIMITY_REWARD_SCALE = float(mdp.get("proximity_reward_scale", 0.0))
    ma.PROXIMITY_THRESHOLD = float(mdp.get("proximity_threshold_m", 1.5))

    ma.MAZE_WAYPOINTS = [tuple(map(float, xy)) for xy in wp["maze_waypoints_xy"]]

    # Stash full contract on module for manifest / debugging
    ma._ACTIVE_TRAINING_CONTRACT = contract  # type: ignore[attr-defined]
    ma._ACTIVE_TRAINING_CONTRACT_PATH = os.path.abspath(contract_path)  # type: ignore[attr-defined]


def contract_manifest_extras(contract_path: str, contract: dict[str, Any]) -> dict[str, Any]:
    """Subset embedded into train_manifest.json (frozen snapshot)."""
    sha = file_sha256(contract_path)
    return {
        "training_contract_path": contract_path,
        "training_contract_sha256": sha,
        "training_contract_version": contract.get("contract_version"),
        "training_contract_name": contract.get("contract_name"),
        "frozen_contract_snapshot": contract,
    }


def infer_actor_state_dim_from_checkpoint(state_dict: dict[str, Any]) -> int | None:
    """
    Input feature dimension for the actor's first linear layer.
    SAC / custom: fc1.weight shape [H, state_dim]
    TD3: net.0.weight shape [H, state_dim]
    """
    w = state_dict.get("fc1.weight")
    if w is not None and hasattr(w, "shape") and len(w.shape) == 2:
        return int(w.shape[1])
    w = state_dict.get("net.0.weight")
    if w is not None and hasattr(w, "shape") and len(w.shape) == 2:
        return int(w.shape[1])
    return None


def infer_actor_action_dim_from_checkpoint(state_dict: dict[str, Any]) -> int | None:
    """Best-effort: SAC mean_linear.weight [A, H]; TD3 net[-2] often Linear to action_dim."""
    w = state_dict.get("mean_linear.weight")
    if w is not None and hasattr(w, "shape") and len(w.shape) == 2:
        return int(w.shape[0])
    # TD3 last linear before Tanh is net.6.weight [2, H] in our Sequential
    for key in ("net.6.weight", "net.5.weight"):
        w = state_dict.get(key)
        if w is not None and hasattr(w, "shape") and len(w.shape) == 2:
            return int(w.shape[0])
    return None


def snapshot_network_source_fingerprint() -> dict[str, Any]:
    """Import-time architecture constants (code-defined)."""
    out: dict[str, Any] = {}
    try:
        from networks_sac import HIDDEN as SAC_H  # type: ignore

        out["sac_hidden"] = int(SAC_H)
    except Exception as e:
        out["sac_hidden_error"] = repr(e)
    try:
        from networks_td3 import HIDDEN as TD3_H  # type: ignore

        out["td3_hidden"] = int(TD3_H)
    except Exception as e:
        out["td3_hidden_error"] = repr(e)
    return out


def resolve_world_path(agent_dir: str, basename: str) -> str:
    rel = os.path.join(agent_dir, "sim_assets", "worlds", basename)
    return os.path.abspath(rel)
