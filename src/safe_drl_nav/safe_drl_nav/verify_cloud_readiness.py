#!/usr/bin/env python3
"""
verify_cloud_readiness.py — fail-fast checks before cloud / headless training.

  python3 verify_cloud_readiness.py \\
      --contract training_contract.yaml \\
      --algo sac \\
      --checkpoint trained_models/sac_actor_maze.pth \\
      --manifest pfe_logs/run_manifest_XXX_sac.json

Exit code 0 = all checks passed.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import torch


def _die(msg: str) -> None:
    print(f"[FAIL] {msg}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    ap = argparse.ArgumentParser(description="Verify MDP / checkpoint / manifest parity.")
    ap.add_argument("--contract", type=str, default="", help="training_contract.yaml path")
    ap.add_argument("--algo", type=str, default="sac", help="sac | td3 | custom")
    ap.add_argument("--checkpoint", type=str, default="", help="Actor .pth path")
    ap.add_argument("--manifest", type=str, default="", help="Optional run_manifest.json")
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    if args.contract.strip():
        cpath = os.path.abspath(os.path.expanduser(args.contract.strip()))
    else:
        from training_contract import default_contract_path

        cpath = default_contract_path()

    from training_contract import (
        apply_contract_to_main_agent,
        file_sha256,
        infer_actor_action_dim_from_checkpoint,
        infer_actor_state_dim_from_checkpoint,
        load_contract,
        resolve_world_path,
        snapshot_network_source_fingerprint,
    )

    if not os.path.isfile(cpath):
        _die(f"Contract not found: {cpath}")

    contract = load_contract(cpath)

    import main_agent as ma

    apply_contract_to_main_agent(ma, contract, contract_path=cpath)

    obs = contract["observation"]
    exp_sd = int(obs["state_dim"])
    exp_ad = 2
    if ma.LIDAR_BINS + 2 != exp_sd:
        _die(f"Contract state_dim {exp_sd} != lidar_bins+2 ({ma.LIDAR_BINS}+2)")

    print(f"[OK] Contract loaded  sha256={file_sha256(cpath)}")
    print(f"[OK] Patched main_agent  LIDAR_BINS={ma.LIDAR_BINS}  GOAL_RADIUS={ma.GOAL_RADIUS}")

    worlds = contract.get("worlds", {})
    wb = worlds.get("training_default_basename", "current_random_lab.world")
    wabs = resolve_world_path(script_dir, wb)
    if os.path.isfile(wabs):
        print(f"[OK] Training world exists  {wb}  sha256={file_sha256(wabs)[:16]}…")
    else:
        print(f"[WARN] Training world missing (optional for pure unit checks): {wabs}")

    fp = snapshot_network_source_fingerprint()
    print(f"[OK] Network fingerprint  {fp}")

    if args.checkpoint.strip():
        ckpt_path = os.path.abspath(os.path.expanduser(args.checkpoint.strip()))
        if not os.path.isfile(ckpt_path):
            _die(f"Checkpoint not found: {ckpt_path}")
        sd = torch.load(ckpt_path, map_location="cpu")
        if not isinstance(sd, dict):
            _die("Checkpoint is not a state_dict mapping")

        ain = infer_actor_state_dim_from_checkpoint(sd)
        aout = infer_actor_action_dim_from_checkpoint(sd)
        if ain is None:
            _die("Cannot infer actor input dim (unexpected keys — wrong architecture?)")
        if ain != exp_sd:
            _die(f"Checkpoint actor input dim {ain} != contract state_dim {exp_sd}")
        if aout is not None and aout != exp_ad:
            _die(f"Checkpoint actor output dim {aout} != expected action_dim {exp_ad}")

        # Try structural load
        algo = args.algo.strip().lower()
        try:
            from safe_drl_nav.evaluate_agent import _make_actor
        except ImportError:
            from evaluate_agent import _make_actor
        actor = _make_actor(algo, exp_sd, exp_ad)
        try:
            actor.load_state_dict(sd)
        except Exception as exc:
            _die(f"load_state_dict failed: {exc!r}")
        print(f"[OK] Checkpoint compatible with --algo {algo!r}  ({ckpt_path})")

    if args.manifest.strip():
        mp = os.path.abspath(os.path.expanduser(args.manifest.strip()))
        if not os.path.isfile(mp):
            _die(f"Manifest not found: {mp}")
        with open(mp, encoding="utf-8") as f:
            man = json.load(f)
        bundle = man.get("training_contract_bundle") or {}
        snap = bundle.get("frozen_contract_snapshot")
        if snap:
            if snap.get("contract_version") != contract.get("contract_version"):
                print(
                    "[WARN] manifest contract_version differs from file "
                    f"(manifest={snap.get('contract_version')} file={contract.get('contract_version')})"
                )
            msha = bundle.get("training_contract_sha256")
            csha = file_sha256(cpath)
            if msha and csha and msha != csha:
                _die(f"Manifest contract SHA256 mismatch  manifest={msha[:16]}… file={csha[:16]}…")
        print(f"[OK] Manifest readable  {mp}")

    print("\nAll requested checks passed.")
    raise SystemExit(0)


if __name__ == "__main__":
    main()
