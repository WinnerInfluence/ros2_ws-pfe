"""
evaluate_agent.py — Post-training benchmark harness for thesis analysis.

Uses SafeNavAgent.reset_env() + SafeNavAgent.step_environment() from main_agent.py
so reward, shield, waypoint radii, and LiDAR logic cannot drift from training.

Run from the package directory (or train_menu.sh → 3):

    cd /path/to/ros2_ws/src/safe_drl_nav/safe_drl_nav
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 evaluate_agent.py --algo sac --episodes 20

Do not run main_agent training in parallel in another terminal: both use node
name safe_nav_agent and publish /cmd_vel.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Twist

# SafeNavAgent + contract-patched MDP constants
try:
    import safe_drl_nav.main_agent as ma
except ImportError:
    import main_agent as ma

actor_checkpoint_in_features = ma.actor_checkpoint_in_features

# Matches train_menu.sh Phase 2 waypoint training (also pass explicitly in paper runs).
DEFAULT_EVAL_WAYPOINT_GOAL_RADIUS_M = 0.68


def run_eval_episodes(
    agent: ma.SafeNavAgent,
    algo: str,
    n_episodes: int,
    max_steps: int,
    *,
    sac_use_sample: bool,
    td3_policy_noise_std: float,
    require_reset: bool,
) -> list[dict]:
    """Roll out N maze episodes using the same step_environment as training."""
    results: list[dict] = []
    n_wp = len(agent._maze_waypoints)
    device = agent.device

    for ep in range(1, n_episodes + 1):
        state = agent.reset_env()
        if require_reset and not agent._no_sim_reset and not agent._last_sim_reset_applied:
            agent.get_logger().error(
                "Strict reset required (--paper-eval / --require-reset) but "
                f"{agent._reset_service!r} was not invoked — aborting so JSON is not misleading."
            )
            raise SystemExit(2)

        ep_rew = 0.0
        ep_step = 0
        solved = False

        for _step in range(max_steps):
            st = torch.FloatTensor(state).unsqueeze(0).to(device)
            with torch.no_grad():
                if algo == "sac":
                    if sac_use_sample:
                        raw_action, _, _ = agent.actor.sample(st)
                        raw_action = raw_action.detach().cpu().numpy()[0]
                    else:
                        mean, _ = agent.actor(st)
                        raw_action = torch.tanh(mean).cpu().numpy()[0]
                elif algo == "custom":
                    raw_action = agent.actor.predict(st).detach().cpu().numpy()[0]
                else:
                    raw_action = agent.actor(st).detach().cpu().numpy()[0]
                    if td3_policy_noise_std > 0.0:
                        raw_action = np.clip(
                            raw_action + np.random.normal(0.0, td3_policy_noise_std, 2),
                            -1.0,
                            1.0,
                        )

            applied_action = np.array(
                [
                    float(np.clip((float(raw_action[0]) + 1.0) * 0.2, 0.0, 0.4)),
                    float(np.clip(float(raw_action[1]), -1.0, 1.0)),
                ],
                dtype=np.float32,
            )

            state, reward, done = agent.step_environment(applied_action)
            ep_rew += float(reward)
            ep_step += 1
            rclpy.spin_once(agent, timeout_sec=0.0)
            if done:
                solved = bool(agent._maze_solved_this_episode)
                break

        wp_cleared = int(agent.current_waypoint_idx + int(agent._maze_solved_this_episode))
        if not solved:
            solved = bool(agent._maze_solved_this_episode)

        results.append(
            {
                "episode": ep,
                "reward": round(ep_rew, 3),
                "steps": ep_step,
                "waypoints_cleared": wp_cleared,
                "solved": solved,
                "shield_active": bool(agent.use_shield),
            }
        )
        agent.get_logger().info(
            f"[Eval {ep:>3}/{n_episodes}]  reward={ep_rew:>8.1f}  "
            f"steps={ep_step:>5}  WP={wp_cleared}/{n_wp}  "
            f"{'✓ SOLVED' if solved else '✗'}"
        )

    return results


# ---------------------------------------------------------------------------
# Summary statistics + reporting
# ---------------------------------------------------------------------------


def _summarise(results: list[dict], n_wp: int) -> dict:
    rewards = [r["reward"] for r in results]
    steps = [r["steps"] for r in results]
    wps = [int(r["waypoints_cleared"]) for r in results]
    solved = [r["solved"] for r in results]
    ntot = max(len(results), 1)
    ge1 = sum(1 for w in wps if w >= 1)
    on_final_leg = sum(1 for w in wps if n_wp > 1 and w >= n_wp - 1)
    return {
        "episodes_total": len(results),
        "episodes_solved": int(sum(solved)),
        "success_rate": round(sum(solved) / max(len(solved), 1), 4),
        "episodes_reached_at_least_one_wp": ge1,
        "fraction_reached_at_least_one_wp": round(ge1 / ntot, 4),
        "episodes_on_final_waypoint_leg": on_final_leg,
        "fraction_on_final_waypoint_leg": round(on_final_leg / ntot, 4) if n_wp > 1 else 0.0,
        "best_waypoints_cleared_single_episode": int(max(wps)) if wps else 0,
        "mean_reward": round(float(np.mean(rewards)), 3),
        "std_reward": round(float(np.std(rewards)), 3),
        "min_reward": round(float(np.min(rewards)), 3),
        "max_reward": round(float(np.max(rewards)), 3),
        "mean_waypoints": round(float(np.mean(wps)), 3),
        "std_waypoints": round(float(np.std(wps)), 3),
        "mean_steps": round(float(np.mean(steps)), 1),
        "std_steps": round(float(np.std(steps)), 1),
        "total_waypoints_possible": n_wp,
    }


def _print_table(summary: dict, algo: str, model_path: str, n_wp: int) -> None:
    W = 62
    sep = "─" * W
    print(f"\n┌{sep}┐")
    print(f"│{'  EVALUATION RESULTS':^{W}}│")
    print(f"│{'  Algorithm : ' + algo.upper():^{W}}│")
    path_short = model_path if len(model_path) <= W - 14 else "…" + model_path[-(W - 15) :]
    print(f"│{'  Model     : ' + path_short:^{W}}│")
    print(f"├{sep}┤")
    print(f"│  {'Episodes evaluated':<28} {summary['episodes_total']:>6}{'':>26}│")
    pct = summary["success_rate"] * 100
    print(
        f"│  {'Maze solved (all WPs)':<28} {summary['episodes_solved']:>3} / "
        f"{summary['episodes_total']:<3}  ({pct:5.1f} %){'':>7}│"
    )
    p1 = summary["fraction_reached_at_least_one_wp"] * 100
    print(
        f"│  {'Reached ≥1 waypoint':<28} {summary['episodes_reached_at_least_one_wp']:>3} / "
        f"{summary['episodes_total']:<3}  ({p1:5.1f} %){'':>13}│"
    )
    if n_wp > 2:
        p2 = summary["fraction_on_final_waypoint_leg"] * 100
        print(
            f"│  {'On final WP leg (≥2 of 3 clr.)':<28} {summary['episodes_on_final_waypoint_leg']:>3} / "
            f"{summary['episodes_total']:<3}  ({p2:5.1f} %){'':>7}│"
        )
    print(
        f"│  {'Best WP count (single ep.)':<28} {summary['best_waypoints_cleared_single_episode']:>3} / {n_wp}{'':>26}│"
    )
    print(
        f"│  {'Mean reward':<28} {summary['mean_reward']:>+10.2f}  ±{summary['std_reward']:>8.2f}{'':>7}│"
    )
    print(
        f"│  {'Mean waypoints cleared':<28} {summary['mean_waypoints']:>8.2f}  ±{summary['std_waypoints']:>8.2f}  / {n_wp}{'':>3}│"
    )
    print(
        f"│  {'Mean steps / episode':<28} {summary['mean_steps']:>10.1f}  ±{summary['std_steps']:>8.1f}{'':>7}│"
    )
    print(f"└{sep}┘\n")


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "Evaluate a trained policy via SafeNavAgent (same MDP as training). "
            "Outputs JSON metrics for thesis benchmarking."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--algo", type=str, default="sac", help="Algorithm key: sac | td3 | custom")
    p.add_argument(
        "--model",
        type=str,
        default="",
        help="Path to actor .pth (default: trained_models/<algo>_actor_maze_best_ever.pth …)",
    )
    p.add_argument("--episodes", type=int, default=20, help="Number of evaluation episodes.")
    p.add_argument(
        "--max-steps",
        type=int,
        default=4000,
        help="Hard step cap per episode (default 4000 matches menu / slack for slow solves).",
    )
    p.add_argument(
        "--env-step-sleep-sec",
        type=float,
        default=0.05,
        help="Sleep after each env step — must match training pacing for fair sim dynamics.",
    )
    p.add_argument(
        "--no-reset",
        action="store_true",
        help="Skip /reset_simulation between episodes (invalid for strict benchmarking).",
    )
    p.add_argument(
        "--no-shield",
        action="store_true",
        help="Disable shield (only if the policy was trained without --use-shield).",
    )
    p.add_argument("--reset-service", type=str, default="/reset_simulation")
    p.add_argument(
        "--reset-service-wait-sec",
        type=float,
        default=45.0,
        help="Max seconds to wait for reset service (aligned with Phase 2 train_menu).",
    )
    p.add_argument(
        "--reset-reply-wait-sec",
        type=float,
        default=5.0,
        help="Max seconds to wait for reset RPC reply when not using fire-and-forget.",
    )
    p.add_argument(
        "--reset-wait-for-reply",
        action="store_true",
        help="Disable --reset-fire-and-forget behavior (wait for Empty response).",
    )
    p.add_argument("--device", type=str, default="cpu", choices=("cpu", "cuda"))
    p.add_argument(
        "--tag",
        type=str,
        default="",
        help="Output JSON filename tag (e.g. 'phase2_sac'). Default: <algo>_<unix_time>.",
    )
    p.add_argument(
        "--paper-eval",
        action="store_true",
        help="≥30 episodes; combine with --require-reset for thesis-grade invalid-run guard.",
    )
    p.add_argument(
        "--require-reset",
        action="store_true",
        help="Exit 2 on first episode where sim reset was not invoked (unless --no-reset).",
    )
    p.add_argument(
        "--training-contract",
        type=str,
        default="",
        help="Frozen MDP YAML as in training (default $TRAINING_CONTRACT or bundled).",
    )
    p.add_argument(
        "--waypoint-goal-radius",
        type=float,
        default=DEFAULT_EVAL_WAYPOINT_GOAL_RADIUS_M,
        metavar="M",
        help=(
            "Waypoint clear radius in metres (Phase 2 menu uses 0.68). "
            "Use 0 for contract GOAL_RADIUS only (stricter)."
        ),
    )
    p.add_argument(
        "--sac-deterministic-mean",
        action="store_true",
        help="SAC: tanh(mean) instead of sample() (off-policy mismatch vs training).",
    )
    p.add_argument(
        "--td3-policy-noise-std",
        type=float,
        default=0.0,
        metavar="SIGMA",
        help=(
            "TD3: Gaussian noise on actor output before affine map (training uses 0.1). "
            "Default 0 — greedy eval. Set 0.1 only for distribution-matched rollouts."
        ),
    )
    return p


def main() -> None:
    agent_dir = os.path.dirname(os.path.abspath(__file__))
    parser = _build_parser()
    args, _ = parser.parse_known_args()

    from training_contract import apply_contract_to_main_agent, default_contract_path, load_contract

    _tcp = args.training_contract or os.environ.get("TRAINING_CONTRACT", "").strip()
    _cp = os.path.abspath(os.path.expanduser(_tcp) if _tcp else default_contract_path())
    apply_contract_to_main_agent(ma, load_contract(_cp), contract_path=_cp)

    algo = args.algo.strip().lower()
    if args.model:
        model_path = args.model
    else:
        _mdir = os.path.join(agent_dir, "trained_models")
        _candidates = []
        _pw = os.environ.get("PFE_WORLD", "").strip()
        if _pw:
            _base = os.path.basename(os.path.abspath(os.path.expanduser(_pw)))
            if _base.startswith("eval_") and _base.endswith(".world"):
                _scope = _base[: -len(".world")]
                _candidates.append(
                    os.path.join(_mdir, f"{algo}_actor_maze_best_ever_{_scope}.pth")
                )
                _candidates.append(
                    os.path.join(_mdir, f"{algo}_actor_phase3_{_scope[5:]}.pth")
                )
        _candidates.extend([
            os.path.join(_mdir, f"{algo}_actor_maze_best_ever.pth"),
            os.path.join(_mdir, f"{algo}_actor_maze.pth"),
            os.path.join(_mdir, f"{algo}_actor_adapt_best_ever.pth"),
            os.path.join(_mdir, f"{algo}_actor_adapt.pth"),
        ])
        model_path = next(
            (c for c in _candidates if os.path.isfile(c)),
            _candidates[0],
        )
    model_path = os.path.abspath(os.path.expanduser(model_path))

    if not os.path.isfile(model_path):
        print(f"[ERROR] Model file not found: {model_path}", file=sys.stderr)
        print(
            "  Run a training phase first, or pass --model /path/to/actor.pth",
            file=sys.stderr,
        )
        sys.exit(1)

    use_shield = not args.no_shield
    require_reset = bool(args.require_reset or (args.paper_eval and not args.no_reset))
    n_episodes = max(30, args.episodes) if args.paper_eval else args.episodes
    wgr = float(args.waypoint_goal_radius)
    _clear_r = float(ma.GOAL_RADIUS) if wgr <= 1e-6 else wgr
    sac_use_sample = not bool(args.sac_deterministic_mean)
    td3_noise = float(args.td3_policy_noise_std)
    reset_ff = not bool(args.reset_wait_for_reply)

    _pfe = os.environ.get("PFE_WORLD", "").strip()
    if _pfe:
        print(
            f"[eval] PFE_WORLD={_pfe}\n"
            "      (Gazebo must load this world — match the [GAZEBO] line in the sim terminal.)"
        )
    else:
        print(
            "[eval] PFE_WORLD is not set.\n"
            "      Prefer train_menu.sh → 3 so the sim restarts on the intended .world."
        )
    print(
        "[eval] Using SafeNavAgent.step_environment (same MDP/shield/LiDAR as main_agent training).\n"
        "      Do not run main_agent in parallel (same ROS node/cmd_vel topic)."
    )
    if args.paper_eval:
        print(
            f"[paper-eval] episodes={n_episodes} (floor 30), "
            f"strict_reset={'ON' if require_reset else 'OFF'}"
        )

    rclpy.init()
    agent = ma.SafeNavAgent(
        algo,
        force_restart=True,
        device_str=args.device,
        randomize_goal=False,
        goal_mode="box",
        waypoint_mode=True,
        use_shield=use_shield,
        waypoint_goal_radius_m=wgr,
        no_sim_reset=args.no_reset,
        reset_service=args.reset_service,
        reset_service_wait_sec=args.reset_service_wait_sec,
        reset_reply_wait_sec=args.reset_reply_wait_sec,
        reset_fire_and_forget=reset_ff,
        env_step_sleep_sec=args.env_step_sleep_sec,
        lr=3e-4,
        enable_tensorboard=False,
    )

    ckpt = torch.load(model_path, map_location=agent.device)
    ckpt_in = actor_checkpoint_in_features(ckpt)
    if ckpt_in is not None and ckpt_in != agent.state_dim:
        print(
            f"[ERROR] Model state_dim={ckpt_in} does not match SafeNavAgent "
            f"state_dim={agent.state_dim} (LiDAR bins + 2 from contract).",
            file=sys.stderr,
        )
        rclpy.shutdown()
        sys.exit(1)
    try:
        agent.actor.load_state_dict(ckpt)
    except Exception as exc:
        print(f"[ERROR] Failed to load checkpoint into actor: {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    agent.actor.eval()

    agent.get_logger().info(
        f"Evaluating {algo.upper()} | model={model_path} | "
        f"episodes={n_episodes} | max_steps={args.max_steps} | "
        f"env_step_sleep_sec={args.env_step_sleep_sec:g} | device={agent.device} | "
        f"shield={'ON' if use_shield else 'OFF'} | "
        f"waypoint_clear_radius_m={_clear_r:g} | "
        f"sac_action={'sample' if sac_use_sample else 'tanh(mean)'} | "
        f"td3_policy_noise_std={td3_noise:g} | "
        f"reset_fire_and_forget={reset_ff} | "
        f"paper_eval={bool(args.paper_eval)} | require_reset={require_reset}"
    )

    try:
        results = run_eval_episodes(
            agent,
            algo,
            n_episodes,
            args.max_steps,
            sac_use_sample=sac_use_sample,
            td3_policy_noise_std=td3_noise,
            require_reset=require_reset,
        )
    except KeyboardInterrupt:
        agent.cmd_vel_pub.publish(Twist())
        results = []
    finally:
        rclpy.shutdown()

    if not results:
        print("No episodes completed.")
        return

    n_wp = len(ma.MAZE_WAYPOINTS)
    summary = _summarise(results, n_wp)
    _print_table(summary, algo, model_path, n_wp)

    log_dir = os.path.join(agent_dir, "pfe_logs")
    os.makedirs(log_dir, exist_ok=True)
    tag = args.tag.strip() or f"{algo}_{int(time.time())}"
    out_path = os.path.join(log_dir, f"eval_{tag}.json")
    payload = {
        "algo": algo,
        "model_path": model_path,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "waypoints": list(ma.MAZE_WAYPOINTS),
        "eval_config": {
            "harness": "SafeNavAgent_step_environment",
            "max_steps_per_episode": args.max_steps,
            "env_step_sleep_sec": args.env_step_sleep_sec,
            "shield": use_shield,
            "waypoint_goal_radius_cli": wgr,
            "waypoint_clear_radius_m": _clear_r,
            "sac_action": "sample" if sac_use_sample else "tanh_mean",
            "td3_policy_noise_std": td3_noise,
            "reset_fire_and_forget": reset_ff,
            "reset_service_wait_sec": args.reset_service_wait_sec,
            "pfe_world": os.environ.get("PFE_WORLD", ""),
            "paper_eval": bool(args.paper_eval),
            "require_reset": require_reset,
            "episodes_run": n_episodes,
            "training_contract_path": _cp,
        },
        "episodes": results,
        "summary": summary,
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"Results saved → {out_path}")


if __name__ == "__main__":
    main()
