#!/usr/bin/env python3
"""
Persistent evaluation node that **hot-swaps actor weights at runtime**.

Uses SafeNavAgent (same MDP as evaluate_agent.py). Subscribes to /policy_reload
for website upload_server.py uploads.

  ros2 topic pub --once /policy_reload std_msgs/msg/String \
    "{data: '/path/to/uploaded_brain.pth'}"
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

try:
    import main_agent as ma  # noqa: PLC0411 — same dir when run via start_hot_swap_for_web.sh
except ImportError:
    import safe_drl_nav.main_agent as ma

actor_checkpoint_in_features = ma.actor_checkpoint_in_features


def _actor_state_dict_from_checkpoint(ckpt: object) -> dict:
    if isinstance(ckpt, dict) and "actor" in ckpt:
        inner = ckpt["actor"]
        if isinstance(inner, dict):
            return inner
    if isinstance(ckpt, dict):
        return ckpt
    raise TypeError(f"expected dict checkpoint, got {type(ckpt)!r}")


def _infer_algo_from_state_dict(state_dict: dict) -> str | None:
    keys = set(state_dict.keys())
    if any(k.startswith("mean_linear.") or k.startswith("fc1.") for k in keys):
        return "sac"
    if any(k.startswith("net.") for k in keys):
        return "td3"
    return None


def _parse_reload_message(raw: str) -> tuple[str, str | None]:
    """Return (checkpoint path, optional algo hint)."""
    text = raw.strip()
    if not text:
        return "", None
    if text.startswith("{"):
        try:
            obj = json.loads(text)
            if isinstance(obj, dict):
                path = str(obj.get("path") or obj.get("model") or "").strip()
                algo = obj.get("algo")
                if algo is not None:
                    return path, str(algo).strip().lower()
                return path, None
        except json.JSONDecodeError:
            pass
    return text, None


DEFAULT_WAYPOINT_GOAL_RADIUS_M = 0.68
TELEMETRY_FILE = Path(
    os.environ.get(
        "TELEMETRY_FILE",
        os.path.expanduser("~/ros2_ws/pfe_logs/telemetry_live.json"),
    )
)


def _viz24_from_bins(bins36: list[float] | np.ndarray) -> list[float]:
    n_in = len(bins36)
    if n_in == 0:
        return [3.5] * 24
    out: list[float] = []
    for i in range(24):
        lo = int(i * n_in / 24)
        hi = int((i + 1) * n_in / 24)
        seg = bins36[lo:hi]
        out.append(round(float(min(seg)), 3))
    return out


def _wp_dict(w, i: int) -> dict:
    """Normalize waypoint as (x,y) tuple/list or {x,y,n} dict from maze_web_layout."""
    if isinstance(w, dict):
        return {
            "x": float(w.get("x", 0.0)),
            "y": float(w.get("y", 0.0)),
            "n": int(w.get("n", i + 1)),
        }
    return {"x": float(w[0]), "y": float(w[1]), "n": i + 1}


def _maze_web_meta(wp_idx: int) -> dict:
    _default_wps = [
        {"x": -2.8, "y": -1.8, "n": 1},
        {"x": -4.5, "y": -0.2, "n": 2},
        {"x": -3.2, "y": 1.2, "n": 3},
    ]
    try:
        scripts = Path(__file__).resolve().parents[3] / "scripts"
        if str(scripts) not in sys.path:
            sys.path.insert(0, str(scripts))
        from maze_web_layout import MAZE_ENV_NAME, MAZE_OBSTACLES, MAZE_WAYPOINTS

        wps = [_wp_dict(w, i) for i, w in enumerate(MAZE_WAYPOINTS)]
        return {
            "env_name": MAZE_ENV_NAME,
            "waypoints": wps,
            "obstacles": MAZE_OBSTACLES,
            "wp_idx": int(wp_idx),
        }
    except Exception:
        return {
            "env_name": "Waypoint demo",
            "waypoints": _default_wps,
            "obstacles": [],
            "wp_idx": int(wp_idx),
        }


def _write_web_telemetry(
    *,
    x: float,
    y: float,
    yaw: float,
    scan24: list[float],
    ep: int,
    step: int,
    rew: float,
    tot: float,
    ok_ep: bool,
    wp_idx: int,
) -> None:
    try:
        # Same schema as upload_telemetry_sidecar + website lidar_live (t=s, scan, ep, step).
        payload = {
            "ok": True,
            "updated_at": time.time(),
            "t": "s",
            "x": round(float(x), 3),
            "y": round(float(y), 3),
            "yaw": round(float(yaw), 3),
            "scan": scan24,
            "ep": int(ep),
            "step": int(step),
            "rew": round(float(rew), 3),
            "tot": round(float(tot), 3),
            "hit": False,
            "episode_ok": bool(ok_ep),
            **_maze_web_meta(wp_idx),
        }
        TELEMETRY_FILE.parent.mkdir(parents=True, exist_ok=True)
        TELEMETRY_FILE.write_text(json.dumps(payload), encoding="utf-8")
    except Exception:
        pass  # Web telemetry must not interrupt the live eval control loop


def _raw_to_applied(raw_action: np.ndarray) -> np.ndarray:
    return np.array(
        [
            float(np.clip((float(raw_action[0]) + 1.0) * 0.2, 0.0, 0.4)),
            float(np.clip(float(raw_action[1]), -1.0, 1.0)),
        ],
        dtype=np.float32,
    )


def _select_action_raw(
    algo: str,
    actor: torch.nn.Module,
    state: np.ndarray,
    device: torch.device,
    *,
    sac_use_sample: bool = False,
) -> np.ndarray:
    st = torch.FloatTensor(state).unsqueeze(0).to(device)
    with torch.no_grad():
        if algo == "sac":
            if sac_use_sample:
                raw_action, _, _ = actor.sample(st)
                return raw_action.detach().cpu().numpy()[0]
            mean, _ = actor(st)
            return torch.tanh(mean).cpu().numpy()[0]
        if algo == "custom":
            return actor.predict(st).detach().cpu().numpy()[0]
        return actor(st).cpu().numpy()[0]


class HotSwapEvalNode(ma.SafeNavAgent):
    """SafeNavAgent + /policy_reload and timer-driven stepping."""

    STATE_DIM = 38

    def __init__(
        self,
        algo: str,
        *,
        env_step_sleep_sec: float,
        reload_resets_episode: bool,
        control_period_sec: float,
        max_steps: int,
        device: torch.device,
        waypoint_goal_radius_m: float = DEFAULT_WAYPOINT_GOAL_RADIUS_M,
        no_reset: bool = False,
        reset_service: str = "/reset_simulation",
        reset_service_wait_sec: float = 45.0,
        reset_fire_and_forget: bool = True,
        sac_use_sample: bool = False,
    ) -> None:
        super().__init__(
            algo,
            force_restart=True,
            device_str=str(device),
            randomize_goal=False,
            goal_mode="box",
            waypoint_mode=True,
            use_shield=True,
            waypoint_goal_radius_m=waypoint_goal_radius_m,
            no_sim_reset=no_reset,
            reset_service=reset_service,
            reset_service_wait_sec=reset_service_wait_sec,
            reset_fire_and_forget=reset_fire_and_forget,
            env_step_sleep_sec=env_step_sleep_sec,
            enable_tensorboard=False,
        )
        self.algo = algo.strip().lower()
        self._sac_use_sample = bool(sac_use_sample)

        cbg = ReentrantCallbackGroup()
        try:
            self.destroy_subscription(self.scan_sub)
            self.destroy_subscription(self.odom_sub)
        except Exception:
            pass

        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            qos_profile_sensor_data,
            callback_group=cbg,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=cbg,
        )

        self._max_steps = int(max_steps)
        self._reload_reset = bool(reload_resets_episode)
        self._hs_running = False
        self._hs_state: np.ndarray | None = None
        self._hs_step_in_episode = 0
        self._hs_episode = 0
        self._hs_ep_tot_rew = 0.0
        self._hs_reset_busy = False

        self.create_subscription(String, "/policy_reload", self._cb_reload, 10, callback_group=cbg)
        self.create_subscription(String, "/policy_control", self._cb_control, 10, callback_group=cbg)
        self.create_timer(float(control_period_sec), self._cb_timer, callback_group=cbg)

        sac_mode = "sample" if (self.algo == "sac" and self._sac_use_sample) else (
            "tanh(mean)" if self.algo == "sac" else self.algo
        )
        self.get_logger().info(
            f"HotSwapEvalNode ready — sac_action={sac_mode} — "
            "/policy_reload, /policy_control (pause|resume|reset_episode)"
        )

    def reset_env(self):
        """Web eval: avoid spin_once() inside the control timer (deadlocks MultiThreadedExecutor)."""
        if self._no_sim_reset:
            self.cmd_vel_pub.publish(Twist())
            self._maze_solved_this_episode = False
            self.current_waypoint_idx = 0
            self.target_x, self.target_y = self._maze_waypoints[0]
            self.get_logger().info(
                f"Waypoint mode reset → WP 1/{len(self._maze_waypoints)}: "
                f"({self.target_x:.2f}, {self.target_y:.2f})"
            )
            self.prev_dist = math.hypot(
                self.target_x - self.robot_x, self.target_y - self.robot_y
            )
            self.prev_cmd[:] = 0.0
            self._last_sim_reset_applied = True
            self.scan_received = True
            self.odom_received = True
            return self.get_state()
        return super().reset_env()

    def laser_callback(self, msg) -> None:
        super().laser_callback(msg)

    def odom_callback(self, msg) -> None:
        super().odom_callback(msg)

    def _rebuild_actor(self, algo: str) -> None:
        algo = algo.strip().lower()
        if algo == "td3":
            from networks_td3 import ActorNetwork
        elif algo == "custom":
            from networks_custom import ActorNetwork  # type: ignore[import]
        else:
            from networks_sac import ActorNetwork
            algo = "sac"
        self.algo = algo
        self.algo_choice = algo
        self.actor = ActorNetwork(self.state_dim, self.action_dim).to(self.device)
        self.actor.eval()

    def _apply_checkpoint(self, raw_path: str, *, algo_hint: str | None = None) -> bool:
        path = os.path.abspath(os.path.expanduser(raw_path.strip()))
        if not path or not os.path.isfile(path):
            self.get_logger().error(f"Reload failed — not a file: {path!r}")
            return False
        try:
            ckpt = torch.load(path, map_location=self.device)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"torch.load failed: {exc}")
            return False
        try:
            state_dict = _actor_state_dict_from_checkpoint(ckpt)
        except TypeError as exc:
            self.get_logger().error(str(exc))
            return False
        ckpt_in = actor_checkpoint_in_features(state_dict)
        if ckpt_in is not None and ckpt_in != self.state_dim:
            self.get_logger().error(
                f"Checkpoint state_dim={ckpt_in} expected {self.state_dim}."
            )
            return False
        target_algo = (algo_hint or "").strip().lower() or None
        inferred = _infer_algo_from_state_dict(state_dict)
        if target_algo not in ("sac", "td3", "custom"):
            target_algo = inferred
        if target_algo and target_algo != self.algo:
            self.get_logger().info(
                f"Switching actor architecture {self.algo} → {target_algo} for reload."
            )
            self._rebuild_actor(target_algo)
        elif inferred and inferred != self.algo:
            self.get_logger().info(
                f"Inferred algo {inferred} (was {self.algo}) — rebuilding actor."
            )
            self._rebuild_actor(inferred)
        try:
            self.actor.load_state_dict(state_dict)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"load_state_dict failed: {exc}")
            return False
        self.actor.eval()
        self.get_logger().info(f"Weights loaded ({self.algo}): {path}")
        self._hs_ep_tot_rew = 0.0
        self._hs_state = None
        return True

    def _publish_web_from_ros(self) -> None:
        scan = _viz24_from_bins(self.current_scan)
        _write_web_telemetry(
            x=self.robot_x,
            y=self.robot_y,
            yaw=self.robot_yaw,
            scan24=scan,
            ep=self._hs_episode,
            step=self._hs_step_in_episode,
            rew=getattr(self, "_hs_last_rew", 0.0),
            tot=self._hs_ep_tot_rew,
            ok_ep=False,
            wp_idx=int(self.current_waypoint_idx),
        )

    def _cb_reload(self, msg: String) -> None:
        self.cmd_vel_pub.publish(Twist())
        path, algo_hint = _parse_reload_message(msg.data)
        if not self._apply_checkpoint(path, algo_hint=algo_hint):
            return
        self._hs_running = True
        if self._reload_reset:
            self._hs_state = None

    def _cb_control(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd in ("pause", "stop"):
            self._hs_running = False
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("Policy paused.")
        elif cmd in ("resume", "start", "run"):
            self._hs_running = True
            self.get_logger().info("Policy resumed.")
        elif cmd in ("reset_episode", "reset"):
            self._hs_state = None
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("Next tick starts a fresh episode reset.")
        else:
            self.get_logger().warn(f"Unknown /policy_control: {msg.data!r}")

    def _cb_timer(self) -> None:
        if not self._hs_running:
            return
        self.scan_received = True
        self.odom_received = True

        if self._hs_state is None:
            if self._hs_reset_busy:
                return
            self._hs_reset_busy = True
            try:
                self._hs_state = self.reset_env()
            except RuntimeError as exc:
                self.get_logger().error(f"reset_env failed: {exc}")
                self._hs_state = None
            finally:
                self._hs_reset_busy = False
            self._hs_step_in_episode = 0
            return

        if self._hs_step_in_episode >= self._max_steps:
            self.get_logger().warning("Episode step cap — resetting.")
            self._hs_state = self.reset_env()
            self._hs_step_in_episode = 0
            return

        raw = _select_action_raw(
            self.algo,
            self.actor,
            self._hs_state,
            self.device,
            sac_use_sample=self._sac_use_sample,
        )
        applied = _raw_to_applied(raw)
        self._hs_state, _rew, done = self.step_environment(applied)
        self._hs_step_in_episode += 1
        self._hs_ep_tot_rew += float(_rew)
        self._hs_last_rew = float(_rew)
        self._publish_web_from_ros()
        if self._hs_step_in_episode % 40 == 0:
            self.get_logger().info(
                f"step {self._hs_step_in_episode} ep {self._hs_episode} "
                f"rew={float(_rew):.2f} pos=({self.robot_x:.2f},{self.robot_y:.2f}) "
                f"wp={int(self.current_waypoint_idx)+1}/3"
            )

        if done:
            solved = bool(self._maze_solved_this_episode)
            self.get_logger().info(
                f"Episode end solved={solved} steps={self._hs_step_in_episode}"
            )
            _write_web_telemetry(
                x=self.robot_x,
                y=self.robot_y,
                yaw=self.robot_yaw,
                scan24=_viz24_from_bins(self.current_scan),
                ep=self._hs_episode,
                step=self._hs_step_in_episode,
                rew=float(_rew),
                tot=self._hs_ep_tot_rew,
                ok_ep=solved,
                wp_idx=int(self.current_waypoint_idx),
            )
            self._hs_episode += 1
            self._hs_ep_tot_rew = 0.0
            try:
                self._hs_state = self.reset_env()
            except RuntimeError as exc:
                self.get_logger().error(f"reset_env after episode: {exc}")
                self._hs_state = None
            self._hs_step_in_episode = 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Hot-swap policy evaluation node")
    parser.add_argument("--algo", type=str, default="sac", help="sac | td3 | custom")
    parser.add_argument("--model", type=str, default="", help="Optional initial .pth path")
    parser.add_argument("--device", type=str, default="cpu", choices=("cpu", "cuda"))
    parser.add_argument("--max-steps", type=int, default=4000)
    parser.add_argument("--env-step-sleep-sec", type=float, default=0.05)
    parser.add_argument("--control-period", type=float, default=0.05)
    parser.add_argument("--no-reset", action="store_true")
    parser.add_argument("--reset-service", type=str, default="/reset_simulation")
    parser.add_argument("--reset-service-wait-sec", type=float, default=45.0)
    parser.add_argument("--no-restart-on-reload", action="store_true")
    parser.add_argument("--training-contract", type=str, default="")
    parser.add_argument(
        "--waypoint-goal-radius",
        type=float,
        default=DEFAULT_WAYPOINT_GOAL_RADIUS_M,
    )
    parser.add_argument(
        "--sac-sample",
        action="store_true",
        help="SAC actor.sample() — matches evaluate_agent.py stochastic policy mode.",
    )
    args, _unknown = parser.parse_known_args()

    from training_contract import (
        apply_contract_to_main_agent,
        default_contract_path,
        load_contract,
    )

    _tcp = args.training_contract or os.environ.get("TRAINING_CONTRACT", "").strip()
    _cp = os.path.abspath(os.path.expanduser(_tcp) if _tcp else default_contract_path())
    apply_contract_to_main_agent(ma, load_contract(_cp), contract_path=_cp)

    device = torch.device(
        "cuda" if args.device == "cuda" and torch.cuda.is_available() else "cpu"
    )

    rclpy.init()
    node = HotSwapEvalNode(
        args.algo,
        env_step_sleep_sec=args.env_step_sleep_sec,
        reload_resets_episode=not args.no_restart_on_reload,
        control_period_sec=args.control_period,
        max_steps=args.max_steps,
        device=device,
        waypoint_goal_radius_m=float(args.waypoint_goal_radius),
        no_reset=args.no_reset,
        reset_service=args.reset_service,
        reset_service_wait_sec=args.reset_service_wait_sec,
        reset_fire_and_forget=True,
        sac_use_sample=bool(args.sac_sample),
    )

    model_path = args.model.strip()
    if model_path:
        model_path = os.path.abspath(os.path.expanduser(model_path))
        if not node._apply_checkpoint(model_path):
            rclpy.shutdown()
            sys.exit(1)
        node._hs_running = True
    else:
        print(
            "[hot_swap_eval] No --model — waiting for /policy_reload from upload.",
            file=sys.stderr,
        )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.cmd_vel_pub.publish(Twist())
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
