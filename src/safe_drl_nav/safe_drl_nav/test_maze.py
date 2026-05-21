import argparse
import math
import os
import sys
import time
from typing import Callable

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from main_agent import (
    CRAWL_ANG_THRESH,
    CRAWL_LIN_THRESH,
    LIDAR_BINS,
    SHIELD_BACKUP_LIN,
    SHIELD_CRITICAL_RANGE,
    SHIELD_TURN_ANG,
    TIGHT_CORNER_CLEARANCE,
    actor_checkpoint_in_features,
    pool_laser_to_bins,
)
from networks_sac import ActorNetwork as SACActor
from networks_td3 import ActorNetwork as TD3Actor

# Extend actor_ctor(algo_key) when adding new policies (e.g. PPO, DDPG).
ActorCtor = Callable[[int, int], torch.nn.Module]
_ALGO_ACTOR_REGISTRY: dict[str, ActorCtor] = {
    "sac": lambda sd, ad: SACActor(sd, ad),
    "td3": lambda sd, ad: TD3Actor(sd, ad),
}


def _resolve_algo_key(raw: str) -> str:
    return raw.strip().lower()


def _register_algo(name: str, ctor: ActorCtor) -> None:
    """Optional hook for notebooks or future plugins: register additional algorithms."""
    _ALGO_ACTOR_REGISTRY[_resolve_algo_key(name)] = ctor


def _make_actor(algo: str, state_dim: int, action_dim: int) -> torch.nn.Module:
    key = _resolve_algo_key(algo)
    if key not in _ALGO_ACTOR_REGISTRY:
        known = ", ".join(sorted(_ALGO_ACTOR_REGISTRY))
        raise SystemExit(
            f"Unknown --algo {algo!r} (normalized {key!r}). "
            f"Registered actors: {known}. "
            f"Add a network under networks_*.py and register it in _ALGO_ACTOR_REGISTRY in test_maze.py."
        )
    return _ALGO_ACTOR_REGISTRY[key](state_dim, action_dim)


def _min_front_side_lidar(
    current_scan: np.ndarray,
    pool_edges: np.ndarray,
    n_rays: int,
    angle_min: float,
    angle_inc: float,
    lidar_bins: int,
) -> float:
    """Minimum pooled range in forward ±90° (robot frame); matches main_agent SafeNavAgent."""
    if pool_edges is None or n_rays < 1:
        return float(np.min(current_scan))
    vals: list[float] = []
    for j in range(lidar_bins):
        r0 = int(pool_edges[j])
        r1 = int(pool_edges[j + 1])
        mid = (r0 + r1 - 1) // 2
        mid = int(np.clip(mid, 0, n_rays - 1))
        ang = angle_min + float(mid) * angle_inc
        ang = (ang + math.pi) % (2.0 * math.pi) - math.pi
        if abs(ang) <= math.pi / 2.0 + 1e-3:
            vals.append(float(current_scan[j]))
    return float(min(vals)) if vals else float(np.min(current_scan))


def _closest_front_side_obstacle_bearing(
    current_scan: np.ndarray,
    pool_edges: np.ndarray,
    n_rays: int,
    angle_min: float,
    angle_inc: float,
    lidar_bins: int,
) -> float:
    if pool_edges is None or n_rays < 1:
        return 0.0
    best_j = 0
    best_r = float("inf")
    for j in range(lidar_bins):
        r0 = int(pool_edges[j])
        r1 = int(pool_edges[j + 1])
        mid = (r0 + r1 - 1) // 2
        mid = int(np.clip(mid, 0, n_rays - 1))
        ang = angle_min + float(mid) * angle_inc
        ang = (ang + math.pi) % (2.0 * math.pi) - math.pi
        if abs(ang) <= math.pi / 2.0 + 1e-3:
            r = float(current_scan[j])
            if r < best_r:
                best_r = r
                best_j = j
    r0 = int(pool_edges[best_j])
    r1 = int(pool_edges[best_j + 1])
    mid = (r0 + r1 - 1) // 2
    mid = int(np.clip(mid, 0, n_rays - 1))
    b = angle_min + float(mid) * angle_inc
    return (b + math.pi) % (2.0 * math.pi) - math.pi


def _shield_cmd_override_from_scan(
    current_scan: np.ndarray,
    pool_edges: np.ndarray,
    n_rays: int,
    angle_min: float,
    angle_inc: float,
    lidar_bins: int,
) -> tuple[float, float]:
    theta = _closest_front_side_obstacle_bearing(
        current_scan, pool_edges, n_rays, angle_min, angle_inc, lidar_bins
    )
    lin = float(np.clip(-SHIELD_BACKUP_LIN, -0.4, 0.0))
    if abs(theta) < 0.08:
        ang = float(SHIELD_TURN_ANG)
    else:
        ang = float(-np.sign(theta) * SHIELD_TURN_ANG)
    ang = float(np.clip(ang, -1.0, 1.0))
    return lin, ang


class MazeTester(Node):
    def __init__(
        self,
        algo: str,
        model_path: str,
        target_x: float,
        target_y: float,
        *,
        env_name: str,
        use_shield: bool = False,
        lidar_hard_stop_m: float | None = None,
        freeze_tight_crawl: bool = False,
        sac_use_mean: bool = True,
    ):
        super().__init__('maze_tester')
        self.device = torch.device("cpu")
        self.state_dim = LIDAR_BINS + 2
        self.algo_key = _resolve_algo_key(algo)
        self.env_name = (env_name or "").strip() or "unknown"

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self._lidar_range_max = 30.0
        self._lidar_range_min = 0.12
        self.current_scan = np.ones(LIDAR_BINS, dtype=np.float32) * self._lidar_range_max
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Prevent driving before sensors connect
        self.scan_received = False
        self.odom_received = False
        
        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.use_shield = bool(use_shield)

        self._scan_angle_min = -math.pi
        self._scan_angle_inc = 0.01
        self._n_rays = 360
        self._pool_edges: np.ndarray | None = None

        self.actor = _make_actor(algo, self.state_dim, 2).to(self.device)
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        ckpt = torch.load(model_path, map_location=self.device)
        ckpt_in = actor_checkpoint_in_features(ckpt)
        if ckpt_in is not None and ckpt_in != self.state_dim:
            raise RuntimeError(
                f"Model {model_path} expects state_dim={ckpt_in} (LiDAR bins + 2 = {(ckpt_in - 2)} + 2), "
                f"but this code uses LIDAR_BINS={LIDAR_BINS} → state_dim={self.state_dim}. "
                f"Either retrain with the current code, or temporarily set LIDAR_BINS to {(ckpt_in - 2)} "
                f"everywhere to match that checkpoint (not recommended long-term)."
            )
        self.actor.load_state_dict(ckpt)
        self.actor.eval()
        self._lidar_hard_stop_m = lidar_hard_stop_m
        self._freeze_tight_crawl = freeze_tight_crawl
        self._sac_use_mean = sac_use_mean

        self.get_logger().info(
            f"MazeTester env={self.env_name!r} algo={self.algo_key!r} "
            f"use_shield={self.use_shield} model={model_path}"
        )

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.odom_received = True

    def laser_callback(self, msg):
        self._lidar_range_max = float(msg.range_max)
        self._lidar_range_min = float(msg.range_min)
        ranges = msg.ranges
        self._n_rays = max(1, len(ranges))
        self._scan_angle_min = float(msg.angle_min)
        self._scan_angle_inc = float(msg.angle_increment)
        self._pool_edges = np.linspace(0, self._n_rays, LIDAR_BINS + 1, dtype=int)
        self.current_scan = pool_laser_to_bins(
            ranges, self._lidar_range_max, self._lidar_range_min, LIDAR_BINS
        )
        self.scan_received = True

    def drive(self):
        # Wait for Gazebo to actually send data before thinking
        if not self.scan_received or not self.odom_received:
            return

        min_b = float(np.min(self.current_scan))
        if self._lidar_hard_stop_m is not None and min_b < self._lidar_hard_stop_m:
            self.cmd_vel_pub.publish(Twist())
            return

        delta_x = self.target_x - self.robot_x
        delta_y = self.target_y - self.robot_y
        
        denom = max(self._lidar_range_max, 1e-6)
        scan_n = np.clip(self.current_scan.astype(np.float64), 0.0, denom) / denom
        state = np.append(scan_n.astype(np.float32), [np.float32(delta_x), np.float32(delta_y)])
        st = torch.FloatTensor(state).unsqueeze(0).to(self.device)

        with torch.no_grad():
            if self.algo_key == "td3":
                action = self.actor(st).cpu().numpy()[0]
            elif self.algo_key == "sac":
                if self._sac_use_mean:
                    mean, _ = self.actor(st)
                    action = torch.tanh(mean).cpu().numpy()[0]
                else:
                    action, _, _ = self.actor.sample(st)
                    action = action.cpu().numpy()[0]
            else:
                # DDPG / PPO-critic-style or any actor that returns actions directly.
                action = self.actor(st).cpu().numpy()[0]

        msg = Twist()
        cmd_lin = float(np.clip(action[0], 0.0, 0.4))
        cmd_ang = float(np.clip(action[1], -1.0, 1.0))

        if self.use_shield and self._pool_edges is not None:
            min_fs = _min_front_side_lidar(
                self.current_scan,
                self._pool_edges,
                self._n_rays,
                self._scan_angle_min,
                self._scan_angle_inc,
                LIDAR_BINS,
            )
            if min_fs < SHIELD_CRITICAL_RANGE:
                cmd_lin, cmd_ang = _shield_cmd_override_from_scan(
                    self.current_scan,
                    self._pool_edges,
                    self._n_rays,
                    self._scan_angle_min,
                    self._scan_angle_inc,
                    LIDAR_BINS,
                )

        msg.linear.x = cmd_lin
        msg.angular.z = cmd_ang
        if self._freeze_tight_crawl:
            if (
                min_b < TIGHT_CORNER_CLEARANCE
                and msg.linear.x <= CRAWL_LIN_THRESH
                and abs(msg.angular.z) <= CRAWL_ANG_THRESH
            ):
                msg.linear.x = 0.0
                msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run a trained policy against a live ROS2 / Gazebo stack. "
            "Uses parse_known_args() so extra ROS args pass through."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog=(
            "Adding a new algorithm: implement ActorNetwork in networks_<algo>.py, import it above, "
            "and add an entry to _ALGO_ACTOR_REGISTRY."
        ),
    )
    parser.add_argument(
        "--algo",
        type=str,
        default="sac",
        metavar="NAME",
        help="Policy / checkpoint family key (drives actor ctor and default model filename).",
    )
    parser.add_argument(
        "--shield",
        action="store_true",
        help="Enable LiDAR safety shield (override cmd_vel near obstacles); same idea as training --use-shield.",
    )
    parser.add_argument(
        "--env",
        type=str,
        default="current_random_lab",
        metavar="WORLD",
        help="Logical Gazebo world / maze name for logging (your sim must already load this world).",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=4000,
        metavar="N",
        help="Max control-loop iterations (spin + drive + sleep), then exit cleanly. 0 = run until Ctrl+C.",
    )
    parser.add_argument(
        "--model",
        type=str,
        default="",
        help="Path to actor .pth (default: trained_models/<algo>_actor_maze.pth next to this file).",
    )
    parser.add_argument(
        "--target-x",
        type=float,
        default=-3.5,
        help="Goal X in odom/world frame (match training or test zero-shot to a new goal).",
    )
    parser.add_argument(
        "--target-y",
        type=float,
        default=-3.5,
        help="Goal Y in odom/world frame.",
    )
    parser.add_argument(
        "--lidar-hard-stop",
        type=float,
        default=None,
        metavar="M",
        help="If set, publish zero cmd_vel when min LiDAR bin < M meters. "
        "Default: off (matches training: step_environment does not hard-stop on LiDAR).",
    )
    parser.add_argument(
        "--freeze-tight-crawl",
        action="store_true",
        help="Re-enable legacy test-only rule that zeroed cmd_vel in tight corridors for small "
        "twists. Training never applied this to commands (only reward shaping); it often freezes "
        "the robot in mazes.",
    )
    parser.add_argument(
        "--sac-stochastic",
        action="store_true",
        help="Use SAC policy sample() instead of tanh(mean). Closer to training exploration but noisy.",
    )
    return parser


def main() -> None:
    agent_dir = os.path.dirname(os.path.abspath(__file__))
    parser = _build_arg_parser()
    args, _unknown = parser.parse_known_args()
    if args.episodes < 0:
        print("error: --episodes must be >= 0", file=sys.stderr)
        sys.exit(2)
    model_path = args.model or os.path.join(
        agent_dir, "trained_models", f"{_resolve_algo_key(args.algo)}_actor_maze.pth"
    )

    rclpy.init()
    tester = MazeTester(
        args.algo,
        model_path,
        args.target_x,
        args.target_y,
        env_name=args.env,
        use_shield=args.shield,
        lidar_hard_stop_m=args.lidar_hard_stop,
        freeze_tight_crawl=args.freeze_tight_crawl,
        sac_use_mean=not args.sac_stochastic,
    )
    tester.get_logger().info("Starting neural policy test loop…")

    steps = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(tester, timeout_sec=0.05)
            tester.drive()
            time.sleep(0.05)
            if args.episodes > 0:
                steps += 1
                if steps >= args.episodes:
                    tester.get_logger().info(
                        f"Reached --episodes {args.episodes}; stopping (env={tester.env_name!r})."
                    )
                    break
    except KeyboardInterrupt:
        tester.cmd_vel_pub.publish(Twist())
    finally:
        tester.cmd_vel_pub.publish(Twist())
        rclpy.shutdown()

if __name__ == '__main__':
    main()