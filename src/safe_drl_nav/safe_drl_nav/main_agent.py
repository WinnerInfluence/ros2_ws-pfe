from __future__ import annotations

import argparse
import csv
import json
import math
import os
import socket
import subprocess
import sys
import time

import numpy as np
import rclpy
import torch
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from torch.utils.tensorboard import SummaryWriter

# LiDAR: 360/36 = 10 deg/bin — enough angular detail for maze corners vs 24 bins (~15 deg).
LIDAR_BINS = 36
# Dense shaping: positive when moving toward the goal along straight-line distance.
DISTANCE_MULTIPLIER = 18.0
STEP_PENALTY = 0.05
GOAL_RADIUS = 0.5
GOAL_REWARD = 100.0
# Shield / collision: front-side hemisphere (±90° from forward) clearance threshold (meters).
SHIELD_CRITICAL_RANGE = 0.35
COLLISION_TERMINAL_PENALTY = 15.0
SHIELD_BACKUP_LIN = 0.12
SHIELD_TURN_ANG = 0.9
# Used by test_maze.py (policy test harness) for tight-corridor heuristics.
TIGHT_CORNER_CLEARANCE = 0.48
CRAWL_LIN_THRESH = 0.065
CRAWL_ANG_THRESH = 0.14

# ---------------------------------------------------------------------------
# Sequential waypoint navigation (--waypoint-mode)
# Coordinates sourced from randomize_world.py red cylinders (corridor_shift=0
# nominal; update when regenerating worlds with non-zero corridor_shift).
# Order: visit in sequence from robot spawn at (-2.0, -2.0).
# ---------------------------------------------------------------------------
MAZE_WAYPOINTS: list[tuple[float, float]] = [
    (-2.8, -1.8),  # enemy_apex_2 — nearest to spawn, lower corridor apex
    (-4.5, -0.2),  # enemy_wide_1 — mid-maze wide section
    (-3.2,  1.2),  # enemy_apex_1 — upper corridor apex
]
# Intermediate waypoint bonus (episode continues after this reward).
WAYPOINT_REWARD = 50.0
# Reward when the LAST waypoint is cleared (episode ends — maze fully solved).
FINAL_REWARD = 250.0
# Per-step penalty each time the safety shield fires.
# CRITICAL: keep this small relative to the distance-shaping signal (20.0 ×
# dist_improvement per step). At 2.0 the shield fires 450×/ep after WP1 →
# -900 penalty completely drowns the WP2 direction signal → policy freezes.
# At 0.3 the same 450 fires = -135, leaving distance shaping audible → robot
# navigates through the corridor instead of vibrating against walls.
# The training_contract.yaml must also set shield_step_penalty: 0.3.
SHIELD_STEP_PENALTY = 0.3

# Proximity bonus constants — training_contract.py overwrites these at startup.
# A continuous per-step reward of up to PROXIMITY_REWARD_SCALE is given when the
# robot is within PROXIMITY_THRESHOLD metres of the next waypoint, tapering
# linearly to 0 at the threshold.  This creates a smooth gradient "funnel" that
# pulls the agent into each waypoint zone without overriding the distance signal.
PROXIMITY_REWARD_SCALE = 2.0
PROXIMITY_THRESHOLD = 1.5

def actor_checkpoint_in_features(state_dict: dict) -> int | None:
    """Infer actor input dim from weights (SAC: fc1.weight; TD3: net.0.weight)."""
    from training_contract import infer_actor_state_dim_from_checkpoint

    return infer_actor_state_dim_from_checkpoint(state_dict)


def pool_laser_to_bins(ranges: np.ndarray, range_max: float, range_min: float, n_bins: int) -> np.ndarray:
    """Min-pool laser ranges into n_bins covering the full sweep (no dropped tail rays)."""
    raw = np.asarray(ranges, dtype=np.float64)
    raw = np.where(raw < range_min, range_max, raw)
    raw = np.where(np.isfinite(raw), raw, range_max)
    n = len(raw)
    if n == 0:
        return np.full(n_bins, range_max, dtype=np.float32)
    edges = np.linspace(0, n, n_bins + 1, dtype=int)
    out = np.empty(n_bins, dtype=np.float32)
    for i in range(n_bins):
        seg = raw[edges[i]:edges[i + 1]]
        out[i] = float(np.min(seg)) if seg.size else range_max
    return out


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


PRESETS: dict[str, dict] = {
    "none": {},
    "pfe_sac_adapt": {
        "algo": "sac",
        "use_shield": True,
        "randomize_goal": True,
        # "box" goals: random targets in range (maze-friendly). --goal-mode safe_box enforces
        # straight-ray LiDAR clearance (often fails in dense mazes → repeated warnings).
        "goal_mode": "box",
        "grad_steps": 2,
        "goal_x_range": [-4.5, -2.0],
        "goal_y_range": [-4.5, -2.0],
        # Default 0.1s × 4000 steps ≈ 6.7 min/episode wall time; 0.05 halves that (override on CLI if needed).
        "env_step_sleep_sec": 0.05,
    },
    "pfe_td3_adapt": {
        "algo": "td3",
        "use_shield": True,
        "randomize_goal": True,
        "goal_mode": "box",
        "grad_steps": 2,
        "goal_x_range": [-4.5, -2.0],
        "goal_y_range": [-4.5, -2.0],
        "env_step_sleep_sec": 0.05,
    },
    # Full-maze sequential waypoint presets — visit all MAZE_WAYPOINTS in order.
    # Warm-start from a pre-trained collision-avoidance checkpoint with --load-pretrained.
    # Shield MUST be on: without it the episode ends on the first wall touch (~40 steps)
    # and the robot never has enough time to navigate past WP1. With the shield the robot
    # bounces off walls and the episode continues, allowing WP2/WP3 exploration.
    # grad_steps=1: waypoint episodes are longer so there are more gradient steps per
    # episode already; 2 caused aggressive policy updates that collapsed WP1 behavior.
    "pfe_sac_waypoint": {
        "algo": "sac",
        "waypoint_mode": True,
        "use_shield": True,
        "grad_steps": 1,
        "env_step_sleep_sec": 0.05,
        # Hard cap raised to 1500 steps (75 s).  With adaptive_steps=True the
        # actual per-episode budget starts at 600 (30 s) and ramps up as the
        # agent demonstrates waypoint progress, so early stuck episodes stay
        # short while later episodes get the room they need for WP2/WP3.
        "max_episode_steps": 1500,
        # Adaptive step budget: starts at adaptive_base_steps (600) and adds
        # adaptive_step_inc (300) for every waypoint the agent reached in the
        # best episode seen so far, capped at max_episode_steps.
        "adaptive_steps": True,
        "adaptive_base_steps": 600,
        "adaptive_step_inc": 300,
        # Regenerate world every 50 episodes (randomly chosen style 0-3) so the
        # replay buffer gets style diversity quickly.  Gazebo must be restarted
        # to load the new .world; train_waypoint.sh handles this via _train().
        # Each Gazebo session uses one layout style; per-session regen reloads geometry
        # friction + shift randomness within the active style.
        "world_regen_script": "sim_assets/scripts/randomize_world.py",
        "world_regen_interval": 50,
    },
    "pfe_td3_waypoint": {
        "algo": "td3",
        "waypoint_mode": True,
        "use_shield": True,
        "grad_steps": 1,
        "env_step_sleep_sec": 0.05,
        "max_episode_steps": 1500,
        "adaptive_steps": True,
        "adaptive_base_steps": 600,
        "adaptive_step_inc": 300,
        "world_regen_script": "sim_assets/scripts/randomize_world.py",
        "world_regen_interval": 50,
    },
}


def inject_preset_defaults(parser: argparse.ArgumentParser) -> None:
    argv = sys.argv[1:]
    if "--preset" not in argv:
        return
    i = argv.index("--preset")
    if i + 1 >= len(argv):
        return
    name = argv[i + 1]
    if name in PRESETS and name != "none":
        parser.set_defaults(**PRESETS[name])


def git_rev_and_dirty(repo_root: str) -> tuple[str, bool | None]:
    try:
        rev = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        h = (rev.stdout or "").strip() or "unknown"
        st = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        dirty = bool((st.stdout or "").strip())
        return h, dirty
    except (OSError, subprocess.SubprocessError):
        return "unknown", None


def write_run_manifest(
    path: str,
    args_ns: argparse.Namespace,
    workspace_dir: str,
    algo: str,
    *,
    contract_bundle: dict | None = None,
) -> None:
    ros2_ws = os.path.abspath(os.path.join(workspace_dir, "..", "..", ".."))
    commit, dirty = git_rev_and_dirty(ros2_ws)
    payload = {
        "time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "hostname": socket.gethostname(),
        "pid": os.getpid(),
        "argv": sys.argv,
        "algo": algo,
        "git_commit": commit,
        "git_dirty": dirty,
        "ros2_ws": ros2_ws,
        "args": {k: getattr(args_ns, k) for k in vars(args_ns)},
        "torch_version": torch.__version__,
        "numpy_version": np.__version__,
    }
    if contract_bundle:
        payload["training_contract_bundle"] = contract_bundle
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


class SafeNavAgent(Node):
    def __init__(
        self,
        algo_choice,
        force_restart,
        *,
        device_str: str = "cpu",
        randomize_goal: bool = False,
        goal_mode: str = "box",
        goal_x_range: tuple[float, float] = (-5.0, -2.0),
        goal_y_range: tuple[float, float] = (-5.0, -2.0),
        train_seed: int | None = None,
        goal_sample_attempts: int = 48,
        goal_safe_margin: float = 1.15,
        goal_sector_halfwidth_bins: int = 2,
        no_sim_reset: bool = False,
        reset_service: str = "/reset_simulation",
        reset_service_wait_sec: float = 45.0,
        reset_reply_wait_sec: float = 5.0,
        reset_fire_and_forget: bool = False,
        env_step_sleep_sec: float = 0.1,
        use_shield: bool = False,
        waypoint_mode: bool = False,
        waypoints: list[tuple[float, float]] | None = None,
        lr: float = 3e-4,
        waypoint_goal_radius_m: float = 0.0,
        enable_tensorboard: bool = True,
    ):
        super().__init__('safe_nav_agent')
        self.algo_choice = algo_choice
        # Thesis benchmark: standard RL (False) vs shield-intervention hybrid (True).
        self.use_shield = bool(use_shield)
        # Sequential waypoint mode: visit all MAZE_WAYPOINTS in order before done=True.
        self._waypoint_mode = bool(waypoint_mode)
        self.current_waypoint_idx = 0
        # Set True in step_environment when all waypoints cleared in one episode (for early-stop).
        self._maze_solved_this_episode = False
        # DR-ready waypoint list: None → copies module-level MAZE_WAYPOINTS default.
        # For Phase 3 domain randomisation, call set_waypoints() before each reset_env().
        self._maze_waypoints: list[tuple[float, float]] = (
            list(waypoints) if waypoints is not None else list(MAZE_WAYPOINTS)
        )
        # Waypoint "clear" radius: slightly larger than GOAL_RADIUS reduces false
        # negatives from odom / control jitter; dense shaping still requires progress.
        _wgr = float(waypoint_goal_radius_m)
        self._goal_clear_radius = (
            _wgr if (self._waypoint_mode and _wgr > 1e-6) else float(GOAL_RADIUS)
        )
        if self._waypoint_mode and randomize_goal:
            # Waypoint mode owns the target; randomize_goal is silently disabled.
            randomize_goal = False
        self._no_sim_reset = bool(no_sim_reset)
        self._reset_service = reset_service.strip() or "/reset_simulation"
        self._reset_service_wait_sec = max(5.0, float(reset_service_wait_sec))
        self._reset_reply_wait_sec = max(0.0, float(reset_reply_wait_sec))
        self._reset_fire_and_forget = bool(reset_fire_and_forget)
        # If waiting for reset service times out (common without Gazebo), skip sim reset on later episodes.
        self._reset_skip_sim_reset = False
        self._logged_invalid_sim_training = False
        self._env_step_sleep_sec = float(max(0.0, min(2.0, env_step_sleep_sec)))
        if device_str == "cuda" and not torch.cuda.is_available():
            self.get_logger().warning("CUDA requested but not available; using CPU.")
            self.device = torch.device("cpu")
        else:
            self.device = torch.device(device_str)
        
        # ROS 2 Communications
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.reset_client = self.create_client(Empty, self._reset_service)

        # Set each reset_env(); evaluate_agent --require-reset reads this.
        self._last_sim_reset_applied = True

        # --- NEW: SENSOR AND GOAL DATA ---
        self.lidar_bins = LIDAR_BINS
        self.state_dim = self.lidar_bins + 2  # LiDAR + DeltaX + DeltaY
        self.action_dim = 2
        
        self._lidar_range_max = 30.0
        self._lidar_range_min = 0.12
        self.current_scan = np.ones(self.lidar_bins, dtype=np.float32) * self._lidar_range_max
        self.robot_x = -2.0
        self.robot_y = -2.0
        self.robot_yaw = 0.0
        self.scan_received = False
        self.odom_received = False
        self._scan_angle_min = -math.pi
        self._scan_angle_inc = 0.01
        self._n_rays = 360
        self._pool_edges: np.ndarray | None = None

        # Default goal (fixed maze exit); overridden each episode if randomize_goal
        self._default_target = (-3.5, -3.5)
        self.target_x, self.target_y = self._default_target
        self._randomize_goal = bool(randomize_goal)
        self._goal_mode = (goal_mode or "box").strip().lower()
        self._goal_x_range = (min(goal_x_range), max(goal_x_range))
        self._goal_y_range = (min(goal_y_range), max(goal_y_range))
        self._goal_sample_attempts = max(4, int(goal_sample_attempts))
        self._goal_safe_margin = float(goal_safe_margin)
        self._goal_sector_halfwidth_bins = max(0, int(goal_sector_halfwidth_bins))
        self._rng = np.random.default_rng(train_seed)
        self.prev_dist = 99.0  # Previous Euclidean distance to goal (dense shaping).
        self.prev_cmd = np.zeros(2, dtype=np.float64)  # [linear.x, angular.z] after clipping
        if self._waypoint_mode and abs(self._goal_clear_radius - float(GOAL_RADIUS)) > 1e-6:
            self.get_logger().info(
                f"Waypoint clear radius = {self._goal_clear_radius:.2f} m "
                f"(contract GOAL_RADIUS={GOAL_RADIUS} m)."
            )

        # Load Algorithm — add new algos by creating networks_<name>.py +
        # trainer_<name>.py that match the interface documented in networks_custom.py.
        from replay_buffer import ExperienceReplay
        if self.algo_choice == 'td3':
            from networks_td3 import ActorNetwork, CriticNetwork
            from trainer_td3 import TD3Trainer as Trainer
        elif self.algo_choice == 'custom':
            from networks_custom import ActorNetwork, CriticNetwork  # type: ignore[import]
            from trainer_custom import CustomTrainer as Trainer      # type: ignore[import]
        else:  # default: sac
            from networks_sac import ActorNetwork, CriticNetwork
            from trainer_sac import SACTrainer as Trainer
            
        # 500k for waypoint mode: 16 GB DDR4 machine has ~8 GB free; 500k costs ~160 MB.
        # Larger buffer keeps rare WP2/WP3 experiences alive longer (300k was fine but
        # better diversity helps once WP2 is first reached). 200k for adapt mode (was 100k).
        _replay_cap = 500_000 if self._waypoint_mode else 200_000
        self.memory = ExperienceReplay(max_size=_replay_cap, state_dim=self.state_dim, action_dim=self.action_dim)
        self.actor = ActorNetwork(self.state_dim, self.action_dim).to(self.device)
        self.critic = CriticNetwork(self.state_dim, self.action_dim).to(self.device)
        # Trainer deepcopies critic → critic_target before torch.compile, so the target
        # stays a plain Module (safe for deepcopy/soft-update) while actor+critic gain
        # torch.compile's AVX-512 / oneDNN fusion on every forward+backward pass.
        self.trainer = Trainer(self.actor, self.critic, self.memory, device=self.device, lr=lr)
        # NOTE: torch.compile is applied AFTER all checkpoint loading (see apply_torch_compile()).
        # Do NOT compile here — load_state_dict on an OptimizedModule requires _orig_mod.* keys.
        if self._randomize_goal:
            self.get_logger().info(
                f"Adaptation-style training: mode={self._goal_mode} goals in "
                f"x∈[{self._goal_x_range[0]:.2f},{self._goal_x_range[1]:.2f}] "
                f"y∈[{self._goal_y_range[0]:.2f},{self._goal_y_range[1]:.2f}] (seed={train_seed})"
            )
        if self._no_sim_reset:
            self.get_logger().warning(
                "--no-sim-reset: episodes will NOT call /reset_simulation (robot is not teleported)."
            )

        # Logging / checkpoints next to this file (works regardless of $HOME layout)
        self.workspace_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_dir = os.path.join(self.workspace_dir, "pfe_logs")
        self.model_dir = os.path.join(self.workspace_dir, "trained_models")
        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.model_dir, exist_ok=True)
        
        # Separate adapt vs waypoint paths so an adapt run never overwrites a maze checkpoint.
        # adapt mode  → sac_actor_adapt.pth  /  sac_adapt_metrics.csv
        # waypoint mode → sac_actor_maze.pth  /  sac_maze_metrics.csv
        _mode_tag = "maze" if self._waypoint_mode else "adapt"
        if enable_tensorboard:
            self.writer = SummaryWriter(os.path.join(self.log_dir, f"tb_{self.algo_choice}_{_mode_tag}"))
        else:
            self.writer = None
        self.csv_path = os.path.join(self.log_dir, f"{self.algo_choice}_{_mode_tag}_metrics.csv")
        self.actor_path = os.path.join(self.model_dir, f"{self.algo_choice}_actor_{_mode_tag}.pth")
        
        if not force_restart and os.path.exists(self.actor_path):
            ckpt = torch.load(self.actor_path, map_location=self.device)
            ckpt_in = actor_checkpoint_in_features(ckpt)
            if ckpt_in is not None and ckpt_in != self.state_dim:
                self.get_logger().warning(
                    f"Checkpoint {self.actor_path} was trained with state_dim={ckpt_in} "
                    f"(e.g. {(ckpt_in - 2)} LiDAR bins + goal), but this code uses state_dim={self.state_dim} "
                    f"({self.lidar_bins} bins + goal). Starting with a fresh actor. "
                    f"Remove/rename the old .pth or use --force-restart to silence this. "
                    f"To reuse old weights you would need the same LiDAR bin count as when it was trained."
                )
            else:
                try:
                    self.actor.load_state_dict(ckpt)
                    self.get_logger().info(f"🟢 {self.algo_choice.upper()} Maze Brain Resumed!")
                except RuntimeError as _load_exc:
                    self.get_logger().warning(
                        f"Checkpoint shape mismatch ({_load_exc!r}). "
                        "Network architecture likely changed (hidden size upgrade). "
                        "Starting with fresh weights — delete the old .pth to suppress this warning."
                    )
            
        self.total_steps = 0
        # Goals-specific log (goal_x, goal_y per episode) — only written in adapt mode.
        # Kept separate from csv_path so the two files never share a format.
        self._adapt_csv_path = os.path.join(self.log_dir, f"{self.algo_choice}_adapt_goals.csv")
        if self._randomize_goal and not os.path.isfile(self._adapt_csv_path):
            with open(self._adapt_csv_path, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(
                    ["episode", "reward", "total_env_steps", "ep_steps", "avg_time_ms", "goal_x", "goal_y"]
                )

    def apply_torch_compile(self) -> None:
        """Call ONCE after all checkpoint loading is complete.

        torch.compile wraps modules in OptimizedModule whose state_dict keys gain an
        '_orig_mod.' prefix.  Loading a plain .pth into a compiled module therefore
        fails.  By deferring compilation to after every load_state_dict call we keep
        full checkpoint compatibility while still getting AVX-512/oneDNN fusion for
        all forward/backward passes during the actual training loop.
        """
        if not hasattr(torch, "compile"):
            return
        try:
            _ca = torch.compile(self.actor,  mode="reduce-overhead")
            _cc = torch.compile(self.critic, mode="reduce-overhead")
            # Patch both agent refs AND trainer refs (trainer holds the same objects).
            self.actor  = self.trainer.actor  = _ca
            self.critic = self.trainer.critic = _cc
            self.get_logger().info(
                "torch.compile(reduce-overhead) ON — "
                "expect ~20-30 s extra on the FIRST episode (one-time JIT compile)."
            )
        except Exception as _ce:
            self.get_logger().warning(f"torch.compile skipped ({_ce}); using eager mode.")

    def odom_callback(self, msg):
        """Constantly updates the robot's GPS location"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def laser_callback(self, msg):
        """Compress full sweep LiDAR into lidar_bins via min-pooling (preserves nearest hit per sector)."""
        self._lidar_range_max = float(msg.range_max)
        self._lidar_range_min = float(msg.range_min)
        ranges = msg.ranges
        self._scan_angle_min = float(msg.angle_min)
        self._scan_angle_inc = float(msg.angle_increment)
        self._n_rays = max(1, len(ranges))
        self._pool_edges = np.linspace(0, self._n_rays, self.lidar_bins + 1, dtype=int)
        self.current_scan = pool_laser_to_bins(
            ranges, self._lidar_range_max, self._lidar_range_min, self.lidar_bins
        )
        self.scan_received = True

    def _world_bearing_to_ray_index(self, bearing_world: float) -> int:
        b = bearing_world - self.robot_yaw
        b = (b + math.pi) % (2.0 * math.pi) - math.pi
        if abs(self._scan_angle_inc) < 1e-12:
            return 0
        ri = int(round((b - self._scan_angle_min) / self._scan_angle_inc))
        return int(np.clip(ri, 0, self._n_rays - 1))

    def _ray_index_to_bin(self, ray_index: int) -> int:
        if self._pool_edges is None:
            return 0
        for j in range(self.lidar_bins):
            if self._pool_edges[j] <= ray_index < self._pool_edges[j + 1]:
                return j
        return self.lidar_bins - 1

    def _sample_episode_goal(self) -> None:
        """Set target_x/y after reset once odom + lidar are valid."""
        if not self._randomize_goal:
            return
        rx, ry = self.robot_x, self.robot_y

        if self._goal_mode == "box":
            self.target_x = float(self._rng.uniform(*self._goal_x_range))
            self.target_y = float(self._rng.uniform(*self._goal_y_range))
            return

        if self._goal_mode != "safe_box":
            self.get_logger().warning(f"Unknown goal_mode {self._goal_mode!r}; using box.")
            self.target_x = float(self._rng.uniform(*self._goal_x_range))
            self.target_y = float(self._rng.uniform(*self._goal_y_range))
            return

        # Try strict margin first, then relax (mazes often block the straight ray even when a goal is reachable).
        margins = (
            self._goal_safe_margin,
            max(1.06, self._goal_safe_margin - 0.06),
            max(1.0, self._goal_safe_margin - 0.12),
            1.0,
            0.94,
        )
        tries_per_margin = max(10, self._goal_sample_attempts // len(margins))
        for margin in margins:
            for _ in range(tries_per_margin):
                gx = float(self._rng.uniform(*self._goal_x_range))
                gy = float(self._rng.uniform(*self._goal_y_range))
                dist = math.hypot(gx - rx, gy - ry)
                if dist < 0.12:
                    continue
                bearing_w = math.atan2(gy - ry, gx - rx)
                ri = self._world_bearing_to_ray_index(bearing_w)
                cb = self._ray_index_to_bin(ri)
                w = self._goal_sector_halfwidth_bins
                lo = max(0, cb - w)
                hi = min(self.lidar_bins - 1, cb + w)
                sector_min = float(np.min(self.current_scan[lo : hi + 1]))
                if sector_min >= dist * margin + 0.03:
                    self.target_x, self.target_y = gx, gy
                    if margin + 1e-3 < self._goal_safe_margin:
                        self.get_logger().info(
                            f"safe_box: goal accepted with relaxed clearance margin={margin:.2f} "
                            f"(sector_min={sector_min:.2f} m, dist={dist:.2f} m)."
                        )
                    return

        self.target_x = float(self._rng.uniform(*self._goal_x_range))
        self.target_y = float(self._rng.uniform(*self._goal_y_range))
        self.get_logger().warning(
            "safe_box: no LiDAR-clear sample even after relaxing margin; using uniform goal. "
            "Consider --goal-mode box, wider --goal-*-range, or a box inside known free space."
        )

    def set_waypoints(self, waypoints: list[tuple[float, float]]) -> None:
        """Replace the active waypoint sequence for the NEXT episode.

        Call this BEFORE reset_env() each episode to enable Phase 3 Domain
        Randomisation.  A DR generator can inject freshly-sampled coordinates
        without subclassing or modifying any core logic::

            for episode in range(N):
                agent.set_waypoints(dr_generator.sample())   # Phase 3 hook
                state = agent.reset_env()
                ...

        Args:
            waypoints: Non-empty list of (x, y) world-frame tuples.

        Raises:
            ValueError: if the list is empty.
        """
        if not waypoints:
            raise ValueError("waypoints must contain at least one (x, y) tuple.")
        self._maze_waypoints = list(waypoints)

    def get_state(self):
        """LiDAR normalized to [0,1] (unknown/far = 1) plus goal offset in metres.

        Goal deltas are kept in raw metres (same scale used for all existing checkpoints).
        All trained .pth files expect this exact observation format — do not normalize
        delta_x/delta_y unless training fully from scratch with a new checkpoint.
        """
        delta_x = self.target_x - self.robot_x
        delta_y = self.target_y - self.robot_y
        denom = max(self._lidar_range_max, 1e-6)
        scan_n = np.clip(self.current_scan.astype(np.float64), 0.0, denom) / denom
        state = np.append(scan_n.astype(np.float32), [np.float32(delta_x), np.float32(delta_y)])
        return state

    def _min_front_side_lidar(self) -> float:
        """Minimum pooled LiDAR range over the forward ±90° hemisphere (robot frame)."""
        if self._pool_edges is None or self._n_rays < 1:
            return float(np.min(self.current_scan))
        amin = self._scan_angle_min
        inc = self._scan_angle_inc
        vals: list[float] = []
        for j in range(self.lidar_bins):
            r0 = int(self._pool_edges[j])
            r1 = int(self._pool_edges[j + 1])
            mid = (r0 + r1 - 1) // 2
            mid = int(np.clip(mid, 0, self._n_rays - 1))
            ang = amin + float(mid) * inc
            ang = (ang + math.pi) % (2.0 * math.pi) - math.pi
            if abs(ang) <= math.pi / 2.0 + 1e-3:
                vals.append(float(self.current_scan[j]))
        return float(min(vals)) if vals else float(np.min(self.current_scan))

    def _closest_front_side_obstacle_bearing(self) -> float:
        """Bearing (rad, robot frame) to the pooled bin with min range in the forward hemisphere."""
        if self._pool_edges is None or self._n_rays < 1:
            return 0.0
        amin = self._scan_angle_min
        inc = self._scan_angle_inc
        best_j = 0
        best_r = float("inf")
        for j in range(self.lidar_bins):
            r0 = int(self._pool_edges[j])
            r1 = int(self._pool_edges[j + 1])
            mid = (r0 + r1 - 1) // 2
            mid = int(np.clip(mid, 0, self._n_rays - 1))
            ang = amin + float(mid) * inc
            ang = (ang + math.pi) % (2.0 * math.pi) - math.pi
            if abs(ang) <= math.pi / 2.0 + 1e-3:
                r = float(self.current_scan[j])
                if r < best_r:
                    best_r = r
                    best_j = j
        r0 = int(self._pool_edges[best_j])
        r1 = int(self._pool_edges[best_j + 1])
        mid = (r0 + r1 - 1) // 2
        mid = int(np.clip(mid, 0, self._n_rays - 1))
        b = amin + float(mid) * inc
        return (b + math.pi) % (2.0 * math.pi) - math.pi

    def _shield_cmd_override(self) -> tuple[float, float]:
        """Slow backup and turn away from the nearest front/side obstacle."""
        theta = self._closest_front_side_obstacle_bearing()
        lin = float(np.clip(-SHIELD_BACKUP_LIN, -0.4, 0.0))
        if abs(theta) < 0.08:
            ang = float(SHIELD_TURN_ANG)
        else:
            ang = float(-np.sign(theta) * SHIELD_TURN_ANG)
        ang = float(np.clip(ang, -1.0, 1.0))
        return lin, ang

    def reset_env(self):
        """Teleport and Calculate Initial Distance"""
        self.cmd_vel_pub.publish(Twist())
        # True if a sim reset RPC was issued this call, OR training/eval uses --no-sim-reset.
        sim_reset_issued = False
        do_sim_reset = not self._no_sim_reset
        if do_sim_reset and self._reset_skip_sim_reset:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.reset_client.service_is_ready():
                self.get_logger().info(
                    f"{self._reset_service} available again; resuming sim reset between episodes."
                )
                self._reset_skip_sim_reset = False
            else:
                do_sim_reset = False

        if do_sim_reset:
            self.get_logger().info(
                f"reset_env: waiting for {self._reset_service} "
                f"(max {self._reset_service_wait_sec:.0f}s) ..."
            )
            wait_deadline = time.time() + self._reset_service_wait_sec
            last_warn = time.time()
            while True:
                if not rclpy.ok():
                    raise KeyboardInterrupt
                # Do NOT use Client.wait_for_service() here: it sleeps without spinning this node,
                # so discovery and in-flight service replies stall and the loop looks hung forever.
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.reset_client.service_is_ready():
                    break
                if time.time() > wait_deadline:
                    if self._reset_fire_and_forget:
                        self.get_logger().warning(
                            f"Timed out after {self._reset_service_wait_sec:.0f}s waiting for "
                            f"{self._reset_service}; continuing without sim reset for this and later episodes "
                            "until the service appears (--reset-fire-and-forget). "
                            "Start Gazebo + gazebo_ros to enable /reset_simulation, or use --no-sim-reset."
                        )
                        self._reset_skip_sim_reset = True
                        do_sim_reset = False
                        if not self._logged_invalid_sim_training:
                            self._logged_invalid_sim_training = True
                            self.get_logger().error(
                                "No simulator: /reset_simulation never appeared — this process does NOT launch "
                                "Gazebo. Training without reset repeats the same episode (invalid). Stop (Ctrl+C). "
                                "Then either: (1) bash train_menu.sh → 2 so Gazebo + brain start, or "
                                "(2) bash start_pfe.sh …, or (3) ros2 launch gazebo_ros gazebo.launch.py world:=… "
                                "then spawn the robot, then rerun this brain in a sourced ROS shell."
                            )
                        break
                    self.get_logger().error(
                        f"Timed out after {self._reset_service_wait_sec:.0f}s waiting for "
                        f"{self._reset_service} (std_srvs/srv/Empty).\n"
                        "  • Start Gazebo first, e.g.: ros2 launch gazebo_ros gazebo.launch.py world:=PATH.world\n"
                        "  • Or run without sim reset: add --no-sim-reset (robot is not teleported each episode).\n"
                        "  • Or set a different service: --reset-service /your/empty/service\n"
                        "  • Or add --reset-fire-and-forget to continue if Gazebo is down or reset hangs.\n"
                        "  • Check: ros2 service list | grep -i reset"
                    )
                    raise RuntimeError(f"Missing ROS service {self._reset_service}")
                if time.time() - last_warn > 5.0:
                    remaining = max(0.0, wait_deadline - time.time())
                    self.get_logger().warning(
                        f"Still waiting for {self._reset_service} "
                        f"({remaining:.0f}s left — start Gazebo or check: ros2 service list | grep reset)"
                    )
                    last_warn = time.time()

            if do_sim_reset:
                try:
                    fut = self.reset_client.call_async(Empty.Request())
                    sim_reset_issued = True
                except RuntimeError as exc:
                    # Fire-and-forget can leave a pending future; rclpy then rejects the next call_async.
                    msg = str(exc).lower()
                    if "pending" in msg or "sequence" in msg:
                        self.get_logger().warning(
                            f"Reset client had a stuck pending request ({exc!r}); recreating client."
                        )
                        self.destroy_client(self.reset_client)
                        self.reset_client = self.create_client(Empty, self._reset_service)
                        fut = self.reset_client.call_async(Empty.Request())
                        sim_reset_issued = True
                    else:
                        raise
                # Gazebo's reset handler sometimes never returns; do not block training indefinitely.
                for _ in range(80):
                    if fut.done():
                        break
                    rclpy.spin_once(self, timeout_sec=0.02)
                if self._reset_fire_and_forget and not fut.done():
                    self.get_logger().info(
                        f"{self._reset_service}: fire-and-forget (--reset-fire-and-forget); not waiting for reply."
                    )
                elif not self._reset_fire_and_forget:
                    reply_end = time.time() + self._reset_reply_wait_sec
                    last_pb = 0.0
                    while rclpy.ok() and not fut.done() and time.time() < reply_end:
                        rclpy.spin_once(self, timeout_sec=0.05)
                        if time.time() - last_pb > 2.0:
                            self.get_logger().info(
                                f"Waiting for {self._reset_service} reply (max {self._reset_reply_wait_sec:.1f}s)..."
                            )
                            last_pb = time.time()
                    if not fut.done():
                        self.get_logger().warning(
                            f"{self._reset_service} did not acknowledge within "
                            f"{self._reset_reply_wait_sec:.1f}s — continuing training anyway "
                            "(Gazebo reset can hang; use --reset-fire-and-forget or --no-sim-reset if needed)."
                        )
                # Drain reply or drop pending so the next episode never blocks on a ghost reset call.
                if not fut.done():
                    drain_end = time.time() + 5.0
                    while rclpy.ok() and not fut.done() and time.time() < drain_end:
                        rclpy.spin_once(self, timeout_sec=0.05)
                if fut.done():
                    try:
                        fut.result()
                    except Exception as exc:
                        self.get_logger().warning(f"{self._reset_service} returned: {exc!r}")
                else:
                    self.reset_client.remove_pending_request(fut)
                    self.get_logger().info(
                        f"Cleared pending {self._reset_service} request (no reply) so the next reset "
                        "can be sent — Gazebo often never completes this service."
                    )
        else:
            time.sleep(0.05)

        # If LiDAR/odom are already flowing, do not clear flags here: spin_once() from
        # inside another callback (e.g. hot_swap timer) can deadlock and leave flags false.
        if not (self.scan_received and self.odom_received):
            self.scan_received = False
            self.odom_received = False
            timeout = time.time() + 2.0
            while (not self.scan_received or not self.odom_received) and time.time() < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.45)
        for _ in range(8):
            rclpy.spin_once(self, timeout_sec=0.05)

        if self._waypoint_mode:
            self._maze_solved_this_episode = False
            # Reset to first waypoint; state vector will naturally point there.
            # _maze_waypoints may have been updated by set_waypoints() before this
            # call — that is the Phase 3 DR hook; no other changes needed here.
            self.current_waypoint_idx = 0
            self.target_x, self.target_y = self._maze_waypoints[0]
            self.get_logger().info(
                f"Waypoint mode reset → WP 1/{len(self._maze_waypoints)}: "
                f"({self.target_x:.2f}, {self.target_y:.2f})"
            )
        else:
            self._sample_episode_goal()

        # Starting distance for dense distance shaping (must match first step's prev_dist).
        self.prev_dist = math.hypot(self.target_x - self.robot_x, self.target_y - self.robot_y)
        self.prev_cmd[:] = 0.0
        # Harnesses (evaluate_agent --require-reset) can abort if sim reset was skipped while Gazebo was expected.
        self._last_sim_reset_applied = bool(self._no_sim_reset or sim_reset_issued)
        return self.get_state()

    def step_environment(self, action):
        cmd_lin = float(np.clip(action[0], 0.0, 0.4))
        cmd_ang = float(np.clip(action[1], -1.0, 1.0))

        min_fs = self._min_front_side_lidar()
        shield_intervention = bool(self.use_shield and min_fs < SHIELD_CRITICAL_RANGE)
        if shield_intervention:
            cmd_lin, cmd_ang = self._shield_cmd_override()
        # Record the ACTUALLY EXECUTED command so the replay buffer stores the real
        # MDP transition.  When the shield fires, cmd_lin/cmd_ang differ from action[];
        # storing the pre-shield action would make the critic learn wrong Q-values.
        self._last_executed_action = np.array([cmd_lin, cmd_ang], dtype=np.float32)

        msg = Twist()
        msg.linear.x = cmd_lin
        msg.angular.z = cmd_ang
        self.cmd_vel_pub.publish(msg)
        if self._env_step_sleep_sec > 0.0:
            time.sleep(self._env_step_sleep_sec)

        self.prev_cmd[:] = np.array([cmd_lin, cmd_ang], dtype=np.float64)

        # Distance to the CURRENT target (waypoint or single goal).
        current_dist = math.hypot(self.target_x - self.robot_x, self.target_y - self.robot_y)
        done = False
        reward = 0.0

        # Dense distance-shaping: positive when closing in, negative when drifting away.
        reward += (self.prev_dist - current_dist) * DISTANCE_MULTIPLIER
        reward -= STEP_PENALTY

        # Proximity bonus: continuous reward that pulls the agent into the next
        # waypoint zone.  Tapers linearly from +PROXIMITY_REWARD_SCALE at dist=0
        # to 0 at PROXIMITY_THRESHOLD.  Zero outside the threshold so it does not
        # interfere with long-range navigation.
        if PROXIMITY_THRESHOLD > 0.0 and current_dist < PROXIMITY_THRESHOLD:
            reward += PROXIMITY_REWARD_SCALE * (1.0 - current_dist / PROXIMITY_THRESHOLD)

        goal_reached = current_dist < self._goal_clear_radius
        # Re-sample LiDAR AFTER the action has been applied (post-motion collision check).
        min_fs_post = self._min_front_side_lidar()
        collision_unshielded = bool(
            not self.use_shield and min_fs_post < SHIELD_CRITICAL_RANGE
        )

        if goal_reached and self._waypoint_mode:
            # --- Sequential waypoint logic ---
            n_wp = len(self._maze_waypoints)
            is_final = (self.current_waypoint_idx >= n_wp - 1)

            if is_final:
                # All waypoints cleared → maze fully solved.
                reward += FINAL_REWARD
                done = True
                self._maze_solved_this_episode = True
                self.prev_cmd[:] = 0.0
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info(
                    f"🏆 MAZE SOLVED! All {n_wp} waypoints reached in sequence!"
                )
            else:
                # Intermediate waypoint reached — advance to next, episode continues.
                reward += WAYPOINT_REWARD
                self.current_waypoint_idx += 1
                self.target_x, self.target_y = self._maze_waypoints[self.current_waypoint_idx]
                # CRITICAL: reset prev_dist to new target so shaping on the very next
                # step is computed against the correct reference distance, not the old one.
                self.prev_dist = math.hypot(
                    self.target_x - self.robot_x, self.target_y - self.robot_y
                )
                self.get_logger().info(
                    f"✅ Waypoint {self.current_waypoint_idx}/{n_wp} reached! "
                    f"→ Next target: ({self.target_x:.2f}, {self.target_y:.2f})"
                )
                # Return immediately: prev_dist is already updated above; do NOT
                # overwrite it with current_dist (which was distance to the OLD target).
                return self.get_state(), reward, done

        elif goal_reached:
            # Single-target mode (backward-compatible, used when --waypoint-mode is off).
            reward += GOAL_REWARD
            done = True
            self.prev_cmd[:] = 0.0
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("🎯 TARGET REACHED!")

        elif shield_intervention:
            # Shield fired and overrode the command — robot is saved, episode CONTINUES.
            # Apply a small per-step penalty to discourage approaching walls, but do not
            # terminate: terminating here would penalise the shield for doing its job.
            reward -= SHIELD_STEP_PENALTY

        elif collision_unshielded:
            # Actual collision in no-shield mode — terminate with large penalty.
            reward -= COLLISION_TERMINAL_PENALTY
            done = True
            self.prev_cmd[:] = 0.0
            self.cmd_vel_pub.publish(Twist())

        self.prev_dist = current_dist
        return self.get_state(), reward, done

    def log_csv(self, ep, rew, total_env_steps, ep_steps, avg_time):
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([ep, round(rew, 2), total_env_steps, ep_steps, round(avg_time, 4)])

    def log_adapt_csv(self, ep, rew, total_env_steps, ep_steps, avg_time):
        if not self._randomize_goal:
            return
        with open(self._adapt_csv_path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(
                [
                    ep,
                    round(rew, 2),
                    total_env_steps,
                    ep_steps,
                    round(avg_time, 4),
                    round(self.target_x, 4),
                    round(self.target_y, 4),
                ]
            )

def _actor_state_dict(actor) -> dict:
    """Return the plain state_dict regardless of whether actor is torch.compile-wrapped.

    torch.compile wraps the module in OptimizedModule; its state_dict() prefixes every
    key with '_orig_mod.' which makes the saved .pth incompatible with a plain load.
    Unwrapping via ._orig_mod gives the original clean keys.
    """
    unwrapped = getattr(actor, "_orig_mod", actor)
    return unwrapped.state_dict()


def _safe_backup_pth(src: str, backup_dir: str) -> str | None:
    """Copy an existing .pth to backup_dir/{name}_{timestamp}.pth; return dest or None."""
    if not os.path.isfile(src):
        return None
    import shutil
    from datetime import datetime as _dt
    os.makedirs(backup_dir, exist_ok=True)
    ts   = _dt.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.splitext(os.path.basename(src))[0]
    dest = os.path.join(backup_dir, f"{base}_{ts}.pth")
    shutil.copy2(src, dest)
    return dest


def _infer_best_ever_scope_from_env() -> str:
    """Map PFE_WORLD basename → scoped best_ever tag (eval_maze, lab, …)."""
    w = os.environ.get("PFE_WORLD", "").strip()
    if not w:
        return ""
    base = os.path.basename(os.path.abspath(os.path.expanduser(w)))
    if base.startswith("eval_") and base.endswith(".world"):
        return base[: -len(".world")]
    if "current_random_lab" in base:
        return "lab"
    return ""


def _normalize_best_ever_scope(scope: str) -> str:
    s = (scope or "").strip().lower().replace("/", "_").replace(".", "_")
    if s in ("", "global", "default", "any"):
        return ""
    if s == "maze" and not s.startswith("eval_"):
        return "eval_maze"
    return s


def _resolve_best_ever_path(model_dir: str, algo: str, mode_tag: str, scope: str) -> str:
    """Global lab peak vs per-eval-world peak (e.g. …_best_ever_eval_maze.pth)."""
    suf = _normalize_best_ever_scope(scope)
    stem = f"{algo}_actor_{mode_tag}_best_ever"
    name = f"{stem}_{suf}.pth" if suf else f"{stem}.pth"
    return os.path.join(model_dir, name)


def _read_best_ever_sidecar(path: str, suffix: str, default):
    p = path + suffix
    if not os.path.isfile(p):
        return default
    try:
        with open(p, encoding="utf-8") as f:
            return type(default)(f.read().strip())
    except (OSError, ValueError, TypeError):
        return default


def _write_best_ever_sidecar(path: str, suffix: str, value) -> None:
    with open(path + suffix, "w", encoding="utf-8") as f:
        f.write(str(value))


def _clear_best_ever_sidecars(best_ever_path: str) -> None:
    for suf in (".wp", ".floor", ".steps", ".solved"):
        try:
            os.remove(best_ever_path + suf)
        except OSError:
            pass


def _waypoint_run_beats_disk(
    wp_cleared: int,
    solved: bool,
    ep_steps: int,
    ep_reward: float,
    *,
    stored_wp: int,
    stored_solved: bool,
    stored_steps: int,
    n_wp: int,
    stored_reward: float,
) -> bool:
    """Lexicographic: more WPs → full solve → fewer steps → reward (not reward alone)."""
    if wp_cleared > stored_wp:
        return True
    if wp_cleared < stored_wp:
        return False
    # Same waypoint tier
    if solved and not stored_solved:
        return True
    if stored_solved and not solved:
        return False
    if solved and stored_solved and wp_cleared >= n_wp:
        if ep_steps < stored_steps - 5:
            return True
        if ep_steps > stored_steps + 5:
            return False
        return ep_reward > stored_reward + 1e-6
    # Partial progress at same tier: allow reward tie-break only (lab shaping differs by world)
    return ep_reward > stored_reward + 1e-6


def main():
    parser = argparse.ArgumentParser(
        description="SAC/TD3 local training for goal navigation; use --preset pfe_sac_adapt or --randomize-goal."
    )
    parser.add_argument(
        "--preset",
        type=str,
        default="none",
        choices=("none", "pfe_sac_adapt", "pfe_td3_adapt", "pfe_sac_waypoint", "pfe_td3_waypoint"),
        help="Inject defaults before parsing other flags (put --preset first, then overrides).",
    )
    parser.add_argument("--algo", type=str, default="td3")
    parser.add_argument("--force-restart", action="store_true")
    parser.add_argument(
        "--max-episode-steps",
        type=int,
        default=4000,
        help="Hard cap per episode. Wall time per episode is roughly max_episode_steps × env_step_sleep_sec.",
    )
    parser.add_argument(
        "--env-step-sleep-sec",
        type=float,
        default=0.1,
        metavar="SEC",
        help="Sleep after each env step (pacing vs Gazebo). Example: 4000 steps × 0.1s ≈ 6.7 min/episode; "
        "use 0.05 or 0.03 to go faster (preset pfe_*_adapt defaults to 0.05). Set 0 to disable (CPU-heavy).",
    )
    parser.add_argument(
        "--use-shield",
        action="store_true",
        help="Enable LiDAR safety shield (override cmd_vel near obstacles); off benchmarks standard SAC/TD3.",
    )
    parser.add_argument(
        "--waypoint-mode",
        action="store_true",
        help=(
            "Enable sequential waypoint navigation through MAZE_WAYPOINTS. "
            "The episode ends only when ALL waypoints are visited (maze solved) or "
            "the robot collides. The 38-dim observation is unchanged — target_x/y "
            "update dynamically to the next waypoint after each one is cleared."
        ),
    )
    parser.add_argument(
        "--load-pretrained",
        type=str,
        default="",
        metavar="PATH",
        help=(
            "Warm-start: load pre-trained actor weights from PATH before training. "
            "The checkpoint MUST have been trained with an identical network "
            "(state_dim=38, action_dim=2 — i.e. LIDAR_BINS=36). "
            "If dimensions mismatch the load is skipped with an error log. "
            "Example: --load-pretrained trained_models/sac_actor_maze.pth"
        ),
    )
    parser.add_argument("--max-episodes", type=int, default=10000)
    parser.add_argument(
        "--early-stop-consecutive-maze-solves",
        type=int,
        default=0,
        metavar="N",
        help=(
            "Waypoint mode only: after N consecutive episodes that fully solve the maze "
            "(all WPs in order, same as 🏆 MAZE SOLVED), save actor checkpoint and exit. "
            "0 = disabled (run until --max-episodes). Saves wall-clock once policy is stable."
        ),
    )
    parser.add_argument(
        "--best-ever-scope",
        type=str,
        default="",
        metavar="TAG",
        help=(
            "Separate best_ever file per world tag (e.g. eval_maze → "
            "sac_actor_maze_best_ever_eval_maze.pth). Empty = infer from $PFE_WORLD; "
            "'global' = sac_actor_maze_best_ever.pth (lab peak, not mixed with hedge maze)."
        ),
    )
    parser.add_argument(
        "--best-ever-reset",
        action="store_true",
        help=(
            "At session start, remove this scope's best_ever .pth and sidecars (.wp, .floor, "
            ".steps, .solved) so eval-world fine-tuning is not blocked by lab reward floors."
        ),
    )
    parser.add_argument(
        "--lr",
        type=float,
        default=3e-4,
        metavar="LR",
        help="Actor/critic learning rate. Use 1e-4 for Phase 2+ fine-tuning to prevent catastrophic forgetting.",
    )
    parser.add_argument("--device", type=str, default="cpu", choices=("cpu", "cuda"))
    parser.add_argument(
        "--randomize-goal",
        action="store_true",
        help="Sample a new (x,y) goal every episode so the policy cannot memorize one coordinate pair.",
    )
    parser.add_argument(
        "--goal-mode",
        type=str,
        default="box",
        choices=("box", "safe_box"),
        help="box=uniform; safe_box=rejects goals if LiDAR sector along bearing is shorter than path.",
    )
    parser.add_argument(
        "--goal-x-range",
        type=float,
        nargs=2,
        default=[-5.0, -2.0],
        metavar=("LO", "HI"),
        help="Goal sampling range in world X (meters).",
    )
    parser.add_argument(
        "--goal-y-range",
        type=float,
        nargs=2,
        default=[-5.0, -2.0],
        metavar=("LO", "HI"),
        help="Goal sampling range in world Y (meters).",
    )
    parser.add_argument(
        "--goal-sample-attempts",
        type=int,
        default=48,
        help="Max tries per episode for safe_box LiDAR-clear goal sampling.",
    )
    parser.add_argument(
        "--goal-safe-margin",
        type=float,
        default=1.15,
        help="Require min LiDAR sector range >= margin * straight-line distance to goal.",
    )
    parser.add_argument(
        "--goal-sector-halfwidth-bins",
        type=int,
        default=2,
        help="Half-width in LiDAR bins around bearing when checking clearance.",
    )
    parser.add_argument(
        "--train-seed",
        type=int,
        default=None,
        help="RNG seed for goal sampling (reproducible runs for papers / AWS replay).",
    )
    parser.add_argument(
        "--grad-steps",
        type=int,
        default=1,
        help="Optimizer steps per env step after replay warmup (raise when sim is slow vs GPU).",
    )
    parser.add_argument(
        "--replay-warmup-steps",
        type=int,
        default=5000,
        metavar="N",
        help=(
            "Do not run optimizer updates until the replay buffer has at least N transitions. "
            "Waypoint Phase 2: use 20000–40000 so the buffer is seeded mostly from the loaded "
            "policy before off-policy updates — prevents early '0/WP' episodes from erasing WP1."
        ),
    )
    parser.add_argument(
        "--waypoint-goal-radius",
        type=float,
        default=0.0,
        metavar="M",
        help=(
            "If >0 and --waypoint-mode: treat a waypoint as reached when distance < M metres "
            "(default 0 uses module GOAL_RADIUS). Try 0.65–0.72 if odometry jitters cause "
            "near-misses at 0.5 m."
        ),
    )
    parser.add_argument(
        "--world-regen-script",
        type=str,
        default="",
        help="Path to randomize_world.py (or similar); run every --world-regen-interval episodes.",
    )
    parser.add_argument(
        "--world-regen-interval",
        type=int,
        default=0,
        help="If >0 and script set, run world regen every N episodes (restart Gazebo to load new .world).",
    )
    parser.add_argument(
        "--adaptive-steps",
        action="store_true",
        help=(
            "Grow the per-episode step budget as the agent learns to reach waypoints. "
            "Starts at --adaptive-base-steps and adds --adaptive-step-inc for every "
            "waypoint reached in the best episode so far, capped at --max-episode-steps. "
            "Keeps early stuck episodes short and opens up room for WP2/WP3 later."
        ),
    )
    parser.add_argument(
        "--adaptive-base-steps",
        type=int,
        default=600,
        metavar="N",
        help="Starting step budget when --adaptive-steps is active (default 600 = 30 s at 0.05 s/step).",
    )
    parser.add_argument(
        "--adaptive-step-inc",
        type=int,
        default=300,
        metavar="N",
        help="Extra steps granted per waypoint reached in the best episode so far (default 300 = 15 s).",
    )
    parser.add_argument(
        "--manifest-tag",
        type=str,
        default="",
        help="Optional tag for run_manifest JSON filename (default: unix time).",
    )
    parser.add_argument(
        "--no-sim-reset",
        action="store_true",
        help="Do not call /reset_simulation between episodes (unblocks training if reset service hangs; robot is not teleported).",
    )
    parser.add_argument(
        "--reset-service",
        type=str,
        default="/reset_simulation",
        help="std_srvs/srv/Empty service used to reset the sim between episodes (gazebo_ros default).",
    )
    parser.add_argument(
        "--reset-service-wait-sec",
        type=float,
        default=45.0,
        help="Max seconds to wait for the reset service to appear at each episode start.",
    )
    parser.add_argument(
        "--reset-reply-wait-sec",
        type=float,
        default=5.0,
        help="Max seconds to wait for reset Empty *response* after calling it (Gazebo often hangs here).",
    )
    parser.add_argument(
        "--reset-fire-and-forget",
        action="store_true",
        help="Never wait for reset RPC reply; if the reset service never appears, stop waiting after "
        "--reset-service-wait-sec and continue without sim reset (until the service shows up again).",
    )
    inject_preset_defaults(parser)
    parser.add_argument(
        "--training-contract",
        type=str,
        default="",
        help=(
            "Frozen MDP YAML/JSON path. Default: $TRAINING_CONTRACT or bundled "
            "training_contract.yaml next to main_agent.py."
        ),
    )
    p, _ = parser.parse_known_args()

    # ── CPU-performance knobs (must run before any tensor ops) ────────────────
    # i5-1135G7: 4 cores / 8 threads — use all of them unless the user overrides
    # via OMP_NUM_THREADS (already set by pfe_training_cpu_math_env in the menu).
    _n_cpu = int(os.environ.get("OMP_NUM_THREADS", "") or os.cpu_count() or 4)
    torch.set_num_threads(_n_cpu)
    # interop threads: independent ops (e.g. two simultaneous tensor computations).
    # 2 is enough; more can fight intra-op threads for the same cores.
    torch.set_num_interop_threads(max(1, min(2, _n_cpu // 4)))
    # oneDNN (MKL-DNN) is already available; this makes it verbose=0 (no spam).
    torch.backends.mkldnn.enabled = True

    from training_contract import (
        apply_contract_to_main_agent,
        contract_manifest_extras,
        default_contract_path,
        file_sha256,
        load_contract,
        resolve_world_path,
        snapshot_network_source_fingerprint,
    )

    _tcp = (p.training_contract or os.environ.get("TRAINING_CONTRACT", "").strip())
    contract_path = os.path.abspath(
        os.path.expanduser(_tcp) if _tcp else default_contract_path()
    )
    _contract = load_contract(contract_path)
    apply_contract_to_main_agent(sys.modules[__name__], _contract, contract_path=contract_path)

    rclpy.init()
    agent = None
    try:
        agent = SafeNavAgent(
            p.algo,
            p.force_restart,
            device_str=p.device,
            randomize_goal=p.randomize_goal,
            goal_mode=p.goal_mode,
            goal_x_range=(p.goal_x_range[0], p.goal_x_range[1]),
            goal_y_range=(p.goal_y_range[0], p.goal_y_range[1]),
            train_seed=p.train_seed,
            goal_sample_attempts=p.goal_sample_attempts,
            goal_safe_margin=p.goal_safe_margin,
            goal_sector_halfwidth_bins=p.goal_sector_halfwidth_bins,
            no_sim_reset=p.no_sim_reset,
            reset_service=p.reset_service,
            reset_service_wait_sec=p.reset_service_wait_sec,
            reset_reply_wait_sec=p.reset_reply_wait_sec,
            reset_fire_and_forget=p.reset_fire_and_forget,
            env_step_sleep_sec=p.env_step_sleep_sec,
            use_shield=p.use_shield,
            waypoint_mode=p.waypoint_mode,
            lr=p.lr,
            waypoint_goal_radius_m=float(getattr(p, "waypoint_goal_radius", 0.0) or 0.0),
        )

        # ------------------------------------------------------------------ #
        # Warm-start transfer learning: load pre-trained actor weights AFTER  #
        # normal checkpoint loading so the --load-pretrained file takes        #
        # precedence.  Observation space MUST remain 38-dim (36 LiDAR + 2     #
        # goal) and action space 2-dim; mismatches abort the load cleanly.    #
        # ------------------------------------------------------------------ #
        if getattr(p, "load_pretrained", "").strip():
            pretrain_path = os.path.abspath(os.path.expanduser(p.load_pretrained.strip()))
            if os.path.isfile(pretrain_path):
                pt_ckpt = torch.load(pretrain_path, map_location=agent.device)
                pt_in = actor_checkpoint_in_features(pt_ckpt)
                if pt_in is not None and pt_in != agent.state_dim:
                    agent.get_logger().error(
                        f"Warm-start ABORTED: checkpoint at {pretrain_path!r} was trained "
                        f"with state_dim={pt_in} ({pt_in - 2} LiDAR bins + 2 goal dims), "
                        f"but the current model expects state_dim={agent.state_dim} "
                        f"({agent.lidar_bins} bins + 2). "
                        "Fix: retrain with matching LIDAR_BINS, or use --force-restart to "
                        "start fresh (no pre-trained weights)."
                    )
                else:
                    agent.actor.load_state_dict(pt_ckpt)
                    agent.get_logger().info(
                        f"Warm-start OK: actor weights loaded from {pretrain_path!r} "
                        f"(state_dim={agent.state_dim}, action_dim={agent.action_dim})."
                    )
            else:
                agent.get_logger().warning(
                    f"--load-pretrained: file not found: {pretrain_path!r} — "
                    "training will proceed with random / previously saved weights."
                )

        # All checkpoint loading is done — safe to compile now.
        agent.apply_torch_compile()

        tag = p.manifest_tag.strip() or str(int(time.time()))
        manifest_path = os.path.join(agent.log_dir, f"run_manifest_{tag}_{p.algo}.json")
        worlds_cfg = _contract.get("worlds", {})
        wb = worlds_cfg.get("training_default_basename", "current_random_lab.world")
        w_abs = resolve_world_path(agent.workspace_dir, wb)
        w_hashes = (
            {os.path.basename(w_abs): file_sha256(w_abs)} if os.path.isfile(w_abs) else {}
        )
        contract_bundle = {
            **contract_manifest_extras(contract_path, _contract),
            "network_source_fingerprint": snapshot_network_source_fingerprint(),
            "world_files_sha256": w_hashes,
            "checkpoint_format": {
                "saved_as": "torch.save(actor.state_dict(), path)",
                "payload": "Actor network state_dict ONLY (no critic, optimizer, alpha)",
                "inference_load": "actor.load_state_dict(torch.load(path)); actor.eval()",
                "full_resume_note": "Replay buffer not checkpointed — cold replay on resume",
            },
        }
        write_run_manifest(
            manifest_path,
            p,
            agent.workspace_dir,
            p.algo,
            contract_bundle=contract_bundle,
        )
        agent.get_logger().info(f"Run manifest written: {manifest_path}")
        est_ep_wall = p.max_episode_steps * max(0.0, p.env_step_sleep_sec)
        agent.get_logger().info(
            f"Env pacing: env_step_sleep_sec={p.env_step_sleep_sec:g} s → up to ~{est_ep_wall:.0f} s wall time "
            f"per episode if it runs all {p.max_episode_steps} steps (plus short resets)."
        )
        _rwu = max(1000, int(getattr(p, "replay_warmup_steps", 5000) or 5000))
        agent.get_logger().info(
            f"Replay warmup: optimizer disabled until buffer size >{_rwu} "
            f"(then {max(1, p.grad_steps)} grad-step(s) per env step)."
        )

        best_reward = -9999
        ep_best_wp = 0  # best waypoint index reached in any episode so far (adaptive budget)
        consec_maze_solves = 0
        # best_ever path: mode-specific so adapt runs never clobber the maze peak policy.
        _mode_tag = "maze" if agent._waypoint_mode else "adapt"
        _be_scope = _normalize_best_ever_scope(
            getattr(p, "best_ever_scope", "") or _infer_best_ever_scope_from_env()
        )
        best_ever_path = _resolve_best_ever_path(
            agent.model_dir, agent.algo_choice, _mode_tag, _be_scope
        )
        if getattr(p, "best_ever_reset", False):
            for _rm in (best_ever_path, best_ever_path + ".tmp"):
                try:
                    os.remove(_rm)
                except OSError:
                    pass
            _clear_best_ever_sidecars(best_ever_path)
            agent.get_logger().info(
                f"[best_ever] scope={_be_scope or 'global'} reset — fresh promotion for this world."
            )
        if _be_scope:
            agent.get_logger().info(
                f"[best_ever] scoped checkpoint: {os.path.basename(best_ever_path)}"
            )
        # Autonomous checkpoint backup at session start (no launcher script required).
        _backup_dir = os.path.join(agent.model_dir, "backups")
        _be_bk = _safe_backup_pth(best_ever_path, _backup_dir)
        if _be_bk:
            agent.get_logger().info(
                f"[checkpoint-backup] session start: best_ever → backups/{os.path.basename(_be_bk)}"
            )
        _cur_bk = _safe_backup_pth(agent.actor_path, _backup_dir)
        if _cur_bk:
            agent.get_logger().info(
                f"[checkpoint-backup] session start: actor → backups/{os.path.basename(_cur_bk)}"
            )
        # Waypoint-tier + solve/steps stored next to best_ever so lab reward floors
        # (dense shaping >> waypoint bonuses) never block hedge-maze full solves.
        best_ever_wp_stored = 0
        best_ever_solved_stored = False
        best_ever_steps_stored = 999_999
        best_ever_wp_stored = _read_best_ever_sidecar(best_ever_path, ".wp", 0)
        best_ever_solved_stored = bool(
            _read_best_ever_sidecar(best_ever_path, ".solved", 0)
        )
        best_ever_steps_stored = _read_best_ever_sidecar(
            best_ever_path, ".steps", 999_999
        )

        # Seed best_reward from best_ever sidecar so promotion cannot regress the floor
        if os.path.exists(best_ever_path):
            try:
                _be = torch.load(best_ever_path, map_location=agent.device)
                # Reward is not stored in the checkpoint; load floor from sidecar text
                # so a new run must beat the stored floor to overwrite best_ever.
                _floor_path = best_ever_path + ".floor"
                if os.path.exists(_floor_path):
                    with open(_floor_path) as _f:
                        best_reward = float(_f.read().strip())
                    agent.get_logger().info(
                        f"best_ever checkpoint exists; reward floor = {best_reward:.1f}, "
                        f"waypoint tier = {best_ever_wp_stored}, "
                        f"solved={best_ever_solved_stored}, steps={best_ever_steps_stored}. "
                        "Promotion: more WPs → full solve → fewer steps → reward tie-break."
                    )
                del _be
            except Exception:
                pass

        if agent._no_sim_reset:
            agent.get_logger().info("Training loop starting (--no-sim-reset; no /reset_simulation).")
        elif agent._reset_fire_and_forget:
            agent.get_logger().info(
                f"Training loop starting (waits up to {agent._reset_service_wait_sec:.0f}s for "
                f"{agent._reset_service} per episode; without Gazebo, continues without reset after timeout)."
            )
        else:
            agent.get_logger().info(
                f"Training loop starting (episode 1 waits for {agent._reset_service}; "
                "start Gazebo first or use --no-sim-reset / --reset-fire-and-forget)."
            )
        for episode in range(1, p.max_episodes + 1):
            # ── Adaptive step budget ──────────────────────────────────────────
            # Starts short (fewer wasted steps on stuck episodes), then expands
            # as the agent demonstrates it can reach later waypoints.
            if p.adaptive_steps and agent._waypoint_mode:
                effective_steps = min(
                    p.max_episode_steps,
                    p.adaptive_base_steps + p.adaptive_step_inc * ep_best_wp,
                )
            else:
                effective_steps = p.max_episode_steps

            state = agent.reset_env()
            ep_reward = 0.0
            ep_exec_times = []
            ep_steps = 0

            for _step in range(effective_steps):
                st = torch.FloatTensor(state).unsqueeze(0).to(agent.device)
                start_time = time.perf_counter()

                with torch.no_grad():
                    if agent.algo_choice == 'sac':
                        raw_action, _, _ = agent.actor.sample(st)
                        raw_action = raw_action.detach().cpu().numpy()[0]
                    elif agent.algo_choice == 'custom':
                        raw_action = agent.actor.predict(st).detach().cpu().numpy()[0]
                    else:  # td3
                        raw_action = agent.actor(st).detach().cpu().numpy()[0]
                        raw_action = np.clip(
                            raw_action + np.random.normal(0, 0.1, 2), -1.0, 1.0
                        )

                ep_exec_times.append((time.perf_counter() - start_time) * 1000.0)

                # ── Affine rescaling: actor outputs tanh ∈ [-1, 1]. ─────────
                # lin: (-1 → 0 m/s, 0 → 0.2 m/s, +1 → 0.4 m/s).
                # ang: [-1, 1] rad/s is the full usable range — keep as-is.
                # Store the applied action (not raw tanh) so Q-values match executed motion.
                # Without this, ~50 % of SAC samples have lin<0 → applied=0,
                # but the buffer stores the negative value → corrupted dataset.
                applied_action = np.array([
                    float(np.clip((raw_action[0] + 1.0) * 0.2, 0.0, 0.4)),
                    float(np.clip(raw_action[1], -1.0, 1.0)),
                ], dtype=np.float32)

                next_state, reward, done = agent.step_environment(applied_action)
                # Use the actually executed action (post-shield) for the buffer so the
                # critic sees the real (s, a, r, s') MDP transition, not the pre-shield intent.
                executed_action = getattr(agent, "_last_executed_action", applied_action)

                agent.memory.add(state, executed_action, reward, next_state, done)

                _warm = max(1000, int(getattr(p, "replay_warmup_steps", 5000) or 5000))
                if agent.memory.size > _warm:
                    for _ in range(max(1, p.grad_steps)):
                        agent.trainer.train(batch_size=256)

                state = next_state
                ep_reward += reward
                agent.total_steps += 1
                ep_steps += 1
                rclpy.spin_once(agent, timeout_sec=0)
                if done:
                    break

            avg_time = sum(ep_exec_times) / len(ep_exec_times) if ep_exec_times else 0.0

            agent.writer.add_scalar("Reward", ep_reward, episode)
            agent.writer.add_scalar("ExecutionTime_ms", avg_time, episode)
            agent.writer.add_scalar("EpisodeSteps", ep_steps, episode)
            agent.writer.add_scalar("adapt/goal_x", agent.target_x, episode)
            agent.writer.add_scalar("adapt/goal_y", agent.target_y, episode)
            if agent._waypoint_mode:
                n_wp = len(agent._maze_waypoints)
                # current_waypoint_idx stays at n_wp-1 on a full solve (the final WP
                # index is never incremented past the list end).  Add the solved flag
                # so TensorBoard shows 100% on a complete maze run, not (n-1)/n.
                wp_cleared = agent.current_waypoint_idx + int(agent._maze_solved_this_episode)
                agent.writer.add_scalar("waypoint/idx_at_end", wp_cleared, episode)
                agent.writer.add_scalar(
                    "waypoint/progress_pct",
                    wp_cleared / n_wp * 100.0,
                    episode,
                )
                agent.writer.add_scalar("waypoint/best_wp_so_far", ep_best_wp, episode)
            if p.adaptive_steps:
                agent.writer.add_scalar("curriculum/effective_steps", effective_steps, episode)
            agent.log_csv(episode, ep_reward, agent.total_steps, ep_steps, avg_time)
            agent.log_adapt_csv(episode, ep_reward, agent.total_steps, ep_steps, avg_time)

            wp_suffix = ""
            if agent._waypoint_mode:
                n_wp = len(agent._maze_waypoints)
                wp_cleared = agent.current_waypoint_idx + int(agent._maze_solved_this_episode)
                ep_best_wp = max(ep_best_wp, wp_cleared)
                wp_suffix = f" | WP: {wp_cleared}/{n_wp}"
            step_suffix = (
                f" | budget: {effective_steps}" if p.adaptive_steps else ""
            )
            agent.get_logger().info(
                f"Ep: {episode} | Rew: {ep_reward:.1f} | ep_steps: {ep_steps} "
                f"| total_steps: {agent.total_steps}{wp_suffix}{step_suffix}"
            )

            # Dense distance + proximity shaping can exceed waypoint bonuses without
            # ever entering GOAL_RADIUS — warn so logs are not mis-read as "good maze".
            if agent._waypoint_mode:
                _wpc = (
                    agent.current_waypoint_idx + int(agent._maze_solved_this_episode)
                )
                if _wpc == 0 and ep_reward > 400.0:
                    agent.get_logger().warning(
                        f"Episode return {ep_reward:.1f} with 0/{len(agent._maze_waypoints)} "
                        "waypoints — mostly dense shaping / proximity, not maze progress. "
                        "This will NOT update best_ever (waypoint tier must advance)."
                    )

            # Save actor_path when: (a) better episode, OR (b) periodic heartbeat but
            # ONLY if the episode isn't catastrophically bad (floor = -100).  This
            # prevents a crash-recovery resume from starting with terrible weights.
            _periodic_ok = episode % 50 == 0 and ep_reward > -100
            if ep_reward > best_reward or _periodic_ok:
                best_reward = max(ep_reward, best_reward)
                if _periodic_ok and ep_reward <= best_reward and os.path.isfile(agent.actor_path):
                    _ab = _safe_backup_pth(agent.actor_path, _backup_dir)
                    if _ab:
                        agent.get_logger().info(
                            f"[checkpoint-backup] periodic (ep {episode}) actor → "
                            f"backups/{os.path.basename(_ab)}"
                        )
                tmp_path = agent.actor_path + ".tmp"
                torch.save(_actor_state_dict(agent.actor), tmp_path)
                os.replace(tmp_path, agent.actor_path)

            # best_ever: adapt mode = reward floor only. Waypoint mode = lexicographic
            # (waypoints_cleared, return) vs (.wp, .floor) so a high-shaping / 0-WP spike
            # cannot block later maze-quality checkpoints (session best_reward can hit
            # thousands without any waypoint).
            if ep_reward > -50:
                _floor_p = best_ever_path + ".floor"
                _old_floor: float | None = None
                if os.path.isfile(_floor_p):
                    try:
                        with open(_floor_p, encoding="utf-8") as _fp:
                            _old_floor = float(_fp.read().strip())
                    except (OSError, ValueError):
                        _old_floor = None
                if agent._waypoint_mode:
                    n_wp_be = len(agent._maze_waypoints)
                    wp_cleared_be = agent.current_waypoint_idx + int(
                        agent._maze_solved_this_episode
                    )
                    _wp_progress_ok = wp_cleared_be >= 1 or agent._maze_solved_this_episode
                    _stored_r = (
                        float(_old_floor)
                        if _old_floor is not None
                        else -9999.0
                    )
                    _lex_beats_disk = _waypoint_run_beats_disk(
                        wp_cleared_be,
                        bool(agent._maze_solved_this_episode),
                        ep_steps,
                        ep_reward,
                        stored_wp=best_ever_wp_stored,
                        stored_solved=best_ever_solved_stored,
                        stored_steps=best_ever_steps_stored,
                        n_wp=n_wp_be,
                        stored_reward=_stored_r,
                    )
                    _strict_new_record = _wp_progress_ok and _lex_beats_disk
                else:
                    _strict_new_record = (
                        ep_reward >= best_reward  # session best (or tie on first ep)
                        and (_old_floor is None or ep_reward > _old_floor + 1e-6)
                    )
                if _strict_new_record:
                    if os.path.isfile(best_ever_path):
                        _bb = _safe_backup_pth(best_ever_path, _backup_dir)
                        if _bb:
                            agent.get_logger().info(
                                f"[checkpoint-backup] new best_ever (ep {episode}, "
                                f"R={ep_reward:.1f}>{_old_floor if _old_floor is not None else '—'}) → "
                                f"backups/{os.path.basename(_bb)}"
                            )
                    tmp_be = best_ever_path + ".tmp"
                    torch.save(_actor_state_dict(agent.actor), tmp_be)
                    os.replace(tmp_be, best_ever_path)
                    with open(best_ever_path + ".floor", "w", encoding="utf-8") as _f:
                        _f.write(str(ep_reward))
                    if agent._waypoint_mode:
                        wp_cleared_be = agent.current_waypoint_idx + int(
                            agent._maze_solved_this_episode
                        )
                        best_ever_wp_stored = wp_cleared_be
                        best_ever_solved_stored = bool(agent._maze_solved_this_episode)
                        best_ever_steps_stored = ep_steps
                        _write_best_ever_sidecar(best_ever_path, ".wp", best_ever_wp_stored)
                        _write_best_ever_sidecar(
                            best_ever_path, ".solved", int(best_ever_solved_stored)
                        )
                        _write_best_ever_sidecar(
                            best_ever_path, ".steps", best_ever_steps_stored
                        )
                        agent.get_logger().info(
                            f"[best_ever] saved wp={best_ever_wp_stored}/{n_wp_be} "
                            f"solved={best_ever_solved_stored} steps={ep_steps} "
                            f"reward={ep_reward:.1f}"
                        )

            n_early = int(getattr(p, "early_stop_consecutive_maze_solves", 0) or 0)
            if n_early > 0 and agent._waypoint_mode:
                if agent._maze_solved_this_episode:
                    consec_maze_solves += 1
                    agent.writer.add_scalar("early_stop/consecutive_maze_solves", consec_maze_solves, episode)
                else:
                    consec_maze_solves = 0
                if consec_maze_solves >= n_early:
                    agent.get_logger().info(
                        f"[early-stop] {consec_maze_solves} consecutive full maze solves "
                        f"(threshold {n_early}) — saving actor to {agent.actor_path!r} and exiting."
                    )
                    tmp_es = agent.actor_path + ".tmp"
                    torch.save(_actor_state_dict(agent.actor), tmp_es)
                    os.replace(tmp_es, agent.actor_path)
                    break

            if (
                p.world_regen_script
                and p.world_regen_interval > 0
                and episode % p.world_regen_interval == 0
            ):
                script_path = os.path.abspath(os.path.expanduser(p.world_regen_script))
                if os.path.isfile(script_path):
                    cwd = os.path.dirname(script_path) or "."
                    agent.get_logger().info(
                        f"Running world regen script (episode {episode}): {script_path}"
                    )
                    try:
                        subprocess.run(
                            [sys.executable, script_path],
                            cwd=cwd,
                            timeout=180,
                            check=False,
                        )
                    except subprocess.TimeoutExpired:
                        agent.get_logger().error("World regen script timed out.")
                    agent.get_logger().warning(
                        "World file may be updated; restart Gazebo (or your launch) to load the new "
                        "world unless your stack hot-reloads worlds."
                    )
                else:
                    agent.get_logger().warning(
                        f"--world-regen-script not found ({script_path}); skipping."
                    )

    except KeyboardInterrupt:
        pass
    finally:
        if agent is not None:
            try:
                getattr(agent.writer, "flush", lambda: None)()
                agent.writer.close()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__': main()