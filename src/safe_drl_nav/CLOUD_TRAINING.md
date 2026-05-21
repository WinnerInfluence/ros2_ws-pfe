# Cloud / headless training — frozen contract & parity checklist

This package trains with **custom PyTorch SAC/TD3** (not Stable-Baselines3). The **MDP interface** is frozen in `safe_drl_nav/training_contract.yaml`.

## 1) Exact inventory — obs, action, reward, `done`

| Concern | Location | Function / notes |
|--------|----------|------------------|
| Observation vector | `safe_drl_nav/main_agent.py` | `SafeNavAgent.get_state()` — LiDAR min-pool + `[Δx, Δy]` |
| LiDAR pooling | `main_agent.py` | `pool_laser_to_bins()`, `laser_callback()` |
| Action scaling (train) | `main_agent.py` | `main()` loop: `(raw_tanh[0]+1)*0.2` clipped `[0,0.4]`, `raw[1]` clipped `[-1,1]` → `step_environment()` |
| Command execution | `main_agent.py` | `step_environment()` — shield override if `--use-shield` |
| Reward + termination | `main_agent.py` | `step_environment()` — dense dist, penalties, waypoint chain, collision |
| Episode reset | `main_agent.py` | `reset_env()` — `/reset_simulation`, waypoint idx, `prev_dist` |
| Eval parity | `safe_drl_nav/evaluate_agent.py` | `EvalNode._get_state()`, `EvalNode.step()` — uses `ma.*` constants after `--training-contract` |
| Hot swap | `safe_drl_nav/hot_swap_eval_node.py` | Same `EvalNode.step`; loads contract in `main()` |

## 2) Single source of truth

- **Canonical:** `safe_drl_nav/training_contract.yaml`
- **Loader:** `safe_drl_nav/training_contract.py` → `load_contract()`, `apply_contract_to_main_agent()`
- **CLI:** `--training-contract /path/to/copy.yaml`
- **Env:** `TRAINING_CONTRACT=/path/to/copy.yaml`
- **Training** applies contract **before** `SafeNavAgent` is constructed (`main_agent.main()`).
- **Remaining duplication:** preset knobs (`PRESETS` in `main_agent.py`), launch scripts (`train_menu.sh`, `start_pfe.sh`), TurtleBot spawn pose — not yet YAML-driven. Hyperparameters (LR, gamma) remain in `trainer_*.py` unless added to the contract YAML.

## 3) Train manifest

Each training run writes `pfe_logs/run_manifest_<tag>_<algo>.json` including:

- `training_contract_bundle`: path, SHA-256, **full YAML snapshot**, torch/numpy versions, `network_source_fingerprint`, training world SHA-256 (if file exists), checkpoint format notes.

Copy **manifest + identical `training_contract.yaml` + checkpoint** to the cloud box together.

## 4) Verify before training

```bash
cd /path/to/ros2_ws/src/safe_drl_nav/safe_drl_nav
pip install pyyaml torch numpy  # if needed
python3 verify_cloud_readiness.py \
  --contract training_contract.yaml \
  --algo sac \
  --checkpoint trained_models/sac_actor_maze.pth \
  --manifest pfe_logs/run_manifest_<your_tag>_sac.json   # optional
```

After `colcon build`:

```bash
ros2 run safe_drl_nav verify_cloud_ready -- --contract $(ros2 pkg prefix safe_drl_nav)/...
```

(Adjust path — installed YAML lives under `site-packages/safe_drl_nav/training_contract.yaml`.)

## 5) Headless Ubuntu launch (classic Gazebo + ROS 2 Humble)

```bash
export ROS_DISTRO=humble
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Optional virtual display for gzclient-less setups
export DISPLAY=:1   # if Xvfb :1 running

# Frozen MDP (must match training manifest)
export TRAINING_CONTRACT=/abs/path/to/training_contract.yaml

# World — match training (generate lab first if using randomize_world.py)
export PFE_WORLD=/abs/path/to/safe_drl_nav/safe_drl_nav/sim_assets/worlds/current_random_lab.world

# Headless server-only (no GUI) — typical pattern:
gzserver --verbose "$PFE_WORLD" &
# OR ros2 launch gazebo_ros gazebo.launch.py world:=$PFE_WORLD

# Train (CPU-only is fine — set device cpu)
python3 /abs/path/to/main_agent.py \
  --algo sac \
  --preset pfe_sac_waypoint \
  --device cpu \
  --training-contract "$TRAINING_CONTRACT" \
  --reset-fire-and-forget \
  --reset-service-wait-sec 12 \
  --env-step-sleep-sec 0.05
```

**RTF:** leave default real-time factor `1` unless you intentionally speed physics (changes contact behaviour → **not** interface-neutral).

**CPU-only:** `--device cpu`; PyTorch CPU build sufficient; no CUDA requirement.

## 6) Checkpoint format

- **Saved:** `torch.save(agent.actor.state_dict(), '.pth')` — **actor weights only**
- **Inference / hot-swap:** `actor.load_state_dict(torch.load(path, map_location=device)); actor.eval()`
- **Resume training:** loads actor only — **replay buffer is empty**; optimiser state not saved → “warm restart”, not bit-identical continuation.

## 7) Cloud compatibility checklist

- [ ] Same **Git commit** as manifest (or document drift if `git_dirty: true`)
- [ ] Identical **`training_contract.yaml`** byte-for-byte or matching SHA-256
- [ ] `verify_cloud_readiness.py` passes with your `.pth`
- [ ] ROS **Humble** + same **`gazebo_ros`** stack (classic `gzserver`)
- [ ] Same **`/scan`**, **`/odom`**, **`/cmd_vel`**, **`/reset_simulation`** names (or update contract + code together)
- [ ] Same **`TURTLEBOT3_MODEL`** / spawn pose if they affect initial state
- [ ] Same **`env_step_sleep_sec`** pacing policy (or accept timing-only drift)
- [ ] Torch **same major** recommended (minor mismatch usually OK for inference)

## 8) Known non-determinism

- Floating-point order across CPU models / thread counts
- Gazebo contact solver + RTF jitter
- Concurrent ROS callbacks (`spin_once` pacing vs real-time)
- LiDAR noise if enabled in sim (currently none explicit)
- `torch` nondeterministic algorithms on GPU (you use CPU often — still not bitwise deterministic)
- Episode length variance from physics timing

**Mitigations:** fix seeds where supported (`--train-seed` for goal sampling only), single-thread executor where practical, fixed `max_step_size`, avoid RTF ≠ 1 during comparative runs.

## Stack facts (from repo)

| Item | Value |
|------|--------|
| ROS | 2 **Humble** (`package.xml`) |
| Gazebo | Classic via **`gazebo_ros`** (`gzserver` / `gzclient`) — Fortress/Harmonic naming is distro-specific |
| Algorithm | **Custom** Torch SAC/TD3 (`networks_*.py`, `trainer_*.py`) |
