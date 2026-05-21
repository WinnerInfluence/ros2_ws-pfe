# Safe DRL Navigation (ROS 2)

## *Fast Adaptation Methods in Deep Reinforcement Learning for Autonomous Robotics*

**Core problem:** achieve **safe, sample-efficient navigation** for a differential-drive robot when the training lab no longer matches deployment maps—via shielded off-policy RL (**SAC** / **TD3** baseline), a frozen training contract, waypoint curricula, and short **cross-world fine-tuning** (lab → Egypt, Rabat, maze).

[![Safe DRL Navigation - Explainer Video](https://img.youtube.com/vi/nBZXk_lRonE/maxresdefault.jpg)](https://youtu.be/nBZXk_lRonE)

*Video Demonstration: Shielded SAC Waypoint Curriculum and Cross-World Transfer (ROS 2 / Gazebo).*

<details>
<summary><strong>Documentation</strong> — build, run, layout, checkpoints</summary>

ROS 2 Humble workspace for **SAC/TD3** waypoint navigation in Gazebo with a LiDAR safety shield (`safe_drl_nav` package).

## Requirements

- Ubuntu 22.04 (or compatible) with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Gazebo Classic (via `gazebo_ros`)
- Python 3.10+ with PyTorch

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select safe_drl_nav
source install/setup.bash
```

## Run

Interactive training and evaluation menu:

```bash
cd src/safe_drl_nav/safe_drl_nav
bash train_menu.sh
```

Roadmap launcher (phases, evaluation presets):

```bash
bash start_pfe.sh
```

Frozen cross-world evaluation (thesis tables, n=33):

```bash
bash ~/ros2_ws/scripts/run_thesis_crossworld_sac.sh
```

Optional public web demo (requires local `sim_demo.env` from `sim_demo.env.example`):

```bash
bash ~/ros2_ws/scripts/start_web_demo.sh sac
bash ~/ros2_ws/scripts/stop_web_demo.sh
```

## Repository layout

| Path | Description |
|------|-------------|
| `src/safe_drl_nav/` | ROS 2 package: agents, Gazebo worlds, checkpoints |
| `scripts/` | Evaluation harnesses, recording, optional web demo |
| `docs/` | Entrypoints, disk safety, recording notes |
| `website/ros/` | Static dashboard assets for the web demo |

Launcher reference: [docs/ENTRYPOINTS.md](docs/ENTRYPOINTS.md).

## Checkpoints and logs

Primary actor weights live under `src/safe_drl_nav/safe_drl_nav/trained_models/`.  
Runtime metrics and eval JSON are written to `pfe_logs/` (gitignored).

Before pushing changes:

```bash
bash scripts/check_repo_integrity.sh
```

Do not commit `sim_demo.env`, `*.pem`, or API keys.

## License

See package metadata in `src/safe_drl_nav/package.xml`.

</details>
