# Launcher entrypoints

Use **one** primary path; others are aliases for old habits.

| Script | Use when |
|--------|----------|
| `src/safe_drl_nav/safe_drl_nav/train_menu.sh` | **Default** — Phase 1/2, evaluate (menu steps A–E), TensorBoard, short demo (7) |
| `src/safe_drl_nav/safe_drl_nav/start_pfe.sh` | Full roadmap (adapt, phases 9–15, eval 16) |
| `src/safe_drl_nav/safe_drl_nav/train_waypoint.sh` | Unattended multi-phase roadmap in one terminal |
| `src/safe_drl_nav/safe_drl_nav/run_auto_train.sh` | Overnight SAC waypoint (headless, no prompts) |
| `scripts/run_thesis_crossworld_sac.sh` | Thesis Table: rabat/egypt/maze × zeroshot/phase3 |
| `scripts/run_presentation_demo.sh` | Short Gazebo recording (hot_swap + sample) |
| `train_sac_maze.sh` (ws root) | Alias → `run_auto_train.sh` |
| `train_adapt_local.sh` | Alias → `start_pfe.sh 1` |
| `run_pfe_train.sh` | Alias → `train_menu.sh` |

All Gazebo launchers source `pfe_gazebo_env.sh` for `IGN_IP`, ROS domain, and optional headless GUI.
