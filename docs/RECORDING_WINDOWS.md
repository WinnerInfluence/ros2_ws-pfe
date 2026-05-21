# Recording demos on Windows (quick guide)

This repo runs the simulations on Linux (ROS 2 + Gazebo). For a thesis presentation video on a **Windows laptop**, the simplest workflow is:

- Run the demo on the Linux machine (or inside WSL2 with GUI support if you know it well)
- Record the screen on Windows **from the display output** (HDMI capture) or by copying the resulting video file

## Recommended: record on Linux desktop (fastest)

### Rabat (Tour Hassan) – original performance world

```bash
bash ~/ros2_ws/scripts/kill_gazebo.sh
PFE_TEXTURED=1 EPISODES=1 MAX_STEPS=2500 ENV_STEP_SLEEP=0.02 \
  bash ~/ros2_ws/scripts/eval_rabat_fast.sh
```

### Rabat (Tour Hassan) – “beautiful textured” world for visuals

```bash
bash ~/ros2_ws/scripts/kill_gazebo.sh
EPISODES=1 MAX_STEPS=2500 ENV_STEP_SLEEP=0.02 \
  bash ~/ros2_ws/scripts/record_rabat_textured.sh
```

### Maze – best checkpoint (most likely to show a solve)

```bash
bash ~/ros2_ws/scripts/kill_gazebo.sh
bash ~/ros2_ws/scripts/open_gazebo.sh maze spawn

cd ~/ros2_ws/src/safe_drl_nav/safe_drl_nav
python3 evaluate_agent.py --algo sac \
  --model trained_models/sac_actor_maze_best_ever_eval_maze.pth \
  --episodes 1 --max-steps 2000 --env-step-sleep-sec 0.02 \
  --sac-deterministic-mean \
  --waypoint-goal-radius 0.68 --tag maze_short_video
```

## Scene selection (keep it short)

- **Keep**: Rabat visuals + Maze solve clip
- **Skip** (per your note): DR-training lab / random-target world – no need to record these

