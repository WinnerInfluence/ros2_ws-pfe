#!/usr/bin/env bash
# One-shot Rabat on laptops: light world + robot + optional SAC eval (skip heavy textures).
#
#   bash ~/ros2_ws/scripts/go_rabat.sh              # Gazebo + robot (record in gzclient)
#   bash ~/ros2_ws/scripts/go_rabat.sh eval         # + 1-episode eval, no GUI
#   bash ~/ros2_ws/scripts/go_rabat.sh record       # + 1 ep eval with GUI for video
set -euo pipefail

MODE="${1:-view}"
export PFE_TEXTURED=0
export PFE_NO_GUI=0

case "$MODE" in
    view)
        bash "$(dirname "$0")/kill_gazebo.sh"
        exec bash "$(dirname "$0")/open_gazebo.sh" rabat_fast spawn
        ;;
    eval)
        bash "$(dirname "$0")/kill_gazebo.sh"
        export PFE_NO_GUI=1
        EPISODES=3 MAX_STEPS=2000 ENV_STEP_SLEEP=0.05 \
            exec bash "$(dirname "$0")/eval_rabat_fast.sh"
        ;;
    record)
        bash "$(dirname "$0")/kill_gazebo.sh"
        EPISODES=1 MAX_STEPS=2500 ENV_STEP_SLEEP=0.05 \
            exec bash "$(dirname "$0")/eval_rabat_fast.sh"
        ;;
    *)
        echo "Usage: $0 [view|eval|record]"
        exit 1
        ;;
esac
