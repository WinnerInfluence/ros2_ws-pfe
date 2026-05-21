#!/usr/bin/env bash
# Stop everything started by start_web_demo.sh
set -eo pipefail

WS_PATH="${WS_PATH:-$HOME/ros2_ws}"

pkill -f "hot_swap_eval_node.py" 2>/dev/null || true
pkill -f "safe_nav_agent" 2>/dev/null || true
bash "${WS_PATH}/scripts/stop_sim_demo_remote.sh" 2>/dev/null || true

for pat in gzserver gzclient "ros2 launch gazebo_ros" spawn_entity.py move_enemy.py; do
  pkill -9 -f "$pat" 2>/dev/null || true
done

echo "Stopped web demo (Gazebo, hot_swap, API, tunnel)."
