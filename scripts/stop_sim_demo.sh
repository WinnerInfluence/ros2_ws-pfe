#!/usr/bin/env bash
# stop_sim_demo.sh — kill tmux sim_demo + Gazebo / bridge / upload_server
set -euo pipefail

SESSION="${SIM_DEMO_TMUX_SESSION:-sim_demo}"

tmux kill-session -t "$SESSION" 2>/dev/null || true

for pat in upload_server.py ws_bridge.py ws_bridge_node gzserver gzclient \
  "ros2 launch gazebo_ros" evaluate_agent.py hot_swap_eval; do
  pkill -9 -f "$pat" 2>/dev/null || true
done

echo "SIM demo stopped (session ${SESSION})."
