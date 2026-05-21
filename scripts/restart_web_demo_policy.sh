#!/usr/bin/env bash
# Fix web dashboard: eval_maze world, kill telemetry sidecar, restart SAC hot_swap, reset sim.
set -euo pipefail
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
cd "$WS_PATH"

set -a
# shellcheck disable=SC1091
source "${WS_PATH}/sim_demo.env"
set +a

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
[[ -f "${WS_PATH}/install/setup.bash" ]] && source "${WS_PATH}/install/setup.bash"
set -u

AGENT_DIR="${WS_PATH}/src/safe_drl_nav/safe_drl_nav"
export PFE_WORLD="${PFE_WORLD:-$AGENT_DIR/sim_assets/worlds/eval_maze.world}"

echo "=== Gazebo eval_maze (matches web dashboard walls) ==="
if ! pgrep -f "gzserver.*eval_maze" >/dev/null 2>&1; then
  echo "  restarting Gazebo with eval_maze.world…"
  pkill -9 -f gzserver 2>/dev/null || true
  sleep 2
  export SIM_DEMO_GAZEBO_DETACH=1
  nohup bash "${WS_PATH}/scripts/_sim_demo_gazebo.sh" >>"${WS_PATH}/pfe_logs/gazebo.log" 2>&1 &
  for ((i = 0; i < 45; i++)); do
    if ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
      echo "  Gazebo ready (eval_maze)."
      break
    fi
    sleep 2
  done
fi

echo "=== stop sidecar (zeros step on dashboard) ==="
pkill -f upload_telemetry_sidecar.py 2>/dev/null || true

echo "=== restart hot_swap (SAC) ==="
pkill -f hot_swap_eval_node.py 2>/dev/null || true
sleep 1
nohup bash "${WS_PATH}/scripts/start_hot_swap_for_web.sh" sac \
  >>"${WS_PATH}/pfe_logs/hot_swap.log" 2>&1 &
sleep 5
if ! pgrep -f hot_swap_eval_node.py >/dev/null; then
  echo "ERROR: hot_swap failed — tail pfe_logs/hot_swap.log" >&2
  tail -15 "${WS_PATH}/pfe_logs/hot_swap.log" >&2
  exit 1
fi

echo "=== reset Gazebo robot to maze spawn ==="
if ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
  ros2 service call /reset_simulation std_srvs/srv/Empty "{}" 2>/dev/null || true
  sleep 2
  ros2 topic pub --once /policy_control std_msgs/msg/String "{data: reset_episode}" 2>/dev/null || true
else
  echo "WARN: Gazebo not up — start start_web_demo.sh first"
fi

echo "=== restart API (upload_server fix + no sidecar) ==="
bash "${WS_PATH}/scripts/start_sim_demo_remote.sh"

echo ""
echo "OK. Hard-refresh the web dashboard after restart."
echo "Check: curl -s http://127.0.0.1:5001/lidar_live | python3 -m json.tool | head -12"
