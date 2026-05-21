#!/usr/bin/env bash
# One-shot: Gazebo + optional public web stack + hot_swap (SAC recommended)
# Usage:  ~/ros2_ws/scripts/start_web_demo.sh [sac|td3]
# Stop:   ~/ros2_ws/scripts/stop_web_demo.sh
set -euo pipefail

WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
ALGO="${1:-sac}"
cd "$WS_PATH"
mkdir -p pfe_logs

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

if ! python3 -c "import rclpy" 2>/dev/null; then
  echo "ERROR: source ROS first or: sudo apt install ros-humble-rclpy" >&2
  exit 1
fi

echo "=== [1/3] Gazebo (headless) ==="
_need_gz=1
if ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
  if timeout 2 ros2 topic hz /scan --window 2 2>/dev/null | grep -q "average rate"; then
    echo "  Gazebo already up (/scan OK) — skipping launch."
    _need_gz=0
  else
    echo "  Gazebo up but no /scan — restarting sim…"
    pkill -9 -f gzserver 2>/dev/null || true
    sleep 2
  fi
fi
if [[ "$_need_gz" == "1" ]]; then
  export PFE_WORLD="${PFE_WORLD:-${WS_PATH}/src/safe_drl_nav/safe_drl_nav/sim_assets/worlds/eval_maze.world}"
  export SIM_DEMO_GAZEBO_DETACH=1
  nohup bash "${WS_PATH}/scripts/_sim_demo_gazebo.sh" >>"${WS_PATH}/pfe_logs/gazebo.log" 2>&1 &
  echo "  booting… (log: pfe_logs/gazebo.log)"
  for ((i = 0; i < 90; i++)); do
    if ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
      echo "  Gazebo ready."
      break
    fi
    sleep 2
    if (( i == 89 )); then
      echo "  WARN: /reset_simulation not seen — check pfe_logs/gazebo.log" >&2
    fi
  done
fi

echo "=== [2/3] API + tunnel (optional public proxy) ==="
bash "${WS_PATH}/scripts/start_sim_demo_remote.sh"

echo "=== [3/3] hot_swap_eval (${ALGO}) ==="
pkill -f "upload_telemetry_sidecar.py" 2>/dev/null || true
pkill -f "hot_swap_eval_node.py" 2>/dev/null || true
sleep 1
nohup bash "${WS_PATH}/scripts/start_hot_swap_for_web.sh" "${ALGO}" \
  >>"${WS_PATH}/pfe_logs/hot_swap.log" 2>&1 &

sleep 4
if pgrep -f "hot_swap_eval_node.py" >/dev/null; then
  echo "  hot_swap running (log: pfe_logs/hot_swap.log)"
  if ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
    ros2 service call /reset_simulation std_srvs/srv/Empty "{}" 2>/dev/null || true
    sleep 1
  fi
else
  echo "  WARN: hot_swap failed — tail pfe_logs/hot_swap.log" >&2
  tail -8 "${WS_PATH}/pfe_logs/hot_swap.log" 2>/dev/null || true
fi

echo ""
echo "Web demo stack UP."
echo "  Local:  http://127.0.0.1:5001/health"
echo "  Tip:    use 'sac' (default) for maze_best_ever; 'td3' needs td3_actor_*.pth"
echo "  Public: configure reverse proxy (see SIM_DEMO_README.md)"
echo "  Stop:   ${WS_PATH}/scripts/stop_web_demo.sh"
echo "  Logs:   ${WS_PATH}/pfe_logs/"
