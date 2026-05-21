#!/usr/bin/env bash
# Headless gzserver + TurtleBot3 spawn (no DRL brain — eval via upload_server / hot_swap)
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="${WS_PATH}/src/safe_drl_nav/safe_drl_nav"
SIM_ASSETS="${AGENT_DIR}/sim_assets"
_DEFAULT_WORLD="${SIM_ASSETS}/worlds/current_random_lab.world"
PFE_WORLD="${PFE_WORLD:-$_DEFAULT_WORLD}"
TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
TURTLEBOT_SDF="${TURTLEBOT_SDF:-/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf}"

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-minimal}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://localhost:11345}"
unset DISPLAY

for pat in gzserver gzclient "ros2 launch gazebo_ros" evaluate_agent.py main_agent.py; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 2

echo "[gazebo] world=${PFE_WORLD}"
ros2 launch gazebo_ros gazebo.launch.py \
  world:="${PFE_WORLD}" \
  gui:=false \
  verbose:=false &
GZ_PID=$!
sleep "${SIM_DEMO_GZ_BOOT_SLEEP:-18}"
kill -0 "$GZ_PID" 2>/dev/null || { echo "[gazebo] gzserver exited early"; exit 1; }

echo "[gazebo] spawning robot…"
ros2 run gazebo_ros spawn_entity.py -timeout 120 \
  -entity my_robot \
  -file "${TURTLEBOT_SDF}" \
  -x "${SPAWN_X:--2.0}" -y "${SPAWN_Y:--2.0}" -z "${SPAWN_Z:-0.15}" \
  || echo "[gazebo] spawn warning (entity may already exist)"

echo "[gazebo] waiting for /reset_simulation…"
for ((i = 0; i < 120; i += 2)); do
  if ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
    echo "[gazebo] ready."
    if [[ "${SIM_DEMO_GAZEBO_DETACH:-0}" == "1" ]]; then
      echo "[gazebo] detached (gzserver pid ${GZ_PID})."
      exit 0
    fi
    wait "$GZ_PID"
    exit 0
  fi
  sleep 2
done
echo "[gazebo] ERROR: /reset_simulation not found"
exit 1
