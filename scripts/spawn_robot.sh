#!/usr/bin/env bash
# Spawn TurtleBot3 in the running Gazebo (after open_gazebo.sh or eval gzserver is up).
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS="${ROS2_WS:-$HOME/ros2_ws}"
PKG="$WS/src/safe_drl_nav/safe_drl_nav"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"
ENTITY="${PFE_ROBOT_ENTITY:-my_robot}"
if [[ "${PFE_HIDE_LIDAR:-1}" == "1" ]]; then
  TB="$(bash "$WS/scripts/pfe_turtlebot_no_lidar_sdf.sh")"
else
  TB="/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
fi

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
if [[ -f "$PKG/pfe_gazebo_env.sh" ]]; then
    # shellcheck source=/dev/null
    source "$PKG/pfe_gazebo_env.sh"
    pfe_gazebo_disable_online_model_db
fi
set -u

if ! pgrep -x gzserver >/dev/null; then
    echo "[spawn] ERROR: gzserver not running — start Gazebo first."
    exit 1
fi

export SPAWN_X SPAWN_Y SPAWN_Z PFE_ROBOT_ENTITY="$ENTITY" TURTLEBOT_SDF="$TB"
if ! ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
    echo "[spawn] Starting /reset_simulation helper…"
    python3 "$PKG/pfe_reset_simulation.py" &
    sleep 2
fi

echo "[spawn] Spawning $ENTITY at ($SPAWN_X, $SPAWN_Y)…"
ros2 run gazebo_ros spawn_entity.py -timeout 120 \
    -entity "$ENTITY" -file "$TB" \
    -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z"
echo "[spawn] done — look for the burger robot in gzclient."
