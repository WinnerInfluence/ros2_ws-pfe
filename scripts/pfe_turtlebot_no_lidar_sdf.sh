#!/usr/bin/env bash
# Cached TurtleBot3 burger SDF with Gazebo laser fan disabled (<visualize>false</visualize>).
# Usage: export TURTLEBOT_SDF="$(bash ~/ros2_ws/scripts/pfe_turtlebot_no_lidar_sdf.sh)"
set -euo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
PKG="${PFE_PKG:-$WS/src/safe_drl_nav/safe_drl_nav}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
MODEL="${TURTLEBOT3_MODEL:-burger}"
SRC="${TURTLEBOT_SDF_SRC:-/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_${MODEL}/model.sdf}"
OUT="$PKG/sim_assets/robot/turtlebot3_${MODEL}_no_lidar_vis.sdf"

[[ -f "$SRC" ]] || { echo "[pfe_turtlebot] missing source SDF: $SRC" >&2; exit 1; }

mkdir -p "$(dirname "$OUT")"
if [[ ! -f "$OUT" ]] || [[ "$SRC" -nt "$OUT" ]]; then
  sed 's/<visualize>true<\/visualize>/<visualize>false<\/visualize>/g' "$SRC" > "$OUT"
fi
echo "$OUT"
