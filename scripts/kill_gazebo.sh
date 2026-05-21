#!/usr/bin/env bash
# Stop all Gazebo / sim processes and free master port 11345 (fixes "Preparing your world" freeze).
set -u
WS="${ROS2_WS:-$HOME/ros2_ws}"
PKG="$WS/src/safe_drl_nav/safe_drl_nav"
echo "[kill_gazebo] stopping sim processes…"
if [[ -f "$PKG/pfe_gazebo_env.sh" ]]; then
    # shellcheck source=/dev/null
    source "$PKG/pfe_gazebo_env.sh"
    pfe_gazebo_kill_all
else
    for _ in 1 2 3; do
        pkill -f 'ros2 launch gazebo_ros' 2>/dev/null || true
        pkill -f 'gazebo .*\.world' 2>/dev/null || true
        pkill -x gzclient 2>/dev/null || true
        pkill -x gzserver 2>/dev/null || true
        sleep 1
        pgrep -x gzserver &>/dev/null || break
    done
    pkill -9 -x gzclient 2>/dev/null || true
    pkill -9 -x gzserver 2>/dev/null || true
    command -v fuser >/dev/null && fuser -k 11345/tcp 2>/dev/null || true
    sleep 1
fi
if pgrep -x gzserver &>/dev/null; then
    echo "[kill_gazebo] WARNING: gzserver still running (pid $(pgrep -x gzserver))"
    exit 1
fi
echo "[kill_gazebo] done — port 11345 free, safe to launch Gazebo"
