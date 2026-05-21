#!/usr/bin/env bash
# Open Gazebo with stable split launch (gzserver + plain gzclient — no eol_gui crash).
#
#   bash ~/ros2_ws/scripts/open_gazebo.sh spawn        # light Rabat + robot (default)
#   bash ~/ros2_ws/scripts/open_gazebo.sh rabat_fast spawn
#   PFE_TEXTURED=1 … open_gazebo.sh rabat spawn        # full textures (often freezes on laptops)
#
# Textured eval_rabat.world is cosmetic only — same waypoints/physics as rabat_fast.
set -euo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
PKG="$WS/src/safe_drl_nav/safe_drl_nav"
ROS_DISTRO="${ROS_DISTRO:-humble}"
MAT="$PKG/sim_assets/materials/scripts"
KILL_SCRIPT="$(dirname "$0")/kill_gazebo.sh"
SPAWN_SCRIPT="$(dirname "$0")/spawn_robot.sh"

WORLD_KEY="${1:-rabat_fast}"
DO_SPAWN=0
if [[ "${1:-}" == "spawn" ]]; then
    WORLD_KEY="rabat_fast"
    DO_SPAWN=1
elif [[ "${2:-}" == "spawn" || "${PFE_SPAWN_ROBOT:-0}" == "1" ]]; then
    DO_SPAWN=1
fi

case "$WORLD_KEY" in
    rabat)
        if [[ "${PFE_TEXTURED:-0}" == "1" ]]; then
            WORLD="$PKG/sim_assets/worlds/eval_rabat.world"
        else
            echo "[gazebo] rabat → rabat_fast (textures freeze gzclient on many laptops)."
            WORLD="$PKG/sim_assets/worlds/eval_rabat_fast.world"
        fi
        ;;
    rabat_fast) WORLD="$PKG/sim_assets/worlds/eval_rabat_fast.world" ;;
    egypt)      WORLD="$PKG/sim_assets/worlds/eval_egypt.world" ;;
    maze)       WORLD="$PKG/sim_assets/worlds/eval_maze.world" ;;
    lab)        WORLD="$PKG/sim_assets/worlds/current_random_lab.world" ;;
    *)
        if [[ -f "$WORLD_KEY" ]]; then WORLD="$WORLD_KEY"
        else echo "[ERROR] unknown world: $WORLD_KEY"; exit 1; fi
        ;;
esac

[[ -f "$WORLD" ]] || { echo "[ERROR] missing $WORLD"; exit 1; }

if [[ -z "${DISPLAY:-}" ]]; then
    echo "[ERROR] DISPLAY not set — use a desktop terminal."
    exit 1
fi

bash "$KILL_SCRIPT" || true

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
set -u

# shellcheck source=/dev/null
source "$PKG/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env
export PFE_GAZEBO_GUI=1
export PFE_GAZEBO_STABLE="${PFE_GAZEBO_STABLE:-1}"
export PFE_WORLD="$(readlink -f "$WORLD")"
pfe_gazebo_prepend_resource_path "$MAT"

echo ""
echo "  World: $PFE_WORLD"
if [[ "$DO_SPAWN" == "0" ]]; then
    echo "  Robot:  bash ~/ros2_ws/scripts/spawn_robot.sh"
    echo "  Or:     bash ~/ros2_ws/scripts/open_gazebo.sh ${WORLD_KEY} spawn"
fi
echo ""

pfe_gazebo_start_server_bg "$PFE_WORLD" >/dev/null
if ! pfe_gazebo_wait_gzserver "${PFE_GAZEBO_WAIT:-90}"; then
    echo "[ERROR] gzserver did not start."
    exit 1
fi

if [[ "$DO_SPAWN" == "1" ]]; then
    bash "$SPAWN_SCRIPT"
fi

if [[ "${PFE_NO_GUI:-0}" == "1" ]]; then
    echo "[gazebo] PFE_NO_GUI=1 — sim running headless (robot + /scan OK). Ctrl+C to stop."
    wait
else
    pfe_gazebo_start_gzclient_fg
fi
