#!/usr/bin/env bash
# TD3 Phase-1 brain only. Gazebo must be running (see below).
#
# One-shot (Gazebo + spawn + brain):
#   bash start_pfe.sh 8
#
# Two-step (brain after sim is up):
#   Terminal A: ros2 launch gazebo_ros gazebo.launch.py world:=.../current_random_lab.world
#   Terminal B: bash run_td3_adapt_brain.sh
#
# Usage: bash run_td3_adapt_brain.sh [--no-wait]
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
WAIT_SEC="${PFE_RESET_WAIT:-120}"
NO_WAIT=0

for arg in "$@"; do
    case "$arg" in
        --no-wait) NO_WAIT=1 ;;
        -h|--help)
            sed -n '2,12p' "$0"
            exit 0
            ;;
    esac
done

# shellcheck disable=SC1091
source "$SCRIPT_DIR/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS_PATH/install/setup.bash" ]] && source "$WS_PATH/install/setup.bash"

_reset_ready() {
    ros2 service list 2>/dev/null | grep -q '/reset_simulation'
}

if _reset_ready; then
    echo "[OK] /reset_simulation is available."
elif [[ "$NO_WAIT" -eq 1 ]]; then
    echo "ERROR: /reset_simulation not found (Gazebo is not running)." >&2
    echo "  Easiest: bash $SCRIPT_DIR/start_pfe.sh 8" >&2
    exit 1
else
    echo "Waiting up to ${WAIT_SEC}s for /reset_simulation ..."
    echo "  (Start Gazebo if you have not — or run: bash $SCRIPT_DIR/start_pfe.sh 8)"
    _found=0
    for (( _t = 0; _t < WAIT_SEC; _t += 2 )); do
        if _reset_ready; then
            echo "[OK] /reset_simulation is available (after ${_t}s)."
            _found=1
            break
        fi
        if (( _t > 0 && _t % 10 == 0 )); then
            echo "  ... still waiting (${_t}s)"
        fi
        sleep 2
    done
    if [[ "$_found" -ne 1 ]]; then
        echo "ERROR: /reset_simulation not found after ${WAIT_SEC}s." >&2
        echo "  Run this in another terminal (leave it open):" >&2
        echo "    source /opt/ros/${ROS_DISTRO}/setup.bash" >&2
        echo "    ros2 launch gazebo_ros gazebo.launch.py world:=$SCRIPT_DIR/sim_assets/worlds/current_random_lab.world" >&2
        echo "  Or one command for everything: bash $SCRIPT_DIR/start_pfe.sh 8" >&2
        exit 1
    fi
fi

cd "$SCRIPT_DIR"
exec python3 main_agent.py --preset pfe_td3_adapt --use-shield \
    --reset-fire-and-forget --reset-service-wait-sec 45 --device cpu "$@"
