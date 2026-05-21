#!/usr/bin/env bash
# Local adaptation-style training (source before run or call from your Desktop menu).
set -euo pipefail

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DESKTOP="${DESKTOP:-$HOME/Desktop}"

# shellcheck disable=SC1091
source "$AGENT_DIR/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env

source /opt/ros/humble/setup.bash
source "$WS_PATH/install/setup.bash"
cd "$AGENT_DIR"

PRESET="${PFE_PRESET:-pfe_sac_adapt}"
SEED="${PFE_TRAIN_SEED:-42}"
REGEN="${PFE_WORLD_REGEN_SCRIPT:-$DESKTOP/randomize_world.py}"
INTERVAL="${PFE_WORLD_REGEN_INTERVAL:-0}"

CMD=(python3 main_agent.py --preset "$PRESET" --train-seed "$SEED")
if [[ "${PFE_NO_SIM_RESET:-0}" == "1" ]]; then
  CMD+=(--no-sim-reset)
fi
if [[ -f "$REGEN" && "${INTERVAL}" != "0" ]]; then
  CMD+=(--world-regen-script "$REGEN" --world-regen-interval "${INTERVAL}")
elif [[ ! -f "$REGEN" ]]; then
  echo "Note: no world regen script at $REGEN (set PFE_WORLD_REGEN_SCRIPT). Geometry stays fixed until you restart Gazebo."
fi

echo "Running: ${CMD[*]}"
exec "${CMD[@]}"
