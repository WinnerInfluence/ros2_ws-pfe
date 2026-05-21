#!/usr/bin/env bash
# Short presentation recording: checkpoint + stochastic SAC (matches paper-eval sampling).
# Prerequisite: Gazebo GUI, robot spawned on PFE_WORLD.
#
# Usage:
#   bash ~/ros2_ws/scripts/run_presentation_demo.sh
#
# Between takes (new episode):
#   ros2 service call /reset_simulation std_srvs/srv/Empty "{}"
#   ros2 topic pub --once /policy_control std_msgs/msg/String "{data: reset_episode}"
set -euo pipefail

WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="${WS_PATH}/src/safe_drl_nav/safe_drl_nav"
_WORLD_MAZE="$AGENT_DIR/sim_assets/worlds/eval_maze.world"
_WORLD_EGYPT="$AGENT_DIR/sim_assets/worlds/eval_egypt.world"
_MAZE_PEAK="$AGENT_DIR/trained_models/sac_actor_maze_best_ever_eval_maze.pth"
_PHASE3_EGYPT="$AGENT_DIR/trained_models/sac_actor_phase3_egypt.pth"
_PHASE3_MAZE="$AGENT_DIR/trained_models/sac_actor_phase3_maze.pth"
_PHASE3_RABAT="$AGENT_DIR/trained_models/sac_actor_phase3_rabat.pth"
_LAB_WORLD="$AGENT_DIR/sim_assets/worlds/current_random_lab.world"
_LAB_BEST="$AGENT_DIR/trained_models/sac_actor_maze_best_ever.pth"
WORLD="${PFE_WORLD:-$_LAB_WORLD}"
if [[ ! -f "$WORLD" ]]; then
  echo "ERROR: world file missing: ${PFE_WORLD:-$_LAB_WORLD}" >&2
  exit 2
fi
MODEL="${EVAL_MODEL:-}"
if [[ -z "$MODEL" ]]; then
  case "$(basename "$WORLD")" in
    eval_maze.world)
      if [[ -f "$_PHASE3_MAZE" ]]; then MODEL="$_PHASE3_MAZE"
      elif [[ -f "$_MAZE_PEAK" ]]; then MODEL="$_MAZE_PEAK"
      fi ;;
    eval_egypt.world)
      [[ -f "$_PHASE3_EGYPT" ]] && MODEL="$_PHASE3_EGYPT" ;;
    eval_rabat.world)
      [[ -f "$_PHASE3_RABAT" ]] && MODEL="$_PHASE3_RABAT" ;;
  esac
  [[ -z "$MODEL" ]] && MODEL="$_LAB_BEST"
fi
if [[ ! -f "$MODEL" ]]; then
  echo "ERROR: checkpoint missing: $MODEL" >&2
  exit 2
fi
SLEEP="${VIDEO_STEP_SLEEP:-0.03}"
PERIOD="${VIDEO_CONTROL_PERIOD:-0.03}"

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
[[ -f "$WS_PATH/install/setup.bash" ]] && source "$WS_PATH/install/setup.bash"
set -u

export PFE_WORLD="$WORLD"
export PYTHONPATH="${AGENT_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
export TRAINING_CONTRACT="${TRAINING_CONTRACT:-$AGENT_DIR/training_contract.yaml}"

if ! ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
  echo "ERROR: start Gazebo + spawn robot first." >&2
  exit 2
fi

echo "[presentation-demo] world=$(basename "$WORLD") model=$(basename "$MODEL")"
echo "[presentation-demo] sac=sample sleep=${SLEEP}s control=${PERIOD}s"
echo "[presentation-demo] teleport once, then record Gazebo…"

ros2 service call /reset_simulation std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true
sleep 0.5

pkill -f hot_swap_eval_node.py 2>/dev/null || true
sleep 0.2

cd "$AGENT_DIR"
exec python3 ./hot_swap_eval_node.py \
  --algo sac \
  --model "$MODEL" \
  --no-reset \
  --sac-sample \
  --env-step-sleep-sec "$SLEEP" \
  --control-period "$PERIOD" \
  --waypoint-goal-radius 0.68 \
  --training-contract "$TRAINING_CONTRACT"
