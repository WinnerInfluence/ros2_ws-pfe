#!/usr/bin/env bash
# Run with Gazebo up — required for web dashboard upload and live eval.
set -euo pipefail
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="${WS_PATH}/src/safe_drl_nav/safe_drl_nav"
ALGO="${1:-sac}"
MODEL_DIR="$AGENT_DIR/trained_models"
default_model_for_algo() {
  local a="$1"
  local candidates=()
  case "$a" in
    td3)
      candidates=(
        td3_actor_maze_best_ever.pth
        td3_actor_maze.pth
        td3_actor_adapt_best_ever.pth
        td3_actor_adapt.pth
      )
      ;;
    *)
      candidates=(
        sac_actor_maze_best_ever.pth
        sac_actor_maze.pth
        sac_actor_phase3_maze.pth
      )
      ;;
  esac
  local f
  for f in "${candidates[@]}"; do
    if [[ -f "$MODEL_DIR/$f" ]]; then
      echo "$MODEL_DIR/$f"
      return 0
    fi
  done
  echo "$MODEL_DIR/sac_actor_maze_best_ever.pth"
}
MODEL="${2:-$(default_model_for_algo "$ALGO")}"
if [[ ! -f "$MODEL" ]]; then
  echo "ERROR: checkpoint not found: $MODEL" >&2
  echo "  algo=$ALGO — pass an explicit .pth as second argument." >&2
  exit 1
fi

if [[ ! -f "$AGENT_DIR/hot_swap_eval_node.py" ]]; then
  echo "ERROR: missing $AGENT_DIR/hot_swap_eval_node.py" >&2
  exit 1
fi
if grep -qE 'import EvalNode|from evaluate_agent import EvalNode' "$AGENT_DIR/hot_swap_eval_node.py" 2>/dev/null; then
  echo "ERROR: hot_swap_eval_node.py still imports EvalNode — rebuild workspace." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
[[ -f "$WS_PATH/install/setup.bash" ]] && source "$WS_PATH/install/setup.bash"
set -u

if ! python3 -c "import rclpy" 2>/dev/null; then
  echo "ERROR: rclpy not found. Run:" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  echo "  sudo apt install ros-humble-rclpy" >&2
  exit 1
fi

export PYTHONPATH="${AGENT_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
export PFE_WORLD="${PFE_WORLD:-$AGENT_DIR/sim_assets/worlds/eval_maze.world}"
export TRAINING_CONTRACT="${TRAINING_CONTRACT:-$AGENT_DIR/training_contract.yaml}"

cd "$AGENT_DIR"
echo "hot_swap_eval: algo=$ALGO model=$MODEL"
# Default --no-reset: reset_env() must not call spin_once() inside the control timer
# (MultiThreadedExecutor deadlock → no /cmd_vel, frozen LiDAR). Teleport once via:
#   ros2 service call /reset_simulation std_srvs/srv/Empty "{}"
# Set HOT_SWAP_SIM_RESET=1 only if you accept possible hang (not for local video).
EXTRA=(--no-reset)
[[ "${HOT_SWAP_SAC_SAMPLE:-0}" == "1" || "${PRESENTATION_DEMO:-0}" == "1" ]] && EXTRA+=(--sac-sample)
[[ "${HOT_SWAP_SIM_RESET:-0}" == "1" ]] && EXTRA=()
SLEEP="${HOT_SWAP_ENV_STEP_SLEEP:-0.05}"
PERIOD="${HOT_SWAP_CONTROL_PERIOD:-0.05}"
exec python3 ./hot_swap_eval_node.py \
  --algo "$ALGO" \
  --model "$MODEL" \
  "${EXTRA[@]}" \
  --env-step-sleep-sec "$SLEEP" \
  --control-period "$PERIOD" \
  --waypoint-goal-radius 0.68 \
  --training-contract "$TRAINING_CONTRACT"
