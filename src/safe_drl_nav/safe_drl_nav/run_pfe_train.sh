#!/usr/bin/env bash
# One-shot SAC adaptation training (sources ROS + ws, avoids stuck /reset_simulation reply by default).
#
# One-liner equivalent (from anywhere):
#   bash ~/ros2_ws/src/safe_drl_nav/safe_drl_nav/run_pfe_train.sh
#
# Env (optional):
#   WS_PATH              default ~/ros2_ws
#   PFE_PRESET           default pfe_sac_adapt  (or pfe_td3_adapt)
#   PFE_TRAIN_SEED       default 42
#   PFE_SIM_RESET        fire = --reset-fire-and-forget (default)
#                        none = --no-sim-reset
#                        wait = normal reset reply wait (--reset-reply-wait-sec from extra args)
#   PFE_RESET_SERVICE_WAIT  seconds to wait for /reset_simulation when MODE=fire (default 12)
#   PFE_ENV_STEP_SLEEP   optional, e.g. 0.03 → --env-step-sleep-sec (faster episodes)
#   REBUILD=1            run colcon build --packages-select safe_drl_nav first
#
# Extra args are passed through, e.g.:
#   ./run_pfe_train.sh --max-episodes 500
set -eo pipefail
# Do not use `set -u` before sourcing ROS: setup.bash reads vars like
# AMENT_TRACE_SETUP_FILES that may be unset.

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
WS="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="$WS/src/safe_drl_nav/safe_drl_nav"

# shellcheck disable=SC1091
source "$AGENT_DIR/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

if [[ "${REBUILD:-0}" == "1" ]]; then
  (cd "$WS" && colcon build --packages-select safe_drl_nav)
  source "$WS/install/setup.bash"
fi

cd "$AGENT_DIR"

PRESET="${PFE_PRESET:-pfe_sac_adapt}"
SEED="${PFE_TRAIN_SEED:-42}"
MODE="${PFE_SIM_RESET:-fire}"

RESET_FLAGS=()
case "$MODE" in
  none) RESET_FLAGS+=(--no-sim-reset) ;;
  wait) ;; # user can add --reset-reply-wait-sec 10 etc. in "$@"
  *)
    # Short service wait: without Gazebo, agent skips reset after timeout instead of hanging 45s.
    RESET_FLAGS+=(--reset-fire-and-forget --reset-service-wait-sec "${PFE_RESET_SERVICE_WAIT:-12}")
    ;;
esac

STEP_FLAGS=()
if [[ -n "${PFE_ENV_STEP_SLEEP:-}" ]]; then
  STEP_FLAGS+=(--env-step-sleep-sec "$PFE_ENV_STEP_SLEEP")
fi

exec python3 main_agent.py --preset "$PRESET" --train-seed "$SEED" "${RESET_FLAGS[@]}" "${STEP_FLAGS[@]}" "$@"
