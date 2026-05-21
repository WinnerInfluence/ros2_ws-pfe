#!/usr/bin/env bash
# =============================================================================
# Master orchestrator: interactive (or numeric) launcher for test_maze.py
# Policy evaluation against a *already running* ROS 2 / Gazebo stack.
# For full sim + training, use start_pfe.sh instead.
# =============================================================================
set -euo pipefail

# --- User-editable defaults -------------------------------------------------
DEFAULT_ENV="${DEFAULT_ENV:-current_random_lab}"
DEFAULT_ALGO="${DEFAULT_ALGO:-sac}"
DEFAULT_EPISODES="${DEFAULT_EPISODES:-4000}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
AGENT_DIR="${AGENT_DIR:-$SCRIPT_DIR}"
if [[ ! -f "$AGENT_DIR/test_maze.py" ]]; then
  AGENT_DIR="$WS_PATH/src/safe_drl_nav/safe_drl_nav"
fi

# Non-interactive: START_TEST_CHOICE=1 bash start.sh   OR   bash start.sh 2
# Extra passthrough after -- is forwarded to test_maze.py, e.g.:
#   bash start.sh 1 -- --target-x -3 --target-y -4
# -----------------------------------------------------------------------------

EXTRA_ARGS=()
CHOICE_FROM_CLI=""
if [[ "${1:-}" == "--" ]]; then
  shift
  EXTRA_ARGS+=("$@")
elif [[ $# -gt 0 ]] && [[ "$1" != -* ]]; then
  CHOICE_FROM_CLI="$1"
  shift || true
  if [[ "${1:-}" == "--" ]]; then
    shift
    EXTRA_ARGS+=("$@")
  fi
fi

_die() {
  echo "error: $*" >&2
  exit 1
}

_source_ros2_ws() {
  local ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
  local ws_setup="$WS_PATH/install/setup.bash"
  [[ -f "$ros_setup" ]] || _die "Missing $ros_setup — install ROS 2 ${ROS_DISTRO} or set ROS_DISTRO."
  # shellcheck source=/dev/null
  source "$ros_setup"
  if [[ -f "$ws_setup" ]]; then
    # shellcheck source=/dev/null
    source "$ws_setup"
  else
    echo "warning: $ws_setup not found — run 'colcon build' in $WS_PATH or fix WS_PATH." >&2
  fi
}

_prompt_nonempty() {
  local prompt="$1" default="$2" var
  read -r -p "$prompt [${default}]: " var || true
  var="${var:-$default}"
  [[ -n "$var" ]] || { echo "invalid: empty value" >&2; return 1; }
  REPLY="$var"
}

_prompt_int_ge0() {
  local prompt="$1" default="$2" x
  while true; do
    read -r -p "$prompt [${default}]: " x || true
    x="${x:-$default}"
    if [[ "$x" =~ ^[0-9]+$ ]]; then
      REPLY="$x"
      return 0
    fi
    echo "invalid: enter a non-negative integer." >&2
  done
}

_prompt_bool_shield() {
  local x
  while true; do
    read -r -p "Enable LiDAR shield? (y/N): " x || true
    x="$(echo "${x:-n}" | tr '[:upper:]' '[:lower:]')"
    case "$x" in
      y|yes) REPLY=1; return 0 ;;
      n|no|"") REPLY=0; return 0 ;;
      *) echo "invalid: answer y or n." >&2 ;;
    esac
  done
}

_run_test_maze() {
  local algo="$1" env_name="$2" episodes="$3" shield="$4"
  shift 4
  local -a cmd=(python3 "$AGENT_DIR/test_maze.py" --algo "$algo" --env "$env_name" --episodes "$episodes")
  [[ "$shield" == "1" ]] && cmd+=(--shield)
  cmd+=("$@")
  echo "----------------------------------------------------------------"
  echo " AGENT_DIR:  $AGENT_DIR"
  echo " ENV_LOG:    $env_name   (ensure Gazebo is running this world)"
  echo " CMD:        ${cmd[*]}"
  echo "----------------------------------------------------------------"
  (cd "$AGENT_DIR" && "${cmd[@]}")
}

_run_menu_custom() {
  local algo env_name episodes shield
  _prompt_nonempty "Algorithm key (e.g. sac, td3)" "$DEFAULT_ALGO" || return 1
  algo="$REPLY"
  _prompt_nonempty "Environment / world name" "$DEFAULT_ENV" || return 1
  env_name="$REPLY"
  _prompt_int_ge0 "Max control iterations (0 = until Ctrl+C)" "$DEFAULT_EPISODES" || return 1
  episodes="$REPLY"
  _prompt_bool_shield || return 1
  shield="$REPLY"
  _run_test_maze "$algo" "$env_name" "$episodes" "$shield" "${EXTRA_ARGS[@]}"
}

_show_menu_loop() {
  local choice=""
  while true; do
    clear 2>/dev/null || true
    echo "========================================================"
    echo "  Neural maze test (test_maze.py)                      "
    echo "========================================================"
    echo "  Defaults: env=$DEFAULT_ENV  algo=$DEFAULT_ALGO  episodes=$DEFAULT_EPISODES"
    echo "  Workspace: $WS_PATH  |  ROS: $ROS_DISTRO"
    echo "--------------------------------------------------------"
    echo "  1) SAC  — no shield"
    echo "  2) SAC  — with --shield"
    echo "  3) TD3  — no shield"
    echo "  4) TD3  — with --shield"
    echo "  5) Custom (enter algo / env / episodes / shield manually)"
    echo "  6) Exit"
    echo "========================================================"
    if [[ -n "${EXTRA_ARGS[*]:-}" ]]; then
      echo "  Extra args forwarded to test_maze.py: ${EXTRA_ARGS[*]}"
    fi
    read -r -p "Choice [1-6]: " choice || exit 1
    choice="$(echo "$choice" | tr -d '[:space:]')"
    case "$choice" in
      1) _run_test_maze sac "$DEFAULT_ENV" "$DEFAULT_EPISODES" 0 "${EXTRA_ARGS[@]}"; break ;;
      2) _run_test_maze sac "$DEFAULT_ENV" "$DEFAULT_EPISODES" 1 "${EXTRA_ARGS[@]}"; break ;;
      3) _run_test_maze td3 "$DEFAULT_ENV" "$DEFAULT_EPISODES" 0 "${EXTRA_ARGS[@]}"; break ;;
      4) _run_test_maze td3 "$DEFAULT_ENV" "$DEFAULT_EPISODES" 1 "${EXTRA_ARGS[@]}"; break ;;
      5) _run_menu_custom; break ;;
      6) echo "Bye."; exit 0 ;;
      *) echo "invalid choice: use 1–6." >&2; sleep 1 ;;
    esac
  done
}

main() {
  _source_ros2_ws
  [[ -f "$AGENT_DIR/test_maze.py" ]] || _die "test_maze.py not found under $AGENT_DIR"

  local c="${CHOICE_FROM_CLI:-${START_TEST_CHOICE:-}}"
  if [[ -n "$c" ]]; then
    case "$c" in
      1) _run_test_maze sac "$DEFAULT_ENV" "$DEFAULT_EPISODES" 0 "${EXTRA_ARGS[@]}" ;;
      2) _run_test_maze sac "$DEFAULT_ENV" "$DEFAULT_EPISODES" 1 "${EXTRA_ARGS[@]}" ;;
      3) _run_test_maze td3 "$DEFAULT_ENV" "$DEFAULT_EPISODES" 0 "${EXTRA_ARGS[@]}" ;;
      4) _run_test_maze td3 "$DEFAULT_ENV" "$DEFAULT_EPISODES" 1 "${EXTRA_ARGS[@]}" ;;
      5) _run_menu_custom ;;
      6) exit 0 ;;
      *) _die "Invalid START_TEST_CHOICE / first arg: $c (use 1–6)" ;;
    esac
    return 0
  fi
  _show_menu_loop
}

main "$@"
