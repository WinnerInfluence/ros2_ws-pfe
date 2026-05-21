#!/usr/bin/env bash
# =============================================================================
# run_auto_train.sh — Unattended Gazebo + SAC waypoint training (no menu prompts)
#
# Default (lab overnight): one lab-maze session, train_menu Phase 2 flags,
# warm-start from best_ever, headless Gazebo, log under pfe_logs/.
#
# Usage:
#   bash run_auto_train.sh                    # lab maze, background stack
#   bash run_auto_train.sh --foreground       # stay attached (see all logs)
#   bash run_auto_train.sh --roadmap          # full train_waypoint.sh phases 1→4
#   bash run_auto_train.sh --roadmap --phase 2
#
# Environment (optional):
#   PFE_GAZEBO_GUI=0          headless (default for auto)
#   PRETRAINED_CKPT=...       default: trained_models/sac_actor_maze_best_ever.pth
#   PFE_WORLD=...             default: sim_assets/worlds/current_random_lab.world
#   AUTO_EARLY_STOP=10        consecutive maze solves to exit (0 = run until Ctrl+C)
#   AUTO_MAX_EPISODES=10000   episode cap
#   AUTO_REGEN_STYLE=0        randomize_world.py style before lab run (0=S-curve)
# =============================================================================
set -eo pipefail

WS="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="$WS/src/safe_drl_nav/safe_drl_nav"
ROS_DISTRO="${ROS_DISTRO:-humble}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
TURTLEBOT_SDF="/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf"
LOG_DIR="$AGENT_DIR/pfe_logs"
MODEL_DIR="$AGENT_DIR/trained_models"
REGEN_SCRIPT="$AGENT_DIR/sim_assets/scripts/randomize_world.py"
WORLD_DEFAULT="$AGENT_DIR/sim_assets/worlds/current_random_lab.world"

export PFE_GAZEBO_GUI="${PFE_GAZEBO_GUI:-0}"
PRETRAINED="${PRETRAINED_CKPT:-$MODEL_DIR/sac_actor_maze_best_ever.pth}"
export PFE_WORLD="${PFE_WORLD:-$WORLD_DEFAULT}"
AUTO_EARLY_STOP="${AUTO_EARLY_STOP:-10}"
AUTO_MAX_EPISODES="${AUTO_MAX_EPISODES:-10000}"
AUTO_REGEN_STYLE="${AUTO_REGEN_STYLE:-0}"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"

FOREGROUND=0
ROADMAP=0
ROADMAP_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --foreground|-f) FOREGROUND=1; shift ;;
        --roadmap)       ROADMAP=1; shift ;;
        --phase|--world)
            ROADMAP=1
            ROADMAP_ARGS+=("$1" "$2")
            shift 2
            ;;
        --dry-run)
            ROADMAP=1
            ROADMAP_ARGS+=("$1")
            shift
            ;;
        -h|--help)
            sed -n '1,22p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1 (try --help)" >&2
            exit 1
            ;;
    esac
done

# shellcheck disable=SC1091
source "$AGENT_DIR/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"

_ts() { date +%Y%m%d_%H%M%S; }

_kill_stale() {
    echo "── Stopping stale Gazebo / agent processes ──"
    local procs=(gzserver gzclient rviz2 main_agent.py evaluate_agent.py)
    local p
    for p in "${procs[@]}"; do pkill -9 -f "$p" 2>/dev/null || true; done
    sleep 2
    local _i
    for ((_i = 0; _i < 20; _i++)); do
        pgrep -f gzserver >/dev/null 2>&1 || break
        pkill -9 -f gzserver 2>/dev/null || true
        sleep 1
    done
}

_wait_reset_service() {
    local deadline=$((SECONDS + 120))
    echo "── Waiting for /reset_simulation (max 120 s) ──"
    while (( SECONDS < deadline )); do
        if ros2 service list 2>/dev/null | grep -qx '/reset_simulation'; then
            echo "   /reset_simulation ready."
            return 0
        fi
        sleep 2
    done
    echo "ERROR: /reset_simulation not available — Gazebo did not start correctly." >&2
    return 1
}

_run_lab_session() {
    local log_file="$LOG_DIR/auto_train_lab_$(_ts).log"
    local pid_file="$LOG_DIR/auto_train.pid"

    if [[ ! -f "$PRETRAINED" ]]; then
        echo "WARN: Pretrained not found: $PRETRAINED" >&2
        echo "      Falling back to $MODEL_DIR/sac_actor_maze.pth" >&2
        PRETRAINED="$MODEL_DIR/sac_actor_maze.pth"
    fi

    _kill_stale

    if [[ -f "$REGEN_SCRIPT" ]]; then
        echo "── Regenerating lab world (style ${AUTO_REGEN_STYLE}) ──"
        python3 "$REGEN_SCRIPT" "$AUTO_REGEN_STYLE"
        export PFE_WORLD="$WORLD_DEFAULT"
    fi

    local gz_gui
    gz_gui="$(pfe_gazebo_gui_suffix)"
    echo "── Launching Gazebo (headless=${PFE_GAZEBO_GUI}) ──"
    echo "   World: $PFE_WORLD"
    ros2 launch gazebo_ros gazebo.launch.py "world:=$PFE_WORLD" $gz_gui &
    local gz_pid=$!
    sleep 12

    _wait_reset_service

    echo "── Spawning TurtleBot3 (${TURTLEBOT3_MODEL}) ──"
    ros2 run gazebo_ros spawn_entity.py -timeout 120 \
        -entity my_robot -file "$TURTLEBOT_SDF" \
        -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z"
    sleep 3

    local pretrain_arg=()
    [[ -f "$PRETRAINED" ]] && pretrain_arg=(--load-pretrained "$PRETRAINED")

    local train_cmd=(
    python3 main_agent.py
    --preset pfe_sac_waypoint
    "${pretrain_arg[@]}"
    --device cpu
    --env-step-sleep-sec 0.03
    --adaptive-base-steps 1500
    --max-episode-steps 2400
    --max-episodes "$AUTO_MAX_EPISODES"
    --lr 1e-4
    --replay-warmup-steps 28000
    --waypoint-goal-radius 0.68
    --early-stop-consecutive-maze-solves "$AUTO_EARLY_STOP"
    --reset-fire-and-forget
    --reset-service-wait-sec 45
  )

    echo "── Starting SAC waypoint training ──"
    echo "   Warm-start : $PRETRAINED"
    echo "   Early-stop : ${AUTO_EARLY_STOP} consecutive full maze solves (0=disabled)"
    echo "   Log file   : $log_file"
    cd "$AGENT_DIR"

    # CPU thread caps (same idea as train_menu)
    # shellcheck disable=SC1090
    eval "$(pfe_training_cpu_math_env)"

    if [[ "$FOREGROUND" == "1" ]]; then
        echo "   Mode: foreground (Ctrl+C stops training; Gazebo may keep running)"
        "${train_cmd[@]}" 2>&1 | tee -a "$log_file"
    else
        nohup "${train_cmd[@]}" >>"$log_file" 2>&1 &
        local train_pid=$!
        echo "$train_pid" >"$pid_file"
        echo "   Training PID: $train_pid  (also in $pid_file)"
        echo "   Gazebo PID  : $gz_pid"
        echo ""
        echo "══ AUTO TRAIN STARTED ══"
        echo "  tail -f $log_file"
        echo "  tail -f $AGENT_DIR/pfe_logs/sac_maze_metrics.csv"
        echo "  stop: kill \$(cat $pid_file); pkill -f gzserver"
    fi
}

if [[ "$ROADMAP" == "1" ]]; then
    echo "══ Delegating to train_waypoint.sh (4-phase roadmap) ══"
    exec bash "$AGENT_DIR/train_waypoint.sh" "${ROADMAP_ARGS[@]}"
fi

_run_lab_session
