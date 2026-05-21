#!/usr/bin/env bash
# =============================================================================
# run_td3_overnight.sh — Unattended TD3: Phase 1 adapt → Phase 2 waypoints (lab)
#
# No menu prompts. Headless Gazebo by default. Restarts sim between phases.
# Logs: pfe_logs/td3_overnight_phase1_*.log , td3_overnight_phase2_*.log
#
# Usage (before sleep):
#   bash run_td3_overnight.sh
#   bash run_td3_overnight.sh --foreground    # watch both phases in this terminal
#
# Environment:
#   TD3_PHASE1_EP=600       adapt episodes (default 600 ≈ few hours)
#   AUTO_EARLY_STOP=10        Phase 2 maze early-stop (0 = until Ctrl+C)
#   PFE_GAZEBO_GUI=0          headless overnight (default)
#   TD3_SKIP_PHASE1=1         skip adapt if td3_actor_adapt.pth already exists
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
export PFE_WORLD="${PFE_WORLD:-$WORLD_DEFAULT}"
TD3_PHASE1_EP="${TD3_PHASE1_EP:-600}"
AUTO_EARLY_STOP="${AUTO_EARLY_STOP:-10}"
TD3_SKIP_PHASE1="${TD3_SKIP_PHASE1:-0}"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"

FOREGROUND=0
[[ "${1:-}" == "--foreground" || "${1:-}" == "-f" ]] && FOREGROUND=1

# shellcheck disable=SC1091
source "$AGENT_DIR/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"

_ts() { date +%Y%m%d_%H%M%S; }

_kill_stale() {
    echo "── Stopping stale Gazebo / agents ──"
    local p
    for p in gzserver gzclient rviz2 main_agent.py evaluate_agent.py; do
        pkill -9 -f "$p" 2>/dev/null || true
    done
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
    echo "── Waiting for /reset_simulation ──"
    while (( SECONDS < deadline )); do
        if ros2 service list 2>/dev/null | grep -qx '/reset_simulation'; then
            echo "   ready."
            return 0
        fi
        sleep 2
    done
    echo "ERROR: /reset_simulation missing — Gazebo failed to start." >&2
    return 1
}

_start_sim() {
    if [[ -f "$REGEN_SCRIPT" ]]; then
        echo "── Regenerating lab world (S-curve style 0) ──"
        python3 "$REGEN_SCRIPT" 0
        export PFE_WORLD="$WORLD_DEFAULT"
    fi
    local gz_gui
    gz_gui="$(pfe_gazebo_gui_suffix)"
    echo "── Gazebo (GUI=${PFE_GAZEBO_GUI}) world=$PFE_WORLD ──"
    ros2 launch gazebo_ros gazebo.launch.py "world:=$PFE_WORLD" $gz_gui &
    GZ_PID=$!
    sleep 12
    _wait_reset_service
    echo "── Spawning robot ──"
    ros2 run gazebo_ros spawn_entity.py -timeout 120 \
        -entity my_robot -file "$TURTLEBOT_SDF" \
        -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z"
    sleep 3
}

_run_train() {
    local label="$1"
    shift
    local log_file="$LOG_DIR/td3_overnight_${label}_$(_ts).log"
    echo "── TD3 ${label} → $log_file ──"
    cd "$AGENT_DIR"
  # shellcheck disable=SC1090
    eval "$(pfe_training_cpu_math_env)"
    if [[ "$FOREGROUND" == "1" ]]; then
        python3 main_agent.py "$@" 2>&1 | tee -a "$log_file"
    else
        python3 main_agent.py "$@" >>"$log_file" 2>&1
    fi
}

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  TD3 overnight — Phase 1 adapt → Phase 2 waypoint (lab)      ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo "  Phase 1 episodes : ${TD3_PHASE1_EP}"
echo "  Phase 2 early-stop: ${AUTO_EARLY_STOP} consecutive maze solves"
echo "  Headless           : PFE_GAZEBO_GUI=${PFE_GAZEBO_GUI}"
echo ""

_kill_stale
_start_sim

ADAPT_CKPT="$MODEL_DIR/td3_actor_adapt.pth"

if [[ "$TD3_SKIP_PHASE1" == "1" && -f "$ADAPT_CKPT" ]]; then
    echo "── Skipping Phase 1 (TD3_SKIP_PHASE1=1, adapt exists) ──"
else
    _run_train "phase1" \
        --algo td3 \
        --max-episodes "$TD3_PHASE1_EP" \
        --reset-fire-and-forget \
        --reset-service-wait-sec 45 \
        --device cpu
fi

if [[ ! -f "$ADAPT_CKPT" ]]; then
    echo "WARN: $ADAPT_CKPT not found — Phase 2 may start from random weights." >&2
fi

echo "── Restarting sim for Phase 2 ──"
_kill_stale
_start_sim

_run_train "phase2" \
    --preset pfe_td3_waypoint \
    --load-pretrained "$ADAPT_CKPT" \
    --device cpu \
    --env-step-sleep-sec 0.03 \
    --adaptive-base-steps 1500 \
    --max-episode-steps 2400 \
    --lr 1e-4 \
    --replay-warmup-steps 28000 \
    --waypoint-goal-radius 0.68 \
    --early-stop-consecutive-maze-solves "$AUTO_EARLY_STOP" \
    --reset-fire-and-forget \
    --reset-service-wait-sec 45

echo ""
echo "══ TD3 OVERNIGHT FINISHED ══"
echo "  Checkpoints: td3_actor_adapt.pth , td3_actor_maze.pth , td3_actor_maze_best_ever.pth"
echo "  Logs:        $LOG_DIR/td3_overnight_phase*.log"
echo "  Metrics:     $LOG_DIR/td3_maze_metrics.csv (after Phase 2)"
