#!/usr/bin/env bash
# =============================================================================
# train_waypoint.sh — 4-Phase Waypoint Training Roadmap
#
# Guarantees the policy works on ALL 4 worlds:
#   current_random_lab  (training)
#   eval_rabat          (Hassan Tower pillars)
#   eval_egypt          (Pyramid desert)
#   eval_maze           (Hedge labyrinth)
#
# Each Phase restarts Gazebo with the correct world so domain randomisation
# actually loads into the simulator.
#
# Usage:
#   bash train_waypoint.sh                       # all phases
#   bash train_waypoint.sh --phase 2             # resume from Phase 2
#   bash train_waypoint.sh --phase 3 --world egypt  # fine-tune on egypt only
#   bash train_waypoint.sh --dry-run             # show commands, no execution
#
# Environment overrides:
#   TRAIN_WAYPOINT_ALGO  sac (default) | td3 — picks preset and checkpoint basename
#   PRETRAINED_CKPT      warm-start path (default: trained_models/<algo>_actor_maze.pth)
#   WS_PATH              ROS 2 workspace root (default: ~/ros2_ws)
#   PHASE1_EP         episodes for Phase 1 (default: 2000)
#   PHASE2_EP         episodes for Phase 2 (default: 1500)
#   PHASE3_EP         episodes per eval world in Phase 3 (default: 400)
# =============================================================================
# Do NOT use -u: ROS setup.bash reads AMENT_TRACE_SETUP_FILES which may be
# unset, and nounset (-u) would abort the script before sourcing completes.
set -eo pipefail

# ── Paths ─────────────────────────────────────────────────────────────────────
WS="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="$WS/src/safe_drl_nav/safe_drl_nav"
WORLDS_DIR="$AGENT_DIR/sim_assets/worlds"
REGEN_SCRIPT="$AGENT_DIR/sim_assets/scripts/randomize_world.py"
MODELS_DIR="$AGENT_DIR/trained_models"
ROS_DISTRO="${ROS_DISTRO:-humble}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
TURTLEBOT_SDF="/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"

PRETRAINED="${PRETRAINED_CKPT:-$MODELS_DIR/sac_actor_maze.pth}"
# Calibrated for i5-1135G7 (no GPU), env_step_sleep=0.05s:
#   Adaptive steps: avg episode ≈ 20-50 s depending on waypoint progress.
#   Phase1: 1000 ep × ~25s = ~7h
#   Phase2: 300 ep × 8 mini-sessions (random styles) × ~25s = ~17h
#   Phase3: 250 ep × 3 worlds × ~20s = ~4h
#   Total: ~28 h
PHASE1_EP="${PHASE1_EP:-1000}"
# Halved per-style episodes: replaced by 2 randomised passes over all 4 styles
# (8 mini-sessions) so the replay buffer mixes styles instead of saturating
# with one style for 500 episodes, which causes catastrophic forgetting.
PHASE2_EP="${PHASE2_EP:-300}"
PHASE3_EP="${PHASE3_EP:-250}"

START_PHASE=1
DRY_RUN=0
ONLY_WORLD=""
ONLY_PHASE=0

# ── Argument parsing ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --phase)  START_PHASE="$2"; shift 2 ;;
        --world)  ONLY_WORLD="$2";  shift 2 ;;
        --only)   ONLY_PHASE=1;     shift   ;;
        --dry-run) DRY_RUN=1;       shift   ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

# ── Helpers ───────────────────────────────────────────────────────────────────
_src() {
    # Outputs ready-to-eval source commands (no extra 'source' prefix needed).
    local s=""
    [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] \
        && s+="source '/opt/ros/${ROS_DISTRO}/setup.bash'; "
    [[ -f "$WS/install/setup.bash" ]] \
        && s+="source '$WS/install/setup.bash'; "
    printf '%s' "$s"
}

_banner() {
    echo ""
    echo "╔══════════════════════════════════════════════════════════════╗"
    printf  "║  %-60s  ║\n" "$1"
    echo "╚══════════════════════════════════════════════════════════════╝"
}

_run() {
    if [[ "$DRY_RUN" == "1" ]]; then
        echo "[DRY-RUN] $*"
    else
        eval "$*"
    fi
}

_kill_gazebo() {
    echo "  → Stopping Gazebo..."
    pkill -9 -f gzserver   2>/dev/null || true
    pkill -9 -f gzclient   2>/dev/null || true
    pkill -9 -f main_agent 2>/dev/null || true
    pkill -9 -f rviz2      2>/dev/null || true
    sleep 3
}

_start_gazebo() {
    local world="$1"
    echo "  → Launching Gazebo: $(basename "$world")"
    _run "$(_src) ros2 launch gazebo_ros gazebo.launch.py world:='$world' &"
    sleep 10
    echo "  → Spawning TurtleBot3..."
    _run "$(_src) ros2 run gazebo_ros spawn_entity.py \
            -timeout 60 -entity my_robot \
            -file '$TURTLEBOT_SDF' \
            -x -2.0 -y -2.0 -z 0.15 &"
    sleep 5
}

_train() {
    local world="$1" ckpt="$2" episodes="$3" label="$4"
    _kill_gazebo
    _start_gazebo "$world"

    local ckpt_arg=""
    [[ -f "$ckpt" ]] && ckpt_arg="--load-pretrained '$ckpt'"

    echo "  → Training: $label  ($episodes ep)"
    _run "$(_src) cd '$AGENT_DIR' && \
        python3 main_agent.py \
            --preset pfe_sac_waypoint \
            $ckpt_arg \
            --max-episodes '$episodes' \
            --reset-fire-and-forget \
            --reset-service-wait-sec 12"
}

_checkpoint_path() {
    # Always the live actor path; caller copies it before overwriting.
    echo "$MODELS_DIR/${ALGO_KEY}_actor_maze.pth"
}

_save_phase_ckpt() {
    local tag="$1"
    local wname="${2:-}"
    local src; src="$(_checkpoint_path)"
    if [[ -n "$wname" ]]; then
        local scoped; scoped="$(_scoped_best_ever_path "$wname")"
        if [[ -f "$scoped" ]]; then
            src="$scoped"
            echo "  → Phase checkpoint from scoped best_ever: $(basename "$scoped")"
        fi
    fi
    local dst="$MODELS_DIR/${ALGO_KEY}_actor_phase${tag}.pth"
    [[ "$DRY_RUN" == "1" ]] && echo "[DRY-RUN] cp $src $dst" && return
    cp "$src" "$dst" 2>/dev/null && echo "  → Checkpoint saved: $dst" || true
    # Keep live actor in sync with the best policy for this world.
    if [[ -n "$wname" && "$src" != "$(_checkpoint_path)" ]]; then
        cp "$src" "$(_checkpoint_path)" 2>/dev/null || true
    fi
}

# ── Phase 1: Core Maze Training (S-curve) ────────────────────────────────────
run_phase1() {
    _banner "PHASE 1 / 4 — Core Maze Training  ($PHASE1_EP episodes)"
    echo "  World  : S-Curve randomised maze (current_random_lab.world)"
    echo "  Goal   : Robot reliably reaches WP1 + WP2 ≥ 30 % of episodes"
    echo "  Shield : ON (no early termination on wall touch)"

    # Regenerate a fresh S-curve world (style 0)
    [[ "$DRY_RUN" == "0" ]] && python3 "$REGEN_SCRIPT" 0 || echo "[DRY-RUN] python3 $REGEN_SCRIPT 0"

    _train "$WORLDS_DIR/current_random_lab.world" \
           "$PRETRAINED" \
           "$PHASE1_EP" \
           "Phase1-CoreMaze"
    _save_phase_ckpt "1"
}

# ── Phase 2: Domain Randomisation  (all 4 styles, one per session) ────────────
run_phase2() {
    _banner "PHASE 2 / 4 — Domain Randomisation  ($PHASE2_EP episodes × 4 styles)"
    echo "  Cycles through all 4 obstacle styles to build a GENERAL policy."
    echo "  Goal : WP1+WP2 in ≥ 50 % of episodes across all styles"

    # Honour explicit PRETRAINED_CKPT env override first; only fall back to
    # phase1 checkpoint if no override was given.
    local ckpt
    if [[ -n "${PRETRAINED_CKPT:-}" && -f "${PRETRAINED_CKPT}" ]]; then
        ckpt="$PRETRAINED_CKPT"
        echo "  Using PRETRAINED_CKPT override: $ckpt"
    else
        ckpt="$MODELS_DIR/${ALGO_KEY}_actor_phase1.pth"
        [[ -f "$ckpt" ]] || ckpt="$PRETRAINED"
    fi

    for style in 1 2 3 0; do
        local label; label="Phase2-Style${style}"
        local style_names=( "S-Curve" "PillarForest" "BlockObstacles" "CorridorMaze" )
        local sname="${style_names[$style]}"
        _banner "  Phase 2 — Style ${style}: ${sname}"
        [[ "$DRY_RUN" == "0" ]] && python3 "$REGEN_SCRIPT" "$style" || echo "[DRY-RUN] regen style $style"
        _train "$WORLDS_DIR/current_random_lab.world" \
               "$ckpt" \
               "$PHASE2_EP" \
               "$label ($sname)"
        _save_phase_ckpt "2_s${style}"
        # Use the latest checkpoint for the next style
        ckpt="$MODELS_DIR/sac_actor_phase2_s${style}.pth"
        [[ -f "$ckpt" ]] || ckpt="$(_checkpoint_path)"
    done
}

# ── Phase 3: Fine-tuning on Eval Worlds ───────────────────────────────────────
run_phase3() {
    _banner "PHASE 3 / 4 — Fine-tuning on Eval Worlds  ($PHASE3_EP ep each)"
    echo "  Fine-tunes the policy in each eval world using only a few hundred"
    echo "  episodes — fast adaptation since the core policy is already strong."
    echo "  Goal : ≥ 60 % WP1+WP2 success in each eval world"

    # Warm-start: explicit PRETRAINED_CKPT / phase3_<world> when resuming one world,
    # else Phase 2 final → phase1 → default PRETRAINED.
    local ckpt=""
    if [[ -n "${PRETRAINED_CKPT:-}" && -f "$PRETRAINED_CKPT" ]]; then
        ckpt="$PRETRAINED_CKPT"
    elif [[ -n "$ONLY_WORLD" ]]; then
        local p3="$MODELS_DIR/${ALGO_KEY}_actor_phase3_${ONLY_WORLD}.pth"
        [[ -f "$p3" ]] && ckpt="$p3"
    fi
    [[ -n "$ckpt" && -f "$ckpt" ]] || ckpt="$MODELS_DIR/${ALGO_KEY}_actor_phase2_final.pth"
    [[ -f "$ckpt" ]] || ckpt="$MODELS_DIR/${ALGO_KEY}_actor_phase1.pth"
    [[ -f "$ckpt" ]] || ckpt="$PRETRAINED"
    echo "  → Warm-start checkpoint: $(basename "$ckpt")"

    declare -A EVAL_WORLDS=(
        [rabat]="$WORLDS_DIR/eval_rabat.world"
        [egypt]="$WORLDS_DIR/eval_egypt.world"
        [maze]="$WORLDS_DIR/eval_maze.world"
    )

    for wname in rabat egypt maze; do
        [[ -n "$ONLY_WORLD" && "$wname" != "$ONLY_WORLD" ]] && continue
        local wpath="${EVAL_WORLDS[$wname]}"
        if [[ ! -f "$wpath" ]]; then
            echo "  ⚠  $wpath not found — run: python3 sim_assets/scripts/generate_eval_worlds.py"
            continue
        fi
        _banner "  Phase 3 — Fine-tuning: $wname"
        local p3_extra; p3_extra="$(_phase3_train_extra "$wname")"
        _train "$wpath" "$ckpt" "$PHASE3_EP" "Phase3-$wname" "$p3_extra"
        _save_phase_ckpt "3_${wname}" "$wname"
    done
}

# ── Phase 4: Evaluation ───────────────────────────────────────────────────────
run_phase4() {
    _banner "PHASE 4 / 4 — Evaluation"
    echo "  Runs evaluate_agent.py in each world and prints success rates."
    echo "  Expected: ≥ 70 % WP1  ≥ 40 % WP2  ≥ 15 % WP3 (maze fully solved)"

    local ckpt; ckpt="$(_checkpoint_path)"

    declare -A EVAL_WORLDS=(
        [training]="$WORLDS_DIR/current_random_lab.world"
        [rabat]="$WORLDS_DIR/eval_rabat.world"
        [egypt]="$WORLDS_DIR/eval_egypt.world"
        [maze]="$WORLDS_DIR/eval_maze.world"
    )

    for wname in training rabat egypt maze; do
        local wpath="${EVAL_WORLDS[$wname]}"
        [[ ! -f "$wpath" ]] && echo "  ⚠  $wpath missing — skip" && continue
        _kill_gazebo
        _start_gazebo "$wpath"
        echo "  → Evaluating on $wname..."
        local wqe ckqe
        wqe="$(printf '%q' "$wpath")"
        ckqe="$(printf '%q' "$ckpt")"
        _run "$(_src) export PFE_WORLD=${wqe}; cd '$AGENT_DIR' && \
            python3 evaluate_agent.py \
                --algo ${ALGO_KEY} \
                --model ${ckqe} \
                --episodes 50 \
                --max-steps 4000 \
                --env-step-sleep-sec 0.05 \
                --waypoint-goal-radius 0.68 \
                --reset-fire-and-forget \
                --reset-service-wait-sec 45 \
                --tag waypoint_roadmap_${ALGO_KEY}_${wname} || true"
        _kill_gazebo
    done
}

# ── Main dispatch ─────────────────────────────────────────────────────────────
_banner "PFE Waypoint Training Roadmap"
echo "  Start phase : $START_PHASE"
echo "  Algorithm   : $ALGO_KEY  (preset $PRESET_WP; override with TRAIN_WAYPOINT_ALGO=td3)"
echo "  Pretrained  : $PRETRAINED"
echo "  Phases 1-4 cover ALL 4 world types → guaranteed generalisation"
echo ""

_run_if_phase() {
    local n="$1"
    local fn="$2"
    # Do not use `[[ … ]] && fn` here: with set -e, a false [[ ]] aborts the script.
    if [[ "$ONLY_PHASE" == "1" ]]; then
        if [[ "$START_PHASE" -eq "$n" ]]; then
            "$fn"
        fi
    else
        if [[ "$START_PHASE" -le "$n" ]]; then
            "$fn"
        fi
    fi
}

_run_if_phase 1 run_phase1
_run_if_phase 2 run_phase2
_run_if_phase 3 run_phase3
_run_if_phase 4 run_phase4

_banner "TRAINING COMPLETE"
echo "  Final checkpoint : $MODELS_DIR/${ALGO_KEY}_actor_maze.pth"
echo "  Phase checkpoints: $MODELS_DIR/${ALGO_KEY}_actor_phase*.pth"
