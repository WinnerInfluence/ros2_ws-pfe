#!/bin/bash
# =============================================================================
# train_menu.sh — Interactive DRL training & evaluation launcher
# Main: 1=Phase1  2=Phase2  3=Evaluate  4=TensorBoard  5=Stop sim  6=Exit
#
# Launch (any cwd is OK if you use bash with the full path):
#   bash /path/to/ros2_ws/src/safe_drl_nav/safe_drl_nav/train_menu.sh
# Or:
#   cd "$(dirname "$0")" && ./train_menu.sh
#
# Interactive menu fields (in order):
#   A — Algorithm     SAC | TD3 | Custom
#   B — Gazebo .world (sets PFE_WORLD; same layout training uses for that file)
#   C — Checkpoint    (Evaluate only) maze current | maze best-ever | adapt best-ever | custom
#   D — Episodes      (Evaluate only)
#   E — JSON tag      (Evaluate only) → pfe_logs/eval_<tag>.json
#   Thesis preset     (Evaluate) optional: --paper-eval (≥30 ep, strict /reset_simulation)
#
# Checkpoints: Phase 1 (adapt) → *_actor_adapt.pth ; Phase 2 (waypoints) → *_actor_maze.pth
# Session backups: main_agent.py copies existing .pth into trained_models/backups/ at startup
# (and on strict best_ever / periodic saves) — no shell-side backup step.
# =============================================================================
set -euo pipefail

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$SCRIPT_DIR/main_agent.py" ]]; then
    echo "ERROR: main_agent.py not found next to this script (SCRIPT_DIR=$SCRIPT_DIR)." >&2
    echo "  Fix: run  bash $SCRIPT_DIR/train_menu.sh   (do not copy train_menu.sh elsewhere)." >&2
    exit 1
fi
if [[ ! -f "$SCRIPT_DIR/pfe_gazebo_env.sh" ]]; then
    echo "ERROR: pfe_gazebo_env.sh missing in $SCRIPT_DIR" >&2
    exit 1
fi
# shellcheck disable=SC1091
source "$SCRIPT_DIR/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env

AGENT_SCRIPT="$SCRIPT_DIR/main_agent.py"
EVAL_SCRIPT="$SCRIPT_DIR/evaluate_agent.py"
GEN_WORLD_PY="$SCRIPT_DIR/sim_assets/scripts/generate_eval_world.py"   # legacy single-world
MODEL_DIR="$SCRIPT_DIR/trained_models"
LOG_DIR="$SCRIPT_DIR/pfe_logs"
WORLD_DIR="$SCRIPT_DIR/sim_assets/worlds"
WORLD_TRAINING="$WORLD_DIR/current_random_lab.world"
WORLD_EVAL="$WORLD_DIR/hassan_pyramid_eval.world"
WORLD_RABAT="$WORLD_DIR/eval_rabat.world"
WORLD_EGYPT="$WORLD_DIR/eval_egypt.world"
WORLD_MAZE_RUNNER="$WORLD_DIR/eval_maze.world"
GEN_WORLDS_PY="$SCRIPT_DIR/sim_assets/scripts/generate_eval_worlds.py"
TENSORBOARD_PORT=6006

# ── ROS / Robot ───────────────────────────────────────────────────────────────
ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"
TURTLEBOT_SDF="/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf"
SIM_ASSETS="$SCRIPT_DIR/sim_assets"

# ── Colours ───────────────────────────────────────────────────────────────────
R='\033[1;31m'; G='\033[1;32m'; Y='\033[1;33m'
B='\033[1;34m'; M='\033[1;35m'; C='\033[1;36m'
W='\033[1;37m'; D='\033[2m';    X='\033[0m'

# ── Runtime state (populated by sub-menu functions) ───────────────────────────
ALGO_KEY=""           # sac | td3 | custom
ALGO_MODEL=""         # maze checkpoint filename (evaluate / Phase 2 default)
ALGO_MODEL_ADAPT=""   # adapt-mode checkpoint (Phase 1 saves here)
ALGO_MODEL_MAZE=""    # waypoint-mode checkpoint (Phase 2 saves here)
ALGO_PRESET_P1=""     # flags for Phase 1
ALGO_PRESET_P2=""     # flags for Phase 2
WORLD_PATH=""         # absolute path to selected .world file
WORLD_LABEL=""        # short label used in JSON tag

# ── Header ────────────────────────────────────────────────────────────────────
_header() {
    clear 2>/dev/null || true
    echo -e "${C}${W}"
    echo "  ╔══════════════════════════════════════════════════════════╗"
    echo "  ║       safe_drl_nav  ·  DRL Training & Evaluation        ║"
    echo "  ║             SAC  ·  TD3  ·  Custom Research Slot        ║"
    echo "  ╚══════════════════════════════════════════════════════════╝"
    echo -e "${X}"
    echo -e "${D}  Agent  : $AGENT_SCRIPT"
    echo -e "  Models : $MODEL_DIR"
    echo -e "  Logs   : $LOG_DIR"
    echo -e "  Worlds : $WORLD_DIR${X}\n"
}

_menu() {
    echo -e "${W}  ┌─ Main Menu ────────────────────────────────────────────┐"
    echo -e "  │${X}  ${G}1)${X} Phase 1   — Adapt (random goal)                    ${W}│"
    echo -e "  │${X}     ${D}Checkpoint: <algo>_actor_adapt.pth  ·  prompts A → B${X}        ${W}│"
    echo -e "  │                                                      ${W}│"
    echo -e "  │${X}  ${G}2)${X} Phase 2   — Waypoints (WP1→WP2→WP3)               ${W}│"
    echo -e "  │${X}     ${D}CPU: device=cpu · sleep=0.03s · lr=1e-4 · replay-warmup 28k · WP r=0.68m · cap=2400 · early-stop${X} ${W}│"
    echo -e "  │${X}     ${D}prompts A → B${X}                                        ${W}│"
    echo -e "  │                                                      ${W}│"
    echo -e "  │${X}  ${B}3)${X} Evaluate  — N episodes → pfe_logs/eval_<tag>.json   ${W}│"
    echo -e "  │${X}     ${D}prompts A → C → B → D → E  ·  restarts Gazebo for chosen world${X} ${W}│"
    echo -e "  │                                                      ${W}│"
    echo -e "  │${X}  ${M}4)${X} TensorBoard — port ${TENSORBOARD_PORT}                      ${W}│"
    echo -e "  │                                                      ${W}│"
    echo -e "  │${X}  ${R}5)${X} Stop all  — kill Gazebo / ROS zombie processes      ${W}│"
    echo -e "  │                                                      ${W}│"
    echo -e "  │${X}  ${Y}6)${X} Exit                                              ${W}│"
    echo -e "  └──────────────────────────────────────────────────────┘${X}\n"
}

# ── Step A: Algorithm selection ───────────────────────────────────────────────
# Sets globals: ALGO_KEY  ALGO_MODEL_*  ALGO_PRESET_P1  ALGO_PRESET_P2
_select_algo() {
    echo -e "\n  ${W}Step A — Algorithm (checkpoints use this prefix)${X}"
    echo -e "    ${G}[1]${X} ${W}SAC${X}     ${D}recommended · Phase1→${G}sac_actor_adapt.pth${D}  Phase2→${G}sac_actor_maze.pth${X}"
    echo -e "    ${G}[2]${X} ${W}TD3${X}     ${D}Phase1→${G}td3_actor_adapt.pth${D}  Phase2→${G}td3_actor_maze.pth${X}"
    echo -e "    ${Y}[3]${X} ${W}Custom${X}  ${D}networks_custom.py + trainer_custom${X}\n"
    local ac
    read -r -p "  A — Algorithm [1-3, default=1]: " ac
    ac="${ac:-1}"
    case "$ac" in
        2)
            ALGO_KEY="td3"
            ALGO_MODEL_ADAPT="td3_actor_adapt.pth"
            ALGO_MODEL_MAZE="td3_actor_maze.pth"
            ALGO_MODEL="$ALGO_MODEL_MAZE"
            ALGO_PRESET_P1="--preset pfe_td3_adapt"
            ALGO_PRESET_P2="--preset pfe_td3_waypoint"
            ;;
        3)
            ALGO_KEY="custom"
            ALGO_MODEL_ADAPT="custom_actor_adapt.pth"
            ALGO_MODEL_MAZE="custom_actor_maze.pth"
            ALGO_MODEL="$ALGO_MODEL_MAZE"
            ALGO_PRESET_P1="--algo custom"
            ALGO_PRESET_P2="--algo custom --waypoint-mode"
            ;;
        *)
            ALGO_KEY="sac"
            ALGO_MODEL_ADAPT="sac_actor_adapt.pth"
            ALGO_MODEL_MAZE="sac_actor_maze.pth"
            ALGO_MODEL="$ALGO_MODEL_MAZE"
            ALGO_PRESET_P1="--preset pfe_sac_adapt"
            ALGO_PRESET_P2="--preset pfe_sac_waypoint"
            ;;
    esac
    echo -e "\n  ${G}▶${X} Algorithm ${W}${ALGO_KEY^^}${X}  │  adapt: ${D}${ALGO_MODEL_ADAPT}${X}  │  maze: ${D}${ALGO_MODEL_MAZE}${X}\n"
}

# ── Step B: Gazebo world (.world file loaded after stale sim is killed) ───────
# Sets globals: WORLD_PATH  WORLD_LABEL  exports PFE_WORLD
_select_world() {
    echo -e "\n  ${W}Step B — Gazebo world file (this exact map is loaded next)${X}"
    echo -e "    ${G}[1]${X} Training lab      ${D}current_random_lab.world${X}  ${Y}(default; same family as Phase 1/2 training)${X}"
    echo -e "    ${C}[2]${X} Rabat eval        ${D}eval_rabat.world${X}"
    echo -e "    ${Y}[3]${X} Egypt eval        ${D}eval_egypt.world${X}"
    echo -e "    ${M}[4]${X} Maze-runner eval  ${D}eval_maze.world${X}  ${D}(green hedge labyrinth)${X}\n"
    local wc ng=0
    read -r -p "  B — World [1-4, default=1]: " wc
    wc="${wc:-1}"
    case "$wc" in
        2) WORLD_PATH="$WORLD_RABAT";       WORLD_LABEL="rabat";       ng=1 ;;
        3) WORLD_PATH="$WORLD_EGYPT";       WORLD_LABEL="egypt";       ng=1 ;;
        4) WORLD_PATH="$WORLD_MAZE_RUNNER"; WORLD_LABEL="maze_runner"; ng=1 ;;
        *) WORLD_PATH="$WORLD_TRAINING";    WORLD_LABEL="training" ;;
    esac
    export PFE_WORLD="$WORLD_PATH"
    if [[ $ng -eq 1 && ! -f "$WORLD_PATH" ]]; then
        echo -e "\n  ${Y}World file not found — generating all eval worlds...${X}"
        python3 "$GEN_WORLDS_PY"
    fi
    echo -e "\n  ${G}▶${X} ${W}PFE_WORLD${X} = ${D}${PFE_WORLD}${X}\n"
}

# ── Internal helpers ───────────────────────────────────────────────────────────
_src() {
    local s=""
    [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] \
        && s+="source '/opt/ros/${ROS_DISTRO}/setup.bash'; "
    [[ -f "$WS_PATH/install/setup.bash" ]] \
        && s+="source '$WS_PATH/install/setup.bash'; "
    printf '%s' "$s"
}

_term() {
    local title="$1" cmd="$2"
    local full_cmd; full_cmd="$(pfe_term_env_prefix)$cmd"
    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --geometry=110x35 --title="$title" -- bash -c "$full_cmd; exec bash"
    else
        echo -e "${Y}  gnome-terminal not found — running inline.${X}"
        bash -c "$full_cmd"
    fi
}

_kill_stale() {
    echo -e "  ${R}Cleaning stale processes...${X}"
    local procs=(gzserver gzclient rviz2 main_agent.py evaluate_agent.py hot_swap_eval_node.py)
    local p
    for p in "${procs[@]}"; do pkill -9 -f "$p" 2>/dev/null || true; done
    sleep 2
    local _i
    for ((_i = 0; _i < 20; _i++)); do
        pgrep -f gzserver >/dev/null 2>&1 || break
        pkill -9 -f gzserver 2>/dev/null || true
        sleep 1
    done
    sleep 1
}

# Launch Gazebo + spawner + RViz. Requires PFE_WORLD to be exported already.
# Optional $1 = extra seconds to wait after gz starts (eval uses 14, training 18).
_launch_sim() {
    local post_sleep="${1:-10}"
    local src gz_gui world_abs wq
    src="$(_src)"
    gz_gui="$(pfe_gazebo_gui_suffix)"
    world_abs="${PFE_WORLD}"
    if [[ -f "$world_abs" ]]; then
        world_abs="$(readlink -f "$world_abs" 2>/dev/null || realpath "$world_abs" 2>/dev/null || echo "$world_abs")"
    fi
    wq="$(printf '%q' "$world_abs")"
    export PFE_WORLD="$world_abs"
    echo -e "  ${C}Launching Gazebo → ${world_abs}${X}"
    if [[ "${PFE_GAZEBO_STABLE:-1}" == "1" && "${PFE_GAZEBO_GUI:-1}" == "1" ]]; then
        gnome-terminal --title="GAZEBO_CORE" -- bash -c \
            "$(pfe_term_env_prefix)$src export PFE_WORLD=$wq; $(pfe_gazebo_stable_one_terminal_cmd "$wq" "${post_sleep}")"
    else
        gnome-terminal --title="GAZEBO_CORE" -- bash -c \
            "$(pfe_term_env_prefix)$src export PFE_WORLD=$wq; echo \"[GAZEBO] world=\$PFE_WORLD\"; ros2 launch gazebo_ros gazebo.launch.py world:=\$PFE_WORLD$gz_gui; exec bash"
    fi
    sleep "${post_sleep}"

    local move_py=""
    for loc in "$SIM_ASSETS/scripts/move_enemy.py" "$HOME/Desktop/move_enemy.py"; do
        [[ -f "$loc" ]] && move_py="$loc" && break
    done

    local spawn_cmd
    spawn_cmd="$(pfe_term_env_prefix)source '/opt/ros/${ROS_DISTRO}/setup.bash'"
    spawn_cmd+="; ros2 run gazebo_ros spawn_entity.py -timeout 120"
    spawn_cmd+=" -entity my_robot -file '$TURTLEBOT_SDF'"
    spawn_cmd+=" -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z; sleep 2"
    [[ -n "$move_py" ]] && spawn_cmd+="; python3 '$move_py' 2>/dev/null"
    spawn_cmd+="; exec bash"
    gnome-terminal --title="SPAWNER" -- bash -c "$spawn_cmd"

    gnome-terminal --title="RVIZ_MONITOR" -- bash -c \
        "$(pfe_term_env_prefix)source '/opt/ros/${ROS_DISTRO}/setup.bash'; \
rviz2 --ros-args -p use_sim_time:=true; exec bash"
}

_wait_reset_service() {
    local max_sec="${1:-90}" src
    src="$(_src)"
    echo -e "  ${D}Waiting up to ${max_sec}s for /reset_simulation (Gazebo must finish loading)...${X}"
    if bash -c "${src}for ((_i=0; _i<${max_sec}; _i+=2)); do
        ros2 service list 2>/dev/null | grep -q '/reset_simulation' && exit 0
        sleep 2
    done
    exit 1"; then
        echo -e "  ${G}▶ /reset_simulation is up.${X}"
        return 0
    fi
    echo -e "  ${Y}⚠  /reset_simulation not seen yet — brain will wait up to 45s per episode.${X}"
    return 1
}

_check_model() {
    local model_path="$MODEL_DIR/$ALGO_MODEL"
    if [[ ! -f "$model_path" ]]; then
        echo -e "\n  ${R}⚠  Pre-trained model not found:${X}"
        echo -e "  ${R}   $model_path${X}\n"
        echo -e "  ${Y}Run Option 1 (Phase 1) with ${ALGO_KEY^^} first.${X}\n"
        local yn
        read -r -p "  Continue anyway? [y/N]: " yn
        [[ "${yn,,}" == "y" ]] || return 1
    fi
    return 0
}

# ── Option 1: Phase 1 — Train from Scratch ────────────────────────────────────
do_phase1() {
    echo -e "\n${G}${W}► Phase 1 — Adapt (random goal)${X}"
    echo -e "  ${D}Flow: Step A → Step B · Checkpoints: <algo>_actor_adapt.pth · backups in main_agent${X}\n"
    _select_algo
    echo -e "\n  ${G}▶${X} Phase 1 algorithm: ${W}${ALGO_KEY^^}${X}  ${D}(${ALGO_PRESET_P1})${X}\n"
    _select_world

    local src; src="$(_src)"
    local brain_title="DRL_P1_${ALGO_KEY^^}"
    local brain_cmd
    brain_cmd="${src}$(pfe_training_cpu_math_env)cd '$SCRIPT_DIR' && python3 main_agent.py \
${ALGO_PRESET_P1} \
--use-shield \
--reset-fire-and-forget \
--reset-service-wait-sec 45"

    _kill_stale
    _launch_sim 18
    _wait_reset_service 90
    echo -e "  ${G}Launching brain: ${brain_title}${X}"
    echo -e "  ${D}Writes: ${ALGO_MODEL_ADAPT}  (TensorBoard: tb_${ALGO_KEY}_adapt) · backups: autonomous in main_agent${X}"
    _term "$brain_title" "$brain_cmd"
    echo -e "\n${G}Stack launched.${X}  World: $PFE_WORLD\n"
}

# ── Option 2: Phase 2 — Curriculum Waypoint Training ──────────────────────────
do_phase2() {
    echo -e "\n${C}${W}► Phase 2 — Waypoint curriculum (WP1→WP2→WP3)${X}"
    echo -e "  ${D}Flow: … ${G}device cpu${D} · ${G}sleep 0.03s${D} · ${G}base 1500${D} · ${G}cap 2400${D} · ${G}lr 1e-4${D} · ${G}replay-warmup 28k${D} · ${G}WP clear r=0.68m${D} · Writes: <algo>_actor_maze.pth${X}"
    echo -e "  ${D}Early-stop: ${G}10 consecutive${D} solves for lab (thorough), ${G}3 consecutive${D} for world fine-tune (fast transfer)${X}\n"
    _select_algo
    echo -e "\n  ${G}▶${X} Phase 2 algorithm: ${W}${ALGO_KEY^^}${X}  ${D}(${ALGO_PRESET_P2})${X}\n"

    # Warm-start priority:
    #   adapt best → adapt current → maze best → maze current → (last resort) true cold
    local best_adapt="$MODEL_DIR/${ALGO_KEY}_actor_adapt_best_ever.pth"
    local curr_adapt="$MODEL_DIR/${ALGO_KEY}_actor_adapt.pth"
    local best_maze="$MODEL_DIR/${ALGO_KEY}_actor_maze_best_ever.pth"
    local curr_maze="$MODEL_DIR/$ALGO_MODEL_MAZE"
    local pretrain_path=""

    if [[ -f "$best_adapt" ]]; then
        pretrain_path="$best_adapt"
        echo -e "\n  ${G}▶${X} Warm-start: ${W}$(basename "$best_adapt")${X}  ${Y}(best Phase 1 adapt)${X}"
    elif [[ -f "$curr_adapt" ]]; then
        pretrain_path="$curr_adapt"
        echo -e "\n  ${Y}▶ Warm-start: $(basename "$curr_adapt")  (Phase 1 adapt, no best_ever yet).${X}"
    elif [[ -f "$best_maze" ]]; then
        pretrain_path="$best_maze"
        echo -e "\n  ${G}▶${X} No adapt checkpoint — warm-start from ${W}$(basename "$best_maze")${X}"
        echo -e "  ${D}(You still resume ${ALGO_MODEL_MAZE} in the agent; --load-pretrained aligns actor to peak maze.)${X}"
    elif [[ -f "$curr_maze" ]]; then
        pretrain_path="$curr_maze"
        echo -e "\n  ${Y}▶${X} No adapt checkpoint — warm-start from current ${W}$(basename "$curr_maze")${X}"
    else
        echo -e "\n  ${R}No adapt AND no maze .pth — Phase 2 would start from random weights.${X}"
        echo -e "  ${D}Run Phase 1 (adapt) or Phase 2 once to create checkpoints, or cancel.${X}"
        read -r -p "  Continue with random init anyway? [y/N]: " yn
        [[ "${yn,,}" == "y" ]] || return 0
    fi

    _select_world

    # ── Step F — early-stop threshold ─────────────────────────────────────────
    echo -e "\n  ${W}Step F — Early-stop threshold (consecutive full maze solves)${X}"
    echo -e "    ${G}[1]${X} ${W}10 consecutive${X}  ${D}thorough — lab world main training (overnight)${X}"
    echo -e "    ${G}[2]${X} ${W} 5 consecutive${X}  ${D}balanced${X}"
    echo -e "    ${G}[3]${X} ${W} 3 consecutive${X}  ${D}fast transfer — world fine-tuning sessions${X}"
    echo -e "    ${G}[4]${X} ${W} 0 (disabled)${X}   ${D}run until you Ctrl+C${X}\n"
    local fc
    read -r -p "  F — Stop threshold [1-4, default=1]: " fc
    fc="${fc:-1}"
    local early_stop_n
    case "$fc" in
        2) early_stop_n=5 ;;
        3) early_stop_n=3 ;;
        4) early_stop_n=0 ;;
        *) early_stop_n=10 ;;
    esac
    echo -e "\n  ${G}▶${X} Early-stop: ${W}${early_stop_n}${X} consecutive maze solves${X}\n"

    local src; src="$(_src)"
    local brain_title="DRL_P2_${ALGO_KEY^^}_WP"
    local pretrain_flag=""
    [[ -n "$pretrain_path" ]] && pretrain_flag="--load-pretrained '${pretrain_path}'"
    local brain_cmd
    # CPU: explicit --device cpu. 0.03s sleep ≈ 40% less wall time vs preset 0.05; raise to 0.05 if Gazebo lags.
    brain_cmd="${src}$(pfe_training_cpu_math_env)cd '$SCRIPT_DIR' && python3 main_agent.py \
${ALGO_PRESET_P2} \
${pretrain_flag} \
--device cpu \
--env-step-sleep-sec 0.03 \
--adaptive-base-steps 1500 \
--max-episode-steps 2400 \
--lr 1e-4 \
--replay-warmup-steps 28000 \
--waypoint-goal-radius 0.68 \
--early-stop-consecutive-maze-solves ${early_stop_n} \
--reset-fire-and-forget \
--reset-service-wait-sec 45"

    _kill_stale
    _launch_sim 18
    _wait_reset_service 90
    echo -e "  ${G}Launching brain: ${brain_title}${X}"
    [[ -n "$pretrain_path" ]] && echo -e "  ${D}Warm-start: $pretrain_path${X}"
    echo -e "  ${D}Writes: ${ALGO_MODEL_MAZE}  (TensorBoard: tb_${ALGO_KEY}_maze) · backups: autonomous in main_agent${X}"
    _term "$brain_title" "$brain_cmd"
    echo -e "\n${G}Stack launched.${X}  World: $PFE_WORLD\n"
}

# ── Option 3: Evaluate — Benchmark harness ────────────────────────────────────
do_evaluate() {
    echo -e "\n${B}${W}► Evaluate — benchmark (shield ON by default in script)${X}"
    echo -e "  ${D}Flow: A → C → B → thesis preset? → D → E · Kills Gazebo, relaunches B, then EVAL.${X}"
    echo -e "  ${D}Eval defaults: env_step_sleep=0.05 s (training-like), max_steps=4000 (long horizon for full maze).${X}"
    echo -e "  ${D}Thesis preset: ${G}--paper-eval${D} (≥30 ep, abort if reset never ready). Fast batch: --env-step-sleep-sec 0.${X}"
    echo -e "  ${D}Checkpoint: C→2 maze best-ever on B→1 lab before harder worlds / TD3.${X}\n"
    _select_algo

    local default_ckpt="$MODEL_DIR/$ALGO_MODEL_MAZE"
    local best_maze="$MODEL_DIR/${ALGO_KEY}_actor_maze_best_ever.pth"
    local best_adapt="$MODEL_DIR/${ALGO_KEY}_actor_adapt_best_ever.pth"
    local maze_peak="$MODEL_DIR/sac_actor_maze_best_ever_eval_maze.pth"

    echo -e "\n  ${W}Step C — Actor checkpoint (.pth)${X}"
    echo -e "    ${G}[1]${X} Current waypoint run   ${D}${ALGO_MODEL_MAZE}${X}"
    if [[ -f "$best_maze" ]]; then
        echo -e "    ${G}[2]${X} Lab best-ever (24/33)  ${D}$(basename "$best_maze")${X}  ${Y}recommended lab${X}"
    else
        echo -e "    ${D}[2] Lab best-ever missing — finish Phase 2 or pick [1].${X}"
    fi
    if [[ -f "$maze_peak" ]]; then
        echo -e "    ${M}[3]${X} Maze peak (ep6)        ${D}$(basename "$maze_peak")${X}  ${Y}video / eval_maze${X}"
    fi
    if [[ -f "$best_adapt" ]]; then
        echo -e "    ${C}[4]${X} Adapt best-ever        ${D}$(basename "$best_adapt")${X}  ${D}(Phase 1 only — weak on WP3)${X}"
    else
        echo -e "    ${D}[4] Adapt best-ever missing — run Phase 1 first.${X}"
    fi
    echo -e "    ${G}[5]${X} Custom path (absolute path, or filename under trained_models/)\n"

    local ckdef="1"
    [[ -f "$best_maze" ]] && ckdef="2"
    read -r -p "  C — Checkpoint [1-5, default=${ckdef}]: " ckchoice
    ckchoice="${ckchoice:-$ckdef}"

    local model_path=""
    case "$ckchoice" in
        2)
            if [[ -f "$best_maze" ]]; then model_path="$best_maze"
            else echo -e "  ${Y}Lab best-ever missing — falling back to current save.${X}"; model_path="$default_ckpt"
            fi ;;
        3)
            if [[ -f "$maze_peak" ]]; then model_path="$maze_peak"
            else echo -e "  ${Y}Maze peak missing — falling back to lab best-ever.${X}"
                model_path="${best_maze:-$default_ckpt}"
            fi ;;
        4)
            if [[ -f "$best_adapt" ]]; then model_path="$best_adapt"
            else echo -e "  ${Y}Adapt best-ever missing — falling back to current save.${X}"; model_path="$default_ckpt"
            fi ;;
        5)
            read -r -p "  C — Path to actor .pth: " model_path
            model_path="${model_path/#\~/$HOME}"
            [[ "$model_path" != /* ]] && model_path="$MODEL_DIR/$model_path"
            ;;
        *)
            model_path="$default_ckpt"
            ;;
    esac

    if [[ ! -f "$model_path" ]]; then
        echo -e "  ${R}Model not found: ${model_path}${X}"
        echo -e "  ${Y}Train Phase 1/2 first, or pick another checkpoint.${X}\n"
        return
    fi
    echo -e "\n  ${G}▶${X} Model: ${W}${model_path}${X}\n"

    _select_world

    echo -e "\n  ${W}Thesis-grade eval preset${X}"
    echo -e "    ${D}Passes ${G}--paper-eval${D}: at least 30 episodes; exits if ${G}/reset_simulation${D} never becomes ready.${X}"
    read -r -p "  Use thesis-grade preset? [Y/n]: " paper_yn
    paper_yn="${paper_yn:-Y}"
    local paper_flag="" ep_def=20
    if [[ "${paper_yn,,}" != "n" ]]; then
        paper_flag="--paper-eval"
        ep_def=30
    fi

    local n_ep
    read -r -p "  D — Number of evaluation episodes [default=${ep_def}]: " n_ep
    n_ep="${n_ep:-$ep_def}"
    [[ "$n_ep" =~ ^[0-9]+$ ]] && (( n_ep >= 1 )) || n_ep="$ep_def"
    if [[ -n "$paper_flag" ]] && (( n_ep < 30 )); then
        echo -e "  ${Y}Thesis preset: episodes ${n_ep} < 30 — using 30 for meaningful variance.${X}"
        n_ep=30
    fi

    local tag
    read -r -p "  E — JSON output tag → pfe_logs/eval_<tag>.json [default=${ALGO_KEY}_${WORLD_LABEL}]: " tag
    tag="${tag:-${ALGO_KEY}_${WORLD_LABEL}}"

    local wqe
    wqe="$(printf '%q' "$PFE_WORLD")"
    local src; src="$(_src)"
    local eval_title="EVAL_${ALGO_KEY^^}"
    local eval_cmd
    eval_cmd="${src}export PFE_WORLD=${wqe}; cd '$SCRIPT_DIR' && python3 evaluate_agent.py \
--algo '${ALGO_KEY}' \
--model '${model_path}' \
--episodes ${n_ep} \
--max-steps 4000 \
--env-step-sleep-sec 0.05 \
--waypoint-goal-radius 0.68 \
${paper_flag} \
--tag '${tag}'"

    _kill_stale
    _launch_sim 14
    echo -e "  ${Y}Verify Gazebo terminal shows:${X}  ${D}[GAZEBO] world=…${X}  ${Y}same path as Step B above.${X}"
    echo -e "  ${B}Launching evaluator: ${eval_title}${X}"
    local _pe=""
    [[ -n "$paper_flag" ]] && _pe=" ${paper_flag}"
    echo -e "  ${D}python3 evaluate_agent.py --algo ${ALGO_KEY} --episodes ${n_ep} --max-steps 4000 --env-step-sleep-sec 0.05 --waypoint-goal-radius 0.68${_pe} --tag ${tag}${X}\n"
    _term "$eval_title" "$eval_cmd"
    echo -e "${G}Evaluation launched.${X}  Results → ${D}${LOG_DIR}/eval_${tag}.json${X}\n"
}

# ── Option 4: TensorBoard ─────────────────────────────────────────────────────
do_tensorboard() {
    echo -e "\n${M}► TensorBoard on port ${TENSORBOARD_PORT}${X}"
    if ! command -v tensorboard &>/dev/null; then
        echo -e "${R}  tensorboard not found.${X}  Install: pip install tensorboard\n"
        return
    fi
    if [[ ! -d "$LOG_DIR" ]]; then
        echo -e "${Y}  Log dir not found: ${LOG_DIR}${X}"
        echo -e "${Y}  Start a training run first.${X}\n"
        return
    fi
    _term "TENSORBOARD" "tensorboard --logdir='$LOG_DIR' --port=${TENSORBOARD_PORT}"
    echo -e "${G}  Launched.${X}  Open: ${W}http://localhost:${TENSORBOARD_PORT}${X}\n"
}

# ── Option 5: Stop all simulation processes ───────────────────────────────────
do_stop_all() {
    echo -e "\n${R}► Stopping all Gazebo / ROS zombie processes...${X}\n"
    local killed=0
    for proc in gzserver gzclient rviz2 main_agent.py evaluate_agent.py rosmaster ros2; do
        if pgrep -f "$proc" &>/dev/null; then
            echo -e "  ${R}Killing:${X} $proc"
            pkill -9 -f "$proc" 2>/dev/null && (( killed++ )) || true
        else
            echo -e "  ${D}Not running:${X} $proc"
        fi
    done
    echo
    sleep 1
    fuser -k 11311/tcp 2>/dev/null || true
    if (( killed > 0 )); then
        echo -e "${G}  Terminated ${killed} process group(s). RAM freed.${X}\n"
    else
        echo -e "${D}  Nothing to kill — already clean.${X}\n"
    fi
}

# ── Main loop ──────────────────────────────────────────────────────────────────
main() {
    local choice="${1:-}"
    while true; do
        _header
        _menu

        if [[ -z "$choice" ]]; then
            read -r -p "  Main menu — enter option [1-6]: " choice
        fi
        echo

        case "$choice" in
            1) do_phase1      ;;
            2) do_phase2      ;;
            3) do_evaluate    ;;
            4) do_tensorboard ;;
            5) do_stop_all   ;;
            6)
                echo -e "${Y}  Goodbye.${X}\n"
                exit 0
                ;;
            *)
                echo -e "${R}  Invalid option '${choice}'.${X}  ${D}Enter 1–6 (see menu above).${X}\n"
                sleep 1
                ;;
        esac

        # Non-interactive mode (CLI arg): exit after one action.
        [[ -n "${1:-}" ]] && exit 0

        choice=""
        echo -e "${D}  Press Enter to return to menu...${X}"
        read -r
    done
}

main "$@"
