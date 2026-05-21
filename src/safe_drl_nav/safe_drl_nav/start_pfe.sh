#!/usr/bin/env bash
# =============================================================================
# start_pfe.sh — Launch Gazebo + robot spawn + RViz + DRL brain
#
# World resolution (first non-empty wins):
#   1. $PFE_WORLD          (env var — set by train_menu.sh or manually)
#   2. sim_assets/worlds/current_random_lab.world  (default training world)
#
# Usage:
#   bash start_pfe.sh                          # interactive menu
#   bash start_pfe.sh 1                        # SAC adapt, no prompt
#   PFE_WORLD=/path/to.world bash start_pfe.sh 1
#   bash start_pfe.sh 1 -- --use-shield        # extra flags → main_agent.py
#   PFE_NO_SIM_RESET=1 bash start_pfe.sh 1    # brain only, skip Gazebo
#   PFE_GAZEBO_GUI=0 bash start_pfe.sh 1     # headless gzserver if gzclient freezes
#   PFE_GAZEBO_STABLE=0 …                    # disable QT_X11_NO_MITSHM workaround
# =============================================================================
set -eo pipefail

# ── Defaults ──────────────────────────────────────────────────────────────────
ROS_DISTRO="${ROS_DISTRO:-humble}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="${AGENT_DIR:-$WS_PATH/src/safe_drl_nav/safe_drl_nav}"
[[ -f "$SCRIPT_DIR/main_agent.py" ]] && AGENT_DIR="$SCRIPT_DIR"

SIM_ASSETS="$AGENT_DIR/sim_assets"
_DEFAULT_WORLD="$SIM_ASSETS/worlds/current_random_lab.world"

# Respect externally set PFE_WORLD; fall back to default training world
PFE_WORLD="${PFE_WORLD:-$_DEFAULT_WORLD}"

SEED="${PFE_TRAIN_SEED:-42}"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"
TURTLEBOT_SDF="${TURTLEBOT_SDF:-/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf}"

EXTRA_BRAIN_ARGS=()
CHOICE_FROM_CLI=""

# ── Colours ───────────────────────────────────────────────────────────────────
R='\033[1;31m'; G='\033[1;32m'; Y='\033[1;33m'
B='\033[1;34m'; M='\033[1;35m'; C='\033[1;36m'
W='\033[1;37m'; D='\033[2m';    X='\033[0m'

# ── Helpers ───────────────────────────────────────────────────────────────────
_die() { echo -e "${R}error:${X} $*" >&2; exit 1; }
_q()   { printf '%q' "$1"; }

_check_ros() {
    [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] \
        || _die "ROS ${ROS_DISTRO} not found. Set ROS_DISTRO correctly."
}

_src() {
    local s="source $(_q "/opt/ros/${ROS_DISTRO}/setup.bash")"
    [[ -f "$WS_PATH/install/setup.bash" ]] \
        && s="$s; source $(_q "$WS_PATH/install/setup.bash")"
    printf '%s' "$s"
}

_parse_cli() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --)
                shift
                while [[ $# -gt 0 ]]; do EXTRA_BRAIN_ARGS+=("$1"); shift; done
                return 0
                ;;
            *)
                if [[ -z "$CHOICE_FROM_CLI" && "$1" =~ ^[0-9]+$ ]]; then
                    CHOICE_FROM_CLI="$1"
                else
                    echo "error: unexpected arg '$1'  (use: $0 [N] [-- brain_args...])" >&2
                    exit 2
                fi
                shift
                ;;
        esac
    done
}

# ── Kill stale processes before a fresh launch ────────────────────────────────
_kill_stale() {
    local procs=(gzserver gzclient rviz2 main_agent.py evaluate_agent.py hot_swap_eval_node.py hot_swap_eval)
    for proc in "${procs[@]}"; do
        pkill -9 -f "$proc" 2>/dev/null || true
    done
    sleep 2
}

# ── Build the brain command string ────────────────────────────────────────────
_brain_cmd() {
    local -a a=(python3 main_agent.py)
    [[ -n "${TRAIN_PRESET:-}" ]] \
        && a+=(--preset "$TRAIN_PRESET" --train-seed "$SEED") \
        || a+=(--algo "${TRAIN_ALGO:-sac}")
    if [[ "${PFE_NO_SIM_RESET:-0}" == "1" ]]; then
        a+=(--no-sim-reset)
    else
        a+=(--reset-fire-and-forget \
            --reset-service-wait-sec "${PFE_RESET_SERVICE_WAIT:-12}")
    fi
    [[ -n "${PFE_ENV_STEP_SLEEP:-}" ]] \
        && a+=(--env-step-sleep-sec "$PFE_ENV_STEP_SLEEP")
    [[ "${TRAIN_USE_SHIELD:-0}" == "1" ]] && a+=(--use-shield)
    a+=("${EXTRA_BRAIN_ARGS[@]}")
    local out="" part
    for part in "${a[@]}"; do out="${out:+$out }$(_q "$part")"; done
    printf '%s' "$out"
}

# ── Main launch sequence ───────────────────────────────────────────────────────
_run_stack() {
    TRAIN_PRESET="${TRAIN_PRESET:-}"
    TRAIN_USE_SHIELD="${TRAIN_USE_SHIELD:-0}"
    [[ -z "$TRAIN_PRESET" ]] && TRAIN_ALGO="${TRAIN_ALGO:-sac}"

    local tag="${TRAIN_PRESET:-${TRAIN_ALGO}}"
    [[ "$TRAIN_USE_SHIELD" == "1" ]] && tag="${tag}_SHIELD"
    local brain_title="DRL_BRAIN_${tag^^}"

    echo "── [1/4] Killing stale Gazebo / ROS processes ────────────────"
    _kill_stale

    echo "── [2/4] Randomising world (training mode only) ──────────────"
    if [[ "$PFE_WORLD" == "$_DEFAULT_WORLD" ]]; then
        local rnd_script=""
        for loc in \
            "$SIM_ASSETS/scripts/randomize_world.py" \
            "$HOME/Desktop/randomize_world.py" \
            "$AGENT_DIR/randomize_world.py"; do
            [[ -f "$loc" ]] && rnd_script="$loc" && break
        done
        [[ -n "$rnd_script" ]] \
            && python3 "$rnd_script" \
            || echo "  randomize_world.py not found — skipping."
    else
        echo "  Eval world selected — skipping randomisation."
    fi

    echo "── [3/4] Launching Gazebo ────────────────────────────────────"
    echo "  World: $PFE_WORLD"
    local src gz_gui; src="$(_src)" gz_gui="$(pfe_gazebo_gui_suffix)"
    gnome-terminal --title="GAZEBO_CORE" -- bash -c \
        "$(pfe_term_env_prefix)$src; ros2 launch gazebo_ros gazebo.launch.py \
world:=$(_q "$PFE_WORLD")$gz_gui; exec bash"
    sleep 8

    echo "── [4/4] Spawning robot · RViz · DRL brain ───────────────────"
    local move_py=""
    for loc in "$SIM_ASSETS/scripts/move_enemy.py" "$HOME/Desktop/move_enemy.py"; do
        [[ -f "$loc" ]] && move_py="$loc" && break
    done

    local spawn_cmd
    spawn_cmd="$(pfe_term_env_prefix)source $(_q "/opt/ros/${ROS_DISTRO}/setup.bash")"
    spawn_cmd+="; ros2 run gazebo_ros spawn_entity.py -timeout 120"
    spawn_cmd+=" -entity my_robot"
    spawn_cmd+=" -file $(_q "$TURTLEBOT_SDF")"
    spawn_cmd+=" -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z"
    spawn_cmd+="; sleep 2"
    [[ -n "$move_py" ]] && spawn_cmd+="; python3 $(_q "$move_py") 2>/dev/null"
    spawn_cmd+="; exec bash"
    gnome-terminal --title="SPAWNER" -- bash -c "$spawn_cmd"

    gnome-terminal --title="RVIZ_MONITOR" -- bash -c \
        "$(pfe_term_env_prefix)source $(_q "/opt/ros/${ROS_DISTRO}/setup.bash"); \
rviz2 --ros-args -p use_sim_time:=true; exec bash"

    local bcmd; bcmd="$(_brain_cmd)"
    gnome-terminal --geometry=90x30 --title="$brain_title" -- bash -c \
        "$(pfe_term_env_prefix)$src; cd $(_q "$AGENT_DIR"); eval $bcmd; exec bash"

    echo "══ BOOT COMPLETE ════════════════════════════════════════════"
    echo "  World : $PFE_WORLD"
    echo "  Brain : $brain_title"
    echo "  Seed  : $SEED"
}

# ── Helper: open a dedicated gnome-terminal and run train_waypoint.sh phase ───
_phase_terminal() {
    local title="$1" phase="$2" extra="${3:-}"
    local src; src="$(_src)"
    gnome-terminal --geometry=100x35 --title="$title" -- bash -c \
        "$(pfe_term_env_prefix)$src; cd $(_q "$AGENT_DIR") && bash train_waypoint.sh --phase $phase $extra; exec bash"
}

# ── Interactive menu ───────────────────────────────────────────────────────────
_menu() {
    local choice="" ok=0
    while [[ $ok -eq 0 ]]; do
        clear 2>/dev/null || true
        local ckpt_file="$AGENT_DIR/trained_models/sac_actor_maze.pth"
        local ckpt_label="${D}none — will train from scratch${X}"
        [[ -f "$ckpt_file" ]] && ckpt_label="${G}$(basename "$ckpt_file")${X}  ${D}($(stat -c%y "$ckpt_file" | cut -d' ' -f1,2 | cut -d'.' -f1))${X}"
        echo -e "${C}${W}"
        echo -e "  ╔══════════════════════════════════════════════════════════╗"
        echo -e "  ║        safe_drl_nav  ·  DRL Training Launcher           ║"
        echo -e "  ║           SAC  ·  TD3  ·  Waypoint  ·  DR               ║"
        echo -e "  ╚══════════════════════════════════════════════════════════╝${X}"
        echo -e "${D}  World : $(basename "$PFE_WORLD")"
        echo -e "  Ckpt  : ${X}$ckpt_label"
        echo ""
        echo -e "${W}  ┌─ Standard Training ────────────────────────────────────┐"
        echo -e "  │${X}  ${G}1)${X} SAC adapt   ${D}(pfe_sac_adapt, randomised goal)${X}       ${W}│"
        echo -e "  │${X}  ${G}2)${X} TD3 adapt   ${D}(pfe_td3_adapt, randomised goal)${X}       ${W}│"
        echo -e "  │${X}  ${G}7)${X} SAC adapt + shield                               ${W}│"
        echo -e "  │${X}  ${G}8)${X} TD3 adapt + shield                               ${W}│"
        echo -e "  ├─ Waypoint Roadmap  WP1 → WP2 → WP3 ──────────────────────┤"
        echo -e "  │${X}  ${C}9)${X}  Phase 1 — Core Maze      ${D}~5 h  · 1000 ep${X}          ${W}│"
        echo -e "  │${X}  ${C}11)${X} Phase 2 — Domain Rand.   ${D}~10 h · 4 styles×500 ep${X}  ${W}│"
        echo -e "  │${X}  ${B}12)${X} Phase 3a — Fine-tune Rabat  ${D}~1 h · 250 ep${X}         ${W}│"
        echo -e "  │${X}  ${B}13)${X} Phase 3b — Fine-tune Egypt  ${D}~1 h · 250 ep${X}         ${W}│"
        echo -e "  │${X}  ${B}14)${X} Phase 3c — Fine-tune Maze   ${D}~1 h · 250 ep${X}         ${W}│"
        echo -e "  │${X}  ${M}15)${X} Phase 4 — Evaluate all worlds  ${D}50 ep each${X}          ${W}│"
        echo -e "  │${X}  ${Y}10)${X} Full Roadmap — ALL phases  ${D}~18 h unattended${X}         ${W}│"
        echo -e "  ├─ Utilities ────────────────────────────────────────────────┤"
        echo -e "  │${X}  ${M}3)${X}  TensorBoard                                      ${W}│"
        echo -e "  │${X}  ${D}5)${X}  SAC raw  ${D}(no preset)${X}                             ${W}│"
        echo -e "  │${X}  ${D}6)${X}  TD3 raw  ${D}(no preset)${X}                             ${W}│"
        echo -e "  │${X}  ${R}4)${X}  Exit                                             ${W}│"
        echo -e "  └──────────────────────────────────────────────────────────┘${X}"
        echo ""
        read -r -p "  Choice [1-15]: " choice
        choice="${choice// /}"
        case "$choice" in
            1)  TRAIN_PRESET="pfe_sac_adapt"    TRAIN_ALGO="" TRAIN_USE_SHIELD=0 _run_stack; ok=1 ;;
            2)  TRAIN_PRESET="pfe_td3_adapt"    TRAIN_ALGO="" TRAIN_USE_SHIELD=0 _run_stack; ok=1 ;;
            3)  gnome-terminal --title="TENSORBOARD" -- bash -c \
                "cd $(_q "$AGENT_DIR") && tensorboard --logdir=pfe_logs; exec bash"; ok=1 ;;
            4)  echo "Bye."; exit 0 ;;
            5)  TRAIN_PRESET="" TRAIN_ALGO="sac" TRAIN_USE_SHIELD=0 _run_stack; ok=1 ;;
            6)  TRAIN_PRESET="" TRAIN_ALGO="td3" TRAIN_USE_SHIELD=0 _run_stack; ok=1 ;;
            7)  TRAIN_PRESET="pfe_sac_adapt"    TRAIN_ALGO="" TRAIN_USE_SHIELD=1 _run_stack; ok=1 ;;
            8)  TRAIN_PRESET="pfe_td3_adapt"    TRAIN_ALGO="" TRAIN_USE_SHIELD=1 _run_stack; ok=1 ;;
            9)  _phase_terminal "PFE_PHASE1_CORE"   1; ok=1 ;;
            10) _phase_terminal "PFE_FULL_ROADMAP"  1; ok=1 ;;
            11) _phase_terminal "PFE_PHASE2_DR"     2; ok=1 ;;
            12) _phase_terminal "PFE_PHASE3_RABAT"  3 "--world rabat"; ok=1 ;;
            13) _phase_terminal "PFE_PHASE3_EGYPT"  3 "--world egypt"; ok=1 ;;
            14) _phase_terminal "PFE_PHASE3_MAZE"   3 "--world maze";  ok=1 ;;
            15) _phase_terminal "PFE_PHASE4_EVAL"   4; ok=1 ;;
            *)  echo "Invalid — enter 1-15." >&2; sleep 1 ;;
        esac
    done
}

_dispatch() {
    case "$1" in
        1)  TRAIN_PRESET="pfe_sac_adapt" TRAIN_ALGO="" TRAIN_USE_SHIELD=0 _run_stack ;;
        2)  TRAIN_PRESET="pfe_td3_adapt" TRAIN_ALGO="" TRAIN_USE_SHIELD=0 _run_stack ;;
        3)  gnome-terminal --title="TENSORBOARD" -- bash -c \
            "cd $(_q "$AGENT_DIR") && tensorboard --logdir=pfe_logs; exec bash" ;;
        4)  exit 0 ;;
        5)  TRAIN_PRESET="" TRAIN_ALGO="sac" TRAIN_USE_SHIELD=0 _run_stack ;;
        6)  TRAIN_PRESET="" TRAIN_ALGO="td3" TRAIN_USE_SHIELD=0 _run_stack ;;
        7)  TRAIN_PRESET="pfe_sac_adapt"    TRAIN_ALGO="" TRAIN_USE_SHIELD=1 _run_stack ;;
        8)  TRAIN_PRESET="pfe_td3_adapt"    TRAIN_ALGO="" TRAIN_USE_SHIELD=1 _run_stack ;;
        9)  _phase_terminal "PFE_PHASE1_CORE"  1 ;;
        10) _phase_terminal "PFE_FULL_ROADMAP" 1 ;;
        11) _phase_terminal "PFE_PHASE2_DR"    2 ;;
        12) _phase_terminal "PFE_PHASE3_RABAT" 3 "--world rabat" ;;
        13) _phase_terminal "PFE_PHASE3_EGYPT" 3 "--world egypt" ;;
        14) _phase_terminal "PFE_PHASE3_MAZE"  3 "--world maze"  ;;
        15) _phase_terminal "PFE_PHASE4_EVAL"  4 ;;
        *)  _die "Invalid choice '$1' — use 1-15." ;;
    esac
}

main() {
    _parse_cli "$@"
    _check_ros
    local c="${CHOICE_FROM_CLI:-${START_PFE_CHOICE:-}}"
    [[ -n "$c" ]] && _dispatch "$c" || _menu
}

main "$@"
