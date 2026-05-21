#!/usr/bin/env bash
# =============================================================================
# run_thesis_crossworld_sac.sh — Cross-world SAC eval for thesis Table tab:world-eval
#
# Runs TWO conditions per eval world (recommended for the thesis narrative):
#   zeroshot — sac_actor_maze_best_ever.pth   (lab policy, no world-specific FT)
#   phase3   — sac_actor_phase3_<world>.pth   (after Phase 3 fine-tune on that world)
#
# Same harness as eval_trainlab.json: --paper-eval, n=33, sample SAC, shield ON.
#
# Usage (workstation with Gazebo GUI; close any web demo first):
#   bash ~/ros2_ws/scripts/run_thesis_crossworld_sac.sh
#
# Env overrides:
#   RUN_SET=zeroshot|phase3|both     (default: both)
#   WORLDS=rabat,egypt,maze          (default: all three)
#   EPISODES=33
#   SCREENSHOTS=1|0                  capture Gazebo before each eval (default 0 — manual after)
#   COPY_TO_THESIS=1|0               copy PNGs → thesis assets/screenshots/
#   DRY_RUN=1                        print commands only
#   MANUAL_SIM=1                     skip launch; you already have Gazebo on PFE_WORLD
#   PAUSE_BETWEEN=1                  read -p before each run (default 0)
#   LAUNCH_RVIZ=0|1                  RViz during sim (default 0 — saves CPU)
#   EVAL_KILL_GUI=1|0                after screenshot, kill gzclient+rviz; keep gzserver (default 1)
# =============================================================================
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="$WS_PATH/src/safe_drl_nav/safe_drl_nav"
SCRIPT_DIR="$AGENT_DIR"
WORLD_DIR="$AGENT_DIR/sim_assets/worlds"
MODEL_DIR="$AGENT_DIR/trained_models"
LOG_DIR="$AGENT_DIR/pfe_logs"
THESIS_SHOTS="${THESIS_SHOTS:-$WS_PATH/pfe_report/thesis_pfe_pro/assets/screenshots}"
RUN_SET="${RUN_SET:-both}"
WORLDS="${WORLDS:-rabat,egypt,maze}"
EPISODES="${EPISODES:-33}"
SCREENSHOTS="${SCREENSHOTS:-0}"
COPY_TO_THESIS="${COPY_TO_THESIS:-1}"
DRY_RUN="${DRY_RUN:-0}"
MANUAL_SIM="${MANUAL_SIM:-0}"
PAUSE_BETWEEN="${PAUSE_BETWEEN:-0}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-0}"
EVAL_KILL_GUI="${EVAL_KILL_GUI:-1}"
CAPTURE_BIN="$WS_PATH/scripts/capture_thesis_screenshot.sh"
BEST_EVER="$MODEL_DIR/sac_actor_maze_best_ever.pth"
POST_SLEEP="${POST_SLEEP:-16}"

# shellcheck disable=SC1091
source "$AGENT_DIR/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env

TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
TURTLEBOT_SDF="/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"

_r() { echo -e "\033[1;31m$*\033[0m"; }
_g() { echo -e "\033[1;32m$*\033[0m"; }
_y() { echo -e "\033[1;33m$*\033[0m"; }
_d() { echo -e "\033[2m$*\033[0m"; }

# ROS setup.bash references unset vars; incompatible with bash -u (nounset).
_source_ros() {
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    if [[ -f "${WS_PATH}/install/setup.bash" ]]; then
        source "${WS_PATH}/install/setup.bash"
    fi
    set -u
}

# Prefix for gnome-terminal child shells (fresh bash, no -u).
_src() {
    local s="set +u; source /opt/ros/${ROS_DISTRO}/setup.bash; "
    if [[ -f "${WS_PATH}/install/setup.bash" ]]; then
        s+="source ${WS_PATH}/install/setup.bash; "
    fi
    printf '%s' "$s"
}

_kill_stale() {
    local p
    for p in hot_swap_eval upload_server upload_telemetry main_agent.py evaluate_agent.py; do
        pkill -9 -f "$p" 2>/dev/null || true
    done
    sleep 2
    local _i
    for ((_i = 0; _i < 20; _i++)); do
        pgrep -f gzserver >/dev/null 2>&1 || break
        pkill -9 -f gzserver 2>/dev/null || true
        sleep 1
    done
    sleep 1
}

# Drop GUI only — physics/sim (gzserver) and spawned robot stay up for evaluate_agent.py.
_kill_gui_clients() {
    _d "Stopping Gazebo client (gzclient) and RViz — gzserver stays up for eval."
    pkill -x gzclient 2>/dev/null || pkill -f gzclient 2>/dev/null || true
    pkill -x rviz2 2>/dev/null || pkill -f 'rviz2' 2>/dev/null || true
    sleep 1
    if pgrep -x gzclient >/dev/null 2>&1; then
        pkill -9 -x gzclient 2>/dev/null || true
    fi
    if pgrep -x rviz2 >/dev/null 2>&1; then
        pkill -9 -x rviz2 2>/dev/null || true
    fi
    sleep 1
}

_wait_reset_service() {
    local max_sec="${1:-120}"
    _d "Waiting up to ${max_sec}s for /reset_simulation ..."
    local t=0
    while (( t < max_sec )); do
        if ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
            _g "▶ /reset_simulation ready."
            return 0
        fi
        sleep 2
        t=$((t + 2))
    done
    _y "⚠ /reset_simulation not seen — eval may exit with code 2."
    return 1
}

_launch_sim() {
    local world_abs="$1"
    local want_gui="${2:-0}"
    world_abs="$(readlink -f "$world_abs" 2>/dev/null || realpath "$world_abs")"
    export PFE_WORLD="$world_abs"
    local wq src gz_gui
    wq="$(printf '%q' "$world_abs")"
    src="$(_src)"
    if [[ "$want_gui" == "1" ]]; then
        gz_gui="$(pfe_gazebo_gui_suffix)"
    else
        gz_gui=" gui:=false"
    fi

    if [[ "$want_gui" == "1" && -z "${DISPLAY:-}" ]]; then
        _y "DISPLAY unset — launching headless (gui:=false); screenshots may be skipped."
        gz_gui=" gui:=false"
        want_gui=0
    fi

    if [[ "$want_gui" == "1" ]]; then
        _d "Launching Gazebo → $world_abs (GUI on for screenshot, then client killed for eval)"
    else
        _d "Launching Gazebo → $world_abs (headless gzserver only)"
    fi
    gnome-terminal --title="GAZEBO_CORE" -- bash -c \
        "${src}export PFE_WORLD=${wq}; echo \"[GAZEBO] world=\$PFE_WORLD\"; ros2 launch gazebo_ros gazebo.launch.py world:=\$PFE_WORLD${gz_gui}; exec bash"
    sleep "$POST_SLEEP"

    local spawn_cmd
    spawn_cmd="$(_src)"
    spawn_cmd+="ros2 run gazebo_ros spawn_entity.py -timeout 120"
    spawn_cmd+=" -entity my_robot -file '$TURTLEBOT_SDF'"
    spawn_cmd+=" -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z; sleep 2; exec bash"
    gnome-terminal --title="SPAWNER" -- bash -c "$spawn_cmd"

    if [[ "$LAUNCH_RVIZ" == "1" && "$want_gui" == "1" ]]; then
        gnome-terminal --title="RVIZ_MONITOR" -- bash -c \
            "$(_src)rviz2 --ros-args -p use_sim_time:=true; exec bash"
        sleep 4
    fi
}

_world_path() {
    case "$1" in
        rabat) echo "$WORLD_DIR/eval_rabat.world" ;;
        egypt) echo "$WORLD_DIR/eval_egypt.world" ;;
        maze)  echo "$WORLD_DIR/eval_maze.world" ;;
        *) _r "Unknown world: $1"; exit 1 ;;
    esac
}

_thesis_shot_name() {
    local world="$1" condition="$2"
    if [[ "$condition" == "phase3" ]]; then
        case "$world" in
            rabat) echo "phase3_rabat.png" ;;
            egypt) echo "phase3_egypt.png" ;;
            maze)  echo "phase3_maze_runner.png" ;;
        esac
    else
        echo "eval_zeroshot_${world}.png"
    fi
}

_capture_world_shots() {
    local world="$1" condition="$2"
    [[ "$SCREENSHOTS" == "1" ]] || return 0
    [[ -x "$CAPTURE_BIN" ]] || { _y "Skip screenshots: $CAPTURE_BIN missing"; return 0; }

    local fname staging
    fname="$(_thesis_shot_name "$world" "$condition")"
    staging="$LOG_DIR/screenshots/${fname}"
    mkdir -p "$(dirname "$staging")"

    _y "Screenshot pause (10s) — focus Gazebo, then RViz if you want a second angle."
    sleep 3
    bash "$CAPTURE_BIN" "$staging" gazebo || true
    if [[ "$COPY_TO_THESIS" == "1" ]]; then
        cp -f "$staging" "$THESIS_SHOTS/$fname" 2>/dev/null && _g "Copied → $THESIS_SHOTS/$fname" || _y "Could not copy to $THESIS_SHOTS (create dir?)"
    fi
}

_run_eval() {
    local world="$1" condition="$2"
    local wpath model tag

    wpath="$(_world_path "$world")"
    [[ -f "$wpath" ]] || { _r "Missing world: $wpath"; return 1; }

    if [[ "$condition" == "zeroshot" ]]; then
        model="$BEST_EVER"
        tag="sac_${world}_${EPISODES}_zeroshot"
    else
        model="$MODEL_DIR/sac_actor_phase3_${world}.pth"
        tag="sac_${world}_${EPISODES}_phase3"
    fi

    [[ -f "$model" ]] || { _r "Missing checkpoint: $model"; return 1; }

  if [[ "$PAUSE_BETWEEN" == "1" ]]; then
        read -r -p "Enter to start eval: $condition on $world ($tag) ..." _
    fi

    local want_gui=0
    [[ "$SCREENSHOTS" == "1" ]] && want_gui=1

    if [[ "$MANUAL_SIM" != "1" ]]; then
        _kill_stale
        # Full stop — headless gzserver alone blocks the next GUI launch.
        pkill -9 -x gzserver 2>/dev/null || true
        pkill -9 -x gzclient 2>/dev/null || true
        sleep 2
        _source_ros
        _launch_sim "$wpath" "$want_gui"
        _wait_reset_service 120 || true
    else
        export PFE_WORLD="$(readlink -f "$wpath")"
        _d "MANUAL_SIM: using PFE_WORLD=$PFE_WORLD"
    fi

    _capture_world_shots "$world" "$condition"

    if [[ "$EVAL_KILL_GUI" == "1" ]]; then
        _kill_gui_clients
    fi

    local json_out="$LOG_DIR/eval_${tag}.json"
    if [[ "$DRY_RUN" == "1" ]]; then
        _d "[DRY_RUN] evaluate_agent.py --model $model --tag $tag"
        return 0
    fi

    _g "Eval: $condition | $world | $tag"
    (
        _source_ros
        export PFE_WORLD="$(readlink -f "$wpath")"
        cd "$AGENT_DIR"
        python3 evaluate_agent.py \
            --algo sac \
            --model "$model" \
            --paper-eval \
            --episodes "$EPISODES" \
            --max-steps 4000 \
            --env-step-sleep-sec 0.05 \
            --waypoint-goal-radius 0.68 \
            --tag "$tag" \
            2>&1 | tee "$LOG_DIR/eval_${tag}.log"
    )

    if [[ -f "$json_out" ]]; then
        python3 - "$json_out" <<'PY'
import json, sys
p = sys.argv[1]
d = json.load(open(p))
s = d["summary"]
print(f"\n=== {p} ===")
print(f"  solved: {s['episodes_solved']}/{s['episodes_total']} ({100*s['success_rate']:.1f}%)")
print(f"  mean_wp: {s['mean_waypoints']} ± {s['std_waypoints']}")
print(f"  mean_return: {s['mean_reward']} ± {s['std_reward']}")
print(f"  world: ...{d['eval_config'].get('pfe_world','')[-45:]}")
print(f"  model: ...{d['model_path'][-50:]}")
PY
    else
        _r "JSON not written: $json_out"
    fi

    if [[ "$MANUAL_SIM" != "1" ]]; then
        _kill_stale
    fi
}

_should_run_condition() {
    local c="$1"
    case "$RUN_SET" in
        both) return 0 ;;
        zeroshot) [[ "$c" == "zeroshot" ]] && return 0 || return 1 ;;
        phase3)   [[ "$c" == "phase3" ]] && return 0 || return 1 ;;
        *) _r "RUN_SET must be both|zeroshot|phase3"; exit 1 ;;
    esac
}

# ── Main ──────────────────────────────────────────────────────────────────────
if [[ ! -f "$AGENT_DIR/evaluate_agent.py" ]]; then
    _r "Agent dir not found: $AGENT_DIR"
    exit 1
fi
if [[ ! -f "$BEST_EVER" ]]; then
    _r "Missing lab checkpoint: $BEST_EVER"
    exit 1
fi

_kill_stale

_g "Thesis cross-world SAC eval"
_d "  RUN_SET=$RUN_SET  WORLDS=$WORLDS  EPISODES=$EPISODES"
_d "  zeroshot → $BEST_EVER"
_d "  phase3   → sac_actor_phase3_<world>.pth"
_d "  JSON     → $LOG_DIR/eval_sac_<world>_${EPISODES}_<condition>.json"
_d "  LAUNCH_RVIZ=$LAUNCH_RVIZ  EVAL_KILL_GUI=$EVAL_KILL_GUI (gzclient+rviz off during eval)"
echo ""
_y "Why before AND after fine-tune?"
echo "  • zeroshot = sim-to-real / domain-shift gap (lab → eval world)"
echo "  • phase3   = value of short world-specific adaptation (Chapter 5 Phase 3)"
echo "  Phase 3 weights should already exist locally — this script measures transfer, not re-training."
echo ""

_source_ros

IFS=',' read -r -a WORLD_ARR <<< "$WORLDS"
for world in "${WORLD_ARR[@]}"; do
    world="$(echo "$world" | tr -d ' ')"
    [[ -n "$world" ]] || continue
    if _should_run_condition zeroshot; then
        _run_eval "$world" zeroshot
    fi
    if _should_run_condition phase3; then
        _run_eval "$world" phase3
    fi
done

_g "Done. Summaries:"
for f in "$LOG_DIR"/eval_sac_*_"${EPISODES}"_*.json; do
    [[ -f "$f" ]] || continue
    python3 - "$f" <<'PY' 2>/dev/null || true
import json, sys, os
d = json.load(open(sys.argv[1]))
s = d["summary"]
base = os.path.basename(sys.argv[1])
print(f"  {base}: {s['episodes_solved']}/{s['episodes_total']} ({100*s['success_rate']:.1f}%)  wp={s['mean_waypoints']:.2f}")
PY
done
_d "Screenshots (if enabled): $THESIS_SHOTS/"
_d "Fill thesis Table tab:world-transfer from the JSON files above."
