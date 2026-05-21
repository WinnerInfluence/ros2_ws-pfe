#!/usr/bin/env bash
# Multi-segment transfer demo for screen recording (lab → Egypt ZS → Egypt FT → maze).
#
#   cd ~/ros2_ws && bash scripts/record_transfer_video.sh
#
# Env:
#   TRANSFER_SEGMENTS=5|4|3   5 = lab+rabat+egypt_zs+egypt_p3+maze(slow) (recommended)
#   RECORD_SKIP_PROMPT=1      auto-advance timers (less control)
#   SEGMENT_PAUSE_SEC=45      seconds per segment if SKIP_PROMPT=1
#   VIDEO_STEP_SLEEP=0.02     default sim pace (egypt/lab)
#   VIDEO_MAZE_STEP_SLEEP=0.09   maze only — slower robot for recording (not “fast solve”)
#   VIDEO_MAZE_CONTROL_PERIOD=0.09
#
# Between segments: script stops policy + Gazebo, shows title text, waits for Enter.
# Edit on-screen titles: seg*_head / seg*_detail variables below.
set -euo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
PKG="$WS/src/safe_drl_nav/safe_drl_nav"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PRESENTATION="$WS/scripts/run_presentation_demo.sh"

# shellcheck source=/dev/null
source "$PKG/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"
TURTLEBOT_SDF="/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf"
SIM_ASSETS="$PKG/sim_assets"
WORLDS="$SIM_ASSETS/worlds"
MODELS="$PKG/trained_models"

export VIDEO_STEP_SLEEP="${VIDEO_STEP_SLEEP:-0.02}"
export VIDEO_CONTROL_PERIOD="${VIDEO_CONTROL_PERIOD:-0.02}"
VIDEO_MAZE_STEP_SLEEP="${VIDEO_MAZE_STEP_SLEEP:-0.09}"
VIDEO_MAZE_CONTROL_PERIOD="${VIDEO_MAZE_CONTROL_PERIOD:-0.09}"
TRANSFER_SEGMENTS="${TRANSFER_SEGMENTS:-5}"
SEGMENT_PAUSE_SEC="${SEGMENT_PAUSE_SEC:-50}"
RECORD_SKIP_PROMPT="${RECORD_SKIP_PROMPT:-0}"

LAB="$WORLDS/current_random_lab.world"
EGYPT="$WORLDS/eval_egypt.world"
MAZE="$WORLDS/eval_maze.world"
BEST="$MODELS/sac_actor_maze_best_ever.pth"
P3_EGYPT="$MODELS/sac_actor_phase3_egypt.pth"
P3_RABAT="$MODELS/sac_actor_phase3_rabat.pth"
P3_MAZE="$MODELS/sac_actor_phase3_maze.pth"

_src() {
    local s=""
    [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] && s+="source '/opt/ros/${ROS_DISTRO}/setup.bash'; "
    [[ -f "$WS/install/setup.bash" ]] && s+="source '$WS/install/setup.bash'; "
    printf '%s' "$s"
}

_kill_stale() {
    for p in gzserver gzclient rviz2 main_agent.py evaluate_agent.py hot_swap_eval_node.py; do
        pkill -9 -f "$p" 2>/dev/null || true
    done
    sleep 2
}

_wait_reset() {
    local i
    for i in $(seq 1 30); do
        if ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
            return 0
        fi
        sleep 1
    done
    echo "[WARN] /reset_simulation not ready yet"
    return 1
}

_launch_gazebo() {
    local world="$1" title="$2"
    local src gz_gui wq
    src="$(_src)"
    gz_gui="$(pfe_gazebo_gui_suffix)"
    wq="$(printf '%q' "$world")"

    gnome-terminal --geometry=140x45 --title="$title" -- bash -c \
        "$(pfe_term_env_prefix)${src}export PFE_WORLD=${wq};
echo \"[GAZEBO] world=\$PFE_WORLD\";
ros2 launch gazebo_ros gazebo.launch.py world:=\$PFE_WORLD${gz_gui}; exec bash" 2>/dev/null || true

    sleep 14
    local spawn_cmd
    spawn_cmd="$(pfe_term_env_prefix)source '/opt/ros/${ROS_DISTRO}/setup.bash'"
    spawn_cmd+="; ros2 run gazebo_ros spawn_entity.py -timeout 120"
    spawn_cmd+=" -entity my_robot -file '$TURTLEBOT_SDF'"
    spawn_cmd+=" -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z; exec bash"
    gnome-terminal --title="SPAWNER" -- bash -c "$spawn_cmd" 2>/dev/null || true
    sleep 12
}

_run_policy() {
    local world="$1" model="$2" step_sleep="${3:-}" control_period="${4:-}"
    export PFE_WORLD="$world"
    export EVAL_MODEL="$model"
    if [[ -n "$step_sleep" ]]; then
        export VIDEO_STEP_SLEEP="$step_sleep"
        export VIDEO_CONTROL_PERIOD="${control_period:-$step_sleep}"
    fi
    gnome-terminal --geometry=92x26 --title="POLICY — $(basename "$model")" -- bash -c \
        "$(_src) export VIDEO_STEP_SLEEP='${VIDEO_STEP_SLEEP}' VIDEO_CONTROL_PERIOD='${VIDEO_CONTROL_PERIOD}';
cd '$PKG' && bash '$PRESENTATION'; exec bash" 2>/dev/null \
        || bash "$PRESENTATION"
}

_show_title() {
    local n="$1" headline="$2" detail="$3"
    clear 2>/dev/null || true
    echo ""
    echo "  ╔══════════════════════════════════════════════════════════════╗"
    printf "  ║  SEGMENT %s — %-47s║\n" "$n" "$headline"
    echo "  ╠══════════════════════════════════════════════════════════════╣"
    # word-wrap detail roughly to 62 chars
    echo "  ║"
    while IFS= read -r line; do
        printf "  ║  %s\n" "$line"
    done <<< "$detail"
    echo "  ║"
    echo "  ╚══════════════════════════════════════════════════════════════╝"
    echo ""
    echo "  → Record this text as a TITLE CARD in your editor (or hold 5 s on screen)."
    echo "  → Edit titles: seg*_head / seg*_detail in scripts/record_transfer_video.sh"
    echo ""
}

_pause() {
    local msg="$1"
    if [[ "$RECORD_SKIP_PROMPT" == "1" ]]; then
        echo "  (auto) $msg — waiting ${SEGMENT_PAUSE_SEC}s…"
        sleep "$SEGMENT_PAUSE_SEC"
    else
        read -r -p "  $msg [Enter] " _
    fi
}

_run_segment() {
    local idx="$1" headline="$2" detail="$3" world="$4" model="$5" gz_title="$6"
    local step_sleep="${7:-}" control_period="${8:-}"

    [[ -f "$world" ]] || { echo "[ERROR] Missing world: $world"; exit 1; }
    [[ -f "$model" ]] || { echo "[ERROR] Missing model: $model"; exit 1; }

    world="$(readlink -f "$world" 2>/dev/null || realpath "$world")"

    _show_title "$idx" "$headline" "$detail"
    _pause "Title card ready? Launch Gazebo for this segment"

    _kill_stale
    echo "  Launching: $(basename "$world") + $(basename "$model")"
    _launch_gazebo "$world" "$gz_title"

    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    [[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
    set -u
    _wait_reset || sleep 8

    _run_policy "$world" "$model" "$step_sleep" "$control_period"
    echo ""
    echo "  ● RECORD NOW: $gz_title"
    [[ -n "$step_sleep" ]] && echo "  ● Sim pace: sleep=${step_sleep}s (slower = easier to film)"
    echo "  ● Robot: click my_robot → F → View → uncheck Laser Scan"
    echo "  ● New take: ros2 service call /reset_simulation std_srvs/srv/Empty \"{}\""
    echo ""
    _pause "Done recording this segment? (next segment will restart Gazebo)"
    _kill_stale
}

# ── Segment definitions ─────────────────────────────────────────────────────
# Format: headline | detail (multiline ok) | world | model | gnome title

seg1_head="TRAINING LAB (in-distribution)"
seg1_detail="Phase 1: random-goal adaptation (warm-start)
Phase 2: waypoint curriculum WP1→WP2→WP3
Policy: sac_actor_maze_best_ever.pth
(Edit seg1_head / seg1_detail in scripts/record_transfer_video.sh)"

seg2_head="MEDIUM — Zero-shot Egypt"
seg2_detail="Same lab checkpoint · eval_egypt.world
No Phase 3 fine-tune on this map
~7/33 full maze solve (thesis eval)"

seg3_head="MEDIUM+ — Egypt after Phase 3 FT"
seg3_detail="Fine-tuned on Egypt target world
Policy: sac_actor_phase3_egypt.pth
~14/33 full solve"

seg_rabat_zs_head="HARD ZS — Rabat pillars"
seg_rabat_zs_detail="eval_rabat.world · lab checkpoint (zero-shot)
Dense pillar forest — hardest zero-shot (~0/33)
Optional: re-run Phase 3 on Rabat to improve demo"

seg_rabat_p3_head="Rabat after Phase 3 FT"
seg_rabat_p3_detail="Policy: sac_actor_phase3_rabat.pth
Already trained (~250 ep) · thesis ~2/33
More training: ONLY_WORLD=rabat PHASE3_EP=400 bash train_waypoint.sh"

seg4_head="HARD — Hedge maze (slow pace)"
seg4_detail="eval_maze.world · Phase 3 maze fine-tune
Policy: sac_actor_phase3_maze.pth
SLOW sim for video (not sped-up) · ~2/33 full solve"

clear 2>/dev/null || true
echo ""
echo "  ══════════════════════════════════════════════════════════"
echo "   TRANSFER VIDEO RECORDER — ${TRANSFER_SEGMENTS} segment(s)"
echo "  ══════════════════════════════════════════════════════════"
echo "  Edit title text: seg*_head / seg*_detail in this script"
echo "  Tip: record each Gazebo clip, add title cards in your video editor."
echo ""

if [[ "$TRANSFER_SEGMENTS" == "3" ]]; then
    _run_segment "1/3" "$seg2_head" "$seg2_detail" "$EGYPT" "$BEST" "★ SEG1 EGYPT ZEROSHOT"
    _run_segment "2/3" "$seg3_head" "$seg3_detail" "$EGYPT" "$P3_EGYPT" "★ SEG2 EGYPT PHASE3"
    _run_segment "3/3" "$seg4_head" "$seg4_detail" "$MAZE" "$P3_MAZE" "★ SEG3 MAZE SLOW" \
        "$VIDEO_MAZE_STEP_SLEEP" "$VIDEO_MAZE_CONTROL_PERIOD"
elif [[ "$TRANSFER_SEGMENTS" == "4" ]]; then
    _run_segment "1/4" "$seg1_head" "$seg1_detail" "$LAB" "$BEST" "★ SEG1 TRAINING LAB"
    _run_segment "2/4" "$seg2_head" "$seg2_detail" "$EGYPT" "$BEST" "★ SEG2 EGYPT ZEROSHOT"
    _run_segment "3/4" "$seg3_head" "$seg3_detail" "$EGYPT" "$P3_EGYPT" "★ SEG3 EGYPT PHASE3"
    _run_segment "4/4" "$seg4_head" "$seg4_detail" "$MAZE" "$P3_MAZE" "★ SEG4 MAZE SLOW" \
        "$VIDEO_MAZE_STEP_SLEEP" "$VIDEO_MAZE_CONTROL_PERIOD"
else
  # 5 = lab → Rabat (trained) → Egypt ZS → Egypt P3 → maze slow
    RABAT="$WORLDS/eval_rabat.world"
    _run_segment "1/5" "$seg1_head" "$seg1_detail" "$LAB" "$BEST" "★ SEG1 TRAINING LAB"
    _run_segment "2/5" "$seg_rabat_p3_head" "$seg_rabat_p3_detail" "$RABAT" "$P3_RABAT" "★ SEG2 RABAT PHASE3"
    _run_segment "3/5" "$seg2_head" "$seg2_detail" "$EGYPT" "$BEST" "★ SEG3 EGYPT ZEROSHOT"
    _run_segment "4/5" "$seg3_head" "$seg3_detail" "$EGYPT" "$P3_EGYPT" "★ SEG4 EGYPT PHASE3"
    _run_segment "5/5" "$seg4_head" "$seg4_detail" "$MAZE" "$P3_MAZE" "★ SEG5 MAZE SLOW" \
        "$VIDEO_MAZE_STEP_SLEEP" "$VIDEO_MAZE_CONTROL_PERIOD"
fi

echo ""
echo "  All segments done. Assemble in your editor: title → clip → title → clip …"
echo ""
