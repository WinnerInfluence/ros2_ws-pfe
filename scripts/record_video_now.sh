#!/usr/bin/env bash
# Launch THE maze solving scene: eval_maze.world + maze-peak SAC + hot_swap in Gazebo.
#
#   cd ~/ros2_ws && bash scripts/record_video_now.sh
#
# Record the window: ★ MAZE GAZEBO — RECORD THIS
#   click my_robot → F (follow) → View → uncheck Laser Scan
set -euo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
PKG="$WS/src/safe_drl_nav/safe_drl_nav"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# shellcheck source=/dev/null
source "$PKG/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"
TURTLEBOT_SDF="/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf"
SIM_ASSETS="$PKG/sim_assets"

MAZE_WORLD="$SIM_ASSETS/worlds/eval_maze.world"
MAZE_PEAK="$PKG/trained_models/sac_actor_maze_best_ever_eval_maze.pth"
LAB_BEST="$PKG/trained_models/sac_actor_maze_best_ever.pth"
GEN_WORLDS="$SIM_ASSETS/scripts/generate_eval_worlds.py"

if [[ ! -f "$MAZE_WORLD" ]]; then
    echo "Generating eval_maze.world…"
    [[ -f "$GEN_WORLDS" ]] && python3 "$GEN_WORLDS" || python3 "$SIM_ASSETS/scripts/generate_eval_world.py"
fi
[[ -f "$MAZE_WORLD" ]] || { echo "[ERROR] Missing $MAZE_WORLD"; exit 1; }

if [[ -f "$MAZE_PEAK" ]]; then MODEL="$MAZE_PEAK"
elif [[ -f "$LAB_BEST" ]]; then MODEL="$LAB_BEST"
else echo "[ERROR] No SAC checkpoint in $PKG/trained_models"; exit 1; fi

export PFE_WORLD="$(readlink -f "$MAZE_WORLD" 2>/dev/null || realpath "$MAZE_WORLD")"
export EVAL_MODEL="$MODEL"
# Default 0.09 for maze video (slow, filmable). Fast: VIDEO_STEP_SLEEP=0.02 bash ...
export VIDEO_STEP_SLEEP="${VIDEO_STEP_SLEEP:-0.09}"
export VIDEO_CONTROL_PERIOD="${VIDEO_CONTROL_PERIOD:-0.09}"

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

_launch_maze_sim() {
    local src gz_gui wq
    src="$(_src)"
    gz_gui="$(pfe_gazebo_gui_suffix)"
    wq="$(printf '%q' "$PFE_WORLD")"

    gnome-terminal --geometry=140x45 --title="★ MAZE GAZEBO — RECORD THIS" -- bash -c \
        "$(pfe_term_env_prefix)${src}export PFE_WORLD=${wq};
echo '';
echo '  ╔════════════════════════════════════════════════════╗';
echo '  ║  GREEN HEDGE MAZE — screen-record THIS window      ║';
echo '  ║  1) Click my_robot   2) Press F (follow camera)    ║';
echo '  ║  3) View → UNCHECK Laser Scan                      ║';
echo '  ╚════════════════════════════════════════════════════╝';
echo '';
ros2 launch gazebo_ros gazebo.launch.py world:=\$PFE_WORLD${gz_gui}; exec bash" 2>/dev/null || true

    sleep 14

    local spawn_cmd
    spawn_cmd="$(pfe_term_env_prefix)source '/opt/ros/${ROS_DISTRO}/setup.bash'"
    spawn_cmd+="; ros2 run gazebo_ros spawn_entity.py -timeout 120"
    spawn_cmd+=" -entity my_robot -file '$TURTLEBOT_SDF'"
    spawn_cmd+=" -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z; exec bash"
    gnome-terminal --title="SPAWNER" -- bash -c "$spawn_cmd" 2>/dev/null || true
}

clear 2>/dev/null || true
echo ""
echo "  ══════════════════════════════════════════════════════"
echo "   MAZE SOLVING SCENE (eval_maze + maze-peak policy)"
echo "  ══════════════════════════════════════════════════════"
echo "   World : eval_maze.world  (green hedge labyrinth)"
echo "   Model : $(basename "$MODEL")"
echo ""
echo "   ▶ Start screen recorder on the GAZEBO window."
echo "   ▶ Press Enter to launch Gazebo + robot + policy."
echo "  ══════════════════════════════════════════════════════"
read -r _

_kill_stale
_launch_maze_sim

echo "  Waiting 16s for Gazebo + spawn…"
sleep 16

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
set -u

if ! ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
    echo "[WARN] /reset_simulation not up yet — waiting 10s more…"
    sleep 10
fi

PRESENTATION="$WS/scripts/run_presentation_demo.sh"
[[ -f "$PRESENTATION" ]] || { echo "[ERROR] Missing $PRESENTATION"; exit 1; }

echo "  Starting policy (hot_swap, SAC sample) — robot drives in the maze."
gnome-terminal --geometry=90x24 --title="POLICY (auto)" -- bash -c \
    "$(_src) cd '$PKG' && bash '$PRESENTATION'; exec bash" 2>/dev/null \
    || bash "$PRESENTATION"

echo ""
echo "  Scene is live. Record: ★ MAZE GAZEBO — RECORD THIS"
echo "  New take: ros2 service call /reset_simulation std_srvs/srv/Empty \"{}\""
