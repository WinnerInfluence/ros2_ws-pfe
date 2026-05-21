#!/usr/bin/env bash
# =============================================================================
# cloud_train_headless.sh — Hetzner / headless VPS: Gazebo (no gzclient) + SAC training
#
# RTF (real-time factor) is dominated by your .world physics + main_agent stepping.
# Tune the world's <physics><max_step_size> and <real_time_update_rate>; lower
# PFE_ENV_STEP_SLEEP for faster stepping on fast CPUs (not wall-clock fidelity).
#
# Usage (on the VPS, after cloning this workspace and building):
#   export WS_PATH=/root/ros2_ws
#   export PFE_WORLD="$WS_PATH/src/safe_drl_nav/safe_drl_nav/sim_assets/worlds/current_random_lab.world"
#   bash scripts/cloud_train_headless.sh             # SAC waypoint preset + training
#
# Override brain args after -- :
#   bash scripts/cloud_train_headless.sh -- --preset pfe_sac_waypoint --train-seed 7
#
# Prerequisites: ROS 2 Humble, gazebo_ros, turtlebot3_description, CUDA optional.
# =============================================================================
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_PATH="${WS_PATH:-$HOME/ros2_ws}"

AGENT_DIR="${AGENT_DIR:-$WS_PATH/src/safe_drl_nav/safe_drl_nav}"

if [[ ! -f "$WS_PATH/install/setup.bash" ]]; then
  echo "error: build workspace first — cd $WS_PATH && colcon build" >&2
  exit 1
fi

# Headless EGL / GLX fallbacks — adjust if NVidia GLX is wired on the VPS.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-minimal}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://localhost:11345}"
unset DISPLAY

TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
SIM_ASSETS="$AGENT_DIR/sim_assets"
_DEFAULT_WORLD="$SIM_ASSETS/worlds/current_random_lab.world"
PFE_WORLD="${PFE_WORLD:-$_DEFAULT_WORLD}"
TURTLEBOT_SDF="${TURTLEBOT_SDF:-/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf}"

EXTRA_BRAIN=()
if [[ "${1:-}" == "--" ]]; then
  shift
  EXTRA_BRAIN=("$@")
fi

_spawn_x="${SPAWN_X:--2.0}"
_spawn_y="${SPAWN_Y:--2.0}"
_spawn_z="${SPAWN_Z:-0.15}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "$WS_PATH/install/setup.bash"

_die() { echo "error: $*" >&2; exit 1; }
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] \
  || _die "ROS ${ROS_DISTRO} not found."

_kill_cloud() {
  for pat in gzserver gzclient "ros2 launch gazebo_ros" main_agent.py; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 2
}

echo "══ Cloud headless training ══"
echo "  World       : $PFE_WORLD"
echo "  Agent dir   : $AGENT_DIR"
echo ""

_kill_cloud

echo "── [randomize_world] ──"
rnd_script=""
for loc in \
  "$SIM_ASSETS/scripts/randomize_world.py" \
  "$AGENT_DIR/randomize_world.py"; do
  [[ -f "$loc" ]] && rnd_script="$loc" && break
done
if [[ "$PFE_WORLD" == "$_DEFAULT_WORLD" && -n "$rnd_script" ]]; then
  python3 "$rnd_script"
else
  echo "  (skipped — custom world or script missing)"
fi

echo "── [gzserver, gui:=false] ──"
ros2 launch gazebo_ros gazebo.launch.py \
  world:="$PFE_WORLD" \
  gui:=false \
  verbose:=false &
GZ_PID=$!
sleep "${CLOUD_GZ_BOOT_SLEEP:-12}"
kill -0 "$GZ_PID" 2>/dev/null || _die "Gazebo exited early — check ROS logs."

echo "── [spawn TurtleBot3 + train] ──"
ros2 run gazebo_ros spawn_entity.py -timeout 120 \
  -entity my_robot \
  -file "$TURTLEBOT_SDF" \
  -x "$_spawn_x" -y "$_spawn_y" -z "$_spawn_z" || echo "spawn may warn if entity exists"

sleep 2

brain_args=(python3 main_agent.py)
if ((${#EXTRA_BRAIN[@]})); then
  brain_args+=("${EXTRA_BRAIN[@]}")
else
  brain_args+=(--preset pfe_sac_waypoint --train-seed "${PFE_TRAIN_SEED:-42}")
  brain_args+=(--reset-fire-and-forget --reset-service-wait-sec "${PFE_RESET_SERVICE_WAIT:-12}")
fi
[[ -n "${PFE_ENV_STEP_SLEEP:-}" ]] \
  && brain_args+=(--env-step-sleep-sec "$PFE_ENV_STEP_SLEEP")

cd "$AGENT_DIR"
exec "${brain_args[@]}"
