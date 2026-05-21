#!/usr/bin/env bash
# Textured Tour Hassan + TurtleBot + SAC — optimized record path for laptops.
#   bash ~/ros2_ws/scripts/record_rabat_textured.sh
#
# Visuals: orange/light-brown plaza floor, no blue LiDAR fan in Gazebo.
# More chances to see all 3 waypoints: EPISODES=30 (benchmark: ~6%% full solve on rabat pth).
#   PFE_MODEL=.../sac_actor_phase3_rabat.pth   # Rabat-trained (default)
#   PFE_HIDE_LIDAR=0                           # show laser fan again
#
# Ubuntu screen recorder: start when you see "▶ START RECORDING NOW".
set -euo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
PKG="$WS/src/safe_drl_nav/safe_drl_nav"
SCRIPTS="${PKG}/sim_assets/scripts"
ROS_DISTRO="${ROS_DISTRO:-humble}"
MAT="$PKG/sim_assets/materials/scripts"
WORLD="$PKG/sim_assets/worlds/eval_rabat_record.world"
MODEL="${PFE_MODEL:-$PKG/trained_models/sac_actor_phase3_rabat.pth}"
LOG_DIR="$PKG/pfe_logs"

EPISODES="${EPISODES:-10}"
MAX_STEPS="${MAX_STEPS:-4000}"
ENV_STEP_SLEEP="${ENV_STEP_SLEEP:-0.05}"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"
TAG="${TAG:-rabat_textured_record}"

export SPAWN_X SPAWN_Y SPAWN_Z PFE_ROBOT_ENTITY="${PFE_ROBOT_ENTITY:-my_robot}"
if [[ "${PFE_HIDE_LIDAR:-1}" == "1" ]]; then
  TURTLEBOT_SDF="$(bash "$WS/scripts/pfe_turtlebot_no_lidar_sdf.sh")"
  export TURTLEBOT_SDF
else
  export TURTLEBOT_SDF="/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
fi
export PFE_TEXTURED=1
export PFE_FLOOR_PAVEMENT="${PFE_FLOOR_PAVEMENT:-1}"
export PFE_GZCLIENT_DELAY_SEC="${PFE_GZCLIENT_DELAY_SEC:-8}"
export PFE_GAZEBO_WAIT="${PFE_GAZEBO_WAIT:-180}"
export PFE_SERVER_WARMUP_SEC="${PFE_SERVER_WARMUP_SEC:-15}"

[[ -z "${DISPLAY:-}" ]] && { echo "[ERROR] Need a desktop session (DISPLAY)."; exit 1; }

# shellcheck source=/dev/null
source "$PKG/pfe_gazebo_env.sh"

echo "[record] Regenerating eval_rabat_record.world (textures, no procedural sky)…"
(
    cd "$SCRIPTS"
    python3 -c "from generate_eval_worlds import gen_rabat_record; gen_rabat_record()"
) || { echo "[WARN] world gen failed — using existing eval_rabat_record.world"; }

if [[ ! -f "$WORLD" ]]; then
    WORLD="$PKG/sim_assets/worlds/eval_rabat.world"
    echo "[record] fallback world: $WORLD"
fi
[[ -f "$MODEL" ]] || { echo "[ERROR] missing model $MODEL"; exit 1; }

WORLD="$(readlink -f "$WORLD")"
export PFE_WORLD="$WORLD"
mkdir -p "$LOG_DIR"

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
set -u

pfe_gazebo_record_env
pfe_kill_gazebo
sleep 2

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
pfe_gazebo_ros_daemon_refresh

pfe_gazebo_source_all
pfe_gazebo_prepend_resource_path "$MAT"

echo "[record] (1/5) gzserver — textured Rabat…"
pfe_gazebo_start_server_bg "$WORLD" >/dev/null
if ! pfe_gazebo_wait_gzserver "$PFE_GAZEBO_WAIT"; then
    echo "[ERROR] gzserver failed (textured load often needs 60–90s; try: PFE_GAZEBO_WAIT=240 …)"
    exit 1
fi
pfe_gazebo_ros_daemon_refresh
pfe_gazebo_server_warmup "$PFE_SERVER_WARMUP_SEC"

echo "[record] (2/5) reset service…"
python3 "$PKG/pfe_reset_simulation.py" &
RESET_PID=$!
sleep 2
t=0
while (( t < 60 )); do
    if ros2 service list 2>/dev/null | grep -q '/reset_simulation' \
        && ros2 service list 2>/dev/null | grep -q '/spawn_entity'; then
        break
    fi
    sleep 2
    t=$((t + 2))
done

echo "[record] (3/5) spawn TurtleBot…"
echo "  entity=$PFE_ROBOT_ENTITY  sdf=$TURTLEBOT_SDF"
SPAWN_TO="${PFE_SPAWN_TIMEOUT:-180}"
ros2 run gazebo_ros spawn_entity.py -timeout "$SPAWN_TO" -entity "$PFE_ROBOT_ENTITY" \
    -file "$TURTLEBOT_SDF" -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z"
t=0
while (( t < 90 )); do
    ros2 topic list 2>/dev/null | grep -q '^/scan$' && break
    sleep 2
    t=$((t + 2))
done
ros2 topic list 2>/dev/null | grep -q '^/scan$' || echo "[WARN] /scan not up yet — eval may still work after gzclient"

echo "[record] (4/5) gzclient (robot already in sim — do not open GUI before spawn)…"
pfe_gazebo_start_gzclient_bg
sleep 6

echo ""
echo "══════════════════════════════════════════════════════════"
echo "  ▶ START RECORDING NOW (Ubuntu screen recorder)"
echo "  World: $(basename "$WORLD")  |  ep=$EPISODES  steps=$MAX_STEPS"
echo "  Model: $(basename "$MODEL")  |  LiDAR vis: $([[ "${PFE_HIDE_LIDAR:-1}" == 1 ]] && echo off || echo on)"
echo "══════════════════════════════════════════════════════════"
echo ""

echo "[record] (5/5) SAC eval (same settings as Phase-3 Rabat training: shield ON, SAC sample)…"
cd "$PKG"
python3 evaluate_agent.py \
    --algo sac \
    --model "$MODEL" \
    --episodes "$EPISODES" \
    --max-steps "$MAX_STEPS" \
    --env-step-sleep-sec "$ENV_STEP_SLEEP" \
    --waypoint-goal-radius 0.68 \
    --reset-wait-for-reply \
    --reset-reply-wait-sec 90 \
    --tag "$TAG" \
    2>&1 | tee "$LOG_DIR/eval_${TAG}.log"

echo ""
echo "[record] done — stop recording. Close sim: bash ~/ros2_ws/scripts/kill_gazebo.sh"
if [[ -n "${RESET_PID:-}" ]] && kill -0 "$RESET_PID" 2>/dev/null; then
    kill "$RESET_PID" 2>/dev/null || true
    sleep 1
fi
