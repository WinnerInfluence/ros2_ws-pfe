#!/usr/bin/env bash
# Rabat eval: Gazebo + robot spawn + Phase-3 SAC (for thesis video / recording).
#
#   bash ~/ros2_ws/scripts/eval_rabat_fast.sh
#
# Long run (many episodes, fast world, shorter step sleep by default):
#   bash ~/ros2_ws/scripts/eval_rabat_long_fast.sh
#
#   EPISODES=1 MAX_STEPS=2500 bash ~/ros2_ws/scripts/eval_rabat_fast.sh   # shorter clip
set -euo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
PKG="$WS/src/safe_drl_nav/safe_drl_nav"
ROS_DISTRO="${ROS_DISTRO:-humble}"
# Default: light world (same layout/waypoints; textured world freezes gzclient on laptops).
if [[ "${PFE_TEXTURED:-0}" == "1" ]]; then
    WORLD="$PKG/sim_assets/worlds/eval_rabat.world"
else
    WORLD="$PKG/sim_assets/worlds/eval_rabat_fast.world"
fi
MODEL="${PFE_MODEL:-$PKG/trained_models/sac_actor_phase3_rabat.pth}"
MAT="$PKG/sim_assets/materials/scripts"
LOG_DIR="$PKG/pfe_logs"

EPISODES="${EPISODES:-3}"
ENV_STEP_SLEEP="${ENV_STEP_SLEEP:-0.05}"
MAX_STEPS="${MAX_STEPS:-4000}"
POST_SLEEP="${POST_SLEEP:-25}"
SPAWN_X="${SPAWN_X:--2.0}"
SPAWN_Y="${SPAWN_Y:--2.0}"
SPAWN_Z="${SPAWN_Z:-0.15}"
TAG="${TAG:-rabat_phase3_fast}"

export SPAWN_X SPAWN_Y SPAWN_Z
export PFE_ROBOT_ENTITY="${PFE_ROBOT_ENTITY:-my_robot}"
if [[ "${PFE_HIDE_LIDAR:-1}" == "1" ]]; then
    TURTLEBOT_SDF="$(bash "$WS/scripts/pfe_turtlebot_no_lidar_sdf.sh")"
    export TURTLEBOT_SDF
else
    export TURTLEBOT_SDF="/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
fi

# shellcheck source=/dev/null
source "$PKG/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env
export PFE_GAZEBO_GUI=1

[[ -f "$WORLD" ]] || { echo "[ERROR] missing $WORLD"; exit 1; }
[[ -f "$MODEL" ]] || { echo "[ERROR] missing $MODEL"; exit 1; }

WORLD="$(readlink -f "$WORLD")"
export PFE_WORLD="$WORLD"
mkdir -p "$LOG_DIR"

if [[ -z "${DISPLAY:-}" ]]; then
    echo "[ERROR] DISPLAY not set — use a normal desktop terminal."
    exit 1
fi

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
set -u

echo "[eval-rabat] Stopping stale Gazebo / eval (frees port 11345)…"
pkill -9 -f evaluate_agent.py 2>/dev/null || true
pfe_kill_gazebo
sleep 3

pfe_gazebo_source_all
pfe_gazebo_prepend_resource_path "$MAT"

echo "[eval-rabat] Starting gzserver (split launch — avoids Gazebo GUI freeze)…"
pfe_gazebo_start_server_bg "$WORLD" >/dev/null
if ! pfe_gazebo_wait_gzserver 120; then
    echo "[ERROR] gzserver did not start."
    exit 1
fi

if [[ "${PFE_NO_GUI:-0}" == "1" ]]; then
    echo "[eval-rabat] PFE_NO_GUI=1 — skipping gzclient (metrics only)."
else
    echo "[eval-rabat] Starting gzclient…"
    pfe_gazebo_start_gzclient_bg
fi

echo "[eval-rabat] Starting /reset_simulation helper…"
python3 "$PKG/pfe_reset_simulation.py" &
RESET_PID=$!
sleep 2

echo "[eval-rabat] Waiting for /reset_simulation and /set_entity_state…"
t=0
while (( t < 90 )); do
    if ros2 service list 2>/dev/null | grep -q '/reset_simulation' \
        && ros2 service list 2>/dev/null | grep -q '/set_entity_state'; then
        echo "[eval-rabat] reset + teleport services ready."
        break
    fi
    sleep 2
    t=$((t + 2))
done
if ! ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
    echo "[ERROR] /reset_simulation missing — is pfe_reset_simulation.py running?"
    exit 2
fi
if ! ros2 service list 2>/dev/null | grep -q '/set_entity_state'; then
    echo "[ERROR] /set_entity_state missing — regenerate eval_rabat.world (gazebo_ros_state plugin)."
    exit 2
fi

echo "[eval-rabat] Spawning TurtleBot at ($SPAWN_X, $SPAWN_Y)…"
if ! ros2 run gazebo_ros spawn_entity.py -timeout 120 -entity "$PFE_ROBOT_ENTITY" \
    -file "$TURTLEBOT_SDF" -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z"; then
    echo "[ERROR] spawn_entity failed."
    exit 3
fi
sleep 3

if ! ros2 topic list 2>/dev/null | grep -q '^/scan$'; then
    echo "[WARN] /scan not visible yet — waiting 5s…"
    sleep 5
fi

est_min=$(( (EPISODES * MAX_STEPS * ENV_STEP_SLEEP / 60) + 1 ))
echo ""
echo "[eval-rabat] ▶ START RECORDING NOW (Ubuntu screen recorder)"
echo "[eval-rabat] ${EPISODES} episodes × ${MAX_STEPS} steps × ${ENV_STEP_SLEEP}s ≈ ${est_min}+ min"
echo "[eval-rabat] model=$(basename "$MODEL")"
echo ""

cd "$PKG"
python3 evaluate_agent.py \
    --algo sac \
    --model "$MODEL" \
    --episodes "$EPISODES" \
    --max-steps "$MAX_STEPS" \
    --env-step-sleep-sec "$ENV_STEP_SLEEP" \
    --waypoint-goal-radius 0.68 \
    --tag "$TAG" \
    2>&1 | tee "$LOG_DIR/eval_${TAG}.log"

json="$LOG_DIR/eval_${TAG}.json"
if [[ -f "$json" ]]; then
    python3 - "$json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
s = d["summary"]
print(f"\n=== RABAT EVAL ===")
print(f"  solved: {s['episodes_solved']}/{s['episodes_total']} ({100*s['success_rate']:.1f}%)")
print(f"  mean_wp: {s['mean_waypoints']} ± {s['std_waypoints']}")
print(f"  JSON: {sys.argv[1]}")
PY
fi

echo ""
echo "[eval-rabat] done — stop recording. To close sim: bash ~/ros2_ws/scripts/kill_gazebo.sh"
kill "$RESET_PID" 2>/dev/null || true
