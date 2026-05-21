#!/usr/bin/env bash
# Maze eval — same harness as thesis / run_thesis_crossworld_sac.sh (n=33).
# Default model: sac_actor_maze_best_ever_eval_maze.pth (ep-6 scoped peak).
set -euo pipefail

WS="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="$WS/src/safe_drl_nav/safe_drl_nav"
MODEL="${EVAL_MODEL:-$AGENT_DIR/trained_models/sac_actor_maze_best_ever_eval_maze.pth}"
TAG="${EVAL_TAG:-sac_maze_33_eval_maze_peak}"
EPISODES="${EPISODES:-33}"

if [[ ! -f "$MODEL" ]]; then
  echo "Missing: $MODEL" >&2
  exit 1
fi

echo "[eval] Stopping maze training if running…"
pkill -f "main_agent.py --preset pfe_sac_waypoint" 2>/dev/null || true
sleep 2

set +u
source /opt/ros/humble/setup.bash
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
set -u

export PFE_WORLD="${PFE_WORLD:-eval_maze}"

if ! ros2 service list 2>/dev/null | grep -q '/reset_simulation'; then
  echo "[eval] ERROR: Gazebo not ready (/reset_simulation missing)." >&2
  echo "  Start eval_maze, or: WORLDS=maze RUN_SET=zeroshot bash run_thesis_crossworld_sac.sh" >&2
  exit 2
fi

cd "$AGENT_DIR"
echo "[eval] n=$EPISODES  model=$(basename "$MODEL")  tag=$TAG"
echo "[eval] Harness matches run_thesis_crossworld_sac.sh (paper-eval, sleep=0.05, max-steps=4000, wp=0.68)"

exec python3 evaluate_agent.py --algo sac \
  --model "$MODEL" \
  --paper-eval --require-reset \
  --episodes "$EPISODES" \
  --max-steps 4000 \
  --env-step-sleep-sec 0.05 \
  --waypoint-goal-radius 0.68 \
  --tag "$TAG"
