#!/usr/bin/env bash
# Wait for maze zeroshot eval to finish, stop the cross-world eval script (so it
# does not launch maze phase3 eval on top of training), then start overnight
# Phase 3 fine-tune on eval_maze.world.
#
# Usage:
#   nohup bash ~/ros2_ws/scripts/run_maze_train_after_zeroshot.sh >> ~/ros2_ws/pfe_logs/maze_overnight_queue.log 2>&1 &
#
# Env:
#   PHASE3_EP=800          (default 800)
#   STOP_EVAL_SCRIPT=1     kill run_thesis_crossworld_sac.sh after zeroshot (default 1)
#   POLL_SEC=30
set -euo pipefail

WS="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="$WS/src/safe_drl_nav/safe_drl_nav"
LOG_DIR="$AGENT_DIR/pfe_logs"
JSON="$LOG_DIR/eval_sac_maze_33_zeroshot.json"
PHASE3_EP="${PHASE3_EP:-800}"
STOP_EVAL_SCRIPT="${STOP_EVAL_SCRIPT:-1}"
POLL_SEC="${POLL_SEC:-30}"

_r() { echo -e "\033[1;31m$*\033[0m"; }
_g() { echo -e "\033[1;32m$*\033[0m"; }
_y() { echo -e "\033[1;33m$*\033[0m"; }

_json_complete() {
    [[ -f "$JSON" ]] || return 1
    python3 - "$JSON" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
s = d.get("summary", {})
ok = s.get("episodes_total") == 33 and len(d.get("episodes", [])) == 33
sys.exit(0 if ok else 1)
PY
}

echo "[$(date -Is)] Waiting for maze zeroshot eval (33/33) → $JSON"
while true; do
    if _json_complete; then
        _g "[$(date -Is)] Zeroshot JSON complete."
        break
    fi
    if pgrep -f 'evaluate_agent.py.*sac_maze_33_zeroshot' >/dev/null; then
        _y "[$(date -Is)] Still evaluating… (poll ${POLL_SEC}s)"
    elif [[ -f "$JSON" ]]; then
        _y "[$(date -Is)] JSON exists but not 33 episodes yet — waiting."
    else
        _y "[$(date -Is)] No JSON yet — waiting."
    fi
    sleep "$POLL_SEC"
done

if [[ "$STOP_EVAL_SCRIPT" == "1" ]]; then
    _y "[$(date -Is)] Stopping run_thesis_crossworld_sac.sh (avoids maze phase3 eval vs training)."
    pkill -f 'run_thesis_crossworld_sac.sh' 2>/dev/null || true
    sleep 5
    pkill -f 'evaluate_agent.py' 2>/dev/null || true
    sleep 3
    pkill -f 'gzserver' 2>/dev/null || true
    sleep 5
fi

while pgrep -f 'evaluate_agent.py|gzserver' >/dev/null; do
    _y "[$(date -Is)] Waiting for eval/Gazebo to exit…"
    sleep 5
done

_g "[$(date -Is)] Starting Phase 3 maze train (PHASE3_EP=$PHASE3_EP)."
export PFE_GAZEBO_GUI=0
export PHASE3_EP
cd "$AGENT_DIR"
exec bash train_waypoint.sh --phase 3 --world maze
