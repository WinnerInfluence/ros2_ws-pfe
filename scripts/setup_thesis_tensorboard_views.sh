#!/usr/bin/env bash
# One TensorBoard run = one training session (no merged restarts → no 2h trap / zig-zag).
set -euo pipefail

AGENT="${WS_PATH:-$HOME/ros2_ws}/src/safe_drl_nav/safe_drl_nav"
LOGS="$AGENT/pfe_logs"
OUT="$LOGS/thesis_tb_views"

pick_best_file() {
  python3 - "$1" "$2" <<'PY'
import sys
from pathlib import Path
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

tb, mode = Path(sys.argv[1]), sys.argv[2]
best, best_key = None, (-1.0, -1)

for f in sorted(tb.glob("events.out.tfevents*")):
    ea = EventAccumulator(str(f), size_guidance={"scalars": 0})
    ea.Reload()
    tags = ea.Tags().get("scalars", [])
    tag = "Reward" if "Reward" in tags else ("Reward/Episode" if "Reward/Episode" in tags else None)
    if not tag:
        continue
    pts = ea.Scalars(tag)
    if not pts:
        continue
    hrs = (pts[-1].wall_time - pts[0].wall_time) / 3600.0
    n = len(pts)
    if mode == "largest_n":
        key = (float(n), hrs)
    else:  # longest_wall
        key = (hrs, float(n))
    if key > best_key:
        best_key, best = key, f

print(best or "")
PY
}

wall_hours() {
  python3 - "$1" "$2" <<'PY'
import sys
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

ea = EventAccumulator(sys.argv[1], size_guidance={"scalars": 0})
ea.Reload()
tag = sys.argv[2]
if tag not in ea.Tags().get("scalars", []):
    print("0")
else:
    s = ea.Scalars(tag)
    print(f"{(s[-1].wall_time - s[0].wall_time) / 3600:.1f}")
PY
}

copy_run() {
  local name="$1" src="$2"
  local dir="$OUT/$name"
  mkdir -p "$dir"
  cp "$src" "$dir/events.out.tfevents.data"
  local hrs
  hrs=$(wall_hours "$src" Reward 2>/dev/null || wall_hours "$src" "Reward/Episode")
  echo "  $name  ←  $(basename "$src")  (~${hrs}h wall on Reward)"
}

mkdir -p "$OUT"
rm -rf "$OUT"/*

echo "Building $OUT ..."

# Phase 1 adapt (tb_sac): longest single session by episode count (7776 ep, ~11h)
f1=$(pick_best_file "$LOGS/tb_sac" largest_n)
# Phase 2 lab: largest single session in tb_sac_maze (5729 ep, ~21h) — NOT phase 1
f2=$(pick_best_file "$LOGS/tb_sac_maze" largest_n)
# Current maze boost: newest event file
f3=$(ls -t "$LOGS/tb_sac_maze"/events.out.tfevents* 2>/dev/null | sed -n '1p')

[[ -n "$f1" ]] && copy_run "01_phase1_sac_adapt_7776ep_11h" "$f1"
[[ -n "$f2" ]] && copy_run "02_lab_waypoints_5729ep_21h" "$f2"
[[ -n "$f3" && "$f3" != "$f2" ]] && copy_run "03_maze_boost_latest" "$f3"

echo ""
echo "Done. Start TensorBoard:"
echo "  tensorboard --logdir=$OUT --port 6006 --load_fast=false"
echo ""
echo "Runs (check ONE at a time for screenshots):"
echo "  01_phase1_sac_adapt_7776ep_11h  — SAC random-goal adaptation (~11h, 7776 ep)"
echo "  02_lab_waypoints_5729ep_21h     — Long lab waypoint training (~21h, 5729 ep)"
echo "  03_maze_boost_latest            — Current maze boost session"
echo ""
echo "Important:"
echo "  • ~21h training is run 02, not 01."
echo "  • On run 01, use scalar Reward (ExecutionTime_ms was only logged in a later 2h restart)."
echo "  • X-axis: Wall time (Relative), not Step."
