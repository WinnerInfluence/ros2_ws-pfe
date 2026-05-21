#!/usr/bin/env bash
# Export scalar data using TensorBoard's own CSV API (not matplotlib / Playwright).
#
# Prerequisite: TensorBoard serving thesis_tb_views, e.g.:
#   bash ~/ros2_ws/scripts/setup_thesis_tensorboard_views.sh
#   tensorboard --logdir=.../pfe_logs/thesis_tb_views --port 6006 --load_fast=false
#
# Usage:
#   bash ~/ros2_ws/scripts/export_tensorboard_csv.sh
#   bash ~/ros2_ws/scripts/export_tensorboard_csv.sh --port 6006
set -euo pipefail

PORT="${TB_PORT:-6006}"
BASE="http://127.0.0.1:${PORT}"
WS="${WS_PATH:-$HOME/ros2_ws}"
OUT="$WS/pfe_report/thesis_pfe_pro/assets/tensorboard_csv"
LOGDIR="$WS/src/safe_drl_nav/safe_drl_nav/pfe_logs/thesis_tb_views"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; BASE="http://127.0.0.1:${PORT}"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,12p' "$0"
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

if ! curl -sf "${BASE}/data/runs" >/dev/null; then
  echo "TensorBoard not reachable at ${BASE}" >&2
  echo "Start it in another terminal:" >&2
  echo "  tensorboard --logdir=${LOGDIR} --port=${PORT} --load_fast=false" >&2
  exit 1
fi

mkdir -p "$OUT"

# run|tag — same sessions as thesis_tb_views
EXPORTS=(
  "01_phase1_sac_adapt_7776ep_11h|Reward"
  "02_lab_waypoints_5729ep_21h|Reward"
  "02_lab_waypoints_5729ep_21h|ExecutionTime_ms"
  "03_maze_boost_latest|Reward"
  "03_maze_boost_latest|ExecutionTime_ms"
  "03_maze_boost_latest|waypoint/progress_pct"
  "03_maze_boost_latest|waypoint/best_wp_so_far"
  "03_maze_boost_latest|curriculum/effective_steps"
  "03_maze_boost_latest|EpisodeSteps"
)

echo "TensorBoard CSV export → $OUT"
echo "Source: ${BASE}"
echo ""

for spec in "${EXPORTS[@]}"; do
  run="${spec%%|*}"
  tag="${spec#*|}"
  safe_tag="${tag//\//_}"
  dest="$OUT/${run}__${safe_tag}.csv"
  url="${BASE}/data/plugin/scalars/scalars?run=$(python3 -c "import urllib.parse; print(urllib.parse.quote('${run}'))")&tag=$(python3 -c "import urllib.parse; print(urllib.parse.quote('${tag}'))")&format=csv"
  if curl -sf "$url" -o "$dest"; then
    lines=$(wc -l < "$dest")
    echo "  OK  ${run} / ${tag}  →  $(basename "$dest")  (${lines} lines)"
  else
    echo "  SKIP  ${run} / ${tag}  (not in log)" >&2
    rm -f "$dest"
  fi
done

# Summary via tensorboard --inspect (built-in, no custom Python)
echo ""
echo "=== tensorboard --inspect (summary) ==="
tensorboard --logdir="$LOGDIR" --inspect 2>/dev/null | head -80 || true

echo ""
echo "Done. CSV columns: Wall time, Step, Value (TensorBoard native format)."
echo ""
echo "For thesis PNGs: screenshot the same runs in Firefox at ${BASE}"
echo "  SCALARS tab · one run checked · save to assets/screenshots/"
