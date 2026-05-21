#!/usr/bin/env bash
# Open TensorBoard URLs for manual screenshots — one scalar card per capture.
# Target look: single card, raw + smoothed lines, legend table (see assets/screenshots/_REFERENCE_tb_single_card.png)
set -euo pipefail

PORT="${TB_PORT:-6006}"
BASE="http://127.0.0.1:${PORT}"
WS="${WS_PATH:-$HOME/ros2_ws}"
LOGDIR="$WS/src/safe_drl_nav/safe_drl_nav/pfe_logs/thesis_tb_views"
SHOTS="$WS/pfe_report/thesis_pfe_pro/assets/screenshots"

enc() { python3 -c "import urllib.parse,sys; print(urllib.parse.quote(sys.argv[1]))" "$1"; }

# run|tag|save_as_hint
CARDS=(
  "01_phase1_sac_adapt_7776ep_11h|Reward|phase1_adaptation.png (main)"
  "02_lab_waypoints_5729ep_21h|ExecutionTime_ms|training_execution_time.png"
  "02_lab_waypoints_5729ep_21h|Reward|training_reward_curve.png"
  "03_maze_boost_latest|waypoint/progress_pct|tensorboard_waypoint_progress.png"
  "03_maze_boost_latest|waypoint/best_wp_so_far|phase2_waypoints (extra)"
  "03_maze_boost_latest|curriculum/effective_steps|phase2_waypoints (extra)"
  "03_maze_boost_latest|ExecutionTime_ms|phase2_waypoints (extra)"
  "03_maze_boost_latest|adapt/goal_x|like your reference screenshot"
  "03_maze_boost_latest|adapt/goal_y|phase1 caption: goal coordinates"
)

if ! curl -sf "${BASE}/data/runs" >/dev/null 2>&1; then
  echo "Start TensorBoard first:"
  echo "  bash ~/ros2_ws/scripts/setup_thesis_tensorboard_views.sh"
  echo "  tensorboard --logdir=${LOGDIR} --port=${PORT} --load_fast=false"
  exit 1
fi

echo "=== TensorBoard manual capture ==="
echo "Logdir: ${LOGDIR}"
echo "Save crops to: ${SHOTS}/"
echo ""
echo "Style (match _REFERENCE_tb_single_card.png):"
echo "  • TIME SERIES or SCALARS tab"
echo "  • Check ONLY the run named below (uncheck others)"
echo "  • Crop ONE card: title + plot + Smoothed/Value/Step/Relative table"
echo "  • Do NOT include left sidebar or full-page scroll"
echo "  • Smoothing slider ≈ 0.6 (default)"
echo ""

i=0
for spec in "${CARDS[@]}"; do
  run="${spec%%|*}"
  rest="${spec#*|}"
  tag="${rest%%|*}"
  hint="${rest#*|}"
  i=$((i + 1))
  tf=$(enc "$tag")
  url="${BASE}/#timeseries&tagFilter=${tf}"
  printf "%2d. Run: %s\n    Tag: %s\n    →  %s\n    Save as: %s\n\n" "$i" "$run" "$tag" "$url" "$hint"
done

if command -v xdg-open >/dev/null 2>&1 && [[ "${OPEN_FIRST:-}" == "1" ]]; then
  first="${CARDS[0]}"
  tag="${first#*|}"; tag="${tag%%|*}"
  xdg-open "${BASE}/#timeseries&tagFilter=$(enc "$tag")" 2>/dev/null || true
fi

echo "Optional: set OPEN_FIRST=1 to open the first URL in the browser."
