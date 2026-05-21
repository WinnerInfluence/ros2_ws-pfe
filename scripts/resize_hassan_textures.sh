#!/usr/bin/env bash
# Shrink hassan PNGs so gzclient does not hang on "Preparing your world…"
# Backs up originals to *_full.png once, then overwrites with 1024px max edge.
set -euo pipefail
MS="$HOME/ros2_ws/src/safe_drl_nav/safe_drl_nav/sim_assets/materials/scripts"
MAX=1024
for f in hassan_floor.png hassan_pillar.png hassan_tower.png; do
  src="$MS/$f"
  [[ -f "$src" ]] || { echo "missing $src"; exit 1; }
  bak="$MS/${f%.png}_full.png"
  [[ -f "$bak" ]] || cp -a "$src" "$bak"
  convert "$src" -resize "${MAX}x${MAX}>" "$src"
  echo "  $f → $(identify -format '%wx%h %b' "$src")"
done
echo "Done. Re-run: cd sim_assets/scripts && python3 -c 'from generate_eval_worlds import gen_rabat; gen_rabat()'"
