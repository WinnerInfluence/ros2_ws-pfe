#!/usr/bin/env bash
# Maze-only Phase 3 fine-tune with scoped best_ever + longer train horizons.
# Saves: trained_models/sac_actor_maze_best_ever_eval_maze.pth
#        trained_models/sac_actor_phase3_maze.pth
#
# Usage:
#   bash ~/ros2_ws/scripts/train_maze_boost.sh
#   PHASE3_EP=1000 PRETRAINED_CKPT=.../sac_actor_phase3_maze.pth bash ...
set -euo pipefail

WS="${WS_PATH:-$HOME/ros2_ws}"
AGENT_DIR="$WS/src/safe_drl_nav/safe_drl_nav"
MODEL_DIR="$AGENT_DIR/trained_models"

export PFE_GAZEBO_GUI="${PFE_GAZEBO_GUI:-0}"
export PHASE3_EP="${PHASE3_EP:-1000}"
export PRETRAINED_CKPT="${PRETRAINED_CKPT:-$MODEL_DIR/sac_actor_phase3_maze.pth}"
if [[ ! -f "$PRETRAINED_CKPT" ]]; then
  PRETRAINED_CKPT="$MODEL_DIR/sac_actor_phase2_final.pth"
fi

echo "[$(date -Is)] maze boost: PHASE3_EP=$PHASE3_EP warm-start=$(basename "$PRETRAINED_CKPT")"
cd "$AGENT_DIR"
exec bash train_waypoint.sh --phase 3 --world maze --only
