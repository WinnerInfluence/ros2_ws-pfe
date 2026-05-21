#!/usr/bin/env bash
# Long Rabat eval on the FAST world (same layout / waypoints as eval_rabat / record;
# `eval_rabat_fast.world` — quicker load than textured `eval_rabat_record.world`).
#
# Use this to hunt for a full 3-waypoint "maze solved" episode without the record pipeline.
#
#   bash ~/ros2_ws/scripts/eval_rabat_long_fast.sh
#
# Tune (optional):
#   EPISODES=50 MAX_STEPS=4000 ENV_STEP_SLEEP=0.02 bash ~/ros2_ws/scripts/eval_rabat_long_fast.sh
#   PFE_MODEL="$HOME/ros2_ws/src/safe_drl_nav/safe_drl_nav/trained_models/sac_actor_phase3_rabat.pth" ...
#   PFE_TEXTURED=1   # use full eval_rabat.world (slower on laptops)
#   PFE_NO_GUI=1     # gzserver only — fastest
#
set -euo pipefail

export EPISODES="${EPISODES:-33}"
export MAX_STEPS="${MAX_STEPS:-4000}"
export ENV_STEP_SLEEP="${ENV_STEP_SLEEP:-0.03}"
export PFE_TEXTURED="${PFE_TEXTURED:-0}"
export TAG="${TAG:-rabat_long_fast}"

WS="${ROS2_WS:-$HOME/ros2_ws}"
exec bash "$WS/scripts/eval_rabat_fast.sh"
