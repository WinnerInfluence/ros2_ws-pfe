#!/usr/bin/env bash
# Maze scene for video — SLOW robot pace (cinematic), not fast 0.02s stepping.
#
#   cd ~/ros2_ws && bash scripts/record_video_maze_slow.sh
#
# Slower = easier to follow on camera. Even slower: VIDEO_STEP_SLEEP=0.12 bash ...
set -euo pipefail
export VIDEO_STEP_SLEEP="${VIDEO_STEP_SLEEP:-0.09}"
export VIDEO_CONTROL_PERIOD="${VIDEO_CONTROL_PERIOD:-0.09}"
exec bash "$(dirname "$0")/record_video_now.sh"
