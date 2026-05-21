#!/usr/bin/env bash
# Unattended SAC waypoint training (lab). See run_auto_train.sh for options.
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$WS_DIR/src/safe_drl_nav/safe_drl_nav/run_auto_train.sh" "$@"
