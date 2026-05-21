#!/usr/bin/env bash
# Quick smoke test: syntax + import main modules (no Gazebo).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/pfe_gazebo_env.sh"
pfe_export_gazebo_runtime_env

echo "── bash -n launchers ──"
for f in train_menu.sh start_pfe.sh train_waypoint.sh run_auto_train.sh pfe_gazebo_env.sh; do
  bash -n "$SCRIPT_DIR/$f"
  echo "  OK $f"
done

echo "── python imports ──"
cd "$SCRIPT_DIR"
python3 -c "
import main_agent, evaluate_agent, hot_swap_eval_node, training_contract
print('  OK imports')
"
echo "auto_test passed."
