#!/usr/bin/env bash
# Fail if critical project files are missing or zero bytes.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
AGENT="$ROOT/src/safe_drl_nav/safe_drl_nav"
PKG="$ROOT/src/safe_drl_nav"

must_exist() {
  local f="$1"
  if [[ ! -s "$f" ]]; then
    echo "FAIL (missing or empty): $f" >&2
    return 1
  fi
  echo "OK $f"
}

echo "── ROS package ──"
must_exist "$PKG/package.xml"
must_exist "$PKG/setup.py"
must_exist "$PKG/setup.cfg"

echo "── Core Python ──"
must_exist "$AGENT/main_agent.py"
must_exist "$AGENT/evaluate_agent.py"
must_exist "$AGENT/hot_swap_eval_node.py"
must_exist "$AGENT/training_contract.yaml"
must_exist "$AGENT/pfe_gazebo_env.sh"

echo "── Launchers ──"
must_exist "$AGENT/train_menu.sh"
must_exist "$AGENT/start_pfe.sh"
must_exist "$AGENT/train_waypoint.sh"

echo "── Checkpoints (recommended) ──"
for ck in \
  "$AGENT/trained_models/sac_actor_maze_best_ever.pth" \
  "$AGENT/trained_models/sac_actor_maze_best_ever_eval_maze.pth"
do
  if [[ -s "$ck" ]]; then
    echo "OK $ck"
  else
    echo "WARN missing: $ck" >&2
  fi
done

echo "All critical files present."
