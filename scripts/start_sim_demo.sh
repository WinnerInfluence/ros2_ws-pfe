#!/usr/bin/env bash
# start_sim_demo.sh — headless Gazebo + ws_bridge + upload_server (tmux: sim_demo)
set -euo pipefail

WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
ENV_FILE="${WS_PATH}/sim_demo.env"
SESSION="${SIM_DEMO_TMUX_SESSION:-sim_demo}"

if [[ -f "$ENV_FILE" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
AGENT_DIR="${WS_PATH}/src/safe_drl_nav/safe_drl_nav"
BRIDGE_PY="${AGENT_DIR}/ws_bridge.py"
UPLOAD_PY="${WS_PATH}/upload_server.py"

_die() { echo "error: $*" >&2; exit 1; }

[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || _die "ROS ${ROS_DISTRO} not found."
[[ -f "$WS_PATH/install/setup.bash" ]] || _die "Build workspace: cd $WS_PATH && colcon build --packages-select safe_drl_nav"
[[ -f "$BRIDGE_PY" ]] || _die "Missing $BRIDGE_PY"
[[ -f "$UPLOAD_PY" ]] || _die "Missing $UPLOAD_PY"
[[ -f "${TRAINING_CONTRACT:-}" ]] || _die "TRAINING_CONTRACT not found (set in sim_demo.env)"

if ! command -v tmux >/dev/null 2>&1; then
  _die "tmux required. Install: sudo apt install tmux"
fi

# Optional parity check before demo
CKPT="${AGENT_DIR}/trained_models/sac_actor_maze_best_ever.pth"
if [[ -f "$CKPT" ]]; then
  echo "── verify_cloud_readiness (SAC best_ever) ──"
  (
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    cd "$AGENT_DIR"
    python3 verify_cloud_readiness.py \
      --contract "$TRAINING_CONTRACT" \
      --algo sac \
      --checkpoint "$CKPT" \
  ) || echo "WARN: verify_cloud_readiness failed — fix before public demo."
else
  echo "WARN: $CKPT not found — upload eval still works for user .pth files."
fi

"${WS_PATH}/scripts/stop_sim_demo.sh" 2>/dev/null || true
sleep 1

ROS_SETUP="set +u; source /opt/ros/${ROS_DISTRO}/setup.bash; source ${WS_PATH}/install/setup.bash"
ENV_EXPORT="set -a; source ${ENV_FILE}; set +a"

tmux new-session -d -s "$SESSION" -n gazebo \
  "bash -lc '${ENV_EXPORT}; ${ROS_SETUP}; export QT_QPA_PLATFORM=minimal; unset DISPLAY; \
   ${WS_PATH}/scripts/_sim_demo_gazebo.sh; exec bash'"

tmux new-window -t "$SESSION" -n bridge \
  "bash -lc '${ENV_EXPORT}; ${ROS_SETUP}; \
   python3 ${BRIDGE_PY} --host \${SIM_DEMO_WS_HOST:-0.0.0.0} --port \${SIM_DEMO_WS_PORT:-9090}; exec bash'"

tmux new-window -t "$SESSION" -n api \
  "bash -lc '${ENV_EXPORT}; ${ROS_SETUP}; \
   python3 ${UPLOAD_PY} --host \${SIM_DEMO_API_HOST:-0.0.0.0} --port \${SIM_DEMO_API_PORT:-5001}; exec bash'"

sleep 3
SIM_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
PUBLIC_IP="$(curl -sf --max-time 3 https://api.ipify.org 2>/dev/null || true)"

echo ""
echo "══ SIM demo started (tmux session: ${SESSION}) ══"
echo "  tmux attach -t ${SESSION}"
echo "  Windows: gazebo | bridge | api"
echo ""
echo "  Local tests:"
echo "    curl -s http://127.0.0.1:${SIM_DEMO_API_PORT:-5001}/eval_status | head"
echo "    curl -s http://127.0.0.1:${SIM_DEMO_API_PORT:-5001}/health"
echo ""
echo "  SIM_HOST for nginx proxy_pass (optional public demo):"
echo "    ${SIM_IP:-<private-ip>}"
[[ -n "$PUBLIC_IP" ]] && echo "    public: ${PUBLIC_IP}"
echo ""
echo "  Firewall (only if exposing ports directly):"
echo "    sudo ufw allow from <PROXY_IP> to any port ${SIM_DEMO_API_PORT:-5001}"
echo "    sudo ufw allow from <PROXY_IP> to any port ${SIM_DEMO_WS_PORT:-9090}"
echo ""
