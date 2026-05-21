#!/usr/bin/env bash
# Start upload API, ws_bridge, and optional SSH reverse tunnel to a public web proxy.
set -euo pipefail

WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
cd "$WS_PATH"
mkdir -p pfe_logs

set -a
# shellcheck disable=SC1091
source sim_demo.env
set +a

export SIM_WS_LOCAL_PORT="${SIM_WS_LOCAL_PORT:-9091}"

_source_ros() {
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  # shellcheck disable=SC1091
  [[ -f install/setup.bash ]] && source install/setup.bash
  set -u
}
_source_ros

pkill -f "upload_telemetry_sidecar.py" 2>/dev/null || true
pkill -f "upload_server.py" 2>/dev/null || true
pkill -f "ws_bridge.py" 2>/dev/null || true
pkill -f "sim_demo_reverse_tunnel.sh" 2>/dev/null || true
sleep 1
if ss -tln 2>/dev/null | grep -qE ":${SIM_WS_LOCAL_PORT}[[:space:]]"; then
  echo "  freeing port ${SIM_WS_LOCAL_PORT} (stale ws_bridge)…"
  fuser -k "${SIM_WS_LOCAL_PORT}/tcp" 2>/dev/null || true
  sleep 1
fi

export TELEMETRY_FILE="${TELEMETRY_FILE:-$WS_PATH/pfe_logs/telemetry_live.json}"
# hot_swap_eval writes telemetry_live.json; sidecar would overwrite ep/step with zeros.
if [[ "${WEB_SKIP_TELEMETRY_SIDECAR:-0}" != "1" ]]; then
  nohup bash -c '
    set +u
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
    [[ -f "'"$WS_PATH"'/install/setup.bash" ]] && source "'"$WS_PATH"'/install/setup.bash"
    set -u
    export TELEMETRY_FILE="'"$TELEMETRY_FILE"'"
    export WS_PATH="'"$WS_PATH"'"
    exec python3 "'"$WS_PATH"'/scripts/upload_telemetry_sidecar.py"
  ' >>"$WS_PATH/pfe_logs/telemetry_sidecar.log" 2>&1 &
else
  echo "  telemetry: hot_swap only (sidecar skipped)"
fi
nohup bash -c '
  set +u
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  [[ -f "'"$WS_PATH"'/install/setup.bash" ]] && source "'"$WS_PATH"'/install/setup.bash"
  set -u
  export TELEMETRY_FILE="'"$TELEMETRY_FILE"'"
  exec python3 "'"$WS_PATH"'/upload_server.py" --host 127.0.0.1 --port 5001
' >>"$WS_PATH/pfe_logs/upload_server.log" 2>&1 &
nohup python3 src/safe_drl_nav/safe_drl_nav/ws_bridge.py \
  --host 127.0.0.1 --port "$SIM_WS_LOCAL_PORT" \
  >>"$WS_PATH/pfe_logs/ws_bridge.log" 2>&1 &

sleep 3
curl -sf http://127.0.0.1:5001/health >/dev/null || {
  echo "upload_server failed — see pfe_logs/upload_server.log"
  exit 1
}
if ! curl -sf http://127.0.0.1:5001/lidar_live | python3 -c "
import sys, json, time
d = json.load(sys.stdin)
age = time.time() - float(d.get('updated_at') or 0)
sys.exit(0 if d.get('ok') and not d.get('stale') and age < 5 else 1)
" 2>/dev/null; then
  echo "  WARN: lidar_live stale — start Gazebo on the simulation host, then re-run."
fi
if ! pgrep -f "ws_bridge.py.*port ${SIM_WS_LOCAL_PORT}" >/dev/null && \
   ! pgrep -f "ws_bridge.py --host 127.0.0.1 --port ${SIM_WS_LOCAL_PORT}" >/dev/null; then
  echo "ws_bridge failed — see pfe_logs/ws_bridge.log"
  tail -5 "$WS_PATH/pfe_logs/ws_bridge.log" 2>/dev/null || true
  exit 1
fi
echo "  ws_bridge listening on 127.0.0.1:${SIM_WS_LOCAL_PORT}"

nohup bash "$WS_PATH/scripts/sim_demo_reverse_tunnel.sh" \
  >>"$WS_PATH/pfe_logs/reverse_tunnel.log" 2>&1 &

sleep 2
echo "SIM demo remote stack started."
echo "  Local:  http://127.0.0.1:5001/health"
echo "  Public: (configure reverse proxy) → /ros/api/health"
echo "  Tunnel log: $WS_PATH/pfe_logs/reverse_tunnel.log"
echo "  Stop: $WS_PATH/scripts/stop_sim_demo_remote.sh"
