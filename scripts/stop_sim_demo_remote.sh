#!/usr/bin/env bash
# Stop upload_server, ws_bridge, and SSH reverse tunnel (if running).
set -eo pipefail

WS_PATH="${WS_PATH:-$HOME/ros2_ws}"
WS_PORT="${SIM_WS_LOCAL_PORT:-9091}"
API_PORT="${SIM_DEMO_API_PORT:-5001}"

pkill -f "sim_demo_reverse_tunnel.sh" 2>/dev/null || true
pkill -f "ssh.*-R 127.0.0.1:${API_PORT}" 2>/dev/null || true
pkill -f "sim_demo_reverse_tunnel.sh" 2>/dev/null || true
pkill -f "upload_telemetry_sidecar.py" 2>/dev/null || true
pkill -f "upload_server.py" 2>/dev/null || true
pkill -f "ws_bridge.py" 2>/dev/null || true
pkill -f "ws_bridge_node" 2>/dev/null || true

sleep 1
if command -v fuser >/dev/null 2>&1; then
  fuser -k "${API_PORT}/tcp" 2>/dev/null || true
  fuser -k "${WS_PORT}/tcp" 2>/dev/null || true
  sleep 1
fi

echo "Stopped upload_server, ws_bridge, and reverse tunnel."
