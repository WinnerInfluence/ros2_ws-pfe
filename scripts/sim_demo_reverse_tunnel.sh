#!/usr/bin/env bash
# SSH reverse tunnel: VPS 127.0.0.1:5001/9090 → this machine (upload_server + ws_bridge).
# Requires: upload_server on 127.0.0.1:5001, ws_bridge on 127.0.0.1:${SIM_WS_LOCAL_PORT:-9091}
set -euo pipefail

: "${VPS_HOST:?Set VPS_HOST (reverse-proxy server)}"
: "${VPS_USER:?Set VPS_USER}"
: "${SSH_KEY:?Set SSH_KEY to your SSH private key path}"
LOCAL_API_PORT="${LOCAL_API_PORT:-5001}"
LOCAL_WS_PORT="${SIM_WS_LOCAL_PORT:-9091}"
REMOTE_API_PORT="${REMOTE_API_PORT:-5001}"
REMOTE_WS_PORT="${REMOTE_WS_PORT:-9090}"

exec ssh -i "$SSH_KEY" \
  -o ServerAliveInterval=30 \
  -o ServerAliveCountMax=3 \
  -o ExitOnForwardFailure=yes \
  -N \
  -R "127.0.0.1:${REMOTE_API_PORT}:127.0.0.1:${LOCAL_API_PORT}" \
  -R "127.0.0.1:${REMOTE_WS_PORT}:127.0.0.1:${LOCAL_WS_PORT}" \
  "${VPS_USER}@${VPS_HOST}"
