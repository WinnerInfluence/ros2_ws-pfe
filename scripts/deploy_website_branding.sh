#!/usr/bin/env bash
# Patch local website/ros/index.html and optionally upload to a VPS (set VPS_* env vars).
set -euo pipefail

WS="${WS_PATH:-$HOME/ros2_ws}"
# Set VPS_HOST, VPS_USER, SSH_KEY, REMOTE_DIR before running (no defaults in repo).
: "${VPS_HOST:?Set VPS_HOST}"
: "${VPS_USER:?Set VPS_USER}"
: "${SSH_KEY:?Set SSH_KEY to your SSH private key path}"
REMOTE_DIR="${REMOTE_DIR:-/var/www/html/ros}"

cd "$WS"
python3 scripts/patch_web_index_academic.py

echo "── Upload to ${VPS_USER}@${VPS_HOST}:${REMOTE_DIR} ──"
ssh -i "$SSH_KEY" -o StrictHostKeyChecking=accept-new "${VPS_USER}@${VPS_HOST}" "mkdir -p '${REMOTE_DIR}/demo_assets'"
scp -i "$SSH_KEY" -r website/ros/demo_assets/* "${VPS_USER}@${VPS_HOST}:${REMOTE_DIR}/demo_assets/"
scp -i "$SSH_KEY" website/ros/index.html "${VPS_USER}@${VPS_HOST}:${REMOTE_DIR}/index.html"

echo ""
echo "Done. Deployed to ${VPS_USER}@${VPS_HOST}:${REMOTE_DIR}"
echo "Hard-refresh (Ctrl+Shift+R) if you still see the old header."
