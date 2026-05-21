#!/usr/bin/env bash
# Rebuild clean TB views + export native TensorBoard CSV (recommended).
# PNG screenshots: do manually in Firefox — see assets/screenshots/README.md
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$SCRIPT_DIR/setup_thesis_tensorboard_views.sh"
bash "$SCRIPT_DIR/export_tensorboard_csv.sh" "$@"
echo ""
echo "Optional (not recommended for thesis figures):"
echo "  python3 $SCRIPT_DIR/export_tensorboard_screenshots.py --full"
