#!/usr/bin/env bash
# Capture Gazebo / RViz / full-screen for thesis assets.
# Usage:
#   capture_thesis_screenshot.sh <output.png> [gazebo|rviz|screen]
# Writes <stem>_view.png (cropped Gazebo viewport) when ImageMagick is available.
set -euo pipefail

out="${1:?output path required}"
mode="${2:-gazebo}"
mkdir -p "$(dirname "$out")"

_capture_root() {
    if command -v gnome-screenshot >/dev/null 2>&1; then
        gnome-screenshot -f "$out"
        return 0
    fi
    if command -v scrot >/dev/null 2>&1; then
        scrot "$out"
        return 0
    fi
    if command -v import >/dev/null 2>&1; then
        import -window root "$out"
        return 0
    fi
    return 1
}

_capture_window_substr() {
    local sub="$1"
    if ! command -v xdotool >/dev/null 2>&1 || ! command -v import >/dev/null 2>&1; then
        return 1
    fi
    local wid
    wid="$(xdotool search --name "$sub" 2>/dev/null | head -1 || true)"
    [[ -n "$wid" ]] || return 1
    import -window "$wid" "$out"
}

_make_view_crop() {
    local src="$1"
    local base="${src%.png}"
    local view="${base}_view.png"
    if ! command -v convert >/dev/null 2>&1 || [[ ! -s "$src" ]]; then
        return 0
    fi
    # Stacked desktop capture: keep lower band (Gazebo). Single-window: mild center crop.
    if identify -format '%h' "$src" 2>/dev/null | awk -v h="$(identify -format '%h' "$src" 2>/dev/null)" 'BEGIN{exit 0}'; then
        local h
        h="$(identify -format '%h' "$src" 2>/dev/null || echo 0)"
        if [[ "$h" -gt 1400 ]]; then
            convert "$src" -gravity South -crop 100x58%+0+0 +repage "$view"
        else
            convert "$src" -gravity Center -crop 100x88%+0+0 +repage "$view"
        fi
        echo "[capture] view → $view"
    fi
}

case "$mode" in
    gazebo)
        _capture_window_substr "Gazebo" || _capture_root
        ;;
    rviz)
        _capture_window_substr "RViz" || _capture_window_substr "rviz" || _capture_root
        ;;
    screen|*)
        _capture_root
        ;;
esac

if [[ ! -s "$out" ]]; then
    echo "[capture] FAILED: $out (install gnome-screenshot, scrot+imagemagick, or xdotool+imagemagick)" >&2
    exit 1
fi
_make_view_crop "$out"
echo "[capture] OK → $out ($(du -h "$out" | awk '{print $1}'))"
echo "[capture] Thesis uses ${out%.png}_view.png when present (one window, no desktop clutter)."
