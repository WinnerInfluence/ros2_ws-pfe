"""Static maze layout for web dashboard (matches generate_eval_worlds corridor_maze)."""
from __future__ import annotations

MAZE_ENV_NAME = "Maze · 3 waypoints"

# Center (x, y), width, height — same frame as index.html drawWorld rects.
MAZE_OBSTACLES: list[dict] = [
    {"x": -4.10, "y": -1.00, "w": 1.80, "h": 0.2},
    {"x": -1.10, "y": -1.00, "w": 2.20, "h": 0.2},
    {"x": -4.50, "y": 0.50, "w": 1.00, "h": 0.2},
    {"x": -1.25, "y": 0.50, "w": 2.50, "h": 0.2},
    {"x": -4.25, "y": 2.20, "w": 1.50, "h": 0.2},
    {"x": -1.75, "y": 2.20, "w": 1.50, "h": 0.2},
    {"x": -0.50, "y": -0.25, "w": 0.2, "h": 1.50},
    {"x": -2.00, "y": 1.35, "w": 0.2, "h": 1.70},
    {"x": -4.50, "y": -2.60, "w": 0.2, "h": 1.50},
    {"x": -0.50, "y": -2.80, "w": 2.00, "h": 0.2},
    {"x": -1.25, "y": 1.40, "w": 0.2, "h": 0.90},
]

MAZE_WAYPOINTS: list[dict] = [
    {"x": -2.8, "y": -1.8, "n": 1},
    {"x": -4.5, "y": -0.2, "n": 2},
    {"x": -3.2, "y": 1.2, "n": 3},
]
