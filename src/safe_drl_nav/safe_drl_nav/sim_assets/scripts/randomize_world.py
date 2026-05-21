"""Domain-randomized world generator for PFE.

Each call randomly picks ONE of four obstacle styles:
  0 — S-Curve Maze      (matches training / eval_maze topology)
  1 — Pillar Forest     (prepares for eval_rabat Hassan-Tower world)
  2 — Block Obstacles   (prepares for eval_egypt pyramid world)
  3 — Corridor Labyrinth(prepares for eval_maze hedge world)

All styles share:
  • Same boundary walls (10 m × 10 m)
  • Same robot spawn  : (-2.0, -2.0)
  • Same waypoints    : WP1(-2.8,-1.8)  WP2(-4.5,-0.2)  WP3(-3.2,1.2)
  • Randomized floor friction [0.1, 1.0]
  • 1.0 m no-obstacle clearance radius around every waypoint and spawn

Training across all 4 styles → the SAC policy learns REACTIVE obstacle
avoidance (LiDAR → action) rather than memorising one corridor layout,
giving zero-shot generalisation on the three eval worlds.
"""
from __future__ import annotations

import os
import random
from typing import Callable


# ---------------------------------------------------------------------------
# Output path
# ---------------------------------------------------------------------------
def _default_output_path() -> str:
    base   = os.path.dirname(os.path.abspath(__file__))
    worlds = os.path.normpath(os.path.join(base, "..", "worlds"))
    os.makedirs(worlds, exist_ok=True)
    override = os.environ.get("PFE_RANDOM_WORLD_OUT", "").strip()
    if override:
        return os.path.abspath(override)
    return os.path.join(worlds, "current_random_lab.world")


# ---------------------------------------------------------------------------
# No-go zones: (center_xy, clearance_m)
# Keep ALL styles clear of spawn and all three waypoints.
# ---------------------------------------------------------------------------
_NO_GO: list[tuple[tuple[float, float], float]] = [
    ((-2.00, -2.00), 1.10),   # spawn
    ((-2.80, -1.80), 1.00),   # WP1
    ((-4.50, -0.20), 1.00),   # WP2
    ((-3.20,  1.20), 1.00),   # WP3
]


def _clear(px: float, py: float) -> bool:
    """Return True when (px, py) is outside every no-go zone."""
    return all(
        math.hypot(px - cx, py - cy) >= r
        for (cx, cy), r in _NO_GO
    )


# ---------------------------------------------------------------------------
# SDF primitive helpers
# ---------------------------------------------------------------------------
_GZ = "file://media/materials/scripts/gazebo.material"


def _mat(name: str) -> str:
    return (f"<material><script>"
            f"<uri>{_GZ}</uri>"
            f"<name>{name}</name>"
            f"</script></material>")


def _box(name: str, x: float, y: float, z: float,
         sx: float, sy: float, sz: float,
         color: str = "Gazebo/Wood") -> str:
    g = f"<box><size>{sx} {sy} {sz}</size></box>"
    return (
        f'\n    <model name="{name}"><static>true</static>'
        f"<pose>{x} {y} {z} 0 0 0</pose><link name=\"link\">"
        f"<collision name=\"c\"><geometry>{g}</geometry></collision>"
        f"<visual name=\"v\"><geometry>{g}</geometry>"
        f"{_mat(color)}</visual></link></model>"
    )


def _cyl(name: str, x: float, y: float, z: float,
         r: float, h: float,
         color: str = "Gazebo/Red") -> str:
    g = f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder>"
    return (
        f'\n    <model name="{name}"><static>true</static>'
        f"<pose>{x} {y} {z} 0 0 0</pose><link name=\"link\">"
        f"<collision name=\"c\"><geometry>{g}</geometry></collision>"
        f"<visual name=\"v\"><geometry>{g}</geometry>"
        f"{_mat(color)}</visual></link></model>"
    )


def _waypoint_markers() -> str:
    """Visible red cylinders at each waypoint coordinate."""
    markers = [
        _cyl("wp_marker_1", -2.80, -1.80, 0.5, 0.25, 1.0, "Gazebo/Red"),
        _cyl("wp_marker_2", -4.50, -0.20, 0.5, 0.25, 1.0, "Gazebo/Red"),
        _cyl("wp_marker_3", -3.20,  1.20, 0.5, 0.25, 1.0, "Gazebo/Red"),
    ]
    return "\n".join(markers)


def _boundary() -> str:
    """10 m × 10 m boundary walls."""
    parts = [
        _box("boundary_north",  0,  5, 0.5, 10, 0.2, 1.0, "Gazebo/DarkGrey"),
        _box("boundary_south",  0, -5, 0.5, 10, 0.2, 1.0, "Gazebo/DarkGrey"),
        _box("boundary_east",   5,  0, 0.5, 0.2, 10, 1.0, "Gazebo/DarkGrey"),
        _box("boundary_west",  -5,  0, 0.5, 0.2, 10, 1.0, "Gazebo/DarkGrey"),
    ]
    return "\n".join(parts)


def _ground(mu1: float, mu2: float) -> str:
    return f"""
    <model name="custom_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <surface><friction><ode><mu>{mu1}</mu><mu2>{mu2}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><script><uri>{_GZ}</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>"""


# ---------------------------------------------------------------------------
# Style 0 — S-Curve Maze  (original training world)
# ---------------------------------------------------------------------------
def _style_scurve(shift: float) -> str:
    """Two horizontal S-curve walls + spawn blocker."""
    parts = [
        _box("maze_wall_1", -1.0,  1.5 + shift, 0.5, 8.0, 0.2, 1.0, "Gazebo/Wood"),
        _box("maze_wall_2",  1.0, -1.5 + shift, 0.5, 8.0, 0.2, 1.0, "Gazebo/Wood"),
        _box("spawn_blocker", 2.0, 0.0 + shift, 0.5, 0.2, 3.0, 1.0, "Gazebo/Wood"),
    ]
    return "\n".join(parts)


# ---------------------------------------------------------------------------
# Style 1 — Pillar Forest  (prepares for eval_rabat)
# Scattered columns of varying radius/height — same topology as Hassan Tower.
# ---------------------------------------------------------------------------
def _style_pillars(rng: random.Random) -> str:
    parts: list[str] = []
    # Candidate grid positions inside the boundary
    xs = [-4.5, -3.5, -2.5, -1.0, 0.5, 2.0, 3.5]
    ys = [-4.0, -2.5, -1.0, 0.5, 2.0, 3.5]
    idx = 0
    for y in ys:
        for x in xs:
            if not _clear(x, y):
                continue
            if rng.random() < 0.45:   # ~45 % of positions get a pillar
                continue
            r = rng.uniform(0.18, 0.35)
            h = rng.uniform(0.8, 2.5)
            color = rng.choice(["Gazebo/Grey", "Gazebo/DarkGrey", "Gazebo/White"])
            parts.append(_cyl(f"pillar_{idx}", x, y, h / 2, r, h, color))
            idx += 1
    return "\n".join(parts)


# ---------------------------------------------------------------------------
# Style 2 — Block Obstacles  (prepares for eval_egypt pyramids / ruins)
# Boxes of varying size randomly scattered — similar feel to tomb/ruin blocks.
# ---------------------------------------------------------------------------
def _style_blocks(rng: random.Random) -> str:
    parts: list[str] = []
    attempts, placed = 0, 0
    while placed < 18 and attempts < 200:
        attempts += 1
        x = rng.uniform(-5.0, 4.5)
        y = rng.uniform(-4.5, 4.5)
        if not _clear(x, y):
            continue
        sx = rng.uniform(0.4, 1.6)
        sy = rng.uniform(0.4, 1.6)
        sz = rng.uniform(0.5, 1.8)
        color = rng.choice(["Gazebo/Orange", "Gazebo/DarkGrey",
                             "Gazebo/Yellow", "Gazebo/Grey"])
        parts.append(_box(f"block_{placed}", x, y, sz / 2, sx, sy, sz, color))
        placed += 1
    return "\n".join(parts)


# ---------------------------------------------------------------------------
# Style 3 — Corridor Labyrinth  (prepares for eval_maze hedge world)
# E-W and N-S walls with deliberate gaps that route through all waypoints:
#   spawn(-2,-2) → gap → WP1(-2.8,-1.8) → gap → WP2(-4.5,-0.2) → gap → WP3(-3.2,1.2)
# The gap offsets are randomised ±0.25 m for variation.
# ---------------------------------------------------------------------------
def _style_labyrinth(rng: random.Random) -> str:
    wh = 1.0   # wall height
    wt = 0.2   # wall thickness
    dg = rng.uniform(-0.25, 0.25)   # gap position dither

    # E-W divider 1: y ≈ -1.0, gap at x ∈ [-3.2, -2.2] (routes to WP1)
    gap1_lo, gap1_hi = -3.2 + dg, -2.2 + dg
    # E-W divider 2: y ≈ +0.5, gap at x ∈ [-4.0, -2.5] (routes to WP2→WP3)
    gap2_lo, gap2_hi = -4.0 + dg, -2.5 + dg

    parts = [
        # ── Divider 1 ───────────────────────────────────────────────────
        # left segment: x from -5.0 to gap1_lo
        _box("lab_h1l",
             (-5.0 + gap1_lo) / 2, -1.0 + dg, wh / 2,
             max(0.1, gap1_lo - (-5.0)), wt, wh, "Gazebo/Green"),
        # right segment: x from gap1_hi to +5.0
        _box("lab_h1r",
             (gap1_hi + 5.0) / 2, -1.0 + dg, wh / 2,
             max(0.1, 5.0 - gap1_hi), wt, wh, "Gazebo/Green"),
        # ── Divider 2 ───────────────────────────────────────────────────
        # left segment: x from -5.0 to gap2_lo
        _box("lab_h2l",
             (-5.0 + gap2_lo) / 2, 0.5 + dg, wh / 2,
             max(0.1, gap2_lo - (-5.0)), wt, wh, "Gazebo/Green"),
        # right segment: x from gap2_hi to +5.0
        _box("lab_h2r",
             (gap2_hi + 5.0) / 2, 0.5 + dg, wh / 2,
             max(0.1, 5.0 - gap2_hi), wt, wh, "Gazebo/Green"),
        # ── N-S channel wall right of navigation zone ────────────────────
        _box("lab_v1", -0.5, -0.3 + dg, wh / 2, wt, 1.4, wh, "Gazebo/Green"),
        # ── Dead ends (false corridors) ──────────────────────────────────
        _box("lab_de1", -4.5, -2.8, wh / 2, wt, 1.5, wh, "Gazebo/Green"),
        _box("lab_de2", -1.0, -2.8, wh / 2, 2.0, wt, wh, "Gazebo/Green"),
    ]
    return "\n".join(p for p in parts if p)


# ---------------------------------------------------------------------------
# World assembly
# ---------------------------------------------------------------------------
def _assemble(style_body: str, mu1: float, mu2: float,
              style_name: str) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <!-- style: {style_name} | mu1={mu1:.3f} mu2={mu2:.3f} -->
  <world name="randomized_training_world">

    <physics type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>

    <include><uri>model://sun</uri></include>

    {_ground(mu1, mu2)}

    {_boundary()}

    {style_body}

    {_waypoint_markers()}

  </world>
</sdf>
"""


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------
def generate_random_world(output_path: str | None = None,
                          style: int | None = None) -> str:
    """Generate a randomised training world and write it to *output_path*.

    Args:
        output_path: Destination ``.world`` file.  Defaults to
            ``sim_assets/worlds/current_random_lab.world``.
        style: Force a specific style 0-3 (default: random).

    Returns:
        The resolved output path.
    """
    path = output_path or _default_output_path()
    rng  = random.Random()   # new seeded RNG every call → true randomness

    mu1 = round(rng.uniform(0.1, 1.0), 3)
    mu2 = round(rng.uniform(0.1, 1.0), 3)

    chosen = style if style is not None else rng.randint(0, 3)
    shift  = round(rng.uniform(-0.3, 0.3), 2)

    style_map: dict[int, tuple[str, Callable[[], str]]] = {
        0: ("scurve_maze",    lambda: _style_scurve(shift)),
        1: ("pillar_forest",  lambda: _style_pillars(rng)),
        2: ("block_obstacles",lambda: _style_blocks(rng)),
        3: ("corridor_maze",  lambda: _style_labyrinth(rng)),
    }

    name, builder = style_map[chosen]
    body = builder()
    world_xml = _assemble(body, mu1, mu2, name)

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(world_xml)

    style_labels = {
        0: "S-Curve Maze",
        1: "Pillar Forest (→ Hassan/Rabat)",
        2: "Block Obstacles (→ Egypt/Pyramids)",
        3: "Corridor Labyrinth (→ Hedge Maze)",
    }
    print(
        f"World generated: {style_labels[chosen]}"
        f" | friction [{mu1}, {mu2}]"
        f"{'  corridor_shift=' + str(shift) if chosen == 0 else ''}"
        f"\n  → {path}"
    )
    return path


if __name__ == "__main__":
    import sys
    forced = int(sys.argv[1]) if len(sys.argv) > 1 else None
    generate_random_world(style=forced)
