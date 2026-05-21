"""
generate_eval_world.py
======================
Generates  hassan_pyramid_eval.world  — the cinematic Zero-Shot Evaluation
environment for the Master's Thesis.

Theme : "The Hassan-Pyramid Labyrinth"
         Pillars of Rabat  ×  Egyptian Pyramids  ×  Maze Runner

Design decisions & engineering trade-offs
------------------------------------------
Visual quality vs. i5 performance
  * shadows=false  — single biggest frame-time win (~40 % GPU load saved).
  * All geometry is SDF primitives (box / cylinder / plane).
    No .dae / .stl mesh imports → zero mesh-loading overhead.
  * Two directional lights max (no point / spot lights, which are expensive).
  * Total model count ≤ 33.  Gazebo re-uses the same shader for every
    Gazebo/X material, so N primitives  is essentially free on the CPU.
  * physics @ 100 Hz ODE-quick solver — keeps stable contact without
    spending cycles on precise iterative solvers (50 iterations is enough
    for a wheeled robot on a flat plane).

Golden-Hour atmosphere
  * Scene ambient: warm ochre (0.45 0.30 0.15) tints all surfaces.
  * Primary sun: direction (-0.4, 0.25, -0.85) = low SW angle, warm 1.0/0.78/0.42.
  * Sky fill: soft overhead bounce (0.22 0.20 0.32) simulating atmospheric
    purple-blue scatter that contrasts the warm sun — this is what makes the
    golden-hour pop even without ray-traced GI.
  * Linear fog (starts 6 m, full at 15 m, warm sand colour): hides the world
    boundary walls, gives an infinite-desert illusion, adds depth cue for the
    pyramid silhouette.

Collision vs. visual separation strategy
  * Because every model uses a primitive, collision == visual.
    For the pyramid the four stacked boxes share the same primitive geometry
    for both tags (no separate low-poly hull needed).
  * LiDAR (2-D laser scan) only ever hits the collision mesh; the visual mesh
    complexity is irrelevant to the robot's perception.

Usage
-----
    python3 generate_eval_world.py               # writes to default path
    PFE_EVAL_WORLD=/tmp/test.world python3 ...   # custom output path

To launch:
    PFE_WORLD=<path>/hassan_pyramid_eval.world bash start_pfe.sh 1
or:
    python3 main_agent.py --preset pfe_sac_waypoint \\
        --load-pretrained trained_models/sac_actor_maze.pth
    # then set PFE_WORLD env-var before starting Gazebo.
"""
from __future__ import annotations
import os, math

# ---------------------------------------------------------------------------
# Output path
# ---------------------------------------------------------------------------
def _default_output() -> str:
    base   = os.path.dirname(os.path.abspath(__file__))
    worlds = os.path.normpath(os.path.join(base, "..", "worlds"))
    os.makedirs(worlds, exist_ok=True)
    override = os.environ.get("PFE_EVAL_WORLD", "").strip()
    return os.path.abspath(override) if override else os.path.join(worlds, "hassan_pyramid_eval.world")

# ---------------------------------------------------------------------------
# SDF primitive helpers  (collision = visual, same primitive both tags)
# ---------------------------------------------------------------------------
_MAT_URI = "file://media/materials/scripts/gazebo.material"

def _mat_block(name: str) -> str:
    return (
        f"          <material>\n"
        f"            <script>\n"
        f"              <uri>{_MAT_URI}</uri>\n"
        f"              <name>{name}</name>\n"
        f"            </script>\n"
        f"          </material>"
    )

def box_model(name: str, x: float, y: float, z: float,
              sx: float, sy: float, sz: float,
              mat: str = "Gazebo/Grey") -> str:
    geo = f"<box><size>{sx} {sy} {sz}</size></box>"
    m   = _mat_block(mat)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>{geo}</geometry>
        </collision>
        <visual name="visual">
          <geometry>{geo}</geometry>
{m}
        </visual>
      </link>
    </model>"""

def cylinder_model(name: str, x: float, y: float, z: float,
                   radius: float, length: float,
                   mat: str = "Gazebo/Grey") -> str:
    geo = f"<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>"
    m   = _mat_block(mat)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>{geo}</geometry>
        </collision>
        <visual name="visual">
          <geometry>{geo}</geometry>
{m}
        </visual>
      </link>
    </model>"""

# ---------------------------------------------------------------------------
# World generator
# ---------------------------------------------------------------------------

def generate_eval_world(output_path: str | None = None) -> str:
    path = output_path or _default_output()

    parts: list[str] = []

    # ====================================================================== #
    # 1.  PHYSICS — i5 optimised                                             #
    # ====================================================================== #
    parts.append("""
    <physics type="ode">
      <!-- 100 Hz physics, ODE-quick: stable for differential-drive robot    -->
      <!-- on a flat plane without wasting CPU on high-iter solvers.         -->
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>""")

    # ====================================================================== #
    # 2.  SCENE — Golden-Hour atmosphere                                     #
    # ====================================================================== #
    parts.append("""
    <!-- ── GOLDEN-HOUR ATMOSPHERE ─────────────────────────────────────── -->
    <!-- shadows=false  saves ~40 % render time on i5 with no quality loss  -->
    <!-- for a LiDAR-navigation thesis (the robot can't "see" shadows).     -->
    <scene>
      <ambient>0.45 0.30 0.15 1.0</ambient>
      <background>0.55 0.40 0.25 1.0</background>
      <shadows>false</shadows>
      <grid>false</grid>
      <fog>
        <!-- Warm desert haze: starts at 6 m, full opacity at 15 m.         -->
        <!-- Hides boundary edges, makes the pyramid appear to recede into   -->
        <!-- the distance — pure atmosphere with zero performance cost.      -->
        <color>0.72 0.57 0.37 1.0</color>
        <type>linear</type>
        <start>6.0</start>
        <end>15.0</end>
        <density>0.07</density>
      </fog>
    </scene>""")

    # ====================================================================== #
    # 3.  LIGHTING                                                           #
    # ====================================================================== #
    parts.append("""
    <!-- ── PRIMARY SUN (Golden Hour — low SW angle, warm 1.0/0.78/0.42) ── -->
    <light type="directional" name="golden_sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>1.00 0.78 0.42 1.0</diffuse>
      <specular>0.55 0.32 0.10 1.0</specular>
      <!-- direction: slightly south-west, low elevation — long shadow angle -->
      <direction>-0.40 0.25 -0.85</direction>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <!-- ── SKY FILL (atmospheric blue-purple bounce from the sky dome) ─── -->
    <!-- Contrasts warm sun → depth perception, makes materials "pop".      -->
    <light type="directional" name="sky_fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.22 0.20 0.32 1.0</diffuse>
      <specular>0.00 0.00 0.00 1.0</specular>
      <direction>0.10 -0.05 -1.00</direction>
      <attenuation>
        <range>1000</range><constant>1.0</constant>
        <linear>0.0</linear><quadratic>0.0</quadratic>
      </attenuation>
    </light>""")

    # ====================================================================== #
    # 4.  GROUND PLANE — warm desert sand                                   #
    # ====================================================================== #
    parts.append("""
    <!-- ── DESERT SAND FLOOR ──────────────────────────────────────────── -->
    <!-- The Gazebo/Grey material under warm golden ambient light appears   -->
    <!-- as a sun-baked sandy desert floor — no texture file required.      -->
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>22 22</size></plane></geometry>
          <surface><friction><ode>
            <mu>0.65</mu><mu2>0.65</mu2>
          </ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>22 22</size></plane></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>""")

    # ====================================================================== #
    # 5.  BOUNDARY WALLS — monumental stone perimeter                       #
    # ====================================================================== #
    # 3 m tall, 0.3 m thick — taller than training maze walls for immersion.
    WALL_H  = 3.0   # height
    WALL_T  = 0.3   # thickness
    WALL_HZ = WALL_H / 2.0   # z-centre
    BOUND   = 5.15  # world half-extent + half-thickness

    parts += [
        box_model("boundary_north",  0,  BOUND, WALL_HZ, 10.6,  WALL_T, WALL_H, "Gazebo/DarkGrey"),
        box_model("boundary_south",  0, -BOUND, WALL_HZ, 10.6,  WALL_T, WALL_H, "Gazebo/DarkGrey"),
        box_model("boundary_east",   BOUND,  0, WALL_HZ, WALL_T, 10.6,  WALL_H, "Gazebo/DarkGrey"),
        box_model("boundary_west",  -BOUND,  0, WALL_HZ, WALL_T, 10.6,  WALL_H, "Gazebo/DarkGrey"),
    ]

    # ====================================================================== #
    # 6.  EGYPTIAN PYRAMID — stacked boxes, positive quadrant landmark      #
    # ====================================================================== #
    # Stepped-mastaba style: 4 layers progressively smaller.
    # Collision is identical to visual (each box IS the physics shape).
    # The robot navigates in the negative-x zone; the pyramid is purely
    # visual / LiDAR depth landmark from 7+ m away.
    PYR_X, PYR_Y = 3.0, 3.5
    pyramid_layers = [
        # (z_centre, x_size, y_size, z_size, material)
        (0.80,  4.20, 4.20, 1.60, "Gazebo/Orange"),   # base
        (2.40,  2.90, 2.90, 1.60, "Gazebo/Orange"),   # l2
        (4.00,  1.70, 1.70, 1.60, "Gazebo/Orange"),   # l3
        (5.35,  0.60, 0.60, 0.70, "Gazebo/Yellow"),   # gold capstone
    ]
    for i, (pz, px, py_s, psz, pmat) in enumerate(pyramid_layers):
        parts.append(box_model(f"pyramid_l{i+1}", PYR_X, PYR_Y, pz, px, py_s, psz, pmat))

    # ====================================================================== #
    # 7.  OBELISK — Egyptian needle next to pyramid                         #
    # ====================================================================== #
    # 8 m tall, 0.38 m square base — the tallest object in the scene.
    # Creates vertical silhouette visible from anywhere in the arena.
    parts.append(box_model("obelisk", 1.1, 4.5, 4.0, 0.38, 0.38, 8.0, "Gazebo/DarkGrey"))
    # Obelisk pyramidion (small gold cap on top)
    parts.append(box_model("obelisk_cap", 1.1, 4.5, 8.2, 0.50, 0.50, 0.40, "Gazebo/Yellow"))

    # ====================================================================== #
    # 8.  HASSAN TOWER PILLAR FOREST                                        #
    # ====================================================================== #
    # 5 m tall, 0.28 m radius cylinders.
    # NAVIGATION ZONE: 5 pillars inside the maze corridor area — the robot
    # must weave between them (pure LiDAR obstacle challenge).
    # SHOWCASE ZONE:   7 pillars in the right half as visual "ruins" forest —
    # visible from afar, add depth and historical atmosphere.
    PILL_R  = 0.28   # radius
    PILL_H  = 5.00   # height
    PILL_Z  = 2.50   # z-centre = half-height

    PILLAR_POSITIONS = [
        # ── Navigation zone pillars (robot must route around these) ──────
        ("pillar_nav_1",  -2.5,   0.30),   # upper zone, near the corridor gap
        ("pillar_nav_2",  -3.5,   0.50),   # between WP2 and WP3 approach path
        ("pillar_nav_3",  -4.20,  1.50),   # far upper-left, near WP3 flank
        ("pillar_nav_4",  -1.80,  1.00),   # right side of upper zone
        ("pillar_nav_5",  -0.50,  0.50),   # near the gap-right entry
        # ── Showcase pillars (right half — visual depth, not navigation) ─
        ("pillar_dec_1",   1.00, -1.50),
        ("pillar_dec_2",   1.00,  0.50),
        ("pillar_dec_3",   1.00,  2.50),
        ("pillar_dec_4",   2.20, -1.00),
        ("pillar_dec_5",   2.20,  1.00),
        ("pillar_dec_6",   2.20,  3.00),
        ("pillar_dec_7",  -0.30, -3.50),   # lone pillar near south boundary
    ]
    for pname, px, py in PILLAR_POSITIONS:
        parts.append(cylinder_model(pname, px, py, PILL_Z, PILL_R, PILL_H, "Gazebo/Grey"))

    # ====================================================================== #
    # 9.  HEDGE MAZE WALLS — navigation challenge                           #
    # ====================================================================== #
    # Layout is deliberately DIFFERENT from the training maze (S-curve with
    # two long horizontals) to provide a genuine zero-shot test.
    #
    # Gap analysis — robot path (spawn → WP1 → WP2 → WP3):
    #   Spawn (-2.0,-2.0) → WP1 (-2.8,-1.8)   both in the LOWER zone   ✓
    #   WP1               → gap at x≈-2.25, y=-0.5 → upper zone         ✓
    #   gap               → WP2 (-4.5,-0.2)    move left in upper zone   ✓
    #   WP2               → WP3 (-3.2, 1.2)    diagonal up-right         ✓
    #
    # H1: Left horizontal  — x: -5.0 → -3.0  y=-0.5  seals the left side
    # H2: Right horizontal — x: -1.5 →  0.0  y=-0.5  seals the right side
    #   → GAP between H1 and H2: x=-3.0 to -1.5 (1.5 m wide)
    # H3: Upper horizontal — x: -5.0 → -1.5  y= 2.0  caps the upper zone
    # V1: Right vertical   — x=-1.5  y:-0.5 → 2.0    connects H2 to H3

    HW_T = 0.25   # thickness
    HW_H = 2.50   # height
    HW_Z = 1.25   # z-centre

    HEDGE_WALLS = [
        # name        cx      cy     cz     sx   sy    sz
        ("hedge_h1", -4.00, -0.50, HW_Z,  2.00, HW_T, HW_H),  # left horizontal
        ("hedge_h2", -0.75, -0.50, HW_Z,  1.50, HW_T, HW_H),  # right horizontal
        ("hedge_h3", -3.25,  2.00, HW_Z,  3.50, HW_T, HW_H),  # upper horizontal
        ("hedge_v1", -1.50,  0.75, HW_Z,  HW_T, 2.50, HW_H),  # right vertical
    ]
    for (wname, wx, wy, wz, wsx, wsy, wsz) in HEDGE_WALLS:
        parts.append(box_model(wname, wx, wy, wz, wsx, wsy, wsz, "Gazebo/Green"))

    # ====================================================================== #
    # 10. WAYPOINT RED CYLINDERS                                            #
    # ====================================================================== #
    # Same canonical coordinates as MAZE_WAYPOINTS in main_agent.py.
    # The robot was trained to reach these positions; zero-shot challenge is
    # that the SURROUNDING environment is completely different.
    WAYPOINTS = [
        ("waypoint_apex2",  -2.80, -1.80),  # WP1 — enemy_apex_2
        ("waypoint_wide1",  -4.50, -0.20),  # WP2 — enemy_wide_1
        ("waypoint_apex1",  -3.20,  1.20),  # WP3 — enemy_apex_1
    ]
    for wname, wx, wy in WAYPOINTS:
        parts.append(cylinder_model(wname, wx, wy, 0.5, 0.25, 1.0, "Gazebo/Red"))

    # ====================================================================== #
    # Assemble SDF                                                           #
    # ====================================================================== #
    body = "\n".join(parts)
    world_xml = f"""<?xml version="1.0"?>
<!--
  Hassan-Pyramid Labyrinth — Zero-Shot Evaluation World
  ======================================================
  Theme  : Pillars of Rabat × Egyptian Pyramids × Maze Runner
  Lighting: Golden Hour (low SW sun + atmospheric sky fill + desert fog)
  Physics: ODE-quick 100 Hz (i5 optimised, no shadow rendering)
  Models : {len(parts) - 5} structural + 3 waypoints + 4 boundary walls
  Author : Generated by generate_eval_world.py
  Usage  : PFE_WORLD=<path>/hassan_pyramid_eval.world bash start_pfe.sh 1
-->
<sdf version="1.6">
  <world name="hassan_pyramid_labyrinth">
{body}

  </world>
</sdf>
"""

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(world_xml)

    n_models = sum(1 for p in parts if "<model name=" in p)
    print(f"✅ Hassan-Pyramid Labyrinth written → {path}")
    print(f"   Models: {n_models} static models (shadows=false, 100 Hz ODE-quick)")
    print(f"   Waypoints: WP1(-2.8,-1.8)  WP2(-4.5,-0.2)  WP3(-3.2,1.2)")
    print(f"   Launch: PFE_WORLD={path!r} bash start_pfe.sh 1")
    return path


if __name__ == "__main__":
    generate_eval_world()
