# Tour Hassan Rabat (`eval_rabat.world`)

**Geometry:** `hassan_floor` + pillar stumps (`p_0` …) + one `hassan_mosque` minaret + blue sky.  
**Excluded:** Egypt pyramids, dense eval forest, or tower texture on four walls.

Regenerate the world:

```bash
cd ~/ros2_ws/src/safe_drl_nav/safe_drl_nav/sim_assets/scripts
python3 -c "from generate_eval_worlds import gen_rabat; gen_rabat()"
```

---

# Hassan custom textures

## Asset files

Place source PNGs on `~/Desktop` (or set paths in `generate_eval_worlds.py`):

| File | Material | Appearance |
|------|----------|------------|
| `hassan_floor.png` | `Hassan/Floor` | Tiled stone pavers |
| `hassan_pillar.png` | `Hassan/Pillar` | Column ruins (`p_0` …) |
| `hassan_tower.png` | `Hassan/Tower` | White walls + backdrop tower |
| `hassan.material` | — | Ogre script (required) |

Sky uses Gazebo `<background>` and `<sky><clouds>` in the world file (no sky PNG).

## Regenerate after texture changes

```bash
cd ~/ros2_ws/src/safe_drl_nav/safe_drl_nav/sim_assets/scripts
python3 -c "from generate_eval_worlds import gen_rabat; gen_rabat()"
pkill -9 -f gzserver
bash ~/ros2_ws/scripts/local/record_video_rabat_now.sh
```

`gen_rabat()` copies PNGs from Desktop and updates `hassan.material` scale from image dimensions.

Optional resize (faster Gazebo load):

```bash
bash ~/ros2_ws/scripts/local/resize_hassan_textures.sh
```

**Recommended aspect ratios:** pillar ~1:3, floor tile ~2:1, tower ~4:5 or 3:4. Re-run `gen_rabat()` after replacing Desktop PNGs.
