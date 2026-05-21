# Transfer demo video — storyboard (edit your on-screen text)

Record **one Gazebo clip per segment**, then cut together in your editor (CapCut, DaVinci, etc.) with **title cards** between clips.

**Launcher:** `bash ~/ros2_ws/scripts/record_transfer_video.sh`  
**Single world only:** `record_video_egypt_now.sh`, `record_video_now.sh` (maze).

---

## Recommended 5-part narrative (default script)

```bash
bash ~/ros2_ws/scripts/record_transfer_video.sh   # TRANSFER_SEGMENTS=5
```

| # | Segment | World | Checkpoint | Transfer | Thesis full-solve |
|---|---------|-------|------------|----------|-------------------|
| 1 | Training lab | `current_random_lab.world` | `sac_actor_maze_best_ever.pth` | In-distribution | ~24/33 |
| 2 | **Rabat (after Phase 3 train)** | `eval_rabat.world` | `sac_actor_phase3_rabat.pth` | Hard — pillar forest | **~2/33** |
| 3 | Egypt zero-shot | `eval_egypt.world` | `best_ever` | Medium | ~7/33 |
| 4 | Egypt fine-tuned | `eval_egypt.world` | `sac_actor_phase3_egypt.pth` | Medium+ | ~14/33 |
| 5 | Maze **slow pace** | `eval_maze.world` | `sac_actor_phase3_maze.pth` | Hardest | ~2/33 |

**Rabat is not missing from training** — you already have `sac_actor_phase3_rabat.pth` (Phase 3, ~250 ep). It was only missing from the video script; segment 2 shows that world.

**Maze for video:** segment 5 uses **slow** sim (`VIDEO_MAZE_STEP_SLEEP=0.09`, not 0.02). The robot moves slower on screen for a clear recording, not maximum solve speed.

### Rabat — what you can still do (optional)

| Goal | Command |
|------|---------|
| Re-film Rabat only | `bash ~/ros2_ws/scripts/record_video_rabat_now.sh` |
| Zero-shot Rabat clip (0/33, dramatic) | `EVAL_MODEL=trained_models/sac_actor_maze_best_ever.pth` + menu eval world Rabat |
| **Improve** Rabat demo (more training) | `cd safe_drl_nav && ONLY_WORLD=rabat PHASE3_EP=400 bash train_waypoint.sh` (hours) |

### Maze — slow recording only

```bash
bash ~/ros2_ws/scripts/record_video_maze_slow.sh
# Even slower: VIDEO_STEP_SLEEP=0.12 bash scripts/record_video_maze_slow.sh
```

## Shorter 4-part (no Rabat)

`TRANSFER_SEGMENTS=4 bash ~/ros2_ws/scripts/record_transfer_video.sh`

---

## Title card text (English — edit as needed)

### Card 1 — Training domain
```
Safe DRL Navigation — SAC + Control Barrier Shield
Phase 1: Random-goal adaptation (exploration warm-start)
Phase 2: Multi-waypoint curriculum on randomized training lab (domain randomization)
```

### Card 2 — Medium transfer (zero-shot)
```
Cross-world evaluation — ZERO-SHOT
Same checkpoint as training · New terrain (Egypt pyramids & pillars)
No world-specific fine-tuning
```

### Card 3 — After Phase 3 fine-tune
```
Cross-world evaluation — AFTER FINE-TUNING
Phase 3: Short adaptation on target world (eval_egypt)
Policy: sac_actor_phase3_egypt.pth
```

### Card 2 — Rabat (Phase 3 trained)
```
Rabat / Hassan pillar forest (eval_rabat.world)
Phase 3 fine-tune on this map — sac_actor_phase3_rabat.pth
Hardest pillar layout · modest full-solve rate (honest)
```

### Card 4 — Hard transfer (maze, slow demo)
```
Hard transfer — Hedge maze (eval_maze.world)
Phase 3 fine-tune · slow navigation for demonstration
(Not accelerated — watch the shield + LiDAR behaviour)
```

---

## French title cards (defense / PFE)

### Carte 1
```
Navigation DRL sécurisée — SAC + bouclier (CBF)
Phase 1 : adaptation objectif aléatoire (warm-start)
Phase 2 : waypoints WP1→WP2→WP3 sur labo d'entraînement randomisé
```

### Carte 2
```
Transfert inter-mondes — ZÉRO-SHOT
Même politique entraînée sur le labo · Nouveau décor (Égypte)
```

### Carte 3
```
Après fine-tuning Phase 3 sur le monde cible (Égypte)
```

### Carte 4
```
Transfert difficile — Labyrinthe (eval_maze)
```

---

## Recording tips

1. **Stable lab layout:** run segment 1 without re-running `randomize_world.py` that day, or copy a fixed `.world` so the “training” clip matches your thesis screenshots.
2. **Gazebo:** click `my_robot` → **F** (follow) → **View → uncheck Laser Scan**.
3. **Clip length:** 30–90 s per segment is enough; use `/reset_simulation` for a second good take in the same world.
4. **Numbers on screen:** optional small overlay: `24/33 lab`, `7/33 Egypt ZS`, `14/33 Egypt FT`, `2/33 maze` — cite `pfe_logs/eval_*.json` from your thesis run.
5. **Order in editor:** Title card → gameplay → title card → … → optional 10 s recap table.

---

## Shorter 3-world cut (no lab)

Use launcher: `TRANSFER_SEGMENTS=3 bash ~/ros2_ws/scripts/record_transfer_video.sh`  
→ Egypt zero-shot → Egypt phase3 → Maze phase3.

---

## Technical map (for you, not on video)

| JSON (thesis) | Model | Full solve |
|---------------|-------|------------|
| `eval_trainlab.json` | `sac_actor_maze_best_ever.pth` | 24/33 |
| `eval_sac_egypt_33_zeroshot.json` | best_ever | ~7/33 |
| `eval_sac_egypt_33_phase3.json` | phase3_egypt | ~14/33 |
| `eval_sac_maze_33_phase3.json` | phase3_maze | ~2/33 |
| `eval_sac_rabat_33_zeroshot.json` | best_ever | 0/33 |
