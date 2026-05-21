# Checkpoints (`trained_models/`)

| File | Role |
|------|------|
| `sac_actor_maze_best_ever.pth` | Lab global best (thesis 24/33, zeroshot) |
| `sac_actor_maze_best_ever_eval_maze.pth` | Maze peak (ep6, video / eval_maze) |
| `sac_actor_maze.pth` | Current training save (may drift) |
| `sac_actor_phase3_{rabat,egypt,maze}.pth` | Phase-3 fine-tunes |
| `thesis_locked_20260515/` | Frozen copy + manifests from good run |

Sidecar files (`.floor`, `.wp`, `.solved`) go with `best_ever` — keep them in git if present.

`backups/` and `archive/` are gitignored (regenerable).
