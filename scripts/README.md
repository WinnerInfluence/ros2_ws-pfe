# Workspace scripts

| Script | Purpose |
|--------|---------|
| `run_thesis_crossworld_sac.sh` | Cross-world thesis eval (n=33) |
| `run_presentation_demo.sh` | Short Gazebo demo (hot_swap, sample) |
| `run_maze_peak_eval.sh` | Maze peak checkpoint eval |
| `eval_rabat_fast.sh` | Rabat eval helper (fast world by default; `PFE_TEXTURED=1` for original world) |
| `eval_rabat_long_fast.sh` | Longer Rabat rollouts (defaults: more episodes, faster wall-clock) |
| `record_rabat_textured.sh` | Textured Rabat world for recording (beautiful visuals) |
| `check_repo_integrity.sh` | Pre-push: no 0-byte critical files |
| `capture_thesis_screenshot.sh` | PNG for thesis figures |
| `start_web_demo.sh` / `stop_web_demo.sh` | Web stack (optional) |
