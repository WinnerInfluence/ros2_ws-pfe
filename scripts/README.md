# Workspace scripts (public)

Thesis and supervisor-facing helpers only. Recording, web demo, cloud, and other operator scripts live under `scripts/local/` (gitignored).

| Script | Purpose |
|--------|---------|
| `run_thesis_crossworld_sac.sh` | Cross-world SAC eval (n=33, zeroshot + phase3) |
| `run_presentation_demo.sh` | Short Gazebo demo (`hot_swap_eval_node`, sample SAC) |
| `capture_thesis_screenshot.sh` | PNG capture for thesis figures |
| `print_thesis_eval_table.py` | Print eval JSON summary tables |
| `check_repo_integrity.sh` | Pre-push: critical files present |
