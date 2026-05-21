#!/usr/bin/env python3
"""Print LaTeX-ready rows from eval_sac_*_33_*.json for thesis Table tab:world-transfer."""
from __future__ import annotations

import json
import glob
import os
import sys

LOG = os.path.expanduser(
    "~/ros2_ws/src/safe_drl_nav/safe_drl_nav/pfe_logs"
)


def main() -> None:
    paths = sorted(glob.glob(os.path.join(LOG, "eval_sac_*_33_*.json")))
    if not paths:
        print("No eval_sac_*_33_*.json files found.", file=sys.stderr)
        sys.exit(1)
    for p in paths:
        d = json.load(open(p, encoding="utf-8"))
        s = d["summary"]
        name = os.path.basename(p).replace("eval_", "").replace(".json", "")
        solved = s["episodes_solved"]
        total = s["episodes_total"]
        pct = 100.0 * s["success_rate"]
        wp = s["mean_waypoints"]
        wp_std = s["std_waypoints"]
        ge1 = s["fraction_reached_at_least_one_wp"] * 100
        print(
            f"{name:32}  {solved:2}/{total}  {pct:5.1f}%  "
            f"mean_wp={wp:.2f}±{wp_std:.2f}  ≥1WP={ge1:5.1f}%"
        )


if __name__ == "__main__":
    main()
