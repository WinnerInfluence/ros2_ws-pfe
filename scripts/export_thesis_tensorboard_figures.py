#!/usr/bin/env python3
"""
Thesis figures from TensorBoard / CSV — wall-clock time, no merged-episode artifacts.

The old plots looked "buggy" because 76 training restarts were drawn on one
episode axis (lines jump backward). This script uses:
  - wall time (hours) on the x-axis, and/or
  - ONE session per panel (longest lab run or current run).

Usage:
  python3 ~/ros2_ws/scripts/export_thesis_tensorboard_figures.py
  python3 ~/ros2_ws/scripts/export_thesis_tensorboard_figures.py --session longest
  python3 ~/ros2_ws/scripts/export_thesis_tensorboard_figures.py --session newest

Requires: tensorboard, matplotlib, numpy
"""
from __future__ import annotations

import argparse
import datetime as dt
from dataclasses import dataclass
from pathlib import Path

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np

try:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
except ImportError as e:
    raise SystemExit("pip install tensorboard matplotlib numpy") from e

WS = Path.home() / "ros2_ws"
AGENT_LOGS = WS / "src/safe_drl_nav/safe_drl_nav/pfe_logs"
OUT_DIR = WS / "pfe_report/thesis_pfe_pro/assets/screenshots"

# Simple thesis style — no heavy smoothing / neon overlays
plt.rcParams.update(
    {
        "figure.dpi": 120,
        "savefig.dpi": 200,
        "font.size": 10,
        "axes.labelsize": 10,
        "axes.titlesize": 11,
        "axes.grid": True,
        "grid.linestyle": ":",
        "grid.alpha": 0.5,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "axes.spines.top": False,
        "axes.spines.right": False,
    }
)


@dataclass
class Series:
    episode: np.ndarray
    wall: np.ndarray  # matplotlib date numbers
    hours: np.ndarray  # hours since series start
    value: np.ndarray
    label: str


def _event_file_series(path: Path, tag: str) -> Series | None:
    ea = EventAccumulator(str(path), size_guidance={"scalars": 0})
    ea.Reload()
    if tag not in ea.Tags().get("scalars", []):
        return None
    ev = ea.Scalars(tag)
    if not ev:
        return None
    wall = np.array([e.wall_time for e in ev])
    t0 = wall[0]
    return Series(
        episode=np.array([e.step for e in ev], dtype=float),
        wall=np.array([mdates.date2num(dt.datetime.fromtimestamp(t)) for t in wall]),
        hours=(wall - t0) / 3600.0,
        value=np.array([e.value for e in ev], dtype=float),
        label=path.name.split(".")[2][:10],
    )


def pick_event_file(tb_dir: Path, session: str) -> Path:
    files = [p for p in tb_dir.glob("events.out.tfevents*") if p.is_file()]
    if not files:
        raise FileNotFoundError(f"No event files in {tb_dir}")
    if session == "newest":
        return max(files, key=lambda p: p.stat().st_mtime)
    # longest: most Reward points
    best, best_n = files[0], -1
    for f in files:
        ea = EventAccumulator(str(f), size_guidance={"scalars": 0})
        ea.Reload()
        n = len(ea.Scalars("Reward")) if "Reward" in ea.Tags().get("scalars", []) else 0
        if n > best_n:
            best_n, best = n, f
    return best


def load_chronological_wall(tb_dir: Path, tag: str) -> Series | None:
    """All sessions sorted by wall time — for CPU / execution time trends only."""
    pts: list[tuple[float, float, int]] = []
    for f in tb_dir.glob("events.out.tfevents*"):
        ea = EventAccumulator(str(f), size_guidance={"scalars": 0})
        ea.Reload()
        if tag not in ea.Tags().get("scalars", []):
            continue
        for e in ea.Scalars(tag):
            pts.append((e.wall_time, e.value, e.step))
    if not pts:
        return None
    pts.sort(key=lambda x: x[0])
    wall = np.array([p[0] for p in pts])
    t0 = wall[0]
    return Series(
        episode=np.array([p[2] for p in pts], dtype=float),
        wall=np.array([mdates.date2num(dt.datetime.fromtimestamp(t)) for t in wall]),
        hours=(wall - t0) / 3600.0,
        value=np.array([p[1] for p in pts], dtype=float),
        label="all sessions (chrono)",
    )


def rolling_median(y: np.ndarray, w: int) -> np.ndarray:
    w = max(3, min(w, len(y) // 5 if len(y) > 10 else 3))
    if len(y) < w:
        return y.copy()
    out = np.empty_like(y)
    half = w // 2
    for i in range(len(y)):
        lo, hi = max(0, i - half), min(len(y), i + half + 1)
        out[i] = np.median(y[lo:hi])
    return out


def plot_vs_time(ax, s: Series, color: str, ylab: str, smooth_win: int = 15) -> None:
    ax.plot(s.hours, s.value, color=color, alpha=0.35, lw=0.7, label="per episode")
    if len(s.value) > smooth_win:
        ax.plot(s.hours, rolling_median(s.value, smooth_win), color=color, lw=1.6, label="rolling median")
    ax.set_xlabel("Wall time (hours since session start)")
    ax.set_ylabel(ylab)
    ax.legend(loc="best", fontsize=8, framealpha=0.95)


def save(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"  {path}")


def export_execution_time_focus(out: Path, session: str) -> None:
    """Main figure: execution time vs wall clock (user request)."""
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=False)

    # Phase 1 — chronological CPU trend across all adapt runs
    s1 = load_chronological_wall(AGENT_LOGS / "tb_sac", "ExecutionTime_ms")
    ax = axes[0]
    if s1 is not None:
        ax.plot(s1.hours, s1.value, color="#2e7d32", alpha=0.4, lw=0.5)
        if len(s1.value) > 20:
            ax.plot(s1.hours, rolling_median(s1.value, 31), color="#1b5e20", lw=1.5)
        ax.set_ylabel("Mean step time (ms)")
        ax.set_xlabel("Wall time (hours, all Phase-1 sessions)")
        ax.set_title("Phase 1 — execution time (tb_sac, chronological)")
    else:
        ax.set_title("Phase 1 — no ExecutionTime_ms data")

    # Phase 2/3 — single clean session (longest or newest)
    tb_maze = AGENT_LOGS / "tb_sac_maze"
    ef = pick_event_file(tb_maze, session)
    s2 = _event_file_series(ef, "ExecutionTime_ms")
    ax = axes[1]
    if s2 is not None:
        plot_vs_time(ax, s2, "#1565c0", "Mean step time (ms)", smooth_win=11)
        dur = s2.hours[-1] - s2.hours[0] if len(s2.hours) else 0
        n = len(s2.value)
        ax.set_title(f"Waypoint / maze training — one session ({n} ep, {dur:.1f} h wall) [{session}]")
    else:
        ax.set_title("Phase 2/3 — no data")

    fig.suptitle("SAC training — compute time per control step", fontsize=12, y=1.01)
    save(out / "training_execution_time.png")

    # Also refresh thesis filename: simple single-panel for Ch.5
    if s2 is not None:
        fig, ax = plt.subplots(figsize=(9, 3.8))
        plot_vs_time(ax, s2, "#37474f", "Execution time (ms / env step)", smooth_win=11)
        ax.set_title(f"Episode step latency — eval_maze boost ({session} session, {len(s2.value)} episodes)")
        save(out / "tensorboard_waypoint_progress.png")


def export_phase1(out: Path, session: str) -> None:
    ef = pick_event_file(AGENT_LOGS / "tb_sac", session)
    fig, axes = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    for ax, tag, ylab, c in [
        (axes[0], "Reward", "Episode return", "#1f4e79"),
        (axes[1], "ExecutionTime_ms", "Execution time (ms)", "#2e7d32"),
    ]:
        s = _event_file_series(ef, tag)
        if s is None:
            continue
        plot_vs_time(ax, s, c, ylab)
    axes[1].set_xlabel("Wall time (hours since session start)")
    fig.suptitle(f"Phase 1 adaptation — single session ({session})", fontsize=11)
    save(out / "phase1_adaptation.png")


def export_phase2_waypoints(out: Path, session: str) -> None:
    ef = pick_event_file(AGENT_LOGS / "tb_sac_maze", session)
    tags = [
        ("ExecutionTime_ms", "Execution time (ms)", "#37474f"),
        ("waypoint/progress_pct", "Waypoint progress (%)", "#1565c0"),
        ("curriculum/effective_steps", "Step budget", "#ef6c00"),
        ("EpisodeSteps", "Steps used", "#546e7a"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    for ax, (tag, ylab, c) in zip(axes.flat, tags):
        s = _event_file_series(ef, tag)
        if s is None:
            ax.set_visible(False)
            continue
        plot_vs_time(ax, s, c, ylab, smooth_win=9)
    s0 = _event_file_series(ef, "Reward")
    title_extra = ""
    if s0 is not None:
        title_extra = f" — {len(s0.value)} ep, {s0.hours[-1]:.1f} h"
    fig.suptitle(f"SAC waypoint curriculum ({session} session){title_extra}", fontsize=11)
    save(out / "phase2_waypoints.png")


def export_reward_curve(out: Path, session: str) -> None:
    ef = pick_event_file(AGENT_LOGS / "tb_sac_maze", session)
    s = _event_file_series(ef, "Reward")
    if s is None:
        return
    fig, ax = plt.subplots(figsize=(9, 3.6))
    plot_vs_time(ax, s, "#0d47a1", "Episode return")
    ax.set_title(f"Episode return vs wall time ({session} session)")
    save(out / "training_reward_curve.png")


def export_longest_lab_from_csv(out: Path) -> None:
    csv_path = AGENT_LOGS / "sac_maze_metrics.csv"
    if not csv_path.is_file():
        return
    rows: list[tuple[int, float, int, int, float]] = []
    with csv_path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) < 5:
                continue
            try:
                rows.append((int(p[0]), float(p[1]), int(p[2]), int(p[3]), float(p[4])))
            except ValueError:
                continue
    # find longest session by local ep reset
    best_start, best_len, start, length, prev = 0, 0, 0, 0, 0
    for i, (ep, *_) in enumerate(rows):
        if i and ep <= prev:
            if length > best_len:
                best_len, best_start = length, start
            start, length = i, 1
        else:
            length += 1 if i else 1
            if i == 0:
                start = 0
        prev = ep
    if length > best_len:
        best_len, best_start = length, start
    seg = rows[best_start : best_start + best_len]
    if best_len < 50:
        return
    # cumulative wall time estimate: ep_steps * avg_time (sec) per row col 3,4
    cum_h = np.zeros(len(seg))
    t = 0.0
    for i, (_, _, _, ep_steps, avg_t) in enumerate(seg):
        cum_h[i] = t / 3600.0
        t += ep_steps * avg_t
    rew = np.array([r[1] for r in seg])
    exec_ms = np.array([r[4] for r in seg])  # avg_time_ms per env step (same as TensorBoard)

    fig, axes = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    axes[0].plot(cum_h, exec_ms, color="#37474f", alpha=0.5, lw=0.6)
    axes[0].plot(cum_h, rolling_median(exec_ms, 31), color="#1565c0", lw=1.4)
    axes[0].set_ylabel("Step time (ms)")
    axes[0].set_title(f"Longest lab session (~{best_len:,} ep, ~{cum_h[-1]:.0f} h estimated)")
    axes[1].plot(cum_h, rew, color="#90a4ae", alpha=0.4, lw=0.5)
    axes[1].plot(cum_h, rolling_median(rew, 41), color="#1b5e20", lw=1.3)
    axes[1].set_ylabel("Return")
    axes[1].set_xlabel("Estimated wall time (hours, from ep_steps × step latency)")
    save(out / "phase2_long_session_reward.png")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--session",
        choices=("newest", "longest"),
        default="longest",
        help="newest = current maze boost; longest = best lab marathon (~5k ep)",
    )
    p.add_argument("--out", type=Path, default=OUT_DIR)
    p.add_argument("--full-csv", action="store_true", help="Export longest-session CSV plot")
    args = p.parse_args()

    if not AGENT_LOGS.is_dir():
        raise SystemExit(f"Missing {AGENT_LOGS}")

    print(f"session={args.session}  out={args.out}")
    export_execution_time_focus(args.out, args.session)
    export_phase1(args.out, args.session)
    export_phase2_waypoints(args.out, args.session)
    export_reward_curve(args.out, args.session)
    if args.full_csv:
        export_longest_lab_from_csv(args.out)
    print("Done — use training_execution_time.png for CPU/wall-time focus.")


if __name__ == "__main__":
    main()
