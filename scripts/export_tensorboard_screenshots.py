#!/usr/bin/env python3
"""
Export real TensorBoard UI screenshots (native orange theme, all key scalars).

Uses thesis_tb_views (one session per run — no zig-zag). High-DPI Chromium via Playwright.

  bash ~/ros2_ws/scripts/setup_thesis_tensorboard_views.sh
  python3 ~/ros2_ws/scripts/export_tensorboard_screenshots.py

Outputs to pfe_report/thesis_pfe_pro/assets/screenshots/
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from playwright.sync_api import Page, sync_playwright

WS = Path.home() / "ros2_ws"
AGENT_LOGS = WS / "src/safe_drl_nav/safe_drl_nav/pfe_logs"
VIEWS = AGENT_LOGS / "thesis_tb_views"
EXPORT_ROOT = AGENT_LOGS / "_tb_export_sessions"
OUT_DIR = WS / "pfe_report/thesis_pfe_pro/assets/screenshots"

VIEWPORT_W = 1920
VIEWPORT_H = 1080
DEVICE_SCALE = 2  # 3840×2160 effective


@dataclass
class CaptureJob:
    run_dir: str
    tags: list[str]
    out_name: str
    title: str
    use_scalars_tab: bool = True
    wall_time: bool = True


def isolated_logdir(run_name: str) -> Path:
    src = VIEWS / run_name
    if not src.is_dir():
        raise FileNotFoundError(f"Missing {src} — run setup_thesis_tensorboard_views.sh")
    d = EXPORT_ROOT / run_name
    if d.exists():
        shutil.rmtree(d)
    d.mkdir(parents=True)
    for f in src.glob("events.out.tfevents*"):
        shutil.copy2(f, d / f.name)
    return d


def start_tensorboard(logdir: Path, port: int) -> subprocess.Popen:
    return subprocess.Popen(
        [
            "tensorboard",
            "--logdir",
            str(logdir),
            "--port",
            str(port),
            "--host",
            "127.0.0.1",
            "--load_fast=false",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def wait_tb(port: int, timeout: float = 60.0) -> None:
    import urllib.request

    url = f"http://127.0.0.1:{port}/"
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with urllib.request.urlopen(url, timeout=3) as r:
                if r.status == 200:
                    return
        except OSError:
            pass
        time.sleep(0.8)
    raise TimeoutError(f"TensorBoard not ready on port {port}")


def wait_charts(page: Page, min_canvas: int = 1, timeout_ms: int = 90_000) -> None:
    page.wait_for_load_state("networkidle", timeout=timeout_ms)
    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        n = page.locator("canvas, svg.mv-line-chart, .line-chart").count()
        if n >= min_canvas:
            time.sleep(2.5)
            return
        time.sleep(0.5)
    # TB 2.x may still render; continue after sleep
    time.sleep(4)


def set_wall_time_axis(page: Page) -> None:
    """Switch x-axis from Step → wall time (TensorBoard 2.x Time Series)."""
    for sel in (
        'button:has-text("Step")',
        '[aria-label*="Step"]',
        'mat-select',
    ):
        try:
            page.locator(sel).first.click(timeout=2500)
            time.sleep(0.6)
            break
        except Exception:
            continue
    for label in (
        "Relative",
        "Wall time",
        "Wall Time",
        "Relative wall time",
    ):
        try:
            page.get_by_text(label, exact=False).first.click(timeout=2500)
            time.sleep(2)
            return
        except Exception:
            continue
    # JS fallback for mat-option menus
    page.evaluate(
        """() => {
          const opts = [...document.querySelectorAll('mat-option, [role="option"], button')];
          const hit = opts.find(el => /wall|relative/i.test(el.textContent || ''));
          if (hit) hit.click();
        }"""
    )
    time.sleep(2)


def hide_sidebar_for_capture(page: Page) -> None:
    page.add_style_tag(
        content="""
        .sidebar { width: 220px !important; }
        tensorboard-root { --tb-sidebar-width: 220px; }
        """
    )


def capture_page(
    page: Page,
    port: int,
    *,
    tab: str,
    tag_filter: str | None,
    out: Path,
    full_page: bool,
    wall_time: bool,
) -> None:
    frag = tab
    if tag_filter:
        frag += f"&tagFilter={tag_filter}"
    url = f"http://127.0.0.1:{port}/#{frag}"
    page.goto(url, wait_until="networkidle", timeout=120_000)
    wait_charts(page)
    if tab == "timeseries" and wall_time:
        set_wall_time_axis(page)
        time.sleep(2)
        wait_charts(page)
    hide_sidebar_for_capture(page)
    time.sleep(1)
    page.screenshot(path=str(out), full_page=full_page)


def capture_tag_cards(
    page: Page,
    port: int,
    tags: list[str],
    tmp_dir: Path,
    *,
    wall_time: bool,
) -> list[Path]:
    shots: list[Path] = []
    for i, tag in enumerate(tags):
        out = tmp_dir / f"tag_{i:02d}_{tag.replace('/', '_')}.png"
        capture_page(
            page,
            port,
            tab="timeseries",
            tag_filter=tag,
            out=out,
            full_page=False,
            wall_time=wall_time,
        )
        shots.append(out)
    return shots


def capture_scalars_dashboard(
    page: Page,
    port: int,
    out: Path,
    tag_filter: str | None = None,
) -> None:
    capture_page(
        page,
        port,
        tab="scalars",
        tag_filter=tag_filter,
        out=out,
        full_page=True,
        wall_time=False,
    )


def _load_font(size: int) -> ImageFont.ImageFont:
    for name in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    ):
        p = Path(name)
        if p.is_file():
            return ImageFont.truetype(str(p), size)
    return ImageFont.load_default()


def stitch_vertical(
    images: list[Path],
    out: Path,
    titles: list[str] | None = None,
    header: str | None = None,
    bg: str = "#ffffff",
) -> None:
    ims = [Image.open(p).convert("RGB") for p in images]
    pad = 24
    title_h = 36 if titles else 0
    header_h = 52 if header else 0
    w = max(im.width for im in ims)
    total_h = header_h + sum(im.height + title_h + pad for im in ims) + pad
    canvas = Image.new("RGB", (w, total_h), bg)
    draw = ImageDraw.Draw(canvas)
    font = _load_font(22)
    header_font = _load_font(28)
    y = pad
    if header:
        draw.text((pad, y), header, fill="#e65100", font=header_font)
        y += header_h
    for i, im in enumerate(ims):
        if titles and i < len(titles):
            draw.text((pad, y), titles[i], fill="#37474f", font=font)
            y += title_h
        x = (w - im.width) // 2
        canvas.paste(im, (x, y))
        y += im.height + pad
    out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out, format="PNG", optimize=True)
    print(f"  wrote {out} ({w}×{total_h})")


def export_all(out: Path, port_base: int, *, compact: bool = False) -> None:
    if not VIEWS.is_dir():
        raise SystemExit(f"Run: bash ~/ros2_ws/scripts/setup_thesis_tensorboard_views.sh")

    tmp = EXPORT_ROOT / "_stitch_tmp"
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)

    jobs: list[tuple[CaptureJob, str]] = [
        (
            CaptureJob(
                "01_phase1_sac_adapt_7776ep_11h",
                ["Reward"],
                "phase1_adaptation.png",
                "Phase 1 — SAC adaptation (random goals, 7776 ep, ~11 h)",
            ),
            "phase1",
        ),
        (
            CaptureJob(
                "02_lab_waypoints_5729ep_21h",
                ["ExecutionTime_ms", "Reward"],
                "phase2_lab_panel.png",
                "Phase 2 — Lab waypoint marathon (~21 h, 5729 ep)",
            ),
            "lab",
        ),
        (
            CaptureJob(
                "03_maze_boost_latest",
                [
                    "ExecutionTime_ms",
                    "Reward",
                    "waypoint/progress_pct",
                    "waypoint/best_wp_so_far",
                    "curriculum/effective_steps",
                ],
                "phase2_maze_panel.png",
                "Phase 2/3 — Maze boost (waypoints + curriculum)",
            ),
            "maze",
        ),
    ]

    procs: list[subprocess.Popen] = []
    ports: dict[str, int] = {}
    try:
        for i, (_, key) in enumerate(jobs):
            run_name = jobs[i][0].run_dir
            logdir = isolated_logdir(run_name)
            p = port_base + i
            ports[key] = p
            procs.append(start_tensorboard(logdir, p))
        for p in ports.values():
            wait_tb(p)

        out.mkdir(parents=True, exist_ok=True)

        with sync_playwright() as pw:
            browser = pw.chromium.launch(
                headless=True,
                args=["--disable-dev-shm-usage", "--font-render-hinting=medium"],
            )
            page = browser.new_page(
                viewport={"width": VIEWPORT_W, "height": VIEWPORT_H},
                device_scale_factor=DEVICE_SCALE,
            )

            captured: dict[str, list[Path]] = {}

            for job, key in jobs:
                print(f"\n[{key}] {job.run_dir}")
                p = ports[key]
                job_tmp = tmp / key
                job_tmp.mkdir(parents=True)

                # Full scalars dashboard (all tags visible, native TB colors)
                dash = job_tmp / "scalars_dashboard.png"
                capture_scalars_dashboard(page, p, dash)
                captured[key] = [dash]

                if not compact:
                    tag_shots = capture_tag_cards(
                        page, p, job.tags, job_tmp, wall_time=job.wall_time
                    )
                    captured[key].extend(tag_shots)

            if compact:
                # One extra card each where scalars dashboard is thin
                capture_tag_cards(
                    page,
                    ports["phase1"],
                    ["Reward"],
                    tmp / "phase1",
                    wall_time=True,
                )
                captured["phase1"].extend(
                    list((tmp / "phase1").glob("tag_*.png"))
                )
                capture_tag_cards(
                    page,
                    ports["maze"],
                    ["waypoint/progress_pct"],
                    tmp / "maze_extra",
                    wall_time=True,
                )
                captured["maze_extra"] = list((tmp / "maze_extra").glob("tag_*.png"))

            browser.close()

        # --- Compose thesis filenames (real TB renders) ---
        lab = captured["lab"]
        maze = captured["maze"]

        if compact:
            stitch_vertical(
                captured["phase1"][:2],
                out / "phase1_adaptation.png",
                titles=["Scalars", "Reward (wall time)"],
                header=jobs[0][0].title,
            )
            stitch_vertical(
                [lab[0], maze[0]],
                out / "phase2_waypoints.png",
                titles=["Lab marathon (~21 h, 5729 ep)", "Maze boost (waypoints + curriculum)"],
                header="Phase 2 — TensorBoard scalars",
            )
        else:
            stitch_vertical(
                captured["phase1"],
                out / "phase1_adaptation.png",
                titles=["Scalars dashboard", "Reward (wall time)"][: len(captured["phase1"])],
                header=jobs[0][0].title,
            )
            phase2_parts = []
            phase2_titles = []
            if lab:
                phase2_parts.extend([lab[0], *lab[1:3]] if len(lab) >= 3 else lab)
                phase2_titles.extend(
                    [
                        "Lab marathon — scalars dashboard",
                        "ExecutionTime_ms (wall time)",
                        "Reward (wall time)",
                    ][: len(phase2_parts)]
                )
            if maze:
                phase2_parts.append(maze[0])
                phase2_titles.append("Maze boost — scalars (waypoints, curriculum)")
                if len(maze) >= 4:
                    phase2_parts.append(maze[3])
                    phase2_titles.append("waypoint/progress_pct (wall time)")
            stitch_vertical(
                phase2_parts,
                out / "phase2_waypoints.png",
                titles=phase2_titles,
                header="Phase 2 — Lab waypoint + maze curriculum (TensorBoard)",
            )

        # Single-tag thesis figures (best card = last tag shot for Reward / waypoint / exec)
        def pick_tag(key: str, tag: str) -> Path:
            safe = tag.replace("/", "_")
            for p in captured.get(key, []):
                if safe in p.name:
                    return p
            return captured[key][-1]

        if compact:
            shutil.copy2(lab[0], out / "training_reward_curve.png")
            wp = captured.get("maze_extra", [maze[0]])[0]
            shutil.copy2(wp, out / "tensorboard_waypoint_progress.png")
            stitch_vertical(
                [lab[0]],
                out / "training_execution_time.png",
                titles=["ExecutionTime_ms + Reward (lab, ~21 h)"],
                header="Training latency — TensorBoard scalars",
            )
        else:
            shutil.copy2(pick_tag("lab", "Reward"), out / "training_reward_curve.png")
            shutil.copy2(
                pick_tag("maze", "waypoint/progress_pct"),
                out / "tensorboard_waypoint_progress.png",
            )
            stitch_vertical(
                [
                    pick_tag("lab", "ExecutionTime_ms"),
                    pick_tag("maze", "ExecutionTime_ms"),
                ],
                out / "training_execution_time.png",
                titles=[
                    "Phase 2 lab — ExecutionTime_ms (~21 h)",
                    "Maze boost — ExecutionTime_ms",
                ],
                header="Training — mean step latency (TensorBoard, wall time)",
            )
        print(f"  wrote {out / 'training_reward_curve.png'}")
        print(f"  wrote {out / 'tensorboard_waypoint_progress.png'}")

        # Overview: scalars dashboards only
        stitch_vertical(
            [captured[k][0] for k in ("phase1", "lab", "maze")],
            out / "training_dashboard_overview.png",
            titles=[jobs[i][0].title for i in range(3)],
            header="SAC PFE — TensorBoard scalars overview",
        )

    finally:
        for proc in procs:
            proc.terminate()
        for proc in procs:
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, default=OUT_DIR)
    p.add_argument("--port", type=int, default=6030)
    p.add_argument(
        "--compact",
        action="store_true",
        help="Thesis-friendly height: scalars dashboards + one key card per phase",
    )
    p.add_argument(
        "--full",
        action="store_true",
        help="All tag cards (tall phase2_waypoints; slow)",
    )
    args = p.parse_args()
    compact = args.compact or not args.full
    print(f"Exporting real TensorBoard UI → {args.out}  (compact={compact})")
    print(f"Resolution: {VIEWPORT_W * DEVICE_SCALE}×… (device_scale={DEVICE_SCALE})")
    export_all(args.out, args.port, compact=compact)
    print("\nDone — native TensorBoard theme (orange header, blue curves).")


if __name__ == "__main__":
    main()
