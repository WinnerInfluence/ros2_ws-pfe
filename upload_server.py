#!/usr/bin/env python3
"""
upload_server.py — Model Upload & Eval-Trigger backend for live_game.html

Routes
------
  POST /upload           multipart: file=.pth, algo, episodes [, spawn_eval]
                         Saves to trained_models/uploaded_brain.pth, publishes
                         /policy_reload. If spawn_eval is true (default), also runs
                         evaluate_agent.py subprocess.
  GET  /eval_stream      SSE — streams subprocess stdout/stderr lines in real-time.
  GET  /eval_status      JSON snapshot of the current/last job.
  GET  /eval_results     JSON — contents of the most recently written eval JSON file.
  POST /eval_cancel      Kill a running eval job.

Usage
-----
  pip install flask flask-cors

  Terminal (with ROS sourced, same machine as ros2_ws):
      python3 upload_server.py              # listens on http://localhost:5001

  Form field ``spawn_eval`` (default ``true``):
    * ``true``  — saves ``uploaded_brain.pth``, runs ``evaluate_agent.py`` subprocess.
    * ``false`` — save only + ``ros2 topic pub`` on ``/policy_reload`` for
                  ``hot_swap_eval_node.py`` (no subprocess / no SSE workload).
"""

from __future__ import annotations

import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Generator

from flask import Flask, Response, jsonify, request
from flask_cors import CORS

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

SCRIPT_DIR  = Path(__file__).resolve().parent
AGENT_DIR   = SCRIPT_DIR / "src" / "safe_drl_nav" / "safe_drl_nav"
MODELS_DIR  = AGENT_DIR / "trained_models"
UPLOAD_PATH = MODELS_DIR / "uploaded_brain.pth"
EVAL_SCRIPT = AGENT_DIR / "evaluate_agent.py"
LOGS_DIR    = AGENT_DIR / "pfe_logs"

MODELS_DIR.mkdir(parents=True, exist_ok=True)

# Demo / production env (see sim_demo.env)
TRAINING_CONTRACT = os.environ.get(
    "TRAINING_CONTRACT",
    str(AGENT_DIR / "training_contract.yaml"),
).strip()
PFE_WORLD = os.environ.get("PFE_WORLD", "").strip()
UPLOAD_API_KEY = os.environ.get("UPLOAD_API_KEY", "").strip()
SIM_DEMO_PAPER_EVAL = os.environ.get("SIM_DEMO_PAPER_EVAL", "0").strip().lower() in (
    "1",
    "true",
    "yes",
)
SIM_DEMO_EVAL_EPISODES = int(os.environ.get("SIM_DEMO_EVAL_EPISODES", "5"))
DEFAULT_WAYPOINT_GOAL_RADIUS = os.environ.get("SIM_DEMO_WAYPOINT_GOAL_RADIUS", "0.68")

# Live LiDAR for /lidar_live — written by scripts/local/upload_telemetry_sidecar.py (not in-process ROS).
TELEMETRY_FILE = Path(
    os.environ.get("TELEMETRY_FILE", str(SCRIPT_DIR / "pfe_logs" / "telemetry_live.json"))
)


def _normalize_telemetry_snap(snap: dict) -> dict:
    """Unify hot_swap_eval vs upload_telemetry_sidecar field names for the website."""
    if snap.get("t") is None and snap.get("x") is not None:
        snap["t"] = "s"
    if "scan" not in snap and "scan24" in snap:
        snap["scan"] = snap["scan24"]
    if snap.get("ep") is None and "episode" in snap:
        snap["ep"] = snap["episode"]
    if snap.get("rew") is None and "reward" in snap:
        snap["rew"] = snap["reward"]
    if snap.get("tot") is None and "total_reward" in snap:
        snap["tot"] = snap["total_reward"]
    if "updated_at" not in snap:
        snap["updated_at"] = time.time()
    if "ok" not in snap:
        snap["ok"] = True
    return snap


def _read_telemetry_snap() -> dict:
    """Read telemetry JSON; ok=false if missing or unreadable."""
    try:
        raw = TELEMETRY_FILE.read_text(encoding="utf-8")
        snap = json.loads(raw)
        if isinstance(snap, dict):
            return _normalize_telemetry_snap(snap)
    except (OSError, json.JSONDecodeError, TypeError):
        pass
    return {"ok": False, "updated_at": 0.0, "t": "s"}


def _check_api_key() -> bool:
    """Return True if request is authorized (no key configured = open)."""
    if not UPLOAD_API_KEY:
        return True
    supplied = (
        request.headers.get("X-API-Key", "").strip()
        or request.form.get("api_key", "").strip()
    )
    return supplied == UPLOAD_API_KEY


# ---------------------------------------------------------------------------
# Global job state (one eval at a time)
# ---------------------------------------------------------------------------

class _Job:
    def __init__(self) -> None:
        self.lock       = threading.Lock()
        self.status     = "idle"          # idle | running | done | error | cancelled
        self.algo       = ""
        self.episodes   = 0
        self.started_at: float | None = None
        self.ended_at:   float | None = None
        self.output_lines: list[str]  = []
        self.proc:  subprocess.Popen | None = None
        # Subscribers waiting for SSE lines
        self._queues: list[queue.Queue] = []

    def reset(self, algo: str, episodes: int) -> None:
        with self.lock:
            self.status     = "running"
            self.algo       = algo
            self.episodes   = episodes
            self.started_at = time.time()
            self.ended_at   = None
            self.output_lines = []
            self._queues    = []

    def push_line(self, line: str) -> None:
        with self.lock:
            self.output_lines.append(line)
            for q in self._queues:
                q.put(line)

    def finish(self, status: str) -> None:
        with self.lock:
            self.status   = status
            self.ended_at = time.time()
            sentinel = None
            for q in self._queues:
                q.put(sentinel)

    def subscribe(self) -> "queue.Queue[str | None]":
        q: queue.Queue[str | None] = queue.Queue()
        with self.lock:
            # Back-fill existing lines so a late subscriber sees full output
            for line in self.output_lines:
                q.put(line)
            if self.status not in ("running",):
                q.put(None)  # already done
            else:
                self._queues.append(q)
        return q

    def snapshot(self) -> dict:
        with self.lock:
            elapsed = None
            if self.started_at:
                end = self.ended_at or time.time()
                elapsed = round(end - self.started_at, 1)
            return {
                "status":   self.status,
                "algo":     self.algo,
                "episodes": self.episodes,
                "elapsed_s": elapsed,
                "lines":    len(self.output_lines),
                "tail":     self.output_lines[-20:],
            }


JOB = _Job()

HOT_SWAP_SCRIPT = SCRIPT_DIR / "scripts" / "start_hot_swap_for_web.sh"
HOT_SWAP_LOG = SCRIPT_DIR / "pfe_logs" / "hot_swap.log"


def _hot_swap_running() -> bool:
    try:
        proc = subprocess.run(
            ["pgrep", "-f", "hot_swap_eval_node.py"],
            capture_output=True,
            timeout=3,
        )
        return proc.returncode == 0
    except (subprocess.TimeoutExpired, OSError):
        return False


def _ensure_hot_swap_running(algo: str = "sac") -> bool:
    """Start ``hot_swap_eval_node.py`` if the website upload has no subscriber."""
    if _hot_swap_running():
        return True
    if not HOT_SWAP_SCRIPT.is_file():
        return False
    HOT_SWAP_LOG.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["pkill", "-f", "hot_swap_eval_node.py"], capture_output=True, timeout=5)
    time.sleep(0.5)
    with HOT_SWAP_LOG.open("a", encoding="utf-8") as logf:
        logf.write(f"\n[upload_server] auto-start hot_swap algo={algo}\n")
        subprocess.Popen(
            ["bash", str(HOT_SWAP_SCRIPT), algo.strip().lower()],
            stdout=logf,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            cwd=str(SCRIPT_DIR),
        )
    for _ in range(20):
        if _hot_swap_running():
            time.sleep(2.0)
            return True
        time.sleep(1.0)
    return False


def _publish_policy_reload(path: Path, algo: str = "sac") -> bool:
    """Notify ``hot_swap_eval_node`` via ROS 2 CLI (requires ``ros2`` on PATH)."""
    if shutil.which("ros2") is None and not Path("/opt/ros/humble/setup.bash").is_file():
        return False
    if not _ensure_hot_swap_running(algo):
        return False
    p = path.resolve().as_posix()
    reload_data = json.dumps({"path": p, "algo": algo.strip().lower()})
    # Valid ROS 2 CLI YAML: {data: "<json string>"} — do not use repr() on the whole message.
    ros_msg = "{data: " + json.dumps(reload_data) + "}"
    ros_setup = f"/opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash"
    ws_setup = SCRIPT_DIR / "install" / "setup.bash"
    cmd = (
        f"set +u; source {ros_setup} 2>/dev/null; "
        f"{f'source {ws_setup.as_posix()} 2>/dev/null;' if ws_setup.is_file() else ''} "
        f"set -u; ros2 topic pub --once /policy_reload std_msgs/msg/String '{ros_msg}'"
    )
    try:
        proc = subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True,
            text=True,
            timeout=15,
        )
        return proc.returncode == 0
    except (subprocess.TimeoutExpired, OSError, ValueError):
        return False


# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------

app = Flask(__name__)
CORS(app)  # allow fetch from file:// and any origin


@app.get("/health")
def health():
    return jsonify({
        "ok": True,
        "training_contract": TRAINING_CONTRACT,
        "contract_exists": os.path.isfile(TRAINING_CONTRACT),
        "pfe_world": PFE_WORLD or None,
        "upload_api_key_required": bool(UPLOAD_API_KEY),
        "hot_swap_running": _hot_swap_running(),
    })


def _maze_layout_defaults() -> dict:
    try:
        scripts = SCRIPT_DIR / "scripts"
        if str(scripts) not in sys.path:
            sys.path.insert(0, str(scripts))
        from maze_web_layout import MAZE_ENV_NAME, MAZE_OBSTACLES, MAZE_WAYPOINTS

        return {
            "env_name": MAZE_ENV_NAME,
            "waypoints": MAZE_WAYPOINTS,
            "obstacles": MAZE_OBSTACLES,
        }
    except ImportError:
        return {
            "env_name": "Maze · 3 waypoints",
            "waypoints": [
                {"x": -2.8, "y": -1.8, "n": 1},
                {"x": -4.5, "y": -0.2, "n": 2},
                {"x": -3.2, "y": 1.2, "n": 3},
            ],
            "obstacles": [],
        }


@app.get("/lidar_live")
def lidar_live():
    """HTTP polling fallback for live dashboard (proxied as /ros/api/lidar_live)."""
    snap = _read_telemetry_snap()
    for key, val in _maze_layout_defaults().items():
        snap.setdefault(key, val)
    age = time.time() - float(snap.get("updated_at") or 0.0)
    snap["stale"] = age > 3.0
    if not snap.get("ok"):
        snap["stale"] = True
    return jsonify(snap)


@app.post("/upload")
def upload():
    """Receive .pth file + algo + episodes; save and optionally run evaluation."""
    if not _check_api_key():
        return jsonify({"error": "Invalid or missing API key."}), 401

    spawn_eval = (
        request.form.get("spawn_eval", "true").strip().lower()
        not in ("0", "false", "no", "off")
    )

    if spawn_eval and JOB.status == "running":
        return jsonify({"error": "An evaluation is already running. Cancel it first."}), 409

    # ---- validate form fields ----
    if "file" not in request.files:
        return jsonify({"error": "No file part in request."}), 400
    file = request.files["file"]
    if not file.filename or not file.filename.endswith(".pth"):
        return jsonify({"error": "File must be a .pth checkpoint."}), 400

    algo = request.form.get("algo", "sac").strip().lower()
    if algo not in ("sac", "td3", "custom"):
        return jsonify({"error": f"Unknown algo {algo!r}. Use sac, td3, or custom."}), 400

    paper_eval = (
        request.form.get("paper_eval", "").strip().lower() in ("1", "true", "yes", "on")
        or SIM_DEMO_PAPER_EVAL
    )
    try:
        default_ep = 30 if paper_eval else SIM_DEMO_EVAL_EPISODES
        episodes = int(request.form.get("episodes", default_ep))
        if paper_eval and episodes < 30:
            episodes = 30
        if episodes < 1 or episodes > 100:
            raise ValueError
    except ValueError:
        return jsonify({"error": "episodes must be an integer 1–100."}), 400

    # ---- save file ----
    file.save(str(UPLOAD_PATH))
    reload_ok = _publish_policy_reload(UPLOAD_PATH, algo=algo)
    hot_swap_ok = _hot_swap_running()

    if not spawn_eval:
        msg = (
            "Saved weights and published /policy_reload for hot_swap_eval_node."
            if reload_ok
            else (
                "Saved weights but could not publish /policy_reload "
                "(start: ~/ros2_ws/scripts/local/start_hot_swap_for_web.sh "
                f"{algo} — or restart upload_server after fixing ROS)."
            )
        )
        return jsonify({
            "ok":          True,
            "spawn_eval":  False,
            "algo":        algo,
            "episodes":    episodes,
            "saved_to":    str(UPLOAD_PATH),
            "reload_pub":  reload_ok,
            "hot_swap":    hot_swap_ok,
            "message":     msg,
        })

    # ---- launch eval in background thread ----
    JOB.reset(algo, episodes)
    t = threading.Thread(
        target=_run_eval, args=(algo, episodes, paper_eval), daemon=True
    )
    t.start()

    return jsonify({
        "ok":          True,
        "spawn_eval":  True,
        "reload_pub":  reload_ok,
        "algo":        algo,
        "episodes":    episodes,
        "paper_eval":  paper_eval,
        "saved_to":    str(UPLOAD_PATH),
        "message":     "Evaluation started — open /eval_stream for live output.",
    })


@app.get("/eval_stream")
def eval_stream():
    """
    Server-Sent Events stream.  Each event is one line of subprocess output.
    Client opens  EventSource('/eval_stream')  after POSTing to /upload.
    """
    q = JOB.subscribe()

    def generate() -> Generator[str, None, None]:
        yield "retry: 1000\n\n"
        while True:
            try:
                line = q.get(timeout=30)
            except queue.Empty:
                yield ": keepalive\n\n"
                continue
            if line is None:
                snap = JOB.snapshot()
                yield f"event: done\ndata: {json.dumps(snap)}\n\n"
                return
            safe = line.replace("\n", " ").replace("\r", "")
            yield f"data: {json.dumps(safe)}\n\n"

    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.get("/eval_status")
def eval_status():
    return jsonify(JOB.snapshot())


@app.get("/eval_results")
def eval_results():
    """Return the most recently written eval JSON from pfe_logs/."""
    if not LOGS_DIR.exists():
        return jsonify({"error": "No pfe_logs directory found."}), 404
    files = sorted(LOGS_DIR.glob("eval_*.json"), key=lambda p: p.stat().st_mtime)
    if not files:
        return jsonify({"error": "No eval result files found."}), 404
    with open(files[-1], encoding="utf-8") as f:
        return jsonify(json.load(f))


@app.post("/eval_cancel")
def eval_cancel():
    with JOB.lock:
        if JOB.status != "running":
            return jsonify({"error": "No running job to cancel."}), 400
        proc = JOB.proc
    if proc:
        proc.terminate()
    return jsonify({"ok": True, "message": "Termination signal sent."})


# ---------------------------------------------------------------------------
# Background eval runner
# ---------------------------------------------------------------------------

def _run_eval(algo: str, episodes: int, paper_eval: bool) -> None:
    cmd = [
        sys.executable,
        str(EVAL_SCRIPT),
        "--algo",     algo,
        "--model",    str(UPLOAD_PATH),
        "--episodes", str(episodes),
        "--tag",      "uploaded_brain",
        "--max-steps", "4000",
        "--env-step-sleep-sec", os.environ.get("PFE_ENV_STEP_SLEEP", "0.05"),
        "--reset-service-wait-sec",
        os.environ.get("PFE_RESET_SERVICE_WAIT", "45"),
        "--waypoint-goal-radius", DEFAULT_WAYPOINT_GOAL_RADIUS,
        "--device", "cpu",
    ]
    if TRAINING_CONTRACT and os.path.isfile(TRAINING_CONTRACT):
        cmd.extend(["--training-contract", TRAINING_CONTRACT])
    if paper_eval:
        cmd.append("--paper-eval")
        cmd.append("--require-reset")

    env = os.environ.copy()
    if PFE_WORLD:
        env["PFE_WORLD"] = PFE_WORLD
    env["PYTHONPATH"] = str(AGENT_DIR) + os.pathsep + env.get("PYTHONPATH", "")

    JOB.push_line(f"[upload_server] Running: {' '.join(cmd)}")
    JOB.push_line(f"[upload_server] Working dir: {AGENT_DIR}")

    try:
        proc = subprocess.Popen(
            cmd,
            cwd=str(AGENT_DIR),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        with JOB.lock:
            JOB.proc = proc

        for line in proc.stdout:  # type: ignore[union-attr]
            JOB.push_line(line.rstrip())

        proc.wait()
        status = "done" if proc.returncode == 0 else "error"
        JOB.push_line(
            f"[upload_server] Process exited with code {proc.returncode}."
        )
        JOB.finish(status)

    except Exception as exc:  # noqa: BLE001
        JOB.push_line(f"[upload_server] Exception: {exc}")
        JOB.finish("error")
    finally:
        with JOB.lock:
            JOB.proc = None


# ---------------------------------------------------------------------------
# WSGI: LiteSpeed forwards /ros/api/* without stripping the prefix
# ---------------------------------------------------------------------------

class _StripRosApiPrefix:
  """Map /ros/api/health → /health when behind a reverse proxy."""

  _PREFIXES = ("/ros/api", "/ROS/api")

  def __init__(self, wsgi_app):
    self._app = wsgi_app

  def __call__(self, environ, start_response):
    path = environ.get("PATH_INFO", "") or ""
    for prefix in self._PREFIXES:
      if path == prefix:
        environ["PATH_INFO"] = "/"
        break
      if path.startswith(prefix + "/"):
        environ["PATH_INFO"] = path[len(prefix) :] or "/"
        break
    return self._app(environ, start_response)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5001)
    args = parser.parse_args()

    print(f"\n  Model Upload server  →  http://{args.host}:{args.port}")
    print(f"  Telemetry file       :  {TELEMETRY_FILE}")
    print(f"  Eval script         :  {EVAL_SCRIPT}")
    print(f"  Upload destination  :  {UPLOAD_PATH}\n")

    wsgi = _StripRosApiPrefix(app)
    from werkzeug.serving import run_simple

    run_simple(args.host, args.port, wsgi, threaded=True, use_reloader=False)
