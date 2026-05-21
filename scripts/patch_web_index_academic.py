#!/usr/bin/env python3
"""Inject academic header into website/ros/index.html (logos, author, supervisors, year)."""
from __future__ import annotations

import os
import re
import sys
import urllib.request
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
OUT_DIR = WS / "website" / "ros"
OUT_INDEX = OUT_DIR / "index.html"
SRC_ASSETS = WS / "demo_assets"
# Optional: fetch remote index before patch (default: use local website/ros/index.html only)
LIVE_URL = os.environ.get("WEB_INDEX_URL", "").strip()
MARKER = "pfe-academic-hdr-v2"

HDR_CSS = r"""
/* pfe-academic-hdr-v2 */
#hdr {
  flex-direction: column;
  align-items: stretch;
  padding: 0;
  gap: 0;
}
.hdr-pfe-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 14px;
  padding: 10px 20px 9px;
  background: linear-gradient(90deg, rgba(0,212,255,.08) 0%, rgba(5,9,18,.95) 40%, rgba(0,255,136,.05) 100%);
  border-bottom: 1px solid #162035;
  flex-wrap: wrap;
}
.hdr-pfe-logo {
  height: 54px;
  width: auto;
  max-width: 112px;
  object-fit: contain;
  filter: drop-shadow(0 1px 3px rgba(0,0,0,.5));
}
.hdr-pfe-center {
  flex: 1;
  min-width: 200px;
  text-align: center;
  line-height: 1.3;
}
.hdr-pfe-uni {
  font-size: .78rem;
  font-weight: 700;
  letter-spacing: .07em;
  text-transform: uppercase;
  color: #00d4ff;
}
.hdr-pfe-sub {
  font-size: .72rem;
  color: #3a5878;
  margin-top: 2px;
}
.hdr-pfe-year {
  display: inline-block;
  margin-top: 5px;
  padding: 3px 11px;
  border-radius: 999px;
  font-size: .72rem;
  font-weight: 700;
  color: #ffcc00;
  border: 1px solid rgba(255,204,0,.35);
}
.hdr-pfe-meta {
  display: flex;
  flex-direction: column;
  gap: 2px;
  font-size: .72rem;
  color: #c8e0f8;
  min-width: 148px;
}
.hdr-pfe-meta.right { text-align: right; align-items: flex-end; }
.hdr-pfe-meta label {
  font-size: .62rem;
  text-transform: uppercase;
  letter-spacing: .1em;
  color: #3a5878;
}
.hdr-pfe-meta .nm { font-weight: 700; font-size: .8rem; color: #c8e0f8; }
.hdr-pfe-meta .sup { font-weight: 600; font-size: .74rem; color: #00d4ff; }
.hdr-main-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 10px 20px;
  gap: 12px;
}
@media (max-width: 720px) {
  .hdr-pfe-meta { display: none; }
  .hdr-pfe-logo { height: 44px; max-width: 92px; }
}
"""

HDR_HTML = """<header id="hdr">
  <div class="hdr-pfe-row">
    <img class="hdr-pfe-logo" src="/ros/demo_assets/uae.png" alt="Université Abdelmalek Essaâdi" />
    <div class="hdr-pfe-center">
      <div class="hdr-pfe-uni">Université Abdelmalek Essaâdi · FST Al Hoceima</div>
      <div class="hdr-pfe-sub">MST Embedded Systems &amp; Robotics (SER) — PFE demo</div>
      <span class="hdr-pfe-year">2025/2026</span>
    </div>
    <img class="hdr-pfe-logo" src="/ros/demo_assets/fsth.png" alt="FST Al Hoceima" />
    <div class="hdr-pfe-meta">
      <label>Author</label>
      <span class="nm">RABEH Houssam Eddine</span>
    </div>
    <div class="hdr-pfe-meta right">
      <label>Supervisors</label>
      <span class="sup">Pr. Nabil El Akchioui</span>
      <span class="sup">Pr. Walid Jebrane</span>
    </div>
  </div>
  <div class="hdr-main-row">
  <div id="hdr-left">
    <span class="hex">⬡</span>
    <h1>Neural Nav <em>· Fast Adaptation Lab</em></h1>
  </div>
  <div style="display:flex;align-items:center;gap:12px">
    <div id="ws-pill"><span class="dot"></span><span id="ws-text">OFFLINE</span><span id="build-tag" style="font-size:.65rem;color:#3a5878;margin-left:6px">v14</span></div>
  </div>
  </div>
</header>"""


def fetch_live() -> str:
    req = urllib.request.Request(LIVE_URL, headers={"User-Agent": "pfe-patch/1"})
    with urllib.request.urlopen(req, timeout=30) as resp:
        return resp.read().decode("utf-8", errors="replace")


ACADEMIC_CSS_RE = re.compile(
    r"/\* pfe-academic-hdr-v\d+ \*/.*?@media \(max-width: 720px\) \{\s*"
    r"\.hdr-pfe-meta \{ display: none; \}\s*"
    r"\.hdr-pfe-logo \{ height: \d+px; max-width: \d+px; \}\s*\}",
    re.DOTALL,
)


def patch(text: str) -> str:
    if ACADEMIC_CSS_RE.search(text):
        text = ACADEMIC_CSS_RE.sub(HDR_CSS.strip(), text, count=1)
        print("updated academic CSS")
    else:
        text = text.replace("</style>", HDR_CSS + "\n</style>", 1)

    hdr_pat = re.compile(r"<header id=\"hdr\">.*?</header>", re.DOTALL)
    if not hdr_pat.search(text):
        raise SystemExit("Could not find <header id=\"hdr\">…</header>")
    text = hdr_pat.sub(HDR_HTML, text, count=1)

    text = re.sub(r"<!-- ros-live-v\d+ -->", "<!-- ros-live-v14 -->", text, count=1)
    text = re.sub(r"id=\"build-tag\"[^>]*>v\d+</span>", 'id="build-tag" style="font-size:.65rem;color:#3a5878;margin-left:6px">v14</span>', text, count=1)
    text = re.sub(r"config\.js\?v=\d+", "config.js?v=14", text)
    text = re.sub(r"ros_live_canvas\.js\?v=\d+", "ros_live_canvas.js?v=4", text)
    return text


def main() -> None:
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    if src and src.is_file():
        text = src.read_text(encoding="utf-8")
        print(f"read {src}")
    elif LIVE_URL:
        print(f"fetch {LIVE_URL}")
        text = fetch_live()
    elif OUT_INDEX.is_file():
        text = OUT_INDEX.read_text(encoding="utf-8")
        print(f"read {OUT_INDEX}")
    else:
        raise SystemExit(f"Missing {OUT_INDEX} — copy a base index.html first or pass a source file.")

    patched = patch(text)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name in ("uae.png", "fsth.png"):
        (OUT_DIR / "demo_assets").mkdir(parents=True, exist_ok=True)
        dest = OUT_DIR / "demo_assets" / name
        dest.write_bytes((SRC_ASSETS / name).read_bytes())
    OUT_INDEX.write_text(patched, encoding="utf-8")
    print(f"Wrote {OUT_INDEX} ({len(patched)} bytes)")
    print(f"Assets → {OUT_DIR / 'demo_assets'}")


if __name__ == "__main__":
    main()
