#!/usr/bin/env python3
"""
generate_eval_worlds.py — Three cinematic evaluation environments.
Downloads CC0 textures from Poly Haven, writes custom.material,
then generates eval_rabat.world, eval_egypt.world, eval_maze.world.
Falls back to built-in Gazebo materials if download fails.
"""
from __future__ import annotations
import math, os, shutil, time, urllib.error, urllib.request

# ── Paths ──────────────────────────────────────────────────────────────────────
_SD  = os.path.dirname(os.path.abspath(__file__))          # .../scripts/
_SA  = os.path.normpath(os.path.join(_SD, ".."))           # .../sim_assets/
_WS  = os.path.normpath(os.path.join(_SD, "..", "..", "..", ".."))  # ros2_ws/
_DEMO_ASSETS = os.path.join(_WS, "demo_assets")
_WD  = os.path.join(_SA, "worlds")
_MS  = os.path.join(_SA, "materials", "scripts")           # material + textures here
_HASSAN_MAT_URI = f"file://{os.path.abspath(_MS)}/hassan.material"
for _d in (_WD, _MS):
    os.makedirs(_d, exist_ok=True)


def _sync_hassan_from_desktop() -> None:
    """Copy hassan_*.png from ~/Desktop (never copy hassan.material — often truncated)."""
    desk = os.path.join(os.path.expanduser("~"), "Desktop")
    for name in ("hassan_floor.png", "hassan_pillar.png", "hassan_tower.png"):
        src = os.path.join(desk, name)
        dst = os.path.join(_MS, name)
        if os.path.isfile(src):
            shutil.copy2(src, dst)


def _cover_crop(im, tw: int, th: int):
    """Center-crop resize so texture fills the whole PNG (no letterbox → no black/white gaps)."""
    from PIL import Image  # type: ignore

    w, h = im.size
    scale = max(tw / w, th / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    im = im.resize((nw, nh), getattr(Image, "LANCZOS", 1))
    left = (nw - tw) // 2
    top = (nh - th) // 2
    return im.crop((left, top, left + tw, top + th))


def _fix_pillar_png_for_cylinder(path: str) -> None:
    """Stone-only atlas: OGRE cylinder caps sample the image centre — no black bullseye."""
    from PIL import Image  # type: ignore

    im = Image.open(path)
    stone = (192, 186, 178)
    if im.mode == "RGBA":
        bg = Image.new("RGB", im.size, stone)
        bg.paste(im, mask=im.split()[3])
        im = bg
    else:
        im = im.convert("RGB")
    px = im.load()
    w, h = im.size
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            if r < 50 and g < 50 and b < 50:
                px[x, y] = stone
    # Average colour from side strip (middle 60% height) for caps + centre fill
    ys, rs, gs, bs, n = int(h * 0.2), 0, 0, 0, 0
    ye = int(h * 0.8)
    for y in range(ys, ye):
        for x in range(w):
            r, g, b = px[x, y]
            if r > 50 or g > 50 or b > 50:
                rs += r
                gs += g
                bs += b
                n += 1
    if n:
        stone = (rs // n, gs // n, bs // n)
    cx, cy = w // 2, h // 2
    cap_r = int(min(w, h) * 0.42)
    cap_r2 = cap_r * cap_r
    for y in range(h):
        for x in range(w):
            dy, dx = y - cy, x - cx
            if dx * dx + dy * dy <= cap_r2:
                px[x, y] = stone
    band = max(4, h // 14)
    for y in list(range(band)) + list(range(h - band, h)):
        for x in range(w):
            px[x, y] = stone
    im = _cover_crop(im, 256, 768)
    im.save(path, optimize=True)


def _lighten_image(im, brightness: float = 1.32):
    from PIL import ImageEnhance  # type: ignore

    im = ImageEnhance.Brightness(im).enhance(brightness)
    return ImageEnhance.Color(im).enhance(0.88)


def _avg_rgb(im, y0_frac: float = 0.0, y1_frac: float = 1.0) -> tuple[int, int, int]:
    px = im.load()
    w, h = im.size
    y0, y1 = int(h * y0_frac), int(h * y1_frac)
    rs = gs = bs = n = 0
    for y in range(y0, y1):
        for x in range(w):
            r, g, b = px[x, y]
            rs += r
            gs += g
            bs += b
            n += 1
    if not n:
        return (200, 195, 188)
    return (rs // n, gs // n, bs // n)


def _reference_photo_path() -> str | None:
    candidates = [os.path.join(_MS, "hassan_reference.png")]
    if os.path.isdir(_DEMO_ASSETS):
        for name in os.listdir(_DEMO_ASSETS):
            if name.lower().endswith((".png", ".jpg", ".jpeg")) and "hassan" in name.lower():
                candidates.append(os.path.join(_DEMO_ASSETS, name))
    for p in candidates:
        if os.path.isfile(p):
            return p
    return None


def _plaza_screenshot_path() -> str | None:
    """Optional paving reference image for ground UV tiling."""
    p = os.path.join(_MS, "hassan_floor_plaza.png")
    if os.path.isfile(p):
        return p
    if os.path.isdir(_DEMO_ASSETS):
        for name in sorted(os.listdir(_DEMO_ASSETS)):
            low = name.lower()
            if low.endswith((".png", ".jpg", ".jpeg")) and "plaza" in low:
                return os.path.join(_DEMO_ASSETS, name)
    return None


def _crop_plaza_tile_from_reference(ref: str, dst: str) -> None:
    """Plaza paving tile — full frame for close-up slab photos, else crop from wide shot."""
    from PIL import Image  # type: ignore

    im = Image.open(ref).convert("RGB")
    w, h = im.size
    if w / max(1, h) > 1.35:
        crop = im
    else:
        crop = im.crop((int(w * 0.06), int(h * 0.52), int(w * 0.94), int(h * 0.90)))
    tw = 896
    th = max(160, int(tw * crop.height / max(1, crop.width)))
    crop = crop.resize((tw, th), Image.LANCZOS)
    crop.save(dst, optimize=True)


def _neutralize_pillar_stone(path: str) -> None:
    """Keep stone grain but force light warm grey (no sky-blue / black)."""
    from PIL import Image  # type: ignore

    im = Image.open(path).convert("RGB")
    px = im.load()
    w, h = im.size
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            lum = int(0.30 * r + 0.59 * g + 0.11 * b)
            lum = max(155, min(215, lum + 22))
            px[x, y] = (lum + 10, lum + 6, lum + 2)
    im.save(path, optimize=True)


def _crop_pillar_texture_from_reference(ref: str, dst: str) -> tuple[int, int, int]:
    """Light grey stone columns sampled from the reference colonnade."""
    from PIL import Image  # type: ignore

    im = Image.open(ref).convert("RGB")
    w, h = im.size
    # Stone stumps only (avoid sky / tower)
    patch = im.crop((int(w * 0.20), int(h * 0.48), int(w * 0.38), int(h * 0.74)))
    patch = patch.resize((256, 768), Image.LANCZOS)
    patch.save(dst, optimize=True)
    _neutralize_pillar_stone(dst)
    return _avg_rgb(Image.open(dst))


def _tile_floor_plaza(src: str, dst: str, out_size: int = 1024) -> None:
    """Repeat the real plaza screenshot to cover the whole floor UV atlas."""
    from PIL import Image  # type: ignore

    tile = Image.open(src).convert("RGB")
    tw, th = tile.size
    out = Image.new("RGB", (out_size, out_size))
    for y in range(0, out_size, th):
        for x in range(0, out_size, tw):
            out.paste(tile, (x, y))
    out.save(dst, optimize=True)


def _tile_floor_plaza_few(
    src: str, dst: str, *, nx: int = 2, ny: int = 2, out_size: int = 1024
) -> None:
    """Tile the plaza photo only a few times (2×2) — visible paving, not a blur."""
    from PIL import Image  # type: ignore

    tile = Image.open(src).convert("RGB")
    tw, th = tile.size
    cell_w = out_size // nx
    cell_h = int(cell_w * th / max(1, tw))
    if cell_h * ny > out_size:
        cell_h = out_size // ny
        cell_w = int(cell_h * tw / max(1, th))
    tile = tile.resize((cell_w, cell_h), Image.LANCZOS)
    bg = _avg_rgb(tile)
    out = Image.new("RGB", (out_size, out_size), bg)
    for j in range(ny):
        for i in range(nx):
            out.paste(tile, (i * cell_w, j * cell_h))
    out.save(dst, optimize=True)


def _rgb_to_ogre(
    rgb: tuple[int, int, int], *, amb: float = 0.80, diff: float = 0.92
) -> tuple[float, float, float, float, float, float]:
    r, g, b = rgb
    return (
        r / 255 * amb,
        g / 255 * amb,
        b / 255 * amb,
        r / 255 * diff,
        g / 255 * diff,
        b / 255 * diff,
    )


def _avg_rgb_from_png(name: str) -> tuple[int, int, int] | None:
    path = os.path.join(_MS, name)
    if not os.path.isfile(path):
        return None
    try:
        from PIL import Image  # type: ignore

        return _avg_rgb(Image.open(path))
    except Exception:
        return None


def _tile_stone_ground(
    pillar_uv: str, dst: str, *, out_size: int = 1024, tiles_across: int = 4
) -> None:
    """Plaza floor — same stone family as columns (cohesive site palette)."""
    from PIL import Image  # type: ignore

    im = Image.open(pillar_uv).convert("RGB")
    tw, th = im.size
    cell = max(96, out_size // tiles_across)
    cell_h = max(48, int(cell * th / max(1, tw)))
    tile = im.resize((cell, cell_h), Image.LANCZOS)
    tw, th = tile.size
    bg = _avg_rgb(tile)
    out = Image.new("RGB", (out_size, out_size), bg)
    for y in range(0, out_size, th):
        for x in range(0, out_size, tw):
            out.paste(tile, (x, y))
    out.save(dst, optimize=True)


def _make_tower_top_cap(brown: tuple[int, int, int], dst: str, size: int = 256) -> None:
    """Flat mosque roof — warm brown stone (no yellow wash-out in OGRE)."""
    from PIL import Image  # type: ignore

    r, g, b = brown
    im = Image.new("RGB", (size, size), brown)
    px = im.load()
    for y in range(size):
        for x in range(size):
            v = ((x * 5 + y * 3) % 11) - 5
            px[x, y] = (
                max(0, min(255, r + v)),
                max(0, min(255, g + v - 2)),
                max(0, min(255, b + v - 5)),
            )
    im.save(dst, optimize=True)


def _tile_floor_plaza_cover(
    src: str, dst: str, *, out_size: int = 1024, tiles_across: int = 5
) -> None:
    """Tile paving photo edge-to-edge on the floor UV (stone only, no sky)."""
    from PIL import Image  # type: ignore

    im = Image.open(src)
    if im.mode == "RGBA":
        bg = Image.new("RGB", im.size, (195, 176, 145))
        bg.paste(im, mask=im.split()[3])
        im = bg
    else:
        im = im.convert("RGB")
    tw, th = im.size
    cell_w = max(80, out_size // tiles_across)
    cell_h = max(40, int(cell_w * th / max(1, tw)))
    tile = im.resize((cell_w, cell_h), Image.LANCZOS)
    tw, th = tile.size
    out = Image.new("RGB", (out_size, out_size))
    for y in range(0, out_size, th):
        for x in range(0, out_size, tw):
            out.paste(tile, (x, y))
    out.save(dst, optimize=True)


def _fill_tower_whites(path: str, threshold: int = 205) -> tuple[int, int, int]:
    """Replace near-white pixels with brown sampled from the mosque texture."""
    from PIL import Image  # type: ignore

    im = Image.open(path).convert("RGB")
    px = im.load()
    w, h = im.size
    rs = gs = bs = n = 0
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            if r > threshold and g > threshold and b > threshold - 5:
                continue
            if r < 70 and g < 70 and b < 70:
                continue
            rs += r
            gs += g
            bs += b
            n += 1
    fill = (rs // n, gs // n, bs // n) if n else (198, 172, 138)
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            if r > threshold and g > threshold and b > threshold - 8:
                px[x, y] = fill
            elif r > 235 and g > 235 and b > 230:
                px[x, y] = fill
    im.save(path, optimize=True)
    return fill


def _make_grey_plaza_floor(path: str, base: tuple[int, int, int] = (168, 163, 155)) -> None:
    """Medium grey plaza tiles (darker than pillars — readable in Gazebo)."""
    from PIL import Image  # type: ignore

    w, h = 256, 256
    hi = tuple(min(255, c + 10) for c in base)
    lo = tuple(max(0, c - 12) for c in base)
    tile = Image.new("RGB", (w, h), base)
    px = tile.load()
    for y in range(h):
        band = (y // 64) % 2
        col = lo if band else hi
        for x in range(w):
            v = ((x * 3 + y * 5) % 11) - 5
            px[x, y] = tuple(max(0, min(255, c + v)) for c in col)
    out = Image.new("RGB", (1024, 1024), base)
    for y in range(0, 1024, h):
        for x in range(0, 1024, w):
            out.paste(tile, (x, y))
    out.save(path, optimize=True)


def _make_grey_plaza_pillar(path: str, base: tuple[int, int, int]) -> None:
    """Grey column texture — same tone as light plaza (no brown drums)."""
    from PIL import Image  # type: ignore

    w, h = 256, 768
    hi = tuple(min(255, c + 12) for c in base)
    lo = tuple(max(0, c - 14) for c in base)
    im = Image.new("RGB", (w, h), base)
    px = im.load()
    for y in range(h):
        band = (y // 56) % 2
        col = lo if band else hi
        for x in range(w):
            v = ((x * 5 + y * 2) % 13) - 6
            px[x, y] = tuple(max(0, min(255, c + v)) for c in col)
    im.save(path, optimize=True)


def _prepare_hassan_simple() -> None:
    """Light grey plaza, grey pillars, mosque with 4 facades + brown top."""
    _sync_hassan_from_desktop()
    try:
        from PIL import Image  # type: ignore
    except ImportError:
        print("  ⚠  Pillow missing — using raw PNGs")
        return

    plaza_grey = (168, 163, 155)
    stone_grey = (192, 188, 184)
    floor_uv = os.path.join(_MS, "hassan_floor_uv.png")
    ref = _reference_photo_path()

    pillar_uv = os.path.join(_MS, "hassan_pillar_uv.png")
    if ref:
        pillar_grey = _crop_pillar_texture_from_reference(ref, pillar_uv)
        _fix_pillar_png_for_cylinder(pillar_uv)
        print(f"  ✅  light grey pillars ← reference photo {pillar_grey}")
    else:
        _make_grey_plaza_pillar(pillar_uv, stone_grey)
        _fix_pillar_png_for_cylinder(pillar_uv)
        print(f"  ✅  light grey pillars {stone_grey}")

    # Default to the "beautiful" paving look. Set PFE_FLOOR_PAVEMENT=0 to force grey floor.
    use_pavement = os.environ.get("PFE_FLOOR_PAVEMENT", "1").strip().lower() not in (
        "0",
        "false",
        "no",
        "off",
    )
    plaza_dst = os.path.join(_MS, "hassan_floor_plaza.png")
    plaza_shot = _plaza_screenshot_path() if use_pavement else None
    if plaza_shot:
        _crop_plaza_tile_from_reference(plaza_shot, plaza_dst)
        _tile_floor_plaza_cover(plaza_dst, floor_uv, tiles_across=5)
        print(f"  ✅  ground = paving photo ({os.path.basename(plaza_dst)})")
    else:
        _make_grey_plaza_floor(floor_uv, plaza_grey)
        print(f"  ✅  ground = light grey plaza {plaza_grey}")

    tower_src = os.path.join(_MS, "hassan_tower.png")
    backdrop = os.path.join(_MS, "hassan_backdrop.png")
    src = tower_src if os.path.isfile(tower_src) else backdrop
    if os.path.isfile(src):
        face_path = os.path.join(_MS, "hassan_tower_face.png")
        im = Image.open(src).convert("RGB")
        im = _cover_crop(im, 512, 1024)
        im.save(face_path, optimize=True)
        core_brown = _fill_tower_whites(face_path)
        _make_tower_top_cap(core_brown, os.path.join(_MS, "hassan_tower_top.png"))
        print(f"  ✅  mosque faces (whites→brown {core_brown}) + top cap {core_brown}")
    else:
        brown = (208, 188, 158)
        Image.new("RGB", (128, 128), brown).save(
            os.path.join(_MS, "hassan_tower_top.png"), optimize=True
        )
        Image.new("RGB", (512, 1024), brown).save(
            os.path.join(_MS, "hassan_tower_face.png"), optimize=True
        )
        print("  ✅  mosque fallback light-brown textures")


def _make_pillar_stone_atlas(path: str) -> None:
    """Procedural drum-stone column (no sky blue → no 'battery' caps on cylinders)."""
    from PIL import Image  # type: ignore

    w, h = 256, 640
    base = (192, 186, 176)
    joint = (168, 160, 150)
    im = Image.new("RGB", (w, h), base)
    px = im.load()
    for y in range(h):
        band = (y // 48) % 2
        col = joint if band else base
        for x in range(w):
            # slight variation
            v = ((x * 7 + y * 3) % 17) - 8
            px[x, y] = tuple(max(0, min(255, c + v)) for c in col)
    im.save(path, optimize=True)


def _prepare_pillar_uv() -> None:
    """Stone column texture only — never crop sky from the plaza reference photo."""
    from PIL import Image  # type: ignore

    dst = os.path.join(_MS, "hassan_pillar_uv.png")
    src = os.path.join(_MS, "hassan_pillar.png")
    desk = os.path.join(os.path.expanduser("~"), "Desktop", "hassan_pillar.png")
    if os.path.isfile(desk):
        shutil.copy2(desk, src)
    if os.path.isfile(src):
        im = Image.open(src).convert("RGB")
        im = _cover_crop(im, 280, 720)
        im.save(dst, optimize=True)
        print(f"  ✅  hassan_pillar_uv.png ← hassan_pillar.png (column stone)")
        return
    _make_pillar_stone_atlas(dst)
    print("  ✅  hassan_pillar_uv.png (procedural stone drums)")


def _find_hassan_reference() -> str | None:
    """Real Tour Hassan photo (plaza + tower + sky) — best visual source."""
    here = os.path.join(_MS, "hassan_reference.png")
    if os.path.isfile(here):
        return here
    desk = os.path.join(os.path.expanduser("~"), "Desktop", "hassan_reference.jpg")
    if os.path.isfile(desk):
        return desk
    if os.path.isdir(_DEMO_ASSETS):
        for name in os.listdir(_DEMO_ASSETS):
            if name.lower().endswith((".png", ".jpg", ".jpeg")) and "hassan" in name.lower():
                return os.path.join(_DEMO_ASSETS, name)
    return None


def _prepare_hassan_uv_textures() -> None:
    """
    Build textures from the real Hassan Tower reference photo (not tiny strips on cylinders).
    - hassan_backdrop.png → full tower + sky on a vertical plane (like the real view)
    - hassan_floor_uv.png → plaza paving from photo
    Pillars use hassan_pillar.png / procedural stone (never sky from reference).
    """
    try:
        from PIL import Image  # type: ignore
    except ImportError:
        print("  ⚠  PIL missing — skip UV prep (pip install Pillow)")
        return

    ref = _find_hassan_reference()
    if ref:
        print(f"  📷  reference photo → {ref}")
        im = Image.open(ref).convert("RGB")
        w, h = im.size
        # Tower + sky (center-top) — one flat backdrop, no cylinder wrap
        bd = im.crop((int(0.20 * w), 0, int(0.80 * w), int(0.70 * h)))
        bd = _cover_crop(bd, 960, 1152)
        bd.save(os.path.join(_MS, "hassan_backdrop.png"), optimize=True)
        print("  ✅  hassan_backdrop.png (tower + sky, for backdrop plane)")
        # Plaza floor
        fl = im.crop((0, int(0.48 * h), w, h))
        fl = _cover_crop(fl, 1024, 1024)
        fl.save(os.path.join(_MS, "hassan_floor_uv.png"), optimize=True)
        print("  ✅  hassan_floor_uv.png (plaza paving)")
        _prepare_pillar_uv()
        ref_dst = os.path.join(_MS, "hassan_reference.png")
        if os.path.abspath(ref) != os.path.abspath(ref_dst):
            shutil.copy2(ref, ref_dst)
        return

    _prepare_pillar_uv()
    if os.path.isfile(os.path.join(_MS, "hassan_floor.png")):
        im = Image.open(os.path.join(_MS, "hassan_floor.png")).convert("RGB")
        im = _cover_crop(im, 1024, 1024)
        im.save(os.path.join(_MS, "hassan_floor_uv.png"), optimize=True)


def _png_size(name: str) -> tuple[int, int]:
    """Return (width, height) in pixels; used to auto-fit OGRE texture scale."""
    path = os.path.join(_MS, name)
    try:
        from PIL import Image  # type: ignore

        with Image.open(path) as im:
            return im.size
    except Exception:
        return (1024, 1024)


def _clamp(v: float, lo: float = 0.06, hi: float = 8.0) -> float:
    return max(lo, min(hi, v))


def _hassan_texture_scales() -> dict[str, tuple[float, float, str]]:
    floor_tex = "hassan_floor_uv.png"
    if not os.path.isfile(os.path.join(_MS, floor_tex)):
        floor_tex = "hassan_floor_plaza.png"
    if not os.path.isfile(os.path.join(_MS, floor_tex)):
        floor_tex = "hassan_pillar_uv.png"
    face_tex = "hassan_tower_face.png"
    if not os.path.isfile(os.path.join(_MS, face_tex)):
        face_tex = "hassan_tower_uv.png"
    if floor_tex == "hassan_floor_plaza.png":
        floor_scale = (6.0, 2.2, floor_tex)
    else:
        floor_scale = (5.0, 5.0, floor_tex)
    return {
        "floor": floor_scale,
        "pillar": (1.0, 0.35, "hassan_pillar_uv.png"),
        "tower_face": (1.0, 1.0, face_tex),
        # Roof uses same stone texture as vertical facades (avoids wrong tint / "green" cap).
        "tower_top": (1.0, 1.0, face_tex),
    }


def _floor_grey_rgba() -> tuple[float, float, float, float, float, float]:
    """Floor tint; keep it readable if textures fail."""
    rgb = _avg_rgb_from_png("hassan_floor_uv.png") or (175, 156, 141)
    # Slightly darker than before, but not the very dark grey fallback.
    return _rgb_to_ogre(rgb, amb=0.64, diff=0.74)


def _tower_top_rgba() -> tuple[float, float, float, float, float, float]:
    """Roof cap — solid brown (no texture), same as mosque core stone."""
    return _tower_core_rgba()


def _tower_core_rgba() -> tuple[float, float, float, float, float, float]:
    """Light brown core matching filled mosque stone (no white gaps)."""
    face = os.path.join(_MS, "hassan_tower_face.png")
    if os.path.isfile(face):
        try:
            from PIL import Image  # type: ignore

            r, g, b = _avg_rgb(Image.open(face))
            return (
                r / 255 * 0.85,
                g / 255 * 0.85,
                b / 255 * 0.85,
                r / 255 * 0.95,
                g / 255 * 0.95,
                b / 255 * 0.95,
            )
        except Exception:
            pass
    return (0.72, 0.62, 0.48, 0.82, 0.70, 0.54)


def _write_hassan_material() -> None:
    """Write hassan.material with auto-fitted texture scales from PNG + geometry."""
    path = os.path.join(_MS, "hassan.material")
    s = _hassan_texture_scales()
    fsu, fsv, ftex = s["floor"]
    psu, psv, ptex = s["pillar"]
    tfu, tfv, tftex = s["tower_face"]
    fa, fb, fc, fd, fe, ff = _floor_grey_rgba()
    ca, cb, cc, cd, ce, cf = _tower_core_rgba()
    tpa, tpb, tpc, tpd, tpe, tpf = _tower_top_rgba()
    body = f"""
material Hassan/Floor
{{
  technique
  {{
    pass
    {{
      ambient  {fa:.3f} {fb:.3f} {fc:.3f} 1.0
      diffuse  {fd:.3f} {fe:.3f} {ff:.3f} 1.0
      specular 0.03 0.03 0.03 1.0 4.0
      texture_unit
      {{
        texture {ftex}
        scale {fsu} {fsv}
        tex_address_mode wrap
      }}
    }}
  }}
}}

material Hassan/Pillar
{{
  technique
  {{
    pass
    {{
      ambient  0.84 0.82 0.80 1.0
      diffuse  0.92 0.90 0.88 1.0
      specular 0.05 0.05 0.05 1.0 4.0
      texture_unit
      {{
        texture {ptex}
        scale {psu} {psv}
        tex_address_mode clamp
      }}
    }}
  }}
}}

material Hassan/TowerCore
{{
  technique
  {{
    pass
    {{
      ambient  {ca:.3f} {cb:.3f} {cc:.3f} 1.0
      diffuse  {cd:.3f} {ce:.3f} {cf:.3f} 1.0
      specular 0.04 0.04 0.04 1.0 4.0
    }}
  }}
}}

material Hassan/TowerFace
{{
  technique
  {{
    pass
    {{
      ambient  0.92 0.88 0.80 1.0
      diffuse  1.0 0.96 0.88 1.0
      specular 0.05 0.05 0.05 1.0 4.0
      texture_unit
      {{
        texture {tftex}
        scale {tfu} {tfv}
        tex_address_mode clamp
      }}
    }}
  }}
}}

material Hassan/TowerTop
{{
  technique
  {{
    pass
    {{
      ambient  {tpa:.3f} {tpb:.3f} {tpc:.3f} 1.0
      diffuse  {tpd:.3f} {tpe:.3f} {tpf:.3f} 1.0
      specular 0.05 0.05 0.05 1.0 4.0
    }}
  }}
}}
"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(body.strip() + "\n")
    print(
        f"  ✅  hassan.material → floor pillar tower_face tower_top"
    )


def _hassan_assets_ok() -> bool:
    return os.path.isfile(os.path.join(_MS, "hassan_pillar_uv.png"))


def _hassan_mat(name: str, png: str = "") -> str:
    """Single directory URI — duplicate .png URI on every visual freezes gzclient."""
    dir_uri = f"file://{os.path.abspath(_MS)}"
    return (
        f"          <material><script>"
        f"<uri>{dir_uri}</uri><name>{name}</name></script></material>"
    )

# Absolute URI embedded in SDF <uri> tags (file:// + /abs/path = 3 slashes on Linux)
_MAT_URI = f"file://{os.path.abspath(_MS)}/custom.material"
_GZ_URI  = "file://media/materials/scripts/gazebo.material"

# ── Texture manifest (Poly Haven CC0 defaults; override by dropping JPGs in _MS) ─
#   sand.jpg   → floor (horizontal plane)
#   pillar.jpg → columns (use concrete — NOT rock_wall / stone.jpg on cylinders)
#   stone.jpg  → walls / rubble boxes only
_T = {
    "sand": dict(
        url="https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/sand_01/sand_01_diff_1k.jpg",
        file="sand.jpg", fallback="Gazebo/Orange", mat="Custom/Sand", scale="0.12 0.12"),
    "pillar": dict(
        url="https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/concrete_wall_008/concrete_wall_008_diff_1k.jpg",
        file="pillar.jpg", fallback="Gazebo/Grey", mat="Custom/Pillar", scale="0.35 0.35"),
    "stone": dict(
        url="https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/rough_concrete/rough_concrete_diff_1k.jpg",
        file="stone.jpg", fallback="Gazebo/DarkGrey", mat="Custom/Stone", scale="0.20 0.20"),
    "hedge": dict(
        url="https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/brown_mud_leaves_01/brown_mud_leaves_01_diff_1k.jpg",
        file="hedge.jpg", fallback="Gazebo/Green", mat="Custom/Hedge", scale="0.20 0.20"),
}
_ok: dict[str, bool] = {}


# ── 1. Texture downloader ──────────────────────────────────────────────────────
def _download_textures() -> None:
    print("── Downloading textures ─────────────────────────────────────────────")
    for k, v in _T.items():
        dst = os.path.join(_MS, v["file"])
        if os.path.exists(dst) and os.path.getsize(dst) > 4096:
            print(f"  ✅  {k}: cached ({os.path.getsize(dst)//1024} KB)")
            _ok[k] = True
            continue
        try:
            req = urllib.request.Request(
                v["url"], headers={"User-Agent": "Mozilla/5.0 safe_drl_nav/thesis"})
            t0 = time.time()
            with urllib.request.urlopen(req, timeout=25) as r, open(dst, "wb") as f:
                f.write(r.read())
            print(f"  ✅  {k}: {os.path.getsize(dst)//1024} KB in {time.time()-t0:.1f}s")
            _ok[k] = True
        except Exception as e:
            print(f"  ⚠   {k}: {e}  →  fallback: {v['fallback']}")
            _ok[k] = False


# ── 2. Ogre material script ────────────────────────────────────────────────────
def _write_material() -> None:
    lines = ["// custom.material — generated by generate_eval_worlds.py\n"]
    for k, v in _T.items():
        if not _ok.get(k):
            continue
        lines += [
            f"material {v['mat']}", "{", "    technique", "    {",
            "        pass", "        {",
            "            ambient  0.80 0.80 0.80 1.0",
            "            diffuse  1.00 1.00 1.00 1.0",
            "            specular 0.05 0.05 0.05 1.0 8.0",
            "            texture_unit", "            {",
            f"                texture {v['file']}",
            f"                scale   {v['scale']}",
            "            }", "        }", "    }", "}", "",
        ]
    path = os.path.join(_MS, "custom.material")
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"  ✅  custom.material → {path}")


# ── 3. SDF primitive helpers ───────────────────────────────────────────────────
def _mat(k: str | None, fb: str = "Gazebo/Grey") -> str:
    if k and _ok.get(k):
        return (f"          <material><script>"
                f"<uri>{_MAT_URI}</uri>"
                f"<name>{_T[k]['mat']}</name>"
                f"</script></material>")
    return (f"          <material><script>"
            f"<uri>{_GZ_URI}</uri>"
            f"<name>{fb}</name>"
            f"</script></material>")


def box(name: str, x: float, y: float, z: float,
        sx: float, sy: float, sz: float,
        tk: str | None = None, fb: str = "Gazebo/Grey") -> str:
    g = f"<box><size>{sx} {sy} {sz}</size></box>"
    return (
        f'\n  <model name="{name}"><static>true</static>'
        f"<pose>{x} {y} {z} 0 0 0</pose><link name=\"link\">"
        f"<collision name=\"c\"><geometry>{g}</geometry></collision>"
        f"<visual name=\"v\"><geometry>{g}</geometry>\n{_mat(tk, fb)}\n"
        f"</visual></link></model>"
    )


def cyl(name: str, x: float, y: float, z: float,
        r: float, h: float,
        tk: str | None = None, fb: str = "Gazebo/Grey") -> str:
    g = f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder>"
    return (
        f'\n  <model name="{name}"><static>true</static>'
        f"<pose>{x} {y} {z} 0 0 0</pose><link name=\"link\">"
        f"<collision name=\"c\"><geometry>{g}</geometry></collision>"
        f"<visual name=\"v\"><geometry>{g}</geometry>\n{_mat(tk, fb)}\n"
        f"</visual></link></model>"
    )


# ── 4. Shared world fragments ──────────────────────────────────────────────────
_PHYSICS = """
  <physics type="ode">
    <max_step_size>0.01</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>100</real_time_update_rate>
    <ode>
      <solver><type>quick</type><iters>50</iters><sor>1.3</sor></solver>
      <constraints><cfm>0</cfm><erp>0.2</erp>
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>"""

# Teleport robot between episodes (pfe_reset_simulation.py → /set_entity_state).
_GAZEBO_ROS_STATE = """
  <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros><namespace>/</namespace></ros>
    <update_rate>20.0</update_rate>
  </plugin>"""


def _scene(amb: str, bg: str, fog: str, f0: float, f1: float) -> str:
    return (f"\n  <scene><ambient>{amb}</ambient><background>{bg}</background>"
            f"<shadows>false</shadows><grid>false</grid>"
            f"<fog><color>{fog}</color><type>linear</type>"
            f"<start>{f0}</start><end>{f1}</end><density>0.05</density>"
            f"</fog></scene>")


def _gui_camera_rabat() -> str:
    """Default orbit camera — plaza view toward Tour Hassan backdrop."""
    return """
  <gui fullscreen='0'>
    <camera name='user_camera'>
      <pose>0.0 -10.0 6.5 0 0.45 1.57</pose>
      <view_controller>orbit</view_controller>
      <projection_type>perspective</projection_type>
    </camera>
  </gui>"""


def _scene_hassan_sky() -> str:
    """Blue outdoor scene — flat background only (procedural <sky> freezes gzclient)."""
    return _scene_hassan_record()


def _scene_hassan_record() -> str:
    """Textured Rabat: flat ciel bleu (no procedural <sky> — freezes gzclient)."""
    return """
  <scene>
    <ambient>0.92 0.95 1.0 1</ambient>
    <background>0.55 0.78 0.98 1</background>
    <shadows>false</shadows>
    <grid>false</grid>
  </scene>
  <light type="directional" name="sun_warm">
    <cast_shadows>false</cast_shadows>
    <diffuse>1.0 0.97 0.88 1</diffuse>
    <specular>0.15 0.12 0.08 1</specular>
    <direction>-0.35 0.12 -0.92</direction>
  </light>
  <light type="directional" name="sky_fill">
    <cast_shadows>false</cast_shadows>
    <diffuse>0.45 0.55 0.75 1</diffuse>
    <direction>0.1 -0.05 -1.0</direction>
  </light>"""


def _hassan_floor_fast(size: float = 22.0) -> str:
    m = _mat(None, "Gazebo/Grey")
    return (
        f"\n  <model name=\"hassan_floor\"><static>true</static><pose>0 0 0 0 0 0</pose>"
        f"<link name=\"link\"><collision name=\"c\"><geometry>"
        f"<plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>"
        f"<surface><friction><ode><mu>0.65</mu><mu2>0.65</mu2></ode></friction></surface>"
        f"</collision><visual name=\"v\"><geometry>"
        f"<plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>\n{m}\n"
        f"</visual></link></model>"
    )


def _hassan_floor(size: float = 26.0) -> str:
    m = _hassan_mat("Hassan/Floor")
    return (
        f"\n  <model name=\"hassan_floor\"><static>true</static><pose>0 0 0 0 0 0</pose>"
        f"<link name=\"link\"><collision name=\"c\"><geometry>"
        f"<plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>"
        f"<surface><friction><ode><mu>0.65</mu><mu2>0.65</mu2></ode></friction></surface>"
        f"</collision><visual name=\"v\"><geometry>"
        f"<plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>\n{m}\n"
        f"</visual></link></model>"
    )


# Rabat drivable plaza (single plane — no outer border ring)
_RABAT_ARENA_SIZE = 28.0
_RABAT_ARENA_HALF = _RABAT_ARENA_SIZE / 2.0


def _hassan_arena_ground(*, fast: bool) -> list[str]:
    """One paving plane for the whole arena — no brown/grey border."""
    size = _RABAT_ARENA_SIZE
    m = _mat(None, "Gazebo/Grey") if fast else _hassan_mat("Hassan/Floor")
    return [
        (
            f"\n  <model name=\"hassan_plaza\"><static>true</static><pose>0 0 0 0 0 0</pose>"
            f"<link name=\"link\"><collision name=\"c\"><geometry>"
            f"<plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>"
            f"<surface><friction><ode><mu>0.65</mu><mu2>0.65</mu2></ode></friction></surface>"
            f"</collision><visual name=\"paving\"><geometry>"
            f"<plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>\n{m}\n"
            f"</visual></link></model>"
        )
    ]


def _hassan_arena_walls(half: float = _RABAT_ARENA_HALF, *, fast: bool) -> list[str]:
    """Low perimeter walls — robot stays on plaza (like your screenshot frame)."""
    h_col, t_col, hz_vis = 0.55, 0.28, 0.10
    span = half * 2
    mat = _mat(None, "Gazebo/White") if fast else _hassan_mat("Hassan/Pillar")
    zc = h_col / 2

    def wall(n: str, x: float, y: float, sx: float, sy: float) -> str:
        return (
            f"\n  <model name=\"{n}\"><static>true</static>"
            f"<pose>{x} {y} {zc} 0 0 0</pose><link name=\"link\">"
            f"<collision name=\"c\"><geometry><box><size>{sx} {sy} {h_col}</size></box></geometry></collision>"
            f"<visual name=\"v\"><geometry><box><size>{sx} {sy} {hz_vis}</size></box></geometry>\n{mat}\n"
            f"</visual></link></model>"
        )

    o = t_col / 2
    return [
        wall("wall_n", 0, half + o, span + t_col, t_col),
        wall("wall_s", 0, -half - o, span + t_col, t_col),
        wall("wall_e", half + o, 0, t_col, span + t_col),
        wall("wall_w", -half - o, 0, t_col, span + t_col),
    ]


def _hassan_cyl(name: str, x: float, y: float, z: float, r: float, h: float, *, fast: bool = False) -> str:
    if fast:
        return cyl(name, x, y, z, r, h, None, "Gazebo/Grey")
    m = _hassan_mat("Hassan/Pillar")
    return (
        f"\n  <model name=\"{name}\"><static>true</static>"
        f"<pose>{x} {y} {z} 0 0 0</pose><link name=\"link\">"
        f"<collision name=\"c\"><geometry>"
        f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>"
        f"<visual name=\"v\"><geometry>"
        f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>\n{m}\n"
        f"</visual></link></model>"
    )


def _hassan_tower_simple(*, fast: bool = False) -> str:
    """Light-brown core + mosque photo on 4 vertical faces + brown top cap."""
    h, zc, s = 14.0, 7.0, 3.4
    t = 0.06
    g_col = f"<box><size>{s} {s} {h}</size></box>"
    if fast:
        mat = _mat(None, "Gazebo/White")
        return (
            f"\n  <model name=\"hassan_mosque\"><static>true</static>"
            f"<pose>0 5.8 0 0 0 0</pose><link name=\"link\">"
            f"<collision name=\"c\"><pose>0 0 {zc} 0 0 0</pose><geometry>{g_col}</geometry></collision>"
            f"<visual name=\"tower\"><pose>0 0 {zc} 0 0 0</pose><geometry>{g_col}</geometry>\n{mat}\n</visual>"
            f"</link></model>"
        )
    m_core = _hassan_mat("Hassan/TowerCore")
    m_face = _hassan_mat("Hassan/TowerFace")
    m_top = _hassan_mat("Hassan/TowerTop")
    sc = s - 0.08
    hc = h - 0.06
    g_core = f"<box><size>{sc} {sc} {hc}</size></box>"
    g_s = f"<box><size>{s} {t} {h}</size></box>"
    g_e = f"<box><size>{t} {s} {h}</size></box>"
    g_top = f"<box><size>{s} {s} {t}</size></box>"
    half = s / 2 + t / 2
    z_top = zc + h / 2 + t / 2
    return (
        f"\n  <model name=\"hassan_mosque\"><static>true</static>"
        f"<pose>0 5.8 0 0 0 0</pose><link name=\"link\">"
        f"<collision name=\"c\"><pose>0 0 {zc} 0 0 0</pose><geometry>{g_col}</geometry></collision>"
        f"<visual name=\"core\"><pose>0 0 {zc} 0 0 0</pose><geometry>{g_core}</geometry>\n{m_core}\n</visual>"
        f"<visual name=\"face_s\"><pose>0 {-half} {zc} 0 0 0</pose><geometry>{g_s}</geometry>\n{m_face}\n</visual>"
        f"<visual name=\"face_n\"><pose>0 {half} {zc} 0 0 0</pose><geometry>{g_s}</geometry>\n{m_face}\n</visual>"
        f"<visual name=\"face_e\"><pose>{half} 0 {zc} 0 0 0</pose><geometry>{g_e}</geometry>\n{m_face}\n</visual>"
        f"<visual name=\"face_w\"><pose>{-half} 0 {zc} 0 0 0</pose><geometry>{g_e}</geometry>\n{m_face}\n</visual>"
        f"<visual name=\"top\"><pose>0 0 {z_top} 0 0 0</pose><geometry>{g_top}</geometry>\n{m_top}\n</visual>"
        f"</link></model>"
    )


def _rabat_pillar_stumps() -> list[tuple[float, float, float, float, int]]:
    """
    Colonnades left/right of central aisle → minaret (Tour Hassan photo layout).
    Taller toward the mosque; grey stone; aisle kept clear for robot + waypoints.
    """
    NO_GO = [
        ((-2.80, -1.80), 1.4),
        ((-4.50, -0.20), 1.4),
        ((-3.20,  1.20), 1.4),
        ((-2.00, -2.00), 1.5),
        ((0.0, 5.8), 2.5),
    ]

    def clear(px: float, py: float) -> bool:
        return all(math.hypot(px - cx, py - cy) >= r for (cx, cy), r in NO_GO)

    # Left / right colonnades (skip center aisle |x| < 1.1)
    col_x = [-4.3, -3.2, -2.1, 2.1, 3.2, 4.3]
    rows_y = [-3.6, -2.0, -0.4, 1.2, 2.8, 4.2]
    # Height grows toward minaret (perspective like photo)
    h_by_row = [2.6, 3.0, 3.4, 3.8, 4.2, 4.6]

    out: list[tuple[float, float, float, float, int]] = []
    idx = 0
    for j, y in enumerate(rows_y):
        h = h_by_row[j] + (idx % 3) * 0.15
        for x in col_x:
            if abs(x) < 1.15:
                continue
            if not clear(x, y):
                continue
            out.append((x, y, round(h / 2, 2), 0.30, round(h, 2), idx))
            idx += 1
    return out


def _lights(diff: str = "1.00 0.78 0.42 1.0", dr: str = "-0.40 0.25 -0.85") -> str:
    return (
        f"\n  <light type=\"directional\" name=\"golden_sun\">"
        f"<cast_shadows>false</cast_shadows><pose>0 0 20 0 0 0</pose>"
        f"<diffuse>{diff}</diffuse><specular>0.40 0.25 0.08 1.0</specular>"
        f"<direction>{dr}</direction>"
        f"<attenuation><range>1000</range><constant>0.9</constant>"
        f"<linear>0.01</linear><quadratic>0.001</quadratic></attenuation></light>"
        f"\n  <light type=\"directional\" name=\"sky_fill\">"
        f"<cast_shadows>false</cast_shadows><pose>0 0 20 0 0 0</pose>"
        f"<diffuse>0.22 0.20 0.32 1.0</diffuse><specular>0 0 0 1</specular>"
        f"<direction>0.10 -0.05 -1.00</direction>"
        f"<attenuation><range>1000</range><constant>1.0</constant>"
        f"<linear>0.0</linear><quadratic>0.0</quadratic></attenuation></light>"
    )


def _ground(tk: str | None, fb: str, sz: float = 24.0) -> str:
    m = _mat(tk, fb)
    return (
        f"\n  <model name=\"ground_plane\"><static>true</static>"
        f"<pose>0 0 0 0 0 0</pose><link name=\"link\">"
        f"<collision name=\"c\"><geometry>"
        f"<plane><normal>0 0 1</normal><size>{sz} {sz}</size></plane></geometry>"
        f"<surface><friction><ode><mu>0.65</mu><mu2>0.65</mu2></ode></friction></surface>"
        f"</collision><visual name=\"v\"><geometry>"
        f"<plane><normal>0 0 1</normal><size>{sz} {sz}</size></plane></geometry>"
        f"\n{m}\n</visual></link></model>"
    )


def _boundary(half: float = 6.15, h: float = 3.5,
              tk: str | None = None) -> list[str]:
    hz, T, span = h / 2, 0.3, half * 2 + 0.3
    return [
        box("bnd_n",  0,     half,  hz, span,  T, h, tk, "Gazebo/DarkGrey"),
        box("bnd_s",  0,    -half,  hz, span,  T, h, tk, "Gazebo/DarkGrey"),
        box("bnd_e",  half,  0,     hz, T, span, h, tk, "Gazebo/DarkGrey"),
        box("bnd_w", -half,  0,     hz, T, span, h, tk, "Gazebo/DarkGrey"),
    ]


def _waypoints() -> list[str]:
    return [
        cyl("wp1", -2.80, -1.80, 0.5, 0.25, 1.0, None, "Gazebo/Red"),
        cyl("wp2", -4.50, -0.20, 0.5, 0.25, 1.0, None, "Gazebo/Red"),
        cyl("wp3", -3.20,  1.20, 0.5, 0.25, 1.0, None, "Gazebo/Red"),
    ]


def _hassan_pyramid_backdrop(px: float = 3.0, py: float = 3.5) -> list[str]:
    """Decorative pyramid stack (hassan_pyramid_eval.world) — flat Gazebo colors only."""
    layers = [
        (0.8, 4.2, 4.2, 1.6, "Gazebo/Orange"),
        (2.4, 2.9, 2.9, 1.6, "Gazebo/Orange"),
        (4.0, 1.7, 1.7, 1.6, "Gazebo/Orange"),
        (5.35, 0.6, 0.6, 0.7, "Gazebo/Yellow"),
    ]
    out: list[str] = []
    for i, (z, sx, sy, sz, fb) in enumerate(layers):
        out.append(box(f"pyramid_l{i + 1}", px, py, z, sx, sy, sz, "sand", fb))
    return out


def _hassan_obelisk(ox: float = 1.1, oy: float = 4.5) -> list[str]:
    return [
        box("obelisk", ox, oy, 4.0, 0.38, 0.38, 8.0, "stone", "Gazebo/DarkGrey"),
        box("obelisk_cap", ox, oy, 8.25, 0.52, 0.52, 0.4, "sand", "Gazebo/Yellow"),
    ]


def _save(world_name: str, filename: str, parts: list[str], theme: str) -> None:
    n   = sum(1 for p in parts if "<model" in p)
    sdf = (f'<?xml version="1.0"?>\n<!-- {theme} -->\n'
           f'<sdf version="1.6">\n  <world name="{world_name}">\n'
           + "\n".join(parts)
           + _GAZEBO_ROS_STATE
           + "\n\n  </world>\n</sdf>\n")
    path = os.path.join(_WD, filename)
    with open(path, "w", encoding="utf-8") as f:
        f.write(sdf)
    print(f"  ✅  {filename}  ({n} models)  →  {path}")


# ── World 1: Tour Hassan Rabat ONLY (not mixed Egypt/maze eval art) ─────────────
def _gen_rabat_world(*, fast: bool, filename: str, theme: str) -> None:
    pillars = _rabat_pillar_stumps()
    parts: list[str] = [
        _PHYSICS,
        _scene_hassan_sky(),
        _gui_camera_rabat(),
        *_hassan_arena_ground(fast=fast),
        *_hassan_arena_walls(fast=fast),
        _hassan_tower_simple(fast=fast),
    ]
    for x, y, z, r, h, idx in pillars:
        parts.append(_hassan_cyl(f"p_{idx}", x, y, z, r, h, fast=fast))
    parts += _waypoints()
    _save("hassan_rabat", filename, parts, theme)


def _gen_rabat_record_world() -> None:
    """Full textures, no procedural sky / world-embedded GUI (gzclient record path)."""
    pillars = _rabat_pillar_stumps()
    parts: list[str] = [
        _PHYSICS,
        _scene_hassan_record(),
        *_hassan_arena_ground(fast=False),
        *_hassan_arena_walls(fast=False),
        _hassan_tower_simple(fast=False),
    ]
    for x, y, z, r, h, idx in pillars:
        parts.append(_hassan_cyl(f"p_{idx}", x, y, z, r, h, fast=False))
    parts += _waypoints()
    _save(
        "hassan_rabat",
        "eval_rabat_record.world",
        parts,
        "Tour Hassan Rabat — textured record (plaza + pillars + tower)",
    )


def gen_rabat() -> None:
    print("\n── Tour Hassan Rabat (simple) ─────────────────────────────────────")
    _prepare_hassan_simple()
    _write_hassan_material()

    print("\n── eval_rabat_fast.world (grey, if Gazebo slow) ──")
    _gen_rabat_world(
        fast=True,
        filename="eval_rabat_fast.world",
        theme="Tour Hassan — grey fallback",
    )

    if not _hassan_assets_ok():
        print("  ⚠  Put hassan_pillar.png on ~/Desktop → re-run gen_rabat()")
        shutil.copy2(
            os.path.join(_WD, "eval_rabat_fast.world"),
            os.path.join(_WD, "eval_rabat.world"),
        )
        return

    print("\n── eval_rabat.world (stone pillars + plaza + tower) ──")
    _gen_rabat_world(
        fast=False,
        filename="eval_rabat.world",
        theme="Tour Hassan Rabat — plaza + pillar border, mosque",
    )
    print("\n── eval_rabat_record.world (textured, gzclient-safe) ──")
    _gen_rabat_record_world()


def gen_rabat_record() -> None:
    """Regenerate record-only world + materials."""
    _prepare_hassan_simple()
    _write_hassan_material()
    _gen_rabat_record_world()


# ── World 2: Egyptian Pyramid Desert ──────────────────────────────────────────
def gen_egypt() -> None:
    print("\n── Building eval_egypt.world ──────────────────────────────────────")
    parts: list[str] = [
        _PHYSICS,
        _scene("0.50 0.35 0.10 1.0", "0.62 0.46 0.18 1.0",
               "0.80 0.65 0.35 1.0", 10, 22),
        _lights("1.00 0.82 0.45 1.0", "-0.35 0.20 -0.90"),
        _ground("sand", "Gazebo/Orange", 26),
        *_boundary(6.15, 3.5, "sand"),
    ]

    # ── Main Pyramid: center (2.0, 2.0), 5 stacked layers ─────────────────
    PX, PY = 2.0, 2.0
    for i, (z, sx, sy, sz, fb) in enumerate([
        (0.65, 5.0, 5.0, 1.3, "Gazebo/Orange"),
        (1.95, 3.4, 3.4, 1.3, "Gazebo/Orange"),
        (3.25, 2.1, 2.1, 1.3, "Gazebo/Orange"),
        (4.55, 1.0, 1.0, 1.3, "Gazebo/Orange"),
        (5.55, 0.4, 0.4, 0.5, "Gazebo/Yellow"),
    ]):
        parts.append(box(f"pyr_l{i+1}", PX, PY, z, sx, sy, sz, "sand", fb))

    # ── Medium Pyramid: (3.5, -3.0), 4 layers ─────────────────────────────
    for i, (z, sx, sy, sz) in enumerate([
        (0.45, 3.0, 3.0, 0.9), (1.35, 1.9, 1.9, 0.9),
        (2.25, 0.9, 0.9, 0.9), (2.85, 0.3, 0.3, 0.3),
    ]):
        parts.append(box(f"pyr2_l{i+1}", 3.5, -3.0, z, sx, sy, sz, "sand", "Gazebo/Orange"))

    # ── Small Pyramid: (-0.5, 3.5), 3 layers ──────────────────────────────
    for i, (z, sx, sy, sz) in enumerate([
        (0.35, 2.5, 2.5, 0.7), (1.05, 1.5, 1.5, 0.7), (1.60, 0.4, 0.4, 0.3),
    ]):
        parts.append(box(f"pyr3_l{i+1}", -0.5, 3.5, z, sx, sy, sz, "sand", "Gazebo/Orange"))

    # ── 4 Obelisks with gold pyramidion caps ──────────────────────────────
    for i, (ox, oy, oh) in enumerate([
        (0.5, 4.5, 9.0), (4.5, 0.5, 8.0), (-0.5, -4.0, 7.0), (5.0, -2.0, 7.5),
    ]):
        parts.append(box(f"obelisk_{i}",  ox, oy, oh / 2,    0.36, 0.36, oh,   "sand", "Gazebo/DarkGrey"))
        parts.append(box(f"ob_cap_{i}",   ox, oy, oh + 0.25, 0.52, 0.52, 0.40, None,   "Gazebo/Yellow"))

    # ── Sphinx: elongated body + head block in navigation zone ───────────
    # Positioned so it does NOT block any waypoint path
    parts.append(box("sphinx_body", -0.5, -0.5, 0.45, 3.2, 1.2, 0.90, "sand", "Gazebo/Orange"))
    parts.append(box("sphinx_head", -0.5, -0.5, 1.15, 0.80, 0.80, 0.80, "sand", "Gazebo/Orange"))

    # ── Tomb/ruin blocks (low obstacles near navigation zone) ────────────
    parts.append(box("tomb1", -1.5,  1.2, 0.40, 1.2, 1.2, 0.8, "sand", "Gazebo/DarkGrey"))
    parts.append(box("tomb2", -1.0, -0.5, 0.30, 1.0, 0.8, 0.6, "sand", "Gazebo/DarkGrey"))

    parts += _waypoints()
    _save("egypt_desert", "eval_egypt.world",
          parts, "Egyptian Pyramid Desert — Zero-Shot Evaluation")


# ── World 3: Hedge Labyrinth (Maze Runner) ─────────────────────────────────────
def gen_maze() -> None:
    print("\n── Building eval_maze.world ───────────────────────────────────────")
    WH, WT = 3.0, 0.25
    HZ = WH / 2.0

    parts: list[str] = [
        _PHYSICS,
        _scene("0.18 0.28 0.08 1.0", "0.12 0.20 0.05 1.0",
               "0.22 0.32 0.10 1.0", 4, 12),
        _lights("0.70 0.88 0.40 1.0", "-0.20 0.10 -0.97"),
        _ground(None, "Gazebo/Green", 24),
        *_boundary(6.15, WH + 0.5, "hedge"),
    ]

    # ── Navigation path design ─────────────────────────────────────────────
    # spawn(-2,-2) → WP1(-2.8,-1.8)
    #   → gap at y=-1.0, x∈[-3.2, -2.2]  → middle zone
    #   → WP2(-4.5,-0.2)
    #   → gap at y=0.5,  x∈[-4.0, -2.5]  → upper zone
    #   → WP3(-3.2,1.2)
    walls = [
        # ── E-W divider 1: y=-1.0,  gap x=[-3.2, -2.2] ──────────────────
        ("h1l",  -4.10, -1.00, HZ, 1.80, WT, WH),   # x: -5   → -3.2
        ("h1r",  -1.10, -1.00, HZ, 2.20, WT, WH),   # x: -2.2 →  0
        # ── E-W divider 2: y=0.5,   gap x=[-4.0, -2.5] ──────────────────
        ("h2l",  -4.50,  0.50, HZ, 1.00, WT, WH),   # x: -5   → -4.0
        ("h2r",  -1.25,  0.50, HZ, 2.50, WT, WH),   # x: -2.5 →  0
        # ── E-W upper cap:  y=2.2,  gap x=[-4.0, -2.5] ──────────────────
        ("h3l",  -4.25,  2.20, HZ, 1.50, WT, WH),   # x: -5   → -3.5
        ("h3r",  -1.75,  2.20, HZ, 1.50, WT, WH),   # x: -2.5 → -1.0
        # ── N-S channel walls ─────────────────────────────────────────────
        ("v1",   -0.50, -0.25, HZ, WT, 1.50, WH),   # right of mid zone
        ("v2",   -2.00,  1.35, HZ, WT, 1.70, WH),   # upper-right
        # ── Dead ends (false corridors, all clear of waypoints) ───────────
        ("de1",  -4.50, -2.60, HZ, WT, 1.50, WH),   # far SW dead end
        ("de2",  -0.50, -2.80, HZ, 2.00, WT, WH),   # south-east stub
        ("de3",  -1.25,  1.40, HZ, WT, 0.90, WH),   # upper-right stub
    ]
    for n, cx, cy, cz, sx, sy, sz in walls:
        parts.append(box(f"w_{n}", cx, cy, cz, sx, sy, sz, "hedge", "Gazebo/Green"))

    parts += _waypoints()
    _save("maze_runner", "eval_maze.world",
          parts, "Maze Runner — Hedge Labyrinth (Zero-Shot)")


# ── Entry point ────────────────────────────────────────────────────────────────
def main() -> None:
    print("═" * 62)
    print("  generate_eval_worlds.py — Hassan · Egypt · Maze Suite")
    print("═" * 62)
    _sync_hassan_from_desktop()
    _download_textures()
    _write_material()
    gen_rabat()
    gen_egypt()
    gen_maze()
    print("\n" + "═" * 62)
    print("  Worlds written to:", _WD)
    print("  WP1(-2.8,-1.8)  WP2(-4.5,-0.2)  WP3(-3.2,1.2)")
    print("═" * 62)


if __name__ == "__main__":
    main()
