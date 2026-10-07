"""Procedural textures: terrain layers and normal maps, ground detail
textures (DETAILS), the flat normal map and the dust puff, ArUco tags, signs;
and the sRGB <-> linear light conversions colour work is done in."""
import functools
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# DejaVu ships with the pixi environment (matplotlib); fall back to Pillow's.
_FONT_DIRS = [Path(__file__).resolve().parents[2] / ".pixi/envs/default/fonts"] + sorted(
    Path(__file__).resolve().parents[2].glob(".pixi/envs/default/lib/python3*/site-packages/matplotlib/mpl-data/fonts/ttf"))


def font(size, bold=True):
    names = ("DejaVuSans-Bold.ttf", "DejaVuSans.ttf") if bold else ("DejaVuSans.ttf",)
    for directory in _FONT_DIRS:
        for name in names:
            if (directory / name).exists():
                return ImageFont.truetype(str(directory / name), size)
    return ImageFont.load_default(size=size)


def aruco_image(tag_id, px_per_cell=64, dictionary=cv2.aruco.DICT_4X4_50):
    """A 4x4 ArUco tag as URC prints it (rule 1.e.xii): 4x4 data cells, a
    one-cell black border and a one-cell white border, 8 x 8 cells in all."""
    d = cv2.aruco.getPredefinedDictionary(dictionary)
    marker = cv2.aruco.generateImageMarker(d, tag_id, 6 * px_per_cell)
    return cv2.copyMakeBorder(marker, *(px_per_cell,) * 4, cv2.BORDER_CONSTANT, value=255)


def write_aruco(path, tag_id, px_per_cell=64):
    Image.fromarray(aruco_image(tag_id, px_per_cell)).convert("RGB").save(path)


def tileable_noise(size, cells, seed):
    """Smooth noise in [0, 1] that tiles seamlessly."""
    rng = np.random.default_rng(seed)
    grid = rng.uniform(0, 1, (cells, cells)).astype(np.float32)
    big = cv2.resize(np.tile(grid, (3, 3)), (3 * size, 3 * size), interpolation=cv2.INTER_CUBIC)
    return big[size:2 * size, size:2 * size]


def _layers(size, seed):
    """Multi-scale tileable noise in [0, 1]."""
    # Little weight at the tile scale, so repeated tiles do not show as a checkerboard.
    total = sum(tileable_noise(size, cells, seed + k) * w for k, (cells, w) in enumerate(((4, 0.15), (16, 0.45), (64, 0.4))))
    return (total - total.min()) / (total.max() - total.min())


def terrain_texture(path, rgb, seed, size=1024, variation=0.25, pebbles=0.001):
    """Tileable ground texture: base colour with mottling and round, darker
    pebbles (`pebbles` per pixel, 1-2 px radius)."""
    n = _layers(size, seed)
    color = np.asarray(rgb, float)[None, None, :] * (1 - variation / 2 + variation * n[..., None])
    img = np.ascontiguousarray(np.clip(color, 0, 255).astype(np.uint8))
    rng = np.random.default_rng(seed + 100)
    base = np.asarray(rgb, float) * 0.78
    for _ in range(int(pebbles * size * size)):
        x, y = (int(v) for v in rng.integers(0, size, 2))
        r = int(rng.integers(1, 3))
        c = tuple(float(v) for v in np.clip(base * rng.uniform(0.75, 1.3), 0, 255))
        for dx in (-size, 0, size):  # wrap so the texture still tiles
            for dy in (-size, 0, size):
                cv2.circle(img, (x + dx, y + dy), r, c, -1, cv2.LINE_AA)
    Image.fromarray(img).save(path)


def normal_map(path, seed, size=1024, strength=2.0):
    """Tangent-space normal map from tileable bumps."""
    h = _layers(size, seed) * strength
    gx = (np.roll(h, -1, axis=1) - np.roll(h, 1, axis=1)) / 2
    gy = (np.roll(h, -1, axis=0) - np.roll(h, 1, axis=0)) / 2
    n = np.stack([-gx, gy, np.ones_like(h) / 8], axis=-1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    Image.fromarray(((n * 0.5 + 0.5) * 255).astype(np.uint8)).save(path)


def sign_image(path, lines, size=(1024, 640), bg=(250, 248, 240), fg=(15, 15, 15), border=(200, 40, 30)):
    """A sign with centred lines of large text and a coloured border."""
    img = Image.new("RGB", size, bg)
    draw = ImageDraw.Draw(img)
    w, h = size
    pad = h // 16
    draw.rectangle([pad // 2, pad // 2, w - pad // 2, h - pad // 2], outline=border, width=pad // 2)
    fsize = int((h - 3 * pad) / max(len(lines), 1) * 0.75)
    f = font(fsize)
    while fsize > 10 and max(draw.textlength(line, font=f) for line in lines) > w - 3 * pad:
        fsize = int(fsize * 0.9)
        f = font(fsize)
    total = len(lines) * fsize * 1.25
    y = (h - total) / 2
    for line in lines:
        x = (w - draw.textlength(line, font=f)) / 2
        draw.text((x, y), line, font=f, fill=fg)
        y += fsize * 1.25
    img.save(path)


def label_image(path, text, size=(256, 128), bg=(255, 255, 255), fg=(0, 0, 0)):
    """Small text label (e.g. a cross on a first-aid kit, an H on a pad)."""
    img = Image.new("RGB", size, bg)
    draw = ImageDraw.Draw(img)
    f = font(int(size[1] * 0.7))
    box = draw.textbbox((0, 0), text, font=f)
    draw.text(((size[0] - (box[2] - box[0])) / 2 - box[0], (size[1] - (box[3] - box[1])) / 2 - box[1]),
              text, font=f, fill=fg)
    img.save(path)


# --- Colour spaces ----------------------------------------------------------------------

def srgb_to_linear(rgb):
    """sRGB 0-255 -> linear light 0-1 (float32): averages and blends of
    colour are only right in linear light (the render prototype's Terra
    calibration: 0.1 DN off in linear light, 13 DN in sRGB, M)."""
    c = np.asarray(rgb, np.float32) / 255
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4).astype(np.float32)


def linear_to_srgb(lin):
    """Linear light 0-1 -> sRGB 0-255 (uint8), clipped."""
    c = np.clip(np.asarray(lin, np.float32), 0, 1)
    return np.round(255 * np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1 / 2.4) - 0.055)).astype(np.uint8)


def luminance(lin):
    """Relative luminance of linear sRGB (Rec. 709 weights), over the last axis."""
    return np.asarray(lin, np.float32) @ np.array([0.2126, 0.7152, 0.0722], np.float32)


SRGB_TO_XYZ = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]])
D65 = np.array([0.95047, 1.0, 1.08883])  # reference white (XYZ)
LAB_EPS = 6 / 29


def srgb_to_lab(rgb):
    """CIE L*a*b* (D65) of sRGB 0-255 colours (..., 3), float64."""
    c = np.asarray(rgb, float) / 255.0
    lin = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    t = lin @ SRGB_TO_XYZ.T / D65
    f = np.where(t > LAB_EPS ** 3, np.cbrt(t), t / (3 * LAB_EPS ** 2) + 4 / 29)
    return np.stack([116 * f[..., 1] - 16, 500 * (f[..., 0] - f[..., 1]), 200 * (f[..., 1] - f[..., 2])], axis=-1)


def lab_to_srgb(lab):
    """sRGB 0-255 (uint8, clipped to the gamut) of CIE L*a*b* (D65) colours (..., 3)."""
    lab = np.asarray(lab, float)
    fy = (lab[..., 0] + 16) / 116
    f = np.stack([fy + lab[..., 1] / 500, fy, fy - lab[..., 2] / 200], axis=-1)
    t = np.where(f > LAB_EPS, f ** 3, 3 * LAB_EPS ** 2 * (f - 4 / 29)) * D65
    lin = t @ np.linalg.inv(SRGB_TO_XYZ).T
    c = np.where(lin <= 0.0031308, 12.92 * lin, 1.055 * np.clip(lin, 0, None) ** (1 / 2.4) - 0.055)
    return np.clip(np.round(c * 255), 0, 255).astype(np.uint8)


# --- Ground detail textures --------------------------------------------------------------
# Tileable and procedural (no CC0 downloads, Q6), ported from the render
# prototype (sim/data/research/render/prototype/detail_tex.py): noise is
# FFT-filtered (periodic), Worley cells are a periodic jittered grid, pebbles
# are stamped with wrap-around. Each makes an albedo (linear light) and a
# height [m].

DETAIL_PIXELS = 2048  # tile size [px] (render prototype)
DETAIL_NORMAL_STRENGTH = 3.0  # Terra scales a detail layer's normals by its weight, so 3x (M: ~1.2 % of texels clip)


def _spectral_noise(n, seed, beta=2.0, lo=1, hi=None):
    """Periodic noise with power spectrum ~ 1/f^beta between wavenumbers lo
    and hi (cycles per tile), zero mean, unit standard deviation."""
    rng = np.random.default_rng(seed)
    f = np.fft.fftfreq(n) * n
    r = np.hypot(*np.meshgrid(f, f))
    amp = np.where(r > 0, r, 1) ** (-beta / 2)
    amp[r < lo] = 0
    if hi:
        amp *= np.exp(-(r / hi) ** 2)
    z = np.real(np.fft.ifft2(amp * np.exp(1j * rng.uniform(0, 2 * np.pi, (n, n)))))
    return ((z - z.mean()) / (z.std() + 1e-12)).astype(np.float32)


def _worley(n, cells, seed, jitter=1.0):
    """Periodic Worley noise on an n x n tile of cells x cells jittered
    points: distances to the nearest and second-nearest point (in cells) and
    the nearest point's id."""
    rng = np.random.default_rng(seed)
    pts = rng.uniform(0.5 - jitter / 2, 0.5 + jitter / 2, (cells, cells, 2))
    u = (np.arange(n) + 0.5) / n * cells
    X, Y = np.meshgrid(u, u)
    ci, cj = np.floor(X).astype(int), np.floor(Y).astype(int)
    f1 = np.full((n, n), 1e9, np.float32)
    f2 = np.full((n, n), 1e9, np.float32)
    ids = np.zeros((n, n), np.int64)
    for dj in (-1, 0, 1):
        for di in (-1, 0, 1):
            gi, gj = ci + di, cj + dj
            p = pts[gj % cells, gi % cells]
            d = np.hypot(gi + p[..., 0] - X, gj + p[..., 1] - Y).astype(np.float32)
            closer = d < f1
            f2 = np.where(closer, f1, np.minimum(f2, d))
            ids = np.where(closer, (gj % cells) * cells + gi % cells, ids)
            f1 = np.where(closer, d, f1)
    return f1, f2, ids


def _stamp_pebbles(n, count, size_px, seed, palette, height, albedo, embed=0.45, elong=(1.0, 1.8)):
    """Stamp `count` rounded elliptical pebbles (log-normal sizes around
    size_px, wrapping round the tile edges), sunk `embed` of their height,
    into height (where they stand above it) and albedo (linear RGB)."""
    rng = np.random.default_rng(seed)
    pal = srgb_to_linear(palette)
    for _ in range(count):
        r = size_px * math.exp(rng.normal(0, 0.45))
        e = rng.uniform(*elong)
        a, b = r * math.sqrt(e), r / math.sqrt(e)
        th = rng.uniform(0, math.pi)
        cx, cy = rng.uniform(0, n, 2)
        reach = int(math.ceil(a)) + 2
        xs = np.arange(int(cx) - reach, int(cx) + reach + 1)
        ys = np.arange(int(cy) - reach, int(cy) + reach + 1)
        X, Y = np.meshgrid(xs + 0.5 - cx, ys + 0.5 - cy)
        c, s = math.cos(th), math.sin(th)
        u, v = (c * X + s * Y) / a, (-s * X + c * Y) / b
        q = 1 - (u * u + v * v)
        hgt = np.where(q > 0, (np.sqrt(np.clip(q, 0, 1)) - embed) * b * 0.8, -1e9)
        rows, cols = np.ix_(ys % n, xs % n)
        win = hgt > height[rows, cols]
        if not win.any():
            continue
        colour = pal[rng.integers(len(pal))] * rng.uniform(0.75, 1.25)
        shade = (0.85 + 0.15 * np.clip(u * 0.5 - v * 0.5 + 0.5, 0, 1))[..., None]  # lithology mottling
        speck = 1 + 0.08 * rng.standard_normal(X.shape)[..., None]
        albedo[rows, cols] = np.where(win[..., None], np.clip(colour * shade * speck, 0, 1), albedo[rows, cols])
        height[rows, cols] = np.where(win, hgt, height[rows, cols])


def _ambient_occlusion(height, radius_px, strength):
    """Cheap occlusion: how far a texel sits below its blurred surroundings."""
    f = np.fft.fftfreq(height.shape[0])
    g = np.exp(-2 * (np.pi * radius_px / 2) ** 2 * (f[None, :] ** 2 + f[:, None] ** 2))
    blur = np.real(np.fft.ifft2(np.fft.fft2(height) * g))
    return np.clip(1 - strength * np.maximum(blur - height, 0), 0.35, 1).astype(np.float32)


def normal_from_height(height, px_m, strength=1.0):
    """Tangent-space normal map (uint8 RGB, +y towards the top of the image)
    of a periodic height field [m] with px_m metres per texel."""
    gx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) / (2 * px_m)
    gy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) / (2 * px_m)
    nrm = np.stack([-gx * strength, gy * strength, np.ones_like(height)], axis=-1)
    nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
    return np.round((nrm * 0.5 + 0.5) * 255).astype(np.uint8)


def gravel_lag(n, size_m, seed):
    """Desert pavement: a patchy gravel lag over a silty crust with faint
    desiccation cracks; the pebble density varies across the tile, so its
    repeat shows less (the prototype's pavement v2)."""
    px = size_m / n
    soil = _spectral_noise(n, seed, 1.8, lo=1)
    fine = _spectral_noise(n, seed + 1, 0.5, lo=60)
    albedo = srgb_to_linear((180, 152, 122)) * (1 + 0.12 * soil[..., None] + 0.05 * fine[..., None])
    height = 0.003 * soil + 0.0003 * fine
    f1, f2, _ = _worley(n, int(size_m / 0.25), seed + 7)
    crack = np.clip((f2 - f1) / 0.04, 0, 1)
    height -= 0.0015 * (1 - crack)
    albedo *= (0.8 + 0.2 * crack)[..., None]
    palette = [(150, 120, 98), (110, 86, 72), (72, 58, 50), (188, 168, 148), (128, 118, 112), (158, 98, 72),
               (210, 200, 186), (95, 72, 60)]
    dense = _spectral_noise(n, seed + 8, 2.2, lo=1) > -0.385  # the lag covers ~65 % of the tile
    rng = np.random.default_rng(seed + 9)
    for count, size in ((7000, 0.0035), (900, 0.009), (120, 0.02), (12, 0.04)):  # grains to cobbles [m]
        h, a = height.copy(), albedo.copy()
        _stamp_pebbles(n, count, size / px, int(rng.integers(1 << 30)), palette, h, a, embed=0.5)
        keep = dense if size < 0.02 else np.ones_like(dense)  # cobbles anywhere
        height = np.where(keep, h, height)
        albedo = np.where(keep[..., None], a, albedo)
    albedo *= _ambient_occlusion(height, 0.003 / px, 25)[..., None]
    return albedo, height


def cracked_silt(n, size_m, seed):
    """Silt crust broken into 0.3-0.6 m desiccation polygons with curled
    plate edges and drifts of dust: a second detail at a longer repeat."""
    cells = int(size_m / 0.45)
    f1, f2, ids = _worley(n, cells, seed)
    crack = np.clip((f2 - f1) / 0.05, 0, 1)
    curl = np.clip(f1 / 0.7, 0, 1) ** 2
    tone = np.random.default_rng(seed + 1).uniform(0.9, 1.1, cells * cells)[ids].astype(np.float32)
    big = _spectral_noise(n, seed + 2, 2.0, lo=1)
    dust = np.clip(_spectral_noise(n, seed + 3, 1.6, lo=2) - 0.3, 0, 1)[..., None]
    height = 0.004 * crack + 0.002 * curl * crack + 0.006 * big
    albedo = srgb_to_linear((176, 160, 142)) * (tone * (1 + 0.08 * big) * (0.6 + 0.4 * crack))[..., None]
    albedo = albedo * (1 - 0.5 * dust) + srgb_to_linear((200, 184, 160)) * 0.5 * dust
    return albedo, height


def slab_joints(n, size_m, seed):
    """Cemented sandstone (slickrock, caprock): smooth and laminated, cut by
    joints about 1.3 m apart, with a few loose chips and some lichen."""
    px = size_m / n
    u = (np.arange(n) + 0.5) / n
    X, Y = np.meshgrid(u, u)
    big = _spectral_noise(n, seed, 2.2, lo=1)
    warp = _spectral_noise(n, seed + 1, 2.8, lo=1, hi=8)
    laminae = np.sin(2 * np.pi * (14 * Y + 2 * X + 1.5 * warp))  # whole cycles per tile: tileable
    f1, f2, _ = _worley(n, max(2, round(size_m / 1.3)), seed + 2, 0.8)
    joint = np.clip((f2 - f1) / 0.025, 0, 1)
    pits = _spectral_noise(n, seed + 3, 0.8, lo=60)
    height = 0.01 * big + 0.0008 * laminae + 0.012 * joint - 0.0008 * np.clip(pits - 1.5, 0, None)
    albedo = srgb_to_linear((205, 168, 128)) * (1 + 0.10 * big + 0.04 * laminae)[..., None]
    albedo *= (0.5 + 0.5 * joint)[..., None]
    lichen = np.clip(_spectral_noise(n, seed + 4, 1.2, lo=20) - 2.0, 0, 1)[..., None]
    albedo = albedo * (1 - 0.6 * lichen) + srgb_to_linear((60, 58, 50)) * 0.6 * lichen
    _stamp_pebbles(n, 120, 0.015 / px, seed + 5, [(200, 165, 125), (180, 140, 105), (120, 90, 70)], height,
                   albedo, embed=0.2, elong=(1.2, 2.5))
    return albedo, height


def rippled_sand(n, size_m, seed):
    """Wind-rippled sand: crests about 8 cm apart (Sharp 1963, design spec
    5.4 [12]) with a gentle stoss and a steep lee, and a few dark grains."""
    px = size_m / n
    u = (np.arange(n) + 0.5) / n
    X, Y = np.meshgrid(u, u)
    warp = 0.03 * _spectral_noise(n, seed, 2.5, lo=1, hi=6)
    phase = (round(size_m / 0.08) * (X + warp) + 2 * Y) % 1.0  # whole cycles per tile: tileable
    ripple = np.where(phase < 0.7, phase / 0.7, (1 - phase) / 0.3)
    grain = _spectral_noise(n, seed + 1, 0.2, lo=200)
    patch = _spectral_noise(n, seed + 2, 2.0, lo=1)
    height = 0.006 * ripple * (0.6 + 0.4 * np.clip(patch + 0.5, 0, 1)) + 0.0002 * grain
    albedo = srgb_to_linear((214, 182, 140)) * (1 + 0.05 * grain + 0.05 * patch + 0.05 * (ripple - 0.5))[..., None]
    _stamp_pebbles(n, 600, 0.002 / px, seed + 4, [(120, 100, 90), (90, 70, 60), (200, 190, 175)], height, albedo,
                   embed=0.7)
    return albedo, height


def popcorn_crust(n, size_m, seed):
    """Bentonite weathered to 'popcorn' crust: 1-2 cm rounded aggregates in
    polygons bounded by desiccation cracks (crust relief ~3.8 cm, T: Mancos
    Shale [8] of the design spec)."""
    px = size_m / n
    f1, f2, _ = _worley(n, int(size_m / 0.16), seed)  # ~16 cm polygons
    crack = np.clip((f2 - f1) / 0.06, 0, 1)
    cells = int(size_m / 0.015)  # ~1.5 cm aggregates
    p1, _, pid = _worley(n, cells, seed + 1, 0.9)
    knob = np.clip(1 - p1 / 0.75, 0, 1) ** 0.7
    tone = np.random.default_rng(seed + 2).uniform(0.85, 1.15, cells * cells)[pid].astype(np.float32)
    big = _spectral_noise(n, seed + 3, 1.8, lo=2)
    height = 0.006 * knob * crack + 0.004 * crack + 0.003 * big
    albedo = srgb_to_linear((165, 160, 160)) * (tone * (1 + 0.08 * big) * (0.55 + 0.45 * crack))[..., None]
    albedo *= _ambient_occlusion(height, 0.006 / px, 80)[..., None]
    return albedo, height


@dataclass(frozen=True)
class Detail:
    """A shared ground detail texture: its generator (n, size_m, seed) ->
    (albedo, height), the metres one tile covers (its SDF <size>) and seed."""
    make: object
    tile_m: float
    seed: int


DETAILS = {
    "gravel_lag": Detail(gravel_lag, 3.3, 5),  # tile (M: render prototype, at Terra weight ~0.35)
    "cracked_silt": Detail(cracked_silt, 7.7, 41),  # (M: render prototype, at weight ~0.15)
    "slab_joints": Detail(slab_joints, 5.1, 31),  # (M: render prototype, above cap heights)
    "rippled_sand": Detail(rippled_sand, 2.0, 21),  # (A)
    "popcorn_crust": Detail(popcorn_crust, 2.0, 11),  # (A)
}


@functools.lru_cache(maxsize=2)
def _detail_fields(key):
    d = DETAILS[key]
    albedo, height = d.make(DETAIL_PIXELS, d.tile_m, d.seed)
    return np.clip(albedo, 0, 1), height


def detail_texture(path, key, part, mean=None):
    """DETAILS[key]'s diffuse map (part "diffuse", sRGB) or normal map (part
    "normal", DETAIL_NORMAL_STRENGTH x its relief) as a PNG. mean: a linear
    RGB the diffuse map is rescaled to, per channel: Terra's compensation
    (appearance.terra_layers) clips far less when the details average the
    colour map's colour (the render prototype's modulation)."""
    albedo, height = _detail_fields(key)
    if part == "diffuse":
        if mean is not None:
            albedo = albedo * (np.asarray(mean, np.float32) / albedo.reshape(-1, 3).mean(axis=0))
        image = linear_to_srgb(albedo)
    elif part == "normal":
        image = normal_from_height(height, DETAILS[key].tile_m / DETAIL_PIXELS, DETAIL_NORMAL_STRENGTH)
    else:
        raise ValueError(f"part {part!r}: diffuse or normal")
    Image.fromarray(image).save(path, format="PNG")


def flat_normal(path, size=16):
    """A flat tangent-space normal map: Terra's colour-map layer needs a
    <normal> (SDF requires one per texture), and it must change nothing."""
    Image.fromarray(np.full((size, size, 3), (128, 128, 255), np.uint8)).save(path, format="PNG")


def dust_puff(path, rgb, opacity, size=128, seed=7):
    """The dust particle sprite (RGBA): a lumpy, soft-edged puff in `rgb`
    [0-1 sRGB], its alpha (at most 0.72 at the core) times `opacity`: the
    whole of the particle's colour and opacity, as gz-rendering 8 applies
    no colour range (gen_model.DriveParams)."""
    c = (np.arange(size) + 0.5 - size / 2) / (size / 2)
    r = np.hypot(*np.meshgrid(c, c))
    lumps = 0.6 * tileable_noise(size, 4, seed) + 0.4 * tileable_noise(size, 8, seed + 1)
    alpha = np.clip(1.6 * lumps * np.clip(1 - r, 0, 1) ** 1.2, 0, 1) * opacity
    rgba = np.zeros((size, size, 4), np.uint8)
    rgba[..., :3] = np.round(np.asarray(rgb) * 255)
    rgba[..., 3] = np.round(alpha * 255)
    Image.fromarray(rgba, "RGBA").save(path, format="PNG")
