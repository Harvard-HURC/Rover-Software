"""Procedural textures: terrain layers and normal maps, ArUco tags, signs."""
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
