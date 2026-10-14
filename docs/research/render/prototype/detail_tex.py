"""Procedural, tileable ground detail textures (albedo + height -> normal) for desert soils.

Every texture wraps seamlessly: noise is FFT-filtered (periodic), Worley cells use a periodic
jittered grid, pebbles are stamped with wrap-around.
"""
import math

import cv2
import numpy as np
from PIL import Image


def srgb_to_lin(c):
    c = np.asarray(c, np.float32) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def lin_to_srgb(c):
    c = np.clip(c, 0, 1)
    return np.round(255 * np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1 / 2.4) - 0.055)).astype(np.uint8)


def spectral_noise(n, seed, beta=2.0, lo=1, hi=None):
    """Periodic noise with power spectrum ~ 1/f^beta between wavenumbers lo..hi (cycles per tile)."""
    rng = np.random.default_rng(seed)
    f = np.fft.fftfreq(n) * n
    fx, fy = np.meshgrid(f, f)
    r = np.hypot(fx, fy)
    amp = np.where(r > 0, r, 1) ** (-beta / 2)
    amp[r < lo] = 0
    if hi:
        amp *= np.exp(-(r / hi) ** 2)
    phase = rng.uniform(0, 2 * np.pi, (n, n))
    z = np.real(np.fft.ifft2(amp * np.exp(1j * phase)))
    return (z - z.mean()) / (z.std() + 1e-12)


def worley(n, cells, seed, jitter=1.0):
    """Periodic Worley noise on an n x n tile with cells x cells jittered points.
    Returns F1, F2 (in cell units) and the id of the nearest point."""
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
            idn = (gj % cells) * cells + (gi % cells)
            closer = d < f1
            f2 = np.where(closer, f1, np.minimum(f2, d))
            ids = np.where(closer, idn, ids)
            f1 = np.where(closer, d, f1)
    return f1, f2, ids


def stamp_pebbles(n, count, size_px, seed, palette, height, albedo, embed=0.45, elong=(1.0, 1.8)):
    """Stamp `count` elliptical pebbles (log-normal sizes around size_px, wrap-around) into
    height (max-composited) and albedo (linear RGB)."""
    rng = np.random.default_rng(seed)
    pal = np.array([srgb_to_lin(c) for c in palette])
    for _ in range(count):
        r = size_px * math.exp(rng.normal(0, 0.45))
        e = rng.uniform(*elong)
        a, b = r * math.sqrt(e), r / math.sqrt(e)
        th = rng.uniform(0, math.pi)
        cx, cy = rng.uniform(0, n, 2)
        R = int(math.ceil(a)) + 2
        xs = np.arange(int(cx) - R, int(cx) + R + 1)
        ys = np.arange(int(cy) - R, int(cy) + R + 1)
        X, Y = np.meshgrid(xs + 0.5 - cx, ys + 0.5 - cy)
        c, s = math.cos(th), math.sin(th)
        u, v = (c * X + s * Y) / a, (-s * X + c * Y) / b
        q = 1 - (u * u + v * v)
        inside = q > 0
        if not inside.any():
            continue
        # A rounded pebble, sunk `embed` of its height into the soil.
        hgt = np.where(inside, np.sqrt(np.clip(q, 0, 1)) * b * 0.8 - embed * b * 0.8, -1e9)
        rows, cols = ys % n, xs % n
        sub_h = height[np.ix_(rows, cols)]
        win = hgt > sub_h
        if not win.any():
            continue
        col = pal[rng.integers(len(pal))] * rng.uniform(0.75, 1.25)
        shade = (0.85 + 0.15 * np.clip(u * 0.5 - v * 0.5 + 0.5, 0, 1))[..., None]  # lithology mottling
        speck = 1 + 0.08 * rng.standard_normal(X.shape)[..., None]
        sub_a = albedo[np.ix_(rows, cols)]
        sub_a = np.where(win[..., None], np.clip(col * shade * speck, 0, 1), sub_a)
        height[np.ix_(rows, cols)] = np.where(win, hgt, sub_h)
        albedo[np.ix_(rows, cols)] = sub_a


def periodic_blur(img, sigma_px):
    n = img.shape[0]
    f = np.fft.fftfreq(n)
    fx, fy = np.meshgrid(f, f)
    g = np.exp(-2 * (np.pi * sigma_px) ** 2 * (fx * fx + fy * fy))
    return np.real(np.fft.ifft2(np.fft.fft2(img) * g))


def ambient_occlusion(height, radius_px, strength):
    """Cheap AO: how far a pixel sits below its blurred surroundings."""
    blur = periodic_blur(height, radius_px / 2)
    return np.clip(1 - strength * np.maximum(blur - height, 0), 0.35, 1)


def normal_from_height(height, px_m, strength=1.0):
    """Tangent-space normal map (OpenGL convention: +y up = north/top of image) from height [m]."""
    gx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) / (2 * px_m)
    gy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) / (2 * px_m)
    nrm = np.stack([-gx * strength, gy * strength, np.ones_like(height)], axis=-1)
    nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
    return np.round((nrm * 0.5 + 0.5) * 255).astype(np.uint8)


def save(prefix, albedo_lin, height, px_m, strength):
    Image.fromarray(lin_to_srgb(albedo_lin)).save(f"{prefix}_diffuse.png")
    Image.fromarray(normal_from_height(height, px_m, strength)).save(f"{prefix}_normal.png")
    h = (height - height.min()) / (np.ptp(height) + 1e-12)
    Image.fromarray(np.round(h * 65535).astype(np.uint16)).save(f"{prefix}_height.png")


# --- Soils -----------------------------------------------------------------------------

def desert_pavement(prefix, n=2048, size_m=3.0, seed=1):
    """Packed silty soil armoured with small varnished stones (desert pavement / gravel lag)."""
    px = size_m / n
    base = srgb_to_lin((182, 150, 118))
    soil = spectral_noise(n, seed, 1.6, lo=2)
    fine = spectral_noise(n, seed + 1, 0.6, lo=40)
    albedo = base * (1 + 0.10 * soil[..., None] + 0.06 * fine[..., None])
    height = 0.002 * soil + 0.0004 * fine
    palette = [(150, 120, 98), (110, 86, 72), (70, 55, 48), (190, 170, 150), (128, 118, 112), (160, 96, 70),
               (215, 205, 190), (95, 70, 58)]
    stamp_pebbles(n, 9000, 0.004 / px, seed + 2, palette, height, albedo)  # ~8 mm grains
    stamp_pebbles(n, 500, 0.012 / px, seed + 3, palette, height, albedo)  # ~2.5 cm stones
    stamp_pebbles(n, 40, 0.028 / px, seed + 4, palette, height, albedo)  # a few 5-6 cm cobbles
    albedo *= ambient_occlusion(height, 0.01 / px, 60)[..., None]
    save(prefix, albedo, height, px, 1.0)
    return albedo, height


def popcorn_clay(prefix, n=2048, size_m=2.0, seed=11):
    """Bentonitic mudstone weathered to 'popcorn' crust: 1-2 cm rounded aggregates in
    polygons bounded by desiccation cracks."""
    px = size_m / n
    base = srgb_to_lin((165, 160, 160))
    f1, f2, ids = worley(n, int(size_m / 0.16), seed)  # ~16 cm polygons
    crack = np.clip((f2 - f1) / 0.06, 0, 1)  # 0 at cracks
    p1, p2, pid = worley(n, int(size_m / 0.015), seed + 1, 0.9)  # 1.5 cm popcorn
    knob = np.clip(1 - p1 / 0.75, 0, 1) ** 0.7
    tone = np.random.default_rng(seed + 2).uniform(0.85, 1.15, int(size_m / 0.015) ** 2)[pid]
    big = spectral_noise(n, seed + 3, 1.8, lo=2)
    height = 0.006 * knob * crack + 0.004 * crack + 0.003 * big
    albedo = base * (tone * (1 + 0.08 * big))[..., None]
    albedo *= (0.55 + 0.45 * crack)[..., None]
    albedo *= ambient_occlusion(height, 0.006 / px, 80)[..., None]
    save(prefix, albedo, height, px, 1.0)
    return albedo, height


def wind_sand(prefix, n=2048, size_m=2.0, seed=21):
    """Fine wash/aeolian sand with wind ripples (~8 cm wavelength, asymmetric profile)."""
    px = size_m / n
    base = srgb_to_lin((214, 182, 140))
    u = (np.arange(n) + 0.5) / n
    X, Y = np.meshgrid(u, u)
    warp = 0.03 * spectral_noise(n, seed, 2.5, lo=1, hi=6)
    k = round(size_m / 0.08)  # integer cycles per tile keeps it tileable
    phase = (k * (X + 0.15 * np.sin(2 * np.pi * Y) / k * 0 + warp) + 2 * Y) % 1.0
    ripple = np.where(phase < 0.7, phase / 0.7, (1 - phase) / 0.3)  # gentle stoss, steep lee
    grain = spectral_noise(n, seed + 1, 0.2, lo=200)
    patch = spectral_noise(n, seed + 2, 2.0, lo=1)
    height = 0.006 * ripple * (0.6 + 0.4 * np.clip(patch + 0.5, 0, 1)) + 0.0002 * grain
    albedo = base * (1 + 0.05 * grain[..., None] + 0.05 * patch[..., None] + 0.05 * (ripple[..., None] - 0.5))
    rng = np.random.default_rng(seed + 3)
    stamp_pebbles(n, 600, 0.002 / px, seed + 4, [(120, 100, 90), (90, 70, 60), (200, 190, 175)], height, albedo,
                  embed=0.7)
    save(prefix, albedo, height, px, 1.0)
    return albedo, height


def sandstone_slab(prefix, n=2048, size_m=4.0, seed=31):
    """Cemented sandstone slab (slickrock / caprock): smooth, laminated, jointed, with a few loose chips."""
    px = size_m / n
    base = srgb_to_lin((205, 168, 128))
    u = (np.arange(n) + 0.5) / n
    X, Y = np.meshgrid(u, u)
    big = spectral_noise(n, seed, 2.2, lo=1)
    warp = spectral_noise(n, seed + 1, 2.8, lo=1, hi=8)
    lam = np.sin(2 * np.pi * (14 * Y + 2 * X + 1.5 * warp))  # laminae (integer cycles: tileable)
    f1, f2, _ = worley(n, 3, seed + 2, 0.8)  # joints ~1.3 m apart
    joint = np.clip((f2 - f1) / 0.025, 0, 1)
    pits = spectral_noise(n, seed + 3, 0.8, lo=60)
    height = 0.01 * big + 0.0008 * lam + 0.012 * joint - 0.0008 * np.clip(pits - 1.5, 0, None)
    albedo = base * (1 + 0.10 * big[..., None] + 0.04 * lam[..., None])
    albedo *= (0.5 + 0.5 * joint)[..., None]
    lichen = np.clip(spectral_noise(n, seed + 4, 1.2, lo=20) - 2.0, 0, 1)[..., None]
    albedo = albedo * (1 - 0.6 * lichen) + srgb_to_lin((60, 58, 50)) * 0.6 * lichen
    stamp_pebbles(n, 120, 0.015 / px, seed + 5, [(200, 165, 125), (180, 140, 105), (120, 90, 70)], height,
                  albedo, embed=0.2, elong=(1.2, 2.5))
    save(prefix, albedo, height, px, 1.0)
    return albedo, height


if __name__ == "__main__":
    import sys
    out = sys.argv[1]
    for fn, name in ((desert_pavement, "pavement"), (popcorn_clay, "popcorn"), (wind_sand, "sand"),
                     (sandstone_slab, "slab")):
        fn(f"{out}/{name}")
        print(name)


def desert_pavement_v2(prefix, n=2048, size_m=3.3, seed=5):
    """Pavement v2: patchy gravel lag over silty crust (pebble density varies across the tile, so the
    repeat is less obvious), narrow contact shadows instead of halos, faint desiccation cracks."""
    px = size_m / n
    base = srgb_to_lin((180, 152, 122))
    soil = spectral_noise(n, seed, 1.8, lo=1)
    fine = spectral_noise(n, seed + 1, 0.5, lo=60)
    albedo = base * (1 + 0.12 * soil[..., None] + 0.05 * fine[..., None])
    height = 0.003 * soil + 0.0003 * fine
    # faint crust cracks
    f1, f2, _ = worley(n, int(size_m / 0.25), seed + 7)
    crack = np.clip((f2 - f1) / 0.04, 0, 1)
    height -= 0.0015 * (1 - crack)
    albedo *= (0.8 + 0.2 * crack)[..., None]
    palette = [(150, 120, 98), (110, 86, 72), (72, 58, 50), (188, 168, 148), (128, 118, 112), (158, 98, 72),
               (210, 200, 186), (95, 72, 60)]
    patch = spectral_noise(n, seed + 8, 2.2, lo=1)  # where the lag is dense
    rng = np.random.default_rng(seed + 9)
    # Stamp pebbles only where the patch mask allows (rejection sampling through a wrapper).
    for count, size in ((7000, 0.0035), (900, 0.009), (120, 0.02), (12, 0.04)):
        h_tmp = height.copy()
        a_tmp = albedo.copy()
        stamp_pebbles(n, count, size / px, int(rng.integers(1e9)), palette, h_tmp, a_tmp, embed=0.5)
        keep = (patch + 0.6 * rng.standard_normal() * 0 > np.quantile(patch, 0.35))[..., None]
        sel = keep[..., 0] if size < 0.02 else np.ones_like(keep[..., 0])
        height = np.where(sel, h_tmp, height)
        albedo = np.where(sel[..., None], a_tmp, albedo)
    albedo *= ambient_occlusion(height, 0.003 / px, 25)[..., None]
    save(prefix, albedo, height, px, 1.0)
    return albedo, height


def cracked_silt(prefix, n=2048, size_m=7.7, seed=41):
    """Large-scale crust: desiccation polygons (0.3-0.6 m) with slightly curled plates and dust, for a
    second detail layer at a different repeat."""
    px = size_m / n
    base = srgb_to_lin((176, 160, 142))
    f1, f2, ids = worley(n, int(size_m / 0.45), seed)
    crack = np.clip((f2 - f1) / 0.05, 0, 1)
    curl = np.clip(f1 / 0.7, 0, 1) ** 2  # plates curl up at their edges
    tone = np.random.default_rng(seed + 1).uniform(0.9, 1.1, int(size_m / 0.45) ** 2)[ids]
    big = spectral_noise(n, seed + 2, 2.0, lo=1)
    dust = np.clip(spectral_noise(n, seed + 3, 1.6, lo=2) - 0.3, 0, 1)
    height = 0.004 * crack + 0.002 * curl * crack + 0.006 * big
    albedo = base * (tone * (1 + 0.08 * big))[..., None] * (0.6 + 0.4 * crack)[..., None]
    albedo = albedo * (1 - 0.5 * dust[..., None]) + srgb_to_lin((200, 184, 160)) * 0.5 * dust[..., None]
    save(prefix, albedo, height, px, 1.0)
    return albedo, height
