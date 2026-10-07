"""How the ground looks (design spec 5.7): a world's colour map, Terra's
layer 0, and the shared detail layers over it.

- Synthetic worlds: colour_map() bakes the ground raster (ground.png, one
  terrains type per heightmap sample) into colour: each type's palette with
  its measured spread, badland strata, a mild slope darkening, cavity
  shading and far shrubs as dark dots.
- Real ground (Autonomy): ortho_colour_map() drapes NAIP: resampled onto the
  world (resample_raster), de-shaded so slopes are not shaded twice under
  the sim's own sun (deshade), cast shadows and the spots under 3D shrubs
  inpainted, saturation and contrast boosted in linear light (Boost).
  detect_shrubs() finds NAIP's shrubs: dark spots that pass a local NDVI
  anomaly test (absolute NDVI is negative almost everywhere, M).
- terra_layers() lays up to three shared detail textures (textures.DETAILS)
  over the colour map with constant partial weights, and compensates the
  colour map per texel in linear light, so the render shows the colour map
  plus zero-mean detail (M: to 0.1 DN).

Texels: an n x n colour map covers the terrain square edge to edge, texel
(r, c) the square x0 + [c, c + 1] S/n, y0 - [r, r + 1] S/n from the north-west
corner (x0, y0): Terra's layout for a layer whose <size> is the terrain's.
Heightmap samples sit on the square's edges (Heightfield.grid()), so texel
centres fall between samples. All colour arithmetic is in linear light
(textures.srgb_to_linear): averages and blends are only right there.
"""
import datetime
import math
from dataclasses import dataclass, field

import cv2
import numpy as np
from PIL import Image

from . import dem, geo, lighting, sdf, terrain, textures

# --- Colour-map recipe (design spec 5.7) --------------------------------------------------
WARP_M = 1.2  # [m] how far type edges wander from the ground raster's samples (A)
WARP_FEATURE_M = 6.0  # [m] the wander's wavelength (A)
EDGE_SOFTNESS_M = 0.3  # [m] Gaussian sigma of the blend across type edges and strata (A)
SPREAD_FEATURE_M = 3.0  # [m] the shortest wavelength of a type's colour mottling (A)
DEFAULT_SPREAD = 0.08  # relative p10-p90 half-range of a palette that has no measured one (A)
P90_SIGMA = 1.2816  # p90 of a standard normal: palette p10/p90 lie +-1.28 sigma from the median
SLOPE_DARKENING = 0.04  # at 45 deg (M: NAIP is 4-9 % darker at 30-45 deg, mostly lighting; kept 2-4 %)
CAVITY_SIGMA_M = 3.0  # [m] the surroundings a hollow is measured against (A)
CAVITY_DEPTH_M = 0.5  # [m] a hollow this deep takes the full darkening (A)
CAVITY_DARKENING = 0.06  # (A)
DOT_RGB = (127, 105, 103)  # far shrubs from above: NAIP dark spots' p5 (M, colour_stats.json route_area)

# --- Orthophoto (design spec 5.7) --------------------------------------------------------
DARK_FACTOR = 0.88  # a dark spot is > 12 % darker than its surroundings' median (M: colour_stats.json)
NEIGHBOURHOOD_M = 15.0  # [m] those surroundings (M: colour_stats.json)
NDVI_ANOMALY = 0.01  # shrub dark spots exceed it: 53-68 % of their pixels vs 2 % of all (M: critique P)
SHRUB_AREA_M2 = (0.25, 13.0)  # dark-spot sizes taken for shrubs: one NAIP pixel to a 4 m crown (A)
SHRUB_MAX_SLOPE = 25.0  # [deg] dark spots on steeper ground are rock, varnish or shadow (A)
FIT_STRIDE = 3  # the sun fit uses every 3rd texel each way (speed; the fit is over millions)
SUN_FIT_SLOPE = 10.0  # [deg] the sun's azimuth is fitted on steeper ground, where shading outweighs albedo
# (M: fitted azimuth over five windows of the route area 100-120 deg on all ground, 107-119 deg above 10 deg)
SHADOW_COS = 0.05  # cos(illumination) at or below this: self-shadowed (A)
DESHADE_LIMITS = (0.5, 2.0)  # the C-correction factor is clipped to this (A: steep, nearly unlit ground)
INPAINT_RADIUS_PX = 3
NAIP2024_ACQUIRED = datetime.date(2024, 7, 6)  # sim/data/imagery/route_area_naip2024.json (quarter-quad ..._20240706)


@dataclass(frozen=True)
class Boost:
    """Saturation and contrast in linear light: each texel's chroma about its
    own luminance x saturation, then every channel about a fixed luminance
    pivot x contrast (fixed, so a crop boosts like the whole)."""
    saturation: float
    contrast: float
    pivot: float = 0.60  # luminance of NAIP 2024's route-area median #dfc8b0 (M: colour_stats.json)

    def apply(self, lin):
        y = textures.luminance(lin)[..., None]
        return self.pivot + self.contrast * (y + self.saturation * (lin - y) - self.pivot)


# NAIP is hazy and pale. The render prototype boosted the NAIP 2021 mosaic x1.25 / x1.1 (M).
# NAIP 2024 is brighter and more saturated over the route area (luminance 0.60 vs 0.49, chroma /
# luminance 0.34 vs 0.20, M, rev2 of this module); without ground photos there is no target to
# re-tune against (Q13), so it keeps the prototype's factors (A).
NAIP2021_BOOST = Boost(1.25, 1.1)
NAIP2024_BOOST = Boost(1.25, 1.1)


# --- Texels ------------------------------------------------------------------------------

def texel_centres(size, n, center=(0.0, 0.0)):
    """World x of each texel column and y of each texel row (row 0 north)."""
    c = (np.arange(n) + 0.5) * size / n - size / 2
    return center[0] + c, center[1] - c


def at_texels(hf, n, values=None):
    """A field on hf's sample grid (default its heights) at the centres of an
    n x n colour map over hf's square (bilinear, float32)."""
    s = ((np.arange(n) + 0.5) / n * (hf.n - 1)).astype(np.float32)
    X, Y = np.meshgrid(s, s)
    v = hf.z if values is None else values
    return cv2.remap(np.asarray(v, np.float32), X, Y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def disc_mask(points, size, n, center=(0.0, 0.0), grow=1.0):
    """Texels under discs [(x, y, diameter)] (each grown by `grow` and at
    least a texel across): the spots to inpaint under 3D shrubs."""
    mask = np.zeros((n, n), np.uint8)
    texel = size / n
    shift = 4  # cv2 fixed-point bits: sub-texel centres and radii
    for x, y, d in points:
        col = (x - center[0] + size / 2) / texel - 0.5
        row = (center[1] + size / 2 - y) / texel - 0.5
        radius = max(0.5, d * grow / 2 / texel)
        cv2.circle(mask, (round(col * 16), round(row * 16)), round(radius * 16), 1, -1, cv2.LINE_8, shift)
    return mask.astype(bool)


# --- Colour --------------------------------------------------------------------------------

def srgb_to_lab(rgb):
    """CIE L*a*b* (D65) of sRGB 0-255 colours (..., 3)."""
    lin = textures.srgb_to_linear(rgb).astype(np.float64)
    M = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]])
    xyz = lin @ M.T / np.array([0.95047, 1.0, 1.08883])
    f = np.where(xyz > (6 / 29) ** 3, np.cbrt(xyz), xyz / (3 * (6 / 29) ** 2) + 4 / 29)
    return np.stack([116 * f[..., 1] - 16, 500 * (f[..., 0] - f[..., 1]), 200 * (f[..., 1] - f[..., 2])], axis=-1)


def delta_e(a, b):
    """CIE76 colour difference between sRGB 0-255 colours."""
    return np.linalg.norm(srgb_to_lab(a) - srgb_to_lab(b), axis=-1)


def palette_colour(munsell_rgb, naip_rgb, saturation=1.25):
    """A ground type's palette colour by the rule of design spec 5.7: hue and
    saturation from the soil survey's Munsell colour (as sRGB), lightness
    from NAIP's median, saturated like the orthophoto. Returns sRGB."""
    hue = textures.srgb_to_linear(munsell_rgb)
    lin = hue * textures.luminance(textures.srgb_to_linear(naip_rgb)) / textures.luminance(hue)
    y = textures.luminance(lin)
    return tuple(int(v) for v in textures.linear_to_srgb(y + saturation * (lin - y)))


@dataclass(frozen=True)
class Strata:
    """Badland banding (design spec 5.7): ground of the banded types takes
    the colour of the layer it cuts. Layers of random thickness are stacked
    by elevation on a gently dipping plane, their boundaries wandering a
    little (elevation explains 33-70 % of redness on banded mounds,
    strata_colour_ramps.json, M)."""
    # maroon median, p90 and p10, and the white band [sRGB] (M: NAIP, design spec 5.7 table)
    bands: tuple = ((181, 157, 149), (195, 170, 161), (160, 138, 135), (215, 212, 206))
    weights: tuple = (0.4, 0.2, 0.2, 0.2)  # how often each band occurs (A)
    thickness_m: tuple = (0.5, 3.0)  # (A)
    dip_deg: float = 2.0  # (M: fitted dips up to 3.4 deg)
    dip_azimuth_deg: float = 30.0  # (A: the fits did not settle a direction)
    wobble_m: float = 0.3  # (A)


def colour_map(hf, raster, types, rng, n=4096, strata=None, dots=(), dot_rgb=DOT_RGB):
    """The colour map (sRGB uint8, n x n x 3) of a synthetic world's ground.

    hf: the visual terrain (its heightmap) in world coordinates; raster: the
    ground raster on hf's sample grid (uint8 type indices, row 0 north);
    types: the TerrainType of each index (a sequence or a dict); rng: a
    numpy Generator (the noise seeds); strata: {type key: Strata} for the
    banded types; dots: [(x, y, diameter)] shrubs to draw from above (the
    far ones, which have no 3D mesh).

    Each type's colour is its palette median, mottled by noise over its
    p10-p90 range (DEFAULT_SPREAD where none is measured); type edges wander
    WARP_M from the raster's samples, so they are not grid lines, and blend
    over EDGE_SOFTNESS_M. Palettes are final colours: the saturation boost of
    design spec 5.7 belongs in them (palette_colour)."""
    raster = np.asarray(raster)
    assert raster.shape == hf.z.shape, "the ground raster is on the heightmap's grid"
    kinds = dict(enumerate(types)) if not isinstance(types, dict) else dict(types)
    count = max(kinds) + 1
    base = np.zeros((count, 3), np.float32)
    spread = np.zeros((count, 3), np.float32)
    for index, kind in kinds.items():
        palette = kind.appearance.palette
        base[index] = textures.srgb_to_linear(palette.base)
        if palette.p10 is not None and palette.p90 is not None:
            p10, p90 = textures.srgb_to_linear(palette.p10), textures.srgb_to_linear(palette.p90)
            spread[index] = (p90 - p10) / (2 * P90_SIGMA)
        else:
            spread[index] = base[index] * DEFAULT_SPREAD / P90_SIGMA
    missing = set(np.unique(raster)) - set(kinds)
    if missing:
        raise ValueError(f"ground raster indices {sorted(missing)} have no type")

    def noise(feature, octaves):
        z = terrain.fbm(n, hf.size, feature, int(rng.integers(1 << 30)), octaves).astype(np.float32)
        return (z - z.mean()) / (z.std() + 1e-9)

    xs, ys = texel_centres(hf.size, n, hf.center)
    west, north = hf.center[0] - hf.size / 2, hf.center[1] + hf.size / 2
    col = np.clip(np.rint((xs[None, :] + WARP_M * noise(WARP_FEATURE_M, 3) - west) / hf.res), 0, hf.n - 1)
    row = np.clip(np.rint((north - ys[:, None] + WARP_M * noise(WARP_FEATURE_M, 3)) / hf.res), 0, hf.n - 1)
    t = raster[row.astype(np.intp), col.astype(np.intp)]
    del row, col
    mottle = noise(SPREAD_FEATURE_M * 16, 5)  # 5 octaves: 48 m down to 3 m
    lin = base[t] + mottle[..., None] * spread[t]
    for key, recipe in (strata or {}).items():
        indices = [i for i, kind in kinds.items() if kind.key == key]
        if not indices:
            continue
        mask = np.isin(t, indices)
        band = _strata_band(recipe, at_texels(hf, n), xs, ys, mask, rng, noise)
        colours = textures.srgb_to_linear(recipe.bands)
        lin[mask] = colours[band] + 0.5 * mottle[mask][:, None] * spread[t[mask]]
    lin = cv2.GaussianBlur(lin, (0, 0), EDGE_SOFTNESS_M / (hf.size / n))  # soft type edges
    slope = at_texels(hf, n, hf.slope_map())
    lin *= (1 - SLOPE_DARKENING * terrain.smoothstep(0.0, 45.0, slope))[..., None]
    sigma = CAVITY_SIGMA_M / hf.res
    hollow = cv2.GaussianBlur(hf.z.astype(np.float32), (0, 0), sigma) - hf.z.astype(np.float32)
    lin *= (1 - CAVITY_DARKENING * np.clip(at_texels(hf, n, hollow) / CAVITY_DEPTH_M, 0, 1))[..., None]
    if len(dots):
        cover = _dot_cover(dots, hf.size, n, hf.center)
        lin = lin * (1 - cover[..., None]) + textures.srgb_to_linear(dot_rgb) * cover[..., None]
    return textures.linear_to_srgb(lin)


def _strata_band(recipe, heights, xs, ys, mask, rng, noise):
    """Band index of each masked texel: its elevation on the dipping plane
    (plus a wobble) located in a random stack of layers."""
    dip = math.tan(math.radians(recipe.dip_deg))
    az = math.radians(recipe.dip_azimuth_deg)
    plane = dip * (math.sin(az) * xs[None, :] + math.cos(az) * ys[:, None])
    e = (heights + plane + recipe.wobble_m * noise(8.0, 3))[mask]
    lo, hi = float(e.min()), float(e.max())
    layers = int(math.ceil((hi - lo) / recipe.thickness_m[0])) + 2
    edges = lo - recipe.thickness_m[1] + np.cumsum(rng.uniform(*recipe.thickness_m, layers))
    weights = np.asarray(recipe.weights, float)
    choice = [int(rng.choice(len(weights), p=weights / weights.sum()))]
    for _ in range(layers):  # each layer differs from the one below it
        w = weights.copy()
        w[choice[-1]] = 0
        choice.append(int(rng.choice(len(w), p=w / w.sum())))
    return np.array(choice)[np.searchsorted(edges, e)]


def _dot_cover(dots, size, n, center):
    """Anti-aliased coverage [0, 1] of discs [(x, y, diameter)] on the texels."""
    cover = np.zeros((n, n), np.uint8)
    texel = size / n
    for x, y, d in dots:
        col = (x - center[0] + size / 2) / texel - 0.5
        row = (center[1] + size / 2 - y) / texel - 0.5
        cv2.circle(cover, (round(col * 16), round(row * 16)), max(1, round(d / 2 / texel * 16)), 255, -1,
                   cv2.LINE_AA, 4)
    return cover.astype(np.float32) / 255


# --- Orthophoto ----------------------------------------------------------------------------

def lonlat_fit(origin: geo.Origin, size, center=(0.0, 0.0)):
    """(x, y) world -> (lat, lon) over a square: a quadratic fit of
    geo.enu_to_wgs84, within 1e-9 deg over 2 km (an affine map is 0.1 m off
    at the corners: a degree of longitude shortens northwards); the same fit
    as dem.to_heightfield's."""
    def terms(x, y):
        x, y = (np.asarray(x, float) - center[0]) / size, (np.asarray(y, float) - center[1]) / size
        return np.stack([np.ones_like(x), x, y, x * x, x * y, y * y], axis=-1)

    def exact(ticks):
        xy = np.array([(center[0] + a * size, center[1] + b * size) for a in ticks for b in ticks])
        return terms(xy[:, 0], xy[:, 1]), np.array([geo.enu_to_wgs84(origin, x, y)[:2] for x, y in xy])

    coef, *_ = np.linalg.lstsq(*exact((-0.5, 0.0, 0.5)), rcond=None)
    A, check = exact((-0.25, 0.25))
    residual = np.abs(A @ coef - check).max()
    if residual > 1e-9:
        raise ValueError(f"lat/lon fit is off by {residual:.2e} deg over {size} m")
    return lambda x, y: np.moveaxis(terms(x, y) @ coef, -1, 0)


def resample_raster(path, origin: geo.Origin, size, n, center=(0.0, 0.0)):
    """A GeoTIFF (dem.read_raster: DEM or NAIP, every band) at the texel
    centres of an n x n map over the world square: float32 (B, n, n),
    bilinear, area-averaged first where a texel spans several pixels."""
    bands, (lon0, dlon, _, lat0, _, minus_dlat) = dem.read_raster(path)
    xs, ys = texel_centres(size, n, center)
    X, Y = np.meshgrid(xs, ys)
    lat, lon = lonlat_fit(origin, size, center)(X, Y)
    col = ((lon - lon0) / dlon - 0.5).astype(np.float32)
    row = ((lat - lat0) / minus_dlat - 0.5).astype(np.float32)
    if col.min() < 0 or row.min() < 0 or col.max() > bands.shape[2] - 1 or row.max() > bands.shape[1] - 1:
        raise ValueError(f"{path} does not cover a {size} m square around world {center}")
    pixel_m = abs(minus_dlat) * 111_000  # [m] north-south, roughly
    blur = 0.5 * size / n / pixel_m  # [px] a texel's half-width
    out = []
    for band in bands:
        if blur > 0.75:
            band = cv2.GaussianBlur(band, (0, 0), blur)
        out.append(cv2.remap(band, col, row, cv2.INTER_LINEAR))
    return np.stack(out)


@dataclass(frozen=True)
class FittedSun:
    """The sun NAIP was shot under, fitted to its shading (deshade)."""
    elevation_deg: float
    azimuth_deg: float
    k: dict = field(default_factory=dict)  # C-correction constant per ground unit
    r2: float = 0.0  # share of the luminance variance on fitting texels that the shading explains

    @property
    def toward(self):
        el, az = math.radians(self.elevation_deg), math.radians(self.azimuth_deg)
        return np.array([math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el)])


def surface_normals(heights, texel_m):
    """Unit normals (n, n, 3) of a height grid (row 0 north) [m]."""
    gy, gx = np.gradient(np.asarray(heights, np.float32), texel_m)
    normal = np.stack([-gx, gy, np.ones_like(gx)], axis=-1)  # rows run south: dz/dnorth = -gy
    return normal / np.linalg.norm(normal, axis=-1, keepdims=True)


def illumination(normals, sun):
    """cos(angle between each normal and the way to the sun)."""
    return normals @ sun.toward.astype(np.float32)


def cast_shadows(heights, texel_m, sun):
    """Texels in another texel's shadow under `sun`, by ray-marching the
    height grid one texel at a time towards the sun until no rise left in
    the grid could shade anything."""
    h = np.asarray(heights, np.float32)
    n = h.shape[0]
    rise = math.tan(math.radians(sun.elevation_deg))
    reach = int(math.ceil((float(h.max()) - float(h.min())) / max(rise, 1e-3) / texel_m))
    az = math.radians(sun.azimuth_deg)
    dc, dr = math.sin(az), -math.cos(az)  # one texel towards the sun, in columns and rows
    C, R = np.meshgrid(np.arange(n, dtype=np.float32), np.arange(n, dtype=np.float32))
    shadow = np.zeros((n, n), bool)
    for k in range(1, min(reach, 2 * n) + 1):
        ahead = cv2.remap(h, C + k * dc, R + k * dr, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        shadow |= ahead > h + k * texel_m * rise
    return shadow


def deshade(naip, dem_hf, units=None, acquired=NAIP2024_ACQUIRED, site=lighting.MISSION_SITE):
    """NAIP with its shading removed (design spec 5.7, D13): returns
    (linear RGB (n, n, 3), shadow mask, FittedSun).

    naip: (bands, n, n) digital numbers (R, G, B[, NIR]) on the texels of
    dem_hf's square; dem_hf: the real DEM there (world frame); units: ground
    units (int, n x n: SSURGO map units) or None for one unit; acquired: the
    date the image was shot, at site (lat, lon).

    Luminance per unit is fitted as rho (k + cos i), i the illumination angle
    under a fitted sun on the DEM. The sun's azimuth comes from least squares
    of luminance (each unit scaled to its median) on the surface normal's
    horizontal components over ground steeper than SUN_FIT_SLOPE; its
    elevation from the sun's path on the date at
    that azimuth (the vertical component is nearly collinear with the
    intercept, and steep badland slopes are paler: fitted, it comes out
    negative, M). Then k per unit, and each texel is divided by
    (k + cos i) / (k + sin(elevation)), the C-correction [Teillet et al.
    1982]: as lit as flat ground of its unit. Dark spots (shrubs, rocks) and
    shadows stay out of the fits. The mask holds the texels the correction
    cannot recover: cast shadows (ray-marched) and self-shadowed slopes;
    inpaint them."""
    n = naip.shape[1]
    texel = dem_hf.size / n
    lin = np.moveaxis(textures.srgb_to_linear(naip[:3]), 0, -1)
    lum = textures.luminance(lin)
    heights = at_texels(dem_hf, n)
    normals = surface_normals(heights, texel)
    units = np.zeros((n, n), np.int32) if units is None else np.asarray(units)
    keys = [int(u) for u in np.unique(units)]
    usable = ~_dark_spots(lum, texel)
    scaled = np.zeros_like(lum)
    for u in keys:
        inside = units == u
        clean = inside & usable
        scaled[inside] = lum[inside] / np.median(lum[clean] if clean.any() else lum[inside])
    sloped = usable & (normals[..., 2] < math.cos(math.radians(SUN_FIT_SLOPE)))
    sun = _fit_sun(scaled, normals, sloped, acquired, site)
    shadow = cast_shadows(heights, texel, sun) | (illumination(normals, sun) <= SHADOW_COS)
    sun = _fit_sun(scaled, normals, sloped & ~shadow, acquired, site)  # again, without the shadows
    shadow = cast_shadows(heights, texel, sun) | (illumination(normals, sun) <= SHADOW_COS)
    cos_i = illumination(normals, sun)
    fit = usable & ~shadow
    k, explained, total = {}, 0.0, 0.0
    factor = np.ones_like(lum)
    flat = math.sin(math.radians(sun.elevation_deg))
    for u in keys:
        inside = units == u
        sel = inside & fit
        x, y = cos_i[sel].astype(np.float64), lum[sel].astype(np.float64)  # float32 sums lose the slope (M)
        slope = float(np.mean((x - x.mean()) * (y - y.mean())) / max(x.var(), 1e-12))
        intercept = float(y.mean() - slope * x.mean())
        k[u] = max(intercept / slope, 0.05) if slope > 0 else 10.0  # no shading found: barely correct
        explained += slope * slope * x.var() * len(x)
        total += y.var() * len(y)
        factor[inside] = (k[u] + flat) / np.maximum(k[u] + cos_i[inside], 1e-3)
    sun = FittedSun(sun.elevation_deg, sun.azimuth_deg, k, explained / max(total, 1e-12))
    return lin * np.clip(factor, *DESHADE_LIMITS)[..., None], shadow, sun


def _fit_sun(scaled, normals, select, acquired, site):
    """The sun whose azimuth best explains scaled luminance as
    a + c_x n_x + c_y n_y (least squares; (c_x, c_y) points at the sun) and
    whose elevation is the sun's at that azimuth on the date `acquired`."""
    sel = select[::FIT_STRIDE, ::FIT_STRIDE]
    N = normals[::FIT_STRIDE, ::FIT_STRIDE][sel]
    A = np.column_stack([np.ones(len(N)), N[:, :2]])
    coef, *_ = np.linalg.lstsq(A, scaled[::FIT_STRIDE, ::FIT_STRIDE][sel], rcond=None)
    azimuth = math.degrees(math.atan2(coef[1], coef[2])) % 360
    path = [lighting.sun(acquired, datetime.time(h, m, tzinfo=lighting.MDT), *site)
            for h in range(5, 20) for m in range(60)]
    path = [s for s in path if s.elevation_deg > 0]
    best = min(path, key=lambda s: abs((s.azimuth_deg - azimuth + 180) % 360 - 180))
    return FittedSun(best.elevation_deg, azimuth)


def _dark_spots(lum, texel_m):
    """Texels more than 1 - DARK_FACTOR darker than their NEIGHBOURHOOD_M median."""
    grey = textures.linear_to_srgb(lum)
    k = min(255, int(round(NEIGHBOURHOOD_M / texel_m)) | 1)
    return grey < DARK_FACTOR * cv2.medianBlur(grey, k)


@dataclass
class Ortho:
    """An orthophoto colour map: sRGB uint8 (n, n, 3), the texels inpainted
    (cast and self shadows, 3D shrubs) and the fitted NAIP sun (None when
    not de-shaded)."""
    rgb: np.ndarray
    inpainted: np.ndarray
    sun: FittedSun = None


def ortho_colour_map(naip_path, origin: geo.Origin, size, n, dem_hf=None, units=None, inpaint_mask=None,
                     center=(0.0, 0.0), boost=NAIP2024_BOOST):
    """The colour map of a world on real ground from NAIP: resampled onto
    the n x n texels of its square, de-shaded on dem_hf (deshade; skipped
    without a DEM), the shadows and inpaint_mask (bool n x n: spots under 3D
    shrubs, disc_mask) inpainted (Telea), then boosted."""
    naip = resample_raster(naip_path, origin, size, n, center)
    if dem_hf is not None:
        lin, mask, sun = deshade(naip, dem_hf, units)
    else:
        lin, mask, sun = np.moveaxis(textures.srgb_to_linear(naip[:3]), 0, -1), np.zeros((n, n), bool), None
    if inpaint_mask is not None:
        mask = mask | inpaint_mask
    grown = cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8))
    rgb = cv2.inpaint(textures.linear_to_srgb(lin), grown, INPAINT_RADIUS_PX, cv2.INPAINT_TELEA)
    return Ortho(textures.linear_to_srgb(boost.apply(textures.srgb_to_linear(rgb))), grown.astype(bool), sun)


def detect_shrubs(naip_path, origin: geo.Origin, size, slope=None, center=(0.0, 0.0), texel_m=0.5):
    """Shrubs in a NAIP image (design spec D10): [(x, y, diameter)] in world
    coordinates. Dark spots (DARK_FACTOR of the NEIGHBOURHOOD_M median
    luminance) of shrub size whose mean NDVI exceeds that of the non-dark
    ground around them by NDVI_ANOMALY; slope: a function (x, y) -> deg (e.g.
    Heightfield.slope_deg) to drop spots on ground steeper than
    SHRUB_MAX_SLOPE. The diameter is the spot's equivalent disc, shadow and
    blur included."""
    n = int(round(size / texel_m))
    texel = size / n
    r, g, b, nir = resample_raster(naip_path, origin, size, n, center)[:4]
    lum = textures.luminance(np.stack([textures.srgb_to_linear(c) for c in (r, g, b)], axis=-1))
    dark = _dark_spots(lum, texel)
    ndvi = (nir - r) / np.maximum(nir + r, 1.0)
    k = int(round(NEIGHBOURHOOD_M / texel)) | 1
    ground = (~dark).astype(np.float32)
    anomaly = ndvi - cv2.blur(ndvi * ground, (k, k)) / np.maximum(cv2.blur(ground, (k, k)), 1e-6)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(dark.astype(np.uint8), connectivity=8)
    area = stats[:, cv2.CC_STAT_AREA].astype(float)
    mean = np.bincount(labels.ravel(), anomaly.ravel(), minlength=count) / np.maximum(area, 1)
    area_m2 = area * texel * texel
    keep = (mean > NDVI_ANOMALY) & (area_m2 >= SHRUB_AREA_M2[0]) & (area_m2 <= SHRUB_AREA_M2[1])
    keep[0] = False  # the background component
    xs = center[0] - size / 2 + (centroids[:, 0] + 0.5) * texel
    ys = center[1] + size / 2 - (centroids[:, 1] + 0.5) * texel
    if slope is not None:
        keep &= np.asarray(slope(xs, ys)) <= SHRUB_MAX_SLOPE
    d = 2 * np.sqrt(area_m2 / math.pi)
    return [(float(x), float(y), float(dd)) for x, y, dd in zip(xs[keep], ys[keep], d[keep])]


# --- Terra layers --------------------------------------------------------------------------

CONSTANT_FADE = 2000.0  # [m] a fade this long gives a nearly constant weight (M: min -755, fade 2000: 0.32-0.38)


@dataclass(frozen=True)
class DetailLayer:
    """A shared detail texture (textures.DETAILS key) over the colour map at
    a constant weight; with `above`, fading in from that heightmap z to
    `weight` at the terrain's top (slab joints on caprock)."""
    key: str
    weight: float
    above: float = None


DEFAULT_DETAILS = (DetailLayer("gravel_lag", 0.35), DetailLayer("cracked_silt", 0.15))  # (M: render prototype)


@dataclass
class TerraLayers:
    """Terra's layers for a world: layer0 (the compensated colour map, sRGB
    uint8), the detail textures [(diffuse URI, normal URI, size)] bottom to
    top with their blends [(min_height, fade_dist)], and the fraction of
    texels the compensation clipped."""
    layer0: np.ndarray
    textures: list
    blends: list
    clipped: float

    def write(self, heightmap, colour_uri, flat_normal_uri, size):
        """Add the layers to an SDF <heightmap> (the terrain's visual):
        layer 0 is the colour map (colour_uri, written from layer0) over the
        whole terrain (`size`) with a flat normal map."""
        for diffuse, normal, tile in [(colour_uri, flat_normal_uri, size)] + self.textures:
            texture = sdf.sub(heightmap, "texture")
            sdf.sub(texture, "diffuse", diffuse)
            sdf.sub(texture, "normal", normal)
            sdf.sub(texture, "size", tile)
        for min_height, fade in self.blends:
            blend = sdf.sub(heightmap, "blend")
            sdf.sub(blend, "min_height", min_height)
            sdf.sub(blend, "fade_dist", fade)


def inverse_smoothstep(w):
    """The t in [0, 1] where 3t^2 - 2t^3 = w."""
    return 0.5 - math.sin(math.asin(1 - 2 * w) / 3)


def terra_layers(colour_map, hf, details, media):
    """Terra's layers for a colour map (sRGB uint8 n x n x 3) over the
    heightmap hf (its own frame: z as Gazebo draws it) with up to three
    DetailLayers (Terra blends at most four textures, by height only).

    Terra lerps each detail layer over what lies below it by its weight
    w_i = smoothstep(min_i, min_i + fade_i, h), so the render is
    a_0 O' + sum_i a_i D_i, with a_0 = prod(1 - w_i) and
    a_i = w_i prod_{j > i} (1 - w_j). Layer 0 is pre-compensated per texel,
    O' = (O - sum_i a_i m_i) / a_0 in linear light (m_i: the mean of D_i),
    so the render is O plus zero-mean detail (render prototype terra_mix.py).
    The details are rescaled to the colour map's mean colour (Media.detail),
    so O' stays near O: the shared textures alone clipped 78 % of Autonomy's
    texels (M)."""
    if len(details) > 3:
        raise ValueError("Terra blends at most four textures: the colour map and three details")
    n = colour_map.shape[0]
    heights = at_texels(hf, n)
    top, middle = float(heights.max()), float(np.median(heights))
    out = textures.srgb_to_linear(colour_map)
    target = out.reshape(-1, 3).mean(axis=0)
    acc = np.zeros_like(out)
    a0 = np.ones(heights.shape, np.float32)
    layers, blends = [], []
    for d in details:
        if d.above is None:
            fade = CONSTANT_FADE
            low = middle - inverse_smoothstep(d.weight) * fade
        elif d.above < top:
            low, fade = d.above, (top - d.above) / inverse_smoothstep(d.weight)
        else:
            raise ValueError(f"detail {d.key} above {d.above} m: the terrain tops out at {top:.2f} m")
        diffuse, normal = media.detail(d.key, target)
        texture = np.asarray(Image.open(media.path(diffuse)).convert("RGB"))
        mean = textures.srgb_to_linear(texture).reshape(-1, 3).mean(axis=0)  # as saved: 8-bit sRGB
        w = terrain.smoothstep(low, low + fade, heights).astype(np.float32)
        acc = acc * (1 - w[..., None]) + w[..., None] * mean
        a0 *= 1 - w
        layers.append((diffuse, normal, textures.DETAILS[d.key].tile_m))
        blends.append((round(low, 4), round(fade, 4)))
    compensated = (out - acc) / a0[..., None]
    clipped = float(np.mean(np.any((compensated < 0) | (compensated > 1), axis=-1)))
    return TerraLayers(textures.linear_to_srgb(compensated), layers, blends, clipped)
