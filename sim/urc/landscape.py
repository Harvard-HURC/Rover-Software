"""Ground building blocks: what ground is where, the micro-relief under it
and what lies on it (design spec docs/superpowers/specs/2026-10-06-urc-realism-design.md,
sections 5.1-5.5 and 9.1-9.2).

- paint(): the ground raster, one Legend index (terrains.TYPES) per
  heightmap sample, painted by rules in order, later rules winning, then
  by the world's zones (terrains.Zone outlines). A synthetic world declares
  its rules as data next to its FEATURES, saying only where:

      PAINT = [landscape.Base("regolith"),
               landscape.Noise("sand_sheet", feature_m=120, cover=0.4),
               landscape.Along(WASH, "wash_sand", half_width=6.0),
               landscape.Hills(HILLS, slope="badland_slope", floor="silt_flat", cap="caprock"),
               landscape.Steeper(30.0, "rock"),
               landscape.Below("caprock", "block_field", reach_m=30.0)]

  A real-DEM world paints the NRCS soil map instead (Soils, from_ssurgo),
  each map unit by slope (design 5.3). WorldBuilder writes the raster as
  ground.png and its legend, traction table and collision map as
  ground.json (ground_json, design 9.1).
- relief(): a synthetic heightfield's micro-relief (design 5.4): real lidar
  residuals (Swatch, sim/data/relief) under each type, blended so that
  overlapping fields keep their RMS, plus haystacks and rills, kept off
  engineered ground (keep_flat).
- place(): clutter recipes (slabs, rocks, shrubs, risers) to placements for
  the WorldBuilder's rock and shrub machinery (design 5.5).
- window_rms(): the roughness every target is measured in.

Everything is deterministic for a given seed (the rng passed in).
"""
import functools
import json
import math
import zlib
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np

from . import geo, terrain, terrains

GROUND_FORMAT = "rover-ground/2"  # ground.json's format (design 9.1)
PREFIXES = {"rocks_": "rock", "slabs_": "rock", "risers_": "rock"}  # merged clutter collisions by name prefix
RELIEF_DIR = Path(__file__).resolve().parents[1] / "data" / "relief"
SOILS_PATH = Path(__file__).resolve().parents[1] / "data" / "soils" / "ssurgo_polys.geojson"
SLOPE_SMOOTH = 2.0  # [m] paint rules judge slope on the surface blurred this much (design 5.3)
GENTLE_DEG, STEEP_DEG = 8.0, 30.0  # soil-map slope classes (A, design 5.3: tuned against NAIP and landforms)
TYPE_FEATHER = 4.0  # [m] sigma of the blur that feathers one type's relief into the next (A)
KEEP_FLAT_EASE = 5.0  # [m] relief eases in over this from engineered ground (design 5.4)
EASY_ROUTE_FLAT = 3.0  # [m] half-width kept flat round a judges' easy route (design 5.4)
STEP_DEG = 45.0  # ground this steep is already a ledge face: no riser there (M: risers are > 45 deg faces)
RISER_STEP = 1.0  # [m] riser trace step
RISER_SMOOTH = 4.0  # [m] risers follow the contours of the surface blurred this much (A)


# --- Legend and ground.json ---------------------------------------------------------------

class Legend:
    """Ground types by raster index: the catalogue in order (terrains.TYPES),
    then every other type a world paints (calibration surfaces) as it first
    appears. ground.png holds the indices, ground.json the legend."""

    def __init__(self):
        self.types = list(terrains.TYPES.values())
        self._index = {t.key: i for i, t in enumerate(self.types)}

    def index(self, kind):
        """The raster index of a TerrainType (added if new) or of a key of terrains.TYPES."""
        if isinstance(kind, str):
            kind = terrains.TYPES[kind]
        i = self._index.get(kind.key)
        if i is None:
            if len(self.types) == 256:
                raise ValueError("ground.png holds at most 256 types")
            i = self._index[kind.key] = len(self.types)
            self.types.append(kind)
        elif self.types[i] != kind:
            raise ValueError(f"two different ground types are called {kind.key}")
        return i

    def __getitem__(self, index):
        return self.types[index]

    def __len__(self):
        return len(self.types)


def ground_json(legend, size, samples, default, collisions, dig=None):
    """ground.json (design 9.1): the legend with each type's traction under
    the dig-in preset `dig` (terrains.traction) and dust; `default`: the
    world's base ground; collisions: {exact collision name: type key} for
    every collision of the terrain model besides the heightmap that no
    PREFIXES entry covers."""
    known = {t.key for t in legend.types}
    unknown = (set(collisions.values()) | set(PREFIXES.values()) | {default}) - known
    if unknown:
        raise ValueError(f"ground types {sorted(unknown)} are not in the legend")
    return {"format": GROUND_FORMAT, "size_m": size, "samples": samples, "default": legend.index(default),
            "types": [dict(index=i, key=t.key, title=t.title, **asdict(terrains.traction(t, dig)),
                           dust=t.appearance.dust, dust_rgb=list(t.appearance.dust_rgb))
                      for i, t in enumerate(legend.types)],
            "collisions": dict(sorted(collisions.items())), "prefixes": dict(PREFIXES),
            "terrain_default": terrains.TERRAIN_SURFACE, "object_default": terrains.OBJECT_SURFACE}


# --- Painting ------------------------------------------------------------------------------

class Canvas:
    """A ground raster being painted on a Heightfield's grid, with what the
    rules share: the legend, the slope and the grid's coordinates."""

    def __init__(self, hf, legend):
        self.hf = hf
        self.legend = legend
        self.raster = np.full((hf.n, hf.n), legend.index(terrains.DEFAULT_GROUND), np.uint8)

    @functools.cached_property
    def slope(self):
        """Slope [deg] of the surface blurred SLOPE_SMOOTH."""
        return terrain.slope_map(self.hf.z, self.hf.res, SLOPE_SMOOTH)

    def fill(self, where, kind):
        self.raster[where] = self.legend.index(kind)

    def of(self, keys):
        """Samples painted with any of `keys` (all samples for none)."""
        if not keys:
            return np.ones(self.raster.shape, bool)
        return np.isin(self.raster, [self.legend.index(k) for k in keys])

    def pixels(self, points):
        """Layout (x, y) -> fractional (col, row) of the grid."""
        hf = self.hf
        p = np.asarray(points, float)
        return np.stack([(p[..., 0] - hf.center[0] + hf.size / 2) / hf.res,
                         (hf.center[1] + hf.size / 2 - p[..., 1]) / hf.res], axis=-1)

    def polygon(self, outline):
        """Samples inside a layout polygon (OpenCV's fill: a sample on the edge may fall either way)."""
        out = np.zeros(self.raster.shape, np.uint8)
        cv2.fillPoly(out, [np.round(self.pixels(outline) * 16).astype(np.int32)], 1, lineType=cv2.LINE_8, shift=4)
        return out.astype(bool)

    def stroke(self, path, half_width):
        """Samples within about half_width of a layout polyline."""
        out = np.zeros(self.raster.shape, np.uint8)
        thickness = max(1, int(round(2 * half_width / self.hf.res)) + 1)
        cv2.polylines(out, [np.round(self.pixels(path) * 16).astype(np.int32)], False, 1, thickness,
                      lineType=cv2.LINE_8, shift=4)
        return out.astype(bool)


def paint(hf, rules, zones=(), legend=None):
    """The ground raster of a Heightfield (uint8 Legend indices on its grid,
    row 0 north): DEFAULT_GROUND, then each rule in order, then each zone's
    outline with its type, later ones winning."""
    canvas = Canvas(hf, legend or Legend())
    for rule in rules:
        rule.paint(canvas)
    for zone in zones:
        canvas.fill(canvas.polygon(zone.outline), zone.kind)
    return canvas.raster


def _seed(key, seed):
    return (zlib.crc32(key.encode()) + seed) % (1 << 31)


@dataclass(frozen=True)
class Base:
    """Everything `key`."""
    key: str

    def paint(self, canvas):
        canvas.fill(slice(None), self.key)


@dataclass(frozen=True)
class Noise:
    """`key` in patches about feature_m across that cover `cover` of the
    ground (fractal noise above its 1 - cover quantile), only over ground
    already of the types `over` if given."""
    key: str
    feature_m: float
    cover: float
    seed: int = 0
    over: tuple = ()

    def paint(self, canvas):
        hf = canvas.hf
        noise = terrain.fbm(hf.n, hf.size, self.feature_m, _seed(self.key, self.seed), octaves=3)
        canvas.fill((noise > np.quantile(noise, 1 - self.cover)) & canvas.of(self.over), self.key)


@dataclass(frozen=True)
class Along:
    """`key` within half_width of a layout polyline (wash floors)."""
    path: tuple
    key: str
    half_width: float

    def paint(self, canvas):
        canvas.fill(canvas.stroke(self.path, self.half_width), self.key)


@dataclass(frozen=True)
class Hills:
    """Round each of `hills` (features.Mesa): `cap` on its top, `slope` down
    its cliff and `floor` out to reach_m beyond its foot (badland belts and
    their floors); None leaves that part as it is."""
    hills: tuple
    slope: str
    floor: str = None
    cap: str = None
    reach_m: float = 20.0  # (A)

    def paint(self, canvas):
        for hill in self.hills:
            edge, r = canvas.hf.mesa_edge(hill.x, hill.y, hill.radius, hill.seed, hill.irregularity)
            for key, where in ((self.floor, (r > edge + hill.cliff) & (r <= edge + hill.cliff + self.reach_m)),
                               (self.slope, (r > edge) & (r <= edge + hill.cliff)), (self.cap, r <= edge)):
                if key is not None:
                    canvas.fill(where, key)


@dataclass(frozen=True)
class Steeper:
    """`key` where the ground is steeper than `deg` (faces too steep for soil)."""
    deg: float
    key: str

    def paint(self, canvas):
        canvas.fill(canvas.slope > self.deg, self.key)


@dataclass(frozen=True)
class Band:
    """`key` where the layout height lies in [low, high), only over ground
    already of the types `over` if given (height bands)."""
    key: str
    low: float = -math.inf
    high: float = math.inf
    over: tuple = ()

    def paint(self, canvas):
        z = canvas.hf.z
        canvas.fill((z >= self.low) & (z < self.high) & canvas.of(self.over), self.key)


@dataclass(frozen=True)
class Below:
    """`key` on ground within reach_m of `above` and at least drop_m lower
    than the highest of it there, no steeper than max_slope_deg (talus
    aprons under rims; design 5.5: talus slopes 22-38 deg); `above` itself
    stays."""
    above: str
    key: str
    reach_m: float = 30.0
    drop_m: float = 1.0  # (A)
    max_slope_deg: float = 38.0

    def paint(self, canvas):
        hf = canvas.hf
        rim = canvas.raster == canvas.legend.index(self.above)
        if not rim.any():
            return
        distance = cv2.distanceTransform((~rim).astype(np.uint8), cv2.DIST_L2, 5) * hf.res
        k = 2 * int(round(self.reach_m / hf.res)) + 1
        top = cv2.dilate(np.where(rim, hf.z, -1e9).astype(np.float32), np.ones((k, k), np.uint8))
        canvas.fill((distance > 0) & (distance <= self.reach_m) & (hf.z <= top - self.drop_m)
                    & (canvas.slope <= self.max_slope_deg), self.key)


@dataclass(frozen=True)
class Unit:
    """How one SSURGO map unit paints (design 5.3): `gentle` below
    GENTLE_DEG, `moderate` up to STEEP_DEG, `steep` above; then `crest` on
    gentle local highs (CREST), `hollow` on gentle local lows (HOLLOW) and
    `cap` on steep ground near the top of a hill (CAP)."""
    name: str
    gentle: str
    moderate: str
    steep: str = "rock"
    crest: str = None
    hollow: str = None
    cap: str = None


# The soil map units round MDRS and the route area (sim/data/soils/ssurgo_polys.geojson, mukey: unit).
SSURGO_UNITS = {
    "55112": Unit("Sheppard-Leebench complex", "sand_sheet", "sand_sheet", crest="sand"),  # not fan pavement (5.3)
    "55151": Unit("Chipeta-Badland complex", "clay_crust", "badland_slope"),  # shale pediment, gravelly A [2]
    "55156": Unit("Badland-Rock outcrop complex", "silt_flat", "badland_slope", cap="caprock"),
    "55162": Unit("Farb-Farb, very shallow-Rock outcrop complex", "slickrock", "slickrock", hollow="sand_sheet"),
    "55165": Unit("Green River-Myton families complex", "gravel", "gravel", hollow="wash_sand"),
    # Outside the Autonomy square, read the same way (A):
    "55109": Unit("Badland", "silt_flat", "badland_slope"),
    "55149": Unit("Chipeta silty clay, 2 to 15 percent slopes", "clay_crust", "badland_slope"),
    "55166": Unit("Hanksville-Chipeta complex", "silt_flat", "clay_crust"),
    "55222": Unit("Billings silt loam", "silt_flat", "silt_flat"),
}
CREST = (10.0, 0.3)  # a crest stands this high [m] above the mean of the box this half-size [m] round it (A)
HOLLOW = (25.0, -1.0)  # a hollow lies this low [m] below the mean of its box (A: colour_stats' TPI50 < -1 m)
CAP = (15.0, 3.0)  # a cap is within this [m] of the highest ground within this radius [m] (A)


@dataclass(frozen=True)
class Soils:
    """The NRCS soil map [2]: each SSURGO map unit polygon (a GeoJSON of
    WGS84 polygons) painted by its Unit (units: {mukey: Unit}, default
    SSURGO_UNITS); `origin`: the WGS84 of the layout origin. Ground outside
    every polygon keeps what it had."""
    origin: geo.Origin
    path: str = str(SOILS_PATH)
    units: dict = None

    def unit_map(self, canvas):
        """Each sample's mukey index into the polygons' units (0: none) and the mukeys."""
        features = json.loads(Path(self.path).read_text())["features"]
        mukeys = sorted({f["properties"]["mukey"] for f in features})
        out = np.zeros(canvas.raster.shape, np.uint8)
        for f in features:
            geometry = f["geometry"]
            polygons = [geometry["coordinates"]] if geometry["type"] == "Polygon" else geometry["coordinates"]
            for rings in polygons:
                for k, ring in enumerate(rings):  # the exterior, then its holes
                    xy = [geo.wgs84_to_enu(self.origin, lat, lon)[:2] for lon, lat in ring]
                    value = mukeys.index(f["properties"]["mukey"]) + 1 if k == 0 else 0
                    cv2.fillPoly(out, [np.round(canvas.pixels(xy) * 16).astype(np.int32)], value, shift=4)
        return out, mukeys

    def paint(self, canvas):
        hf = canvas.hf
        units = SSURGO_UNITS if self.units is None else self.units
        found, mukeys = self.unit_map(canvas)
        slope = canvas.slope
        z = hf.z.astype(np.float32)

        def tpi(half):
            k = 2 * int(round(half / hf.res)) + 1
            return hf.z - cv2.blur(z, (k, k), borderType=cv2.BORDER_REFLECT)

        gentle, steep = slope < GENTLE_DEG, slope > STEEP_DEG
        for i, mukey in enumerate(mukeys, 1):
            unit = units.get(mukey)
            here = found == i
            if unit is None or not here.any():
                continue
            canvas.fill(here & gentle, unit.gentle)
            canvas.fill(here & ~gentle & ~steep, unit.moderate)
            canvas.fill(here & steep, unit.steep)
            if unit.crest:
                canvas.fill(here & gentle & (tpi(CREST[0]) > CREST[1]), unit.crest)
            if unit.hollow:
                canvas.fill(here & gentle & (tpi(HOLLOW[0]) < HOLLOW[1]), unit.hollow)
            if unit.cap:
                k = 2 * int(round(CAP[0] / hf.res)) + 1
                top = cv2.dilate(z, np.ones((k, k), np.uint8))
                canvas.fill(here & steep & (hf.z > top - CAP[1]), unit.cap)


def from_ssurgo(hf, origin, geojson=SOILS_PATH, slope_rules=None, legend=None):
    """The ground raster of a real-DEM heightfield from the soil map
    (design 5.3): DEFAULT_GROUND, then the Soils rule."""
    return paint(hf, [Soils(origin, str(geojson), slope_rules)], legend=legend)


# --- Relief --------------------------------------------------------------------------------

@dataclass(frozen=True)
class Swatch:
    """A relief swatch (design 9.2): high-pass residuals of real 0.5 m lidar
    windows (float grids, row 0 north) and their provenance."""
    key: str
    windows: tuple
    res_m: float
    sigma_m: float
    source: list
    rms_cm: dict


@functools.lru_cache(maxsize=None)
def swatch(key, directory=RELIEF_DIR):
    """sim/data/relief/<key>.npz (sim/tools/make_relief_swatches.py)."""
    with np.load(Path(directory) / f"{key}.npz") as f:
        names = sorted((n for n in f.files if n == "z" or n.startswith("z_")), key=lambda n: (n != "z", n))
        return Swatch(key, tuple(f[n].astype(np.float32) for n in names), float(f["res_m"]), float(f["sigma_m"]),
                      json.loads(str(f["source"])), json.loads(str(f["rms_cm"])))


def type_masks(raster, res, present=None):
    """{index: weight}: each type's indicator feathered by a blur of
    TYPE_FEATHER (computed on a ~2 m grid), the weights summing to 1."""
    present = np.unique(raster) if present is None else present
    n = raster.shape[0]
    step = max(1, int(round(2.0 / res)))
    coarse = max(2, (n - 1) // step + 1)
    out = {}
    for i in present:
        small = cv2.resize((raster == i).astype(np.float32), (coarse, coarse), interpolation=cv2.INTER_AREA)
        small = cv2.GaussianBlur(small, (0, 0), TYPE_FEATHER / (res * (n - 1) / (coarse - 1)))
        out[int(i)] = cv2.resize(small, (n, n), interpolation=cv2.INTER_LINEAR).astype(float)
    total = sum(out.values())
    return {i: m / np.maximum(total, 1e-9) for i, m in out.items()}


def relief(hf, raster, keep_flat=None, legend=None, seed=0):
    """The micro-relief of a synthetic heightfield (design 5.4): dz to add.

    Under each type its swatch (Relief.swatch, scaled by amplitude): every
    swatch is tiled once over the grid (terrain.swatch_field) and types
    that share one share its tiling. Types meet through feathered weights
    m_t (type_masks); grouped by swatch k, W_k = sum m_t and A_k = sum m_t
    amplitude_t, dz = sum A_k field_k / sqrt(sum W_k^2), types without a
    swatch counting as one group: independent fields keep their RMS across
    a seam, and relief fades out into ground without any. Then haystacks
    and rills on the types whose relief has them. keep_flat: a weight in [0,
    1] (keep_flat()) multiplying all of it, 0 on engineered ground."""
    legend = legend or Legend()
    keep = 1.0 if keep_flat is None else np.asarray(keep_flat)
    masks = type_masks(raster, hf.res)
    weights, amplitudes = {}, {}
    for i, m in masks.items():
        r = legend[i].relief
        group = r.swatch if r.swatch and r.amplitude else None
        weights[group] = weights.get(group, 0.0) + m
        amplitudes[group] = amplitudes.get(group, 0.0) + m * r.amplitude
    norm = np.sqrt(sum(w * w for w in weights.values()))
    dz = np.zeros((hf.n, hf.n))
    for group, a in amplitudes.items():
        if group is None:
            continue
        s = swatch(group)
        sigmas = {legend[i].relief.swatch_sigma_m for i in masks if legend[i].relief.swatch == group}
        if sigmas != {s.sigma_m}:
            raise ValueError(f"swatch {group} was cut with sigma {s.sigma_m} m, its recipes say {sorted(sigmas)}")
        dz += a / np.maximum(norm, 1e-9) * terrain.swatch_field(s.windows, s.res_m, hf.n, hf.size, _seed(group, seed))
    dz *= keep
    surface = terrain.Heightfield(hf.size, hf.n, hf.z + dz, hf.center)
    for i, m in masks.items():
        kind = legend[i]
        r = kind.relief
        if r.haystacks:
            surface.haystacks(m * keep, _seed(kind.key + "/haystacks", seed), r.haystacks)
        if r.rills:
            surface.rills(m * keep, _seed(kind.key + "/rills", seed), r.rills)
    return surface.z - hf.z


def keep_flat(hf, features=(), pads=(), paths=(), ease=KEEP_FLAT_EASE):
    """The weight relief is multiplied by (relief()): 0 on engineered
    ground, easing to 1 over `ease` metres (design 5.4). features: those
    whose keep_flat() names layout polygons (lanes, slopes, ledges, steps,
    the proving-ground tracks); pads: [(x, y, radius)] (C2, objects,
    posts); paths: [(polyline, half_width)] (judges' easy routes,
    EASY_ROUTE_FLAT)."""
    canvas = Canvas(hf, Legend())
    flat = np.zeros((hf.n, hf.n), bool)
    for feature in features:
        for outline in getattr(feature, "keep_flat", lambda: [])():
            flat |= canvas.polygon(outline)
    for x, y, radius in pads:
        disc = np.zeros(flat.shape, np.uint8)
        (c, r), = canvas.pixels([(x, y)])
        cv2.circle(disc, (int(round(c * 16)), int(round(r * 16))), int(round(radius / hf.res * 16)), 1, -1,
                   shift=4)
        flat |= disc.astype(bool)
    for path, half_width in paths:
        flat |= canvas.stroke(path, half_width)
    distance = cv2.distanceTransform((~flat).astype(np.uint8), cv2.DIST_L2, 5) * hf.res
    return terrain.smoothstep(0.0, ease, distance)


# --- Clutter -------------------------------------------------------------------------------

class Placement(NamedTuple):
    """Where one piece of clutter goes: layout (x, y) [m], size [m] (slabs:
    the equivalent diameter D; rocks: the long half-axis, WorldBuilder
    .rock_field's size; shrubs: the diameter), yaw [rad], tilt [rad] (a
    slab leans this much from the ground under it, about its own x axis)
    and height [m] (slabs: thickness; shrubs: height; rocks: 0, the mesh
    decides)."""
    x: float
    y: float
    size: float
    yaw: float
    tilt: float
    height: float


class Riser(NamedTuple):
    """A sub-metre ledge along a contour: its layout polyline (k, 2), the
    height of its face [m] and how far it reaches back into the ground [m]."""
    path: np.ndarray
    height: float
    depth: float


def slab_count(recipe, d):
    """N(>=d) per 100 m2 of a Slabs recipe: its table interpolated log-log,
    the power law of its exponent below it; 0 beyond its last entry."""
    sizes, counts = (np.log(np.array(c, float)) for c in zip(*recipe.counts))
    d = np.asarray(d, float)
    inside = np.exp(np.interp(np.log(np.clip(d, *np.exp(sizes[[0, -1]]))), sizes, counts))
    below = np.exp(counts[0]) * (d / np.exp(sizes[0])) ** -recipe.exponent
    out = np.where(d < np.exp(sizes[0]), below, inside)
    return np.where(d > np.exp(sizes[-1]) + 1e-9, 0.0, out)


def slab_sizes(recipe, u, low, high):
    """D [m] of slabs for uniform draws u in [0, 1): the cumulative counts
    between low and high inverted (log-log)."""
    d = np.geomspace(low, high, 2048)
    n = slab_count(recipe, d)
    target = n[0] - np.asarray(u) * (n[0] - n[-1])
    return np.exp(np.interp(np.log(target), np.log(n[::-1]), np.log(d[::-1])))


def golombek_cover(k, d):
    """Fraction of the ground covered by rocks of diameter >= d [m] for a
    total rock cover k (Golombek & Rapp 1997 [7]: k exp(-q d), q = 1.79 +
    0.152 / k)."""
    return k * np.exp(-(1.79 + 0.152 / k) * np.asarray(d, float))


def _avoid_mask(hf, avoid, clearance):
    """Samples within clearance of any of the layout points `avoid`."""
    out = np.zeros((hf.n, hf.n), np.uint8)
    if len(avoid) and clearance > 0:
        radius = int(round(clearance / hf.res * 16))
        for c, r in Canvas(hf, Legend()).pixels(np.asarray(avoid, float)):
            cv2.circle(out, (int(round(c * 16)), int(round(r * 16))), radius, 1, -1, shift=4)
    return out.astype(bool)


def place(hf, raster, kind, rng, avoid=(), clearance=0.0, legend=None, within=None, sizes=None):
    """Clutter of one kind ("slabs", "rocks", "shrubs" or "risers") on every
    type of the raster whose Clutter recipe has it (design 5.5): Placements
    (Risers for risers), type by type in legend order. Densities hold on
    the ground each type covers within `within` (a boolean grid, default
    everywhere) and `clearance` metres clear of the layout points `avoid`
    (WorldBuilder.keep_clear); sizes: (low, high) [m] instead of the
    recipe's range (real-DEM worlds: slabs of 0.15-1 m only, larger blocks
    are in the DEM, design D9).

    Slabs: D by inverting the tabulated cumulative counts, as many as
    N(>=low) - N(>=high) per 100 m2 (Poisson), thickness recipe.height D x
    U(0.7, 1.3), tilt U(0, max_tilt_deg). Rocks: D log-uniform, as many as
    cover the Golombek fraction of their size range. Shrubs: per_ha, height
    and diameter uniform. Risers: rise_traces()."""
    legend = legend or Legend()
    blocked = _avoid_mask(hf, avoid, clearance)
    allowed = ~blocked if within is None else (~blocked & np.asarray(within, bool))
    out = []
    for i in np.unique(raster):
        recipe = getattr(legend[i].clutter, kind)
        if recipe is None:
            continue
        cells = np.flatnonzero((raster == i) & allowed)
        area = len(cells) * hf.res * hf.res
        if kind == "risers":
            out += rise_traces(hf, (raster == i) & allowed, recipe, rng)
            continue
        if kind == "slabs":
            low, high = sizes or (recipe.d_min, recipe.d_max)
            expected = float(slab_count(recipe, low) - slab_count(recipe, high)) * area / 100.0
        elif kind == "rocks":
            low, high = sizes or recipe.sizes_m
            mean_area = math.pi / 4 * (high * high - low * low) / (2 * math.log(high / low))  # log-uniform D
            expected = float(golombek_cover(recipe.k, low) - golombek_cover(recipe.k, high)) * area / mean_area
        elif kind == "shrubs":
            expected = recipe.per_ha * area / 1e4
        else:
            raise ValueError(f"clutter kind {kind!r}")
        count = int(rng.poisson(expected)) if len(cells) else 0
        picked = cells[rng.integers(0, len(cells), count)] if count else np.zeros(0, int)
        rows, cols = np.divmod(picked, hf.n)
        x = hf.center[0] - hf.size / 2 + (cols + rng.uniform(-0.5, 0.5, count)) * hf.res
        y = hf.center[1] + hf.size / 2 - (rows + rng.uniform(-0.5, 0.5, count)) * hf.res
        yaw = rng.uniform(0, 2 * math.pi, count)
        if kind == "slabs":
            d = slab_sizes(recipe, rng.uniform(0, 1, count), low, high)
            tilt = np.radians(rng.uniform(0, recipe.max_tilt_deg, count))
            height = recipe.height * d * rng.uniform(0.7, 1.3, count)
            size = d
        elif kind == "rocks":
            size = np.exp(rng.uniform(math.log(low), math.log(high), count)) / 2
            tilt, height = np.zeros(count), np.zeros(count)
        else:
            size = rng.uniform(*recipe.diameter_m, count)
            height = rng.uniform(*recipe.height_m, count)
            tilt = np.zeros(count)
        out += [Placement(*map(float, p)) for p in zip(x, y, size, yaw, tilt, height)]
    return out


def rise_traces(hf, where, recipe, rng):
    """Risers along contours where `where` holds (a terrains.Risers recipe):
    seeds on a jittered grid of a pitch drawn from recipe.spacing_m, each a
    segment of length U(segment_m) traced RISER_STEP at a time both ways
    along the contour of the surface blurred RISER_SMOOTH (all seeds at once),
    stopping where it leaves `where`; a riser that would cross ground
    steeper than STEP_DEG (a face the DEM or the macro shape already has) is
    dropped."""
    n = hf.n
    smooth = terrain.blur(hf.z, RISER_SMOOTH / hf.res)
    g_row, g_col = np.gradient(smooth, hf.res)
    steep = terrain.slope_map(hf.z, hf.res) > STEP_DEG
    pitch = rng.uniform(*recipe.spacing_m) / hf.res
    grid = np.stack(np.meshgrid(np.arange(0.0, n - 1, pitch), np.arange(0.0, n - 1, pitch), indexing="ij"),
                    axis=-1).reshape(-1, 2)
    seeds = np.clip(grid + rng.uniform(0, pitch, grid.shape), 0, n - 1)
    ok = where[np.round(seeds[:, 0]).astype(int), np.round(seeds[:, 1]).astype(int)]
    seeds = seeds[ok]
    if not len(seeds):
        return []
    lengths = rng.uniform(*recipe.segment_m, len(seeds))
    heights = rng.uniform(*recipe.height_m, len(seeds))
    depths = rng.uniform(*recipe.depth_m, len(seeds))

    def at(grid, q):
        r, c = (np.clip(np.round(q[:, k]).astype(int), 0, n - 1) for k in (0, 1))
        return grid[r, c]

    def trace(sign):
        p, alive = seeds.copy(), np.ones(len(seeds), bool)
        points, live = [p.copy()], [alive.copy()]
        tangent = np.stack([-at(g_col, p), at(g_row, p)], axis=1)  # along the contour, in (row, col)
        for k in range(int(max(lengths) / 2 / RISER_STEP)):
            t = np.stack([-at(g_col, p), at(g_row, p)], axis=1)
            t = np.where((np.sum(t * tangent, axis=1) < 0)[:, None], -t, t)  # keep going the same way
            norm = np.linalg.norm(t, axis=1)
            t = np.where((norm > 1e-9)[:, None], t / np.maximum(norm, 1e-9)[:, None], tangent)
            q = p + sign * RISER_STEP / hf.res * t
            inside = (q >= 0).all(axis=1) & (q <= n - 1).all(axis=1)
            alive &= inside & ((k + 1) * RISER_STEP <= lengths / 2)
            alive &= at(where, q)
            p = np.where(alive[:, None], q, p)
            tangent = t
            points.append(p.copy())
            live.append(alive.copy())
        return np.stack(points, axis=1), np.stack(live, axis=1)

    forward, live_f = trace(1.0)  # live[:, k]: point k was reached (the seed always)
    back, live_b = trace(-1.0)
    out = []
    for s in range(len(seeds)):
        path = np.concatenate([back[s, 1:live_b[s].sum()][::-1], forward[s, :live_f[s].sum()]])
        if len(path) < 2 or steep[np.round(path[:, 0]).astype(int), np.round(path[:, 1]).astype(int)].any():
            continue
        xy = np.stack([hf.center[0] - hf.size / 2 + path[:, 1] * hf.res,
                       hf.center[1] + hf.size / 2 - path[:, 0] * hf.res], axis=1)
        out.append(Riser(xy, float(heights[s]), float(depths[s])))
    return out


# --- Roughness -----------------------------------------------------------------------------

def windows(grid, k):
    """Non-overlapping k x k windows of a grid, row-major: (count, k * k)."""
    rows, cols = (s // k * k for s in grid.shape)
    return grid[:rows, :cols].reshape(rows // k, k, cols // k, k).transpose(0, 2, 1, 3).reshape(-1, k * k)


def window_rms(z, res, size_m):
    """Plane-detrended RMS height [m] in non-overlapping square windows of
    round(size_m / res) samples (design 5.4: the measure of every roughness
    target), one per window, row-major."""
    k = int(round(size_m / res))
    w = windows(np.asarray(z, float), k)
    w = w - w.mean(axis=1, keepdims=True)
    yy, xx = np.mgrid[0:k, 0:k]
    x, y = (a.ravel() - (k - 1) / 2 for a in (xx, yy))
    sxx = float(x @ x)
    residual = (w * w).sum(axis=1) - (w @ x) ** 2 / sxx - (w @ y) ** 2 / sxx
    return np.sqrt(np.maximum(residual, 0.0) / (k * k))


def window_share(raster, value, res, size_m):
    """Fraction of each window's samples (window_rms's windows) equal to value."""
    return (windows(np.asarray(raster), int(round(size_m / res))) == value).mean(axis=1)
