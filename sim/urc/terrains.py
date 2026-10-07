"""Ground types and friction zones.

TYPES is the one catalogue of ground: every world picks its terrain layers,
zones and decals from it by name, and its textures are generated once into
urc_media and shared (TerrainType.texture). ROCKS and SHRUBS hold the rock
and shrub colours (WorldBuilder.rock_field, WorldBuilder.shrubs) the same way.

Gazebo honours <friction> only on primitive collision shapes (box, plane,
...): the heightmap and every mesh always have mu 1.0 (HEIGHTMAP_MU), and
DART combines two touching shapes as min(mu_a, mu_b). A friction zone with a
lower mu therefore has to be the only thing its wheels touch:

- thin box tiles with the zone's mu lie on the terrain, each tilted to the
  least-squares plane of the ground under it and lifted TILE_CLEARANCE above
  its highest point, so a wheel on the tiles never reaches the heightmap.
  Measured on the proving ground: a rover parked on the 20 deg ramp of the
  mu 0.20 lane slides 5.4 m in 3 s, and holds without the tiles;
- fit_tiles refuses ground that is not planar within FLATNESS under a single
  tile, so a tile sits at most TILE_CLEARANCE + FLATNESS above the ground and
  neighbouring tiles meet without a kerb: the ground under a natural zone is
  levelled first (features.Patch levels out to TILE_REACH past its outline);
- rocks that collide are kept off the tiles (WorldBuilder.rock_field drops
  them, and refuses a zone declared over earlier ones): a rock's mesh has
  mu 1.0 and would be a foothold;
- a textured decal draped over the surface shows the zone.
A zone of mu 1.0 (slickrock, crusts) is only its decal: tiles would change
nothing.

Every collision shape costs time every step whether anything touches it or
not (0.56-0.83 us measured, see world.py), so tiles are merged into
rectangles up to MAX_TILE wherever the ground under them is planar within
FLATNESS.

DART has no sinkage or rolling resistance, so mu is the only lever: each
type's mu is a tuning knob standing in for its traction (a rover holds or
climbs a slope up to about atan(mu)), not a measured property. The tyres have
mu 1.0 along the tread and 0.5 across it (gen_model.Params), so a zone mu
above 1.0 changes nothing, and one above 0.5 only the grip along the tread.
"""
import math
import zlib
from dataclasses import dataclass, field

import numpy as np

from . import meshes, textures

HEIGHTMAP_MU = 1.0  # what Gazebo gives heightmaps and meshes, whatever the SDF says
TILE = 1.5  # [m] tile grid
MAX_TILE = 24.0  # [m] largest merged tile side
FLATNESS = 0.02  # [m] ground under any tile is planar within this (it bounds how far a tile is off the ground)
TILE_THICKNESS = 0.1  # [m]
TILE_CLEARANCE = 0.015  # [m] tile top above the highest ground under it
TILE_OVERLAP = 0.015  # [m] tiles reach this far past their footprint, so wheels never meet a crack
TILE_REACH = TILE / math.sqrt(2) + TILE_OVERLAP  # [m] how far tiles reach past a round zone's outline
DECAL_OFFSET = 0.02  # [m] decal above the original surface
DECAL_MARGIN = 0.2  # [m] the decal reaches this far past the zone outline
DECAL_STEP = 0.5  # [m] decal mesh spacing (and a round zone's outline spacing)
DECAL_TILE = 4.0  # [m] decal texture repeat
NORMAL_STRENGTH = 2.0  # bumpiness of the ground normal maps


@dataclass(frozen=True)
class TerrainType:
    key: str
    title: str
    mu: float
    rgb: tuple  # texture base colour
    pebbles: float = 0.001  # texture: pebbles per pixel
    variation: float = 0.25  # texture: mottling
    notes: str = ""

    @property
    def max_slope_deg(self):
        """Steepest slope a parked or climbing rover holds: tan(slope) = mu."""
        return math.degrees(math.atan(self.mu))

    @property
    def seed(self):
        return zlib.crc32(self.key.encode()) % 100_000

    def texture(self, media):
        """The type's ground texture in urc_media, shared by every world (as a
        terrain layer or a zone decal)."""
        return media.texture(f"ground_{self.key}", textures.terrain_texture, self.rgb, seed=self.seed,
                             variation=self.variation, pebbles=self.pebbles)

    def normal(self, media):
        """The type's normal map in urc_media (terrain layers)."""
        return media.texture(f"ground_{self.key}_normal", textures.normal_map, seed=self.seed + 1,
                             strength=NORMAL_STRENGTH)


# Zone types: below mu 1.0 friction zones (tiles, see above); slickrock is a decal.
GRAVEL = TerrainType("gravel", "Gravel", 0.6, (150, 128, 108), pebbles=0.06, variation=0.35,
                     notes="Loose pebbles on packed ground: rolls under the wheels.")
SAND = TerrainType("sand", "Soft sand", 0.4, (218, 186, 140), pebbles=0.002, variation=0.3,
                   notes="Wind-blown or wash sand. Real sand also sinks and resists rolling; DART cannot, so low "
                         "traction stands in for both.")
SCREE = TerrainType("scree", "Loose scree", 0.35, (138, 112, 94), pebbles=0.05, variation=0.45,
                    notes="Loosely consolidated rock debris on steep slopes (URC 1.c.ii): slides from about 19 deg.")
CLAY = TerrainType("clay", "Dusty bentonite clay", 0.25, (158, 156, 164), pebbles=0.0005, variation=0.6,
                   notes="Morrison bentonite weathered to a 'popcorn' crust over powder: slippery, slides from "
                         "about 14 deg.")
SLICKROCK = TerrainType("slickrock", "Slickrock sandstone slab", 1.0, (222, 184, 140), pebbles=0.0,
                        notes="Bare cemented sandstone: the best grip there is (the tyres' own mu 1.0 limits it); "
                              "smooth, so steep slabs are climbable. A decal only: the heightmap has mu 1.0 too.")
# Ground that looks different but grips like the heightmap (mu 1.0): terrain
# layers and decals.
REGOLITH = TerrainType("regolith", "Packed regolith", 1.0, (190, 150, 115),
                       notes="The default ground: the heightmap itself, mu 1.0 (Gazebo ignores mu on heightmaps).")
PAVEMENT = TerrainType("pavement", "Desert pavement", 1.0, (166, 122, 93), pebbles=0.01,
                       notes="Packed ground armoured with small stones, between the washes and the hills.")
MUDSTONE = TerrainType("mudstone", "Maroon mudstone", 1.0, (138, 82, 70),
                       notes="Morrison-like maroon and purple mudstone bands on the hills.")
BENTONITE = TerrainType("bentonite", "Grey bentonitic mudstone", 1.0, (148, 148, 158),
                        notes="Grey-blue bentonite bands on the hills; where it weathers to powder it is CLAY.")
CAPROCK = TerrainType("caprock", "Sandstone caprock", 1.0, (212, 194, 156),
                      notes="Cemented sandstone capping the heights.")
BIOCRUST = TerrainType("biocrust", "Biological soil crust", 1.0, (72, 60, 50), pebbles=0.004, variation=0.5,
                       notes="Dark knobbly cyanobacteria and lichen crust on stable flats.")
GYPSUM = TerrainType("gypsum", "Gypsum crust", 1.0, (216, 210, 196), pebbles=0.004, variation=0.5,
                     notes="White evaporite crust on a low mound.")
TYPES = {t.key: t for t in (GRAVEL, SAND, SCREE, CLAY, SLICKROCK, REGOLITH, PAVEMENT, MUDSTONE, BENTONITE, CAPROCK,
                            BIOCRUST, GYPSUM)}

# Rock colours by palette: rock_field gives each rock one of its palette's.
ROCKS = {"desert": ((0.47, 0.33, 0.25), (0.55, 0.45, 0.36), (0.42, 0.40, 0.40), (0.60, 0.50, 0.38),
                    (0.38, 0.27, 0.22)),
         "lichen": ((0.62, 0.64, 0.32),)}  # epilithic lichen covering boulders
SHRUBS = ((0.42, 0.47, 0.33), (0.37, 0.42, 0.29), (0.47, 0.49, 0.38))  # sage, dark and grey-green brush


def calibration_surface(mu):
    """A plain test surface of a given mu (proving-ground lanes), coloured
    from pale blue-grey (slippery) to dark (grippy)."""
    t = min(max((mu - 0.2) / 0.8, 0.0), 1.0)
    rgb = tuple(int(round(a + (b - a) * t)) for a, b in zip((182, 200, 214), (88, 80, 74)))
    return TerrainType(f"test_mu{round(mu * 100):03d}", f"Test surface mu {mu:.2f}", mu, rgb, pebbles=0.0,
                       variation=0.15, notes="Uniform friction for calibration lanes.")


@dataclass
class Tile:
    """A box whose top face lies on the plane z = top[2] + g . ((x, y) - top[:2])."""
    footprint: np.ndarray  # (4, 2) layout corners, counter-clockwise
    top: np.ndarray  # layout point of the top face above the footprint centre
    gradient: np.ndarray  # (dz/dx, dz/dy) of the top face
    axes: np.ndarray  # 3x3, columns: the box's x, y and z (the top's normal) in the layout frame
    size: tuple  # box size [m]

    @property
    def center(self):
        return self.top - TILE_THICKNESS / 2 * self.axes[:, 2]


@dataclass
class Zone:
    """A friction zone: an outline, its terrain type and its tiles. Tiles
    are rectangles of a grid in the zone's frame (x, y, yaw) with cell edges
    xs, ys: a cell is used if its centre lies inside the outline, so the
    tiles follow an irregular outline to within half a cell."""
    key: str
    kind: TerrainType
    outline: np.ndarray  # (n, 2) layout polygon, counter-clockwise
    frame: tuple  # (x, y, yaw)
    xs: np.ndarray
    ys: np.ndarray
    decal: dict  # shape parameters for the decal mesh
    tiles: list = field(default_factory=list)

    @property
    def mu(self):
        """The type's: TYPES alone says what a ground grips like."""
        return self.kind.mu

    @property
    def area(self):
        return _polygon_area(self.outline)

    @property
    def tiled_area(self):
        return sum(_polygon_area(t.footprint) for t in self.tiles)

    def to_layout(self, u, v):
        x0, y0, yaw = self.frame
        c, s = math.cos(yaw), math.sin(yaw)
        return x0 + c * np.asarray(u) - s * np.asarray(v), y0 + s * np.asarray(u) + c * np.asarray(v)

    def to_frame(self, x, y):
        x0, y0, yaw = self.frame
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = np.asarray(x) - x0, np.asarray(y) - y0
        return c * dx + s * dy, -s * dx + c * dy


def blob(key, kind, x, y, radius, seed, irregularity=0.3):
    """An irregular round zone (the outline of meshes.blob_outline)."""
    step = min(DECAL_STEP, radius / 8)
    theta, edge = meshes.blob_outline(radius, seed, irregularity, step)
    outline = np.stack([x + edge * np.cos(theta), y + edge * np.sin(theta)], axis=1)
    n = int(math.ceil(radius / TILE))
    edges = np.arange(-n, n + 1) * TILE
    return Zone(key, kind, outline, (x, y, 0.0), edges, edges.copy(),
                dict(shape="blob", radius=radius, seed=seed, irregularity=irregularity, step=step))


def rect(key, kind, x, y, length, width, yaw=0.0, breaks=()):
    """A rectangular zone, `length` along yaw and `width` across, centred on
    (x, y). Tile edges fall on `breaks` (distances from the start): put them
    on the terrain's kinks, so tiles stay planar."""
    xs = _edges([0.0, length, *breaks]) - length / 2
    ys = _edges([0.0, width]) - width / 2
    zone = Zone(key, kind, None, (x, y, yaw), xs, ys,
                dict(shape="rect", length=length, width=width, yaw=yaw, breaks=tuple(breaks)))
    u, v = np.array([-1, 1, 1, -1]) * length / 2, np.array([-1, -1, 1, 1]) * width / 2
    zone.outline = np.stack(zone.to_layout(u, v), axis=1)
    return zone


def _edges(cuts):
    """Sorted cuts, each interval split evenly into pieces of at most TILE."""
    cuts = np.unique(np.round(cuts, 6))
    out = [cuts[0]]
    for a, b in zip(cuts[:-1], cuts[1:]):
        n = int(math.ceil((b - a) / TILE - 1e-9))
        out += list(a + (b - a) * np.arange(1, n + 1) / n)
    return np.array(out)


# --- Tiles ---------------------------------------------------------------------------

def inside(polygon, x, y):
    """Even-odd point-in-polygon test, vectorised over x, y."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    result = np.zeros(np.broadcast(x, y).shape, bool)
    for (x0, y0), (x1, y1) in zip(polygon, np.roll(polygon, -1, axis=0)):
        if y0 == y1:
            continue
        crosses = (y0 > y) != (y1 > y)
        result ^= crosses & (x < x0 + (y - y0) * (x1 - x0) / (y1 - y0))
    return result


def _polygon_area(polygon):
    x, y = np.asarray(polygon).T
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _cells_used(zone):
    """used[i, j]: the centre of cell (row i along ys, column j along xs) lies in the outline."""
    U, V = np.meshgrid((zone.xs[:-1] + zone.xs[1:]) / 2, (zone.ys[:-1] + zone.ys[1:]) / 2)
    return inside(zone.outline, *zone.to_layout(U, V))


def _ground_samples(hf, zone, u0, u1, v0, v1):
    """Layout (x, y, z) of the original surface under a frame rectangle: a grid
    of half the heightfield spacing plus every heightfield sample inside (the
    surface is bilinear between samples, so its extremes are among them)."""
    step = min(hf.res, TILE) / 2
    nu, nv = (max(2, int(math.ceil((b - a) / step)) + 1) for a, b in ((u0, u1), (v0, v1)))
    U, V = (a.ravel() for a in np.meshgrid(np.linspace(u0, u1, nu), np.linspace(v0, v1, nv)))
    X, Y = zone.to_layout(U, V)
    cx, cy = zone.to_layout([u0, u1, u1, u0], [v0, v0, v1, v1])
    col = (np.array([cx.min(), cx.max()]) - hf.center[0] + hf.size / 2) / hf.res
    row = (hf.size / 2 - np.array([cy.max(), cy.min()]) + hf.center[1]) / hf.res
    cols = np.arange(max(int(math.ceil(col[0])), 0), min(int(math.floor(col[1])), hf.n - 1) + 1)
    rows = np.arange(max(int(math.ceil(row[0])), 0), min(int(math.floor(row[1])), hf.n - 1) + 1)
    C, R = np.meshgrid(cols, rows)
    GX = hf.center[0] - hf.size / 2 + C.ravel() * hf.res
    GY = hf.center[1] + hf.size / 2 - R.ravel() * hf.res
    gu, gv = zone.to_frame(GX, GY)
    keep = (gu >= u0) & (gu <= u1) & (gv >= v0) & (gv <= v1)
    X, Y = np.concatenate([X, GX[keep]]), np.concatenate([Y, GY[keep]])
    return X, Y, hf.height(X, Y)


def _plane(hf, zone, u0, u1, v0, v1):
    """Least-squares plane of the ground under a frame rectangle: (centre (x, y),
    (gx, gy, c0), residuals)."""
    X, Y, Z = _ground_samples(hf, zone, u0, u1, v0, v1)
    xc, yc = (float(a) for a in zone.to_layout((u0 + u1) / 2, (v0 + v1) / 2))
    A = np.stack([X - xc, Y - yc, np.ones_like(X)], axis=1)
    coef, *_ = np.linalg.lstsq(A, Z, rcond=None)
    return (xc, yc), coef, Z - A @ coef


def fit_tiles(hf, zone):
    """Cover the zone's used cells with as few planar tiles as possible:
    greedy maximal rectangles, grown along x then y while the ground under
    them stays within FLATNESS of a plane and no side exceeds MAX_TILE.
    Raises ValueError where the ground under a single cell is not planar
    within FLATNESS: its tile would float above the ground in places and
    leave kerbs at its neighbours."""
    ok = _cells_used(zone)
    used = np.zeros_like(ok)
    xs, ys = zone.xs, zone.ys

    def bumpiness(i0, i1, j0, j1):
        residual = _plane(hf, zone, xs[j0], xs[j1], ys[i0], ys[i1])[2]
        return residual.max() - residual.min()

    def flat(i0, i1, j0, j1):
        if xs[j1] - xs[j0] > MAX_TILE + 1e-9 or ys[i1] - ys[i0] > MAX_TILE + 1e-9:
            return False
        if not ok[i0:i1, j0:j1].all() or used[i0:i1, j0:j1].any():
            return False
        return bumpiness(i0, i1, j0, j1) <= FLATNESS

    tiles = []
    for i in range(ok.shape[0]):
        for j in range(ok.shape[1]):
            if not ok[i, j] or used[i, j]:
                continue
            if bumpiness(i, i + 1, j, j + 1) > FLATNESS:
                x, y = (float(a) for a in zone.to_layout((xs[j] + xs[j + 1]) / 2, (ys[i] + ys[i + 1]) / 2))
                raise ValueError(f"zone {zone.key}: the ground at ({x:.1f}, {y:.1f}) is not planar within "
                                 f"{FLATNESS} m under one {TILE} m tile; level it first (features.Patch)")
            j1 = j + 1
            while j1 < ok.shape[1] and flat(i, i + 1, j, j1 + 1):
                j1 += 1
            i1 = i + 1
            while i1 < ok.shape[0] and flat(i, i1 + 1, j, j1):
                i1 += 1
            used[i:i1, j:j1] = True
            tiles.append(_tile(hf, zone, xs[j], xs[j1], ys[i], ys[i1]))
    zone.tiles = tiles
    return tiles


def _tile(hf, zone, u0, u1, v0, v1):
    (xc, yc), (gx, gy, c0), residual = _plane(hf, zone, u0, u1, v0, v1)
    top = np.array([xc, yc, c0 + TILE_CLEARANCE + residual.max()])
    n = np.array([-gx, -gy, 1.0]) / math.sqrt(gx * gx + gy * gy + 1)
    yaw = zone.frame[2]
    u = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    bx = u - (u @ n) * n
    bx /= np.linalg.norm(bx)
    axes = np.stack([bx, np.cross(n, bx), n], axis=1)
    fx, fy = zone.to_layout([u0, u1, u1, u0], [v0, v0, v1, v1])
    footprint = np.stack([fx, fy], axis=1)
    lifted = np.column_stack([footprint, top[2] + gx * (fx - xc) + gy * (fy - yc)]) - top
    half = np.abs(lifted @ axes[:, :2]).max(axis=0)  # the lifted footprint lies in the top face
    size = (2 * half[0] + 2 * TILE_OVERLAP, 2 * half[1] + 2 * TILE_OVERLAP, TILE_THICKNESS)
    return Tile(footprint, top, np.array([gx, gy]), axes, size)


def tops(tiles, x, y):
    """Height of the highest tile top above each layout point (x, y), NaN
    where there is none (vectorised over the points)."""
    x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
    if not tiles:
        return np.full(x.shape, np.nan)
    T = np.array([t.top for t in tiles])
    G = np.array([t.gradient for t in tiles])
    A = np.array([t.axes for t in tiles])
    H = np.array([t.size[:2] for t in tiles]) / 2
    px, py = x.reshape(-1, 1), y.reshape(-1, 1)  # points x tiles
    z = T[:, 2] + G[:, 0] * (px - T[:, 0]) + G[:, 1] * (py - T[:, 1])
    d = np.stack([px - T[:, 0], py - T[:, 1], z - T[:, 2]], axis=-1)
    local = np.einsum("nki,kij->nkj", d, A[:, :, :2])
    hit = np.all(np.abs(local) <= H + 1e-9, axis=-1)
    best = np.where(hit, z, -np.inf).max(axis=1).reshape(x.shape)
    return np.where(np.isfinite(best), best, np.nan)


def top_height(tiles, x, y):
    """Height of the highest tile top above layout (x, y), or None."""
    z = float(tops(tiles, x, y))
    return None if math.isnan(z) else z
