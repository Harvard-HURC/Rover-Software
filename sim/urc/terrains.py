"""Ground types and friction zones.

TYPES is the one catalogue of ground: every world picks its terrain layers,
zones, decals and paint rules (landscape.py) from it by name, and its
textures are generated once into urc_media and shared (TerrainType.texture).
ROCKS and SHRUBS hold the rock and shrub colours (WorldBuilder.rock_field,
WorldBuilder.shrubs) the same way.

Each type carries four groups of fields, the structure of the realism design
(docs/superpowers/specs/2026-10-06-urc-realism-design.md, sections 5.1-5.7;
numbers in brackets are its sources, section 15; (A) marks an assumption,
(M) a measurement, (T) a value measured elsewhere and transferred):
- Traction: what a wheel feels (design 5.6). The drivetrain takes mu as the
  Coulomb limit of the gross tyre force and applies rolling resistance and
  bulldozing as hub forces. Worlds write it to ground.json; DIG picks the
  dig-in preset they are written with (traction()).
- Appearance: the colour-map palette (design 5.7), detail texture and dust.
- Relief: micro-relief a synthetic world adds under the type (design 5.4,
  landscape.relief).
- Clutter: slabs, rocks, shrubs and risers placed on it (design 5.5,
  landscape.place).
Recipes are small frozen dataclasses that types share by reference. Every
world writes ground.png (one type index per heightmap sample) and
ground.json (legend, traction table, collision map) next to its heightmap:
the one record of what ground is where (landscape.Legend,
WorldBuilder.write).

Friction in Gazebo. Gazebo honours <friction> only on primitive collision
shapes (box, plane, ...): the heightmap and every mesh always have mu 1.0
(HEIGHTMAP_MU), and DART combines two touching shapes as min(mu_a, mu_b).
The physical drivetrain therefore sets the friction of every wheel contact
itself from ground.json (design D1: FRICTION = "ground"). Until it is the
rover's default, FRICTION = "tiles" keeps today's friction zones, where a
type's `mu` stands in for its traction and a zone with a lower mu has to be
the only thing its wheels touch:

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

A tile's mu is a tuning knob standing in for the whole traction (a rover
holds or climbs a slope up to about atan(mu)): today's zone types keep
theirs, and a type added since has its net traction mu_k - crr (bare rock
and objects 1.0, like slickrock). The tyres
have mu 1.0 along the tread and 0.5 across it (gen_model.Params), so a zone
mu above 1.0 changes nothing, and one above 0.5 only the grip along the
tread.
"""
import math
import zlib
from dataclasses import dataclass, field, replace

import numpy as np

from . import meshes, textures

HEIGHTMAP_MU = 1.0  # what Gazebo gives heightmaps and meshes, whatever the SDF says
FRICTION = "tiles"  # "tiles": friction zones are box tiles (above); "ground": zones only paint ground.png (design D1)
DIG = "strong"  # dig-in preset worlds are written with: "strong" (the catalogue's, the user's choice) or "mild"
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
DUST_RGB = (0.80, 0.70, 0.56)  # dust puff colour of the tuned particle emitter (design spec 6.5, M)


@dataclass(frozen=True)
class Traction:
    """How the ground grips and resists a wheel; the field names are the keys
    of ground.json's traction table (design spec 5.6, 9.1). mu is the
    Coulomb limit of the gross tyre force: static below the Stribeck speed,
    kinetic above it. Rolling resistance and bulldozing are hub forces
    (fractions of the wheel's load), not friction."""
    mu_s: float = 1.0  # static friction coefficient
    mu_k: float = 1.0  # kinetic friction coefficient
    crr: float = 0.0  # rolling resistance along the heading, x load
    bulldoze: float = 0.0  # sideways resistance in loose soil, along the axle, x load
    slip: float = 0.0  # force-dependent slip: steady slip ratio = slip x traction / load
    sinkage_m: float = 0.0  # [m] static sinkage (the collision carve under the type)
    dig_rate: float = 0.0  # [m/m] extra sinkage per metre of slip while a wheel spins
    dig_max: float = 1.0  # cap on the dig factor D = sinkage / static sinkage

    @classmethod
    def coulomb(cls, mu):
        """Plain Coulomb friction, nothing else: what DART gives a ground today."""
        return cls(mu_s=mu, mu_k=mu)

    @property
    def climb_deg(self):
        """Steepest slope a rover climbs: the net traction mu_k - crr (design 5.6)."""
        return math.degrees(math.atan(max(self.mu_k - self.crr, 0.0)))

    @property
    def hold_deg(self):
        """Steepest slope a parked rover holds with its wheels stopped: mu_s."""
        return math.degrees(math.atan(self.mu_s))


@dataclass(frozen=True)
class Palette:
    """A ground's colour [sRGB 0-255]: the median and, where measured, the
    10th and 90th percentile of its texels (design spec 5.7)."""
    base: tuple
    p10: tuple = None
    p90: tuple = None

    @classmethod
    def survey(cls, munsell, naip=None, spread=None):
        """Hue and chroma of the ground's dry Munsell colour from the soil
        survey (via the RIT renotation [2][28]), lightness of its NAIP median
        (design 5.7: NAIP is hazy and pale, the survey colour is the soil's
        own); spread: NAIP (p10, p50, p90) of a window of that ground, which
        moves p10 and p90 in lightness only. Without NAIP: the survey colour."""
        _, a, b = _lab(munsell)
        lightness = _lab(munsell if naip is None else naip)[0]
        if spread is None:
            return cls(_srgb((lightness, a, b)))
        p10, p50, p90 = (_lab(c)[0] for c in spread)
        return cls(*(_srgb((lightness + d, a, b)) for d in (0.0, p10 - p50, p90 - p50)))


SRGB_TO_XYZ = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]])
D65 = np.array([0.95047, 1.0, 1.08883])  # reference white (XYZ)
LAB_EPS = 6 / 29


def _lab(rgb):
    """CIE L*a*b* (D65) of an sRGB colour [0-255]."""
    c = np.asarray(rgb, float) / 255.0
    linear = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    t = SRGB_TO_XYZ @ linear / D65
    f = np.where(t > LAB_EPS ** 3, np.cbrt(t), t / (3 * LAB_EPS ** 2) + 4 / 29)
    return np.array([116 * f[1] - 16, 500 * (f[0] - f[1]), 200 * (f[1] - f[2])])


def _srgb(lab):
    """The sRGB colour [0-255] of CIE L*a*b* (D65), clipped to the gamut."""
    fy = (lab[0] + 16) / 116
    f = np.array([fy + lab[1] / 500, fy, fy - lab[2] / 200])
    linear = np.linalg.solve(SRGB_TO_XYZ, np.where(f > LAB_EPS, f ** 3, 3 * LAB_EPS ** 2 * (f - 4 / 29)) * D65)
    c = np.where(linear <= 0.0031308, 12.92 * linear, 1.055 * np.clip(linear, 0, None) ** (1 / 2.4) - 0.055)
    return tuple(int(v) for v in np.clip(np.round(c * 255), 0, 255))


@dataclass(frozen=True)
class Appearance:
    """How the ground looks: its colour-map palette, the shared detail
    texture laid over it (None: none) and the dust its wheels raise."""
    palette: Palette
    detail: str = None  # key of a shared detail texture in urc_media
    dust: float = 0.0  # dust factor: scales the emitters' rate behind the wheels (0: none)
    dust_rgb: tuple = DUST_RGB  # [0-1]


@dataclass(frozen=True)
class Haystacks:
    """Rounded badland knobs (a Heightfield op, design spec 5.4): each knob's
    steepest flank is drawn between min_flank_deg and flank_deg, so that a
    belt has a few steep flanks, not all."""
    diameter_m: tuple = (10.0, 40.0)  # (T: Mancos Shale near Hanksville [9])
    flank_deg: float = 37.5  # steepest flank, 35-40 deg (T [9])
    floor: float = 0.5  # fraction of a badland belt left flat (M: 47 % of the Badland-Rock outcrop unit < 5 deg)
    min_flank_deg: float = 22.5  # (A: tuned so that 6 % of a belt lies over 30 deg at 2 m, as measured (M: 5.9 %))


@dataclass(frozen=True)
class Rills:
    """Downslope rill traces carved into slopes (a Heightfield op, design spec 5.4)."""
    depth_m: tuple = (0.05, 0.20)  # deepening downslope (M: lidar rill lines; T: 7.9 cm mean [8])
    width_m: tuple = (0.5, 1.0)  # (A)
    spacing_m: tuple = (10.0, 30.0)  # between seeds (A)
    length_m: float = 60.0  # longest trace (A)
    min_slope_deg: float = 5.0  # a rill starts and runs only on steeper ground (A)


@dataclass(frozen=True)
class Relief:
    """Micro-relief a synthetic world adds to its macro shape under this
    type (design spec 5.4): a real lidar residual ("swatch",
    sim/data/relief/<swatch>.npz) scaled by amplitude, plus haystacks and
    rills. The default adds nothing."""
    swatch: str = None
    swatch_sigma_m: float = 0.0  # [m] the Gaussian high-pass the swatch was cut with
    amplitude: float = 0.0  # x the swatch's own heights
    haystacks: Haystacks = None
    rills: Rills = None


@dataclass(frozen=True)
class Slabs:
    """Tabular blocks (design spec 5.5): D drawn from tabulated cumulative
    counts, interpolated log-log, extrapolated below the table as
    N(>=D) = N(>=D_0) (D / D_0)^-exponent."""
    counts: tuple  # ((D [m], N(>=D) per 100 m2), ...) ascending in D
    d_min: float = 0.3  # [m] (A: below the 1 m the measurements resolve)
    d_max: float = 7.0  # [m] larger features are ledges or macro shape (M)
    exponent: float = 1.1  # local exponent at 1-2 m (M), used below the table (A)
    height: float = 0.4  # thickness / D (M: H/D of the DSM blocks [3]), each x U(0.7, 1.3) (A)
    max_tilt_deg: float = 30.0  # tilted 0 to this on the slope (A)


@dataclass(frozen=True)
class Rocks:
    """Rocks and cobbles: sizes log-uniform in sizes_m, density by the
    cumulative fractional area k (design spec 5.5, A: no local data)."""
    k: float
    sizes_m: tuple = (0.04, 0.3)


@dataclass(frozen=True)
class Shrubs:
    """Visual-only shrubs (design spec 5.5)."""
    per_ha: float
    height_m: tuple
    diameter_m: tuple = (0.4, 1.0)  # (A: NAIP's 0.6 m pixels cannot resolve it)


@dataclass(frozen=True)
class Risers:
    """Sub-metre ledges along contours (design spec 5.5, A: below lidar resolution)."""
    height_m: tuple = (0.1, 1.0)
    segment_m: tuple = (20.0, 40.0)
    spacing_m: tuple = (10.0, 30.0)
    depth_m: tuple = (1.0, 3.0)  # how far the strip reaches back into the ground (A)


@dataclass(frozen=True)
class Clutter:
    """What is placed on the ground (design spec 5.5); None: none of that kind."""
    slabs: Slabs = None
    rocks: Rocks = None
    shrubs: Shrubs = None
    risers: Risers = None


@dataclass(frozen=True)
class TerrainType:
    key: str
    title: str
    mu: float  # friction-zone tile mu (FRICTION = "tiles")
    rgb: tuple  # texture base colour
    pebbles: float = 0.001  # texture: pebbles per pixel
    variation: float = 0.25  # texture: mottling
    notes: str = ""
    traction: Traction = None  # default: Traction.coulomb(mu)
    appearance: Appearance = None  # default: the texture colour, no detail, no dust
    relief: Relief = Relief()
    clutter: Clutter = Clutter()

    def __post_init__(self):
        if self.traction is None:
            object.__setattr__(self, "traction", Traction.coulomb(self.mu))
        if self.appearance is None:
            object.__setattr__(self, "appearance", Appearance(Palette(tuple(self.rgb))))

    @property
    def max_slope_deg(self):
        """Steepest slope a parked or climbing rover holds on a friction-zone
        tile: tan(slope) = mu."""
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


# --- Sources of the recipes -------------------------------------------------------------------

# Colours (design 5.7): dry Munsell colours of the local soil series via the RIT renotation [2][28]
# (sim/data/research/mdrs_terrain_measurements.json, munsell_srgb), and NAIP medians [4]: 2024 per SSURGO
# map unit and landform (colour_stats.json), 2021 material classes and the sunlit (p10, p50, p90) of the
# lidar research windows (mdrs_terrain_measurements.json).
MUNSELL = {"sheppard": (194, 137, 95),  # Sheppard sand, 5YR 6/6
           "chipeta": (160, 148, 124),  # Chipeta shale soil, 2.5Y 6/2
           "leebench": (196, 172, 140),  # Leebench E horizon, 10YR 7/3 (the fan pavement colour too)
           "farb_a": (169, 145, 115),  # Farb A, 10YR 6/3: weathered sandstone
           "farb_c": (174, 144, 105),  # Farb C, 10YR 6/4: fresh sandstone
           "hanksville": (105, 97, 73)}  # Hanksville clay loam, 5Y 4/2
NAIP = {"sheppard": (226, 204, 179),  # map unit 55112 Sheppard-Leebench, MDRS area (#e2ccb3)
        "sheppard_leebench": (223, 200, 173),  # map unit 55112, route area
        "chipeta": (217, 210, 196),  # map unit 55149 Chipeta silty clay (#d9d2c4)
        "badland_rock": (222, 202, 185),  # map unit 55156 Badland-Rock outcrop, MDRS area
        "farb": (217, 192, 164),  # map unit 55162 Farb-Rock outcrop, MDRS area (#d9c0a4)
        "green_river": (224, 204, 183),  # map unit 55165 Green River-Myton, route area
        "billings": (222, 214, 201),  # map unit 55222 Billings silt loam (#ded6c9)
        "wash": (228, 208, 189),  # landform "wash", route area
        "mesa_top": (220, 201, 177),  # landform "mesa top", route area
        "plain": (221, 208, 193),  # landform "plain", MDRS area
        "steep": (215, 192, 170),  # landform "steep slope" (25-40 deg), route area
        "cliff": (219, 199, 179),  # landform "cliff" (> 40 deg), route area
        "maroon": (181, 157, 149),  # maroon mudstone band (2021)
        "grey_shale": (199, 196, 189),  # grey and grey-green shale (2021)
        "white": (215, 212, 206)}  # white bentonite, gypsum, splay sandstone (2021)
WINDOWS = {"sand_sheet_D": ((190, 178, 162), (199, 187, 171), (204, 193, 177)),  # NAIP 2021 sunlit p10, p50, p90
           "shale_pediment_A": ((174, 170, 161), (190, 185, 175), (201, 197, 189)),
           "badland_banded_B": ((176, 155, 147), (198, 186, 179), (212, 207, 202)),
           "slickrock_ledges_H": ((190, 180, 168), (201, 193, 180), (207, 200, 190)),
           "boulder_field_I": ((172, 159, 147), (197, 185, 171), (207, 197, 183))}

# Dig-in (design 5.6, 6.5, D21). The catalogue carries the strong preset, the user's choice (Q12): a
# sustained spin in loose ground digs in until the rover can no longer turn and has to drive out, as
# Team Anveshak's did at URC 2017 [14] (A: D_max 2.0, dig_rate 0.05; a spin stops once
# crr D c + bulldoze D^2 a >= mu_k c, near D 1.3-1.7 on sand, wash sand and powder; a crusted sand sheet
# slows but keeps turning). MILD_DIG is the mild preset (A: a spin slows to ~0.7x and keeps turning;
# [36] gives 3-7x the static sinkage at slip 0.6).
STRONG_DIG = (0.05, 2.0)  # (dig_rate [m/m], dig_max)
MILD_DIG = {"sand": (0.01, 1.25), "wash_sand": (0.012, 1.15), "sand_sheet": (0.005, 1.25), "clay": (0.008, 1.25)}

# Traction columns: mu_s, mu_k, crr, bulldoze, slip, sinkage_m, dig_rate, dig_max (design 5.6). Gross
# mu_k = 0.85 (DP/W + crr) for the Bekker soils, with Bekker for our wheel (W 113 N, D 0.30 m, b 0.10 m)
# [5] and a small-wheel derating of 0.85 (A, direction from [33]); car-tyre rolling resistance [6] (larger
# wheels, so lower bounds) brackets the firm grounds; mu_s = mu_k on loose ground, which has no peak [34];
# slip (A; a Janosi-Hanamoto bracket of 11-31 % on flat sand contains sand's 20 %, design 5.6).
ROCK_TRACTION = Traction(1.00, 0.85, 0.015, 0.0, 0.05, 0.0)  # rubber on rough sandstone 0.8-1.0 (A); concrete [6]
SCREE_TRACTION = Traction(0.46, 0.40, 0.06, 0.0, 0.4, 0.0)  # (A): a parked rover slides above ~25 deg
BADLAND_TRACTION = Traction(0.60, 0.50, 0.10, 0.0, 0.3, 0.01)  # loose granules over a firm crust (A, [10])
PAVEMENT_TRACTION = Traction(0.75, 0.62, 0.03, 0.0, 0.1, 0.0)  # crr between rolled (0.02) and worn gravel (0.04) [6]

# Relief (design 5.4): real 0.5 m lidar residuals (sim/data/relief, sim/tools/make_relief_swatches.py),
# their amplitudes tuned so that each type's median window RMS at 4-16 m lies in the real p25-p75 of that
# type (sim/data/research/terrain_targets.json; test_landscape.Roughness).
# The window RMS of each recipe on flat ground (test_landscape.Roughness) against the targets, p25-p75 [cm]:
SAND_RELIEF = Relief("sand_sheet", 16.0, 0.75)  # sand sheet: 4 m 1.55-4.08, 8 m 3.17-7.89, 16 m 5.47-15.26
LOOSE_SAND_RELIEF = Relief("sand_sheet", 16.0, 1.05)  # dune crests: 4 m 2.37-5.04, 8 m 6.10-9.89
WASH_RELIEF = Relief("sand_sheet", 16.0, 0.45)  # wash floors, smoother than the sheet (A)
PEDIMENT_RELIEF = Relief("shale_pediment", 16.0, 0.45, rills=Rills())  # 4 m 0.92-2.58, 8 m 1.92-5.86, 16 m
# 4.08-12.32
FLAT_RELIEF = Relief("shale_pediment", 16.0, 0.45)  # firm flats and crusts (A: the pediment's)
SILT_RELIEF = Relief("shale_pediment", 16.0, 0.52)  # badland floors: 4 m 1.10-3.19, 8 m 2.47-6.92, 16 m 5.14-13.90
BADLAND_RELIEF = Relief("badland", 2.0, 1.0, haystacks=Haystacks(), rills=Rills())  # 4 m 4.4-10.5, 8 m
# 13.3-28.0, 16 m 36.7-67.4; sigma 2 m: rills come from the rill op, downslope on the synthetic slope (a
# transferred one would point the wrong way)
BLOCK_RELIEF = Relief("block_field", 8.0, 1.0)  # 4 m 5.56-12.48, 8 m 14.4-29.0, 16 m 31.5-61.8 (provisional)
SCREE_RELIEF = Relief("block_field", 8.0, 0.5)  # (A)
SLICKROCK_RELIEF = Relief("slickrock", 2.0, 0.53)  # benches: 4 m 1.66-3.92, 8 m 3.24-6.97, 16 m 5.39-10.12

# Clutter (design 5.5). Slab counts N(>=D) per 100 m2 from DSM top-hat objects [3] (M,
# mdrs_terrain_measurements.json, dsm_tophat).
BLOCK_SLABS = Slabs(((1.0, 1.74), (1.5, 1.144), (2.0, 0.814), (3.0, 0.436), (4.0, 0.236), (5.0, 0.1405),
                     (7.0, 0.0705)))  # mean of windows I and J: 8.6 % cover by 1-7 m blocks
BADLAND_SLABS = Slabs(((1.0, 0.585), (1.5, 0.400), (2.0, 0.308), (3.0, 0.1845), (4.0, 0.1095), (5.0, 0.0695),
                       (7.0, 0.0355)), exponent=0.93)  # badland edges: mean of windows B and G, 3.3 % cover
RISERS = Risers()
SAND_SHRUBS = Shrubs(40.0, (0.15, 0.3))  # (M: 38-92 NAIP dark spots per ha in the route area, 53-68 % of
# them pass the NDVI-anomaly test (critique P); lidar vegetation over 25 cm covers 0.006 % of the plain)
WASH_SHRUBS = Shrubs(480.0, (0.5, 2.5), (0.6, 2.0))  # (M: lidar, wash cut-bank window: 7 % cover, p50 0.85 m,
# p95 2.5 m tall), at the mean area of the diameter range
SPARSE_GRAVEL = Rocks(0.02)  # (A: Leebench and Chipeta surfaces carry 5-20 % gravel, design 5.3, Q14)
COBBLES = Rocks(0.10)  # (A; Myton terraces: 23 % cobbles [2])


# --- The catalogue ----------------------------------------------------------------------------
# `mu`: today's friction-zone tiles (FRICTION = "tiles"); a type added since takes its net traction.
# Zone types: below mu 1.0 friction zones (tiles, see above); slickrock is a decal.
GRAVEL = TerrainType(
    "gravel", "Gravel", 0.6, (150, 128, 108), pebbles=0.06, variation=0.35,
    notes="Loose pebbles on packed ground: rolls under the wheels.",
    traction=Traction(0.62, 0.52, 0.05, 0.0, 0.3, 0.005),  # crr: loose worn gravel 0.04-0.08 [6]; mu (A)
    appearance=Appearance(Palette.survey(MUNSELL["farb_a"], NAIP["green_river"]), "gravel", 0.3),  # hue (A)
    relief=FLAT_RELIEF, clutter=Clutter(rocks=COBBLES))
SAND = TerrainType(
    "sand", "Soft sand", 0.4, (218, 186, 140), pebbles=0.002, variation=0.3,
    notes="Wind-blown or wash sand. Real sand also sinks and resists rolling; DART cannot, so low "
          "traction stands in for both.",
    # Bekker dry sand [5]: 0.85 x (0.41 + 0.20); bulldozing (A) between Rankine passive pressure (0.006) and
    # Bekker's bulldozing formula (0.11) at 2 cm sinkage [39][40], Sheppard's bulk density [2]
    traction=Traction(0.52, 0.52, 0.20, 0.06, 1.0, 0.02, *STRONG_DIG),
    appearance=Appearance(Palette.survey(MUNSELL["sheppard"], NAIP["sheppard"]), "ripples", 0.8),
    relief=LOOSE_SAND_RELIEF, clutter=Clutter(shrubs=SAND_SHRUBS))
SCREE = TerrainType(
    "scree", "Loose scree", 0.35, (138, 112, 94), pebbles=0.05, variation=0.45,
    notes="Loosely consolidated rock debris on steep slopes (URC 1.c.ii): slides from about 19 deg.",
    traction=SCREE_TRACTION,
    appearance=Appearance(Palette.survey(MUNSELL["farb_c"], NAIP["steep"]), None, 0.4),  # hue (A)
    relief=SCREE_RELIEF, clutter=Clutter(rocks=COBBLES))
CLAY = TerrainType(
    "clay", "Dusty bentonite clay", 0.25, (158, 156, 164), pebbles=0.0005, variation=0.6,
    notes="Morrison bentonite weathered to a 'popcorn' crust over powder: slippery, slides from "
          "about 14 deg.",
    # a loose fine layer: no peak [34]; a 4.8 cm pulverised mantle on disturbed slopes (T, Mancos [8]);
    # mu, crr and sinkage (A); dry powder, not the moist clay of [5]
    traction=Traction(0.45, 0.45, 0.15, 0.06, 0.5, 0.02, *STRONG_DIG),
    appearance=Appearance(Palette(NAIP["grey_shale"]), "popcorn", 1.0),
    relief=FLAT_RELIEF)
SLICKROCK = TerrainType(
    "slickrock", "Slickrock sandstone slab", 1.0, (222, 184, 140), pebbles=0.0,
    notes="Bare cemented sandstone: the best grip there is (the tyres' own mu 1.0 limits it); "
          "smooth, so steep slabs are climbable. A decal only: the heightmap has mu 1.0 too.",
    traction=ROCK_TRACTION,
    appearance=Appearance(Palette.survey(MUNSELL["farb_a"], NAIP["farb"], WINDOWS["slickrock_ledges_H"]),
                          "slab_joints", 0.1),
    relief=SLICKROCK_RELIEF, clutter=Clutter(risers=RISERS))
# Ground that looks different but grips like the heightmap (mu 1.0): terrain
# layers and decals.
REGOLITH = TerrainType(
    "regolith", "Packed regolith", 1.0, (190, 150, 115),
    notes="The default ground: the heightmap itself, mu 1.0 (Gazebo ignores mu on heightmaps).",
    traction=Traction(0.62, 0.52, 0.10, 0.0, 0.3, 0.005),  # Bekker sandy loam [5]: 0.85 x (0.51 + 0.10); mu_s (A)
    appearance=Appearance(Palette.survey(MUNSELL["leebench"], NAIP["sheppard_leebench"]), None, 0.4),
    relief=SAND_RELIEF, clutter=Clutter(rocks=SPARSE_GRAVEL))  # relief: Leebench is packed sand sheet (A, Q14)
PAVEMENT = TerrainType(
    "pavement", "Desert pavement", 1.0, (166, 122, 93), pebbles=0.01,
    notes="Packed ground armoured with small stones, between the washes and the hills.",
    traction=PAVEMENT_TRACTION,  # a visual type: fan pavement's traction (design 5.6)
    appearance=Appearance(Palette.survey(MUNSELL["leebench"], NAIP["plain"]), "gravel", 0.3),
    relief=FLAT_RELIEF, clutter=Clutter(rocks=COBBLES))
MUDSTONE = TerrainType(
    "mudstone", "Maroon mudstone", 1.0, (138, 82, 70),
    notes="Morrison-like maroon and purple mudstone bands on the hills.",
    traction=BADLAND_TRACTION,  # a visual type: badland slope's traction (design 5.6)
    appearance=Appearance(Palette(NAIP["maroon"], (160, 138, 135), (195, 170, 161)), "popcorn", 0.7),
    relief=BADLAND_RELIEF, clutter=Clutter(slabs=BADLAND_SLABS))
BENTONITE = TerrainType(
    "bentonite", "Grey bentonitic mudstone", 1.0, (148, 148, 158),
    notes="Grey-blue bentonite bands on the hills; where it weathers to powder it is CLAY.",
    traction=BADLAND_TRACTION,  # a visual type: badland slope's traction (design 5.6)
    appearance=Appearance(Palette(NAIP["grey_shale"]), "popcorn", 0.7),
    relief=BADLAND_RELIEF, clutter=Clutter(slabs=BADLAND_SLABS))
CAPROCK = TerrainType(
    "caprock", "Sandstone caprock", 1.0, (212, 194, 156),
    notes="Cemented sandstone capping the heights.",
    traction=ROCK_TRACTION,
    appearance=Appearance(Palette.survey(MUNSELL["farb_c"], NAIP["mesa_top"]), "slab_joints", 0.1),
    relief=SLICKROCK_RELIEF, clutter=Clutter(risers=RISERS))
BIOCRUST = TerrainType(
    "biocrust", "Biological soil crust", 1.0, (72, 60, 50), pebbles=0.004, variation=0.5,
    notes="Dark knobbly cyanobacteria and lichen crust on stable flats.",
    traction=Traction(0.62, 0.52, 0.12, 0.0, 0.3, 0.005),  # as loam; pinnacles (<= 10 cm [11]) crush (A)
    appearance=Appearance(Palette((72, 60, 50)), None, 0.2),  # today's colour (A)
    relief=SAND_RELIEF)
GYPSUM = TerrainType(
    "gypsum", "Gypsum crust", 1.0, (216, 210, 196), pebbles=0.004, variation=0.5,
    notes="White evaporite crust on a low mound.",
    traction=Traction(0.72, 0.60, 0.06, 0.0, 0.1, 0.0),  # (A)
    appearance=Appearance(Palette(NAIP["white"]), None, 0.6),
    relief=FLAT_RELIEF)
# Types of the realism design (5.3, 5.6), for paint rules and the soil map.
SAND_SHEET = TerrainType(
    "sand_sheet", "Crusted sand sheet", 0.40, (237, 176, 132), pebbles=0.003, variation=0.3,
    notes="Sheppard sand between the shrubs, under a thin crust: a little firmer than loose sand.",
    traction=Traction(0.60, 0.55, 0.15, 0.03, 0.6, 0.015, *STRONG_DIG),  # between sand and loam (A); a crust
    # gives a small peak (A, [34])
    appearance=Appearance(Palette.survey(MUNSELL["sheppard"], WINDOWS["sand_sheet_D"][1], WINDOWS["sand_sheet_D"]),
                          "ripples", 0.6),
    relief=SAND_RELIEF, clutter=Clutter(rocks=SPARSE_GRAVEL, shrubs=SAND_SHRUBS))
WASH_SAND = TerrainType(
    "wash_sand", "Wash sand", 0.27, (255, 198, 154), pebbles=0.004, variation=0.3,
    notes="Loose sand on wash floors (Riverwash: 98 % sand in the top 15 cm): the loosest ground there is.",
    traction=Traction(0.52, 0.52, 0.25, 0.10, 1.2, 0.03, *STRONG_DIG),  # looser than sand (A); URC teams stuck
    # in sand [14][32]
    appearance=Appearance(Palette.survey(MUNSELL["sheppard"], NAIP["wash"]), "ripples", 0.6),  # hue (A)
    relief=WASH_RELIEF, clutter=Clutter(rocks=SPARSE_GRAVEL, shrubs=WASH_SHRUBS))
FAN_PAVEMENT = TerrainType(
    "fan_pavement", "Fan gravel pavement", 0.59, (196, 172, 140), pebbles=0.08, variation=0.35,
    notes="A gravel lag armouring an alluvial fan; designed patches only (design 5.3, Q14).",
    traction=PAVEMENT_TRACTION,
    appearance=Appearance(Palette.survey(MUNSELL["leebench"]), "gravel", 0.3),
    relief=FLAT_RELIEF, clutter=Clutter(rocks=COBBLES))
CLAY_CRUST = TerrainType(
    "clay_crust", "Shale pediment crust", 0.57, (223, 209, 184), pebbles=0.02, variation=0.3,
    notes="Dry crust of gravelly silty clay on the grey shale pediment (Chipeta).",
    traction=Traction(0.78, 0.65, 0.08, 0.0, 0.1, 0.0),  # crr: medium-hard soil 0.04-0.08 [6]; dry crust mu (A)
    appearance=Appearance(Palette.survey(MUNSELL["chipeta"], NAIP["chipeta"], WINDOWS["shale_pediment_A"]),
                          "gravel", 0.5),  # its A horizon is 30 % gravel [2]
    relief=PEDIMENT_RELIEF, clutter=Clutter(rocks=SPARSE_GRAVEL))
BADLAND_SLOPE = TerrainType(
    "badland_slope", "Badland slope", 0.40, (225, 200, 192), pebbles=0.002, variation=0.45,
    notes="Bentonitic mudstone slopes under a popcorn crust: rounded knobs, rills and maroon, grey and white bands.",
    traction=BADLAND_TRACTION,
    appearance=Appearance(Palette.survey(NAIP["maroon"], NAIP["badland_rock"], WINDOWS["badland_banded_B"]),
                          "popcorn", 0.7),  # hue: maroon 10R-5YR (A)
    relief=BADLAND_RELIEF, clutter=Clutter(slabs=BADLAND_SLABS))
SILT_FLAT = TerrainType(
    "silt_flat", "Silt flat", 0.56, (225, 214, 187), pebbles=0.0005, variation=0.3,
    notes="Silt and clay flats on badland floors (Billings, Hanksville): firm when dry.",
    traction=Traction(0.74, 0.62, 0.06, 0.0, 0.1, 0.0),  # crr 0.04-0.08 [6]; mu (A)
    appearance=Appearance(Palette.survey(MUNSELL["hanksville"], NAIP["billings"]), "cracked_silt", 0.9),
    relief=SILT_RELIEF)
ROCK = TerrainType(
    "rock", "Rock face", 1.0, (230, 197, 156), pebbles=0.0, variation=0.35,
    notes="Faces too steep for soil (over 30 deg), and every rock, slab, riser and step by the collision map.",
    traction=ROCK_TRACTION,
    appearance=Appearance(Palette.survey(MUNSELL["farb_c"], NAIP["cliff"]), "slab_joints", 0.1))
BLOCK_FIELD = TerrainType(
    "block_field", "Block field", 0.34, (214, 182, 141), pebbles=0.03, variation=0.45,
    notes="Talus of tabular sandstone blocks under the rims; the ground between them is loose debris.",
    traction=SCREE_TRACTION,  # the debris (A); the blocks are rock (collision map)
    appearance=Appearance(Palette.survey(MUNSELL["farb_c"], WINDOWS["boulder_field_I"][1],
                                         WINDOWS["boulder_field_I"]), None, 0.4),  # hue (A)
    relief=BLOCK_RELIEF, clutter=Clutter(slabs=BLOCK_SLABS, rocks=Rocks(0.05)))
MANMADE = TerrainType(
    "manmade", "Man-made surface", 1.0, (128, 128, 128), pebbles=0.0, variation=0.1,
    notes="Objects whose SDF sets no friction: the landing pad, the C2 pad, the lander.",
    traction=Traction(0.80, 0.70, 0.015, 0.0, 0.05, 0.0),  # rubber on dry concrete 1.0 / 0.7 [23], lowered for
    # painted wood and plastic (A)
    appearance=Appearance(Palette((128, 128, 128))))  # (A) never painted on the ground
TYPES = {t.key: t for t in (GRAVEL, SAND, SCREE, CLAY, SLICKROCK, REGOLITH, PAVEMENT, MUDSTONE, BENTONITE, CAPROCK,
                            BIOCRUST, GYPSUM, SAND_SHEET, WASH_SAND, FAN_PAVEMENT, CLAY_CRUST, BADLAND_SLOPE,
                            SILT_FLAT, ROCK, BLOCK_FIELD, MANMADE)}
DEFAULT_GROUND = "regolith"  # what a world's ground is where nothing else is painted
OBJECT_SURFACE = "manmade"  # objects whose SDF sets no friction (ground.json object_default)
TERRAIN_SURFACE = "rock"  # terrain-model shapes the collision map does not name (ground.json terrain_default)

# Rock colours by palette: rock_field gives each rock one of its palette's.
ROCKS = {"desert": ((0.47, 0.33, 0.25), (0.55, 0.45, 0.36), (0.42, 0.40, 0.40), (0.60, 0.50, 0.38),
                    (0.38, 0.27, 0.22)),
         "lichen": ((0.62, 0.64, 0.32),),  # epilithic lichen covering boulders
         # Slabs (design 5.5): varnished dark tops (105-136, 100-116, 101-116) (M) and fresh tan faces,
         # 10YR 6/3 [2][28]; VARNISHED of them varnished (A).
         "varnish": ((105 / 255, 100 / 255, 101 / 255), (120 / 255, 108 / 255, 108 / 255),
                     (136 / 255, 116 / 255, 116 / 255)),
         "fresh_sandstone": ((169 / 255, 145 / 255, 115 / 255),)}
VARNISHED = 0.7
SHRUBS = ((0.42, 0.47, 0.33), (0.37, 0.42, 0.29), (0.47, 0.49, 0.38))  # sage, dark and grey-green brush


def traction(kind, dig=None):
    """The traction a world writes for `kind` (ground.json) under the dig-in
    preset `dig` (default DIG): "strong", the catalogue's, or "mild"."""
    dig = dig or DIG
    if dig not in ("strong", "mild"):
        raise ValueError(f"dig-in preset {dig!r}: 'strong' or 'mild'")
    if dig == "mild" and kind.key in MILD_DIG:
        rate, cap = MILD_DIG[kind.key]
        return replace(kind.traction, dig_rate=rate, dig_max=cap)
    return kind.traction


def calibration_surface(mu):
    """A plain test surface of a given mu (proving-ground lanes), coloured
    from pale blue-grey (slippery) to dark (grippy): Coulomb traction, nothing
    else (design 5.6: exact calibration)."""
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
