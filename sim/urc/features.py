"""Terrain features: parameterised building blocks any world uses.

A feature acts twice. shape(hf) edits the heightfield before the
WorldBuilder exists (the world frame depends on the finished terrain);
dress(w) adds what lies on it: its zones (terrains.py: they paint the
ground raster, which says how the ground grips) and blocks (ground type rock
in ground.json). Features with zones also say roughly
where they will paint before they are dressed (footprints(): a zone's
irregular outline is drawn when it is dressed), and engineered features name
the ground micro-relief must leave alone (keep_flat(), landscape.keep_flat).
A world lists its features, saying only where and how big:

    FEATURES = [features.Patch("sand_flat", terrains.SAND, 150, 62, 14), ...]
    features.shape(hf, FEATURES)    # in make_terrain()
    features.dress(w, FEATURES)     # after w.terrain(), before placing objects on them

Features are frozen dataclasses of their parameters, so shaping and dressing
see the same feature; dressing in list order keeps a world's zones (and the
random outlines they draw) the same. Rocks are WorldBuilder.scatter /
scatter_each / rock_garden / rock_field calls; the heightfield ops themselves
(noise, mesa, ramp, channel, ridge, level, strip, bump, washboard) are
terrain.Heightfield methods.
"""
import math
from dataclasses import dataclass

from . import terrain, terrains
from .terrains import TerrainType

STEP_COLOR = (0.55, 0.47, 0.40)  # weathered sandstone
STEP_BURY = 0.3  # [m] a step reaches this far below the ground
LEDGE_COLOR = (0.58, 0.40, 0.30)  # a shelf of reddish sandstone
LEDGE_LIP_COLOR = (0.5, 0.35, 0.27)
LEDGE_LIP = (0.3, 0.06)  # [m] depth, height of the lip along the drop's edge
LEDGE_BACK = 3.0  # [m] level ground behind a ledge's shelf, to drive onto it
LEDGE_APRON = 4.0  # [m] level ground in front of a ledge's drop, to land on


def shape(hf, features):
    for feature in features:
        feature.shape(hf)
    return hf


def dress(w, features):
    for feature in features:
        feature.dress(w)


def _axis_point(start, yaw, u, v):
    """Layout (x, y) of the point `u` metres from start along yaw and `v` to its left."""
    c, s = math.cos(yaw), math.sin(yaw)
    return start[0] + c * u - s * v, start[1] + s * u + c * v


def _toward(a, b, distance):
    """The point `distance` metres from a towards b."""
    d = math.dist(a, b)
    return a[0] + (b[0] - a[0]) * distance / d, a[1] + (b[1] - a[1]) * distance / d


def _rect(start, yaw, u0, u1, v0, v1):
    """Layout corners of the rectangle u0..u1 along yaw from start, v0..v1 to its left."""
    return [_axis_point(start, yaw, u, v) for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]


def _disc(x, y, radius, count=64):
    """Layout polygon of a circle."""
    return [(x + radius * math.cos(a), y + radius * math.sin(a))
            for a in (2 * math.pi * k / count for k in range(count))]


@dataclass(frozen=True)
class Patch:
    """A natural zone of `kind`: an irregular patch up to `radius` around
    (x, y) on the ground as it is (a zone only paints the ground raster)."""
    key: str
    kind: TerrainType
    x: float
    y: float
    radius: float
    irregularity: float = 0.3

    def shape(self, hf):
        pass

    def footprints(self):
        """Its zone as a disc of the mean edge radius of its outline (meshes.blob_outline)."""
        return [(_disc(self.x, self.y, self.radius * (1 - self.irregularity / 2)), self.kind)]

    def dress(self, w):
        w.zone(self.key, self.kind, self.x, self.y, self.radius, self.irregularity)


def along(key, kind, path, step, radius, skip=()):
    """Patches every `step` metres (at most) along a polyline, not at its
    ends: sand on a wash floor. skip: indices along the path to leave out."""
    points = terrain.resample(path, step)[1:-1]
    return [Patch(f"{key}_{k}", kind, float(x), float(y), radius) for k, (x, y) in enumerate(points)
            if k not in skip]


@dataclass(frozen=True)
class Wash:
    """A dry wash along a polyline: a channel `depth` deep (0 for one the
    terrain already has, as on a real DEM) whose floor has soft sand patches
    `<key>_sand_<k>` every `sand_step` metres (along), but not at the indices
    in `skip` (where it cuts through a ridge, say). A channel's sand stays on
    its floor, within `half_width` of the path, not up its banks."""
    key: str
    path: tuple
    depth: float = 0.0
    half_width: float = 8.0
    falloff: float = 6.0
    sand_step: float = 60.0
    sand_radius: float = 6.5
    skip: tuple = ()

    @property
    def patches(self):
        return along(f"{self.key}_sand", terrains.SAND, self.path, self.sand_step, self.sand_radius, self.skip)

    def shape(self, hf):
        if self.depth:
            if self.sand_radius > self.half_width:
                raise ValueError(f"{self.key}: sand patches of radius {self.sand_radius} m reach past the "
                                 f"{self.half_width} m half-width of the wash floor")
            hf.channel(self.path, self.depth, self.half_width, self.falloff)

    def footprints(self):
        return [f for patch in self.patches for f in patch.footprints()]

    def dress(self, w):
        dress(w, self.patches)


@dataclass(frozen=True)
class Mesa:
    """A flat-topped hill (terrain.Heightfield.mesa): `height` above the
    ground within an irregular `radius` of (x, y), falling off over `cliff`
    metres."""
    key: str
    x: float
    y: float
    radius: float
    height: float
    cliff: float
    seed: int
    irregularity: float = 0.15

    def shape(self, hf):
        hf.mesa(self.x, self.y, self.radius, self.height, self.cliff, self.seed, self.irregularity)

    def dress(self, w):
        pass


@dataclass(frozen=True)
class Slope:
    """A planar slope of `kind` from `foot` up to `top`, `width` wide, cut or
    filled into a hillside so that its top meets the ground `run_on` metres
    beyond (a scree chute up a mesa)."""
    key: str
    kind: TerrainType
    foot: tuple
    top: tuple
    width: float
    run_on: float = 6.0
    falloff: float = 5.0

    def shape(self, hf):
        behind = _toward(self.top, self.foot, -self.run_on)
        hf.ramp([self.foot, self.top], half_width=self.width / 2 + 1.5, falloff=self.falloff,
                z_end=hf.height(*behind))

    def keep_flat(self):
        yaw = math.atan2(self.top[1] - self.foot[1], self.top[0] - self.foot[0])
        half = self.width / 2 + 1.5
        return [_rect(self.foot, yaw, 0.0, math.dist(self.foot, self.top) + self.run_on, -half, half)]

    def footprints(self):
        yaw = math.atan2(self.top[1] - self.foot[1], self.top[0] - self.foot[0])
        return [(_rect(self.foot, yaw, 0.0, math.dist(self.foot, self.top), -self.width / 2, self.width / 2),
                 self.kind)]

    def dress(self, w):
        (fx, fy), (tx, ty) = self.foot, self.top
        w.zone_rect(self.key, self.kind, (fx + tx) / 2, (fy + ty) / 2, math.dist(self.foot, self.top), self.width,
                    math.atan2(ty - fy, tx - fx))


@dataclass(frozen=True)
class Surface:
    """A zone of `kind` along part of a Lane: `width` wide, its axis
    `offset` metres to the left of the lane's, over the lane's first
    `length` metres (default all)."""
    key: str
    kind: TerrainType
    width: float
    offset: float = 0.0
    length: float = None


@dataclass(frozen=True)
class Lane:
    """An engineered strip (terrain.Heightfield.strip): straight from `start`
    along `yaw`, `width` wide, its profile `segments` [(length [m], grade
    [deg])] climbing from the ground at the start (or z0), tilted `cross` deg
    across (left side up), blended into the terrain over `falloff`.
    surfaces: zones laid along it, their decals creased at the profile's kinks."""
    key: str
    start: tuple
    yaw: float
    width: float
    segments: tuple
    surfaces: tuple = ()
    cross: float = 0.0
    falloff: float = 3.0
    z0: float = None

    @property
    def length(self):
        return sum(length for length, _ in self.segments)

    def describe(self, length=None):
        """The profile in words, over the first `length` metres (default all)."""
        out, done = [], 0.0
        for run, grade in self.segments:
            if length is not None and done >= length - 1e-6:
                break
            done += run
            out.append(f"{run:.0f} m flat" if grade == 0 else f"{run:.0f} m {'up' if grade > 0 else 'down'} "
                       f"at {abs(grade):.0f} deg")
        return ", ".join(out)

    def at(self, u, v=0.0):
        """Layout (x, y) of the point `u` metres along the axis and `v` to its left."""
        return _axis_point(self.start, self.yaw, u, v)

    def shape(self, hf):
        hf.strip(self.start, self.yaw, self.width, self.segments, self.falloff, self.z0, self.cross)

    def keep_flat(self):
        return [_rect(self.start, self.yaw, 0.0, self.length, -self.width / 2, self.width / 2)]

    def footprints(self):
        return [(_rect(self.start, self.yaw, 0.0, s.length or self.length, s.offset - s.width / 2,
                       s.offset + s.width / 2), s.kind) for s in self.surfaces]

    def dress(self, w):
        for surface in self.surfaces:
            length = surface.length or self.length
            breaks = [k for k in terrain.kinks(self.segments) if k < length - 1e-6]
            w.zone_rect(surface.key, surface.kind, *self.at(length / 2, surface.offset), length, surface.width,
                        self.yaw, breaks)


@dataclass(frozen=True)
class Washboard:
    """Corrugated ground (terrain.Heightfield.washboard): crests across a
    strip from `start` along yaw."""
    key: str
    start: tuple
    yaw: float
    length: float
    width: float
    amplitude: float = 0.04
    wavelength: float = 0.8

    def shape(self, hf):
        hf.washboard(self.start, self.yaw, self.length, self.width, self.amplitude, self.wavelength)

    def keep_flat(self):
        return [_rect(self.start, self.yaw, 0.0, self.length, -self.width / 2, self.width / 2)]

    def dress(self, w):
        pass


@dataclass(frozen=True)
class AlternatingBumps:
    """Single-side humps along a track from `start` along yaw: for each of
    `heights`, one under the left wheels, then one under the right, `spacing`
    metres apart; `offset`: the wheel track's distance from the axis. Each
    hump is 1 + 2 h long, so the higher ones are not steeper walls."""
    key: str
    start: tuple
    yaw: float
    heights: tuple
    spacing: float
    offset: float
    width: float = 0.7

    def humps(self):
        """(along, left offset, height) of every hump."""
        return [(self.spacing * (2 * k + side + 1), self.offset * (1 - 2 * side), h)
                for k, h in enumerate(self.heights) for side in (0, 1)]

    def at(self, u, v=0.0):
        return _axis_point(self.start, self.yaw, u, v)

    def shape(self, hf):
        for u, v, h in self.humps():
            hf.bump(*self.at(u, v), h, 1 + 2 * h, self.width, self.yaw)

    def keep_flat(self):
        reach = self.offset + self.width / 2
        return [_rect(self.start, self.yaw, 0.0, self.spacing * (2 * len(self.heights) + 1), -reach, reach)]

    def dress(self, w):
        pass


@dataclass(frozen=True)
class TwistDitch:
    """A straight ditch crossing a track diagonally at (x, y): turned `angle`
    from the direction of travel yaw, so that it takes a front wheel on one
    side and the rear wheel on the other at once and twists the rockers
    against each other (atan(track / wheelbase) does exactly that). Its flat
    floor is wide enough for a wheel to reach: a smooth V of the same depth
    twisted the rockers half as far."""
    key: str
    x: float
    y: float
    yaw: float
    angle: float
    depth: float
    length: float = 4.0
    half_width: float = 0.25
    falloff: float = 0.5

    def shape(self, hf):
        a = self.yaw + self.angle
        dx, dy = math.cos(a) * self.length / 2, math.sin(a) * self.length / 2
        hf.channel([(self.x - dx, self.y - dy), (self.x + dx, self.y + dy)], self.depth, self.half_width,
                   self.falloff)

    def keep_flat(self):
        reach = self.half_width + self.falloff
        return [_rect((self.x, self.y), self.yaw + self.angle, -self.length / 2, self.length / 2, -reach, reach)]

    def dress(self, w):
        pass


@dataclass(frozen=True)
class Step:
    """A flat-topped block `top` metres high (WorldBuilder.block), `length`
    along yaw: driving along yaw it is an up-step, then a drop-off."""
    key: str
    x: float
    y: float
    yaw: float
    length: float
    width: float
    top: float

    def shape(self, hf):
        pass

    def keep_flat(self):
        return [_rect((self.x, self.y), self.yaw, -self.length / 2, self.length / 2, -self.width / 2, self.width / 2)]

    def dress(self, w):
        w.block(self.key, self.x, self.y, self.yaw, (self.length, self.width, self.top + STEP_BURY),
                w.height(self.x, self.y) + self.top, STEP_COLOR)


@dataclass(frozen=True)
class Ledge:
    """A rock shelf with a vertical drop of `drop` metres (rule 1.c.ii) whose
    edge runs across (x, y), facing yaw. shape() steps the ground down by
    `drop` at the edge: level with the edge for `depth` + LEDGE_BACK metres
    behind it, `drop` lower for LEDGE_APRON metres in front, `width` wide,
    blended back into the terrain over `falloff`. A heightfield cannot hold
    a vertical face, so its slope down the step lies under the shelf: a
    block `depth` metres back from the edge, its top level with the ground
    behind it, its front face the drop, with a lip along the edge. Rocks
    keep clear of the edge as of a placed model."""
    key: str
    x: float
    y: float
    yaw: float
    drop: float
    width: float = 6.0
    depth: float = 4.0
    falloff: float = 4.0

    def at(self, u):
        """Layout (x, y) of the point `u` metres from the edge along yaw (forward, over the drop)."""
        return _axis_point((self.x, self.y), self.yaw, u, 0.0)

    def shape(self, hf):
        # Ground more than a cell diagonal from the slope down the step is
        # level (the surface is bilinear between samples): in front of the
        # edge it is the lower level, behind the shelf the upper one.
        cell = hf.res * math.sqrt(2)
        slope = self.depth - 2 * cell
        if slope <= 0:
            raise ValueError(f"{self.key}: on a {hf.res} m grid a ledge's shelf must be over {2 * cell:.2f} m deep")
        back = LEDGE_BACK + self.depth
        hf.strip(self.at(-back), self.yaw, self.width,
                 ((LEDGE_BACK + cell, 0.0), (slope, -math.degrees(math.atan(self.drop / slope))),
                  (cell + LEDGE_APRON, 0.0)), self.falloff, z0=hf.height(self.x, self.y))

    def keep_flat(self):
        return [_rect((self.x, self.y), self.yaw, -(LEDGE_BACK + self.depth), LEDGE_APRON, -self.width / 2,
                      self.width / 2)]

    def dress(self, w):
        top = w.height(*self.at(-self.depth))  # the level ground behind the shelf
        lip_depth, lip_height = LEDGE_LIP
        for part, back, size, z, color in (
                ("shelf", self.depth, (self.depth, self.width, self.drop + 0.5), top, LEDGE_COLOR),
                ("lip", lip_depth, (lip_depth, self.width, lip_height), top + lip_height, LEDGE_LIP_COLOR)):
            w.block(f"{self.key}_{part}", *self.at(-back / 2), self.yaw, size, z, color)
        w.mark(self.key, self.x, self.y)
