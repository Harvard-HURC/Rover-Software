"""Heightfields: procedural desert terrain, queries, and export for Gazebo.

A Heightfield is a square grid of n x n heights z[row, col] covering
size x size metres around `center` (world x east, y north). Row 0 is the
north edge (+y) and column 0 the west edge (-x), the same layout as a Gazebo
heightmap image, so write_png() can hand the grid straight to Gazebo (with
the heightmap's <pos> at the center). All coordinates in the API are world
coordinates.

Synthesis ops edit z in place and return self, so terrain reads as a chain:

    hf = Heightfield(640, 1025).noise(...).mesa(...).ramp(...)
"""
import math

import cv2
import numpy as np
from PIL import Image


def smoothstep(edge0, edge1, x):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def resample(path, step=0.5):
    """Points every `step` metres (at most) along a polyline of (x, y), its
    corners included."""
    path = np.asarray(path, float)
    out = [path[0]]
    for p, q in zip(path[:-1], path[1:]):
        count = max(1, int(np.ceil(np.linalg.norm(q - p) / step)))
        out += [p + (q - p) * k / count for k in range(1, count + 1)]
    return np.array(out)


def path_distance(path, x, y, closed=False):
    """Distance from points (x, y) to a polyline (to a polygon's edges if
    closed), and the arc length [m] of the nearest point on it."""
    P = np.stack([np.asarray(x, float), np.asarray(y, float)], axis=-1)
    path = np.asarray(path, float)
    if closed:
        path = np.concatenate([path, path[:1]])
    best = np.full(P.shape[:-1], np.inf)
    along = np.zeros(P.shape[:-1])
    start = 0.0
    for p, q in zip(path[:-1], path[1:]):
        d = q - p
        length = float(np.linalg.norm(d))
        t = np.clip(((P - p) @ d) / max(length * length, 1e-12), 0, 1)
        dist = np.linalg.norm(P - (p + t[..., None] * d), axis=-1)
        closer = dist < best
        best = np.where(closer, dist, best)
        along = np.where(closer, start + t * length, along)
        start += length
    return best, along


def kinks(segments):
    """Distances from the start of a profile [(length, grade_deg)] to where
    its grade changes."""
    return list(np.cumsum([length for length, _ in segments])[:-1])


def fbm(n, size, feature, seed, octaves=4, persistence=0.5):
    """Fractal value noise on an n x n grid spanning size metres, roughly in
    [-1, 1]. feature is the wavelength of the first octave [m]."""
    rng = np.random.default_rng(seed)
    total = np.zeros((n, n))
    amplitude, norm = 1.0, 0.0
    for k in range(octaves):
        cells = max(2, int(math.ceil(size / (feature / 2**k))) + 1)
        grid = rng.uniform(-1, 1, (cells, cells)).astype(np.float32)
        total += amplitude * cv2.resize(grid, (n, n), interpolation=cv2.INTER_CUBIC)
        norm += amplitude
        amplitude *= persistence
    return total / norm


class Heightfield:
    def __init__(self, size, n, z=None, center=(0.0, 0.0)):
        self.size = float(size)
        self.n = int(n)
        self.center = (float(center[0]), float(center[1]))
        self.z = np.zeros((n, n)) if z is None else np.asarray(z, dtype=float)

    @property
    def res(self):
        return self.size / (self.n - 1)

    def grid(self):
        """World coordinates of every sample: X[row, col], Y[row, col]."""
        c = np.linspace(-self.size / 2, self.size / 2, self.n)
        return np.meshgrid(c + self.center[0], c[::-1] + self.center[1])

    # --- Queries -------------------------------------------------------------

    def height(self, x, y):
        """Bilinear height at world (x, y); clamps outside the grid."""
        col = (np.asarray(x, dtype=float) - self.center[0] + self.size / 2) / self.res
        row = (self.size / 2 - np.asarray(y, dtype=float) + self.center[1]) / self.res
        col = np.clip(col, 0, self.n - 1.000001)
        row = np.clip(row, 0, self.n - 1.000001)
        c0, r0 = np.floor(col).astype(int), np.floor(row).astype(int)
        fc, fr = col - c0, row - r0
        z = self.z
        h = (z[r0, c0] * (1 - fc) * (1 - fr) + z[r0, c0 + 1] * fc * (1 - fr)
             + z[r0 + 1, c0] * (1 - fc) * fr + z[r0 + 1, c0 + 1] * fc * fr)
        return float(h) if np.ndim(h) == 0 else h

    def slope_deg(self, x, y):
        """Steepest slope at (x, y) [deg]."""
        d = self.res
        gx = (self.height(x + d, y) - self.height(x - d, y)) / (2 * d)
        gy = (self.height(x, y + d) - self.height(x, y - d)) / (2 * d)
        return np.degrees(np.arctan(np.hypot(gx, gy)))

    def slope_map(self):
        """Steepest slope at every sample [deg]."""
        gy, gx = np.gradient(self.z, self.res)
        return np.degrees(np.arctan(np.hypot(gx, gy)))

    def line_of_sight(self, a, b, step=0.5):
        """True if the segment between world points a and b (x, y, z) clears the terrain."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        count = max(2, int(np.linalg.norm(b[:2] - a[:2]) / step) + 1)
        t = np.linspace(0, 1, count)[1:-1]
        p = a[None, :] + t[:, None] * (b - a)[None, :]
        return bool(np.all(p[:, 2] >= self.height(p[:, 0], p[:, 1])))

    def max_grade_along(self, path, step=0.5, base=2.0):
        """Steepest climb or descent along a polyline [deg], over `base` metre spans
        (about a rover length, so single-cell noise does not count)."""
        pts = resample(path, step)
        h = self.height(pts[:, 0], pts[:, 1])
        s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        k = max(1, int(round(base / step)))
        if len(s) <= k:
            return 0.0
        return float(np.degrees(np.arctan(np.max(np.abs(h[k:] - h[:-k]) / (s[k:] - s[:-k])))))

    # --- Masks -------------------------------------------------------------

    def radial(self, cx, cy):
        X, Y = self.grid()
        return np.hypot(X - cx, Y - cy)

    def radial_mask(self, cx, cy, inner, outer):
        """1 within `inner` of (cx, cy), easing to 0 at `outer`."""
        return 1 - smoothstep(inner, outer, self.radial(cx, cy))

    def path_distance(self, path):
        """Distance from every sample to a polyline, and the arc length [m] of the
        nearest point on it."""
        return path_distance(path, *self.grid())

    def path_mask(self, path, half_width, falloff):
        dist, _ = self.path_distance(path)
        return 1 - smoothstep(half_width, half_width + falloff, dist)

    # --- Synthesis ops -------------------------------------------------------

    def blend(self, mask, target):
        self.z = self.z * (1 - mask) + np.asarray(target) * mask
        return self

    def noise(self, amplitude, feature, seed, octaves=4, persistence=0.5):
        self.z += amplitude * fbm(self.n, self.size, feature, seed, octaves, persistence)
        return self

    def flatten(self, cx, cy, radius, falloff, z=None):
        """A pad at height z (default: the current height at the centre)."""
        z = self.height(cx, cy) if z is None else z
        return self.blend(self.radial_mask(cx, cy, radius, radius + falloff), z)

    def mesa(self, cx, cy, top_radius, height, cliff_width, seed, irregularity=0.15):
        """A flat-topped hill: `height` above the local terrain within an
        irregular `top_radius`, falling off over `cliff_width` metres."""
        X, Y = self.grid()
        theta = np.arctan2(Y - cy, X - cx)
        rng = np.random.default_rng(seed)
        wobble = sum(rng.uniform(-1, 1) * np.cos(k * theta + rng.uniform(0, 2 * np.pi)) / k
                     for k in range(2, 7))
        edge = top_radius * (1 + irregularity * wobble / 1.5)
        profile = 1 - smoothstep(edge, edge + cliff_width, np.hypot(X - cx, Y - cy))
        self.z += height * profile
        return self

    def ramp(self, path, half_width, falloff, z_start=None, z_end=None):
        """A graded road along a polyline, from z_start to z_end (default: the
        terrain heights at its ends), cut into or filled onto the terrain."""
        path = np.asarray(path, float)
        z0 = self.height(*path[0]) if z_start is None else z_start
        z1 = self.height(*path[-1]) if z_end is None else z_end
        dist, along = self.path_distance(path)
        total = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        target = z0 + (z1 - z0) * np.clip(along / total, 0, 1)
        return self.blend(1 - smoothstep(half_width, half_width + falloff, dist), target)

    def channel(self, path, depth, half_width, falloff):
        """A dry wash: lowers the terrain along a polyline."""
        self.z -= depth * self.path_mask(path, half_width, falloff)
        return self

    def ridge(self, path, height, half_width, falloff):
        self.z += height * self.path_mask(path, half_width, falloff)
        return self

    def level(self, cx, cy, radius, falloff):
        """Levels the ground within `radius` of (cx, cy) to its least-squares
        plane, easing back over `falloff`: a sand or clay flat, a rock slab.
        Planar ground takes few friction-zone tiles (terrains.py)."""
        X, Y = self.grid()
        r = np.hypot(X - cx, Y - cy)
        near = r <= radius
        A = np.stack([X[near] - cx, Y[near] - cy, np.ones(near.sum())], axis=1)
        (gx, gy, c0), *_ = np.linalg.lstsq(A, self.z[near], rcond=None)
        return self.blend(1 - smoothstep(radius, radius + falloff, r), c0 + gx * (X - cx) + gy * (Y - cy))

    # Engineered features: they edit only the samples near them, so a course
    # of many stays fast on a fine grid.

    def _frame(self, origin, yaw, u_range, v_range):
        """Samples under a rectangle of a local frame (u along yaw from origin,
        v to its left): the grid slices and the samples' (U, V)."""
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(origin[0] + c * u - s * v, origin[1] + s * u + c * v) for u in u_range for v in v_range]
        xs, ys = zip(*corners)
        west, north = self.center[0] - self.size / 2, self.center[1] + self.size / 2
        c0, c1 = max(int((min(xs) - west) / self.res), 0), min(int(math.ceil((max(xs) - west) / self.res)), self.n - 1)
        r0, r1 = max(int((north - max(ys)) / self.res), 0), min(int(math.ceil((north - min(ys)) / self.res)), self.n - 1)
        rows, cols = slice(r0, r1 + 1), slice(c0, c1 + 1)
        X, Y = np.meshgrid(west + np.arange(c0, c1 + 1) * self.res, north - np.arange(r0, r1 + 1) * self.res)
        dx, dy = X - origin[0], Y - origin[1]
        return rows, cols, c * dx + s * dy, -s * dx + c * dy

    def strip(self, start, yaw, width, segments, falloff, z0=None, cross=0.0):
        """A straight graded strip (test lanes, ramps, side slopes): from
        `start` along yaw, `width` wide, its centreline following the profile
        `segments` [(length [m], grade [deg])] from z0 (default: the ground at
        the start), tilted `cross` deg across (left side up), blended into the
        terrain over `falloff` metres around it. Before its start the first
        height, beyond its end the last carries on. The surface is bilinear
        between samples, so the samples a cell diagonal outside the strip
        follow it too: the ground is planar out to its very edges, where a
        friction zone's tiles need it (terrains.fit_tiles)."""
        ends = np.concatenate([[0.0], np.cumsum([length for length, _ in segments])])
        rises = np.concatenate([[0.0], np.cumsum([length * math.tan(math.radians(grade)) for length, grade in segments])])
        z0 = self.height(*start) if z0 is None else z0
        half, cell = width / 2, self.res * math.sqrt(2)
        reach = cell + falloff
        rows, cols, U, V = self._frame(start, yaw, (-reach, ends[-1] + reach), (-half - reach, half + reach))
        target = z0 + np.interp(U, ends, rises) + math.tan(math.radians(cross)) * V
        outside = np.hypot(np.maximum(np.maximum(-U, U - ends[-1]) - cell, 0), np.maximum(np.abs(V) - half - cell, 0))
        mask = 1 - smoothstep(0.0, falloff, outside)
        self.z[rows, cols] = self.z[rows, cols] * (1 - mask) + target * mask
        return self

    def bump(self, x, y, height, length, width, yaw=0.0, edge=0.2):
        """A hump under one wheel track: `height` high, a raised cosine
        `length` long along yaw, flat across `width` with `edge`-metre
        shoulders."""
        rows, cols, U, V = self._frame((x, y), yaw, (-length / 2, length / 2), (-width / 2, width / 2))
        along = 0.5 + 0.5 * np.cos(np.pi * np.clip(np.abs(U) / (length / 2), 0, 1))
        across = 1 - smoothstep(width / 2 - edge, width / 2, np.abs(V))
        self.z[rows, cols] += height * along * across
        return self

    def washboard(self, start, yaw, length, width, amplitude, wavelength, fade=1.0):
        """Corrugations across a strip from `start` along yaw: crests every
        `wavelength` metres, `amplitude` up and down, fading in and out over
        `fade` metres at the ends and sides."""
        rows, cols, U, V = self._frame(start, yaw, (0.0, length), (-width / 2, width / 2))
        envelope = (smoothstep(0.0, fade, U) * (1 - smoothstep(length - fade, length, U))
                    * (1 - smoothstep(width / 2 - fade, width / 2, np.abs(V))))
        self.z[rows, cols] += amplitude * np.sin(2 * np.pi * np.clip(U, 0.0, length) / wavelength) * envelope
        return self

    # --- Export ------------------------------------------------------------

    def write_png(self, path, zmin=None, zmax=None):
        """16-bit Gazebo heightmap: z = zmin + pixel / 65535 * (zmax - zmin),
        by default this grid's own range. Gazebo scales pixels by the image's
        own maximum pixel (not 65535) and does not shift the minimum, so the
        highest sample must map to 65535; place the heightmap with <size> ...
        (zmax - zmin) and <pos> z = zmin. Returns (zmin, zmax)."""
        zmin = float(self.z.min()) if zmin is None else zmin
        zmax = float(self.z.max()) if zmax is None else zmax
        assert zmin <= self.z.min() + 1e-9 and abs(self.z.max() - zmax) < 1e-9, "the grid must reach zmax"
        img = np.round((self.z - zmin) / max(zmax - zmin, 1e-9) * 65535).astype(np.uint16)
        Image.fromarray(img).save(path)
        return zmin, zmax
