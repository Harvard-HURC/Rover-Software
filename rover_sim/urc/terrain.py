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


def blur(z, sigma):
    """A Gaussian blur of a grid, sigma in samples (OpenCV's, edges reflected)."""
    return cv2.GaussianBlur(np.asarray(z, np.float32), (0, 0), sigma).astype(float)


def slope_map(z, res, smooth_m=0.0):
    """Steepest slope [deg] at every sample of a grid of spacing res, after
    a Gaussian blur of smooth_m metres (design 5.3 paints on sigma 2 m)."""
    if smooth_m:
        z = blur(z, smooth_m / res)
    gy, gx = np.gradient(np.asarray(z, float), res)
    return np.degrees(np.arctan(np.hypot(gx, gy)))


SWATCH_BLOCK = 64.0  # [m] relief swatches are laid in blocks of this size (design 5.4)
SWATCH_FEATHER = 16.0  # [m] cosine-feathered into their neighbours over this


def _ramp(t, block, feather):
    """Weight of a block along one axis at t metres from its extended start:
    sin^2 up over the first `feather`, 1, cos^2 down over the last; a
    block's ramp down and the next one's up add to 1."""
    up = np.sin(0.5 * np.pi * np.clip(t / feather, 0.0, 1.0)) ** 2
    down = np.cos(0.5 * np.pi * np.clip((t - block) / feather, 0.0, 1.0)) ** 2
    return np.minimum(up, down)


def _detrend(grid):
    """A grid minus its least-squares plane."""
    a = np.asarray(grid, float) - np.mean(grid)
    rows, cols = a.shape
    v, u = (np.arange(k) - (k - 1) / 2 for k in (rows, cols))  # centred, so the fit's terms are orthogonal
    return a - np.outer(v, np.ones(cols)) * (v @ a.sum(axis=1)) / (cols * (v @ v)) \
        - np.outer(np.ones(rows), u) * (u @ a.sum(axis=0)) / (rows * (u @ u))


def swatch_field(windows, res_m, n, size, seed, block=SWATCH_BLOCK, feather=SWATCH_FEATHER):
    """A residual field on an n x n grid of `size` metres, tiled from swatch
    windows (float grids of spacing res_m, row 0 north): blocks of `block`
    metres from random places of a random window, each less its own
    least-squares plane (a block's mean and tilt are relief larger than the
    tiling can carry, and would show as steps along its feathers), turned by
    a random multiple of 90 deg and mirrored at random, laid on a grid of
    random offset and cosine-feathered over `feather` into their
    neighbours. Where blocks overlap with weights w_i the field is sum(w_i
    s_i) / sqrt(sum(w_i^2)): independent residuals added that way keep their
    RMS, which plain feathering lowers by up to 1/sqrt(2) (design 5.4)."""
    rng = np.random.default_rng(seed)
    span = block + feather  # a block's extent, its feathers included
    side = int(round(span / res_m))  # source pixels across a block
    for w in windows:
        if min(w.shape) <= side + 1:
            raise ValueError(f"a swatch window of {w.shape} px is smaller than a {span} m block")
    coords = np.linspace(-size / 2, size / 2, n)  # x of the columns, -y of the rows (row 0 north)
    start = -size / 2 - feather - rng.uniform(0.0, block)
    count = int(math.ceil((size / 2 - start) / block))
    num, den = np.zeros((n, n)), np.zeros((n, n))
    for i in range(count):  # blocks west to east
        for j in range(count):  # north to south
            x0, y0 = start + i * block, start + j * block  # the block's west and north edge, y measured southwards
            cols = np.flatnonzero((coords >= x0) & (coords <= x0 + span))
            rows = np.flatnonzero((coords >= y0) & (coords <= y0 + span))
            if not len(cols) or not len(rows):
                continue
            window = windows[int(rng.integers(len(windows)))]
            r0, c0 = (int(rng.integers(0, s - side)) for s in window.shape)
            patch = _detrend(window[r0:r0 + side + 1, c0:c0 + side + 1])
            patch = np.rot90(patch, int(rng.integers(4)))
            if rng.integers(2):
                patch = patch[:, ::-1]
            u, v = coords[cols] - x0, coords[rows] - y0  # metres east and south of the block's corner
            U, V = np.meshgrid(u / res_m, v / res_m)
            s = cv2.remap(np.ascontiguousarray(patch, np.float32), U.astype(np.float32), V.astype(np.float32),
                          cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
            w = np.outer(_ramp(v, block, feather), _ramp(u, block, feather))
            num[np.ix_(rows, cols)] += w * s
            den[np.ix_(rows, cols)] += w * w
    return num / np.sqrt(np.maximum(den, 1e-12))


HAYSTACK_SHRINK = 0.5  # where the belt leaves no room, a knob shrinks to this fraction of the recipe's smallest
# (A: the proving ground's 16 m badland strip, easing in over 2 m, holds none of 10 m and more)


def haystack_heights(n, size, mask, seed, recipe):
    """Rounded badland knobs on an n x n grid of `size` metres where mask >
    0.5 (a terrains.Haystacks recipe, design 5.4): knobs of diameters in
    recipe.diameter_m, each a raised cosine h = H cos^2(pi r / 2R) whose
    steepest flank is drawn between recipe.min_flank_deg and flank_deg
    (H = tan(flank) 2R / pi), placed at random apart from each other until
    they cover 1 - recipe.floor of the masked area. Each knob lies wholly
    where the mask is over 0.5 (its radius shrinks to fit, down to
    HAYSTACK_SHRINK of the recipe's smallest) and is as much lower as the
    mask is there at its least, whole: the mask fades a belt's knobs, never
    cuts one (multiplied by a mask that falls over 2 m, as the proving
    ground's strips, knobs were cut into 3 m walls of 60 deg). Heights to add
    as they are."""
    rng = np.random.default_rng(seed)
    res = size / (n - 1)
    mask = np.asarray(mask, float)
    belt = (mask > 0.5).astype(np.uint8)
    room = cv2.distanceTransform(np.pad(belt, 1, constant_values=1), cv2.DIST_L2, 5)[1:-1, 1:-1] * res
    inside = np.flatnonzero(room >= HAYSTACK_SHRINK * recipe.diameter_m[0] / 2)  # [m] to the belt's edge
    out = np.zeros((n, n))
    target = (1 - recipe.floor) * np.count_nonzero(belt) * res * res
    knobs = np.zeros((max(1, int(target / (math.pi * (recipe.diameter_m[0] / 2) ** 2)) + 1), 3))  # x, y, radius
    count, cover, tries = 0, 0.0, 0
    while cover < target and tries < 50 * max(1, int(target / 100)) and len(inside) and count < len(knobs):
        tries += 1
        k = inside[int(rng.integers(len(inside)))]
        x, y = (k % n) * res, (k // n) * res  # metres east and south of the north-west corner
        radius = min(rng.uniform(*recipe.diameter_m) / 2, room.flat[k])
        a, b, r = knobs[:count].T
        if np.any(np.hypot(a - x, b - y) < 0.8 * (r + radius)):
            continue
        knobs[count] = x, y, radius
        count += 1
        cover += math.pi * radius * radius
        height = math.tan(math.radians(rng.uniform(recipe.min_flank_deg, recipe.flank_deg))) * 2 * radius / math.pi
        c0, c1 = max(int((x - radius) / res), 0), min(int(math.ceil((x + radius) / res)), n - 1)
        r0, r1 = max(int((y - radius) / res), 0), min(int(math.ceil((y + radius) / res)), n - 1)
        C, R = np.meshgrid(np.arange(c0, c1 + 1) * res, np.arange(r0, r1 + 1) * res)
        r = np.hypot(C - x, R - y)
        under = r < radius
        fade = float(mask[r0:r1 + 1, c0:c1 + 1][under].min()) if under.any() else 0.0
        out[r0:r1 + 1, c0:c1 + 1] += np.where(under, fade * height * np.cos(0.5 * np.pi * r / radius) ** 2, 0.0)
    return out


RILL_SMOOTH = 2.0  # [m] rills descend the surface blurred this much, so single-sample bumps do not trap them (A)
RILL_STEP = 0.5  # [m] trace step


def rill_traces(z, res, mask, seed, recipe):
    """Rill paths down a grid z of spacing res (a terrains.Rills recipe,
    design 5.4): from seeds on a jittered grid of a pitch drawn from
    recipe.spacing_m, where mask > 0.5 and the ground is steeper than
    recipe.min_slope_deg, each steps RILL_STEP down the steepest descent of
    the blurred surface until it leaves the mask, flattens out or reaches
    recipe.length_m; all seeds step together (no loop over samples).
    Returns [(k, 2) array of (row, col) sample coordinates], each at least
    two points long."""
    rng = np.random.default_rng(seed)
    z = np.asarray(z, float)
    n_rows, n_cols = z.shape
    g_row, g_col = (blur(g, RILL_SMOOTH / res) for g in np.gradient(z, res))  # blurring the slope, not the
    # surface, keeps a plane's slope right up to the edges
    mask = np.asarray(mask, np.float32)
    pitch = rng.uniform(*recipe.spacing_m) / res
    seeds = np.stack(np.meshgrid(np.arange(0.0, n_rows - 1, pitch), np.arange(0.0, n_cols - 1, pitch),
                                 indexing="ij"), axis=-1).reshape(-1, 2)
    p = np.clip(seeds + rng.uniform(0, pitch, seeds.shape), 0, [n_rows - 1, n_cols - 1])
    steep = math.tan(math.radians(recipe.min_slope_deg))

    def at(grid, q):
        return cv2.remap(np.asarray(grid, np.float32), q[:, 1].astype(np.float32).reshape(-1, 1),
                         q[:, 0].astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REPLICATE).ravel()

    def going(q):
        gr, gc = at(g_row, q), at(g_col, q)
        inside = (q[:, 0] >= 0) & (q[:, 0] <= n_rows - 1) & (q[:, 1] >= 0) & (q[:, 1] <= n_cols - 1)
        return inside & (at(mask, q) > 0.5) & (np.hypot(gr, gc) > steep), gr, gc

    p = p[going(p)[0]] if len(p) else p
    if not len(p):
        return []
    path, alive = [p.copy()], [np.ones(len(p), bool)]
    step = RILL_STEP / res
    for _ in range(int(recipe.length_m / RILL_STEP)):
        ok, gr, gc = going(p)
        ok &= alive[-1]
        g = np.maximum(np.hypot(gr, gc), 1e-12)
        p = np.where(ok[:, None], p - step * np.stack([gr, gc], axis=1) / g[:, None], p)
        path.append(p.copy())
        alive.append(ok)
        if not ok.any():
            break
    path, alive = np.stack(path, axis=1), np.stack(alive, axis=1)  # (seeds, steps, 2), (seeds, steps)
    lengths = alive.sum(axis=1)  # points of each trace: the seed and every step taken
    return [path[k, :lengths[k]] for k in range(len(path)) if lengths[k] >= 2]


def rill_depths(z, res, mask, seed, recipe):
    """How deep rills cut into a grid z (rill_traces): along each trace the
    depth grows from recipe.depth_m[0] to [1] and the width from
    recipe.width_m[0] to [1] over recipe.length_m, deeper parts drawn last;
    the cut is rounded by a blur of a quarter of the narrowest width where
    the grid resolves it."""
    out = np.zeros(np.shape(z), np.float32)
    (d0, d1), (w0, w1) = recipe.depth_m, recipe.width_m
    segments = []
    for trace in rill_traces(z, res, mask, seed, recipe):
        t = np.arange(len(trace) - 1) * RILL_STEP / recipe.length_m
        for k in range(len(trace) - 1):
            segments.append((d0 + (d1 - d0) * t[k], w0 + (w1 - w0) * t[k], trace[k], trace[k + 1]))
    scale = 16  # cv2 sub-pixel coordinates: 4 fractional bits
    for depth, width, a, b in sorted(segments, key=lambda s: s[0]):
        cv2.line(out, (int(round(a[1] * scale)), int(round(a[0] * scale))),
                 (int(round(b[1] * scale)), int(round(b[0] * scale))), float(depth),
                 thickness=max(1, int(round(width / res))), lineType=cv2.LINE_8, shift=4)
    sigma = w0 / 4 / res
    return (blur(out, sigma) if sigma >= 0.5 else out.astype(float))


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
        edge, r = self.mesa_edge(cx, cy, top_radius, seed, irregularity)
        self.z += height * (1 - smoothstep(edge, edge + cliff_width, r))
        return self

    def mesa_edge(self, cx, cy, top_radius, seed, irregularity=0.15):
        """A mesa's irregular top edge (mesa()): its radius towards every
        sample, and every sample's distance from the centre (landscape.Hills
        paints the top, the cliff and the floor around it)."""
        X, Y = self.grid()
        theta = np.arctan2(Y - cy, X - cx)
        rng = np.random.default_rng(seed)
        wobble = sum(rng.uniform(-1, 1) * np.cos(k * theta + rng.uniform(0, 2 * np.pi)) / k
                     for k in range(2, 7))
        return top_radius * (1 + irregularity * wobble / 1.5), np.hypot(X - cx, Y - cy)

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
        follow it too: the ground is planar out to its very edges."""
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

    # Micro-relief (design spec 5.4): what a synthetic world adds below its
    # macro shape, under a mask in [0, 1] (landscape.relief composes them).

    def detail(self, swatch, mask, amplitude, seed):
        """Adds a real lidar residual (a landscape.Swatch) tiled over the grid
        (swatch_field), times amplitude and mask."""
        self.z += amplitude * np.asarray(mask) * swatch_field(swatch.windows, swatch.res_m, self.n, self.size, seed)
        return self

    def haystacks(self, mask, seed, recipe):
        """Adds rounded badland knobs (haystack_heights) where mask > 0.5,
        each faded whole by the mask under it."""
        self.z += haystack_heights(self.n, self.size, mask, seed, recipe)
        return self

    def rills(self, mask, seed, recipe):
        """Carves rills (rill_depths) down this surface where mask > 0.5,
        faded by the mask."""
        self.z -= np.asarray(mask) * rill_depths(self.z, self.res, mask, seed, recipe)
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
