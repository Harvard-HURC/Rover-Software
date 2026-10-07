"""Wavefront OBJ meshes: rocks, shrubs, textured quads, terrain-draped decals.

Gazebo's mesh loader needs a normal per vertex, so write_obj always writes
them (area-weighted from the faces when not given).
"""
import functools
import math

import numpy as np


def vertex_normals(V, F):
    V, F = np.asarray(V, float), np.asarray(F, int)
    face_n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    N = np.zeros_like(V)
    for k in range(3):
        np.add.at(N, F[:, k], face_n)
    length = np.linalg.norm(N, axis=1, keepdims=True)
    return N / np.where(length > 0, length, 1)


def write_obj(path, V, F, N=None, UV=None):
    """Triangles F (indices into V), counter-clockwise seen from outside."""
    V, F = np.asarray(V, float), np.asarray(F, int)
    N = vertex_normals(V, F) if N is None else np.asarray(N, float)
    lines = [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in V]
    lines += [f"vn {x:.5f} {y:.5f} {z:.5f}" for x, y, z in N]
    if UV is not None:
        lines += [f"vt {u:.5f} {v:.5f}" for u, v in np.asarray(UV, float)]
        lines += ["f " + " ".join(f"{i + 1}/{i + 1}/{i + 1}" for i in tri) for tri in F]
    else:
        lines += ["f " + " ".join(f"{i + 1}//{i + 1}" for i in tri) for tri in F]
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def icosphere(subdivisions=2):
    t = (1 + 5 ** 0.5) / 2
    V = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    F = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2),
         (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11),
         (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    V = [np.array(v) / np.linalg.norm(v) for v in V]
    for _ in range(subdivisions):
        cache, faces = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = V[a] + V[b]
                V.append(m / np.linalg.norm(m))
                cache[key] = len(V) - 1
            return cache[key]

        for a, b, c in F:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        F = faces
    return np.array(V), np.array(F)


def rock(seed, radii, roughness=0.22, subdivisions=2, flat_bottom=0.35):
    """A lumpy rock: an icosphere scaled to `radii` (x, y, z) [m] with smooth
    random displacement, its lower part flattened (rocks sit, they do not
    balance on a point). The origin is at the bottom centre."""
    V, F = icosphere(subdivisions)
    rng = np.random.default_rng(seed)
    bump = np.zeros(len(V))
    for _ in range(6):  # a few random lobes and dents
        d = rng.normal(size=3)
        d /= np.linalg.norm(d)
        bump += rng.uniform(-1, 1) * np.maximum(V @ d, 0) ** 3
    bump += 0.3 * rng.uniform(-1, 1, len(V))
    V = V * (1 + roughness * bump / max(np.abs(bump).max(), 1e-9))[:, None]
    floor = -1 + flat_bottom
    V[:, 2] = np.maximum(V[:, 2], floor)
    V = (V - [0, 0, floor]) * np.asarray(radii)
    return V, F


ROCK_VARIANTS = 8  # rock shapes every world's rocks are drawn from


@functools.lru_cache(maxsize=None)
def rock_variant(variant):
    """Rock shape `variant` (< ROCK_VARIANTS) with long half-axis 1, scaled
    by each rock's size when WorldBuilder merges rocks into meshes."""
    rng = np.random.default_rng(1000 + variant)
    return rock(variant, (1.0, rng.uniform(0.65, 0.95), rng.uniform(0.45, 0.75)))


@functools.lru_cache(maxsize=None)
def rock_base(variant):
    """The vertices of rock_variant(variant) on its flat base (z = 0): its
    lowest points, which WorldBuilder.rock_field sinks below the ground."""
    V = rock_variant(variant)[0]
    return V[V[:, 2] <= 1e-9]


def tilt(gx, gy):
    """Rotation (3x3) about a horizontal axis taking +z to the upward normal
    of the plane z = gx x + gy y: a rock's base laid on sloping ground."""
    n = np.array([-gx, -gy, 1.0]) / math.sqrt(gx * gx + gy * gy + 1)
    axis = np.array([-n[1], n[0], 0.0])  # z x n: |z x n| = sin(angle), z . n = cos(angle)
    s = float(np.linalg.norm(axis))
    if s < 1e-12:
        return np.eye(3)
    kx, ky, _ = axis / s
    K = np.array([[0, 0, ky], [0, 0, -kx], [-ky, kx, 0]])
    return np.eye(3) + s * K + (1 - n[2]) * (K @ K)


SHRUB_VARIANTS = 6  # shrub shapes every world's shrubs are drawn from


@functools.lru_cache(maxsize=None)
def shrub(variant):
    """Desert shrub `variant` (< SHRUB_VARIANTS): 6-10 overlapping round
    clumps, the origin on the ground at its centre. Returns (V, F)."""
    rng = np.random.default_rng(500 + variant)
    V, F = icosphere(1)
    parts = []
    for _ in range(int(rng.integers(6, 11))):
        r = rng.uniform(0.12, 0.25)
        x, y = rng.normal(0, 0.18, 2)
        parts.append((V * r + (x, y, r * 0.8 + rng.uniform(0, 0.15)), F))
    return combine(parts)


@functools.lru_cache(maxsize=None)
def hull_faces(subdivisions):
    """Faces of icosphere(subdivisions). Subdividing only appends vertices, so
    they index the first vertices of any finer icosphere, or of a rock made
    from one: (V[:n], hull_faces(s)) is a coarser mesh of the same surface."""
    return icosphere(subdivisions)[1]


def combine(parts):
    """One mesh from [(V, F)]."""
    V, F, offset = [], [], 0
    for v, f in parts:
        V.append(v)
        F.append(np.asarray(f) + offset)
        offset += len(v)
    return np.concatenate(V), np.concatenate(F)


def quad():
    """Unit square in the y-z plane facing +x, centred on the origin. Seen
    from +x the image is upright and not mirrored."""
    V = np.array([(0, -0.5, -0.5), (0, 0.5, -0.5), (0, 0.5, 0.5), (0, -0.5, 0.5)], float)
    F = np.array([(0, 1, 2), (0, 2, 3)])
    UV = np.array([(0, 0), (1, 0), (1, 1), (0, 1)], float)
    N = np.tile([1.0, 0, 0], (4, 1))
    return V, F, N, UV


def write_quad(path):
    write_obj(path, *quad())


def blob_outline(radius, seed, irregularity=0.35, step=0.5):
    """An irregular round outline: angles theta and edge radii (<= radius),
    one spoke per `step` metres of circumference."""
    rng = np.random.default_rng(seed)
    spokes = max(24, int(2 * np.pi * radius / step))
    theta = np.linspace(0, 2 * np.pi, spokes, endpoint=False)
    wobble = sum(rng.uniform(-1, 1) * np.cos(k * theta + rng.uniform(0, 6.3)) / k for k in range(1, 6))
    return theta, radius * (1 - irregularity / 2 + irregularity / 2 * wobble / np.abs(wobble).max())


def drape(hf, cx, cy, radius, seed, offset=0.03, step=0.5, tile=4.0, irregularity=0.35, grow=0.0):
    """A decal following the terrain: the blob_outline patch around (cx, cy),
    grown outwards by `grow` metres, whose vertices sit `offset` above the
    heightfield. A polar mesh (rings and spokes), so the outline is smooth.
    Coordinates are relative to (cx, cy, 0); UVs repeat every `tile` metres.
    Returns (V, F, UV)."""
    theta, edge = blob_outline(radius, seed, irregularity, step)
    edge = edge + grow
    spokes = len(theta)
    rings = max(2, int(np.ceil(radius / step)))
    X = [0.0]
    Y = [0.0]
    for i in range(1, rings + 1):
        X += list(edge * i / rings * np.cos(theta))
        Y += list(edge * i / rings * np.sin(theta))
    X, Y = np.array(X), np.array(Y)
    V = np.stack([X, Y, hf.height(X + cx, Y + cy) + offset], axis=1)
    UV = np.stack([(X + radius) / tile, (Y + radius) / tile], axis=1)

    def ring(i, j):
        return 0 if i == 0 else 1 + (i - 1) * spokes + j % spokes

    F = [(0, ring(1, j), ring(1, j + 1)) for j in range(spokes)]
    for i in range(1, rings):
        for j in range(spokes):
            a, b, c, d = ring(i, j), ring(i, j + 1), ring(i + 1, j), ring(i + 1, j + 1)
            F += [(a, c, d), (a, d, b)]  # counter-clockwise seen from above
    return V, np.array(F, int), UV


def drape_rect(hf, cx, cy, length, width, yaw=0.0, offset=0.03, step=0.5, tile=4.0, breaks=()):
    """A rectangular decal following the terrain: `length` along yaw, `width`
    across, centred on (cx, cy), vertices `offset` above the heightfield. Grid
    lines also run across at the distances `breaks` from the start (put them
    on the terrain's kinks, so the decal does not cut the corners). Returns
    (V, F, UV) relative to (cx, cy, 0)."""
    us = np.unique(np.concatenate([np.linspace(0, length, max(2, int(np.ceil(length / step)) + 1)),
                                   [b for b in breaks if 0 < b < length]])) - length / 2
    vs = np.linspace(-width / 2, width / 2, max(2, int(np.ceil(width / step)) + 1))
    U, W = (a.ravel() for a in np.meshgrid(us, vs))  # row-major: one row per v
    c, s = np.cos(yaw), np.sin(yaw)
    X, Y = c * U - s * W, s * U + c * W
    V = np.stack([X, Y, hf.height(X + cx, Y + cy) + offset], axis=1)
    UV = np.stack([(U + length / 2) / tile, (W + width / 2) / tile], axis=1)
    n = len(us)
    F = []
    for i in range(len(vs) - 1):
        for j in range(n - 1):
            a, b, d = i * n + j, i * n + j + 1, (i + 1) * n + j
            F += [(a, b, d + 1), (a, d + 1, d)]  # counter-clockwise seen from above
    return V, np.array(F, int), UV
