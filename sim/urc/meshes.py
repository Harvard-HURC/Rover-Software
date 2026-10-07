"""Meshes: rocks, slabs, risers, shrubs, pebbles, textured quads and
terrain-draped decals, written as Wavefront OBJ (write_obj) or binary glTF
(write_glb).

Gazebo's mesh loader needs a normal per vertex, so both writers always write
them (area-weighted from the faces when not given). GLB loads in less memory
than OBJ (gate G3: 512 vs 801 MB for 1.64 M triangles) and collides the same,
so the merged visual-only clutter (shrubs, pebbles) and the far field are
GLB. gz does not rotate glTF's Y-up frame to its Z-up one (M: render
prototype), so write_glb writes the world frame as it is: x east, y north,
z up.
"""
import functools
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path

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


# --- Clutter shapes (design spec 5.5) ----------------------------------------------------

SLAB_VARIANTS = 8  # slab outlines every world's slabs are drawn from
SLAB_HEIGHT = 0.4  # height / diameter of a tabular block (M: DSM top-hat, H/D ~ 0.4)
SLAB_CHAMFER = 0.04  # [x diameter] the 45 deg bevel round a slab's top edge (A)


@functools.lru_cache(maxsize=None)
def slab(variant):
    """Tabular block `variant` (< SLAB_VARIANTS): an irregular 6-10-gon of
    equivalent diameter 1 (the area of a unit-diameter disc) extruded to
    SLAB_HEIGHT, its top edge bevelled; the origin at the bottom centre.
    Scale it by the block's diameter D (its height also by U(0.7, 1.3)),
    then tilt and bury it where it is placed. Sides, bevel, top and bottom
    have their own vertices, so a slab shades flat-faced, not like a pebble.
    Returns (V, F), a closed mesh."""
    rng = np.random.default_rng(2000 + variant)
    k = int(rng.integers(6, 11))
    theta = (np.arange(k) + rng.uniform(-0.3, 0.3, k)) * 2 * np.pi / k
    radius = rng.uniform(0.7, 1.0, k)
    outline = np.stack([radius * np.cos(theta), radius * np.sin(theta)], axis=1)
    x, y = outline.T
    area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    outline *= math.sqrt(math.pi / 4 / area)
    inset = outline * (1 - SLAB_CHAMFER / np.linalg.norm(outline, axis=1, keepdims=True))

    def ring(xy, z):
        return np.column_stack([xy, np.full(k, z)])

    bottom, shoulder, top = ring(outline, 0.0), ring(outline, SLAB_HEIGHT - SLAB_CHAMFER), ring(inset, SLAB_HEIGHT)
    return combine([_band(bottom, shoulder), _band(shoulder, top), _fan(top, (0.0, 0.0, SLAB_HEIGHT)),
                    _fan(bottom[::-1], (0.0, 0.0, 0.0))])


def _band(lower, upper):
    """The quads between two closed rings of equal length (counter-clockwise
    seen from above), each with its own four vertices, facing outwards."""
    k = len(lower)
    V, F = [], []
    for i in range(k):
        j = (i + 1) % k
        V += [lower[i], lower[j], upper[j], upper[i]]
        F += [(4 * i, 4 * i + 1, 4 * i + 2), (4 * i, 4 * i + 2, 4 * i + 3)]
    return np.array(V), np.array(F)


def _fan(ring, centre):
    """A polygon as a triangle fan round `centre`, facing the side from which
    `ring` runs counter-clockwise."""
    k = len(ring)
    return np.vstack([ring, centre]), np.array([(i, (i + 1) % k, k) for i in range(k)])


def riser_strip(polyline, height, depth, seed=0, bury=0.3, step=0.5, roughness=0.05):
    """A ledge (design spec 5.5): a step `height` tall standing on the ground
    along `polyline`, its face looking to the right of the polyline's
    direction (downhill) and its flat top reaching `depth` metres to the left
    (uphill, where the slope buries its back), its foot `bury` metres into
    the ground. polyline: (n, 2) points, or (n, 3) points on the ground (z
    default 0), resampled every `step` metres. The face wanders in and out
    by `roughness` x height and its top edge up and down by as much (seeded,
    A), so it does not read as a kerb. Face, top, back, bottom and ends have
    their own vertices (sharp edges). Returns (V, F), a closed mesh."""
    P = np.asarray(polyline, float)
    if P.shape[1] == 2:
        P = np.column_stack([P, np.zeros(len(P))])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1))])
    t = np.linspace(0.0, s[-1], max(2, int(math.ceil(s[-1] / step)) + 1))
    P = np.stack([np.interp(t, s, P[:, k]) for k in range(3)], axis=1)
    d = np.gradient(P[:, :2], axis=0)
    left = np.stack([-d[:, 1], d[:, 0]], axis=1) / np.linalg.norm(d, axis=1, keepdims=True)
    rng = np.random.default_rng(seed)
    knots = np.linspace(0.0, t[-1], max(2, len(t) // 4))

    def wobble():
        return np.interp(t, knots, rng.uniform(-1, 1, len(knots)))

    face = P[:, :2] + roughness * height * wobble()[:, None] * left
    back = P[:, :2] + depth * left
    top_z = P[:, 2] + height * (1 + roughness * wobble())
    foot_z = P[:, 2] - bury
    A, B = np.column_stack([face, foot_z]), np.column_stack([face, top_z])  # the face: foot, top edge
    C, D = np.column_stack([back, top_z]), np.column_stack([back, foot_z])  # the back: top, foot
    return combine([_strip(A, B), _strip(B, C), _strip(C, D), _strip(D, A),
                    _fan(np.array([A[0], B[0], C[0], D[0]]), (A[0] + C[0]) / 2),
                    _fan(np.array([A[-1], D[-1], C[-1], B[-1]]), (A[-1] + C[-1]) / 2)])


def _strip(lower, upper):
    """Quads between two polylines of equal length, facing the side to which
    lower -> upper turns counter-clockwise from the polylines' direction."""
    n = len(lower)
    F = [f for i in range(n - 1) for f in ((i, i + 1, n + i + 1), (i, n + i + 1, n + i))]
    return np.vstack([lower, upper]), np.array(F)


@functools.lru_cache(maxsize=None)
def shrub_lowpoly(variant):
    """Desert shrub `variant` (< SHRUB_VARIANTS) for the merged GLB chunks:
    6-10 jittered icosahedron clumps of 20 triangles (9,208 such shrubs made
    1.48 M triangles in the render prototype, M), filling a crown of
    diameter 1 and height 1 above the origin on the ground. Scale it by the
    shrub's diameter and height. Returns (V, F)."""
    rng = np.random.default_rng(700 + variant)
    V0, F0 = icosphere(0)
    parts = []
    for _ in range(int(rng.integers(6, 11))):
        r = rng.uniform(0.11, 0.21)
        ox, oy = rng.normal(0, 0.22, 2)
        oz = max(rng.uniform(0.2, 1.0), r * 0.5)
        jitter = 1 + 0.35 * rng.uniform(-1, 1, (len(V0), 1))
        parts.append((V0 * jitter * [r, r, r * rng.uniform(0.7, 1.1)] + [ox, oy, oz], F0))
    V, F = combine(parts)
    lo, hi = V.min(axis=0), V.max(axis=0)
    return (V - [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]]) / (hi - lo), F


PEBBLE_VARIANTS = 12  # pebble shapes (render prototype: 12)


@functools.lru_cache(maxsize=None)
def pebble(variant, subdivisions=1):
    """Pebble `variant` (< PEBBLE_VARIANTS): a flattish rock about 1 across
    and 0.55 high (render prototype), the origin at its bottom centre;
    subdivisions 0 (20 triangles) for the smallest. Scale it by the
    pebble's diameter and sink it about a third. Returns (V, F)."""
    return rock(3000 + variant, (0.5, 0.5, 0.275), roughness=0.25, subdivisions=subdivisions, flat_bottom=0.3)


# --- Binary glTF ------------------------------------------------------------------------

@dataclass(frozen=True)
class Material:
    """A glTF metallic-roughness material: base colour [0-1, linear] times
    the texture (a PNG embedded in the file), if any. A model's SDF
    <material> overrides it (the far field takes its shared texture so)."""
    color: tuple = (1.0, 1.0, 1.0)
    roughness: float = 1.0
    texture: str = None  # path of a PNG


def write_glb(path, V, F, N=None, UV=None, texture=None, color=(1.0, 1.0, 1.0), roughness=1.0):
    """One mesh in one material as a binary glTF file (write_glb_parts)."""
    write_glb_parts(path, [(V, F, N, UV, Material(tuple(color), roughness, texture))])


def write_glb_parts(path, parts):
    """A binary glTF file of one mesh with a primitive per part [(V, F, N,
    UV, Material)]: one file and one visual for many colours. Vertices in
    the world frame (x east, y north, z up: see the module notes), triangles
    counter-clockwise seen from outside; N None: area-weighted normals; UV
    None: no texture coordinates."""
    blob = bytearray()
    views, accessors, primitives, materials, images = [], [], [], [], []

    def view(data, target=None):
        blob.extend(b"\0" * (-len(blob) % 4))
        views.append({"buffer": 0, "byteOffset": len(blob), "byteLength": len(data),
                      **({"target": target} if target else {})})
        blob.extend(data)
        return len(views) - 1

    def accessor(array, kind, component, target, bounds=False):
        entry = {"bufferView": view(array.tobytes(), target), "componentType": component,
                 "count": int(len(array)), "type": kind}
        if bounds:
            entry["min"] = [float(v) for v in array.min(axis=0)]
            entry["max"] = [float(v) for v in array.max(axis=0)]
        accessors.append(entry)
        return len(accessors) - 1

    for V, F, N, UV, material in parts:
        V, F = np.asarray(V, float), np.asarray(F, int)
        N = vertex_normals(V, F) if N is None else np.asarray(N, float)
        attributes = {"POSITION": accessor(V.astype(np.float32), "VEC3", 5126, 34962, bounds=True),
                      "NORMAL": accessor(N.astype(np.float32), "VEC3", 5126, 34962)}
        if UV is not None:
            attributes["TEXCOORD_0"] = accessor(np.asarray(UV, np.float32), "VEC2", 5126, 34962)
        indices = accessor(F.astype(np.uint32).reshape(-1), "SCALAR", 5125, 34963)
        primitives.append({"attributes": attributes, "indices": indices, "material": len(materials)})
        materials.append(_gltf_material(material, images, view))
    gltf = {"asset": {"version": "2.0", "generator": "sim/urc/meshes.py"}, "scene": 0,
            "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}], "meshes": [{"primitives": primitives}],
            "materials": materials, "buffers": [{"byteLength": len(blob)}], "bufferViews": views,
            "accessors": accessors}
    if images:
        gltf["images"] = images
        gltf["samplers"] = [{"magFilter": 9729, "minFilter": 9987, "wrapS": 10497, "wrapT": 10497}]
        gltf["textures"] = [{"source": i, "sampler": 0} for i in range(len(images))]
    text = json.dumps(gltf, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    blob.extend(b"\0" * (-len(blob) % 4))
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(blob)))  # "glTF", version 2
        f.write(struct.pack("<II", len(text), 0x4E4F534A) + text)  # chunk "JSON"
        f.write(struct.pack("<II", len(blob), 0x004E4942) + bytes(blob))  # chunk "BIN"


def _gltf_material(material, images, view):
    pbr = {"baseColorFactor": [float(c) for c in material.color] + [1.0], "metallicFactor": 0.0,
           "roughnessFactor": float(material.roughness)}
    if material.texture is not None:
        images.append({"bufferView": view(Path(material.texture).read_bytes()), "mimeType": "image/png"})
        pbr["baseColorTexture"] = {"index": len(images) - 1}
    return {"pbrMetallicRoughness": pbr}
