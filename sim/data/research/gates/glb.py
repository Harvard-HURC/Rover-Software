"""Minimal glTF 2.0 binary (GLB) and OBJ writers for the gate meshes (scratch)."""
import json
import struct

import numpy as np


def normals(V, F):
    N = np.zeros_like(V)
    face = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    for k in range(3):
        np.add.at(N, F[:, k], face)
    return N / np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)


def write_glb(path, V, F, N=None, UV=None, color=(0.6, 0.5, 0.4, 1.0)):
    """Z-up vertices as given (gz does not rotate glTF's Y-up, spec 5.8)."""
    V = np.asarray(V, np.float32)
    F = np.asarray(F, np.uint32)
    N = (normals(V.astype(float), F) if N is None else np.asarray(N)).astype(np.float32)
    blobs = [V.tobytes(), N.tobytes(), F.ravel().tobytes()]
    if UV is not None:
        blobs.append(np.asarray(UV, np.float32).tobytes())
    views, offset, binary = [], 0, b""
    for k, blob in enumerate(blobs):
        pad = (-len(blob)) % 4
        target = 34963 if k == 2 else 34962
        views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(blob), "target": target})
        binary += blob + b"\0" * pad
        offset += len(blob) + pad
    accessors = [
        {"bufferView": 0, "componentType": 5126, "count": len(V), "type": "VEC3",
         "min": V.min(axis=0).tolist(), "max": V.max(axis=0).tolist()},
        {"bufferView": 1, "componentType": 5126, "count": len(N), "type": "VEC3"},
        {"bufferView": 2, "componentType": 5125, "count": F.size, "type": "SCALAR"},
    ]
    attributes = {"POSITION": 0, "NORMAL": 1}
    if UV is not None:
        accessors.append({"bufferView": 3, "componentType": 5126, "count": len(UV), "type": "VEC2"})
        attributes["TEXCOORD_0"] = 3
    doc = {"asset": {"version": "2.0", "generator": "ws0 gate"}, "scene": 0, "scenes": [{"nodes": [0]}],
           "nodes": [{"mesh": 0}], "meshes": [{"primitives": [{"attributes": attributes, "indices": 2, "material": 0}]}],
           "materials": [{"pbrMetallicRoughness": {"baseColorFactor": list(color), "metallicFactor": 0.0,
                                                   "roughnessFactor": 1.0}}],
           "buffers": [{"byteLength": len(binary)}], "bufferViews": views, "accessors": accessors}
    text = json.dumps(doc, separators=(",", ":")).encode()
    text += b" " * ((-len(text)) % 4)
    out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(binary))
    out += struct.pack("<II", len(text), 0x4E4F534A) + text
    out += struct.pack("<II", len(binary), 0x004E4942) + binary
    with open(path, "wb") as f:
        f.write(out)


def write_obj(path, V, F, N=None):
    N = normals(np.asarray(V, float), np.asarray(F)) if N is None else N
    with open(path, "w") as f:
        f.writelines(f"v {x:.6f} {y:.6f} {z:.6f}\n" for x, y, z in V)
        f.writelines(f"vn {x:.6f} {y:.6f} {z:.6f}\n" for x, y, z in N)
        f.writelines(f"f {a + 1}//{a + 1} {b + 1}//{b + 1} {c + 1}//{c + 1}\n" for a, b, c in F)


def box(size, z0=0.0):
    """A closed box mesh (flat-shaded: 24 vertices), x, y centred, z from z0."""
    sx, sy, sz = (s / 2 for s in size)
    V, F = [], []
    for axis in range(3):
        for sign in (-1, 1):
            u, v = [a for a in range(3) if a != axis]
            corners = []
            for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                p = [0.0, 0.0, 0.0]
                p[axis] = sign
                p[u], p[v] = du, dv
                corners.append(p)
            base = len(V)
            V += corners
            n = np.zeros(3)
            n[axis] = sign
            a, b, c = (np.array(corners[i]) for i in range(3))
            if np.dot(np.cross(b - a, c - a), n) > 0:
                F += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
            else:
                F += [(base, base + 2, base + 1), (base, base + 3, base + 2)]
    V = np.array(V) * [sx, sy, sz] + [0, 0, sz + z0]
    return V, np.array(F)


def wedge(length, width, angle_deg):
    """A ramp rising along +x from z = 0 to length * tan(angle), as a closed prism."""
    h = length * np.tan(np.radians(angle_deg))
    x0, x1, y0, y1 = -length / 2, length / 2, -width / 2, width / 2
    V = np.array([(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0), (x1, y0, h), (x1, y1, h)], float)
    F = np.array([(0, 2, 1), (0, 3, 2),  # bottom
                  (0, 1, 4), (3, 5, 2),  # sides
                  (1, 2, 5), (1, 5, 4),  # back wall
                  (0, 4, 5), (0, 5, 3)])  # slope
    return V, F


def icosphere(subdivisions):
    t = (1 + 5 ** 0.5) / 2
    V = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    F = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6),
         (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10),
         (8, 6, 7), (9, 8, 1)]
    V = [np.array(v, float) / np.linalg.norm(v) for v in V]
    for _ in range(subdivisions):
        cache, out = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = (V[a] + V[b]) / 2
                V.append(m / np.linalg.norm(m))
                cache[key] = len(V) - 1
            return cache[key]
        for a, b, c in F:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            out += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        F = out
    return np.array(V), np.array(F)
