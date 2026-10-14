"""Minimal binary glTF (GLB) writer: one mesh, one primitive, optional base-colour + normal textures."""
import json
import struct

import numpy as np


def _pad(b, fill=b"\x00"):
    return b + fill * ((4 - len(b) % 4) % 4)


def write_glb(path, pos, nrm, uv, idx, base_color_png=None, normal_png=None, yup=True, uv1=None, color=None, roughness=1.0):
    """pos/nrm: (n,3) float in gz frame (x east, y north, z up) - converted to glTF Y-up if yup;
    uv: (n,2); idx: (m,3) uint32."""
    pos = np.asarray(pos, np.float32)
    nrm = np.asarray(nrm, np.float32)
    if yup:  # glTF: +Y up, +Z towards viewer; gz (x, y, z) -> glTF (x, z, -y)
        pos = np.stack([pos[:, 0], pos[:, 2], -pos[:, 1]], axis=1)
        nrm = np.stack([nrm[:, 0], nrm[:, 2], -nrm[:, 1]], axis=1)
    uv = np.asarray(uv, np.float32)
    idx = np.asarray(idx, np.uint32).reshape(-1)
    blobs, views, accessors = [], [], []
    offset = 0

    def add(arr, target, comp, typ, minmax=False):
        nonlocal offset
        b = _pad(arr.tobytes())
        views.append({"buffer": 0, "byteOffset": offset, "byteLength": arr.nbytes, **({"target": target} if target else {})})
        acc = {"bufferView": len(views) - 1, "componentType": comp, "count": int(arr.shape[0]), "type": typ}
        if minmax:
            acc["min"] = [float(v) for v in arr.min(axis=0)]
            acc["max"] = [float(v) for v in arr.max(axis=0)]
        accessors.append(acc)
        blobs.append(b)
        offset += len(b)
        return len(accessors) - 1

    a_pos = add(pos, 34962, 5126, "VEC3", True)
    a_nrm = add(nrm, 34962, 5126, "VEC3")
    a_uv = add(uv, 34962, 5126, "VEC2")
    attrs = {"POSITION": a_pos, "NORMAL": a_nrm, "TEXCOORD_0": a_uv}
    if uv1 is not None:
        attrs["TEXCOORD_1"] = add(np.asarray(uv1, np.float32), 34962, 5126, "VEC2")
    a_idx = add(idx, 34963, 5125, "SCALAR")
    images, textures = [], []
    for png in (base_color_png, normal_png):
        if png:
            data = open(png, "rb").read()
            views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(data)})
            blobs.append(_pad(data))
            offset += len(_pad(data))
            images.append({"bufferView": len(views) - 1, "mimeType": "image/png"})
            textures.append({"source": len(images) - 1})
    mat = {"pbrMetallicRoughness": {"metallicFactor": 0.0, "roughnessFactor": roughness}}
    if color is not None:
        mat["pbrMetallicRoughness"]["baseColorFactor"] = [float(c) for c in color] + [1.0]
    t = 0
    if base_color_png:
        mat["pbrMetallicRoughness"]["baseColorTexture"] = {"index": t}
        t += 1
    if normal_png:
        mat["normalTexture"] = {"index": t}
    gltf = {"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": [{"attributes": attrs, "indices": a_idx, "material": 0}]}],
            "materials": [mat], "buffers": [{"byteLength": offset}], "bufferViews": views, "accessors": accessors}
    if images:
        gltf["images"] = images
        gltf["textures"] = textures
        gltf["samplers"] = [{"magFilter": 9729, "minFilter": 9987, "wrapS": 10497, "wrapT": 10497}]
        for tx in gltf["textures"]:
            tx["sampler"] = 0
    j = _pad(json.dumps(gltf).encode(), b" ")
    binary = b"".join(blobs)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(j) + 8 + len(binary)))
        f.write(struct.pack("<II", len(j), 0x4E4F534A) + j)
        f.write(struct.pack("<II", len(binary), 0x004E4942) + binary)


def grid_mesh(z, size, step=1, uv_scale=None):
    """Heightfield z[row, col] (row 0 north) over size x size metres centred at 0 -> pos, nrm, uv, idx.
    step: sample stride. uv in [0, 1] over the terrain (row 0 north = v 0)."""
    zz = z[::step, ::step].astype(np.float64)
    n = zz.shape[0]
    c = np.linspace(-size / 2, size / 2, n)
    X, Y = np.meshgrid(c, c[::-1])
    gy, gx = np.gradient(zz, size / (n - 1))
    N = np.stack([-gx, gy, np.ones_like(zz)], -1)  # rows run south: dz/dnorth = -gy, so normal y = +gy
    N /= np.linalg.norm(N, axis=-1, keepdims=True)
    pos = np.stack([X, Y, zz], -1).reshape(-1, 3)
    u = (X + size / 2) / size
    v = (size / 2 - Y) / size
    uv = np.stack([u, v], -1).reshape(-1, 2)
    r, cc = np.meshgrid(np.arange(n - 1), np.arange(n - 1), indexing="ij")
    i0 = (r * n + cc).ravel()
    i1, i2, i3 = i0 + 1, i0 + n, i0 + n + 1
    idx = np.concatenate([np.stack([i0, i2, i1], 1), np.stack([i1, i2, i3], 1)])  # CCW seen from +z
    return pos, N.reshape(-1, 3), uv, idx
