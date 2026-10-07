"""A visual-only pebble/cobble field around a point: merged low-poly rocks, one OBJ submesh per colour."""
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, "/Users/alarion239/Desktop/Rover/sim")
from urc import meshes  # noqa: E402

HERE = Path(__file__).resolve().parent
COLORS = [(0.42, 0.33, 0.27), (0.30, 0.24, 0.21), (0.55, 0.47, 0.40), (0.62, 0.40, 0.30), (0.20, 0.17, 0.16),
          (0.70, 0.66, 0.60)]  # varnished brown, dark varnish, tan sandstone, red, black chert, pale quartzite


def heights(heightmap_png, z_max, size=2048.0):
    h = np.asarray(Image.open(heightmap_png)).astype(np.float64) / 65535 * z_max
    n = h.shape[0]
    res = size / (n - 1)

    def at(x, y):
        c = (np.asarray(x) + size / 2) / res
        r = (size / 2 - np.asarray(y)) / res
        c0, r0 = np.floor(c).astype(int), np.floor(r).astype(int)
        fc, fr = c - c0, r - r0
        return (h[r0, c0] * (1 - fc) * (1 - fr) + h[r0, c0 + 1] * fc * (1 - fr) + h[r0 + 1, c0] * (1 - fc) * fr
                + h[r0 + 1, c0 + 1] * fc * fr)
    return at


def field(out_dir, name, center, radius, density, seed=0, hz=None, sub=1, keep_out=1.2):
    """density: [(count per m^2, median diameter [m])]. Returns (triangles, pebbles)."""
    rng = np.random.default_rng(seed)
    protos = [meshes.rock(s, (1.0, 1.0, 0.55), roughness=0.25, subdivisions=sub, flat_bottom=0.3) for s in range(12)]
    parts = {k: ([], []) for k in range(len(COLORS))}
    area = math.pi * radius ** 2
    count = 0
    for per_m2, d50 in density:
        n = rng.poisson(per_m2 * area)
        r = radius * np.sqrt(rng.uniform(0, 1, n))
        th = rng.uniform(0, 2 * np.pi, n)
        x, y = center[0] + r * np.cos(th), center[1] + r * np.sin(th)
        ok = np.hypot(x - center[0], y - center[1]) > keep_out  # not under the rover
        x, y = x[ok], y[ok]
        d = d50 * np.exp(rng.normal(0, 0.5, len(x)))
        z = hz(x, y)
        for i in range(len(x)):
            V, F = protos[rng.integers(len(protos))]
            s = d[i] / 2
            sx, sy, sz = s * rng.uniform(0.7, 1.3), s * rng.uniform(0.7, 1.3), s * rng.uniform(0.5, 1.0)
            yaw = rng.uniform(0, 2 * np.pi)
            c, sn = math.cos(yaw), math.sin(yaw)
            P = V * [sx, sy, sz]
            P = np.stack([c * P[:, 0] - sn * P[:, 1], sn * P[:, 0] + c * P[:, 1], P[:, 2]], axis=1)
            P += [x[i], y[i], z[i] - 0.35 * sz * 1.1]  # embedded a third
            k = int(rng.integers(len(COLORS)))
            parts[k][0].append(P)
            parts[k][1].append(F)
            count += 1
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    mtl = [f"newmtl m{k}\nKd {r:.3f} {g:.3f} {b:.3f}\nKa {r:.3f} {g:.3f} {b:.3f}\nKs 0.05 0.05 0.05\nNs 10\n"
           for k, (r, g, b) in enumerate(COLORS)]
    (out / f"{name}.mtl").write_text("\n".join(mtl))
    lines = [f"mtllib {name}.mtl"]
    vbase = 0
    tris = 0
    for k, (Ps, Fs) in parts.items():
        if not Ps:
            continue
        V = np.concatenate(Ps)
        offs = np.cumsum([0] + [len(p) for p in Ps[:-1]])
        F = np.concatenate([f + o for f, o in zip(Fs, offs)])
        N = meshes.vertex_normals(V, F)
        lines.append(f"o part{k}\nusemtl m{k}")
        lines += [f"v {a:.4f} {b:.4f} {c:.4f}" for a, b, c in V]
        lines += [f"vn {a:.3f} {b:.3f} {c:.3f}" for a, b, c in N]
        lines += [f"f {a + vbase}//{a + vbase} {b + vbase}//{b + vbase} {c + vbase}//{c + vbase}"
                  for a, b, c in F + 1]
        vbase += len(V)
        tris += len(F)
    (out / f"{name}.obj").write_text("\n".join(lines) + "\n")
    return tris, count
