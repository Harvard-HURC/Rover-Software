"""Far-field terrain: a visual-only GLB mesh of the real landscape around the world (USGS 3DEP DEM +
NAIP overview), with Earth curvature, a hole under the detailed heightmap, and geographic UVs."""
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

from glb import write_glb

sys.path.insert(0, "/Users/alarion239/Desktop/Rover/sim")
from urc import geo  # noqa: E402

Image.MAX_IMAGE_PIXELS = None
R_EARTH = 6371000.0


def build(sheet_json, dem_tif, naip_tif, out_glb, spacing=120.0, inner=1024.0, sink=4.0, tex_out=None,
          grade=None):
    sheet = json.loads(Path(sheet_json).read_text())
    o = sheet["origin"]
    origin = geo.Origin(o["lat"], o["lon"], o["alt"])
    d = Image.open(dem_tif)
    z = np.asarray(d).astype(np.float64)
    lon0, lat0 = d.tag_v2[33922][3], d.tag_v2[33922][4]
    dlon, dlat = d.tag_v2[33550][0], d.tag_v2[33550][1]
    rows, cols = z.shape
    lon1, lat1 = lon0 + cols * dlon, lat0 - rows * dlat
    # World extent covered by the DEM (shrunk a little so every vertex samples inside it).
    x_w = geo.wgs84_to_enu(origin, origin.lat, lon0 + 2 * dlon)[0]
    x_e = geo.wgs84_to_enu(origin, origin.lat, lon1 - 2 * dlon)[0]
    y_s = geo.wgs84_to_enu(origin, lat1 + 2 * dlat, origin.lon)[1]
    y_n = geo.wgs84_to_enu(origin, lat0 - 2 * dlat, origin.lon)[1]
    xs = np.arange(np.ceil(x_w / spacing), np.floor(x_e / spacing) + 1) * spacing
    ys = np.arange(np.floor(y_n / spacing), np.ceil(y_s / spacing) - 1, -1) * spacing  # north to south
    X, Y = np.meshgrid(xs, ys)
    # ENU -> lat/lon (small-angle around the origin is not enough at 40 km: use the exact transform per vertex)
    lat = np.empty_like(X)
    lon = np.empty_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            lat[i, j], lon[i, j], _ = geo.enu_to_wgs84(origin, X[i, j], Y[i, j], 0.0)
    import cv2
    col = ((lon - lon0) / dlon - 0.5).astype(np.float32)
    row = ((lat0 - lat) / dlat - 0.5).astype(np.float32)
    elev = cv2.remap(z.astype(np.float32), col, row, cv2.INTER_LINEAR).astype(np.float64)
    r2 = X ** 2 + Y ** 2
    Z = elev - origin.alt - r2 / (2 * R_EARTH)  # curvature drop (refraction ignored)
    # Sink the far mesh under the detailed heightmap so the two never z-fight.
    edge = np.maximum(np.abs(X), np.abs(Y))
    Z -= sink * np.clip((inner + 2 * spacing - edge) / (2 * spacing), 0, 1)
    n_r, n_c = X.shape
    pos = np.stack([X, Y, Z], -1).reshape(-1, 3)
    gy, gx = np.gradient(Z, spacing)
    N = np.stack([-gx, gy, np.ones_like(Z)], -1)
    N /= np.linalg.norm(N, axis=-1, keepdims=True)
    nimg = Image.open(naip_tif)
    nl0, nla0 = nimg.tag_v2[33922][3], nimg.tag_v2[33922][4]
    ndl, ndla = nimg.tag_v2[33550][0], nimg.tag_v2[33550][1]
    w, h = nimg.size
    U = (lon - nl0) / (ndl * w)
    V = (nla0 - lat) / (ndla * h)
    uv = np.stack([U, V], -1).reshape(-1, 2)
    r, c = np.meshgrid(np.arange(n_r - 1), np.arange(n_c - 1), indexing="ij")
    i0 = (r * n_c + c).ravel()
    i1, i2, i3 = i0 + 1, i0 + n_c, i0 + n_c + 1
    tris = np.concatenate([np.stack([i0, i2, i1], 1), np.stack([i1, i2, i3], 1)])
    # Drop triangles entirely inside the detailed terrain square.
    cx = pos[tris][:, :, 0]
    cy = pos[tris][:, :, 1]
    inside = np.all((np.abs(cx) < inner - 1) & (np.abs(cy) < inner - 1), axis=1)
    tris = tris[~inside]
    tex = tex_out or str(Path(out_glb).with_suffix(".png"))
    img = np.asarray(nimg.convert("RGB"))
    if grade:
        img = grade(img)
    Image.fromarray(img).save(tex)
    write_glb(out_glb, pos, N.reshape(-1, 3), uv, tris, base_color_png=tex, yup=False)
    return dict(verts=len(pos), tris=len(tris), extent_m=[float(xs[0]), float(xs[-1]), float(ys[-1]), float(ys[0])],
                z_range=[float(Z.min()), float(Z.max())])
