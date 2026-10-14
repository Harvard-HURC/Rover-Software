"""Resample the NAIP 2021 mosaic onto a world's terrain square (Terra global diffuse map).

python make_ortho.py sheet.json out.png N [--gain g] [--gamma y]
Texel (r, c) covers world x in -size/2 + [c, c+1] * size/N, y in size/2 - [r, r+1] * size/N
(row 0 north, column 0 west), the layout Terra uses for its global diffuse map.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, "/Users/alarion239/Desktop/Rover/sim")
from urc import geo  # noqa: E402

Image.MAX_IMAGE_PIXELS = None
NAIP = Path("/Users/alarion239/Desktop/Rover/sim/data/imagery/naip2021_route_area.tif")


def world_to_lonlat_fit(origin, size):
    def terms(x, y):
        x, y = np.asarray(x, float) / size, np.asarray(y, float) / size
        return np.stack([np.ones_like(x), x, y, x * x, x * y, y * y], axis=-1)
    ticks = (-0.5, -0.25, 0.0, 0.25, 0.5)
    xy = np.array([(a * size, b * size) for a in ticks for b in ticks])
    ll = np.array([geo.enu_to_wgs84(origin, x, y)[:2] for x, y in xy])
    coef, *_ = np.linalg.lstsq(terms(xy[:, 0], xy[:, 1]), ll, rcond=None)
    err = np.abs(terms(xy[:, 0], xy[:, 1]) @ coef - ll).max()
    assert err < 1e-8, err
    return lambda x, y: np.moveaxis(terms(x, y) @ coef, -1, 0)


def build(sheet_path, n, size=None):
    sheet = json.loads(Path(sheet_path).read_text())
    o = sheet["origin"]
    origin = geo.Origin(o["lat"], o["lon"], o["alt"])
    size = size or sheet["terrain"]["size_m"]
    fit = world_to_lonlat_fit(origin, size)
    s = size / n
    c = -size / 2 + (np.arange(n) + 0.5) * s
    X, Y = np.meshgrid(c, -c)  # row 0 north
    lat, lon = fit(X, Y)
    img = Image.open(NAIP)
    tags = img.tag_v2
    lon0, lat0 = tags[33922][3], tags[33922][4]
    dlon, dlat = tags[33550][0], tags[33550][1]
    src = np.asarray(img)
    col = ((lon - lon0) / dlon - 0.5).astype(np.float32)
    row = ((lat0 - lat) / dlat - 0.5).astype(np.float32)
    # The source is oversampled (0.44-0.56 m) relative to most outputs: area-average first.
    out = cv2.remap(src, col, row, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    return out


if __name__ == "__main__":
    sheet_path, out, n = sys.argv[1], sys.argv[2], int(sys.argv[3])
    a = build(sheet_path, n)
    Image.fromarray(a).save(out)
    print(out, a.shape, a.reshape(-1, 3).mean(axis=0))
