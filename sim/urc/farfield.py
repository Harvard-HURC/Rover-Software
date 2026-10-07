"""The far field (design spec D14): the real landscape around a world, out to
the Henry Mountains and Factory Butte, as one visual-only mesh in place of
the old horizon plane.

build() writes a static model with one GLB visual: the USGS 3DEP DEM around
the URC sites (65 x 80 km, sim/data/dem/far_dem_3dep_60km.tif) on a SPACING
grid, dropped by the Earth's curvature, with a hole under the world's own
terrain and its seam sunk below the terrain's edge; and apron.json, the
mesh's heights round the terrain, so that the fly camera keeps clear of
the far field beyond the edge (plugins/fly_camera.cpp). Its texture is the
NAIP 2021 overview of the same bounds
(sim/data/imagery/naip2021_far_60km.tif), brought to NAIP 2024's colour
(OVERVIEW_TO_NAIP2024) and boosted like the near orthophoto, shared by
every world through Media (far_texture); only the geometry is per world.
Measured (M): +260-300 MB
per gz process (render prototype); +0.9 ms per 1280 x 720 frame (4.05 ->
5.0 ms, median of 3 runs, a 1 km terrain from 45 m up; this module); the
Henry Mountains and Factory Butte where they belong (tests/test_render.py).
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from . import appearance, dem, geo, meshes, sdf, textures

DATA = Path(__file__).resolve().parents[1] / "data"
FAR_DEM = DATA / "dem" / "far_dem_3dep_60km.tif"
FAR_IMAGERY = DATA / "imagery" / "naip2021_far_60km.tif"
SPACING = 120.0  # [m] grid (M: 0.72 M triangles over the DEM's extent, render prototype)
SINK = 4.0  # [m] the seam's depth below the terrain's edge, so the two never z-fight (M)
EARTH_RADIUS = 6371000.0  # [m] mean
REFRACTION = 0.13  # standard terrestrial refraction coefficient: lowers the curvature drop by 13 % (R, geodesy)
FIT_ERROR = 1e-6  # [deg] (~0.1 m) the most the cubic lat/lon fit may be off (A)
APRON_REACH = 400.0  # [m] apron.json covers this far beyond the terrain's edge: the fly camera's margin (100 m,
# viewers.FlyParams) and the far-field triangles round it
# The NAIP 2021 overview against NAIP 2024 over the Autonomy square (64^2 texels of 32 m), per linear channel:
# the ratio of their medians (M, 2026-10-07). Unscaled, the boosted overview was 17 % darker than the boosted
# drape beside it (CIE76 11.8 at the seam).
OVERVIEW_TO_NAIP2024 = (1.297, 1.151, 1.024)


def far_texture(path, raster, gain, boost):
    """The far field's texture: the NAIP overview `raster` as a PNG, each
    linear channel times `gain` (to the near imagery's colour), boosted like
    the near orthophoto (an appearance.Boost), north up, edge to edge."""
    bands, _ = dem.read_raster(raster)
    lin = textures.srgb_to_linear(np.moveaxis(bands[:3], 0, -1)) * np.asarray(gain, np.float32)
    Image.fromarray(textures.linear_to_srgb(boost.apply(lin))).save(path, format="PNG")


def build(models_dir, media, name, origin: geo.Origin, terrain, blend_m=2 * SPACING):
    """Write the far-field model `name` (models_dir/name) for a world whose
    origin is `origin` (its altitude ellipsoidal, as NavSat's: the DEM's
    NAVD88 elevations get dem.NAVD88_TO_WGS84) and whose terrain is
    `terrain` (its visual heightmap as a Heightfield in world coordinates);
    returns the name.

    Triangles wholly over the terrain square are dropped, and every vertex
    within blend_m of the square lies SINK below the lowest terrain within a
    grid step of it, blending into the real landscape farther out: the
    triangles that cross the edge reach a grid step inside it, and the mesh
    must not show through there, on synthetic terrain or on a real DEM (the
    far DEM's 120 m triangles span the lidar's washes, which lie up to 19 m
    lower, measured on Autonomy when its seam was only sunk SINK)."""
    d = dem.read_geotiff(FAR_DEM)
    (south, west), (north, east) = d.bounds
    margin = 2 * max(d.dlat, d.dlon)  # [deg] keep every vertex inside the DEM's pixel centres
    corners = [geo.wgs84_to_enu(origin, lat, lon) for lat in (south + margin, north - margin)
               for lon in (west + margin, east - margin)]
    x0, x1 = max(c[0] for c in corners[::2]), min(c[0] for c in corners[1::2])
    y0, y1 = max(c[1] for c in corners[:2]), min(c[1] for c in corners[2:])
    xs = np.arange(math.ceil(x0 / SPACING), math.floor(x1 / SPACING) + 1) * SPACING
    ys = np.arange(math.floor(y1 / SPACING), math.ceil(y0 / SPACING) - 1, -1) * SPACING  # north to south
    X, Y = np.meshgrid(xs, ys)
    span = 2 * max(abs(xs[0]), abs(xs[-1]), abs(ys[0]), abs(ys[-1]))
    lat, lon = geo.lonlat_fit(origin, span, degree=3, tolerance=FIT_ERROR)(X, Y)
    row, col = d.pixel(lat, lon)
    elevation = cv2.remap(d.z, col.astype(np.float32), row.astype(np.float32), cv2.INTER_LINEAR)
    drop = (1 - REFRACTION) * (X ** 2 + Y ** 2) / (2 * EARTH_RADIUS)
    Z = elevation.astype(np.float64) + dem.NAVD88_TO_WGS84 - origin.alt - drop
    center, half = terrain.center, terrain.size / 2
    outside = np.hypot(np.maximum(np.abs(X - center[0]) - half, 0), np.maximum(np.abs(Y - center[1]) - half, 0))
    k = 2 * int(math.ceil(SPACING / terrain.res)) + 1
    low = cv2.erode(terrain.z.astype(np.float32), np.ones((k, k), np.uint8))
    seam = _lookup(terrain, low, X, Y)  # the lowest ground under any triangle of the vertex's
    w = np.clip(outside / blend_m, 0, 1)
    w = w * w * (3 - 2 * w)
    Z = (1 - w) * (seam - SINK) + w * Z
    N = appearance.surface_normals(Z, SPACING)
    bands, (ilon0, idlon, _, ilat0, _, minus_idlat) = dem.read_raster(FAR_IMAGERY)
    UV = np.stack([(lon - ilon0) / (idlon * bands.shape[2]), (lat - ilat0) / (minus_idlat * bands.shape[1])], -1)
    rows, cols = X.shape
    r, c = np.meshgrid(np.arange(rows - 1), np.arange(cols - 1), indexing="ij")
    i0 = (r * cols + c).ravel()
    F = np.concatenate([np.stack([i0, i0 + cols, i0 + 1], 1), np.stack([i0 + 1, i0 + cols, i0 + cols + 1], 1)])
    V = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
    over = np.all((np.abs(V[F, 0] - center[0]) < half - 1) & (np.abs(V[F, 1] - center[1]) < half - 1), axis=1)
    F = F[~over]
    directory = Path(models_dir) / name
    (directory / "meshes").mkdir(parents=True, exist_ok=True)
    meshes.write_glb(directory / "meshes" / "farfield.glb", V, F, N.reshape(-1, 3), UV.reshape(-1, 2))
    _write_apron(directory / APRON, X, Y, Z, center, half)
    root, model = sdf.model_root(name, static=True)
    link = sdf.link(model, "link")
    texture = media.texture("farfield_naip2021", far_texture, FAR_IMAGERY, OVERVIEW_TO_NAIP2024,
                            appearance.NAIP2024_BOOST)
    sdf.visual(link, "farfield", sdf.mesh(sdf.model_uri(name, "meshes", "farfield.glb")), cast_shadows=False,
               albedo=texture, roughness=1.0)
    sdf.write_model(models_dir, name, root, "Far-field landscape around a URC world (visual only).")
    return name


APRON = "apron.json"  # beside meshes/: the far field's heights round the terrain (the fly camera's floor)


def _write_apron(path, X, Y, Z, center, half):
    """The far-field grid's heights within APRON_REACH of the terrain
    square, as plugins/fly_camera.cpp reads them: the vertices row-major
    from the north-west, and the mesh's own split of each cell, from its
    south-west corner to its north-east one."""
    reach = half + APRON_REACH + SPACING
    rows = np.flatnonzero(np.abs(Y[:, 0] - center[1]) <= reach)
    cols = np.flatnonzero(np.abs(X[0] - center[0]) <= reach)
    z = Z[np.ix_(rows, cols)]
    path.write_text(json.dumps({
        "format": "rover-apron/1", "spacing": SPACING, "x0": float(X[0, cols[0]]), "y0": float(Y[rows[0], 0]),
        "rows": len(rows), "cols": len(cols), "z": [round(float(v), 3) for v in z.ravel()],
        "note": "far-field vertex heights [m, world frame], row 0 north, column 0 west, SPACING apart; each cell "
                "is split from its south-west to its north-east corner, as the mesh"}) + "\n")


def _lookup(terrain, values, x, y):
    """`values` on terrain's sample grid at world (x, y), bilinear, clamped
    to its edge outside it."""
    col = np.clip((np.asarray(x) - terrain.center[0] + terrain.size / 2) / terrain.res, 0, terrain.n - 1)
    row = np.clip((terrain.center[1] + terrain.size / 2 - np.asarray(y)) / terrain.res, 0, terrain.n - 1)
    return cv2.remap(values, col.astype(np.float32), row.astype(np.float32), cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REPLICATE).astype(np.float64)
