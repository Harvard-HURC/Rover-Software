"""The far field (design spec D14): the real landscape around a world, out to
the Henry Mountains and Factory Butte, as one visual-only mesh in place of
the old horizon plane.

build() writes a static model with one GLB visual: the USGS 3DEP DEM around
the URC sites (65 x 80 km, sim/data/dem/far_dem_3dep_60km.tif) on a SPACING
grid, dropped by the Earth's curvature, with a hole under the world's own
terrain and its seam sunk below the terrain's edge. Its texture is the NAIP
overview of the same bounds (sim/data/imagery/naip2021_far_60km.tif),
boosted like the near orthophoto and shared by every world through Media
(far_texture); only the geometry is per world. Measured in the render
prototype (M): +260-300 MB per gz process, frame cost within noise, the
Henry Mountains and Factory Butte where they belong.
"""
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


def far_texture(path, raster=str(FAR_IMAGERY), boost=appearance.NAIP2021_BOOST):
    """The far field's texture: the NAIP overview as a PNG, boosted like the
    near orthophoto (appearance.Boost), north up, edge to edge."""
    bands, _ = dem.read_raster(raster)
    lin = textures.srgb_to_linear(np.moveaxis(bands[:3], 0, -1))
    Image.fromarray(textures.linear_to_srgb(boost.apply(lin))).save(path, format="PNG")


def build(models_dir, media, name, origin: geo.Origin, size, center=(0.0, 0.0), terrain=None,
          blend_m=2 * SPACING):
    """Write the far-field model `name` (models_dir/name) for a world whose
    origin is `origin` (its altitude ellipsoidal, as NavSat's: the DEM's
    NAVD88 elevations get dem.NAVD88_TO_WGS84) and whose terrain is the
    `size` square around world `center`; returns the name.

    Triangles wholly over the terrain square are dropped. With `terrain`
    (the world's heightmap as a Heightfield in world coordinates) every
    vertex within blend_m of the square lies SINK below the lowest terrain
    within a grid step of it, blending into the real landscape farther out,
    so the mesh never shows through the terrain, a synthetic one included;
    without it the seam is only sunk SINK."""
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
    lat, lon = _lonlat(origin, xs, ys)(X, Y)
    row, col = d.pixel(lat, lon)
    elevation = cv2.remap(d.z, col.astype(np.float32), row.astype(np.float32), cv2.INTER_LINEAR)
    drop = (1 - REFRACTION) * (X ** 2 + Y ** 2) / (2 * EARTH_RADIUS)
    Z = elevation.astype(np.float64) + dem.NAVD88_TO_WGS84 - origin.alt - drop
    half = size / 2
    outside = np.hypot(np.maximum(np.abs(X - center[0]) - half, 0), np.maximum(np.abs(Y - center[1]) - half, 0))
    if terrain is not None:
        k = 2 * int(math.ceil(SPACING / terrain.res)) + 1
        low = cv2.erode(terrain.z.astype(np.float32), np.ones((k, k), np.uint8))
        seam = _lookup(terrain, low, X, Y)  # the lowest ground under any triangle of the vertex's
        w = np.clip(outside / blend_m, 0, 1)
        w = w * w * (3 - 2 * w)
        Z = (1 - w) * (seam - SINK) + w * Z
    else:
        Z -= SINK * np.clip(1 - outside / (2 * SPACING), 0, 1)
    gy, gx = np.gradient(Z, SPACING)
    N = np.stack([-gx, gy, np.ones_like(Z)], axis=-1)  # rows run south: dz/dnorth = -gy
    N /= np.linalg.norm(N, axis=-1, keepdims=True)
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
    root, model = sdf.model_root(name, static=True)
    link = sdf.link(model, "link")
    sdf.visual(link, "farfield", sdf.mesh(sdf.model_uri(name, "meshes", "farfield.glb")), cast_shadows=False,
               albedo=media.texture("farfield_naip2021", far_texture), roughness=1.0)
    sdf.write_model(models_dir, name, root, "Far-field landscape around a URC world (visual only).")
    return name


def _lonlat(origin, xs, ys):
    """(x, y) world -> (lat, lon) over the grid: a cubic fit of
    geo.enu_to_wgs84 (its quartic terms are ~1 cm at 40 km), checked."""
    span = max(abs(xs[0]), abs(xs[-1]), abs(ys[0]), abs(ys[-1]))

    def terms(x, y):
        x, y = np.asarray(x, float) / span, np.asarray(y, float) / span
        return np.stack([x ** i * y ** j for i in range(4) for j in range(4 - i)], axis=-1)

    def exact(ticks):
        xy = np.array([(a, b) for a in np.linspace(xs[0], xs[-1], ticks) for b in np.linspace(ys[-1], ys[0], ticks)])
        return terms(xy[:, 0], xy[:, 1]), np.array([geo.enu_to_wgs84(origin, x, y)[:2] for x, y in xy])

    coef, *_ = np.linalg.lstsq(*exact(9), rcond=None)
    A, check = exact(8)
    error = np.abs(A @ coef - check).max()
    if error > FIT_ERROR:
        raise ValueError(f"far-field lat/lon fit is off by {error:.2e} deg")
    return lambda x, y: np.moveaxis(terms(x, y) @ coef, -1, 0)


def _lookup(terrain, values, x, y):
    """`values` on terrain's sample grid at world (x, y), bilinear, clamped
    to its edge outside it."""
    col = np.clip((np.asarray(x) - terrain.center[0] + terrain.size / 2) / terrain.res, 0, terrain.n - 1)
    row = np.clip((terrain.center[1] + terrain.size / 2 - np.asarray(y)) / terrain.res, 0, terrain.n - 1)
    return cv2.remap(values, col.astype(np.float32), row.astype(np.float32), cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REPLICATE).astype(np.float64)
