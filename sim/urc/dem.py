"""GeoTIFF DEMs: real terrain (USGS 3DEP, fetched by sim/tools/fetch_dem.py)
in, every world's dem.tif out.

The DEMs are geographic rasters: EPSG:4326, PixelIsArea, so pixel (col, row)
covers lon0 + [col, col + 1] * dlon, lat0 - [row, row + 1] * dlat and its
value is the elevation at the pixel centre (NAVD88 metres for 3DEP; add
NAVD88_TO_WGS84 for the ellipsoidal heights a GNSS receiver reports).
to_heightfield() resamples one onto a terrain.Heightfield in a mission's
layout frame (metres east/north of a geo.Origin); write_geotiff() writes a
Heightfield as one.
"""
from dataclasses import dataclass

import cv2
import numpy as np
from PIL import Image, TiffImagePlugin, TiffTags

from . import geo, terrain

# TIFF tags: GeoTIFF's georeferencing, and GDAL's metadata (3DEP files carry it).
MODEL_PIXEL_SCALE = 33550
MODEL_TIEPOINT = 33922
GEO_KEY_DIRECTORY = 34735
GEO_DOUBLE_PARAMS = 34736
GEO_ASCII_PARAMS = 34737
GDAL_METADATA = 42112
# GeoKeyDirectory keys and the values these DEMs must have.
GEOKEYS = {1024: (2, "geographic model"), 1025: (1, "PixelIsArea"), 2048: (4326, "WGS 84")}
# WGS84 ellipsoidal height (what a GNSS receiver and Gazebo's NavSat report,
# geo.Origin.alt) minus NAVD88 height (the DEMs' elevations) at the URC sites
# (38.40-38.43 N, 110.76-110.80 W). NOAA VDatum, queried 2026-10-06: GEOID18
# N = -20.11 m, plus NAD83(2011) -> WGS84(G2139) at epoch 2027.4 (URC 2027),
# -0.79 m; -20.89 to -20.94 m at every world's origin, VDatum's uncertainty
# 0.06 m.
NAVD88_TO_WGS84 = -20.91  # [m]


@dataclass
class DEM:
    z: np.ndarray  # float32 elevations [m], row 0 north, column 0 west
    lon0: float  # west edge of column 0 [deg]
    lat0: float  # north edge of row 0 [deg]
    dlon: float  # pixel size [deg]
    dlat: float

    @property
    def bounds(self):
        """((south, west), (north, east)) of the pixel edges [deg], like rules.ROUTE_AREA."""
        rows, cols = self.z.shape
        return (self.lat0 - rows * self.dlat, self.lon0), (self.lat0, self.lon0 + cols * self.dlon)

    def pixel(self, lat, lon):
        """Fractional (row, col) of a position, in pixel-centre coordinates."""
        return ((self.lat0 - np.asarray(lat, float)) / self.dlat - 0.5,
                (np.asarray(lon, float) - self.lon0) / self.dlon - 0.5)

    def height(self, lat, lon):
        """Bilinear elevation [m] at (lat, lon); raises outside the pixel centres."""
        row, col = self.pixel(lat, lon)
        rows, cols = self.z.shape
        if np.any((row < 0) | (row > rows - 1) | (col < 0) | (col > cols - 1)):
            raise ValueError(f"({lat}, {lon}) is outside the DEM {self.bounds}")
        r0 = np.minimum(np.floor(row).astype(int), rows - 2)
        c0 = np.minimum(np.floor(col).astype(int), cols - 2)
        fr, fc = row - r0, col - c0
        z = self.z  # float32, promoted to float64 by the float64 weights
        h = (z[r0, c0] * (1 - fc) * (1 - fr) + z[r0, c0 + 1] * fc * (1 - fr)
             + z[r0 + 1, c0] * (1 - fc) * fr + z[r0 + 1, c0 + 1] * fc * fr)
        return float(h) if np.ndim(h) == 0 else h


def read_geotiff(path):
    """A single-band float GeoTIFF in EPSG:4326 with PixelIsArea (what the 3DEP
    exportImage service and write_geotiff write)."""
    with Image.open(path) as img:
        tags = img.tag_v2
        z = np.asarray(img, dtype=np.float32)
        keys = tags[GEO_KEY_DIRECTORY]
        scale, tie = tags[MODEL_PIXEL_SCALE], tags[MODEL_TIEPOINT]
    found = {keys[i]: keys[i + 3] for i in range(4, 4 + 4 * keys[3], 4)}
    for key, (value, meaning) in GEOKEYS.items():
        if found.get(key) != value:
            raise ValueError(f"{path}: GeoKey {key} is {found.get(key)}, expected {value} ({meaning})")
    if tie[:2] != (0.0, 0.0):
        raise ValueError(f"{path}: the tiepoint must be at raster (0, 0), got {tie}")
    if not np.all(np.isfinite(z)) or z.min() < -1000:
        raise ValueError(f"{path}: the DEM has no-data pixels")
    return DEM(z, lon0=tie[3], lat0=tie[4], dlon=scale[0], dlat=scale[1])


def to_heightfield(dem, origin: geo.Origin, size, n, center):
    """The DEM resampled (bilinear) onto an n x n Heightfield of `size` metres
    around `center`, in the layout frame of `origin`: x east, y north,
    z = elevation - origin.alt.

    Layout metres map to lat/lon through a quadratic fit of geo.enu_to_wgs84
    (an affine map is 0.1 m off at the corners of 2 km: a degree of longitude
    shortens northwards); the fit is within ~1e-10 deg (10 um) and lets
    cv2.remap resample millions of samples at once. remap interpolates at
    1/32 pixel: samples are within 3 cm of DEM.height on the route-area DEM.
    """
    hf = terrain.Heightfield(size, n, center=center)

    def terms(x, y):
        x, y = (np.asarray(x, float) - center[0]) / size, (np.asarray(y, float) - center[1]) / size
        return np.stack([np.ones_like(x), x, y, x * x, x * y, y * y], axis=-1)

    def exact(ticks):
        xy = np.array([(center[0] + dx * size, center[1] + dy * size) for dx in ticks for dy in ticks])
        return terms(xy[:, 0], xy[:, 1]), np.array([geo.enu_to_wgs84(origin, x, y)[:2] for x, y in xy])

    coef, *_ = np.linalg.lstsq(*exact((-0.5, 0.0, 0.5)), rcond=None)
    A, check = exact((-0.25, 0.25))
    residual = np.abs(A @ coef - check).max()
    if residual > 1e-9:  # 0.1 mm: the terrain is too large for the fit
        raise ValueError(f"lat/lon fit is off by {residual:.2e} deg over {size} m")
    X, Y = hf.grid()
    lat, lon = np.moveaxis(terms(X, Y) @ coef, -1, 0)
    row, col = dem.pixel(lat, lon)
    rows, cols = dem.z.shape
    if row.min() < 0 or col.min() < 0 or row.max() > rows - 1 or col.max() > cols - 1:
        raise ValueError(f"a {size} m terrain around {center} does not fit in the DEM {dem.bounds}")
    z = cv2.remap(dem.z, col.astype(np.float32), row.astype(np.float32), cv2.INTER_LINEAR)
    hf.z = z.astype(float) - origin.alt
    return hf


def write_geotiff(hf, path, origin: geo.Origin):
    """A float32 GeoTIFF DEM of a Heightfield in the layout frame of `origin`
    (EPSG:4326, PixelIsArea), elevations = origin altitude + z. Degrees per
    pixel are taken at the origin, so the raster is accurate to a few
    centimetres over a kilometre."""
    lat_n, _, _ = geo.enu_to_wgs84(origin, 0, hf.res)
    _, lon_e, _ = geo.enu_to_wgs84(origin, hf.res, 0)
    dlat, dlon = lat_n - origin.lat, lon_e - origin.lon
    half = hf.size / 2 + hf.res / 2  # outer edge of the corner pixels
    lat0, _, _ = geo.enu_to_wgs84(origin, 0, hf.center[1] + half)
    _, lon0, _ = geo.enu_to_wgs84(origin, hf.center[0] - half, 0)
    keys = [v for key, (value, _) in GEOKEYS.items() for v in (key, 0, 1, value)]
    ifd = TiffImagePlugin.ImageFileDirectory_v2()
    for tag, value, kind in ((MODEL_PIXEL_SCALE, (dlon, dlat, 0.0), TiffTags.DOUBLE),
                             (MODEL_TIEPOINT, (0.0, 0.0, 0.0, lon0, lat0, 0.0), TiffTags.DOUBLE),
                             (GEO_KEY_DIRECTORY, (1, 1, 0, len(GEOKEYS), *keys), TiffTags.SHORT)):
        ifd[tag] = value
        ifd.tagtype[tag] = kind
    Image.fromarray((hf.z + origin.alt).astype(np.float32)).save(path, tiffinfo=ifd)
