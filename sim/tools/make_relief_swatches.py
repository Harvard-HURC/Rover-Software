#!/usr/bin/env python3
"""Relief swatches: high-pass residuals of real 0.5 m lidar windows, the
micro-relief synthetic worlds borrow (design spec 5.4, 9.2): pixi run sim-relief.

Each window of the research (sim/data/research/mdrs_terrain_measurements.json,
windows_def: cells of a NAD83(2011) UTM 12N 0.5 m grid) is resampled from its
lidar DEM onto a 0.5 m grid in its own east/north frame (urc.dem.to_grid,
with the datum shift the lidar files carry), with a margin of 3 sigma round
it; the swatch is that ground minus its Gaussian blur of sigma, the margin
cut off again so the blur saw real ground on every side.

Writes sim/data/relief/<swatch>.npz: z (float16 metres, H x W, row 0 north:
the first window), z_1, ... (further windows), res_m, sigma_m, source (JSON:
per window its name, DEM file, centre lat/lon and size) and rms_cm (JSON:
the median plane-detrended RMS in windows of 4, 8, 16 and 32 m over all its
windows). The files are deterministic: the same DEMs give the same bytes.
"""
import argparse
import io
import json
import sys
import zipfile
from pathlib import Path

import numpy as np

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from urc import dem, geo, landscape, terrain  # noqa: E402

DATA = SIM_DIR / "data"
RESEARCH = DATA / "research" / "mdrs_terrain_measurements.json"
ROUTE_LIDAR = DATA / "dem" / "route_area_lidar_0p5m.tif"
MDRS_LIDAR = DATA / "dem" / "mdrs_area_lidar2018_0p5m.tif"
# The research grid: cell (row, col) of 0.5 m with the corner of cell (0, 0) at this UTM 12N position.
GRID_ORIGIN = (517462.5, 4252000.0)  # [m] easting, northing (mdrs_terrain_measurements.json, about)
GRID_RES = 0.5  # [m]
UTM_ZONE = 12
RES = 0.5  # [m] swatch grid
MARGIN_SIGMAS = 3.0
RMS_SCALES = (4, 8, 16, 32)  # [m]
# Swatch: (sigma [m], [(research window, lidar DEM)]) (design spec 5.4). Window I pokes ~6 m past the
# MDRS-area file's east edge, so it comes from the route-area file; J lies south of the route-area file.
# Slickrock is cut with sigma 2 m, not the design's 8 m: window H's relief at 8-32 m is its ledges (risers and
# macro shape), and the slickrock the soil map paints, its gentle benches, has the spectrum of H at sigma 2 m
# (median RMS 4 : 8 : 16 m = 1 : 1.9 : 2.9 painted, 1 : 2.0 : 3.0 at sigma 2 m, 1 : 2.6 : 5.8 at 8 m).
SWATCHES = {
    "sand_sheet": (16.0, [("sand_sheet_D", ROUTE_LIDAR), ("sand_sheet_C", ROUTE_LIDAR)]),
    "shale_pediment": (16.0, [("shale_pediment_A", MDRS_LIDAR)]),
    "badland": (2.0, [("badland_banded_B", MDRS_LIDAR)]),
    "block_field": (8.0, [("boulder_field_I", ROUTE_LIDAR), ("boulder_ridge_J", MDRS_LIDAR)]),
    "slickrock": (2.0, [("slickrock_ledges_H", ROUTE_LIDAR)]),
}


def window_centre(name, dem_path):
    """WGS84 (lat, lon) of a research window's centre in the frame of a
    lidar file (its NAD83(2011) UTM position plus the file's datum shift),
    and its (width, height) [m]."""
    row, col, height, width = json.loads(RESEARCH.read_text())["windows_def"][name]
    easting = GRID_ORIGIN[0] + (col + width / 2) * GRID_RES
    northing = GRID_ORIGIN[1] - (row + height / 2) * GRID_RES
    lat, lon = dem.utm_to_wgs84(easting, northing, UTM_ZONE)
    shift = json.loads(dem_path.with_suffix(".json").read_text())["datum_shift_applied"]
    return (lat + shift["dlat_deg"], lon + shift["dlon_deg"]), (width * GRID_RES, height * GRID_RES)


def residual(d, centre, size, sigma):
    """The window's ground minus its blur of sigma, on a RES grid (row 0 north)."""
    origin = geo.Origin(*centre, 0.0)
    margin = MARGIN_SIGMAS * sigma
    nx, ny = (int(round(s / RES)) + 1 for s in size)
    m = int(round(margin / RES))
    xs = (np.arange(-m, nx + m) - (nx - 1) / 2) * RES
    ys = ((ny - 1) / 2 - np.arange(-m, ny + m)) * RES
    z = dem.to_grid(d, origin, xs, ys)
    return (z - terrain.blur(z, sigma / RES))[m:m + ny, m:m + nx]


def rms_table(grids):
    """{scale: median window RMS [cm]} over all windows of all grids."""
    return {str(s): round(float(np.median(np.concatenate([landscape.window_rms(g, RES, s) for g in grids]))) * 100,
                          2) for s in RMS_SCALES}


def save_npz(path, arrays):
    """np.savez_compressed with fixed member dates, so equal arrays give equal bytes."""
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
        for name, value in arrays.items():
            buffer = io.BytesIO()
            np.lib.format.write_array(buffer, np.asanyarray(value), allow_pickle=False)
            info = zipfile.ZipInfo(f"{name}.npy", date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            zf.writestr(info, buffer.getvalue())


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("swatches", nargs="*", choices=[[]] + sorted(SWATCHES), help="default: all")
    parser.add_argument("--out", type=Path, default=landscape.RELIEF_DIR)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    dems = {}
    for key in args.swatches or sorted(SWATCHES):
        sigma, windows = SWATCHES[key]
        grids, source = [], []
        for name, path in windows:
            d = dems.setdefault(path, dem.read_geotiff(path))
            centre, size = window_centre(name, path)
            grids.append(residual(d, centre, size, sigma))
            source.append({"window": name, "dem": str(path.relative_to(SIM_DIR.parent)),
                           "centre_lat_lon": [round(centre[0], 7), round(centre[1], 7)],
                           "size_m": list(size)})
        arrays = {"z" if k == 0 else f"z_{k}": g.astype(np.float16) for k, g in enumerate(grids)}
        table = rms_table(grids)
        save_npz(args.out / f"{key}.npz", {**arrays, "res_m": RES, "sigma_m": sigma, "source": json.dumps(source),
                                           "rms_cm": json.dumps(table)})
        shapes = ", ".join(f"{g.shape[1]} x {g.shape[0]}" for g in grids)
        print(f"{key}: sigma {sigma} m, {shapes} samples, RMS cm {table}")


if __name__ == "__main__":
    main()
