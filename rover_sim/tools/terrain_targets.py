#!/usr/bin/env python3
"""Roughness targets for the synthetic ground (design spec 5.4, 9.2): the
real 0.5 m lidar painted by the soil-map rules of design 5.3
(urc.landscape.Soils), measured per ground type.

Two real squares are painted: the Autonomy square (2048 m round the URC
square mile, on the route-area lidar, the world-level targets too) and the
largest square of the MDRS-area lidar (it holds the Farb slickrock and more
badland floors). In each, every ground type's plane-detrended RMS height
(urc.landscape.window_rms) is measured in non-overlapping windows of 4, 8,
16 and 32 m that are at least 80 % that type; a type needs 20 windows at a
scale. block_field is not on the soil map: it is measured on the research
block-field windows I and J as a whole (provisional, mixed slopes).

Writes sim/data/research/terrain_targets.json: p25/p50/p75/p90 [cm] per
type and scale, the world-level percentiles, the paint rules and the DEMs
used. Deterministic; about a minute.
"""
import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR / "tools"))
sys.path.insert(0, str(SIM_DIR))
import make_relief_swatches as swatches  # noqa: E402
from urc import dem, geo, landscape, terrains  # noqa: E402
from urc.missions import autonomy  # noqa: E402

OUT = SIM_DIR / "data" / "research" / "terrain_targets.json"
RES = 0.5  # [m]
SCALES = (4, 8, 16, 32)  # [m]
PERCENTILES = (25, 50, 75, 90)
SHARE = 0.8  # a window counts for a type if this much of it is that type (design 5.4)
MIN_WINDOWS = 20
MDRS_SQUARE = 2800.0  # [m] the largest round square inside the MDRS-area lidar (2.93 x 3.49 km)
CENTRAL = 600.0  # [m] the world-level "central" square (design 5.4)


def percentiles(values):
    return [round(float(np.percentile(values, p)) * 100, 2) for p in PERCENTILES]


def autonomy_square():
    """The Autonomy world's layout square on the 0.5 m lidar, and its WGS84 frame (autonomy.make_site's)."""
    d3 = dem.read_geotiff(autonomy.DEM_PATH)
    centre = geo.Origin(*autonomy.SQUARE_MILE_CENTER, d3.height(*autonomy.SQUARE_MILE_CENTER))
    lat, lon, _ = geo.enu_to_wgs84(centre, *autonomy.C2_FROM_CENTER)
    origin = geo.Origin(lat, lon, d3.height(lat, lon))
    lidar = dem.read_geotiff(swatches.ROUTE_LIDAR)
    n = int(round(autonomy.SIZE / RES)) + 1
    return dem.to_heightfield(lidar, origin, autonomy.SIZE, n, autonomy.CENTER), origin


def mdrs_square():
    """The largest round square of the MDRS-area lidar, centred on it, and its frame."""
    lidar = dem.read_geotiff(swatches.MDRS_LIDAR)
    (south, west), (north, east) = lidar.bounds
    lat, lon = (south + north) / 2, (west + east) / 2
    origin = geo.Origin(lat, lon, lidar.height(lat, lon))
    n = int(round(MDRS_SQUARE / RES)) + 1
    return dem.to_heightfield(lidar, origin, MDRS_SQUARE, n, (0.0, 0.0)), origin


def per_type(samples, hf, raster, legend):
    """Append each type's window RMS values [m] per scale to samples[key][scale]."""
    for scale in SCALES:
        rms = landscape.window_rms(hf.z, RES, scale)
        for i in np.unique(raster):
            share = landscape.window_share(raster, i, RES, scale)
            samples.setdefault(legend[i].key, {}).setdefault(scale, []).append(rms[share >= SHARE])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    legend = landscape.Legend()
    samples, cover, squares = {}, {}, {}
    for name, (hf, origin) in (("autonomy_square", autonomy_square()), ("mdrs_area_square", mdrs_square())):
        raster = landscape.from_ssurgo(hf, origin, legend=legend)
        values, counts = np.unique(raster, return_counts=True)
        cover[name] = {legend[i].key: round(float(c) / raster.size, 4) for i, c in zip(values, counts)}
        per_type(samples, hf, raster, legend)
        squares[name] = {"size_m": hf.size, "samples": hf.n, "origin": asdict(origin)}
        if name == "autonomy_square":
            k = int(round(CENTRAL / RES / 2))
            middle = hf.n // 2
            world = {"whole": {str(s): percentiles(landscape.window_rms(hf.z, RES, s)) for s in SCALES},
                     "central_600m": {str(s): percentiles(landscape.window_rms(
                         hf.z[middle - k:middle + k, middle - k:middle + k], RES, s)) for s in SCALES}}
        del hf, raster
    types = {}
    for key, scales in sorted(samples.items()):
        entry = {}
        for scale, parts in scales.items():
            values = np.concatenate(parts)
            if len(values) >= MIN_WINDOWS:
                entry[str(scale)] = percentiles(values)
                entry.setdefault("windows", {})[str(scale)] = int(len(values))
        if entry:
            types[key] = dict(entry, source="soil map")
    blocks = {}
    for name in ("boulder_field_I", "boulder_ridge_J"):  # the raw ground of the windows, not a residual
        path = swatches.ROUTE_LIDAR if name.endswith("I") else swatches.MDRS_LIDAR
        centre, size = swatches.window_centre(name, path)
        d = dem.read_geotiff(path)
        nx, ny = (int(round(s / RES)) + 1 for s in size)
        z = dem.to_grid(d, geo.Origin(*centre, 0.0), (np.arange(nx) - (nx - 1) / 2) * RES,
                        ((ny - 1) / 2 - np.arange(ny)) * RES)
        for scale in SCALES:
            blocks.setdefault(scale, []).append(landscape.window_rms(z, RES, scale))
    types["block_field"] = dict({str(s): percentiles(np.concatenate(v)) for s, v in blocks.items()},
                                windows={str(s): int(sum(len(p) for p in v)) for s, v in blocks.items()},
                                source="research windows I and J as a whole (provisional, mixed slopes)")
    rules = {mukey: asdict(unit) for mukey, unit in landscape.SSURGO_UNITS.items()}
    out = {
        "what": "Roughness targets for the synthetic ground (design spec 5.4): plane-detrended RMS height [cm] in "
                "non-overlapping square windows, p25/p50/p75/p90, per ground type of the real 0.5 m lidar painted "
                "by the soil-map rules (urc.landscape.Soils), and for the whole real Autonomy square. Made by "
                "sim/tools/terrain_targets.py.",
        "percentiles": list(PERCENTILES), "scales_m": list(SCALES), "res_m": RES, "window_share": SHARE,
        "min_windows": MIN_WINDOWS,
        "dems": [str(p.relative_to(SIM_DIR.parent)) for p in (swatches.ROUTE_LIDAR, swatches.MDRS_LIDAR)],
        "squares": squares,
        "paint_rules": {"soil_map": str(landscape.SOILS_PATH.relative_to(SIM_DIR.parent)),
                        "slope_smooth_m": landscape.SLOPE_SMOOTH, "gentle_deg": landscape.GENTLE_DEG,
                        "steep_deg": landscape.STEEP_DEG, "crest": list(landscape.CREST),
                        "hollow": list(landscape.HOLLOW), "cap": list(landscape.CAP), "units": rules},
        "cover": cover, "world": world, "types": types,
        "catalogue": sorted(terrains.TYPES)}
    args.out.write_text(json.dumps(out, indent=1) + "\n")
    for key, entry in types.items():
        print(f"{key:14s}", {s: entry.get(str(s)) for s in SCALES})
    print("world", world)
    print("cover", cover)


if __name__ == "__main__":
    main()
