#!/usr/bin/env python3
"""Fetch a USGS 3DEP elevation GeoTIFF for a lat/lon box.

    python sim/tools/fetch_dem.py            # the URC route-finding area (urc_autonomy)
    python sim/tools/fetch_dem.py --bbox W S E N --out sim/data/dem/area.tif [--resolution-m 1] [--force]

Asks the 3DEP Elevation ImageServer (public domain, no key) for float32
elevations (NAVD88 metres) in EPSG:4326, sized for about `resolution-m`
metres per pixel along each axis. The service returns pixels that are square
in degrees (the east spacing; north-south they are ~1.27x longer at this
latitude) and widens the latitude range to fill the requested rows, so the
file covers a little more than the box. It is stored deflate-compressed with the
floating-point predictor (lossless, ~40 % of the raw size) next to a .json
with its provenance, which the mission sheets quote. With --force a download
replaces the old file only once it reads back as a valid DEM, and the .json
is written last.
"""
import argparse
import datetime
import io
import json
import math
import os
import sys
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image, TiffImagePlugin

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from urc import dem  # noqa: E402

SERVICE = "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage"
ROUTE_AREA_BBOX = (-110.790, 38.407, -110.764, 38.429)  # west, south, east, north: around rules.ROUTE_AREA
ROUTE_AREA_DEM = SIM_DIR / "data" / "dem" / "route_area_3dep.tif"
# GeoTIFF georeferencing (pixel scale, tiepoint, GeoKey directory and its
# double/ASCII parameters) and GDAL's metadata: everything to keep on re-save.
GEOTIFF_TAGS = (dem.MODEL_PIXEL_SCALE, dem.MODEL_TIEPOINT, dem.GEO_KEY_DIRECTORY, dem.GEO_DOUBLE_PARAMS,
                dem.GEO_ASCII_PARAMS, dem.GDAL_METADATA)
PREDICTOR, FLOATING_POINT = 317, 3
# Rule-of-thumb degree lengths (within 0.3 % of the WGS84 ellipsoid here); the
# route-area DEM was requested with them, so the default run asks for the same
# 2268 x 2442 pixels.
M_PER_DEG_LON_EQUATOR = 111_320.0  # [m] times cos(latitude)
M_PER_DEG_LAT = 111_000.0  # [m]


def request_size(bbox, resolution):
    """Pixels across and down for about `resolution` metres per pixel."""
    west, south, east, north = bbox
    width = (east - west) * M_PER_DEG_LON_EQUATOR * math.cos(math.radians((south + north) / 2))
    return round(width / resolution), round((north - south) * M_PER_DEG_LAT / resolution)


def export_url(bbox, size):
    """The exportImage request: the bbox at full precision (repr), so the file
    covers what was asked for and the provenance's url and bbox agree."""
    query = {"bbox": ",".join(repr(float(v)) for v in bbox), "bboxSR": 4326, "imageSR": 4326,
             "size": f"{size[0]},{size[1]}", "format": "tiff", "pixelType": "F32",
             "noDataInterpretation": "esriNoDataMatchAny", "interpolation": "RSP_BilinearInterpolation", "f": "image"}
    return SERVICE + "?" + urllib.parse.urlencode(query, safe=",")


def save_compressed(img, path):
    """Write a float GeoTIFF deflate-compressed, keeping its GeoTIFF tags, to
    `path` and return it as a dem.DEM. It replaces `path` only once the
    pixels read back bit for bit, the tags survive and dem.read_geotiff
    accepts it (georeferencing, no gaps): a bad download never overwrites a
    good DEM."""
    info = TiffImagePlugin.ImageFileDirectory_v2()
    for tag in GEOTIFF_TAGS:
        if tag in img.tag_v2:
            info[tag] = img.tag_v2[tag]
            info.tagtype[tag] = img.tag_v2.tagtype[tag]
    info[PREDICTOR] = FLOATING_POINT
    tmp = Path(path).with_suffix(".tmp.tif")
    img.save(tmp, compression="tiff_deflate", tiffinfo=info)
    try:
        with Image.open(tmp) as back:
            same = np.array_equal(np.asarray(back).view(np.uint32), np.asarray(img).view(np.uint32))
            tags = all(back.tag_v2.get(t) == img.tag_v2.get(t) for t in GEOTIFF_TAGS)
        if not (same and tags):
            raise RuntimeError(f"re-reading {tmp} gave different {'tags' if same else 'pixels'}")
        d = dem.read_geotiff(tmp)
    except Exception:
        tmp.unlink()
        raise
    os.replace(tmp, path)
    return d


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bbox", nargs=4, type=float, metavar=("WEST", "SOUTH", "EAST", "NORTH"),
                        default=ROUTE_AREA_BBOX, help="degrees, WGS84 (default: the URC route-finding area)")
    parser.add_argument("--out", type=Path, default=ROUTE_AREA_DEM, help=f"default: {ROUTE_AREA_DEM}")
    parser.add_argument("--resolution-m", type=float, default=1.0, help="metres per pixel (default 1)")
    parser.add_argument("--force", action="store_true", help="overwrite an existing file")
    args = parser.parse_args()
    meta = args.out.with_suffix(".json")
    existing = [str(p) for p in (args.out, meta) if p.exists()]
    if existing and not args.force:
        sys.exit(f"{' and '.join(existing)}: already there; pass --force to download again")
    size = request_size(args.bbox, args.resolution_m)
    url = export_url(args.bbox, size)
    print(f"fetching {size[0]} x {size[1]} px: {url}", flush=True)
    with urllib.request.urlopen(url, timeout=600) as response:
        body = response.read()
        kind = response.headers.get("Content-Type", "")
    if "tif" not in kind:
        sys.exit(f"the service answered {kind}: {body[:500].decode(errors='replace')}")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with Image.open(io.BytesIO(body)) as img:
        img.load()
        d = save_compressed(img, args.out)
    (south, west), (north, east) = d.bounds
    provenance = {"source": "USGS 3D Elevation Program (3DEP), 3DEPElevation ImageServer exportImage",
                  "license": "public domain (USGS)", "fetched": datetime.date.today().isoformat(), "url": url,
                  "bbox": list(args.bbox), "resolution_m": args.resolution_m,
                  "bounds": {"south": south, "west": west, "north": north, "east": east},
                  "pixels": list(d.z.shape[::-1]), "pixel_deg": d.dlon, "crs": "EPSG:4326, PixelIsArea",
                  "vertical": "NAVD88 metres", "elevation_range_m": [float(d.z.min()), float(d.z.max())]}
    meta.write_text(json.dumps(provenance, indent=2) + "\n")  # last: it describes the file now in place
    print(f"wrote {args.out} ({args.out.stat().st_size / 1e6:.1f} MB), elevations "
          f"{d.z.min():.1f}-{d.z.max():.1f} m, bounds {south:.5f}..{north:.5f} N, {west:.5f}..{east:.5f} E")


if __name__ == "__main__":
    main()
