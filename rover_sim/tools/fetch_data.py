#!/usr/bin/env python3
"""Fetch the git-ignored terrain rasters: pixi run fetch-data [--all] [--dry-run] [--verify].

data/rasters.json lists every raster under data/ that git ignores, with its
size, its SHA-256, the SHA-256 of its content (the pixels and georeferencing
urc.dem.read_raster reads) and the tracked .json that says where it came
from. A missing raster is fetched:
- from its official source when it is one or more ImageServer exportImage
  requests (`official`; its .json lists them as url, urls or requests): each
  tile is placed on the raster's grid by its own georeferencing, the mosaic
  is written as a deflate GeoTIFF and kept only if its content matches;
- otherwise, or when the source fails or now returns other pixels, from the
  team's copy (`mirror`, a GitHub release), kept only if the file matches.
By default only the rasters the simulation reads (`required`); --all also the
ones only the research and tools/make_relief_swatches.py read. Rasters already
here are left alone (--verify checks them against the manifest); the tracked
.json files are never written, so a fresh clone needs no --force.
"""
import argparse
import hashlib
import json
import os
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image, TiffImagePlugin, TiffTags

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from urc import dem  # noqa: E402

DATA_DIR = SIM_DIR / "data"
MANIFEST = "rasters.json"  # in the data directory
TRIES = 3  # per download: the ImageServers answer HTTP 500 now and then
USER_AGENT = "Harvard-HURC Rover-Software fetch-data"


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def content_sha256(path):
    """SHA-256 of what the simulation reads from a raster: dem.read_raster's
    float32 bands (B, H, W) and geotransform. Any lossless encoding of the same
    pixels and georeferencing has the same one."""
    bands, transform = dem.read_raster(path)
    h = hashlib.sha256(np.ascontiguousarray(bands).tobytes())
    h.update(repr(tuple(float(v) for v in transform)).encode())
    return h.hexdigest()


def matches(entry, path):
    """Whether the raster at path is entry's: the same file, or (fetched from
    its official source, so encoded anew) the same content."""
    if file_sha256(path) == entry["sha256"]:
        return True
    try:
        return content_sha256(path) == entry["content_sha256"]
    except Exception:  # not a raster dem.read_raster accepts
        return False


def download(url):
    """The body at url, after up to TRIES attempts."""
    for attempt in range(TRIES):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": USER_AGENT}),
                                        timeout=600) as response:
                return response.read()
        except OSError:
            if attempt == TRIES - 1:
                raise
            time.sleep(5 * (attempt + 1))


def requests_of(provenance):
    """The exportImage requests a raster was made from, as its .json lists them."""
    if "url" in provenance:
        return [provenance["url"]]
    return list(provenance.get("urls") or provenance.get("requests") or [])


def geotags(transform):
    """GeoTIFF tags {tag: (value, type)} for a GDAL geotransform (EPSG:4326, PixelIsArea)."""
    lon0, dlon, _, lat0, _, dlat = transform
    keys = [v for key, (value, _) in dem.GEOKEYS.items() for v in (key, 0, 1, value)]
    return {dem.MODEL_PIXEL_SCALE: ((dlon, -dlat, 0.0), TiffTags.DOUBLE),
            dem.MODEL_TIEPOINT: ((0.0, 0.0, 0.0, lon0, lat0, 0.0), TiffTags.DOUBLE),
            dem.GEO_KEY_DIRECTORY: ((1, 1, 0, len(dem.GEOKEYS), *keys), TiffTags.SHORT)}


def write_geotiff(path, bands, tags):
    """bands (B, H, W), one float32 band or three or four uint8 bands (R, G,
    B(, NIR)), as a deflate GeoTIFF with tags {tag: (value, type)}. PIL writes
    four bands only as RGBA, the fourth marked alpha (ExtraSamples 2); the mark
    is patched to 0, unspecified, as in NAIP's own files: dem.read_raster
    refuses an alpha band."""
    count = len(bands)
    if count == 1:
        img = Image.fromarray(np.ascontiguousarray(bands[0], np.float32))
    else:
        img = Image.fromarray(np.ascontiguousarray(np.moveaxis(bands, 0, -1), np.uint8))
    info = TiffImagePlugin.ImageFileDirectory_v2()
    for tag, (value, kind) in tags.items():
        info[tag] = value
        info.tagtype[tag] = kind
    img.save(path, format="TIFF", compression="tiff_deflate", tiffinfo=info)
    if count == 4:
        entry = dem.EXTRA_SAMPLES.to_bytes(2, "little") + b"\x03\x00\x01\x00\x00\x00"
        data = Path(path).read_bytes()
        if data.count(entry + b"\x02\x00") != 1:
            raise RuntimeError(f"{path}: not one alpha mark to clear")
        Path(path).write_bytes(data.replace(entry + b"\x02\x00", entry + b"\x00\x00"))


def read_tile(body):
    """(bands, geotransform) of a GeoTIFF body."""
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "tile.tif"
        path.write_bytes(body)
        return dem.read_raster(path)


def mosaic(entry, provenance, get, out_path):
    """The raster rebuilt from its exportImage requests into out_path: each
    tile placed by its own georeferencing on the grid of entry["transform"]."""
    lon0, dlon, _, lat0, _, dlat = entry["transform"]
    out = np.zeros(entry["shape"], np.dtype(entry["dtype"]))
    covered = np.zeros(entry["shape"][1:], bool)
    for url in requests_of(provenance):
        bands, transform = read_tile(get(url))
        col, row = (transform[0] - lon0) / dlon, (transform[3] - lat0) / dlat
        if abs(col - round(col)) > 1e-3 or abs(row - round(row)) > 1e-3:
            raise ValueError(f"a tile lies off the raster's grid (column {col:.4f}, row {row:.4f}): {url}")
        col, row = round(col), round(row)
        _, h, w = bands.shape
        out[:, row:row + h, col:col + w] = bands
        covered[row:row + h, col:col + w] = True
    if not covered.all():
        raise ValueError(f"the requests leave {np.count_nonzero(~covered)} pixels uncovered")
    write_geotiff(out_path, out, geotags(entry["transform"]))


def fetch(entry, data_dir, mirror, get=download, log=print):
    """Fetch one raster into data_dir; True once it is in place and verified."""
    path = data_dir / entry["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(f".{path.name}.part")
    try:
        if entry["official"]:
            try:
                mosaic(entry, json.loads((data_dir / entry["provenance"]).read_text()), get, part)
                if content_sha256(part) == entry["content_sha256"]:
                    os.replace(part, path)
                    log(f"{entry['path']}: fetched from its official source, content verified")
                    return True
                log(f"{entry['path']}: the official source now returns other pixels; trying the team's copy")
            except Exception as e:  # any failure of the source: the team's copy is the fallback
                log(f"{entry['path']}: the official source failed ({e}); trying the team's copy")
        url = mirror + Path(entry["path"]).name
        try:
            part.write_bytes(get(url))
        except Exception as e:
            log(f"{entry['path']}: FAILED, no team's copy at {url} ({e})")
            return False
        if file_sha256(part) != entry["sha256"]:
            log(f"{entry['path']}: FAILED, the team's copy at {url} differs from rasters.json")
            return False
        os.replace(part, path)
        log(f"{entry['path']}: fetched from the team's copy, SHA-256 verified")
        return True
    finally:
        part.unlink(missing_ok=True)


def main(argv=None, get=download):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--all", action="store_true",
                        help="also the rasters only the research and the relief tool read")
    parser.add_argument("--dry-run", action="store_true", help="say what would be fetched; fetch nothing")
    parser.add_argument("--verify", action="store_true",
                        help="check the rasters here against rasters.json; fetch nothing")
    parser.add_argument("--data", type=Path, default=DATA_DIR, help=f"data directory (default {DATA_DIR})")
    args = parser.parse_args(argv)
    manifest = json.loads((args.data / MANIFEST).read_text())
    failed = 0
    for entry in manifest["rasters"]:
        if not (args.all or entry["required"]):
            continue
        path = args.data / entry["path"]
        if args.verify:
            state = "verified" if path.exists() and matches(entry, path) else "DIFFERS" if path.exists() else "MISSING"
            print(f"{entry['path']}: {state}")
            failed += state != "verified"
        elif path.exists():
            print(f"{entry['path']}: present")
        elif args.dry_run:
            if entry["official"]:
                count = len(requests_of(json.loads((args.data / entry["provenance"]).read_text())))
                print(f"{entry['path']}: missing, would fetch from the official source, {count} request(s)")
            else:
                print(f"{entry['path']}: missing, would fetch from the team's copy")
        elif not fetch(entry, args.data, manifest["mirror"], get):
            failed += 1
    if failed and not args.verify:
        sys.stdout.flush()  # the hint after the lines above, also in a log
        print(f"fetch-data: {failed} raster(s) not fetched; a clone that has them can link them here: "
              "rover_sim/tools/link_data.sh <that clone> (README, Simulation)", file=sys.stderr)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
