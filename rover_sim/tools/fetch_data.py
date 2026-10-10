#!/usr/bin/env python3
"""Fetch the git-ignored terrain rasters: pixi run fetch-data [--all] [--dry-run] [--verify].

data/rasters.json lists every raster under data/ that git ignores, with its
size, its SHA-256, the SHA-256 of its content (the pixels and georeferencing
urc.dem.read_raster reads) and the tracked .json that says where it came
from. A missing raster is fetched from its official source when it is one or
more ImageServer exportImage requests (`official`; its .json lists them as
url, urls or requests): each tile is placed on the raster's grid by its own
georeferencing, the mosaic is written as a deflate GeoTIFF and kept only if
its content matches. The others fetch-data cannot rebuild yet (the 0.5 m
lidar DEMs and their source map, made by processing that is not in the
repository; the NAIP 2021 route mosaic, whose .json gives only a URL
template): for each one missing it names the worlds that need it (`worlds`)
and what is planned (`rebuild`), and a teammate's clone lends it
(tools/link_data.sh <that clone>). There is no other copy to fetch from;
without such a raster sim-worlds skips the worlds that need it, and sim,
drive and the tests' skips say why (world_missing).
By default only the rasters the simulation reads (`required`); --all also the
ones only the research and tools/make_relief_swatches.py read. Rasters already
here are left alone (--verify checks them against the manifest); the tracked
.json files are never written, so a fresh clone needs no --force.

Exit status: 0 when every listed raster fetch-data can fetch is here, fetched
or already present (those it cannot rebuild yet may be missing; it names
them); 1 when one it can fetch was not fetched (with --verify: a raster here
differs from the manifest, or one it can fetch is missing); 2 for a usage
error.
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
LINK = "link it from a teammate's clone: rover_sim/tools/link_data.sh <that clone> (README, Simulation)"
STATS_BLOCKS = 16  # blocks per side of a raster's stats
# How far the stats of a lidar rebuild that is not bit-identical may be from the manifest's: 1 mm for the DEMs, a
# thousandth of a code for the source map. Rounding differences between platforms (OpenCV's blur, GDAL's bilinear
# kernel) are a few float32 steps (0.1 mm at 1400 m); a wrong tile, shift or step is centimetres (A).
TOLERANCE = 1e-3


def load_manifest(data_dir=DATA_DIR):
    """data_dir's rasters.json."""
    return json.loads((Path(data_dir) / MANIFEST).read_text())


def entry_of(path, data_dir=DATA_DIR):
    """The manifest's entry for the raster at path (under data_dir, or relative to it), or None."""
    path = Path(path)
    if path.is_absolute():
        path = path.relative_to(data_dir)
    return next((e for e in load_manifest(data_dir)["rasters"] if e["path"] == path.as_posix()), None)


def cannot_fetch(entry):
    """What to do about a raster fetch-data cannot rebuild yet (not `official`) when it is missing."""
    return f"fetch-data cannot rebuild it from official sources yet ({entry['rebuild']}); {LINK}"


def needed_by(entry):
    """Which generated worlds read the raster, as a clause."""
    worlds = entry["worlds"]
    return f"{', '.join(worlds)} need{'s' if len(worlds) == 1 else ''} it" if worlds else "no world reads it"


def shown(path):
    """path as a user sees it, from the repository's root when it is in it."""
    path = Path(path).absolute()
    return path.relative_to(SIM_DIR.parent).as_posix() if path.is_relative_to(SIM_DIR.parent) else str(path)


def world_missing(world, data_dir=DATA_DIR):
    """Why the generated world (urc_<mission>, proving_ground) is not here, or None: a raster it reads that
    fetch-data cannot rebuild yet is missing (the manifest's `worlds`), so sim-worlds skips it."""
    for entry in load_manifest(data_dir)["rasters"]:
        path = Path(data_dir) / entry["path"]
        if world in entry.get("worlds", ()) and not path.exists():
            return f"{world} needs {shown(path)}, which is not here: {cannot_fetch(entry)}"
    return None


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def content_digest(bands, transform):
    """SHA-256 of a raster's float32 bands (B, H, W) and geotransform, as dem.read_raster returns them."""
    h = hashlib.sha256(np.ascontiguousarray(bands).tobytes())
    h.update(repr(tuple(float(v) for v in transform)).encode())
    return h.hexdigest()


def content_sha256(path):
    """SHA-256 of what the simulation reads from a raster: dem.read_raster's
    float32 bands (B, H, W) and geotransform. Any lossless encoding of the same
    pixels and georeferencing has the same one."""
    return content_digest(*dem.read_raster(path))


def stats(bands):
    """A raster's stats: the min and max of its float32 bands (B, H, W) and the mean of each of STATS_BLOCKS x
    STATS_BLOCKS blocks per band (rows and columns split as np.array_split splits them), rounded to 1e-6."""
    def r(v):
        return round(float(v), 6)
    return {"min": r(bands.min()), "max": r(bands.max()),
            "block_means": [[[r(block.mean(dtype=np.float64)) for block in np.array_split(rows, STATS_BLOCKS, axis=1)]
                             for rows in np.array_split(band, STATS_BLOCKS, axis=0)] for band in bands]}


def within(got, want, tolerance=TOLERANCE):
    """Whether stats `got` are those of `want` within tolerance, number by number."""
    a = np.array([got["min"], got["max"], *np.ravel(got["block_means"])], float)
    b = np.array([want["min"], want["max"], *np.ravel(want["block_means"])], float)
    return a.shape == b.shape and bool(np.all(np.abs(a - b) <= tolerance))


def verdict(entry, path):
    """How the raster at path matches entry's content: "content verified" (the same pixels and georeferencing),
    "content within TOLERANCE ..." (a lidar rebuild that is not bit-identical but has the same shape, the same
    georeferencing and the manifest's stats within TOLERANCE), or None."""
    try:
        bands, transform = dem.read_raster(path)
    except Exception:  # not a raster dem.read_raster accepts
        return None
    if content_digest(bands, transform) == entry["content_sha256"]:
        return "content verified"
    if ("stats" in entry and list(bands.shape) == entry["shape"] and list(transform) == entry["transform"]
            and within(stats(bands), entry["stats"])):
        return f"content within {TOLERANCE} of rasters.json's stats, not bit-identical"
    return None


def matches(entry, path):
    """Whether the raster at path is entry's: the same file, or (rebuilt from
    its official source, so encoded anew) the same content, or a lidar rebuild
    within TOLERANCE of its stats."""
    return file_sha256(path) == entry["sha256"] or verdict(entry, path) is not None


def check(entry, path):
    """--verify's word for the raster at path: verified (with how, when it is within TOLERANCE), DIFFERS, MISSING."""
    if not path.exists():
        return "MISSING"
    if file_sha256(path) == entry["sha256"]:
        return "verified"
    how = verdict(entry, path)
    return "DIFFERS" if how is None else "verified" if how == "content verified" else f"verified, {how}"


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


def fetch(entry, data_dir, get=download, log=print):
    """Fetch one raster from its official source into data_dir; True once it is in place and its content
    verified."""
    path = data_dir / entry["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(f".{path.name}.part")
    try:
        try:
            mosaic(entry, json.loads((data_dir / entry["provenance"]).read_text()), get, part)
        except Exception as e:  # any failure of the source
            log(f"{entry['path']}: FAILED, the official source failed ({e})")
            return False
        if content_sha256(part) != entry["content_sha256"]:
            log(f"{entry['path']}: FAILED, the official source now returns other pixels than rasters.json lists")
            return False
        os.replace(part, path)
        log(f"{entry['path']}: fetched from its official source, content verified")
        return True
    finally:
        part.unlink(missing_ok=True)


def main(argv=None, get=download):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                     epilog=__doc__[__doc__.index("Exit status"):],  # the docstring's last paragraph
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--all", action="store_true",
                        help="also the rasters only the research and the relief tool read")
    parser.add_argument("--dry-run", action="store_true", help="say what would be fetched; fetch nothing")
    parser.add_argument("--verify", action="store_true",
                        help="check the rasters here against rasters.json; fetch nothing")
    parser.add_argument("--data", type=Path, default=DATA_DIR, help=f"data directory (default {DATA_DIR})")
    args = parser.parse_args(argv)
    failed, missing = 0, []
    for entry in load_manifest(args.data)["rasters"]:
        if not (args.all or entry["required"]):
            continue
        path = args.data / entry["path"]
        if not (entry["official"] or path.exists()):  # nothing to fetch it from: say so, whatever the mode
            print(f"{entry['path']}: missing, {needed_by(entry)}; {cannot_fetch(entry)}")
            missing.append(entry)
        elif args.verify:
            state = check(entry, path)
            print(f"{entry['path']}: {state}")
            failed += not state.startswith("verified")
        elif path.exists():
            print(f"{entry['path']}: present")
        elif args.dry_run:
            count = len(requests_of(json.loads((args.data / entry["provenance"]).read_text())))
            print(f"{entry['path']}: missing, would fetch from the official source, {count} request(s)")
        elif not fetch(entry, args.data, get):
            failed += 1
    sys.stdout.flush()  # the summary after the lines above, also in a log
    if failed and not args.verify:
        print(f"fetch-data: {failed} raster(s) not fetched (FAILED above); a clone that has them can link them "
              "here: rover_sim/tools/link_data.sh <that clone> (README, Simulation)", file=sys.stderr)
    if missing:
        worlds = sorted({world for entry in missing for world in entry["worlds"]})
        print(f"fetch-data: {len(missing)} raster(s) it cannot rebuild yet are not here (above)"
              + (f"; without them sim-worlds skips {', '.join(worlds)} and the tests that need "
                 f"{'it' if len(worlds) == 1 else 'them'} skip" if worlds else ""),
              file=sys.stderr)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
