#!/usr/bin/env python3
"""pixi run fetch-data (tools/fetch_data.py) without the network: the raster
manifest against what the simulation reads and what is here, the dry run of
a fresh clone, official tiles made into a raster and checked, the team's copy
as the fallback, and nothing installed that does not match."""
import contextlib
import io
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from worldfiles import SIM_DIR

sys.path.insert(0, str(SIM_DIR / "tools"))
import fetch_data  # noqa: E402
from urc import dem, farfield  # noqa: E402
from urc.missions import autonomy  # noqa: E402

MANIFEST = json.loads((fetch_data.DATA_DIR / fetch_data.MANIFEST).read_text())
STEP = 0.001  # [deg] the made-up raster's pixel
TRANSFORM = (-110.79, STEP, 0.0, 38.43, 0.0, -STEP)  # its corner and pixel (GDAL geotransform)
MIRROR = "https://mirror.test/"


def raster_bytes(bands, transform):
    """A GeoTIFF of bands (B, H, W) as fetch_data writes it."""
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "r.tif"
        fetch_data.write_geotiff(path, bands, fetch_data.geotags(transform))
        return path.read_bytes()


class Server:
    """The network: url -> body; it records what was asked and 404s the rest."""

    def __init__(self, bodies):
        self.bodies, self.asked = dict(bodies), []

    def __call__(self, url):
        self.asked.append(url)
        if url not in self.bodies:
            raise OSError(f"HTTP Error 404: {url}")
        return self.bodies[url]


def case(directory, bands):
    """A data directory holding the manifest and provenance of one raster,
    dem/made_up.tif = bands (B, H, W), made of 2 x 2 exportImage tiles. Returns
    (data directory, {url: body} of the tiles and of the team's copy)."""
    data = Path(directory)
    (data / "dem").mkdir()
    count, height, width = bands.shape
    whole = raster_bytes(bands, TRANSFORM)
    (data / "whole.tif").write_bytes(whole)
    lon0, dlon, _, lat0, _, dlat = TRANSFORM
    bodies, urls = {MIRROR + "made_up.tif": whole}, []
    for row in (0, height // 2):
        for col in (0, width // 2):
            tile = bands[:, row:row + height // 2, col:col + width // 2]
            url = f"https://example.test/exportImage?row={row}&col={col}"
            bodies[url] = raster_bytes(tile, (lon0 + col * dlon, dlon, 0.0, lat0 + row * dlat, 0.0, dlat))
            urls.append(url)
    (data / "dem" / "made_up.json").write_text(json.dumps({"requests": urls}))
    entry = {"path": "dem/made_up.tif", "provenance": "dem/made_up.json", "required": True, "official": True,
             "bytes": len(whole), "sha256": fetch_data.file_sha256(data / "whole.tif"),
             "content_sha256": fetch_data.content_sha256(data / "whole.tif"), "shape": list(bands.shape),
             "dtype": str(bands.dtype), "transform": list(TRANSFORM)}
    (data / "whole.tif").unlink()
    (data / fetch_data.MANIFEST).write_text(json.dumps({"mirror": MIRROR, "rasters": [entry]}))
    return data, bodies


def run(data, server, *flags):
    """fetch_data.main on data with the fake network: (exit code, printed lines)."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = fetch_data.main([*flags, "--data", str(data)], get=server)
    return code, out.getvalue().splitlines()


def dem_band(height=6, width=8):
    return (1350.0 + np.arange(height * width, dtype=np.float32).reshape(1, height, width) / 7.0).astype(np.float32)


def naip_bands(height=6, width=8):
    return (np.arange(4 * height * width).reshape(4, height, width) * 37 % 256).astype(np.uint8)


class Manifest(unittest.TestCase):
    def test_every_raster_the_simulation_reads_is_required(self):
        required = {e["path"] for e in MANIFEST["rasters"] if e["required"]}
        for path in (*dem.SITE_DEMS, farfield.FAR_DEM, farfield.FAR_IMAGERY, autonomy.DEM_PATH, autonomy.NAIP_PATH):
            self.assertIn(f"{Path(path).parent.name}/{Path(path).name}", required)

    def test_each_raster_has_its_provenance_and_an_official_one_its_requests(self):
        for e in MANIFEST["rasters"]:
            with self.subTest(e["path"]):
                provenance = json.loads((fetch_data.DATA_DIR / e["provenance"]).read_text())
                requests = fetch_data.requests_of(provenance)
                self.assertEqual(bool(requests) and all("exportImage" in url for url in requests), e["official"])

    def test_the_rasters_here_are_the_manifests(self):
        here = [e for e in MANIFEST["rasters"] if (fetch_data.DATA_DIR / e["path"]).exists()]
        if not here:
            self.skipTest("no rasters here (pixi run fetch-data)")
        for e in here:
            with self.subTest(e["path"]):
                self.assertTrue(fetch_data.matches(e, fetch_data.DATA_DIR / e["path"]))


class DryRun(unittest.TestCase):
    def test_a_fresh_clone_says_what_it_would_fetch_and_touches_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            data = Path(d)
            shutil.copy(fetch_data.DATA_DIR / fetch_data.MANIFEST, data)
            for e in MANIFEST["rasters"]:
                (data / e["provenance"]).parent.mkdir(exist_ok=True)
                shutil.copy(fetch_data.DATA_DIR / e["provenance"], data / e["provenance"])
            before = sorted(data.rglob("*"))
            server = Server({})
            code, lines = run(data, server, "--dry-run")
            self.assertEqual((code, server.asked, sorted(data.rglob("*"))), (0, [], before))
            self.assertEqual(len(lines), sum(e["required"] for e in MANIFEST["rasters"]))
            self.assertIn("dem/route_area_3dep.tif: missing, would fetch from the official source, 1 request(s)", lines)
            self.assertIn("imagery/route_area_naip2024.tif: missing, would fetch from the official source, "
                          "9 request(s)", lines)
            self.assertIn("dem/route_area_lidar_0p5m.tif: missing, would fetch from the team's copy", lines)


class Fetch(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_official_tiles_make_the_raster(self):
        for bands in (dem_band(), naip_bands()):
            with self.subTest(dtype=str(bands.dtype)), tempfile.TemporaryDirectory() as d:
                data, bodies = case(d, bands)
                server = Server(bodies)
                code, lines = run(data, server)
                self.assertEqual(code, 0, lines)
                self.assertNotIn(MIRROR + "made_up.tif", server.asked)
                got, transform = dem.read_raster(data / "dem" / "made_up.tif")
                np.testing.assert_array_equal(got, bands.astype(np.float32))
                self.assertEqual(transform, TRANSFORM)
                self.assertEqual(sorted(p.name for p in (data / "dem").iterdir()), ["made_up.json", "made_up.tif"])
                self.assertEqual(run(data, Server({}), "--verify"), (0, ["dem/made_up.tif: verified"]))

    def test_other_pixels_from_the_source_bring_the_teams_copy(self):
        data, bodies = case(self.tmp.name, dem_band())
        first = next(url for url in bodies if "row=0&col=0" in url)
        bodies[first] = raster_bytes(dem_band(3, 4) + 1.0, TRANSFORM)
        code, lines = run(data, Server(bodies))
        self.assertEqual(code, 0, lines)
        self.assertIn("dem/made_up.tif: the official source now returns other pixels; trying the team's copy", lines)
        self.assertEqual((data / "dem" / "made_up.tif").read_bytes(), bodies[MIRROR + "made_up.tif"])

    def test_nothing_is_installed_when_the_source_and_the_copy_fail(self):
        data, bodies = case(self.tmp.name, naip_bands())
        bodies = {url: body for url, body in bodies.items() if "row=0&col=0" not in url}
        bodies[MIRROR + "made_up.tif"] = b"not the raster"
        code, lines = run(data, Server(bodies))
        self.assertEqual(code, 1)
        self.assertTrue(lines[-1].startswith("dem/made_up.tif: FAILED"), lines)
        self.assertEqual(sorted(p.name for p in (data / "dem").iterdir()), ["made_up.json"])

    def test_a_raster_here_is_not_fetched_and_verify_catches_a_wrong_one(self):
        data, bodies = case(self.tmp.name, dem_band())
        (data / "dem" / "made_up.tif").write_bytes(bodies[MIRROR + "made_up.tif"])
        server = Server(bodies)
        self.assertEqual(run(data, server), (0, ["dem/made_up.tif: present"]))
        self.assertEqual(server.asked, [])
        (data / "dem" / "made_up.tif").write_bytes(raster_bytes(dem_band() + 0.5, TRANSFORM))
        self.assertEqual(run(data, server, "--verify"), (1, ["dem/made_up.tif: DIFFERS"]))


if __name__ == "__main__":
    unittest.main()
