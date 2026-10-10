#!/usr/bin/env python3
"""pixi run fetch-data (tools/fetch_data.py) without the network: the raster
manifest against what the simulation reads and what is here, the dry run of
a fresh clone, official tiles made into a raster and checked, nothing
installed that does not match, and the rasters it cannot rebuild yet named
with the worlds that need them and how to get them, without failing the run."""
import contextlib
import hashlib
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
from urc.missions import COURSES, MISSIONS, autonomy  # noqa: E402

MANIFEST = json.loads((fetch_data.DATA_DIR / fetch_data.MANIFEST).read_text())
STEP = 0.001  # [deg] the made-up raster's pixel
TRANSFORM = (-110.79, STEP, 0.0, 38.43, 0.0, -STEP)  # its corner and pixel (GDAL geotransform)
LINK = "link it from a teammate's clone: rover_sim/tools/link_data.sh <that clone> (README, Simulation)"


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
    (data directory, {url: body} of the tiles)."""
    data = Path(directory)
    (data / "dem").mkdir()
    count, height, width = bands.shape
    whole = raster_bytes(bands, TRANSFORM)
    (data / "whole.tif").write_bytes(whole)
    lon0, dlon, _, lat0, _, dlat = TRANSFORM
    bodies, urls = {}, []
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
    (data / fetch_data.MANIFEST).write_text(json.dumps({"rasters": [entry]}))
    return data, bodies


def add_unofficial(data, name, worlds, required=True):
    """Adds to the manifest in data a raster dem/<name>.tif that fetch-data cannot rebuild (its provenance lists
    tiles, not exportImage requests), read by `worlds`. Returns its bytes, as a teammate's clone would have it."""
    body = raster_bytes(dem_band() + 7.0, TRANSFORM)
    (data / "dem" / f"{name}.json").write_text(json.dumps({"source_tiles": ["https://example.test/lidar.tif"]}))
    manifest = json.loads((data / fetch_data.MANIFEST).read_text())
    manifest["rasters"].append({
        "path": f"dem/{name}.tif", "provenance": f"dem/{name}.json", "required": required, "official": False,
        "worlds": worlds, "rebuild": "a made-up script is planned", "bytes": len(body),
        "sha256": hashlib.sha256(body).hexdigest(), "content_sha256": "not checked here",
        "shape": [1, 6, 8], "dtype": "float32", "transform": list(TRANSFORM)})
    (data / fetch_data.MANIFEST).write_text(json.dumps(manifest))
    return body


def run(data, server, *flags, err=None):
    """fetch_data.main on data with the fake network: (exit code, printed lines); err (a StringIO) gets what it
    says on stderr."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err if err is not None else io.StringIO()):
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

    def test_the_rasters_it_cannot_rebuild_name_their_worlds_and_what_is_planned(self):
        """There is no data release: a raster that is not official has only a
        teammate's clone, and the manifest says which worlds need it (the
        required ones, read by the simulation) and what is planned."""
        self.assertNotIn("mirror", MANIFEST)
        names = {f"urc_{key}" for key in MISSIONS} | set(COURSES)
        for e in MANIFEST["rasters"]:
            with self.subTest(e["path"]):
                if e["official"]:
                    self.assertNotIn("worlds", e)
                    continue
                self.assertTrue(set(e["worlds"]) <= names, e["worlds"])
                self.assertEqual(bool(e["worlds"]), e["required"])
                self.assertTrue(e["rebuild"])
        lidar = fetch_data.entry_of(autonomy.DEM_PATH)
        self.assertEqual((lidar["official"], lidar["worlds"]), (False, ["urc_autonomy"]))
        self.assertIn("USGS lidar", lidar["rebuild"])

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
            self.assertIn("dem/route_area_lidar_0p5m.tif: missing, urc_autonomy needs it; fetch-data cannot rebuild it "
                          "from official sources yet (a script that rebuilds it from the USGS lidar its provenance "
                          f"lists is planned); {LINK}", lines)


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
                self.assertEqual(sorted(server.asked), sorted(bodies))
                got, transform = dem.read_raster(data / "dem" / "made_up.tif")
                np.testing.assert_array_equal(got, bands.astype(np.float32))
                self.assertEqual(transform, TRANSFORM)
                self.assertEqual(sorted(p.name for p in (data / "dem").iterdir()), ["made_up.json", "made_up.tif"])
                self.assertEqual(run(data, Server({}), "--verify"), (0, ["dem/made_up.tif: verified"]))

    def test_other_pixels_from_the_source_fail_and_install_nothing(self):
        data, bodies = case(self.tmp.name, dem_band())
        first = next(url for url in bodies if "row=0&col=0" in url)
        bodies[first] = raster_bytes(dem_band(3, 4) + 1.0, TRANSFORM)
        err = io.StringIO()
        code, lines = run(data, Server(bodies), err=err)
        self.assertEqual((code, lines), (1, ["dem/made_up.tif: FAILED, the official source now returns other pixels "
                                             "than rasters.json lists"]))
        self.assertEqual(sorted(p.name for p in (data / "dem").iterdir()), ["made_up.json"])
        self.assertIn("1 raster(s) not fetched (FAILED above); a clone that has them can link them here: "
                      "rover_sim/tools/link_data.sh <that clone>", err.getvalue())

    def test_nothing_is_installed_when_the_source_fails(self):
        data, bodies = case(self.tmp.name, naip_bands())
        bodies = {url: body for url, body in bodies.items() if "row=0&col=0" not in url}
        code, lines = run(data, Server(bodies))
        self.assertEqual(code, 1)
        self.assertTrue(lines[-1].startswith("dem/made_up.tif: FAILED, the official source failed (HTTP Error 404"),
                        lines)
        self.assertEqual(sorted(p.name for p in (data / "dem").iterdir()), ["made_up.json"])

    def test_a_raster_here_is_not_fetched_and_verify_catches_a_wrong_one(self):
        data, bodies = case(self.tmp.name, dem_band())
        (data / "dem" / "made_up.tif").write_bytes(raster_bytes(dem_band(), TRANSFORM))
        server = Server(bodies)
        self.assertEqual(run(data, server), (0, ["dem/made_up.tif: present"]))
        self.assertEqual(server.asked, [])
        (data / "dem" / "made_up.tif").write_bytes(raster_bytes(dem_band() + 0.5, TRANSFORM))
        self.assertEqual(run(data, server, "--verify"), (1, ["dem/made_up.tif: DIFFERS"]))
        (data / "dem" / "made_up.tif").unlink()
        self.assertEqual(run(data, server, "--verify"), (1, ["dem/made_up.tif: MISSING"]))


class CannotRebuild(unittest.TestCase):
    """A raster fetch-data cannot rebuild yet (not official): when it is
    missing, every mode names the worlds that need it, what is planned and
    how to get it, asks the network nothing and does not fail the run; it
    fails only when a raster it can fetch was not fetched."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.data, self.bodies = case(self.tmp.name, dem_band())
        self.lidar = add_unofficial(self.data, "lidar", ["urc_made_up"])
        self.line = ("dem/lidar.tif: missing, urc_made_up needs it; fetch-data cannot rebuild it from official sources "
                     f"yet (a made-up script is planned); {LINK}")

    def test_it_is_named_and_the_others_fetched(self):
        server, err = Server(self.bodies), io.StringIO()
        code, lines = run(self.data, server, err=err)
        self.assertEqual((code, lines), (0, ["dem/made_up.tif: fetched from its official source, content verified",
                                             self.line]))
        self.assertEqual(sorted(server.asked), sorted(self.bodies))
        self.assertEqual(err.getvalue(), "fetch-data: 1 raster(s) it cannot rebuild yet are not here (above); without "
                                         "them sim-worlds skips urc_made_up and the tests that need it skip\n")

    def test_the_dry_run_and_verify_name_it_without_failing(self):
        server = Server(self.bodies)
        self.assertEqual(run(self.data, server, "--dry-run"),
                         (0, ["dem/made_up.tif: missing, would fetch from the official source, 4 request(s)",
                              self.line]))
        (self.data / "dem" / "made_up.tif").write_bytes(raster_bytes(dem_band(), TRANSFORM))
        self.assertEqual(run(self.data, server, "--verify"), (0, ["dem/made_up.tif: verified", self.line]))
        self.assertEqual(server.asked, [])

    def test_it_fails_the_run_only_with_a_raster_it_can_fetch(self):
        bodies = {url: body for url, body in self.bodies.items() if "row=0&col=0" not in url}
        code, lines = run(self.data, Server(bodies))
        self.assertEqual(code, 1)
        self.assertEqual(lines[1:], [self.line])

    def test_from_a_teammates_clone_it_is_present_and_verified(self):
        (self.data / "dem" / "lidar.tif").write_bytes(self.lidar)
        (self.data / "dem" / "made_up.tif").write_bytes(raster_bytes(dem_band(), TRANSFORM))
        self.assertEqual(run(self.data, Server({})), (0, ["dem/made_up.tif: present", "dem/lidar.tif: present"]))
        self.assertEqual(run(self.data, Server({}), "--verify"),
                         (0, ["dem/made_up.tif: verified", "dem/lidar.tif: verified"]))

    def test_one_no_world_reads_only_with_all(self):
        add_unofficial(self.data, "research_only", [], required=False)
        (self.data / "dem" / "made_up.tif").write_bytes(raster_bytes(dem_band(), TRANSFORM))
        err = io.StringIO()
        code, lines = run(self.data, Server({}), "--all", err=err)
        self.assertEqual((code, lines[2]), (0, "dem/research_only.tif: missing, no world reads it; fetch-data cannot "
                                               f"rebuild it from official sources yet (a made-up script is planned); "
                                               f"{LINK}"))
        self.assertEqual(len(run(self.data, Server({}))[1]), 2)
        self.assertIn("2 raster(s) it cannot rebuild yet are not here (above); without them sim-worlds skips "
                      "urc_made_up", err.getvalue())

if __name__ == "__main__":
    unittest.main()
