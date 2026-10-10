#!/usr/bin/env python3
"""pixi run fetch-data (tools/fetch_data.py) without the network: the raster
manifest against what the simulation reads and what is here, the dry run of
a fresh clone, official tiles made into a raster and checked, nothing
installed that does not match, the lidar rebuilt by a recipe from pinned
USGS files and kept within its stats' tolerance, the rasters it cannot
rebuild named with the worlds that need them and how to get them, without
failing the run, and its exit status in --help."""
import contextlib
import dataclasses
import hashlib
import http.client
import io
import json
import shutil
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from worldfiles import SIM_DIR

sys.path.insert(0, str(SIM_DIR / "tools"))
import fetch_data  # noqa: E402
import usgs_lidar  # noqa: E402
from urc import dem, farfield  # noqa: E402
from urc.missions import COURSES, MISSIONS, autonomy  # noqa: E402

MANIFEST = json.loads((fetch_data.DATA_DIR / fetch_data.MANIFEST).read_text())
STEP = 0.001  # [deg] the made-up raster's pixel
TRANSFORM = (-110.79, STEP, 0.0, 38.43, 0.0, -STEP)  # its corner and pixel (GDAL geotransform)
LINK = "link it from a clone that has it: rover_sim/tools/link_data.sh <that clone> (README, Simulation)"


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


def marked_alpha(body):
    """A 4-band GeoTIFF body as write_geotiff writes it, its fourth band marked alpha (ExtraSamples 2), as USDA's
    conus_naip ImageServer marks NAIP 2024's near infrared."""
    entry = dem.EXTRA_SAMPLES.to_bytes(2, "little") + b"\x03\x00\x01\x00\x00\x00"
    assert body.count(entry + b"\x00\x00") == 1
    return body.replace(entry + b"\x00\x00", entry + b"\x02\x00")


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
    tiles, not exportImage requests), read by `worlds`. Returns its bytes, as another clone would have it."""
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


def sha256(body):
    return hashlib.sha256(body).hexdigest()


LIDAR_URL = "https://example.test/lpc/lidar.laz"


def add_lidar(data, offset):
    """Adds to the manifest in data two rasters that a made-up recipe of tools/usgs_lidar.py, "made_up", makes from
    one USGS file: dem/lidar.tif (required) and dem/lidar_map.tif (not required), with their stats. Returns ({url:
    body} of the file, the recipe, the list of its runs); the DEM it makes is the file's plus offset["m"]."""
    body = raster_bytes(dem_band(32, 32) + 7.0, TRANSFORM)
    runs = []

    def make(files, data_dir, work):
        runs.append(files)
        bands, transform = dem.read_raster(files["lidar_tiles"][0])
        made = {"dem/lidar.tif": work / "lidar.tif", "dem/lidar_map.tif": work / "lidar_map.tif"}
        fetch_data.write_geotiff(made["dem/lidar.tif"], bands + offset["m"], fetch_data.geotags(transform))
        fetch_data.write_geotiff(made["dem/lidar_map.tif"], np.full_like(bands, 2.0), fetch_data.geotags(transform))
        return made

    (data / "dem" / "lidar.json").write_text(json.dumps({
        "recipe": "made_up", "lidar_tiles": [{"url": LIDAR_URL, "size_bytes": len(body), "sha256": sha256(body)}]}))
    manifest = json.loads((data / fetch_data.MANIFEST).read_text())
    for name, bands, required in (("lidar", dem_band(32, 32) + 7.0, True),
                                  ("lidar_map", np.full((1, 32, 32), 2.0), False)):
        whole = raster_bytes(bands, TRANSFORM)
        (data / "whole.tif").write_bytes(whole)
        manifest["rasters"].append({
            "path": f"dem/{name}.tif", "provenance": "dem/lidar.json", "required": required, "official": True,
            "bytes": len(whole), "sha256": sha256(whole),
            "content_sha256": fetch_data.content_sha256(data / "whole.tif"),
            "shape": [1, 32, 32], "dtype": "float32", "transform": list(TRANSFORM),
            "stats": fetch_data.stats(dem.read_raster(data / "whole.tif")[0])})
        (data / "whole.tif").unlink()
    (data / fetch_data.MANIFEST).write_text(json.dumps(manifest))
    return {LIDAR_URL: body}, usgs_lidar.Recipe(("lidar_tiles",), ("dem/lidar.tif", "dem/lidar_map.tif"), make), runs


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

    def test_each_raster_has_its_provenance_and_an_official_one_its_source(self):
        """An official raster's .json lists its exportImage requests, or names the recipe of tools/usgs_lidar.py that
        makes it and lists every file that recipe reads with its size and SHA-256; a lidar rebuild has its stats."""
        for e in MANIFEST["rasters"]:
            with self.subTest(e["path"]):
                provenance = json.loads((fetch_data.DATA_DIR / e["provenance"]).read_text())
                requests = fetch_data.requests_of(provenance)
                recipe = fetch_data.recipe_of(provenance)
                exported = bool(requests) and all("exportImage" in url for url in requests)
                self.assertEqual(exported or recipe is not None, e["official"])
                self.assertEqual(recipe is not None, "stats" in e)
                if recipe is not None:
                    self.assertIn(e["path"], recipe.outputs)
                    for tile in fetch_data.inputs_of(provenance):
                        self.assertRegex(tile["sha256"], "^[0-9a-f]{64}$")
                        self.assertGreater(tile["size_bytes"], 0)

    def test_the_lidar_comes_from_usgs(self):
        """The recipes read USGS's own files only: the OPR DEM tiles on The National Map's S3 bucket and the point
        clouds on rockyweb (the user, 2026-10-09: no data release; the lidar DEMs rebuilt from USGS's public lidar)."""
        usgs = ("https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/OPR/Projects/UT_StateWide_2018_A18/",
                "https://rockyweb.usgs.gov/vdelivery/Datasets/Staged/Elevation/LPC/Projects/"
                "UT_StatewideSouth_2020_A20/")
        counts = {}
        for e in MANIFEST["rasters"]:
            provenance = json.loads((fetch_data.DATA_DIR / e["provenance"]).read_text())
            if fetch_data.recipe_of(provenance) is not None:
                tiles = fetch_data.inputs_of(provenance)
                counts[e["path"]] = len(tiles)
                for tile in tiles:
                    self.assertTrue(tile["url"].startswith(usgs), tile["url"])
        self.assertEqual(counts, {"dem/mdrs_area_lidar2018_0p5m.tif": 11, "dem/route_area_lidar_0p5m.tif": 11,
                                  "dem/route_area_lidar_0p5m_source.tif": 11})

    def test_the_rasters_it_cannot_rebuild_name_their_worlds_and_what_is_planned(self):
        """A raster that is not official has no copy to fetch, only another
        clone: the manifest says which worlds need it (the required ones,
        read by the simulation) and what is planned. None is listed today."""
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
        self.assertEqual([e["path"] for e in MANIFEST["rasters"] if not e["official"]], [])  # none, since the lidar
        lidar = fetch_data.entry_of(autonomy.DEM_PATH)
        self.assertEqual((lidar["official"], lidar["required"]), (True, True))

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
            self.assertIn("dem/route_area_lidar_0p5m.tif: missing, would rebuild it from the USGS lidar, 11 file(s), "
                          "371 MB", lines)


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

    def test_naip_tiles_marked_alpha_make_the_raster(self):
        """USDA's NAIP 2024 tiles mark their near infrared alpha, which dem.read_raster refuses; fetch-data clears
        the mark and the raster's content is verified."""
        data, bodies = case(self.tmp.name, naip_bands())
        bodies = {url: marked_alpha(body) for url, body in bodies.items()}
        tile = data / "tile.tif"
        tile.write_bytes(next(iter(bodies.values())))
        with self.assertRaisesRegex(ValueError, "alpha"):
            dem.read_raster(tile)
        tile.unlink()
        self.assertEqual(run(data, Server(bodies)),
                         (0, ["dem/made_up.tif: fetched from its official source, content verified"]))
        np.testing.assert_array_equal(dem.read_raster(data / "dem" / "made_up.tif")[0], naip_bands())

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

    def test_from_another_clone_it_is_present_and_verified(self):
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

    def test_world_missing_says_why_a_world_is_not_here(self):
        """What sim, drive and the tests' skips say about a world sim-worlds skipped."""
        lidar = self.data.absolute() / "dem" / "lidar.tif"  # outside the repository: shown as it is
        self.assertEqual(fetch_data.world_missing("urc_made_up", self.data),
                         f"urc_made_up needs {lidar}, which is not here: fetch-data cannot rebuild it from official "
                         f"sources yet (a made-up script is planned); {LINK}")
        self.assertIsNone(fetch_data.world_missing("urc_other", self.data))  # no raster it reads is missing
        (self.data / "dem" / "lidar.tif").write_bytes(self.lidar)
        self.assertIsNone(fetch_data.world_missing("urc_made_up", self.data))

    def test_the_autonomy_lidar_is_shown_from_the_repository(self):
        self.assertEqual(fetch_data.shown(autonomy.DEM_PATH), "rover_sim/data/dem/route_area_lidar_0p5m.tif")


class Stats(unittest.TestCase):
    """What a lidar rebuild that is not bit-identical must match: min, max and 16 x 16 block means within 1e-3."""

    def test_the_stats_are_min_max_and_block_means(self):
        s = fetch_data.stats(np.arange(32 * 48, dtype=np.float32).reshape(1, 32, 48))
        self.assertEqual((s["min"], s["max"]), (0.0, 1535.0))
        self.assertEqual(np.shape(s["block_means"]), (1, 16, 16))
        self.assertEqual(s["block_means"][0][0][0], 25.0)  # rows 0-1, columns 0-2
        self.assertEqual(s["block_means"][0][15][15], 1510.0)

    def test_within_allows_the_tolerance_and_no_more(self):
        s = fetch_data.stats(dem_band(32, 32))
        near, far = json.loads(json.dumps(s)), json.loads(json.dumps(s))
        near["block_means"][0][3][4] += 0.0009
        far["block_means"][0][3][4] += 0.0011
        self.assertTrue(fetch_data.within(near, s))
        self.assertFalse(fetch_data.within(far, s))
        self.assertFalse(fetch_data.within(fetch_data.stats(np.concatenate([dem_band(32, 32)] * 2)), s))  # 2 bands

    def test_a_raster_within_the_stats_on_another_grid_does_not_match(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "r.tif"
            path.write_bytes(raster_bytes(dem_band(32, 32), TRANSFORM))
            entry = {"content_sha256": "", "shape": [1, 32, 32], "transform": list(TRANSFORM),
                     "stats": fetch_data.stats(dem_band(32, 32))}
            self.assertEqual(fetch_data.verdict(entry, path),
                             "content within 0.001 of rasters.json's stats, not bit-identical")
            path.write_bytes(raster_bytes(dem_band(32, 32), (TRANSFORM[0] + STEP, *TRANSFORM[1:])))
            self.assertIsNone(fetch_data.verdict(entry, path))


class Sources(unittest.TestCase):
    """The USGS files a recipe reads: downloaded once, several at once, checked against the size and SHA-256 its
    provenance lists; a download cut short is tried again."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.tiles = [{"url": f"https://example.test/{name}", "size_bytes": len(body), "sha256": sha256(body)}
                      for name, body in (("a.laz", b"first"), ("b.tif", b"second"))]
        self.server = Server({"https://example.test/a.laz": b"first", "https://example.test/b.tif": b"second"})

    def test_they_are_downloaded_checked_and_used_again(self):
        said = []
        paths = fetch_data.source_files(self.tiles, self.dir, self.server, said.append)
        self.assertEqual(paths, [self.dir / "a.laz", self.dir / "b.tif"])
        self.assertEqual([p.read_bytes() for p in paths], [b"first", b"second"])
        self.assertEqual(said, ["  a.laz: downloaded, 0.0 MB", "  b.tif: downloaded, 0.0 MB"])
        (self.dir / "b.tif").write_bytes(b"spoiled")
        self.assertEqual(fetch_data.source_files(self.tiles, self.dir, self.server, said.append), paths)
        self.assertEqual(len(self.server.asked), 3)  # a.laz once; b.tif again, as the copy here was not it
        self.assertEqual((self.dir / "b.tif").read_bytes(), b"second")

    def test_a_file_that_is_not_the_listed_one_raises_and_is_not_kept(self):
        self.server.bodies["https://example.test/b.tif"] = b"second, changed"
        with self.assertRaisesRegex(ValueError, "https://example.test/b.tif is not the file the provenance lists "
                                                "[(]15 bytes, SHA-256 [0-9a-f]{12}[.][.][.][)]"):
            fetch_data.source_files(self.tiles, self.dir, self.server, lambda line: None)
        self.assertEqual(sorted(p.name for p in self.dir.iterdir()), ["a.laz"])

    def test_they_are_downloaded_at_the_same_time(self):
        both = threading.Barrier(2, timeout=10)

        def get(url):  # answers once both requests are in flight; one at a time breaks the barrier after 10 s
            both.wait()
            return self.server(url)
        self.assertEqual(fetch_data.source_files(self.tiles, self.dir, get, lambda line: None),
                         [self.dir / "a.laz", self.dir / "b.tif"])

    def test_a_download_cut_short_is_tried_again(self):
        class Cut(io.BytesIO):  # a response whose body breaks off, as http.client reports it
            def read(self, *args):
                raise http.client.IncompleteRead(b"fir", 2)
        responses = [Cut(), io.BytesIO(b"first")]
        with mock.patch.object(fetch_data.urllib.request, "urlopen", lambda request, timeout: responses.pop(0)), \
                mock.patch.object(fetch_data.time, "sleep"):
            self.assertEqual(fetch_data.download("https://example.test/a.laz"), b"first")
        self.assertEqual(responses, [])


class Lidar(unittest.TestCase):
    """A raster whose provenance names a recipe of tools/usgs_lidar.py: rebuilt from the USGS files listed there and
    kept when its content matches or, not bit-identical, its stats are within the tolerance."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        (Path(self.tmp.name) / "data").mkdir()
        self.data, bodies = case(Path(self.tmp.name) / "data", dem_band())
        self.offset = {"m": 0.0}
        lidar_bodies, recipe, self.runs = add_lidar(self.data, self.offset)
        self.bodies = {**bodies, **lidar_bodies}
        self.temp = Path(self.tmp.name) / "temp"  # where the run's temporary directories go
        self.temp.mkdir()
        for patcher in (mock.patch.dict(usgs_lidar.RECIPES, {"made_up": recipe}),
                        mock.patch.object(tempfile, "tempdir", str(self.temp))):
            patcher.start()
            self.addCleanup(patcher.stop)

    def files(self):
        return sorted(p.name for p in (self.data / "dem").iterdir())

    def test_a_recipe_makes_its_rasters_in_one_run_and_they_are_verified(self):
        code, lines = run(self.data, Server(self.bodies), "--all")
        self.assertEqual((code, lines), (0, ["dem/made_up.tif: fetched from its official source, content verified",
                                             "  lidar.laz: downloaded, 0.0 MB",
                                             "dem/lidar.tif: rebuilt from the USGS lidar, content verified",
                                             "dem/lidar_map.tif: rebuilt from the USGS lidar, content verified"]))
        self.assertEqual(len(self.runs), 1)
        self.assertEqual(self.files(), ["lidar.json", "lidar.tif", "lidar_map.tif", "made_up.json", "made_up.tif"])
        self.assertEqual(list(self.temp.iterdir()), [])  # the downloads and the work directory are gone
        self.assertEqual(run(self.data, Server({}), "--all", "--verify"),
                         (0, ["dem/made_up.tif: verified", "dem/lidar.tif: verified", "dem/lidar_map.tif: verified"]))

    def test_the_default_run_makes_only_the_required_raster(self):
        code, lines = run(self.data, Server(self.bodies))
        self.assertEqual((code, lines[-1]), (0, "dem/lidar.tif: rebuilt from the USGS lidar, content verified"))
        self.assertNotIn("lidar_map.tif", self.files())

    def test_the_dry_run_says_what_it_would_download(self):
        server = Server(self.bodies)
        self.assertEqual(run(self.data, server, "--dry-run", "--all"),
                         (0, ["dem/made_up.tif: missing, would fetch from the official source, 4 request(s)",
                              "dem/lidar.tif: missing, would rebuild it from the USGS lidar, 1 file(s), 0 MB",
                              "dem/lidar_map.tif: missing, would rebuild it from the USGS lidar, 1 file(s), 0 MB"]))
        self.assertEqual((server.asked, self.runs), ([], []))

    def test_a_file_that_is_not_the_listed_one_fails_and_installs_nothing(self):
        bad = b"X" + self.bodies[LIDAR_URL][1:]
        self.bodies[LIDAR_URL] = bad
        err = io.StringIO()
        code, lines = run(self.data, Server(self.bodies), err=err)
        self.assertEqual((code, lines[1:]), (1, [
            f"dem/lidar.tif: FAILED, the rebuild from the USGS lidar failed ({LIDAR_URL} is not the file the "
            f"provenance lists ({len(bad)} bytes, SHA-256 {sha256(bad)[:12]}...))"]))
        self.assertEqual((self.runs, self.files()), ([], ["lidar.json", "made_up.json", "made_up.tif"]))
        self.assertIn("1 raster(s) not fetched (FAILED above)", err.getvalue())

    def test_a_rebuild_within_the_tolerance_is_kept_and_says_so(self):
        self.offset["m"] = 0.0002  # what another platform's rounding might give
        code, lines = run(self.data, Server(self.bodies))
        how = "content within 0.001 of rasters.json's stats, not bit-identical"
        self.assertEqual((code, lines[-1]), (0, f"dem/lidar.tif: rebuilt from the USGS lidar, {how}"))
        self.assertEqual(run(self.data, Server({}), "--verify"),
                         (0, ["dem/made_up.tif: verified", f"dem/lidar.tif: verified, {how}"]))

    def test_a_rebuild_beyond_the_tolerance_fails(self):
        self.offset["m"] = 0.01  # a wrong tile or step: centimetres
        code, lines = run(self.data, Server(self.bodies))
        self.assertEqual((code, lines[-1]), (1, "dem/lidar.tif: FAILED, the rebuild from the USGS lidar gives other "
                                                "pixels than rasters.json lists"))
        self.assertNotIn("lidar.tif", self.files())

    def test_sources_keeps_the_downloads_for_the_next_run(self):
        sources = Path(self.tmp.name) / "sources"
        self.assertEqual(run(self.data, Server(self.bodies), "--sources", str(sources))[0], 0)
        self.assertEqual(sorted(p.name for p in sources.iterdir()), ["lidar.laz"])
        (self.data / "dem" / "lidar.tif").unlink()
        server = Server({})
        self.assertEqual(run(self.data, server, "--sources", str(sources)),
                         (0, ["dem/made_up.tif: present",
                              "dem/lidar.tif: rebuilt from the USGS lidar, content verified"]))
        self.assertEqual(server.asked, [])

    def test_a_raster_the_recipe_reads_that_is_not_here_fails_before_any_download(self):
        usgs_lidar.RECIPES["made_up"] = dataclasses.replace(usgs_lidar.RECIPES["made_up"], needs=("dem/fill.tif",))
        server = Server(self.bodies)
        code, lines = run(self.data, server)
        self.assertEqual((code, lines[-1]), (1, "dem/lidar.tif: FAILED, the rebuild from the USGS lidar failed "
                                                "(dem/fill.tif is not here (fetch-data fetches it first))"))
        self.assertNotIn(LIDAR_URL, server.asked)


class Help(unittest.TestCase):
    def test_help_gives_the_exit_status(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out), self.assertRaises(SystemExit) as caught:
            fetch_data.main(["--help"])
        self.assertEqual(caught.exception.code, 0)
        self.assertIn("Exit status: 0 when every listed raster fetch-data can fetch is here", out.getvalue())
        self.assertIn("2 for a usage\nerror.", out.getvalue())


if __name__ == "__main__":
    unittest.main()
