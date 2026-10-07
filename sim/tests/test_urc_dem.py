#!/usr/bin/env python3
"""The real-terrain tools: the USGS DEM, its resampling into a mission's
layout frame, and route analysis (no physics; pixi run sim-test)."""
import datetime
import hashlib
import json
import math
import shutil
import subprocess
import sys
import tempfile
import unittest
import urllib.parse
from pathlib import Path
from unittest import mock

import numpy as np
from PIL import Image

from worldfiles import SIM_DIR

sys.path.insert(0, str(SIM_DIR / "tools"))
import fetch_dem  # noqa: E402
from urc import dem, geo, routes, rules, terrain  # noqa: E402

DEM_PATH = fetch_dem.ROUTE_AREA_DEM
LIDAR_PATH = DEM_PATH.parent / "route_area_lidar_0p5m.tif"  # the same area at 0.5 m, georeferenced by GDAL
# USGS Elevation Point Query Service (3DEP), queried 2026-10-06: (lat, lon, metres NAVD88).
EPQS = ((38.4180, -110.7770, 1394.07), (38.411, -110.786, 1379.53), (38.425, -110.768, 1423.24))
DEFLATE = (8, 32946)  # TIFF compression codes (Adobe and old-style deflate)


class GeoTiff(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dem = dem.read_geotiff(DEM_PATH)

    def test_georeferencing(self):
        d = self.dem
        self.assertEqual(d.z.shape, (2442, 2268))
        self.assertEqual((d.lon0, d.lat0), (-110.79, 38.43199735449736))
        self.assertAlmostEqual(d.dlon, 0.026 / 2268, places=15)  # square pixels in degrees
        self.assertAlmostEqual(d.dlat, d.dlon, places=15)
        (south, west), (north, east) = d.bounds
        (s, w), (n, e) = rules.ROUTE_AREA
        self.assertTrue(south < s and west < w and north > n and east > e)
        self.assertTrue(1340 < d.z.min() < d.z.max() < 1432)

    def test_file_is_compressed_and_keeps_its_geotiff_tags(self):
        with Image.open(DEM_PATH) as img:
            self.assertEqual(img.mode, "F")
            self.assertIn(img.tag_v2[259], DEFLATE)
            for tag in fetch_dem.GEOTIFF_TAGS:
                self.assertIn(tag, img.tag_v2, tag)
            self.assertEqual(img.tag_v2[dem.GEO_DOUBLE_PARAMS][1:3], (6378137.0, 298.257223563))  # WGS 84 ellipsoid
            self.assertIn("WGS 84", img.tag_v2[dem.GEO_ASCII_PARAMS])

    def test_heights_match_usgs_point_queries(self):
        for lat, lon, alt in EPQS:  # measured: within 0.05 m
            self.assertAlmostEqual(self.dem.height(lat, lon), alt, delta=0.1, msg=(lat, lon))

    def test_pixel_centres_match_the_lidar(self):
        """DEM.pixel's pixel-centre convention: the EPQS points lie on flat
        ground, where half a pixel moves the height by centimetres, so on
        steep ground (> 25 deg) DEM.height is compared with the 0.5 m lidar
        DEM read at its own pixel centres (PixelIsArea: the tiepoint is the
        corner of pixel (0, 0)). The best-fit horizontal offset between the
        two is under 0.2 m (measured 0.07 m); half a 3DEP pixel off (0.5 m
        east, 0.64 m north) gives 0.3-0.5 m."""
        lidar = dem.read_geotiff(LIDAR_PATH)
        m_north = 111_000.0 * lidar.dlat  # [m] per lidar pixel, close enough for gradients
        m_east = 111_320.0 * math.cos(math.radians(lidar.lat0)) * lidar.dlon
        rng = np.random.default_rng(1)
        rows, cols = (rng.integers(1, n - 1, 400_000) for n in lidar.z.shape)
        z = lidar.z.astype(float)
        east = (z[rows, cols + 1] - z[rows, cols - 1]) / (2 * m_east)
        north = (z[rows - 1, cols] - z[rows + 1, cols]) / (2 * m_north)
        steep = np.flatnonzero(np.hypot(east, north) > math.tan(math.radians(25)))[:5000]
        self.assertEqual(len(steep), 5000)
        rows, cols, east, north = rows[steep], cols[steep], east[steep], north[steep]
        lat, lon = lidar.lat0 - (rows + 0.5) * lidar.dlat, lidar.lon0 + (cols + 0.5) * lidar.dlon
        diff = self.dem.height(lat, lon) - z[rows, cols]
        (bias, dx, dy), *_ = np.linalg.lstsq(np.stack([np.ones_like(diff), east, north], axis=1), diff, rcond=None)
        self.assertLess(math.hypot(dx, dy), 0.2, (dx, dy))
        self.assertLess(abs(bias), 0.05)

    def test_height_outside_the_dem_raises(self):
        with self.assertRaises(ValueError):
            self.dem.height(38.45, -110.777)

    def test_provenance_reproduces_the_request(self):
        info = json.loads(DEM_PATH.with_suffix(".json").read_text())
        self.assertEqual(info["license"], "public domain (USGS)")
        self.assertLessEqual(datetime.date.fromisoformat(info["fetched"]), datetime.date.today())  # a re-fetch dates it
        url = fetch_dem.export_url(fetch_dem.ROUTE_AREA_BBOX, fetch_dem.request_size(fetch_dem.ROUTE_AREA_BBOX, 1.0))
        ours, recorded = (urllib.parse.parse_qs(urllib.parse.urlsplit(u).query) for u in (url, info["url"]))
        bbox = lambda q: [float(v) for v in q.pop("bbox")[0].split(",")]  # noqa: E731
        self.assertEqual(bbox(ours), bbox(recorded))
        self.assertEqual(ours, recorded)

    def test_fetch_refuses_to_overwrite(self):
        result = subprocess.run([sys.executable, str(SIM_DIR / "tools" / "fetch_dem.py"), "--out", str(DEM_PATH)],
                                capture_output=True, text=True, timeout=60)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("--force", result.stderr)

    def test_request_keeps_the_bbox(self):
        bbox = (-110.7855, 38.40425, -110.76275, 38.43175)  # 7 significant digits
        query = urllib.parse.parse_qs(urllib.parse.urlsplit(fetch_dem.export_url(bbox, (10, 10))).query)
        self.assertEqual(tuple(float(v) for v in query["bbox"][0].split(",")), bbox)

    def test_bad_download_keeps_the_old_dem(self):
        """--force with an answer read_geotiff rejects (a no-data cell): the
        DEM and its .json stay as they were, and no temporary file is left."""
        hf = terrain.Heightfield(8, 9)
        hf.z[4, 4] = -3.4e38  # what the service sends for a coverage gap
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "area.tif"
            shutil.copy(DEM_PATH, out)
            shutil.copy(DEM_PATH.with_suffix(".json"), out.with_suffix(".json"))
            before = {p.name: hashlib.sha1(p.read_bytes()).hexdigest() for p in Path(tmp).iterdir()}
            dem.write_geotiff(hf, Path(tmp) / "bad.tif", geo.Origin(38.418, -110.777, 0.0))
            response = mock.MagicMock()
            response.__enter__.return_value.read.return_value = (Path(tmp) / "bad.tif").read_bytes()
            response.__enter__.return_value.headers = {"Content-Type": "image/tiff"}
            (Path(tmp) / "bad.tif").unlink()
            argv = ["fetch_dem.py", "--out", str(out), "--force"]
            with mock.patch.object(sys, "argv", argv), mock.patch.object(fetch_dem.urllib.request, "urlopen",
                                                                         return_value=response):
                with self.assertRaisesRegex(ValueError, "no-data"):
                    fetch_dem.main()
            after = {p.name: hashlib.sha1(p.read_bytes()).hexdigest() for p in Path(tmp).iterdir()}
        self.assertEqual(after, before)

    def test_reads_what_it_writes(self):
        """A mission's exported dem.tif reads back at the same heights."""
        o = geo.Origin(38.418, -110.777, 1400.0)
        hf = terrain.Heightfield(64, 65, center=(10.0, -20.0)).noise(3.0, 20.0, seed=1)
        with tempfile.TemporaryDirectory() as tmp:
            dem.write_geotiff(hf, Path(tmp) / "dem.tif", o)
            d = dem.read_geotiff(Path(tmp) / "dem.tif")
        for x, y in ((10.0, -20.0), (-15.0, 5.0), (37.0, -45.5)):
            lat, lon, _ = geo.enu_to_wgs84(o, x, y)
            self.assertAlmostEqual(d.height(lat, lon), o.alt + hf.height(x, y), delta=0.05)


class LayoutHeightfield(unittest.TestCase):
    """to_heightfield: the DEM in a layout frame, x east, y north, z above the origin."""

    @classmethod
    def setUpClass(cls):
        cls.dem = dem.read_geotiff(DEM_PATH)
        lat, lon = 38.4190, -110.7752
        cls.origin = geo.Origin(lat, lon, cls.dem.height(lat, lon))
        cls.hf = dem.to_heightfield(cls.dem, cls.origin, 600.0, 301, center=(-100.0, 150.0))

    def dem_z(self, x, y):
        lat, lon, _ = geo.enu_to_wgs84(self.origin, x, y)
        return self.dem.height(lat, lon) - self.origin.alt

    def test_origin_at_zero(self):
        self.assertAlmostEqual(self.hf.height(0.0, 0.0), 0.0, delta=0.05)

    def test_north_up_east_right(self):
        # Row 0 is the north edge, column 0 the west edge (terrain.Heightfield, Gazebo heightmaps).
        corners = {(0, 0): (-400.0, 450.0), (0, -1): (200.0, 450.0), (-1, 0): (-400.0, -150.0),
                   (-1, -1): (200.0, -150.0)}
        for (row, col), (x, y) in corners.items():
            self.assertAlmostEqual(self.hf.z[row, col], self.dem_z(x, y), delta=0.05, msg=(row, col))
        rng = np.random.default_rng(3)
        for x, y in rng.uniform((-400, -150), (200, 450), (50, 2)):
            self.assertAlmostEqual(self.hf.height(x, y), self.dem_z(x, y), delta=0.3, msg=(x, y))

    def test_terrain_outside_the_dem_raises(self):
        with self.assertRaises(ValueError):
            dem.to_heightfield(self.dem, self.origin, 4096.0, 33, center=(0.0, 0.0))


class Routes(unittest.TestCase):
    """A 10 m mesa with near-vertical sides and one 14 deg ramp from the east."""

    @classmethod
    def setUpClass(cls):
        hf = terrain.Heightfield(200, 201).mesa(0, 0, 20, 10.0, 4.0, seed=1, irregularity=0.0)
        cls.hf = hf.ramp([(64.0, 0.0), (24.0, 0.0)], 4.0, 8.0, z_start=0.0, z_end=10.0)

    def test_goes_round_to_the_ramp(self):
        path, grade = routes.easy_route(self.hf, (-60.0, 0.0), (0.0, 0.0), 16.0, margin=80.0)
        self.assertLessEqual(grade, 16.0)
        self.assertGreater(grade, 12.0)  # it does climb the ramp
        self.assertEqual((path[0], path[-1]), ((-60.0, 0.0), (0.0, 0.0)))
        self.assertGreater(max(x for x, _ in path), 40.0)  # via the east side

    def test_refuses_a_steeper_climb(self):
        self.assertIsNone(routes.easy_route(self.hf, (-60.0, 0.0), (0.0, 0.0), 10.0, margin=80.0))

    def test_approach_grades(self):
        east, north, west = routes.approach_grades(self.hf, (0.0, 0.0), (90, 0, 270), 6.0, 60.0)
        self.assertLess(east, 16.0)
        self.assertGreater(north, 45.0)
        self.assertGreater(west, 45.0)

    def test_rim(self):
        """Round the mesa's flat top (up to 10 deg: not the 14 deg ramp), and
        widened round a route from where it reaches the top."""
        rim, entry = routes.rim(self.hf, (0.0, 0.0), 10.0)
        self.assertEqual(rim[0], rim[-1])
        r = np.hypot(*np.array(rim).T)  # the top's edge (r = 20 m) blurred over GRADE_BASE, and the ramp's head
        self.assertTrue(np.all((r > 15.0) & (r < 26.0)), (r.min(), r.max()))
        self.assertIsNone(entry)
        route = [(64.0, 0.0), (0.0, 0.0), (0.0, 18.0)]  # up the ramp, across the top to its north edge
        rim, entry = routes.rim(self.hf, (0.0, 0.0), 10.0, route, 6.0)
        self.assertTrue(15.0 < entry[0] < 26.0 and entry[1] == 0.0, entry)
        on_top = terrain.path_distance([entry, (0.0, 0.0), (0.0, 18.0)], *np.array(rim).T)[0]
        self.assertGreater(on_top.min(), 5.0)  # within a cell of 6 m


if __name__ == "__main__":
    unittest.main()
