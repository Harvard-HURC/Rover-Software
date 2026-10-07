#!/usr/bin/env python3
"""Unit tests for the URC scenario generator, sim/urc (no physics; pixi run sim-test)."""
import math
import sys
import tempfile
import threading
import unittest
import zlib
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

import worldfiles  # noqa: F401  (puts sim/ on the path)

import gen_worlds  # noqa: E402
from urc import dem, farfield, geo, meshes, rules, terrain, terrains, textures  # noqa: E402,F401
from urc import media as media_module  # noqa: E402
from urc.media import Media  # noqa: E402

MDRS = geo.Origin(38.4064, -110.7919, 1350.0)


class Geo(unittest.TestCase):
    def test_origin_maps_to_itself(self):
        lat, lon, alt = geo.enu_to_wgs84(MDRS, 0, 0, 0)
        self.assertAlmostEqual(lat, MDRS.lat, places=10)
        self.assertAlmostEqual(lon, MDRS.lon, places=10)
        self.assertAlmostEqual(alt, MDRS.alt, places=6)

    def test_axes_are_east_north_up(self):
        lat, lon, _ = geo.enu_to_wgs84(MDRS, 0, 1000, 0)
        self.assertAlmostEqual(lat - MDRS.lat, 1000 / 111_000, delta=2e-5)  # ~111 km per degree
        self.assertAlmostEqual(lon, MDRS.lon, places=7)
        lat, lon, _ = geo.enu_to_wgs84(MDRS, 1000, 0, 0)
        self.assertGreater(lon, MDRS.lon)
        self.assertAlmostEqual(lon - MDRS.lon, 1000 / (111_320 * math.cos(math.radians(MDRS.lat))), delta=5e-5)

    def test_round_trip(self):
        for x, y, z in ((0, 0, 0), (812.5, -431.2, 12.3), (-1000, 1000, -50)):
            back = geo.wgs84_to_enu(MDRS, *geo.enu_to_wgs84(MDRS, x, y, z))
            for got, want in zip(back, (x, y, z)):
                self.assertAlmostEqual(got, want, delta=1e-3)


class LonLatFit(unittest.TestCase):
    def test_fit_against_the_exact_transform(self):
        """geo.lonlat_fit, the one lat/lon fit (colour maps, DEM resampling,
        the far field): a quadratic over 2 km and a cubic over 80 km agree
        with enu_to_wgs84 to 1e-9 and 1e-6 deg at random points."""
        rng = np.random.default_rng(1)
        for size, degree, tolerance in ((2048.0, 2, 1e-9), (80_000.0, 3, 1e-6)):
            fit = geo.lonlat_fit(MDRS, size, (100.0, -50.0), degree, tolerance)
            for x, y in rng.uniform(-size / 2, size / 2, (20, 2)) + (100.0, -50.0):
                np.testing.assert_allclose(fit(x, y), geo.enu_to_wgs84(MDRS, x, y)[:2], atol=tolerance)
        with self.assertRaises(ValueError):
            geo.lonlat_fit(MDRS, 80_000.0)  # a quadratic is not enough over the far field


class Rules(unittest.TestCase):
    def test_ar_tag_cells(self):
        # 4x4 data + black border + one white cell each side = 8 cells on a 20 cm face (1.e.xii).
        self.assertAlmostEqual(rules.AR_CELL, 0.025)
        self.assertAlmostEqual(8 * rules.AR_CELL, rules.AR_FACE)

    def test_route_area_is_the_square_mile(self):
        (lat0, lon0), (lat1, lon1) = rules.ROUTE_AREA
        self.assertLess(lat0, lat1)
        self.assertLess(lon0, lon1)


class Terrain(unittest.TestCase):
    def plane(self, gx=0.1, gy=-0.05, size=64, n=129):
        hf = terrain.Heightfield(size, n)
        X, Y = hf.grid()
        hf.z = 1.0 + gx * X + gy * Y
        return hf

    def test_grid_layout_matches_gazebo_images(self):
        X, Y = terrain.Heightfield(10, 11).grid()
        self.assertEqual((X[0, 0], Y[0, 0]), (-5, 5))  # row 0 north, column 0 west
        self.assertEqual((X[-1, -1], Y[-1, -1]), (5, -5))

    def test_offset_grid_uses_world_coordinates(self):
        hf = terrain.Heightfield(10, 11, center=(100, -50))
        X, Y = hf.grid()
        self.assertEqual((X[0, 0], Y[0, 0]), (95, -45))
        hf.z = X - 100
        self.assertAlmostEqual(hf.height(102.5, -48), 2.5)

    def test_bilinear_height_on_a_plane(self):
        hf = self.plane()
        for x, y in ((0, 0), (3.3, -7.1), (-20.2, 15.9)):
            self.assertAlmostEqual(hf.height(x, y), 1.0 + 0.1 * x - 0.05 * y, places=9)

    def test_slope(self):
        self.assertAlmostEqual(self.plane().slope_deg(1.0, 2.0), math.degrees(math.atan(math.hypot(0.1, 0.05))),
                               places=6)

    def test_line_of_sight(self):
        hf = terrain.Heightfield(64, 129)
        X, _ = hf.grid()
        hf.z[np.abs(X) < 1] = 5.0  # a 5 m wall along x = 0
        self.assertFalse(hf.line_of_sight((-10, 0, 3), (10, 0, 1)))
        self.assertTrue(hf.line_of_sight((-10, 5, 8), (10, 5, 6)))
        self.assertTrue(hf.line_of_sight((-10, 0, 3), (-2, 0, 1)))

    def test_ramp_has_its_grade(self):
        hf = terrain.Heightfield(128, 257).mesa(0, 0, 20, 10.0, 4.0, seed=1, irregularity=0)
        path = [(-70, 0), (-20, 0)]
        hf.ramp(path, half_width=3, falloff=2, z_end=10.0)
        grade = hf.max_grade_along(path)
        self.assertAlmostEqual(grade, math.degrees(math.atan(10 / 50)), delta=1.0)
        # Off the ramp the hill side stays a cliff.
        self.assertGreater(hf.max_grade_along([(0, -40), (0, -15)]), 45)

    def test_strip_follows_its_profile(self):
        hf = terrain.Heightfield(64, 513)
        segments = ((5.0, 0.0), (10.0, 10.0), (5.0, -10.0))
        hf.strip((-20.0, 3.0), 0.0, 4.1, segments, falloff=2.0)
        t = math.tan(math.radians(10))
        for u, z in ((2.5, 0.0), (10.0, 5 * t), (15.0, 10 * t), (17.5, 7.5 * t), (21.0, 5 * t)):
            self.assertAlmostEqual(hf.height(-20.0 + u, 3.0), z, delta=0.002 if u < 20 else 5 * t, msg=u)
        # On its profile out to its very edge, here between two rows of samples.
        self.assertAlmostEqual(hf.height(-12.43, 3.0 + 2.04), 2.57 * t, delta=1e-9)
        cell = hf.res * math.sqrt(2)
        self.assertEqual(hf.height(-10.0, 3.0 + 2.05 + cell + 2.0 + hf.res), 0.0)  # beyond the falloff: untouched
        hf.strip((-20.0, -20.0), math.pi / 2, 6.0, ((20.0, 0.0),), falloff=2.0, z0=1.0, cross=20.0)
        # Heading north, the left side is west.
        self.assertAlmostEqual(hf.height(-22.0, -10.0), 1.0 + 2 * math.tan(math.radians(20)), delta=0.002)
        self.assertAlmostEqual(hf.height(-20.0, -10.0), 1.0, delta=0.002)

    def test_bump_and_washboard(self):
        hf = terrain.Heightfield(32, 257).bump(1.0, 2.0, 0.3, 1.6, 0.7, yaw=math.pi / 2)
        self.assertAlmostEqual(hf.height(1.0, 2.0), 0.3, places=6)
        self.assertAlmostEqual(hf.height(1.0, 2.4), 0.3 * (0.5 + 0.5 * math.cos(math.pi * 0.4 / 0.8)), places=3)
        step = hf.res  # beyond the edge by a sample, where interpolation no longer reaches in
        self.assertEqual(hf.height(1.0, 2.0 + 0.8 + step), 0.0)  # along yaw (north) it ends at length / 2
        self.assertEqual(hf.height(1.0 + 0.35 + step, 2.0), 0.0)  # across it ends at width / 2
        hf = terrain.Heightfield(32, 513).washboard((-8.0, 0.0), 0.0, 16.0, 4.0, 0.04, 0.8)
        X, Y = hf.grid()
        middle = (np.abs(X) < 6) & (np.abs(Y) < 0.5)
        self.assertAlmostEqual(hf.z[middle].max(), 0.04, delta=0.002)
        self.assertAlmostEqual(hf.z[middle].min(), -0.04, delta=0.002)
        self.assertEqual(np.abs(hf.z[(np.abs(Y) > 2.0) | (X < -8.0) | (X > 8.0)]).max(), 0.0)

    def test_mesa_edge_is_where_the_top_ends(self):
        """mesa_edge (what landscape.Hills paints by): inside the edge the
        mesa is at its full height, a cliff width beyond it untouched."""
        hf = terrain.Heightfield(128, 257).mesa(10.0, -5.0, 20.0, 6.0, 8.0, seed=4)
        edge, r = hf.mesa_edge(10.0, -5.0, 20.0, seed=4)
        np.testing.assert_allclose(hf.z[r <= edge], 6.0)
        np.testing.assert_allclose(hf.z[r >= edge + 8.0], 0.0)
        self.assertTrue(np.all((hf.z[(r > edge) & (r < edge + 8.0)] > 0) & (hf.z[(r > edge) & (r < edge + 8.0)] < 6)))

    def test_relief_ops_chain_and_respect_their_mask(self):
        """detail, haystacks and rills edit z only under their mask and return
        the heightfield (terrain ops chain)."""
        hf = self.plane(gx=0.3, gy=0.0, size=128, n=257)
        before = hf.z.copy()
        mask = np.zeros((257, 257))
        mask[:, :128] = 1.0  # the west half

        class Swatch:
            windows = (np.random.default_rng(1).normal(0, 0.05, (200, 200)).astype(np.float32),)
            res_m = 0.5

        out = hf.detail(Swatch, mask, 1.0, seed=1).haystacks(mask, 2, terrains.Haystacks()).rills(mask, 3,
                                                                                                  terrains.Rills())
        self.assertIs(out, hf)
        np.testing.assert_array_equal(hf.z[:, 129:], before[:, 129:])
        self.assertGreater(np.abs(hf.z[:, :120] - before[:, :120]).max(), 1.0)  # knobs

    def test_png_keeps_its_own_maximum(self):
        """A heightmap normalised to its own maximum from z = 0, though its
        lowest point lies above 0 (a carved world's visual surface): Gazebo
        scales by the highest pixel and does not shift the lowest."""
        hf = self.plane()
        hf.z += 0.03 - hf.z.min()
        with tempfile.TemporaryDirectory() as d:
            _, top = hf.write_png(Path(d) / "h.png", 0.0)
            img = np.asarray(Image.open(Path(d) / "h.png")).astype(float)
        self.assertEqual(img.max(), 65535)
        self.assertGreater(img.min(), 0)
        np.testing.assert_allclose(img / 65535 * top, hf.z, atol=top / 65535)

    def test_png_round_trip(self):
        hf = self.plane()
        with tempfile.TemporaryDirectory() as d:
            zmin, zmax = hf.write_png(Path(d) / "h.png")
            img = np.asarray(Image.open(Path(d) / "h.png")).astype(float)
        self.assertEqual(img.max(), 65535)
        self.assertEqual(img.min(), 0)
        back = zmin + img / 65535 * (zmax - zmin)
        self.assertLess(np.max(np.abs(back - hf.z)), (zmax - zmin) / 65535)

    def test_geotiff_corners(self):
        hf = self.plane(size=100, n=101)
        with tempfile.TemporaryDirectory() as d:
            dem.write_geotiff(hf, Path(d) / "dem.tif", MDRS)
            img = Image.open(Path(d) / "dem.tif")
            tags = img.tag_v2
            z = np.asarray(img)
        lon0, lat0 = tags[dem.MODEL_TIEPOINT][3], tags[dem.MODEL_TIEPOINT][4]
        dlon, dlat = tags[dem.MODEL_PIXEL_SCALE][0], tags[dem.MODEL_PIXEL_SCALE][1]
        # Centre of the north-west pixel is the grid's (-50, 50).
        x, y, _ = geo.wgs84_to_enu(MDRS, lat0 - dlat / 2, lon0 + dlon / 2)
        self.assertAlmostEqual(x, -50, delta=0.05)
        self.assertAlmostEqual(y, 50, delta=0.05)
        self.assertAlmostEqual(float(z[0, 0]), MDRS.alt + hf.z[0, 0], places=2)


class Textures(unittest.TestCase):
    def detect(self, gray):
        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                           cv2.aruco.DetectorParameters())
        _, ids, _ = detector.detectMarkers(gray)
        return [] if ids is None else sorted(ids.flatten().tolist())

    def test_aruco_tags_decode_to_their_ids(self):
        for tag in (0, 1, 2, 5, 6, 49):
            img = textures.aruco_image(tag, px_per_cell=16)
            self.assertEqual(img.shape, (128, 128))  # 8 cells
            self.assertEqual(int(img[0, 0]), 255)  # white border cell
            self.assertEqual(int(img[16, 16]), 0)  # black border cell
            self.assertEqual(self.detect(img), [tag])

    def test_sign_and_terrain_textures(self):
        with tempfile.TemporaryDirectory() as d:
            textures.sign_image(Path(d) / "s.png", ["BRING", "WATER JUG"])
            textures.terrain_texture(Path(d) / "t.png", (190, 140, 100), seed=1)
            textures.normal_map(Path(d) / "n.png", seed=1)
            sign = np.asarray(Image.open(Path(d) / "s.png"))
            tex = np.asarray(Image.open(Path(d) / "t.png")).astype(float)
            normal = np.asarray(Image.open(Path(d) / "n.png")).astype(float)
        self.assertLess(sign.min(), 50)  # there is dark text
        self.assertLess(abs(tex[..., 0].mean() - 190 * 0.98), 25)
        self.assertGreater(normal[..., 2].mean(), 200)  # mostly facing out
        # Tileable: the left and right edges match.
        self.assertLess(np.abs(tex[:, 0] - tex[:, -1]).mean(), np.abs(tex[:, 0] - tex[:, 256]).mean() + 1)


class Meshes(unittest.TestCase):
    def test_rock_sits_on_its_origin(self):
        V, F = meshes.rock(3, (0.3, 0.2, 0.15))
        self.assertAlmostEqual(V[:, 2].min(), 0.0, places=9)
        self.assertLess(V[:, 0].max(), 0.3 * 1.3)
        self.assertGreater(V[:, 2].max(), 0.15)
        N = meshes.vertex_normals(V, F)
        top = np.argmax(V[:, 2])
        self.assertGreater(N[top, 2], 0.5)  # outward normals

    def test_rock_base_is_its_lowest_points(self):
        for variant in range(meshes.ROCK_VARIANTS):
            V = meshes.rock_variant(variant)[0]
            base = meshes.rock_base(variant)
            self.assertGreater(len(base), 8)
            np.testing.assert_allclose(base[:, 2], 0.0, atol=1e-9)
            self.assertAlmostEqual(V[:, 2].min(), 0.0, places=9)

    def test_tilt_lays_the_base_on_the_slope(self):
        for gx, gy in ((0.0, 0.0), (0.3, -0.2), (-1.2, 0.8)):
            R = meshes.tilt(gx, gy)
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(R), 1.0, places=12)
            # The base plane z = 0 lands on the plane z = gx x + gy y.
            P = np.array([(1, 0, 0), (0, 1, 0), (0.5, -0.7, 0)]) @ R.T
            np.testing.assert_allclose(P[:, 2], gx * P[:, 0] + gy * P[:, 1], atol=1e-12)

    def test_shrub_fills_a_unit_crown_on_its_origin(self):
        for variant in range(meshes.SHRUB_VARIANTS):
            V, F = meshes.shrub_lowpoly(variant)
            self.assertGreaterEqual(len(F), 6 * len(meshes.hull_faces(0)))  # at least six clumps
            np.testing.assert_allclose(V.min(axis=0), (-0.5, -0.5, 0.0), atol=1e-9)
            np.testing.assert_allclose(V.max(axis=0), (0.5, 0.5, 1.0), atol=1e-9)

    def test_quad_faces_plus_x(self):
        V, F, N, UV = meshes.quad()
        face_n = np.cross(V[F[0][1]] - V[F[0][0]], V[F[0][2]] - V[F[0][0]])
        self.assertGreater(face_n[0], 0)
        # Seen from +x, the viewer's right is +y: u grows with y, v with z.
        self.assertTrue(np.all(np.sign(UV[:, 0] - 0.5) == np.sign(V[:, 1])))
        self.assertTrue(np.all(np.sign(UV[:, 1] - 0.5) == np.sign(V[:, 2])))

    def test_obj_has_a_normal_and_uv_per_vertex(self):
        V, F, _, UV = meshes.quad()
        with tempfile.TemporaryDirectory() as d:
            meshes.write_obj(Path(d) / "m.obj", V, F, UV=UV)
            text = (Path(d) / "m.obj").read_text()
        self.assertEqual(text.count("\nvn "), len(V))
        self.assertEqual(text.count("\nvt "), len(V))
        self.assertIn("/", text.split("\nf ")[1])


def write_text(path, text):
    """A stand-in media generator (Media names a file by its generator and arguments)."""
    Path(path).write_text(text)


def copy_file(path, source, upper=False):
    """A stand-in generator that reads a file, as farfield.far_texture reads its raster."""
    text = Path(source).read_text()
    Path(path).write_text(text.upper() if upper else text)


class MediaNames(unittest.TestCase):
    """What a shared file's name covers (media._digest)."""

    def test_files_defaults_and_sources_are_in_the_name(self):
        """A file an argument names is part of the key (a re-fetched raster
        makes a new texture), as are defaults left out of the call, and the
        sources of the modules of this package the generator's module uses
        (farfield's texture depends on appearance and textures)."""
        with tempfile.TemporaryDirectory() as tmp:
            media = Media(Path(tmp) / "models")
            source = Path(tmp) / "raster.txt"
            source.write_text("one")
            first = media.texture("copy", copy_file, source)
            self.assertEqual(media.texture("copy", copy_file, source), first)  # current: made once
            self.assertEqual(media.texture("copy", copy_file, source, upper=False), first)  # the default
            source.write_text("two")
            second = media.texture("copy", copy_file, source)
            self.assertNotEqual(second, first)
            self.assertEqual((media.path(second)).read_text(), "two")
            self.assertNotEqual(media.texture("copy", copy_file, source, True), second)
        self.assertNotEqual(media_module._source_crc("urc.farfield"),
                            zlib.crc32(Path(sys.modules["urc.farfield"].__file__).read_bytes()))


class Prune(unittest.TestCase):
    """gen_worlds.prune deletes what no world on disk uses, and never while
    another run is still building: runs overlap when people regenerate their
    own worlds at the same time."""

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.models, self.worlds = Path(tmp.name) / "models", Path(tmp.name) / "worlds"
        self.worlds.mkdir()
        self.media = Media(self.models)

    def file(self, uri):
        return self.models / uri.removeprefix("model://")

    def model(self, name, *uris):
        """A generated model models/<name> whose model.sdf uses `uris`."""
        (self.models / name).mkdir()
        (self.models / name / "model.sdf").write_text("".join(f"<uri>{uri}</uri>" for uri in uris))

    def world(self, name, *uris):
        (self.worlds / f"{name}.sdf").write_text("".join(f"<uri>{uri}</uri>" for uri in uris))

    def test_removes_what_no_world_uses(self):
        used, unused = self.media.texture("used", write_text, "a"), self.media.texture("unused", write_text, "b")
        self.model("urc_used", used)
        self.model("urc_unused")
        self.world("w", "model://urc_used")
        (self.worlds / "copy.sdf").symlink_to(self.worlds / "gone.sdf")  # a temporary world copy, deleted since
        leftover = self.media.dir / "textures" / ".dead.12345.png"  # half written by a run that died
        leftover.write_text("")
        removed = gen_worlds.prune(self.models, self.worlds)
        self.assertEqual(sorted(removed), sorted([self.models / "urc_unused", self.file(unused), leftover]))
        self.assertTrue(self.file(used).is_file())
        self.assertTrue((self.models / "urc_used").is_dir())
        self.assertTrue((self.models / Media.NAME / "model.sdf").is_file())

    def test_waits_for_runs_still_building(self):
        """A run writes its media and models before its world file: a prune
        meanwhile would delete them, so it waits for the run's lock."""
        removed = []
        pruner = threading.Thread(target=lambda: removed.extend(gen_worlds.prune(self.models, self.worlds)))
        with gen_worlds.lock(self.models):
            uri = self.media.texture("new", write_text, "c")
            self.model("urc_new", uri)
            pruner.start()
            pruner.join(0.5)
            self.assertTrue(pruner.is_alive())
            self.world("w", "model://urc_new")
        pruner.join(10.0)
        self.assertFalse(pruner.is_alive())
        self.assertEqual(removed, [])
        self.assertTrue(self.file(uri).is_file())


if __name__ == "__main__":
    unittest.main()
