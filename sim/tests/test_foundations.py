#!/usr/bin/env python3
"""The shared foundations every workstream builds on (no physics; pixi run
sim-test): the one Gazebo environment (gzenv), the multi-band GeoTIFF reader,
the ground catalogue's recipe structure, the viewer cameras' module and the
test helpers of simulate.py."""
import contextlib
import dataclasses
import io
import os
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image, TiffImagePlugin, TiffTags

from simulate import ROVER_URI, twist_at, variant_sdf, world_sdf
from worldfiles import MODELS, SIM_DIR

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
import gzenv  # noqa: E402
import viewers  # noqa: E402
from urc import dem, geo, sdf, terrain, terrains  # noqa: E402

NAIP_PATH = SIM_DIR / "data" / "imagery" / "route_area_naip2024.tif"  # git-ignored; skipped where missing


class Environment(unittest.TestCase):
    BASE = {"PATH": "/usr/bin", "GZ_SIM_RESOURCE_PATH": "/elsewhere/models", "CONDA_PREFIX": "/env"}

    def test_our_models_and_plugins_come_first(self):
        env = gzenv.environment(base=self.BASE)
        self.assertEqual(env["GZ_SIM_RESOURCE_PATH"], os.pathsep.join([str(SIM_DIR / "models"), "/elsewhere/models"]))
        self.assertEqual(env["GZ_SIM_SYSTEM_PLUGIN_PATH"], str(SIM_DIR / "build"))
        self.assertEqual(env["OGRE2_RESOURCE_PATH"], "/env/lib/OGRE-Next")
        self.assertEqual(env["OGRE_RESOURCE_PATH"], "/env/lib/OGRE")
        self.assertEqual(env["PATH"], "/usr/bin")
        for key in ("GZ_PARTITION", "GZ_IP", "GZ_RENDERING_RESOURCE_PATH"):
            self.assertNotIn(key, env)

    def test_applying_it_twice_changes_nothing(self):
        once = gzenv.environment(base=self.BASE)
        self.assertEqual(gzenv.environment(base=once), once)

    def test_a_set_ogre_path_and_a_chosen_build_dir_win(self):
        with tempfile.TemporaryDirectory() as build:
            env = gzenv.environment(build, base=dict(self.BASE, OGRE2_RESOURCE_PATH="/mine"))
            self.assertEqual(env["OGRE2_RESOURCE_PATH"], "/mine")
            self.assertEqual(env["GZ_SIM_SYSTEM_PLUGIN_PATH"], str(Path(build).resolve()))

    def test_patched_media_only_once_complete(self):
        with tempfile.TemporaryDirectory() as build:
            media = Path(build) / gzenv.MEDIA_DIR
            media.mkdir()
            self.assertNotIn("GZ_RENDERING_RESOURCE_PATH", gzenv.environment(build, base=self.BASE))
            (media / gzenv.MEDIA_COMPLETE).touch()
            self.assertEqual(gzenv.environment(build, base=self.BASE)["GZ_RENDERING_RESOURCE_PATH"],
                             str(media.resolve()))

    def test_partition_and_ip_when_asked(self):
        env = gzenv.environment(base=self.BASE, partition="p", ip="127.0.0.1")
        self.assertEqual((env["GZ_PARTITION"], env["GZ_IP"]), ("p", "127.0.0.1"))

    def test_run_sh_gets_the_same_variables(self):
        """sim/run.sh evals what `python sim/gzenv.py` prints."""
        out = subprocess.run([sys.executable, str(SIM_DIR / "gzenv.py")], capture_output=True, text=True,
                             env=self.BASE, check=True).stdout
        script = f"{out}\n" + "".join(f'echo "{key}=${{{key}}}"\n' for key in gzenv.VARIABLES)
        shell = subprocess.run(["/bin/bash", "-c", script], capture_output=True, text=True, env=self.BASE,
                               check=True).stdout
        want = gzenv.environment(base=self.BASE)
        for line in shell.splitlines():
            key, value = line.split("=", 1)
            self.assertEqual(value, want.get(key, ""), key)
        self.assertIn('eval "$(python "$here/gzenv.py")"', (SIM_DIR / "run.sh").read_text())


def _write_four_band_geotiff(path, pixels, lon0, lat0, step, alpha=False):
    """A 4-band uint8 GeoTIFF (EPSG:4326, PixelIsArea). PIL writes four bands
    only as RGBA, the fourth marked alpha (ExtraSamples 2); unless alpha, the
    mark is patched to 0, unspecified, as in NAIP's files (R, G, B, NIR)."""
    keys = [v for key, (value, _) in dem.GEOKEYS.items() for v in (key, 0, 1, value)]
    ifd = TiffImagePlugin.ImageFileDirectory_v2()
    for tag, value, kind in ((dem.MODEL_PIXEL_SCALE, (step, step, 0.0), TiffTags.DOUBLE),
                             (dem.MODEL_TIEPOINT, (0.0, 0.0, 0.0, lon0, lat0, 0.0), TiffTags.DOUBLE),
                             (dem.GEO_KEY_DIRECTORY, (1, 1, 0, len(dem.GEOKEYS), *keys), TiffTags.SHORT)):
        ifd[tag] = value
        ifd.tagtype[tag] = kind
    Image.fromarray(pixels, "RGBA").save(path, tiffinfo=ifd, compression="tiff_deflate")
    if not alpha:  # the IFD entry: tag 338, type SHORT, count 1, value 2 (little-endian)
        entry = (dem.EXTRA_SAMPLES).to_bytes(2, "little") + b"\x03\x00\x01\x00\x00\x00"
        data = path.read_bytes()
        assert data.count(entry + b"\x02\x00") == 1
        path.write_bytes(data.replace(entry + b"\x02\x00", entry + b"\x00\x00"))


class Raster(unittest.TestCase):
    PIXELS = np.random.default_rng(0).integers(0, 256, (5, 7, 4), dtype=np.uint8)

    def test_four_bands_in_file_order(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "four.tif"
            _write_four_band_geotiff(path, self.PIXELS, -110.79, 38.43, 1e-5)
            bands, transform = dem.read_raster(path)
        self.assertEqual((bands.shape, bands.dtype), ((4, 5, 7), np.float32))
        np.testing.assert_array_equal(bands, np.moveaxis(self.PIXELS, -1, 0))
        self.assertEqual(transform, (-110.79, 1e-5, 0.0, 38.43, 0.0, -1e-5))

    def test_an_alpha_band_is_refused(self):
        """OpenCV multiplies the colour bands by a band marked alpha."""
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "alpha.tif"
            _write_four_band_geotiff(path, self.PIXELS, -110.79, 38.43, 1e-5, alpha=True)
            with self.assertRaisesRegex(ValueError, "alpha"):
                dem.read_raster(path)

    def test_a_dem_reads_as_one_band(self):
        origin = geo.Origin(38.42, -110.78, 1400.0)
        hf = terrain.Heightfield(64.0, 33, np.random.default_rng(1).normal(0, 2, (33, 33)))
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "dem.tif"
            dem.write_geotiff(hf, path, origin)
            bands, (lon0, dlon, _, lat0, _, minus_dlat) = dem.read_raster(path)
            d = dem.read_geotiff(path)
        self.assertEqual(bands.shape, (1, 33, 33))
        np.testing.assert_array_equal(bands[0], (hf.z + origin.alt).astype(np.float32))
        np.testing.assert_array_equal(d.z, bands[0])
        self.assertEqual((d.lon0, d.lat0, d.dlon, d.dlat), (lon0, lat0, dlon, -minus_dlat))

    @unittest.skipUnless(NAIP_PATH.is_file(), "NAIP 2024 is not linked (sim/tools/link_data.sh)")
    def test_naip_keeps_its_near_infrared(self):
        bands, _ = dem.read_raster(NAIP_PATH)
        self.assertEqual(bands.shape[0], 4)
        with Image.open(NAIP_PATH) as img:  # PIL: the three visible bands only
            rgb = np.moveaxis(np.asarray(img.convert("RGB")), -1, 0)
        np.testing.assert_array_equal(bands[:3], rgb)
        r, g, b, nir = bands[:, ::16, ::16].reshape(4, -1).mean(axis=1)
        self.assertGreater(r, b)  # pale tan desert (median #dfc8b0, design spec 5.7)
        self.assertNotAlmostEqual(nir, g, delta=1.0)


class Catalogue(unittest.TestCase):
    def test_defaults_are_todays_worlds(self):
        for t in list(terrains.TYPES.values()) + [terrains.calibration_surface(0.35)]:
            self.assertEqual(t.traction, terrains.Traction.coulomb(t.mu), t.key)
            self.assertEqual(t.appearance.palette.base, tuple(t.rgb), t.key)
            self.assertEqual((t.appearance.detail, t.appearance.dust), (None, 0.0), t.key)
            self.assertEqual(t.relief, terrains.Relief(), t.key)
            self.assertEqual(t.clutter, terrains.Clutter(), t.key)

    def test_traction_fields_are_the_ground_json_keys(self):
        """The traction table of ground.json (design spec 9.1) uses these names."""
        self.assertEqual([f.name for f in dataclasses.fields(terrains.Traction)],
                         ["mu_s", "mu_k", "crr", "bulldoze", "slip", "sinkage_m", "dig_rate", "dig_max"])

    def test_recipes_are_frozen_and_shared_by_reference(self):
        rills = terrains.Rills()
        a = terrains.TerrainType("a", "A", 0.5, (1, 2, 3), relief=terrains.Relief(rills=rills))
        b = terrains.TerrainType("b", "B", 0.5, (1, 2, 3), relief=terrains.Relief(rills=rills))
        self.assertIs(a.relief.rills, b.relief.rills)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            rills.depth_m = (0.0, 0.0)
        self.assertEqual(len({a, b, a}), 2)  # hashable, like every frozen recipe


class Viewers(unittest.TestCase):
    def test_gen_model_finds_the_viewer_names(self):
        for name in gen_model.VIEWER_NAMES:
            self.assertIs(getattr(gen_model, name), getattr(viewers, name), name)
        with self.assertRaises(AttributeError):
            gen_model.no_such_name  # noqa: B018

    def test_write_all_writes_the_checked_in_models(self):
        with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()):
            viewers.write_all(d)
            for name in (viewers.CHASE_MODEL, viewers.EYE_MODEL):
                for file in ("model.sdf", "model.config"):
                    self.assertEqual((Path(d) / name / file).read_text(), (MODELS / name / file).read_text())

    def test_camera_element(self):
        sensor = ET.Element("sensor")
        sdf.camera(sensor, 1.2, (640, 480), (0.1, 40.0), image_format="R8G8B8")
        self.assertEqual([(e.tag, e.text) for e in sensor.iter() if e.text],
                         [("horizontal_fov", "1.2"), ("width", "640"), ("height", "480"), ("format", "R8G8B8"),
                          ("near", "0.1"), ("far", "40")])


class SimulateHelpers(unittest.TestCase):
    def test_twist_schedule(self):
        schedule = [(1.0, 0.5, 0.0), (2.0, 0.0, 1.0)]
        self.assertEqual([twist_at(schedule, t) for t in (0.0, 1.0, 1.5, 2.0, 9.0)],
                         [(0.0, 0.0), (0.5, 0.0), (0.5, 0.0), (0.0, 1.0), (0.0, 1.0)])

    def test_world_options(self):
        world = ET.fromstring(world_sdf(rover_uri="file:///tmp/rover", default_surface="clay", solver="pgs"))
        self.assertEqual(world.findtext("world/physics/dart/solver/solver_type"), "pgs")
        ode = world.find("world/model/link/collision/surface/friction/ode")
        self.assertEqual((float(ode.findtext("mu")), float(ode.findtext("mu2"))), (terrains.CLAY.mu,) * 2)
        self.assertEqual(world.findtext("world/include/uri"), "file:///tmp/rover")
        plain = ET.fromstring(world_sdf())
        self.assertIsNone(plain.find("world/physics/dart"))
        self.assertIsNone(plain.find("world/model/link/collision/surface"))
        self.assertEqual(plain.findtext("world/include/uri"), ROVER_URI)

    def test_variant_of_a_world_file(self):
        text = variant_sdf(world_sdf(), rover_uri="file:///tmp/r", solver="dantzig")
        world = ET.fromstring(text)
        self.assertEqual(world.findtext("world/include/uri"), "file:///tmp/r")
        self.assertEqual(world.findtext("world/physics/dart/solver/solver_type"), "dantzig")
        self.assertEqual(world.findtext("world/physics/max_step_size"), "0.001")


if __name__ == "__main__":
    unittest.main()
