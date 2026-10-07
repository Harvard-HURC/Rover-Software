"""The appearance assets of the realism design (sections 5.5 meshes, 5.7, 7),
without Gazebo: the mission sun, clutter meshes and the GLB writer, the
shared detail textures, colour maps from a ground raster, Terra's
compensated layers, NAIP de-shading and shrub detection on the real
imagery, the far field and the soft-failing media patch tool."""
import datetime
import hashlib
import json
import math
import struct
import subprocess
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

from worldfiles import SIM_DIR

import gen_worlds  # noqa: E402  (worldfiles puts sim/ on the path)
import gzenv  # noqa: E402
from urc import appearance as A  # noqa: E402
from urc import dem, farfield, geo, lighting, meshes, sdf, terrain, terrains, textures  # noqa: E402
from urc.media import Media  # noqa: E402
from urc.missions import autonomy  # noqa: E402

import viewers  # noqa: E402

sys.path.insert(0, str(SIM_DIR / "tools"))
import gz_media  # noqa: E402

DATA = SIM_DIR / "data"
NAIP = DATA / "imagery" / "route_area_naip2024.tif"
LIDAR = DATA / "dem" / "route_area_lidar_0p5m.tif"


def read_glb(path):
    """(glTF JSON, binary chunk) of a GLB file."""
    data = Path(path).read_bytes()
    magic, version, length = struct.unpack_from("<III", data)
    assert (magic, version, length) == (0x46546C67, 2, len(data))
    n, kind = struct.unpack_from("<II", data, 12)
    assert kind == 0x4E4F534A
    gltf = json.loads(data[20:20 + n])
    m, kind = struct.unpack_from("<II", data, 20 + n)
    assert kind == 0x004E4942
    return gltf, data[28 + n:28 + n + m]


def accessor(gltf, binary, index):
    a = gltf["accessors"][index]
    view = gltf["bufferViews"][a["bufferView"]]
    dtype = {5126: np.float32, 5125: np.uint32}[a["componentType"]]
    width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3}[a["type"]]
    return np.frombuffer(binary, dtype, a["count"] * width, view["byteOffset"]).reshape(a["count"], width)


def signed_volume(V, F):
    V = np.asarray(V, float)
    return float(np.einsum("ij,ij->i", V[F[:, 0]], np.cross(V[F[:, 1]], V[F[:, 2]])).sum() / 6)


def closed(V, F):
    """Watertight and consistently wound: every directed edge has its
    reverse, vertices taken by position (faces may have vertices of their
    own, for sharp edges)."""
    _, ids = np.unique(np.round(np.asarray(V, float), 9), axis=0, return_inverse=True)
    G = ids.reshape(-1)[np.asarray(F)]
    edges = Counter((int(a), int(b)) for f in G for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])))
    return all(edges[(b, a)] == count for (a, b), count in edges.items())


def real_site():
    """The 0.5 m lidar DEM, an origin at its centre (NAVD88 altitude, as the DEM)."""
    d = dem.read_geotiff(LIDAR)
    (s, w), (n, e) = d.bounds
    lat, lon = (s + n) / 2, (w + e) / 2
    return d, geo.Origin(lat, lon, d.height(lat, lon))


class Sun(unittest.TestCase):
    def test_mission_sun(self):
        """Design spec D13: 2027-05-28 10:30 MDT at MDRS, elevation 49.8,
        azimuth 102 (the render prototype's 102.4, critique 102.5),
        direction (-0.630, 0.138, -0.764)."""
        s = lighting.MISSION
        self.assertAlmostEqual(s.elevation_deg, 49.85, delta=0.15)
        self.assertAlmostEqual(s.azimuth_deg, 102.4, delta=0.3)
        np.testing.assert_allclose(s.direction, (-0.630, 0.138, -0.764), atol=0.005)
        self.assertAlmostEqual(float(np.linalg.norm(s.direction)), 1.0, places=9)

    def test_solstice_noon(self):
        """At the June solstice the sun culminates due south at 90 - lat +
        23.44 deg (plus refraction, < 0.01 deg there)."""
        day = [lighting.sun(datetime.date(2027, 6, 21), datetime.time(h, m, tzinfo=lighting.MDT),
                            *lighting.MISSION_SITE) for h in range(10, 16) for m in range(0, 60, 2)]
        top = max(day, key=lambda s: s.elevation_deg)
        self.assertAlmostEqual(top.elevation_deg, 90 - 38.418 + 23.44, delta=0.1)
        self.assertAlmostEqual(top.azimuth_deg, 180.0, delta=1.0)


class Meshes(unittest.TestCase):
    def test_slabs(self):
        """Closed, outward-facing tabular blocks: equivalent diameter 1
        (the bottom face has a unit disc's area), height SLAB_HEIGHT."""
        for variant in range(meshes.SLAB_VARIANTS):
            V, F = meshes.slab(variant)
            self.assertTrue(closed(V, F))
            self.assertEqual(V[:, 2].min(), 0.0)
            self.assertAlmostEqual(V[:, 2].max(), meshes.SLAB_HEIGHT)
            base = V[F][np.all(V[F][:, :, 2] == 0, axis=1)]
            area = 0.5 * np.abs(np.cross(base[:, 1] - base[:, 0], base[:, 2] - base[:, 0])[:, 2]).sum()
            self.assertAlmostEqual(area, math.pi / 4, places=9)
            volume = signed_volume(V, F)
            self.assertGreater(volume, 0.9 * area * meshes.SLAB_HEIGHT)
            self.assertLessEqual(volume, area * meshes.SLAB_HEIGHT)

    def test_riser_faces_downhill(self):
        """Along +x the face looks to -y and the top reaches `depth` to +y,
        `height` above the ground at the foot, buried `bury` deep; the ground
        rises 0.5 m per metre to +y, so the level top's back is in it."""
        def ground(x, y):
            return 1.0 + np.asarray(x) / 10 + 0.5 * np.asarray(y)

        V, F = meshes.riser_strip([(0, 0, 1.0), (10, 0, 2.0)], 0.5, 2.0, ground, seed=3, bury=0.3)
        self.assertTrue(closed(V, F))
        self.assertGreater(signed_volume(V, F), 0)
        self.assertAlmostEqual(V[:, 1].max(), 2.0, places=6)
        self.assertLess(V[:, 1].min(), 0.05)
        self.assertGreater(V[:, 1].min(), -0.05)
        above = V[:, 2] - (1.0 + V[:, 0] / 10)  # above the ground at the foot
        self.assertAlmostEqual(above.min(), -0.3, places=9)
        np.testing.assert_allclose(above[above > 0.3], 0.5, atol=0.5 * 0.05 + 1e-9)
        back = V[np.abs(V[:, 1] - 2.0) < 1e-6]
        self.assertTrue(np.all(back[:, 2] < ground(back[:, 0], back[:, 1])), "the slope buries the back")

    def test_riser_on_level_ground_is_a_ledge(self):
        """On level ground the top falls back into the ground (RISER_DIP at
        most, reaching past `depth` when it must): no edge of the back stands
        above the ground, unlike a level-topped block (a wall)."""
        V, F = meshes.riser_strip([(0, 0), (10, 0)], 0.8, 1.0, lambda x, y: np.zeros(np.shape(x)), seed=4)
        self.assertTrue(closed(V, F))
        self.assertGreater(signed_volume(V, F), 0)
        proud = V[V[:, 2] > 1e-9]  # the face's top edge only
        self.assertLess(proud[:, 1].max(), 0.05)
        reach = V[:, 1].max()
        self.assertAlmostEqual(reach, 0.8 * 1.05 / math.tan(math.radians(meshes.RISER_DIP)), delta=0.2)
        self.assertLessEqual(V[:, 2][np.abs(V[:, 1] - V[:, 1].max()) < 1e-6].max(), -meshes.RISER_BACK_BURY + 1e-9)

    def test_low_shrubs_and_pebbles(self):
        for variant in range(meshes.SHRUB_VARIANTS):
            V, F = meshes.shrub_lowpoly(variant)
            np.testing.assert_allclose(V.min(axis=0), (-0.5, -0.5, 0.0), atol=1e-9)
            np.testing.assert_allclose(V.max(axis=0), (0.5, 0.5, 1.0), atol=1e-9)
            self.assertLessEqual(len(F), 200)
        self.assertEqual(len(meshes.pebble(0, 0)[1]), 20)
        V, _ = meshes.pebble(1)
        self.assertEqual(V[:, 2].min(), 0.0)

    def test_glb(self):
        """A two-material GLB: positions as given (Z-up), normals, UVs,
        indices, an embedded texture."""
        V, F = meshes.slab(0)
        V2 = V + (0.0, 0.0, 5.0)
        with tempfile.TemporaryDirectory() as d:
            png = Path(d) / "t.png"
            Image.new("RGB", (4, 4), (10, 200, 30)).save(png)
            path = Path(d) / "m.glb"
            meshes.write_glb_parts(path, [(V, F, None, None, meshes.Material((0.5, 0.4, 0.3))),
                                          (V2, F, None, V2[:, :2], meshes.Material(texture=str(png)))])
            gltf, binary = read_glb(path)
            embedded = gltf["bufferViews"][gltf["images"][0]["bufferView"]]
            self.assertEqual(binary[embedded["byteOffset"]:embedded["byteOffset"] + embedded["byteLength"]],
                             png.read_bytes())
        primitives = gltf["meshes"][0]["primitives"]
        self.assertEqual(len(primitives), 2)
        np.testing.assert_allclose(accessor(gltf, binary, primitives[1]["attributes"]["POSITION"]), V2, atol=1e-6)
        np.testing.assert_array_equal(accessor(gltf, binary, primitives[0]["indices"]).reshape(-1, 3), F)
        normals = accessor(gltf, binary, primitives[0]["attributes"]["NORMAL"])
        np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1.0, atol=1e-5)
        self.assertNotIn("TEXCOORD_0", primitives[0]["attributes"])
        self.assertAlmostEqual(gltf["accessors"][primitives[1]["attributes"]["POSITION"]]["max"][2], V2[:, 2].max(),
                               places=6)
        self.assertEqual(gltf["materials"][1]["pbrMetallicRoughness"]["baseColorTexture"], {"index": 0})
        np.testing.assert_allclose(accessor(gltf, binary, primitives[1]["attributes"]["TEXCOORD_0"]), V2[:, :2],
                                   atol=1e-6)
        self.assertEqual(gltf["images"][0]["mimeType"], "image/png")
        self.assertEqual(gltf["materials"][0]["pbrMetallicRoughness"]["baseColorFactor"], [0.5, 0.4, 0.3, 1.0])


class MediaKinds(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.models, self.worlds = Path(tmp.name) / "models", Path(tmp.name) / "worlds"
        self.worlds.mkdir()
        self.media = Media(self.models)

    def test_glb_kind_is_pruned_like_the_rest(self):
        def slab_glb(path, variant):
            meshes.write_glb(path, *meshes.slab(variant))

        used, unused = self.media.glb("slab", slab_glb, 0), self.media.glb("slab", slab_glb, 1)
        self.assertTrue(used.startswith("model://urc_media/meshes/slab_") and used.endswith(".glb"))
        read_glb(self.media.path(used))
        (self.models / "urc_w").mkdir()
        (self.models / "urc_w" / "model.sdf").write_text(f"<uri>{used}</uri>")
        (self.worlds / "w.sdf").write_text("<uri>model://urc_w</uri>")
        removed = gen_worlds.prune(self.models, self.worlds)
        self.assertEqual(removed, [self.media.path(unused)])
        self.assertTrue(self.media.path(used).is_file())

    def test_shared_textures(self):
        flat = np.asarray(Image.open(self.media.path(self.media.flat_normal())))
        self.assertTrue(np.all(flat == (128, 128, 255)))
        puff = np.asarray(Image.open(self.media.path(self.media.dust_puff(terrains.DUST_RGB, 1.0))))
        self.assertEqual(puff.shape[2], 4)
        self.assertEqual(puff[0, 0, 3], 0)
        self.assertGreater(puff[54:74, 54:74, 3].mean(), 100)
        faint = np.asarray(Image.open(self.media.path(self.media.dust_puff(terrains.DUST_RGB, 0.25))))
        self.assertAlmostEqual(faint[..., 3].max(), 0.25 * puff[..., 3].max(), delta=1.0)
        with self.assertRaises(ValueError):
            self.media.path("model://urc_terrain_x/heightmap.png")


class DetailTextures(unittest.TestCase):
    def test_tileable(self):
        """Every detail texture wraps without a seam: the jump across the
        tile edge is no larger than between neighbouring texels inside."""
        for key in textures.DETAILS:
            albedo, height = textures._detail_fields(key)
            for field in (textures.linear_to_srgb(albedo).astype(float), height):
                inside = np.abs(np.diff(field, axis=1)).mean()
                across = np.abs(field[:, 0] - field[:, -1]).mean()
                self.assertLess(across, 2.5 * inside + 1e-9, key)
                inside = np.abs(np.diff(field, axis=0)).mean()
                across = np.abs(field[0] - field[-1]).mean()
                self.assertLess(across, 2.5 * inside + 1e-9, key)

    def test_modulated_to_a_mean(self):
        with tempfile.TemporaryDirectory() as d:
            media = Media(d)
            target = (0.5, 0.4, 0.3)
            diffuse, normal = media.detail("cracked_silt", target)
            self.assertEqual(normal, media.detail("cracked_silt")[1])
            mean = textures.srgb_to_linear(np.asarray(Image.open(media.path(diffuse)))).reshape(-1, 3).mean(0)
            np.testing.assert_allclose(mean, target, atol=0.004)


def flat_world(size=64.0, n=129):
    hf = terrain.Heightfield(size, n)
    hf.z[:] = 0.0
    return hf


def kind(key, base, p10=None, p90=None):
    return terrains.TerrainType(key, key, 1.0, base, appearance=terrains.Appearance(terrains.Palette(base, p10, p90)))


class ColourMap(unittest.TestCase):
    def test_types_keep_their_palette(self):
        """Design spec 11: per-type medians within dE 10 of the palette; the
        spread follows p10-p90 where it is measured."""
        hf = flat_world()
        raster = np.zeros(hf.z.shape, np.uint8)
        raster[:64, 65:] = 1
        raster[65:, :] = 2
        types = [kind("a", (190, 150, 115)), kind("b", (181, 157, 149), (160, 138, 135), (195, 170, 161)),
                 kind("c", (105, 97, 73))]
        cm = A.colour_map(hf, raster, types, np.random.default_rng(1), n=512)
        quarter = {0: cm[20:100, 20:100], 1: cm[20:100, 300:480], 2: cm[330:490, 20:490]}
        for index, block in quarter.items():
            median = np.median(block.reshape(-1, 3), axis=0)
            self.assertLess(float(A.delta_e(median, types[index].appearance.palette.base)), 10, types[index].key)
        b = quarter[1].reshape(-1, 3).astype(float)
        p10, p90 = np.percentile(b, 10, axis=0), np.percentile(b, 90, axis=0)
        np.testing.assert_allclose(p90 - p10, np.subtract((195, 170, 161), (160, 138, 135)), rtol=0.4)

    def test_row_zero_is_north_and_dots_are_drawn(self):
        hf = flat_world()
        raster = np.zeros(hf.z.shape, np.uint8)
        raster[:20] = 1  # the north edge
        types = [kind("a", (200, 180, 150)), kind("b", (90, 90, 160))]
        cm = A.colour_map(hf, raster, types, np.random.default_rng(2), n=256, dots=[(10.0, -10.0, 2.0)])
        self.assertGreater(cm[:30, :, 2].mean(), cm[:30, :, 0].mean())  # blue in the north
        self.assertLess(cm[-30:, :, 2].mean(), cm[-30:, :, 0].mean())
        row, col = int((32 + 10) / 64 * 256), int((32 + 10) / 64 * 256)  # world (10, -10)
        np.testing.assert_allclose(cm[row, col], A.DOT_RGB, atol=6)

    def test_strata_bands(self):
        """A banded type on a hillside shows several of its bands' colours
        in layers that follow the contours (undipped here)."""
        hf = flat_world(128.0, 257)
        X, _ = hf.grid()
        hf.z = 0.2 * (X - X.min())  # 25 m up from west to east
        raster = np.ones(hf.z.shape, np.uint8)
        types = [kind("a", (200, 180, 150)), kind("badland", (181, 157, 149))]
        level = A.Strata(dip_deg=0.0, wobble_m=0.0)
        cm = A.colour_map(hf, raster, types, np.random.default_rng(3), n=512, strata={"badland": level})
        nearest = np.argmin(A.delta_e(cm.reshape(-1, 1, 3), np.array(level.bands)[None]), axis=1).reshape(512, 512)
        self.assertGreaterEqual(len(np.unique(nearest[256])), 3)  # across the contours
        for column in (100, 256, 400):  # along a contour: one band
            same = np.mean(nearest[:, column] == np.bincount(nearest[:, column]).argmax())
            self.assertGreater(same, 0.9)

    def test_slope_darkening_is_mild(self):
        hf = flat_world()
        X, _ = hf.grid()
        steep = terrain.Heightfield(hf.size, hf.n, X.copy())  # 45 deg
        types = [kind("a", (200, 180, 150))]
        raster = np.zeros(hf.z.shape, np.uint8)
        flat = A.colour_map(hf, raster, types, np.random.default_rng(4), n=128)
        tilted = A.colour_map(steep, raster, types, np.random.default_rng(4), n=128)
        ratio = textures.luminance(textures.srgb_to_linear(tilted[20:-20, 20:-20])).mean() / \
            textures.luminance(textures.srgb_to_linear(flat[20:-20, 20:-20])).mean()
        self.assertAlmostEqual(ratio, 1 - A.SLOPE_DARKENING, delta=0.01)

    def test_lab(self):
        """The package's one CIE Lab (textures): sRGB round trips, white is L
        100, and CIE76 is the Lab distance (appearance.delta_e)."""
        rgb = np.array([(120, 200, 40), (237, 176, 132), (0, 0, 0), (255, 255, 255)], np.uint8)
        np.testing.assert_array_equal(textures.lab_to_srgb(textures.srgb_to_lab(rgb)), rgb)
        np.testing.assert_allclose(textures.srgb_to_lab((255, 255, 255)), (100.0, 0.0, 0.0), atol=0.02)
        self.assertAlmostEqual(float(A.delta_e((237, 176, 132), (230, 198, 158))),
                               float(np.linalg.norm(textures.srgb_to_lab((237, 176, 132))
                                                    - textures.srgb_to_lab((230, 198, 158)))))


class ZoneTint(unittest.TestCase):
    def test_zones_show_on_an_orthophoto(self):
        """A zone of a type the picture does not show (clay on sand sheet)
        takes ZONE_TINT of its catalogue palette, the picture's light and
        shade kept; one of a type the picture shows elsewhere takes that
        ground's own colour there; the picture outside stays as it was."""
        n, size = 256, 128.0
        sheet, sand = terrains.TYPES["sand_sheet"], terrains.TYPES["sand"]
        kinds = [sheet, terrains.CLAY, sand]
        painted = np.zeros((n, n), np.int32)
        painted[200:, :] = 2  # sand along the south: the picture shows it there
        rgb = np.zeros((n, n, 3), np.uint8)
        rgb[:] = (230, 198, 158)
        rgb[200:] = (240, 215, 185)
        rgb[1::3, ::3] = (220, 188, 150)  # light and shade
        zones = np.full((n, n), -1, np.int32)
        zones[60:100, 60:100] = 1  # clay
        zones[60:100, 150:190] = 2  # sand, on the sand sheet
        colours = A.zone_colours(rgb, painted, kinds)
        self.assertEqual(colours[1], terrains.CLAY.appearance.palette.base)
        self.assertEqual(colours[2], (240, 215, 185))
        out = A.tint_zones(rgb, zones, colours, size)
        np.testing.assert_array_equal(out[:40], rgb[:40])
        lin = textures.srgb_to_linear
        for (r, c), index in (((80, 80), 1), ((80, 170), 2)):
            mean = lin(out[r - 6:r + 6, c - 6:c + 6]).reshape(-1, 3).mean(axis=0)
            before = lin(rgb[r - 6:r + 6, c - 6:c + 6]).reshape(-1, 3).mean(axis=0)
            target = lin(colours[index])
            np.testing.assert_allclose(mean, (1 - A.ZONE_TINT) * before + A.ZONE_TINT * target, atol=0.02)
            self.assertGreater(np.ptp(out[r - 6:r + 6, c - 6:c + 6, 1]), 3)  # the shade is still there


class TerraLayers(unittest.TestCase):
    def test_compensation_reproduces_the_colour_map(self):
        """Design spec 11: Terra's lerp chain over the compensated layer 0
        gives back the colour map to 1 DN (linear light), detail means as
        saved; constant layers keep their weight over the terrain."""
        hf = terrain.Heightfield(256.0, 513).noise(12.0, 80.0, 5)
        hf.z -= hf.z.min()
        rng = np.random.default_rng(6)
        cm = np.clip(np.array((185, 160, 130)) + rng.normal(0, 12, (256, 256, 3)), 0, 255).astype(np.uint8)
        details = A.DEFAULT_DETAILS + (A.DetailLayer("slab_joints", 0.4, above=float(hf.z.max()) * 0.6),)
        with tempfile.TemporaryDirectory() as d:
            media = Media(d)
            layers = A.terra_layers(cm, hf, details, media)
            heights = A.at_texels(hf, 256)
            out = textures.srgb_to_linear(layers.layer0)
            for (diffuse, _, tile), (low, fade), detail in zip(layers.textures, layers.blends, details):
                self.assertEqual(tile, textures.DETAILS[detail.key].tile_m)
                mean = textures.srgb_to_linear(np.asarray(Image.open(media.path(diffuse)))).reshape(-1, 3).mean(0)
                w = terrain.smoothstep(low, low + fade, heights)
                if detail.above is None:
                    self.assertLess(np.abs(w - detail.weight).max(), 0.02)
                else:
                    self.assertEqual(float(w[heights <= detail.above].max(initial=0)), 0.0)
                    self.assertAlmostEqual(float(w.max()), detail.weight, places=3)
                out = out * (1 - w[..., None]) + w[..., None] * mean
        self.assertLess(layers.clipped, 0.01)
        error = np.abs(textures.linear_to_srgb(out).astype(int) - cm.astype(int))
        self.assertLessEqual(int(np.percentile(error, 99.9)), 1)
        unclipped = np.all((layers.layer0 > 0) & (layers.layer0 < 255), axis=-1)
        self.assertLessEqual(error[unclipped].max(), 2)  # 8-bit layer 0 / a_0

    def test_sdf(self):
        hf = flat_world()
        hf.z = hf.z + np.linspace(0, 1, hf.n)[None, :]
        root, model = sdf.model_root("t")
        heightmap = sdf.sub(model, "heightmap")
        with tempfile.TemporaryDirectory() as d:
            media = Media(d)
            layers = A.terra_layers(np.full((64, 64, 3), 150, np.uint8), hf, A.DEFAULT_DETAILS, media)
            layers.write(heightmap, "model://w/colour.png", media.flat_normal(), hf.size)
        found = heightmap.findall("texture")
        self.assertEqual(len(found), 3)
        self.assertEqual((found[0].findtext("diffuse"), float(found[0].findtext("size"))),
                         ("model://w/colour.png", hf.size))
        self.assertEqual(len(heightmap.findall("blend")), 2)
        with self.assertRaises(ValueError):
            A.terra_layers(np.zeros((8, 8, 3), np.uint8), hf, A.DEFAULT_DETAILS * 2, None)

    def test_inverse_smoothstep(self):
        for w in (0.0, 0.15, 0.35, 0.5, 0.9, 1.0):
            t = A.inverse_smoothstep(w)
            self.assertAlmostEqual(3 * t * t - 2 * t ** 3, w, places=12)


class Orthophoto(unittest.TestCase):
    """On the real route area: NAIP 2024 and the 0.5 m lidar DEM."""

    @classmethod
    def setUpClass(cls):
        cls.dem, cls.origin = real_site()

    def test_resampled_on_the_world_grid(self):
        """resample_raster puts a raster where dem.to_heightfield does."""
        size, n, center = 200.0, 400, (150.0, -250.0)  # 0.5 m texels: no area averaging
        z = A.resample_raster(LIDAR, self.origin, size, n, center)[0] - self.origin.alt
        hf = dem.to_heightfield(self.dem, self.origin, size, 2 * n + 1, center)
        np.testing.assert_allclose(z, hf.z[1::2, 1::2], atol=0.05)

    def test_deshading_removes_the_shading(self):
        """Design spec 11: after de-shading |corr(luminance, cos i)| < 0.1 on
        slopes over 10 deg; NAIP 2024 was shot in the morning (sun in the
        east-south-east, high)."""
        size, n, center = 600.0, 1200, (-300.0, 200.0)
        hf = dem.to_heightfield(self.dem, self.origin, size, n + 1, center)
        naip = A.resample_raster(NAIP, self.origin, size, n, center)
        lin, shadow, sun = A.deshade(naip, hf)
        self.assertTrue(80 < sun.azimuth_deg < 130, sun)
        self.assertTrue(40 < sun.elevation_deg < 70, sun)
        normals = A.surface_normals(A.at_texels(hf, n), size / n)
        cos_i = A.illumination(normals, sun)
        steep = (np.degrees(np.arccos(normals[..., 2])) > 10) & ~shadow
        self.assertGreater(steep.mean(), 0.02)
        before = textures.luminance(np.moveaxis(textures.srgb_to_linear(naip[:3]), 0, -1))
        self.assertGreater(np.corrcoef(before[steep], cos_i[steep])[0, 1], 0.25)
        self.assertLess(abs(np.corrcoef(textures.luminance(lin)[steep], cos_i[steep])[0, 1]), 0.1)
        self.assertTrue(np.all(shadow[cos_i <= A.SHADOW_COS]))

    def test_cast_shadows(self):
        """A 10 m wall shades the ground behind it out to 10 / tan(elev)."""
        h = np.zeros((200, 200), np.float32)
        h[:, 100:102] = 10.0
        shadow = A.cast_shadows(h, 0.5, A.FittedSun(45.0, 90.0))  # sun in the east: shade to the west
        row = shadow[100]
        self.assertTrue(row[81:100].all())
        self.assertFalse(row[:78].any())
        self.assertFalse(row[102:].any())

    def test_shrubs_pass_the_ndvi_filter(self):
        """Design spec 11: detections are dark spots that pass the
        NDVI-anomaly test; 53-68 % of dark-spot pixels pass it (M), so the
        filter keeps a like share of the spots; 38-92 dark spots per hectare
        in the route area before the test (M). The whole Autonomy-sized
        square: shares vary from place to place."""
        size, center = 2048.0, (0.0, 0.0)
        found = A.detect_shrubs(NAIP, self.origin, size, center=center)
        threshold = A.NDVI_ANOMALY
        try:
            A.NDVI_ANOMALY = -1.0
            spots = A.detect_shrubs(NAIP, self.origin, size, center=center)
        finally:
            A.NDVI_ANOMALY = threshold
        self.assertTrue(set(found) <= set(spots))
        hectares = size * size / 1e4
        self.assertTrue(38 < len(spots) / hectares < 92, len(spots) / hectares)
        self.assertTrue(0.5 < len(found) / len(spots) < 0.8, len(found) / len(spots))
        d = np.array([s[2] for s in found])
        self.assertTrue(np.all((d >= 2 * math.sqrt(A.SHRUB_AREA_M2[0] / math.pi) - 1e-9)
                               & (d <= 2 * math.sqrt(A.SHRUB_AREA_M2[1] / math.pi) + 1e-9)))
        xs = np.array([s[0] for s in found])
        self.assertTrue(np.all(np.abs(xs - center[0]) <= size / 2))

    def test_ortho_colour_map(self):
        size, n, center = 300.0, 600, (100.0, 100.0)
        hf = dem.to_heightfield(self.dem, self.origin, size, n + 1, center)
        spots = [(120.0, 110.0, 2.0)]
        mask = A.disc_mask(spots, size, n, center)
        self.assertAlmostEqual(mask.sum() * (size / n) ** 2, math.pi, delta=1.0)  # 2 m disc, 0.5 m texels
        ortho = A.ortho_colour_map(NAIP, self.origin, size, n, hf, inpaint_mask=mask, center=center)
        self.assertEqual((ortho.rgb.shape, ortho.rgb.dtype), ((n, n, 3), np.uint8))
        self.assertTrue(np.all(ortho.inpainted[mask]))
        raw = A.ortho_colour_map(NAIP, self.origin, size, n, center=center, boost=A.Boost(1.0, 1.0))
        self.assertIsNone(raw.sun)

        def chroma(rgb):
            lin = textures.srgb_to_linear(rgb)
            return float(np.linalg.norm(lin - textures.luminance(lin)[..., None], axis=-1).mean())

        self.assertGreater(chroma(ortho.rgb), 1.15 * chroma(raw.rgb))


class FarField(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.models = Path(cls.tmp.name)
        cls.media = Media(cls.models)
        cls.far = dem.read_geotiff(farfield.FAR_DEM)
        lat, lon = lighting.MISSION_SITE
        cls.origin = geo.Origin(lat, lon, cls.far.height(lat, lon) + dem.NAVD88_TO_WGS84)
        cls.size = 2048.0
        # A world on the real DEM, as Autonomy: its terrain is the far DEM itself (NAVD88 + the geoid: ellipsoidal).
        cls.terrain = dem.to_heightfield(cls.far, geo.Origin(lat, lon, cls.origin.alt - dem.NAVD88_TO_WGS84),
                                         cls.size, 257, (0.0, 0.0))
        farfield.build(cls.models, cls.media, "urc_far_a", cls.origin, cls.terrain)
        gltf, binary = read_glb(cls.models / "urc_far_a" / "meshes" / "farfield.glb")
        p = gltf["meshes"][0]["primitives"][0]
        cls.V = accessor(gltf, binary, p["attributes"]["POSITION"]).astype(float)
        cls.F = accessor(gltf, binary, p["indices"]).reshape(-1, 3)
        cls.UV = accessor(gltf, binary, p["attributes"]["TEXCOORD_0"])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_extent_and_hole(self):
        """65 x 80 km around the world, no triangle over the terrain."""
        lo, hi = self.V.min(axis=0), self.V.max(axis=0)
        self.assertGreater(hi[0] - lo[0], 60_000)
        self.assertGreater(hi[1] - lo[1], 75_000)
        half = self.size / 2
        over = np.all((np.abs(self.V[self.F, 0]) < half - 1) & (np.abs(self.V[self.F, 1]) < half - 1), axis=1)
        self.assertFalse(over.any())
        self.assertTrue(np.all((self.UV >= 0) & (self.UV <= 1)))
        self.assertLess(len(self.F), 900_000)

    def test_heights_curvature_and_seam(self):
        """Vertex heights: the DEM (ellipsoidal) minus the origin's altitude,
        minus the curvature drop (1 - k) r^2 / 2R; the seam SINK below the
        lowest terrain within a grid step, also on a real DEM."""
        r = np.hypot(self.V[:, 0], self.V[:, 1])
        drops = []
        for i in np.flatnonzero(r > 4 * self.size)[::997]:
            x, y, z = self.V[i]
            lat, lon, _ = geo.enu_to_wgs84(self.origin, x, y)
            expected = self.far.height(lat, lon) + dem.NAVD88_TO_WGS84 - self.origin.alt
            drops.append((1 - farfield.REFRACTION) * (x * x + y * y) / (2 * farfield.EARTH_RADIUS))
            self.assertAlmostEqual(z, expected - drops[-1], delta=2.0)  # remap's 1/32 px on mountain slopes
        self.assertGreater(max(drops), 40)  # some vertices are > 25 km away
        seam = np.flatnonzero(np.maximum(np.abs(self.V[:, 0]), np.abs(self.V[:, 1])) < self.size / 2)
        X, Y = self.terrain.grid()
        for i in seam[::7]:
            x, y, z = self.V[i]
            window = (np.abs(X - x) <= farfield.SPACING) & (np.abs(Y - y) <= farfield.SPACING)
            self.assertLessEqual(z, self.terrain.z[window].min() - farfield.SINK + 0.01)

    def test_never_above_the_terrain(self):
        """Inside the square the far field's edge triangles (they reach a
        grid step in) stay under the terrain: sampled every 4 m."""
        half = self.size / 2
        inside = self.F[np.any(np.maximum(np.abs(self.V[self.F, 0]), np.abs(self.V[self.F, 1])) < half, axis=1)]
        self.assertGreater(len(inside), 100)
        w = np.stack(np.meshgrid(np.linspace(0, 1, 31), np.linspace(0, 1, 31)), -1).reshape(-1, 2)
        w = w[w.sum(axis=1) <= 1]  # barycentric samples of a triangle
        a, b, c = (self.V[inside[:, k]][:, None, :] for k in range(3))
        P = (a + w[None, :, :1] * (b - a) + w[None, :, 1:] * (c - a)).reshape(-1, 3)
        P = P[np.maximum(np.abs(P[:, 0]), np.abs(P[:, 1])) < half]
        self.assertGreater(len(P), 10_000)
        self.assertLess(float((P[:, 2] - self.terrain.height(P[:, 0], P[:, 1])).max()), 0.0)

    def test_apron_for_the_fly_camera(self):
        """apron.json holds the mesh's own vertex heights round the terrain,
        APRON_REACH beyond its edge and more, past the fly camera's margin."""
        apron = json.loads((self.models / "urc_far_a" / farfield.APRON).read_text())
        self.assertEqual(apron["format"], "rover-apron/1")
        z = np.array(apron["z"]).reshape(apron["rows"], apron["cols"])
        x = apron["x0"] + apron["spacing"] * np.arange(apron["cols"])
        y = apron["y0"] - apron["spacing"] * np.arange(apron["rows"])
        reach = self.size / 2 + farfield.APRON_REACH
        self.assertTrue(x[0] <= -reach and x[-1] >= reach and y[-1] <= -reach and y[0] >= reach)
        self.assertGreaterEqual(farfield.APRON_REACH, viewers.FlyParams().margin + farfield.SPACING)
        for r, c in ((0, 0), (3, 5), (apron["rows"] - 1, apron["cols"] - 1)):
            vertex = np.flatnonzero(np.hypot(self.V[:, 0] - x[c], self.V[:, 1] - y[r]) < 1e-3)
            self.assertEqual(len(vertex), 1)
            self.assertAlmostEqual(self.V[vertex[0], 2], z[r, c], delta=2e-3)

    def test_overview_matches_the_near_imagery(self):
        """The far texture is the NAIP 2021 overview brought to NAIP 2024's
        colour: over the Autonomy square the two, boosted alike, differ by a
        median colour of CIE76 < 3 (11.8 unscaled)."""
        n = 64
        origin = geo.Origin(*autonomy.SQUARE_MILE_CENTER, 0.0)
        far = A.resample_raster(farfield.FAR_IMAGERY, origin, self.size, n)
        near = A.resample_raster(autonomy.NAIP_PATH, origin, self.size, n)
        lin = [np.moveaxis(textures.srgb_to_linear(r[:3]), 0, -1).reshape(-1, 3) for r in (far, near)]
        lin[0] = lin[0] * np.asarray(farfield.OVERVIEW_TO_NAIP2024, np.float32)
        far_median, near_median = (textures.linear_to_srgb(np.median(A.NAIP2024_BOOST.apply(v), axis=0)) for v in lin)
        self.assertLess(float(A.delta_e(far_median, near_median)), 3.0)

    def test_shared_texture_and_a_synthetic_terrain(self):
        """A second world shares the texture; over a synthetic terrain the
        seam lies SINK below the lowest ground within a grid step."""
        hf = terrain.Heightfield(512.0, 513).noise(15.0, 100.0, 9)
        hf.z += 30.0
        farfield.build(self.models, self.media, "urc_far_b", self.origin, hf)
        texts = [(self.models / name / "model.sdf").read_text() for name in ("urc_far_a", "urc_far_b")]
        maps = [t.split("<albedo_map>")[1].split("</albedo_map>")[0] for t in texts]
        self.assertEqual(maps[0], maps[1])
        self.assertTrue(self.media.path(maps[0]).is_file())
        gltf, binary = read_glb(self.models / "urc_far_b" / "meshes" / "farfield.glb")
        V = accessor(gltf, binary, gltf["meshes"][0]["primitives"][0]["attributes"]["POSITION"]).astype(float)
        near = V[np.maximum(np.abs(V[:, 0]), np.abs(V[:, 1])) < 256.0 + farfield.SPACING]
        self.assertGreater(len(near), 0)
        for x, y, z in near:
            window = (np.abs(hf.grid()[0] - np.clip(x, -256, 256)) <= farfield.SPACING) & \
                     (np.abs(hf.grid()[1] - np.clip(y, -256, 256)) <= farfield.SPACING)
            self.assertLessEqual(z, hf.z[window].min() - farfield.SINK + 0.01)


class MediaPatch(unittest.TestCase):
    """sim/tools/gz_media.py on a copy of the environment's stock media."""

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = Path(tmp.name)
        self.source = self.tmp / "stock"
        import shutil
        shutil.copytree(gz_media.environment_media(), self.source)
        self.build = self.tmp / "build"

    def run_tool(self):
        return subprocess.run([sys.executable, str(SIM_DIR / "tools" / "gz_media.py"), "--build-dir", str(self.build),
                               "--source", str(self.source)], capture_output=True, text=True, timeout=120)

    def digest(self, root):
        return {p.relative_to(root): hashlib.md5(p.read_bytes()).hexdigest() for p in sorted(root.rglob("*"))
                if p.is_file()}

    def test_patches_a_copy(self):
        before = self.digest(self.source)
        result = self.run_tool()
        self.assertEqual(result.returncode, 0, result.stderr)
        media = self.build / gzenv.MEDIA_DIR
        self.assertTrue((media / gzenv.MEDIA_COMPLETE).is_file())
        self.assertEqual(self.digest(self.source), before)
        terra = (media / "ogre2/media/Hlms/Terra/Any/800.PixelShader_piece_ps.any").read_text()
        self.assertIn("float roughness@n = 1.0f;", terra)
        sky = (media / "ogre2/media/materials/programs/Metal/skybox_fs.metal").read_text()
        sun = ", ".join(f"{v:.4f}" for v in lighting.MISSION.toward)
        self.assertIn(f"float3( {sun} )", sky)
        haze = (media / "ogre2/media/Hlms/Gz/Pbs/900.RoverHaze_piece_ps.any").read_text()
        self.assertIn(f"{lighting.SKY.haze_beta:.3e}", haze)
        for path in media.rglob("*"):
            if path.is_file() and path.suffix in (".any", ".metal", ".glsl"):
                self.assertNotIn("${", path.read_text(errors="replace"), path)
        env = gzenv.environment(self.build, base={})
        self.assertEqual(env["GZ_RENDERING_RESOURCE_PATH"], str(media.resolve()))

    def test_soft_failure(self):
        """A diff that does not apply (an upgraded environment): a warning,
        exit 0, no media directory (also none left from an earlier run)."""
        self.assertEqual(self.run_tool().returncode, 0)
        self.assertTrue((self.build / gzenv.MEDIA_DIR / gzenv.MEDIA_COMPLETE).is_file())
        terra = self.source / "ogre2/media/Hlms/Terra/Any/800.PixelShader_piece_ps.any"
        terra.write_text(terra.read_text().replace("float roughness@n = 0;", "float roughness@n = 0.0;"))
        result = self.run_tool()
        self.assertEqual(result.returncode, 0)
        self.assertIn("WARNING", result.stderr)
        self.assertFalse((self.build / gzenv.MEDIA_DIR).exists())
        self.assertEqual(list(self.build.iterdir()), [])
        self.assertNotIn("GZ_RENDERING_RESOURCE_PATH", gzenv.environment(self.build, base={}))

    def test_strict_hunks(self):
        root = self.tmp / "tree"
        root.mkdir()
        (root / "a.txt").write_text("one\ntwo\nthree\n")
        diff = "--- x/a.txt\n+++ y/a.txt\n@@ -1,3 +1,3 @@\n one\n-two\n+TWO\n three\n" \
               "--- x/b.txt\n+++ y/b.txt\n@@ -0,0 +1,1 @@\n+new\n"
        gz_media.apply(diff, root)
        self.assertEqual((root / "a.txt").read_text(), "one\nTWO\nthree\n")
        self.assertEqual((root / "b.txt").read_text(), "new\n")
        with self.assertRaises(gz_media.PatchError):
            gz_media.apply(diff.replace("-two", "-zwei"), root)


if __name__ == "__main__":
    unittest.main()
