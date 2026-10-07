#!/usr/bin/env python3
"""Friction zones, ground types, rocks and the proving ground (pixi run sim-test).

The geometry tests need no Gazebo; the physics tests run proving-ground
copies headless (Sensors stripped, the rover respawned).
"""
import json
import math
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image

from simulate import simulate
from worldfiles import WORLDS, gz_check, model_root, rock_vertices, sheet, vec, world_copy
from worldfiles import terrain as world_terrain

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import features, geo, meshes, sdf, terrain, terrains  # noqa: E402
from urc.media import Media  # noqa: E402
from urc.missions import COURSES, MISSIONS, delivery, proving_ground  # noqa: E402
from urc.world import ROCK_BURY, Layer, WorldBuilder  # noqa: E402

WORLDS_WITH_ZONES = ("urc_delivery", "urc_astrobiology", "urc_equipment_servicing", "urc_autonomy", "proving_ground")
CELL_REACH = terrains.TILE / math.sqrt(2)  # a used cell's centre is inside: its corners reach this far out
QUANTUM = 0.003  # [m] a generated world's 16-bit heightmap rounds heights by up to z_max / 131070 (< 1.5 mm)


def bumpy_slope(size=64.0, n=257, grade=20.0):
    """A 20 deg slope rising to the east, with bumps."""
    hf = terrain.Heightfield(size, n)
    X, _ = hf.grid()
    hf.z = math.tan(math.radians(grade)) * X
    return hf.noise(0.06, 3.0, seed=5, octaves=2)


def distance_to_polygon(polygon, x, y):
    """Distance from points to a polygon's edges."""
    return terrain.path_distance(polygon, x, y, closed=True)[0]


def sdf_tiles(world):
    """The friction-zone tiles of a generated world, read from its terrain
    model's zone_* boxes, in world coordinates."""
    name = re.search(r"<uri>model://(urc_terrain_\w+)</uri>", (WORLDS / f"{world}.sdf").read_text()).group(1)
    tiles = []
    for c in model_root(name).iter("collision"):
        if c.get("name").startswith("zone_"):
            x, y, z, roll, pitch, yaw = vec(c.findtext("pose"))
            size = tuple(vec(c.findtext("geometry/box/size")))
            axes = np.array(sdf.rpy_to_matrix(roll, pitch, yaw))
            n = axes[:, 2]
            tiles.append(terrains.Tile(None, np.array([x, y, z]) + size[2] / 2 * n,
                                       np.array([-n[0], -n[1]]) / n[2], axes, size))
    return tiles


def tile_samples(tile, n=9):
    """Layout (x, y) of a grid over a tile's footprint."""
    a, b, _, d = tile.footprint
    s, t = (g.ravel() for g in np.meshgrid(np.linspace(0, 1, n), np.linspace(0, 1, n)))
    p = a + s[:, None] * (b - a) + t[:, None] * (d - a)
    return p[:, 0], p[:, 1]


class Tiles(unittest.TestCase):
    """Tile geometry on a bumpy 20 deg slope: a round zone on ground levelled
    as features.Patch levels it, and a lane on a graded strip with a kink."""

    @classmethod
    def setUpClass(cls):
        cls.hf = bumpy_slope()
        features.Patch("sand", terrains.SAND, 2.0, -3.0, 12.0).shape(cls.hf)
        lane = features.Lane("lane", (-5.0 - 12 * math.cos(0.3), 22.0 - 12 * math.sin(0.3)), 0.3, 5.0,
                             ((7.0, 10.0), (17.0, 0.0)))
        lane.shape(cls.hf)
        cls.blob = terrains.blob("sand", terrains.SAND, 2.0, -3.0, 12.0, seed=3)
        cls.rect = terrains.rect("lane", terrains.CLAY, -5.0, 22.0, 24.0, 5.0, yaw=0.3, breaks=(7.0,))
        for zone in (cls.blob, cls.rect):
            terrains.fit_tiles(cls.hf, zone)

    def random_points(self, zone, count=4000):
        rng = np.random.default_rng(1)
        lo, hi = zone.outline.min(axis=0), zone.outline.max(axis=0)
        x, y = rng.uniform(lo[0], hi[0], count), rng.uniform(lo[1], hi[1], count)
        keep = terrains.inside(zone.outline, x, y)
        return x[keep], y[keep], distance_to_polygon(zone.outline, x[keep], y[keep])

    def test_tiles_cover_the_zone(self):
        for zone, reach in ((self.blob, CELL_REACH), (self.rect, 1e-6)):
            x, y, edge = self.random_points(zone)
            for px, py in zip(x[edge > reach], y[edge > reach]):
                self.assertIsNotNone(terrains.top_height(zone.tiles, px, py), (zone.key, px, py))
            corners = np.concatenate([t.footprint for t in zone.tiles])
            outside = ~terrains.inside(zone.outline, *corners.T)
            self.assertLessEqual(distance_to_polygon(zone.outline, *corners[outside].T).max(initial=0.0),
                                 reach + 1e-6, zone.key)
        self.assertAlmostEqual(self.rect.tiled_area, self.rect.area, delta=1e-6)
        self.assertAlmostEqual(self.blob.tiled_area, self.blob.area, delta=0.1 * self.blob.area)
        self.assertLess(len(self.blob.tiles), 20)  # levelled ground: large tiles, not 200 single cells

    def test_tile_tops_lie_just_above_the_surface(self):
        """Wheels on a tile never reach the heightmap, and no tile floats or
        leaves a kerb at its neighbours: it is TILE_CLEARANCE to
        TILE_CLEARANCE + FLATNESS above the surface everywhere."""
        for zone in (self.blob, self.rect):
            for tile in zone.tiles:
                x, y = tile_samples(tile)
                top = tile.top[2] + tile.gradient[0] * (x - tile.top[0]) + tile.gradient[1] * (y - tile.top[1])
                gap = top - self.hf.height(x, y)
                # The fit samples the ground at half the grid spacing: along a tile edge that is not
                # parallel to the grid the surface is quadratic between samples (< 1 mm off).
                self.assertGreaterEqual(gap.min(), terrains.TILE_CLEARANCE - 0.001, zone.key)
                self.assertLessEqual(gap.max(), terrains.TILE_CLEARANCE + terrains.FLATNESS + 0.001, zone.key)

    def test_bumpy_ground_is_refused(self):
        with self.assertRaisesRegex(ValueError, "not planar"):
            terrains.fit_tiles(bumpy_slope(), terrains.blob("sand", terrains.SAND, 2.0, -3.0, 12.0, seed=3))

    def test_tops_is_top_height_vectorised(self):
        rng = np.random.default_rng(2)
        x, y = rng.uniform(-20, 20, 300), rng.uniform(-20, 20, 300)
        tiles = self.blob.tiles + self.rect.tiles
        z = terrains.tops(tiles, x, y)
        self.assertTrue(np.isnan(z).any() and not np.isnan(z).all())
        for px, py, pz in zip(x, y, z):
            top = terrains.top_height(tiles, px, py)
            self.assertTrue(math.isnan(pz) if top is None else math.isclose(pz, top), (px, py))

    def test_box_pose_puts_the_top_face_on_the_plane(self):
        """The SDF pose (centre, roll, pitch, yaw) of each tile box: its top
        face's corners lie on the tile's top plane and cover its footprint."""
        for zone in (self.blob, self.rect):
            for tile in zone.tiles:
                R = np.array(sdf.rpy_to_matrix(*sdf.matrix_to_rpy(tile.axes)))
                np.testing.assert_allclose(R, tile.axes, atol=1e-9)
                half = np.array(tile.size) / 2
                for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    corner = tile.center + R @ (half * (sx, sy, 1))
                    plane = tile.top[2] + tile.gradient @ (corner[:2] - tile.top[:2])
                    self.assertAlmostEqual(corner[2], plane, delta=1e-9)
                fx, fy = tile_samples(tile, 5)
                self.assertIsNotNone(terrains.top_height([tile], fx.mean(), fy.mean()))
                for px, py in zip(fx, fy):
                    self.assertIsNotNone(terrains.top_height([tile], px, py))


class Features(unittest.TestCase):
    def test_ledge_drops_by_its_height(self):
        """In front of the edge the ground is `drop` lower than the shelf top,
        behind the shelf level with it, and under the shelf never above it,
        on a fine and on a coarse grid."""
        for n in (257, 65):
            hf = bumpy_slope(grade=8.0, n=n)
            ledge = features.Ledge("ledge", 3.0, -2.0, 2.5, 0.6)
            ledge.shape(hf)
            top = hf.height(*ledge.at(-ledge.depth))
            for u in np.linspace(0.0, features.LEDGE_APRON, 9):
                self.assertAlmostEqual(hf.height(*ledge.at(u)), top - ledge.drop, delta=1e-6, msg=(n, u))
            for u in np.linspace(ledge.depth, ledge.depth + features.LEDGE_BACK, 7):
                self.assertAlmostEqual(hf.height(*ledge.at(-u)), top, delta=1e-6, msg=(n, -u))
            self.assertLessEqual(max(hf.height(*ledge.at(-u)) for u in np.linspace(0, ledge.depth, 41)), top + 1e-9)
        with self.assertRaisesRegex(ValueError, "deep"):
            features.Ledge("ledge", 0.0, 0.0, 0.0, 0.6, depth=2.0).shape(terrain.Heightfield(64, 65))

    def test_wash_sand_stays_on_the_floor(self):
        path = ((-30.0, 0.0), (30.0, 0.0))
        with self.assertRaisesRegex(ValueError, "half-width"):
            features.Wash("wash", path, depth=2.0, half_width=6.0, sand_radius=6.0).shape(bumpy_slope())
        features.Wash("wash", path, depth=0.0, half_width=6.0, sand_radius=6.0).shape(bumpy_slope())  # no channel


class Catalogue(unittest.TestCase):
    def test_types_do_not_repeat(self):
        """One catalogue: no two ground types share a colour, no two friction
        zone types a mu."""
        types = list(terrains.TYPES.values())
        self.assertEqual(len({t.rgb for t in types}), len(types))
        friction = [t.mu for t in types if t.mu < terrains.HEIGHTMAP_MU]
        self.assertEqual(len(set(friction)), len(friction))
        self.assertTrue(all(0 < t.mu <= terrains.HEIGHTMAP_MU for t in types))

    def test_textures_are_shared_and_named_by_their_parameters(self):
        with tempfile.TemporaryDirectory() as d:
            media = Media(d)
            uri = terrains.SAND.texture(media)
            self.assertEqual(uri, terrains.SAND.texture(media))
            self.assertTrue(uri.startswith("model://urc_media/textures/ground_sand_"))
            darker = terrains.TerrainType("sand", "Soft sand", 0.4, (200, 170, 130))
            self.assertNotEqual(uri, darker.texture(media))
            self.assertTrue((Path(d) / "urc_media" / "textures" / uri.split("/")[-1]).exists())


class SmallWorld(unittest.TestCase):
    """A small world built from the shared building blocks: a sand patch, a
    clay lane, a slickrock patch, rocks over the zones and round a mesa's
    cliff, and shrubs."""
    FEATURES = [features.Patch("sand", terrains.SAND, -14.0, -14.0, 8.0),
                features.Lane("clay", (10.0 - 7 * math.cos(0.5), -18.0 - 7 * math.sin(0.5)), 0.5, 5.0,
                              ((14.0, 0.0),), (features.Surface("clay", terrains.CLAY, 5.0),)),
                features.Patch("slab", terrains.SLICKROCK, -12.0, 15.0, 6.0, level=False)]
    LATE = features.Patch("late", terrains.GRAVEL, 14.0, 2.0, 3.0)  # levelled; its zone comes too late
    MESA = (20.0, 20.0)

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        d = Path(cls.tmp.name)
        hf = bumpy_slope(size=64.0, n=129, grade=12.0).mesa(*cls.MESA, 5.0, 3.0, 4.0, seed=2)
        features.shape(hf, cls.FEATURES + [cls.LATE])
        w = WorldBuilder("test", "Test", None, geo.Origin(38.4, -110.79, 1370.0), hf, d / "models", d / "worlds",
                         Media(d / "models"), seed=1, name="test")
        w.terrain([Layer("regolith"), Layer("caprock", start=4.0)])
        features.dress(w, cls.FEATURES)
        w.rover(-14.0, -14.0, 0.0)
        sand = cls.FEATURES[0]
        w.rock_field("over_sand", [(sand.x + dx, sand.y + dy, size, 0.4) for dx, dy in ((3, 0), (0, -4), (-2, 2))
                                   for size in (0.05, 0.3)])  # half colliding
        a = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        w.rock_field("round_the_mesa", [(cls.MESA[0] + r * math.cos(t), cls.MESA[1] + r * math.sin(t), 0.5, 2 * t)
                                        for t in a for r in (6.0, 7.5)])
        w.shrubs([(0.0, 0.0), (2.0, 3.0), (-5.0, 5.0)])
        cls.w = w
        cls.world_path, sheet_path = w.write()
        cls.sheet = json.loads(sheet_path.read_text())
        cls.model = ET.parse(d / "models" / "urc_terrain_test" / "model.sdf").getroot()
        cls.dir = d / "models" / "urc_terrain_test"

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_heightmap_is_the_terrain(self):
        """One heightmap, drawn and collided with alike, its lowest point at
        world z = 0 and its highest at the image's maximum (Gazebo scales by
        it)."""
        uris = {h.findtext("uri") for h in self.model.iter("heightmap")}
        self.assertEqual(uris, {"model://urc_terrain_test/heightmap.png"})
        self.assertEqual(len({h.findtext("size") for h in self.model.iter("heightmap")}), 1)
        image = np.asarray(Image.open(self.dir / "heightmap.png"), float)
        self.assertEqual((image.min(), image.max()), (0, 65535))
        z_max = self.sheet["terrain"]["z_max"]
        self.assertAlmostEqual(float(self.model.find(".//heightmap/size").text.split()[2]), z_max, places=3)
        self.assertEqual(self.w.shift[2], self.w.hf.z.min())
        np.testing.assert_allclose(image / 65535 * z_max, self.w.hf.z - self.w.shift[2], atol=z_max / 65535 + 1e-6)

    def test_sheet_lists_the_zones(self):
        zones = self.sheet["terrain_zones"]
        self.assertEqual(set(zones), {"sand", "clay", "slab"})
        for key, kind in (("sand", terrains.SAND), ("clay", terrains.CLAY), ("slab", terrains.SLICKROCK)):
            self.assertEqual(zones[key]["type"], kind.key)
            self.assertEqual(zones[key]["mu"], kind.mu)
            self.assertGreater(zones[key]["area_m2"], 10)
            self.assertEqual(self.sheet["terrain_types"][kind.key]["mu"], kind.mu)
        self.assertEqual(zones["slab"]["tiles"], 0)  # mu 1.0: only a decal
        self.assertGreater(zones["sand"]["tiles"], 0)
        boxes = [c for c in self.model.iter("collision") if c.get("name").startswith("zone_")]
        self.assertEqual(len(boxes), zones["sand"]["tiles"] + zones["clay"]["tiles"])
        for c in boxes:
            self.assertIn(float(c.findtext("surface/friction/ode/mu")), (terrains.SAND.mu, terrains.CLAY.mu))

    def test_rover_starts_on_the_tiles(self):
        start = self.sheet["rover_start"]
        x, y = start["x"] + self.w.shift[0], start["y"] + self.w.shift[1]
        top = terrains.top_height(self.w.zones[0].tiles, x, y)
        self.assertGreaterEqual(self.w.ground(x, y), self.w.height(x, y) + terrains.TILE_CLEARANCE - 1e-9)
        self.assertEqual(self.w.ground(x, y), top)
        self.assertTrue(top + 0.02 - 1e-6 <= start["z"] + self.w.shift[2] <= top + 0.2)

    def test_zone_under_a_placed_model_is_refused(self):
        self.w.place("probe_model", "probe", self.LATE.x, self.LATE.y)
        with self.assertRaisesRegex(ValueError, "placed before"):
            self.w.zone_rect("late", self.LATE.kind, self.LATE.x, self.LATE.y, 4.0, 4.0)

    def test_zone_over_earlier_rocks_is_refused(self):
        """rock_field keeps colliding rocks off the zones declared so far; a
        zone declared after them must not end up under one."""
        x, y = self.LATE.x + 1.5, self.LATE.y  # on the levelled ground, clear of the probe model
        self.w.rock_field("before_late", [(x, y, 0.3, 0.0)])
        with self.assertRaisesRegex(ValueError, "colliding rocks"):
            self.w.zone_rect("late_rect", self.LATE.kind, x, y, 1.5, 1.5)

    def test_colliding_rocks_stay_off_the_tiles(self):
        """A rock's mesh has mu 1: on a friction zone it would be a foothold."""
        self.assertEqual(self.sheet["rocks"]["over_sand"], {"count": 3, "colliding": 0})
        tiles = [t for zone in self.w.zones for t in zone.tiles]
        shift = np.array(self.w.shift)
        for xyz, size, variant, R, _, collides in self.w._rocks:
            if collides:
                V = meshes.rock_variant(variant)[0] * size @ R.T + xyz + shift
                self.assertTrue(np.isnan(terrains.tops(tiles, V[:, 0], V[:, 1])).all())

    def test_rocks_rest_in_the_ground(self):
        """Every rock's flat base is laid on the slope under it and sunk
        ROCK_BURY of its size: no edge floats, even round the mesa's cliff,
        and none is swallowed."""
        shift = np.array(self.w.shift)
        tilts = []
        for xyz, size, variant, R, _, _ in self.w._rocks:
            B = meshes.rock_base(variant) * size @ R.T + xyz + shift
            depth = self.w.height(B[:, 0], B[:, 1]) - B[:, 2]
            self.assertAlmostEqual(depth.min(), ROCK_BURY * size, delta=1e-9)
            self.assertLess(depth.max(), ROCK_BURY * size + 0.25 * size)
            tilts.append(math.degrees(math.acos(R[2, 2])))
        self.assertGreater(max(tilts), 25.0)  # round the cliff

    def test_shrubs_are_merged_meshes(self):
        self.assertNotIn("urc_shrub", self.world_path.read_text())
        shrubs = [v for v in self.model.iter("visual") if v.get("name").startswith("shrubs_")]
        self.assertTrue(shrubs)
        self.assertEqual(self.sheet["shrubs"], 3)


class GeneratedWorlds(unittest.TestCase):
    def test_sheets_list_zones_with_mu(self):
        for world in WORLDS_WITH_ZONES:
            with self.subTest(world=world):
                s = sheet(world)
                zones = s["terrain_zones"]
                self.assertTrue(zones)
                for key, zone in zones.items():
                    kind = s["terrain_types"][zone["type"]]
                    self.assertEqual(zone["mu"], kind["mu"], key)
                    self.assertEqual(zone["tiles"] > 0, zone["mu"] < terrains.HEIGHTMAP_MU, key)
                    if zone["type"] in terrains.TYPES:
                        self.assertEqual(zone["mu"], terrains.TYPES[zone["type"]].mu, key)
                    self.assertGreater(zone["area_m2"], 0, key)
                    self.assertGreaterEqual(len(zone["outline"]), 4, key)
                self.assertTrue(s["rocks"])

    def test_delivery_has_every_ground_of_rule_1_c_ii(self):
        # "soft sandy areas, gravel, rough stony areas, rock and boulder fields, vertical drops and steep
        # loosely consolidated slopes"
        s = sheet("urc_delivery")
        types = {z["type"] for z in s["terrain_zones"].values()}
        self.assertTrue({"sand", "gravel", "scree"} <= types)
        self.assertTrue({"stony_0", "boulder_field"} <= set(s["rocks"]))

    def test_delivery_ledges_drop(self):
        """Each ledge's shelf: in front of its face the ground is `drop` lower
        than its top, behind it level with it, under it never above it."""
        hf = world_terrain("urc_delivery")
        shapes = {c.get("name"): c for c in model_root("urc_terrain_delivery").iter("collision")}
        for k, ledge in enumerate(delivery.LEDGES):
            shelf = shapes[f"ledge_{k}_shelf_collision"]
            x, y, z, _, _, yaw = vec(shelf.findtext("pose"))
            depth, _, height = vec(shelf.findtext("geometry/box/size"))
            top = z + height / 2

            def ground(u):  # u metres from the shelf's front face along yaw
                d = u + depth / 2
                return hf.height(x + d * math.cos(yaw), y + d * math.sin(yaw))

            for u in (0.05, 1.0, 2.0, 3.0):
                self.assertAlmostEqual(top - ground(u), ledge.drop, delta=QUANTUM, msg=(ledge.key, u))
            for u in (0.05, 1.0, 2.0):
                self.assertAlmostEqual(ground(-depth - u), top, delta=QUANTUM, msg=(ledge.key, -u))
            self.assertLess(max(ground(-u) for u in np.linspace(0, depth, 41)), top + QUANTUM, ledge.key)

    def test_tiles_lie_on_the_ground(self):
        """Every friction-zone tile in a generated world is TILE_CLEARANCE to
        TILE_CLEARANCE + FLATNESS above the terrain under it: wheels on it
        never touch the heightmap, and it neither floats nor leaves kerbs."""
        for world in WORLDS_WITH_ZONES:
            with self.subTest(world=world):
                hf = world_terrain(world)
                for tile in sdf_tiles(world):
                    hu, hv = np.array(tile.size[:2]) / 2 - terrains.TILE_OVERLAP
                    U, V = (g.ravel() for g in np.meshgrid(np.linspace(-hu, hu, 7), np.linspace(-hv, hv, 7)))
                    P = tile.top + np.outer(U, tile.axes[:, 0]) + np.outer(V, tile.axes[:, 1])
                    gap = P[:, 2] - hf.height(P[:, 0], P[:, 1])
                    self.assertGreater(gap.min(), terrains.TILE_CLEARANCE - QUANTUM)
                    self.assertLess(gap.max(), terrains.TILE_CLEARANCE + terrains.FLATNESS + QUANTUM)

    def test_colliding_rocks_stay_off_the_tiles(self):
        for world in WORLDS_WITH_ZONES:
            with self.subTest(world=world):
                rocks = rock_vertices(world)
                self.assertTrue(np.isnan(terrains.tops(sdf_tiles(world), rocks[:, 0], rocks[:, 1])).all())

    def test_shrubs_are_merged_into_the_terrain(self):
        """Shrubs are meshes of the terrain's link, not a model each."""
        for world in WORLDS_WITH_ZONES:
            with self.subTest(world=world):
                self.assertNotIn("model://urc_shrub", (WORLDS / f"{world}.sdf").read_text())
                if sheet(world).get("shrubs"):
                    visuals = model_root(f"urc_terrain_{world.removeprefix('urc_')}").iter("visual")
                    self.assertTrue(any(v.get("name").startswith("shrubs_") for v in visuals))

    def test_equipment_approach_is_clear(self):
        """No colliding rock within 2 m of the straight drive from the gate to the lander."""
        s = sheet("urc_equipment_servicing")
        ends = np.array([[s["points"][k][c] for c in ("x", "y")] for k in ("start_gate", "lander")])
        rocks = rock_vertices("urc_equipment_servicing")
        self.assertGreater(terrain.path_distance(ends, rocks[:, 0], rocks[:, 1])[0].min(), 2.0)

    def test_objects_are_clear_of_rocks(self):
        """No colliding rock within 1 m of an object, the rover's start or a URC point."""
        for world in WORLDS_WITH_ZONES:
            with self.subTest(world=world):
                s = sheet(world)
                rocks = rock_vertices(world)
                places = {**s["objects"], "rover_start": s["rover_start"]}
                if world != "proving_ground":  # course points mark rock gardens on purpose
                    places.update({k: p for k, p in s["points"].items() if "radius_m" not in p})
                for name, p in places.items():
                    near = np.hypot(rocks[:, 0] - p["x"], rocks[:, 1] - p["y"]) < 1.0
                    self.assertFalse(near.any(), f"{world}: rock at {name}")


class ProvingGround(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sheet = sheet("proving_ground")

    def test_a_course_not_a_mission(self):
        self.assertIn("proving_ground", COURSES)
        self.assertNotIn("proving_ground", MISSIONS)
        self.assertNotIn("rules", self.sheet)

    def test_gz_accepts_the_world(self):
        result = gz_check(WORLDS / "proving_ground.sdf")
        self.assertIn("Valid", result.stdout, result.stdout + result.stderr)

    def test_every_station_has_a_sign_and_a_description(self):
        text = (WORLDS / "proving_ground.sdf").read_text()
        for key, point in self.sheet["points"].items():
            self.assertTrue(point["description"], key)
            self.assertIn(f"<name>sign_{key}</name>", text)
        expected = {"lane_mu020", "lane_mu035", "lane_mu050", "lane_mu070", "lane_mu095", "sand_pit", "clay_patch",
                    "slickrock_slab", "side_slope_10", "side_slope_20", "garden_10cm", "garden_20cm", "garden_30cm",
                    "garden_40cm", "articulation", "washboard", "step_10cm", "step_20cm", "step_30cm",
                    "boulder_field"}
        self.assertEqual(set(self.sheet["points"]), expected)

    def test_friction_lanes(self):
        mus = [self.sheet["terrain_zones"][f"lane_mu{m:03d}"]["mu"] for m in (20, 35, 50, 70, 95)]
        self.assertEqual(mus, [0.2, 0.35, 0.5, 0.7, 0.95])
        hf = proving_ground.make_terrain()
        lane = proving_ground.FRICTION_HILL
        end = 0.0
        for length, grade in proving_ground.FRICTION_PROFILE:
            end += length
            if grade:  # the ramp's middle third has its grade
                a, b = lane.at(end - 2 * length / 3), lane.at(end - length / 3)
                rise = hf.height(*b) - hf.height(*a)
                self.assertAlmostEqual(math.degrees(math.atan2(rise, math.dist(a, b))), grade, delta=0.5)

    def test_rock_gardens_are_graded(self):
        rocks = self.sheet["rocks"]
        for size in (10, 20, 30, 40):
            self.assertEqual(rocks[f"garden_{size}cm"]["colliding"], rocks[f"garden_{size}cm"]["count"])
        self.assertGreater(rocks["garden_10cm"]["count"], rocks["garden_40cm"]["count"])


class ProvingGroundPhysics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sheet = sheet("proving_ground")

    def run_on(self, x, y, yaw, pitch, seconds, cmd=(0.0, 0.0), lift=0.1):
        with world_copy("proving_ground", rover=(x, y, yaw), pitch=pitch, lift=lift) as world:
            return simulate(seconds, world=world, cmd=cmd).poses["base_link"]

    def parked_on_20_deg_ramp(self, lane):
        """Displacement [m] along the lane's uphill heading of a rover parked
        (wheels held) facing up the middle of the lane's 20 deg ramp."""
        p = self.sheet["points"][lane]
        heading = math.radians(p["heading_deg"])
        u = 8.0 + 8.0 + 4.0 + 3.5  # flat, 10 deg ramp, landing, half the 20 deg ramp
        x, y = p["x"] + u * math.cos(heading), p["y"] + u * math.sin(heading)
        pose = self.run_on(x, y, heading, -math.radians(20.0), 3.0)
        return (pose[0] - x) * math.cos(heading) + (pose[1] - y) * math.sin(heading)

    def test_parked_rover_slides_down_the_mu020_ramp(self):
        self.assertLess(self.parked_on_20_deg_ramp("lane_mu020"), -1.0)

    def test_parked_rover_holds_on_the_mu095_ramp(self):
        self.assertLess(abs(self.parked_on_20_deg_ramp("lane_mu095")), 0.15)

    def test_rover_crosses_the_20cm_rock_garden(self):
        p = self.sheet["points"]["garden_20cm"]
        x0 = p["x"] - 2.5
        pose = self.run_on(x0, p["y"], 0.0, 0.0, 36.0, cmd=(0.5, 0.0), lift=0.05)
        self.assertGreater(pose[0], p["x"] + p["length_m"] + 0.5, pose)  # out the far side
        self.assertLess(max(abs(pose[3]), abs(pose[4])), 0.8, pose)  # not flipped

    def test_twist_ditch_turns_the_rockers_to_their_limit(self):
        ditch = self.sheet["points"]["articulation"]["twist_ditch"]
        with world_copy("proving_ground", rover=(ditch["x"] - 3.0, ditch["y"], 0.0), lift=0.05) as world:
            state = simulate(20.0, world=world, cmd=(0.4, 0.0))
        pose = state.poses["base_link"]
        self.assertGreater(state.rocker_peak, 0.9 * gen_model.Params().rocker_limit)
        self.assertGreater(pose[0], ditch["x"] + 2.0, pose)  # and drives on across
        self.assertLess(max(abs(pose[3]), abs(pose[4])), 0.3, pose)


if __name__ == "__main__":
    unittest.main()
