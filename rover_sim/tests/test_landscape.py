#!/usr/bin/env python3
"""Ground building blocks (sim/urc/landscape.py, design spec 5.1-5.5): the
ground raster and its paint rules, the soil map, relief swatches and their
blend, haystacks and rills, keep-flat masks, clutter recipes and the
roughness targets (no physics; pixi run sim-test)."""
import json
import math
import sys
import unittest
from dataclasses import fields

import numpy as np

from worldfiles import SIM_DIR

sys.path.insert(0, str(SIM_DIR / "tools"))
import make_relief_swatches  # noqa: E402
from urc import dem, features, geo, landscape, terrain, terrains  # noqa: E402

TARGETS = SIM_DIR / "data" / "research" / "terrain_targets.json"
ROUTE_LIDAR = SIM_DIR / "data" / "dem" / "route_area_lidar_0p5m.tif"
ROUTE_3DEP = SIM_DIR / "data" / "dem" / "route_area_3dep.tif"


def flat(size=128.0, n=257, center=(0.0, 0.0)):
    return terrain.Heightfield(size, n, center=center)


def plane(gx, gy, size=128.0, n=257):
    hf = flat(size, n)
    X, Y = hf.grid()
    hf.z = gx * X + gy * Y
    return hf


def keys(raster, legend):
    return np.vectorize(lambda i: legend[i].key)(raster)


class Painting(unittest.TestCase):
    def setUp(self):
        self.legend = landscape.Legend()

    def test_legend(self):
        """The catalogue in order, then each other type once; one key, one type."""
        self.assertEqual([t.key for t in self.legend.types], list(terrains.TYPES))
        self.assertEqual(self.legend.index("regolith"), list(terrains.TYPES).index("regolith"))
        lane = terrains.calibration_surface(0.35)
        i = self.legend.index(lane)
        self.assertEqual((i, self.legend.index(terrains.calibration_surface(0.35))), (len(terrains.TYPES), i))
        with self.assertRaisesRegex(ValueError, "two different"):
            self.legend.index(terrains.TerrainType("sand", "Other sand", 0.4, (1, 2, 3)))

    def test_raster_is_on_the_heightmap_grid(self):
        """One index per heightmap sample, row 0 north, column 0 west."""
        hf = flat(64.0, 129, center=(10.0, -5.0))
        raster = landscape.paint(hf, [landscape.Base("regolith"),
                                      landscape.Along(((-22.0, 27.0), (42.0, 27.0)), "sand", 0.3)], legend=self.legend)
        self.assertEqual((raster.shape, raster.dtype), ((129, 129), np.uint8))
        sand = self.legend.index("sand")
        self.assertTrue((raster[0] == sand).all())  # y = -5 + 32: the north edge
        self.assertFalse((raster[1:] == sand).any())
        self.assertEqual(landscape.paint(hf, [], legend=self.legend)[5, 5], self.legend.index(terrains.DEFAULT_GROUND))

    def test_rules_paint_in_order(self):
        hf = flat(256.0, 513)
        hf.mesa(40.0, 40.0, 20.0, 12.0, 15.0, seed=3)
        hill = features.Mesa("hill", 40.0, 40.0, 20.0, 12.0, 15.0, seed=3)
        rules = [landscape.Base("regolith"), landscape.Noise("sand_sheet", 30.0, 0.4, seed=1),
                 landscape.Along(((-120.0, -60.0), (120.0, -60.0)), "wash_sand", 4.0),
                 landscape.Hills((hill,), slope="badland_slope", floor="silt_flat", cap="caprock"),
                 landscape.Band("gypsum", 11.9, over=("caprock",)), landscape.Steeper(30.0, "rock")]
        k = keys(landscape.paint(hf, rules, legend=self.legend), self.legend)
        X, Y = hf.grid()
        r = np.hypot(X - 40, Y - 40)
        far = r > 80
        self.assertAlmostEqual(np.isin(k[far & (np.abs(Y + 60) > 6)], ["sand_sheet"]).mean(), 0.4, delta=0.05)
        self.assertTrue((k[(np.abs(Y + 60) < 3.5) & (np.abs(X) < 118)] == "wash_sand").all())
        self.assertTrue(np.isin(k[r < 14], ["caprock", "gypsum"]).all())  # the top, its highest part gypsum
        self.assertTrue((k[(r < 14) & (hf.z >= 11.95)] == "gypsum").all())
        cliff = (r > 27) & (r < 33)  # inside the cliff band all round
        self.assertTrue(np.isin(k[cliff], ["badland_slope", "rock"]).all())
        self.assertTrue((k[terrain.slope_map(hf.z, hf.res, landscape.SLOPE_SMOOTH) > 30] == "rock").all())
        self.assertTrue((k[(r > 40) & (r < 52)] == "silt_flat").all())  # its floor: 20 m beyond the cliff

    def test_below_paints_talus_under_rims(self):
        hf = flat(256.0, 513)
        hf.mesa(0.0, 0.0, 30.0, 10.0, 30.0, seed=4, irregularity=0.0)  # rims at r = 30, ~18 deg flanks
        top = features.Mesa("mesa", 0.0, 0.0, 30.0, 10.0, 30.0, seed=4, irregularity=0.0)
        k = keys(landscape.paint(hf, [landscape.Hills((top,), slope="badland_slope", cap="caprock"),
                                      landscape.Below("caprock", "block_field", reach_m=20.0)], legend=self.legend),
                 self.legend)
        r = np.hypot(*hf.grid())
        self.assertTrue((k[r < 29] == "caprock").all())
        self.assertTrue((k[(r > 37) & (r < 48)] == "block_field").all())  # within 20 m and over 1 m lower
        self.assertTrue((k[r > 52] != "block_field").all())

    def test_zones_paint_their_outlines(self):
        hf = flat(64.0, 257)
        zones = [terrains.blob("sand", terrains.SAND, -10.0, 5.0, 9.0, seed=2),
                 terrains.rect("lane", terrains.calibration_surface(0.2), 12.0, -8.0, 20.0, 5.0, yaw=0.4)]
        raster = landscape.paint(hf, [landscape.Base("regolith")], zones, legend=self.legend)
        X, Y = hf.grid()
        for zone in zones:
            inside = terrains.inside(zone.outline, X, Y)
            clear = terrain.path_distance(zone.outline, X, Y, closed=True)[0] > hf.res
            painted = raster == self.legend.index(zone.kind)
            np.testing.assert_array_equal(painted[clear], inside[clear], zone.key)

    def test_features_paint_where_their_zones_will(self):
        """Before a world dresses them, features paint their zones' footprints
        (for relief): a lane surface exactly, a patch as its disc."""
        hf = flat(64.0, 257)
        lane = features.Lane("lane", (-20.0, -10.0), 0.3, 5.0, ((20.0, 0.0),),
                             (features.Surface("clay", terrains.CLAY, 5.0),))
        patch = features.Patch("sand", terrains.SAND, 10.0, 12.0, 8.0)
        early = landscape.paint(hf, [], [lane, patch], legend=self.legend)
        zones = [terrains.rect("clay", terrains.CLAY, *lane.at(10.0), 20.0, 5.0, yaw=0.3),
                 terrains.blob("sand", terrains.SAND, 10.0, 12.0, 8.0, seed=3)]
        dressed = landscape.paint(hf, [], zones, legend=self.legend)
        clay, sand = self.legend.index("clay"), self.legend.index("sand")
        np.testing.assert_array_equal(early == clay, dressed == clay)
        a, b = early == sand, dressed == sand
        self.assertGreater((a & b).sum() / (a | b).sum(), 0.75)

    def test_ground_json(self):
        """ground.json (design 9.1): the legend with every traction field,
        dust, the collision map, prefixes and defaults; the dig-in preset."""
        lane = terrains.calibration_surface(0.5)
        self.legend.index(lane)
        collisions = {"zone_lane_0_collision": lane.key, "step_10cm_collision": "rock"}
        info = landscape.ground_json(self.legend, 256.0, 1025, "regolith", collisions)
        self.assertEqual(info["format"], "rover-ground/2")
        self.assertEqual((info["size_m"], info["samples"]), (256.0, 1025))
        self.assertEqual(info["types"][info["default"]]["key"], "regolith")
        self.assertEqual([t["index"] for t in info["types"]], list(range(len(self.legend))))
        names = {"index", "key", "title", "dust", "dust_rgb"} | {f.name for f in fields(terrains.Traction)}
        for t in info["types"]:
            self.assertEqual(set(t), names)
        sand = next(t for t in info["types"] if t["key"] == "sand")
        self.assertEqual((sand["dig_rate"], sand["dig_max"]), terrains.STRONG_DIG)
        mild = landscape.ground_json(self.legend, 256.0, 1025, "regolith", {}, dig="mild")
        self.assertEqual(next(t for t in mild["types"] if t["key"] == "sand")["dig_max"], terrains.MILD_DIG["sand"][1])
        self.assertEqual(info["prefixes"], {"rocks_": "rock", "slabs_": "rock", "risers_": "rock"})
        self.assertEqual((info["terrain_default"], info["object_default"]), ("rock", "manmade"))
        self.assertEqual(json.loads(json.dumps(info)), info)
        with self.assertRaisesRegex(ValueError, "not in the legend"):
            landscape.ground_json(landscape.Legend(), 256.0, 1025, "regolith", {"x_collision": "test_mu010"})


@unittest.skipUnless(ROUTE_3DEP.is_file(), "the route-area DEM is not linked (sim/tools/link_data.sh)")
class Soils(unittest.TestCase):
    def test_autonomy_square_by_soil_unit_and_slope(self):
        """The Autonomy square painted by the soil map (design 5.3, measured
        on the 0.5 m lidar: sand sheet 49.1 %, clay crust 21.2 %, silt flat
        15.0 %, badland slope 11.9 %, rock 2.8 %); here on the 1 m 3DEP,
        within 3 points."""
        from urc.missions import autonomy
        d = dem.read_geotiff(ROUTE_3DEP)
        centre = geo.Origin(*autonomy.SQUARE_MILE_CENTER, d.height(*autonomy.SQUARE_MILE_CENTER))
        lat, lon, _ = geo.enu_to_wgs84(centre, *autonomy.C2_FROM_CENTER)
        origin = geo.Origin(lat, lon, d.height(lat, lon))
        hf = dem.to_heightfield(d, origin, autonomy.SIZE, autonomy.SAMPLES, autonomy.CENTER)
        legend = landscape.Legend()
        k = keys(landscape.from_ssurgo(hf, origin, legend=legend), legend)
        share = lambda *names: np.isin(k, names).mean() * 100  # noqa: E731
        for names, expected in ((("sand_sheet", "sand"), 49.1), (("clay_crust",), 21.2), (("silt_flat",), 15.0),
                                (("badland_slope",), 11.9), (("rock", "caprock"), 2.8)):
            self.assertAlmostEqual(share(*names), expected, delta=3.0, msg=names)
        self.assertGreater(share("sand"), 0.0)  # dune crests
        self.assertLess(share("regolith"), 0.5)  # every sample lies in a soil polygon

    def test_unit_rules_name_catalogue_types(self):
        for mukey, unit in landscape.SSURGO_UNITS.items():
            for key in (unit.gentle, unit.moderate, unit.steep, unit.crest, unit.hollow, unit.cap):
                self.assertTrue(key is None or key in terrains.TYPES, (mukey, key))
        units = {f["properties"]["mukey"] for f in json.loads(landscape.SOILS_PATH.read_text())["features"]}
        self.assertEqual(units, set(landscape.SSURGO_UNITS))
        self.assertNotIn("fan_pavement", {u.gentle for u in landscape.SSURGO_UNITS.values()})  # Leebench (5.3)


class Swatches(unittest.TestCase):
    def test_files(self):
        """sim/data/relief/<swatch>.npz (design 9.2): float16 metres at 0.5 m,
        the sigma their recipes name, provenance and their RMS table."""
        recipes = {t.relief.swatch: t.relief.swatch_sigma_m for t in terrains.TYPES.values() if t.relief.swatch}
        self.assertEqual(set(recipes), set(make_relief_swatches.SWATCHES))
        for key, sigma in recipes.items():
            with self.subTest(swatch=key):
                path = landscape.RELIEF_DIR / f"{key}.npz"
                self.assertLess(path.stat().st_size, 0.8e6)
                with np.load(path) as f:
                    self.assertEqual(f["z"].dtype, np.float16)
                s = landscape.swatch(key)
                self.assertEqual((s.res_m, s.sigma_m), (0.5, sigma))
                self.assertEqual(len(s.windows), len(make_relief_swatches.SWATCHES[key][1]))
                for window, source in zip(s.windows, s.source):
                    self.assertGreater(min(window.shape), (terrain.SWATCH_BLOCK + terrain.SWATCH_FEATHER) / 0.5)
                    self.assertTrue({"window", "dem", "centre_lat_lon", "size_m"} <= set(source))
                self.assertEqual(s.rms_cm, make_relief_swatches.rms_table(s.windows))

    @unittest.skipUnless(ROUTE_LIDAR.is_file(), "the lidar DEMs are not linked (sim/tools/link_data.sh)")
    def test_windows_lie_where_the_design_says(self):
        """Window centres (design 5.4's table: lat, lon from windows_def, before the lidar's datum shift)."""
        expected = {"sand_sheet_D": (38.40881, -110.76886), "sand_sheet_C": (38.41261, -110.77836),
                    "shale_pediment_A": (38.40796, -110.79770), "badland_banded_B": (38.41336, -110.79310),
                    "boulder_field_I": (38.40604, -110.77066), "boulder_ridge_J": (38.40212, -110.77911),
                    "slickrock_ledges_H": (38.41015, -110.76620)}
        for name, (lat, lon) in expected.items():
            (got_lat, got_lon), _ = make_relief_swatches.window_centre(name, ROUTE_LIDAR)
            shift = json.loads(ROUTE_LIDAR.with_suffix(".json").read_text())["datum_shift_applied"]
            self.assertAlmostEqual(got_lat - shift["dlat_deg"], lat, delta=1e-5, msg=name)
            self.assertAlmostEqual(got_lon - shift["dlon_deg"], lon, delta=1e-5, msg=name)

    def test_blend_keeps_the_rms_across_feathers(self):
        """Overlapping blocks with weights w are summed as sum(w s) / sqrt(sum
        w^2): the field's RMS is the swatch's in the cores and in the feathers
        alike, within 5 % (design 5.4; plain feathering loses up to 29 %)."""
        rng = np.random.default_rng(1)
        windows = []
        for shape in ((400, 300), (300, 520)):
            w = terrain.blur(rng.normal(0, 1, shape), 3.0)
            windows.append((w / w.std()).astype(np.float32))
        size, n, seed = 1024.0, 2049, 7
        field = terrain.swatch_field(windows, 0.5, n, size, seed)
        start = -size / 2 - terrain.SWATCH_FEATHER - np.random.default_rng(seed).uniform(0, terrain.SWATCH_BLOCK)
        coords = np.linspace(-size / 2, size / 2, n)
        phase = (coords - start) % terrain.SWATCH_BLOCK  # within a block's pitch; < FEATHER: an overlap
        seam, core = phase < terrain.SWATCH_FEATHER, (phase > 24.0) & (phase < 56.0)
        both = np.outer(seam, seam)  # four blocks overlap here
        self.assertAlmostEqual(field[np.outer(core, core)].std(), 1.0, delta=0.05)
        self.assertAlmostEqual(field[np.outer(seam, core)].std(), 1.0, delta=0.05)
        self.assertAlmostEqual(field[both].std(), 1.0, delta=0.05)
        self.assertAlmostEqual(field.std(), 1.0, delta=0.05)


class Relief(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.targets = json.loads(TARGETS.read_text())["types"]
        cls.legend = landscape.Legend()

    def roughness(self, key, seed=0, size=512.0, n=1025):
        hf = flat(size, n)
        raster = np.full((n, n), self.legend.index(key), np.uint8)
        dz = landscape.relief(hf, raster, legend=self.legend, seed=seed)
        return dz, {s: float(np.median(landscape.window_rms(dz, hf.res, s))) * 100 for s in (4, 8, 16)}

    def test_each_type_is_as_rough_as_its_real_counterpart(self):
        """On flat ground, each type's median window RMS at 4, 8 and 16 m
        lies within the real p25-p75 of that type (design 5.4, 11), for two
        seeds."""
        for key in ("sand_sheet", "sand", "clay_crust", "silt_flat", "badland_slope", "slickrock", "block_field"):
            target = self.targets[key]
            for seed in (0, 1):
                _, rms = self.roughness(key, seed)
                for scale, value in rms.items():
                    if str(scale) not in target:
                        continue
                    low, _, high, _ = target[str(scale)]
                    with self.subTest(type=key, seed=seed, scale=scale):
                        self.assertTrue(low <= value <= high, f"{value:.2f} cm not in [{low}, {high}]")

    def test_relief_fades_into_ground_without_any(self):
        """No step where relief meets a type without it (rock faces): the
        weights feather it out (TYPE_FEATHER)."""
        hf = flat(256.0, 513)
        raster = np.full((513, 513), self.legend.index("sand_sheet"), np.uint8)
        raster[:, 257:] = self.legend.index("rock")
        dz = landscape.relief(hf, raster, legend=self.legend)
        self.assertGreater(np.abs(dz[:, :200]).max(), 0.05)
        self.assertLess(np.abs(dz[:, 300:]).max(), 0.01)
        step = np.abs(np.diff(dz, axis=1))
        self.assertLessEqual(np.percentile(step[:, 240:275], 99.9), np.percentile(step[:, :200], 99.9))

    def test_keep_flat_spares_engineered_ground(self):
        """Pads, lanes and easy routes stay within 2 cm of the macro shape,
        and a lane keeps its designed grade (design 5.4)."""
        hf = flat(256.0, 513)
        lane = features.Lane("lane", (-60.0, 30.0), 0.0, 5.0, ((10.0, 0.0), (20.0, 15.0), (10.0, 0.0)))
        lane.shape(hf)
        route = ((-100.0, -50.0), (0.0, -30.0), (90.0, -60.0))
        keep = landscape.keep_flat(hf, features=[lane], pads=[(50.0, 50.0, 6.0)],
                                   paths=[(route, landscape.EASY_ROUTE_FLAT)])
        raster = np.full((513, 513), self.legend.index("block_field"), np.uint8)
        dz = landscape.relief(hf, raster, keep, legend=self.legend)
        after = terrain.Heightfield(hf.size, hf.n, hf.z + dz)
        rng = np.random.default_rng(3)
        u, v = rng.uniform(0, lane.length, 300), rng.uniform(-2.5, 2.5, 300)
        points = [lane.at(a, b) for a, b in zip(u, v)]
        points += [(50.0 + r * math.cos(t), 50.0 + r * math.sin(t)) for r, t in zip(rng.uniform(0, 6, 200),
                                                                                    rng.uniform(0, 7, 200))]
        points += [tuple(p) for p in terrain.resample(route, 1.0)]
        x, y = np.array(points).T
        self.assertLess(np.abs(after.height(x, y) - hf.height(x, y)).max(), 0.02)
        self.assertGreater(np.abs(dz).max(), 0.3)  # block-field relief elsewhere
        a, b = lane.at(10.0 + 20.0 / 3), lane.at(10.0 + 40.0 / 3)  # the ramp's middle third
        rise = after.height(*b) - after.height(*a)
        self.assertAlmostEqual(math.degrees(math.atan2(rise, math.dist(a, b))), 15.0, delta=0.5)

    def test_haystacks(self):
        """A badland belt: about half of it flat floor, about 6 % over 30
        deg at 2 m (design 5.4, M: 5.9 %), no flank much past the recipe's
        steepest."""
        recipe = terrains.Haystacks()
        h = terrain.haystack_heights(1025, 512.0, np.ones((1025, 1025)), 3, recipe)
        slope = terrain.slope_map(h[::4, ::4], 2.0)
        self.assertAlmostEqual((slope > 30).mean(), 0.06, delta=0.02)
        self.assertTrue(0.40 < (slope < 5).mean() < 0.65)
        self.assertLess(np.percentile(terrain.slope_map(h, 0.5), 99.9), recipe.flank_deg + 3)
        self.assertLess(h.max(), math.tan(math.radians(recipe.flank_deg)) * recipe.diameter_m[1] / math.pi * 1.6)

    def test_haystacks_stay_inside_their_belt(self):
        """Where the mask falls off (a 16 m strip easing in over 2 m, as the
        proving ground's natural strips; a belt fading over keep-flat's 5 m)
        no knob is cut into a wall: knobs lie where the mask is over 0.5,
        each faded whole by it, so their steepest flank stays within the
        recipe's (cut by the mask, it reached 60-77 deg), and none reaches
        ground the mask keeps flat."""
        recipe = terrains.Haystacks()
        n, size = 513, 128.0
        c = np.linspace(-size / 2, size / 2, n)
        X, Y = np.meshgrid(c, c[::-1])
        for half_width, ease in ((8.0, 2.0), (30.0, 5.0)):
            with self.subTest(width=2 * half_width, ease=ease):
                mask = terrain.smoothstep(0.0, ease, half_width - np.abs(Y))
                h = terrain.Heightfield(size, n).haystacks(mask, 5, recipe).z
                self.assertGreater(h.max(), 1.0)  # knobs there are
                self.assertLess(terrain.slope_map(h, size / (n - 1)).max(), recipe.flank_deg + 3)
                self.assertEqual(np.abs(h[mask < 0.5]).max(), 0.0)

    def test_rills_run_downslope(self):
        """Rill traces descend the slope they are carved into (design 5.4: a
        transferred rill would point the wrong way), 5-20 cm deep."""
        hf = plane(0.25, -0.1, 256.0, 513)  # 15 deg, rising east and a little south
        recipe = terrains.Rills()
        mask = np.ones((513, 513))
        traces = terrain.rill_traces(hf.z, hf.res, mask, 4, recipe)
        self.assertGreater(len(traces), 20)
        downhill = np.array([-0.1, -0.25]) / math.hypot(0.25, 0.1)  # (row, col) of steepest descent: west, north
        for trace in traces:
            heading = trace[-1] - trace[0]
            self.assertGreater(heading @ downhill / np.linalg.norm(heading), math.cos(math.radians(5)))
            self.assertLessEqual(len(trace) - 1, recipe.length_m / terrain.RILL_STEP)
        depth = terrain.rill_depths(hf.z, hf.res, mask, 4, recipe)
        self.assertAlmostEqual(depth.max(), recipe.depth_m[1], delta=0.02)
        carved = depth[depth > 0.005]
        self.assertGreater(carved.min(), 0.0)
        self.assertTrue(0.01 < (depth > 0.005).mean() < 0.3)
        flat_ground = terrain.rill_traces(flat(64.0, 129).z, 0.5, np.ones((129, 129)), 4, recipe)
        self.assertEqual(flat_ground, [])  # nothing runs on flat ground


class Clutter(unittest.TestCase):
    def setUp(self):
        self.legend = landscape.Legend()

    def field(self, key, size=200.0, n=401):
        hf = flat(size, n)
        return hf, np.full((n, n), self.legend.index(key), np.uint8)

    def test_slab_counts_follow_the_table(self):
        recipe = terrains.BLOCK_SLABS
        for d, count in recipe.counts:
            self.assertAlmostEqual(float(landscape.slab_count(recipe, d)), count, places=9)
        self.assertAlmostEqual(float(landscape.slab_count(recipe, 0.5)), 1.74 * 0.5 ** -1.1, places=9)
        self.assertEqual(float(landscape.slab_count(recipe, 7.5)), 0.0)
        d = landscape.slab_sizes(recipe, np.linspace(0, 0.999, 50), 0.3, 7.0)
        self.assertTrue(np.all(np.diff(d) > 0) and 0.3 <= d.min() and d.max() <= 7.0)

    def test_slabs_reproduce_the_measured_block_field(self):
        """N(>=1), N(>=2), N(>=4) per 100 m2 within 30 % of the table, less
        the blocks of 7 m and more, which are ledges and macro shape, not
        slabs (design 5.5); the cover by 1-7 m blocks 8.6 % within 25 %
        (design 11: windows I and J); tilts up to 30 deg, thickness 0.4 D x
        U(0.7, 1.3)."""
        recipe = terrains.BLOCK_SLABS
        hf, raster = self.field("block_field", 400.0, 801)
        slabs = landscape.place(hf, raster, "slabs", np.random.default_rng(5), legend=self.legend)
        d = np.array([s.size for s in slabs])
        area = hf.size ** 2 / 100.0
        for size in (1.0, 2.0, 4.0):
            expected = float(landscape.slab_count(recipe, size) - landscape.slab_count(recipe, recipe.d_max))
            self.assertAlmostEqual((d >= size).sum() / area, expected, delta=0.3 * expected, msg=size)
        cover = (math.pi / 4 * d[(d >= 1.0) & (d <= 7.0)] ** 2).sum() / hf.size ** 2
        self.assertAlmostEqual(cover, 0.086, delta=0.25 * 0.086)
        tilt = np.degrees([s.tilt for s in slabs])
        self.assertTrue(tilt.min() >= 0 and tilt.max() <= 30.0)
        ratio = np.array([s.height / s.size for s in slabs])
        self.assertTrue(ratio.min() >= 0.28 - 1e-9 and ratio.max() <= 0.52 + 1e-9)

    def test_real_dem_slabs_stay_below_a_metre(self):
        hf, raster = self.field("block_field")
        slabs = landscape.place(hf, raster, "slabs", np.random.default_rng(1), legend=self.legend, sizes=(0.15, 1.0))
        d = np.array([s.size for s in slabs])
        self.assertTrue(len(d) and d.min() >= 0.15 and d.max() <= 1.0)

    def test_keep_clear_and_within(self):
        hf, raster = self.field("block_field")
        avoid = [tuple(p) for p in terrain.resample(((-90.0, 0.0), (90.0, 0.0)), 2.0)]
        within = np.zeros(raster.shape, bool)
        within[:, :200] = True  # the west half
        slabs = landscape.place(hf, raster, "slabs", np.random.default_rng(2), avoid=avoid, clearance=4.0,
                                legend=self.legend, within=within)
        x, y = np.array([(s.x, s.y) for s in slabs]).T
        self.assertGreater(len(x), 100)
        self.assertGreater(np.abs(y[np.abs(x) < 88]).min(), 4.0 - hf.res)
        self.assertLess(x.max(), 0.0 + hf.res)

    def test_rocks_shrubs_and_types_without_clutter(self):
        hf, raster = self.field("gravel")
        rng = np.random.default_rng(3)
        rocks = landscape.place(hf, raster, "rocks", rng, legend=self.legend, sizes=(0.1, 0.3))
        recipe = terrains.GRAVEL.clutter.rocks
        mean_area = math.pi / 4 * (0.3 ** 2 - 0.1 ** 2) / (2 * math.log(3.0))
        expected = float(landscape.golombek_cover(recipe.k, 0.1) - landscape.golombek_cover(recipe.k, 0.3))
        expected *= hf.size ** 2 / mean_area
        self.assertAlmostEqual(len(rocks), expected, delta=4 * math.sqrt(expected))
        self.assertTrue(all(0.05 <= r.size <= 0.15 for r in rocks))  # half-axes of 0.1-0.3 m rocks
        hf, raster = self.field("sand_sheet", 400.0, 801)
        shrubs = landscape.place(hf, raster, "shrubs", rng, legend=self.legend)
        self.assertAlmostEqual(len(shrubs), terrains.SAND_SHRUBS.per_ha * 16, delta=4 * math.sqrt(640))
        self.assertTrue(all(0.15 <= s.height <= 0.3 and 0.4 <= s.size <= 1.0 for s in shrubs))
        hf, raster = self.field("rock")
        self.assertEqual(landscape.place(hf, raster, "slabs", rng, legend=self.legend), [])

    def test_risers_follow_contours(self):
        """Risers lie along contours of slickrock, 20-40 m long, 0.1-1 m high
        (design 5.5), and never over ground already a ledge face."""
        hf = plane(0.08, 0.03, 256.0, 513)  # 4.9 deg
        raster = np.full((513, 513), self.legend.index("slickrock"), np.uint8)
        risers = landscape.place(hf, raster, "risers", np.random.default_rng(4), legend=self.legend)
        self.assertGreater(len(risers), 20)
        for riser in risers:
            length = np.linalg.norm(np.diff(riser.path, axis=0), axis=1).sum()
            self.assertLessEqual(length, 40.0 + 1e-6)
            z = hf.height(riser.path[:, 0], riser.path[:, 1])
            self.assertLess(z.max() - z.min(), 0.1)  # along the contour
            self.assertTrue(0.1 <= riser.height <= 1.0 and 1.0 <= riser.depth <= 3.0)
        self.assertGreater(np.median([np.linalg.norm(np.diff(r.path, axis=0), axis=1).sum() for r in risers]), 15.0)
        level = landscape.place(flat(128.0, 257), np.full((257, 257), self.legend.index("slickrock"), np.uint8),
                                "risers", np.random.default_rng(6), legend=self.legend)
        self.assertGreater(len(level), 10)  # level benches too: straight on in a random direction
        for riser in level:
            self.assertGreaterEqual(np.linalg.norm(np.diff(riser.path, axis=0), axis=1).sum(), landscape.RISER_MIN)
        cliff = plane(0.0, 0.0, 128.0, 257)
        cliff.z[:, 128:] = 5.0  # a 5 m face down the middle
        raster = np.full((257, 257), self.legend.index("slickrock"), np.uint8)
        for riser in landscape.place(cliff, raster, "risers", np.random.default_rng(5), legend=self.legend):
            self.assertGreater(np.abs(riser.path[:, 0]).min(), 0.5)


class Targets(unittest.TestCase):
    def test_terrain_targets(self):
        """sim/data/research/terrain_targets.json (design 9.2): p25 <= p50 <=
        p75 <= p90 per type and scale, from the soil map's types and the
        block-field windows; the whole real square as measured for the design
        (5.4: 1.6 / 2.7 / 4.9 / 9.1 cm at 4 m)."""
        t = json.loads(TARGETS.read_text())
        self.assertEqual(t["scales_m"], [4, 8, 16, 32])
        for key in ("sand_sheet", "clay_crust", "silt_flat", "badland_slope", "rock", "slickrock", "block_field"):
            for scale in ("4", "8", "16"):
                values = t["types"][key][scale]
                self.assertEqual(values, sorted(values), (key, scale))
                self.assertGreaterEqual(t["types"][key]["windows"][scale], t["min_windows"])
        np.testing.assert_allclose(t["world"]["whole"]["4"], [1.6, 2.7, 4.9, 9.1], atol=0.06)
        self.assertEqual(set(t["paint_rules"]["units"]), set(landscape.SSURGO_UNITS))
        self.assertTrue(set(t["types"]) <= set(terrains.TYPES))

    def test_window_rms_is_plane_detrended(self):
        hf = plane(0.3, -0.2, 64.0, 129)
        np.testing.assert_allclose(landscape.window_rms(hf.z, hf.res, 4.0), 0.0, atol=1e-7)
        rng = np.random.default_rng(1)
        z = hf.z + rng.normal(0, 0.05, hf.z.shape)
        self.assertAlmostEqual(float(np.median(landscape.window_rms(z, hf.res, 8.0))), 0.05, delta=0.004)
        self.assertEqual(len(landscape.window_rms(z, hf.res, 8.0)), (129 // 16) ** 2)


if __name__ == "__main__":
    unittest.main()
