#!/usr/bin/env python3
"""The generated URC worlds follow the rules (pixi run sim-test; no physics).

Reads the mission sheets and models written by `pixi run sim-worlds`.
"""
import math
import unittest

import cv2
import numpy as np

from worldfiles import WORLDS, gz_check, model_root, rock_vertices, sheet, terrain as world_terrain, vec

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import dem, lander, landscape, props, routes, rules, sdf, terrain, terrains  # noqa: E402
from urc import sheet as sheets  # noqa: E402
from urc.missions import MISSIONS, autonomy, delivery  # noqa: E402


def load(mission):
    """A mission's sheet and its terrain (world coordinates)."""
    return sheet(f"urc_{mission}"), world_terrain(f"urc_{mission}")


def collision_extent(model):
    """Axis-aligned size of all collision shapes (link poses at zero joint angles)."""
    points = []
    for link in model.findall("link"):
        lp = vec(link.findtext("pose", "0 0 0 0 0 0"))
        for col in link.findall("collision"):
            cp = vec(col.findtext("pose", "0 0 0 0 0 0"))
            g = col.find("geometry")
            if g.find("box") is not None:
                half = np.array(vec(g.findtext("box/size"))) / 2
            elif g.find("cylinder") is not None:
                r, l = float(g.findtext("cylinder/radius")), float(g.findtext("cylinder/length"))
                half = np.array([r, r, l / 2])
            elif g.find("sphere") is not None:
                half = np.full(3, float(g.findtext("sphere/radius")))
            else:
                continue
            R = np.array(sdf.rpy_to_matrix(*cp[3:])) @ np.eye(3)
            RL = np.array(sdf.rpy_to_matrix(*lp[3:]))
            for corner in np.array(np.meshgrid(*[[-1, 1]] * 3)).T.reshape(-1, 3) * half:
                points.append(np.array(lp[:3]) + RL @ (np.array(cp[:3]) + R @ corner))
    points = np.array(points)
    return points.max(axis=0) - points.min(axis=0)


def mass(model):
    return sum(float(m.text) for m in model.iter("mass"))


def grade(hf, x, y, base=routes.GRADE_BASE):
    """Steepest slope [deg] at (x, y) over a `base`-metre span: the slope a
    rover feels (routes.GRADE_BASE), not that of the 0.5 m micro-relief."""
    h = base / 2
    gx = (hf.height(x + h, y) - hf.height(x - h, y)) / base
    gy = (hf.height(x, y + h) - hf.height(x, y - h)) / base
    return np.degrees(np.arctan(np.hypot(gx, gy)))


def climb_limits(world, x, y):
    """The steepest slope [deg] the rover climbs at world points (x, y): the
    climb angle of the ground there (the world's ground map, ground.json:
    atan(mu_k - crr), design spec 5.6)."""
    ground = sheets.ground(sheet(world), sheets.path(world))
    climb = {t["key"]: math.degrees(math.atan(max(t["mu_k"] - t["crr"], 0.0))) for t in ground.info["types"]}
    return np.vectorize(lambda px, py: climb[ground(px, py)])(x, y)


class Autonomy(unittest.TestCase):
    """The Autonomy world on the real terrain (the USGS 0.5 m lidar DEM of
    the square mile). The rules are checked on the terrain the world ships
    (the sheet's heightmap, world coordinates): the DEM as it is."""

    ROCK_STOP = 0.25  # [m] the rover crosses 0.2 m rocks, 0.3 m ones stop it (proving ground)
    ROVER_HALF_WIDTH = gen_model.Params().pivot_y + gen_model.Params().wheel_width / 2  # [m] over the wheels

    @classmethod
    def setUpClass(cls):
        cls.sheet, cls.terrain = load("autonomy")
        cls.points = cls.sheet["points"]

    @staticmethod
    def xy(p):
        """World (x, y) of a sheet entry."""
        return p["x"], p["y"]

    def test_route_targets_in_the_square_mile(self):
        (lat0, lon0), (lat1, lon1) = rules.ROUTE_AREA
        for key in ("route_start", "post1", "post2"):
            p = self.points[key]
            self.assertTrue(lat0 <= p["lat"] <= lat1 and lon0 <= p["lon"] <= lon1, key)

    def test_radio_line_of_sight(self):
        # 1.e.xvi: the second target is behind the butte, out of radio line of sight.
        self.assertFalse(self.points["post2"]["radio_los"])
        self.assertTrue(self.points["post1"]["radio_los"])
        self.assertTrue(self.points["route_start"]["radio_los"])

    def test_one_easy_route_up_the_butte(self):
        start, post1 = self.xy(self.points["route_start"]), self.xy(self.points["post1"])
        self.assertGreater(self.points["post1"]["z"] - self.points["route_start"]["z"], 12.0)
        found = routes.easy_route(self.terrain, start, post1, autonomy.EASY_ROUTE_MAX_SLOPE)
        self.assertIsNotNone(found)
        self.assertLessEqual(found[1], 18.0)
        easy = self.sheet["judges_only"]["easy_route"]
        self.assertLessEqual(easy["max_grade_deg"], 18.0)
        self.assertEqual([self.xy(p) for p in (easy["points"][0], easy["points"][-1])], [start, post1])

    def test_only_the_easy_route_climbs_the_butte(self):
        """1.e.xv, by the rover's limits: it climbs ground up to that ground's
        climb angle (packed regolith, the bare ground here: 23 deg, so the
        butte's 26-44 deg faces stop it), a rock standing more than ROCK_STOP
        above the ground stops it, and it passes only a gap wider than
        itself. Over the easy route Post 1 is reachable from the start; with
        the easy route's climb onto the butte closed, from neither the start
        nor any approach bearing outside the rim."""
        res = 0.1
        judges = self.sheet["judges_only"]
        rim = np.array([self.xy(p) for p in judges["rim"]["points"]])
        entry = np.array(self.xy(judges["rim"]["entry"]))
        post1, start = self.xy(self.points["post1"]), self.xy(self.points["route_start"])
        out = [(post1[0] + autonomy.APPROACH_RADII[1] * math.sin(math.radians(b)),
                post1[1] + autonomy.APPROACH_RADII[1] * math.cos(math.radians(b))) for b in autonomy.APPROACH_BEARINGS]
        out = [p for p in out if not terrains.inside(rim, *p)]
        x0, y0 = np.min([*rim, start, *out], axis=0) - 10
        x1, y1 = np.max([*rim, start, *out], axis=0) + 10
        shape = int((y1 - y0) / res), int((x1 - x0) / res)

        def cell(p):
            p = np.asarray(p, float)
            return np.floor(np.stack([(p[..., 0] - x0) / res, (y1 - p[..., 1]) / res], axis=-1)).astype(int)

        blocked = np.zeros(shape, np.uint8)
        V = rock_vertices("urc_autonomy")
        tall = cell(V[V[:, 2] - self.terrain.height(V[:, 0], V[:, 1]) > self.ROCK_STOP][:, :2])
        tall = tall[np.all((tall >= 0) & (tall < shape[::-1]), axis=1)]
        blocked[tall[:, 1], tall[:, 0]] = 1
        reach = int(round(self.ROVER_HALF_WIDTH / res))
        blocked = cv2.dilate(blocked, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * reach + 1,) * 2))
        X, Y = np.meshgrid(np.arange(x0, x1, 1.0), np.arange(y1, y0, -1.0))  # slopes on the heightmap's 1 m grid
        steep = (self.terrain.slope_deg(X, Y) > climb_limits("urc_autonomy", X, Y)).astype(np.uint8)
        blocked |= cv2.resize(steep, shape[::-1], interpolation=cv2.INTER_NEAREST)

        def label(climb_closed):
            free = 1 - blocked
            if climb_closed:  # the easy route near where it climbs onto the butte, rocks or not
                easy = terrain.resample([self.xy(p) for p in judges["easy_route"]["points"]], 0.5)
                for p in easy[np.hypot(*(easy - entry).T) < 2 * autonomy.ROUTE_CLEARANCE]:
                    cv2.circle(free, tuple(map(int, cell(p))), int(round((autonomy.ROUTE_CLEARANCE + 2) / res)), 0, -1)
            labels = cv2.connectedComponents(free, connectivity=8)[1]
            return lambda p: labels[cell(p)[1], cell(p)[0]]

        at = label(False)
        self.assertNotEqual(at(post1), 0)
        self.assertEqual(at(start), at(post1))
        at = label(True)
        self.assertGreaterEqual(len(out), 20)
        for p in [start] + out:
            self.assertNotEqual(at(p), at(post1), p)

    def test_astronaut_assistance(self):
        c2, wait = self.sheet["c2"], self.points["astronaut_wait"]
        self.assertTrue(70.0 <= math.hypot(wait["x"] - c2["x"], wait["y"] - c2["y"]) <= 120.0)
        follow = [self.xy(p) for p in self.sheet["astronaut"]["follow_path"]]
        self.assertTrue(40.0 <= sum(math.dist(a, b) for a, b in zip(follow[:-1], follow[1:])) <= 50.0)
        end, stay_to = self.points["follow_end"], self.points["stay_to"]
        self.assertEqual(follow[-1], self.xy(end))
        self.assertGreater(math.hypot(stay_to["x"] - end["x"], stay_to["y"] - end["y"]), rules.STAY_DISTANCE + 5)
        hammer = self.sheet["objects"]["rock_pick_hammer"]
        self.assertTrue(3.0 <= math.hypot(hammer["x"] - end["x"], hammer["y"] - end["y"]) <= 5.0)
        antenna = tuple(c2["antenna"][k] for k in ("x", "y", "z"))
        for x, y in terrain.resample(follow + [self.xy(stay_to)], 2.0):
            self.assertLessEqual(grade(self.terrain, x, y), 8.0, (x, y))
            self.assertTrue(sheets.radio_los(self.terrain, antenna, x, y, self.terrain.height(x, y)), (x, y))

    def test_ar_posts(self):
        for tag in (rules.AR_START_ID, rules.AR_POST1_ID, rules.AR_POST2_ID):
            model = model_root(f"urc_ar_post_{tag}")
            tags = [v for v in model.iter("visual") if v.get("name").startswith("tag")]
            self.assertEqual(len(tags), rules.AR_FACES)
            for v in tags:
                scale = vec(v.findtext("geometry/mesh/scale"))
                self.assertAlmostEqual(scale[1], rules.AR_FACE)
                z = vec(v.findtext("pose"))[2]
                self.assertTrue(rules.AR_MIN_HEIGHT <= z - rules.AR_FACE / 2 and z + rules.AR_FACE / 2 <= rules.AR_MAX_HEIGHT)
                self.assertIn(f"/aruco_4x4_50_{tag}_", v.findtext("material/pbr/metal/albedo_map"))
        self.assertEqual({self.sheet["objects"][k]["aruco_id"] for k in ("route_start", "post1", "post2")}, {0, 1, 2})
        for key in ("route_start", "post1", "post2"):  # standing on drivable ground
            x, y = self.xy(self.points[key])
            slopes = [self.terrain.slope_deg(x + dx, y + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
            self.assertLess(max(slopes), 15.0, key)

    def test_points_match_the_terrain(self):
        for key, p in self.points.items():
            self.assertAlmostEqual(self.terrain.height(p["x"], p["y"]), p["z"], delta=0.05, msg=key)

    def test_points_match_the_dem(self):
        """Every target's WGS84 position and ellipsoidal altitude is on the
        USGS terrain (NAVD88 elevations, dem.NAVD88_TO_WGS84 apart)."""
        d = dem.read_geotiff(autonomy.DEM_PATH)
        for key, p in self.points.items():
            self.assertAlmostEqual(d.height(p["lat"], p["lon"]) + dem.NAVD88_TO_WGS84, p["alt"], delta=0.1, msg=key)

    # --- Ground: zones and rocks (urc/terrains.py, urc/features.py) ---

    @staticmethod
    def distance(path, x, y):
        """Distance from points (x, y) to a polyline."""
        return terrain.path_distance(path, x, y)[0]

    def test_ground_follows_the_terrain(self):
        """Each ground type lies where the DEM puts it: sand on the wash floors,
        scree on a face steeper than it holds, gravel and clay on gentle
        ground, slickrock on the caprock and on the rib where the easy route
        climbs onto the butte, steeper than the bare ground climbs."""
        zones = self.sheet["terrain_zones"]
        washes = [[self.xy(p) for p in wash] for wash in self.sheet["judges_only"]["washes"].values()]
        easy = [self.xy(p) for p in self.sheet["judges_only"]["easy_route"]["points"]]
        c2_z = self.sheet["c2"]["z"]
        self.assertEqual({z["type"] for z in zones.values()}, {"sand", "scree", "gravel", "clay", "slickrock"})
        rib = [key for key in zones if key.startswith("easy_route_rib_")]
        self.assertTrue(rib)
        for key, zone in zones.items():
            c = zone["center"]
            slope = self.terrain.slope_deg(c["x"], c["y"])
            if zone["type"] == "sand":
                self.assertLess(min(self.distance(w, c["x"], c["y"]) for w in washes), 1.0, key)
            if zone["type"] == "scree":
                self.assertGreater(slope, terrains.SCREE.traction.hold_deg + 5.0, key)
            elif key in rib:
                self.assertLess(self.distance(easy, c["x"], c["y"]), 0.5, key)
                self.assertGreater(slope, autonomy.EASY_ROUTE_RIB[2], key)
            elif zone["type"] == "slickrock":
                self.assertGreater(c["z"] - c2_z, 3.0, key)  # on the butte's caprock (the plain is 6-10 m below C2)
            else:
                self.assertLess(slope, 10.0, key)
        bare = terrains.TYPES[terrains.DEFAULT_GROUND].traction.climb_deg
        steep = np.array([p for p in terrain.resample(easy, 1.0) if self.terrain.slope_deg(*p) > bare])
        self.assertTrue(len(steep))
        self.assertTrue(np.all(climb_limits("urc_autonomy", *steep.T) > bare))  # all on the rib

    def test_terrain_is_the_dem(self):
        """The world's terrain is the USGS DEM: its zones only paint the ground."""
        _, raw = autonomy.make_site()  # the DEM in the layout frame: metres from C2
        world = self.terrain
        c2 = self.sheet["c2"]
        cx, cy = autonomy.CENTER  # the world frame is centred on the terrain
        offset = c2["z"] - raw.height(c2["x"] + cx, c2["y"] + cy)  # world z - layout z
        X, Y = (a[::4, ::4] for a in world.grid())
        lx, ly = X + cx, Y + cy
        diff = np.abs(world.height(X, Y) - offset - raw.height(lx, ly))
        self.assertLess(diff.max(), 0.01)  # 16-bit heightmap: 1.3 mm steps

    def test_easy_route_and_astronaut_walk_stay_clear(self):
        """No zone of ground that climbs worse than the bare ground and no
        colliding rock within 4 m of the judges' easy route or the
        astronaut's walk; no rock within 1 m of a target, an object or the
        rover's start."""
        easy = [self.xy(p) for p in self.sheet["judges_only"]["easy_route"]["points"]]
        walk = [self.xy(p) for p in self.sheet["astronaut"]["follow_path"]]
        walk.append(self.xy(self.sheet["astronaut"]["stay_to"]))
        bare = terrains.TYPES[terrains.DEFAULT_GROUND].traction.climb_deg
        types = self.sheet["terrain_types"]
        rocks = rock_vertices("urc_autonomy")[:, :2]
        for path in (easy, walk):
            for key, zone in self.sheet["terrain_zones"].items():
                if types[zone["type"]]["climb_deg"] < bare:
                    outline = np.array(zone["outline"])
                    self.assertGreater(self.distance(path, *outline.T).min(), 4.0, key)
            self.assertGreater(self.distance(path, *rocks.T).min(), 4.0)
        places = {**self.points, **self.sheet["objects"], "rover_start": self.sheet["rover_start"]}
        for key, p in places.items():
            self.assertGreater(np.hypot(*(rocks - (p["x"], p["y"])).T).min(), 1.0, key)

    def test_post2_stays_reachable(self):
        """Over the plain from the start, across the wash sand: a route no
        steeper than any ground on it climbs."""
        start, post2 = (self.xy(self.points[k]) for k in ("route_start", "post2"))
        found = routes.easy_route(self.terrain, start, post2, 8.0)
        self.assertIsNotNone(found)
        path = terrain.resample(found[0], 1.0)
        self.assertLess(found[1], climb_limits("urc_autonomy", *path.T).min())

    def test_rocks_work_the_rocker(self):
        rocks = self.sheet["rocks"]
        for group in ("stony_plain", "rubble", "talus", "boulders", "rock_garden_0", "rock_garden_1", "rim_boulders"):
            self.assertGreater(rocks[group]["colliding"], 50, group)
        for k, (*_, size, _) in enumerate(autonomy.ROCK_GARDENS):  # 0.2 and 0.3 m: the wheel radius is 0.15 m
            self.assertEqual(rocks[f"rock_garden_{k}"]["colliding"], rocks[f"rock_garden_{k}"]["count"])
            self.assertTrue(0.08 <= size <= 0.4)


class EquipmentServicing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sheet, _ = load("equipment_servicing")

    def test_lander_within_100_m(self):
        self.assertLessEqual(self.sheet["points"]["lander"]["distance_from_gate_m"], rules.LANDER_MAX_DISTANCE)

    def test_everything_to_service_is_below_1_5_m(self):
        model = model_root(lander.NAME)
        for joint in model.findall("joint"):
            if joint.get("type") == "fixed":
                continue
            child = model.find(f"link[@name='{joint.findtext('child')}']")
            z = vec(child.findtext("pose"))[2]
            self.assertLessEqual(z + 0.05, rules.EQUIPMENT_MAX_HEIGHT, joint.get("name"))

    def test_keyboard(self):
        model = model_root(lander.NAME)
        keys = [j for j in model.findall("joint") if j.get("name").startswith("key_")]
        self.assertEqual(len(keys), rules.KEYBOARD_KEYS)
        for letter in "abcdefghijklmnopqrstuvwxyz":
            self.assertEqual(lander.KEYMAP[f"key_{letter}"], letter)
        tags = {v.get("name"): v.findtext("material/pbr/metal/albedo_map") for v in model.iter("visual")
                if v.get("name").startswith("kb_tag")}
        for corner, tag in rules.KEYBOARD_TAG_IDS.items():
            self.assertIn(f"/aruco_4x4_50_{tag}_", tags[f"kb_tag_{corner}_visual"])

    def test_cache_container(self):
        o = self.sheet["objects"]["cache_container"]
        self.assertGreaterEqual(o["handle_length_m"], rules.CACHE_HANDLE_MIN_LENGTH)
        self.assertLessEqual(o["handle_diameter_m"], rules.CACHE_HANDLE_MAX_DIAMETER)
        self.assertLess(mass(model_root("urc_cache_container")), rules.CACHE_MAX_MASS)

    def test_coupler_fits_the_inlet(self):
        self.assertAlmostEqual(lander.INLET_RADIUS * 2, rules.CAMLOCK_DIAMETER + 0.0139, delta=0.005)
        self.assertGreater(props.COUPLER["inner_radius"], lander.INLET_RADIUS)


class Delivery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sheet, _ = load("delivery")

    def test_objects_follow_the_limits(self):
        for task in self.sheet["tasks"]:
            model = model_root(self.sheet["objects"][task["object"]]["model"])
            self.assertLessEqual(mass(model), rules.OBJECT_MAX_MASS, task["object"])
            self.assertTrue(np.all(collision_extent(model) < rules.OBJECT_MAX_SIZE), task["object"])
            for col in model.iter("collision"):
                if "handle_bar" in col.get("name"):
                    self.assertLessEqual(2 * float(col.findtext("geometry/cylinder/radius")), rules.GRASP_MAX_DIAMETER)

    def test_within_a_kilometre(self):
        gate = self.sheet["points"]["start_gate"]
        for name, o in self.sheet["objects"].items():
            self.assertLess(math.hypot(o["x"] - gate["x"], o["y"] - gate["y"]), rules.DELIVERY_MAX_RANGE, name)

    def test_stages(self):
        stage1 = [t for t in self.sheet["tasks"] if t["stage"] == 1]
        stage2 = [t for t in self.sheet["tasks"] if t["stage"] == 2]
        self.assertTrue(all(t["radio_los_at_pickup"] and t["radio_los_at_delivery"] for t in stage1))
        self.assertTrue(any(not t["radio_los_at_delivery"] for t in stage2))
        far = max(math.hypot(o["x"] - self.sheet["c2"]["x"], o["y"] - self.sheet["c2"]["y"])
                  for o in self.sheet["objects"].values())
        self.assertGreater(far, 600)

    def test_wash_can_be_driven_into_and_out_of(self):
        """D6's spectrometer lies in the wash and the way to the ridge pass
        crosses it: every 20 m along the wash, except where it cuts through
        the ridge, a line straight across it within 8 m stays below the 23
        deg the bare ground climbs (features.WASH_BANK plus the terrain's own
        slope and its micro-relief), grades taken over 2 m (a rover's
        length)."""
        hf = world_terrain("urc_delivery")
        wash = [(p["x"], p["y"]) for p in self.sheet["judges_only"]["wash"]]
        ridge = [(p["x"], p["y"]) for p in self.sheet["judges_only"]["ridge"]]
        points = terrain.resample(wash, 20.0)
        climb = terrains.TYPES[terrains.DEFAULT_GROUND].traction.climb_deg
        span = 4  # samples of the 0.5 m profile: 2 m
        crossings = 0
        for a, b, c in zip(points, points[1:], points[2:]):
            if terrain.path_distance(ridge, *b)[0] < 76.0:  # the ridge reaches 46 m out, the profile 30 m
                continue
            t = (c - a) / math.dist(a, c)
            n = np.array([-t[1], t[0]])
            steepest = []
            for offset in np.arange(-8.0, 8.01, 2.0):
                profile = np.array([hf.height(*(b + offset * t + u * n)) for u in np.arange(-30.0, 30.01, 0.5)])
                steepest.append(np.degrees(np.arctan(np.abs(profile[span:] - profile[:-span]) / 2.0)).max())
            self.assertLess(min(steepest), climb, tuple(b))
            crossings += 1
        self.assertGreater(crossings, 20)

    def test_terrain_gets_harder(self):
        """1.c.ii: the ground gets harder with distance from the start: its
        slopes (over the 2 m the paint rules judge) and its micro-relief."""
        hf = delivery.make_terrain()
        r = hf.radial(0, 0)
        slope = terrain.slope_map(hf.z, hf.res, landscape.SLOPE_SMOOTH)
        self.assertGreater(np.percentile(slope[(r > 450) & (r < 700)], 90), 2 * np.percentile(slope[r < 150], 90))
        rms = landscape.window_rms(hf.z, hf.res, 4.0)
        centre = landscape.windows(r, int(round(4.0 / hf.res))).mean(axis=1)
        self.assertGreater(np.median(rms[(centre > 450) & (centre < 700)]), 1.5 * np.median(rms[centre < 150]))


class Astrobiology(unittest.TestCase):
    def test_units_within_half_a_kilometre(self):
        sheet, _ = load("astrobiology")
        units = [sheet["points"][k] for k in sheet["ground_truth_units"]]
        self.assertGreaterEqual(len({u["title"] for u in units}), 5)
        c2 = sheet["c2"]
        for u in units:
            self.assertLess(math.hypot(u["x"] - c2["x"], u["y"] - c2["y"]) + u["radius_m"], rules.SITE_RADIUS)


class Generated(unittest.TestCase):
    def test_every_mission_has_a_world_and_sheet(self):
        for key in MISSIONS:
            self.assertTrue((WORLDS / f"urc_{key}.sdf").exists(), key)
            sheet, _ = load(key)
            self.assertTrue(sheet["tasks"])
            self.assertIn("rover_start", sheet)

    def test_gz_accepts_the_worlds(self):
        for key in MISSIONS:
            result = gz_check(WORLDS / f"urc_{key}.sdf")
            self.assertIn("Valid", result.stdout, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
