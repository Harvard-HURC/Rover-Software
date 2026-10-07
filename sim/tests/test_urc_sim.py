#!/usr/bin/env python3
"""Headless tests of the URC worlds in Gazebo (pixi run sim-test), with the
physical rover (the default model://rover).

MissionRoutes drives each mission's designed routes with a pure-pursuit
driver on ground truth (simulate.follow): minutes of sim time each, so they
run only with ROVER_SLOW=1. The other classes take seconds."""
import contextlib
import json
import math
import os
import re
import sys
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from simulate import SIM_DIR, follow, simulate, spin_ratio
from worldfiles import rock_vertices, sheet, terrain, world_copy

import gz.math7  # noqa: F401  (lets gz.sim8 return Pose3d values)
from gz.sim8 import Joint, Model, TestFixture, World, world_entity
from gz.transport13 import Node

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import geo, routes, terrains  # noqa: E402
from urc import judge as J  # noqa: E402
from urc import sheet as sheets  # noqa: E402
from urc import terrain as heightfields  # noqa: E402
from urc.missions import MISSIONS, delivery  # noqa: E402


CAMERA_CHECK = """
import json, sys
import cv2, numpy as np
sys.path.insert(0, {tests!r})
from simulate import simulate
from gz.msgs10.image_pb2 import Image
topic = {topic!r}
state = simulate(1.5, world={world!r}, subscribe=[(topic, Image)])
img = state.messages[topic][-1]
rgb = np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3)
detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
_, ids, _ = detector.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
print("IDS", json.dumps([] if ids is None else ids.flatten().tolist()))
"""
CAMERA_RANGE = 5.0  # [m] rover centre to the post; the camera is 0.35 m ahead of it
ROCK_STOP = 0.25  # [m] a rock standing higher than this stops the rover (proving ground: 0.2 m crossed, 0.3 m not)
ROVER_HALF_WIDTH = gen_model.Params().pivot_y + gen_model.Params().wheel_width / 2  # [m] over the wheels
SLOW = unittest.skipUnless(os.environ.get("ROVER_SLOW"), "slow (many minutes of sim time): set ROVER_SLOW=1")


@contextlib.contextmanager
def fast_copy(world, rover, lift=0.05):
    """world_copy at real_time_factor 0: as fast as it runs."""
    with world_copy(world, rover=rover, lift=lift) as path:
        p = Path(path)
        p.write_text(re.sub(r"<real_time_factor>[^<]*</real_time_factor>", "<real_time_factor>0</real_time_factor>",
                            p.read_text()))
        yield path


def clear_route(world, start, goal, max_slope, clearance=1.0, margin=3.0):
    """A route (world (x, y)) from start to goal with grades up to max_slope
    (routes.easy_route) that keeps `clearance` past the rover's half width
    from every rock taller than ROCK_STOP and off ground steeper than its
    type climbs less `margin` [deg] (ground.json): the search sees those as
    walls in the terrain. None if there is none."""
    hf = terrain(world)
    s = sheet(world)
    ground = sheets.ground(s, sheets.path(world))
    climb = np.zeros(256)
    for t in ground.info["types"]:
        climb[t["index"]] = math.degrees(math.atan(max(t["mu_k"] - t["crr"], 0.0)))
    blocked = (hf.slope_map() + margin > climb[ground.raster]).astype(np.uint8)
    V = rock_vertices(world)
    tall = V[V[:, 2] - hf.height(V[:, 0], V[:, 1]) > ROCK_STOP]
    cols = np.round((tall[:, 0] - hf.center[0] + hf.size / 2) / hf.res).astype(int)
    rows = np.round((hf.center[1] + hf.size / 2 - tall[:, 1]) / hf.res).astype(int)
    rocks = np.zeros_like(blocked)
    rocks[rows, cols] = 1
    reach = int(math.ceil((ROVER_HALF_WIDTH + clearance) / hf.res))
    blocked |= cv2.dilate(rocks, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * reach + 1,) * 2))
    walls = heightfields.Heightfield(hf.size, hf.n, hf.z + 100.0 * blocked, hf.center)
    found = routes.easy_route(walls, start, goal, max_slope)
    return None if found is None else found[0]


def drive_route(world, path, seconds, speed=0.8, tolerance=2.0, skip=2.5):
    """follow() a world (x, y) route, the rover spawned `skip` metres along
    it (off whatever stands at its start), facing along it."""
    dense = heightfields.resample(path, 0.25)
    k = int(round(skip / 0.25))
    x, y = dense[k]
    yaw = math.atan2(dense[k + 4][1] - y, dense[k + 4][0] - x)
    with fast_copy(world, (float(x), float(y), yaw)) as copy:
        return follow(copy, [tuple(p) for p in dense[k:]], seconds, speed=speed, tolerance=tolerance)


def xy(entry):
    return entry["x"], entry["y"]


class Worlds(unittest.TestCase):
    def test_rover_rests_on_the_terrain(self):
        for mission in MISSIONS:
            with self.subTest(mission=mission):
                ground = terrain(f"urc_{mission}")
                with world_copy(f"urc_{mission}") as world:
                    state = simulate(2.0, world=world)
                x, y, z, roll, pitch, _ = state.poses["base_link"]
                start = sheet(f"urc_{mission}")["rover_start"]
                self.assertLess(math.hypot(x - start["x"], y - start["y"]), 0.1)
                self.assertAlmostEqual(z, ground.height(x, y), delta=0.06)
                self.assertLess(max(abs(roll), abs(pitch)), math.radians(8))
                for wheel in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
                    wx, wy, wz = state.poses[wheel][:3]
                    self.assertAlmostEqual(wz - gen_model.Params().wheel_radius, ground.height(wx, wy), delta=0.06,
                                           msg=f"{mission} {wheel}")

    def test_gnss_reports_the_sheet_position(self):
        from gz.msgs10.navsat_pb2 import NavSat
        s = sheet("urc_autonomy")
        with world_copy("urc_autonomy") as world:
            state = simulate(4.0, world=world, subscribe=[(gen_model.NAVSAT_TOPIC, NavSat)])
        fixes = state.messages[gen_model.NAVSAT_TOPIC][5:]
        self.assertGreater(len(fixes), 20)
        start = s["rover_start"]
        lat = np.mean([m.latitude_deg for m in fixes])
        lon = np.mean([m.longitude_deg for m in fixes])
        origin = geo.Origin(start["lat"], start["lon"], start["alt"])
        east, north, _ = geo.wgs84_to_enu(origin, lat, lon, start["alt"])
        self.assertLess(math.hypot(east, north), 1.0)  # antenna lever arm 0.25 m + noise

    def test_rim_boulders_stop_a_straight_climb(self):
        """1.e.xv in the physics: driven straight up the butte's 32-40 deg
        north face at Post 1, steeper than its packed regolith climbs (23 deg),
        and lined with the rim's boulders, the rover stays well below Post 1."""
        post = sheet("urc_autonomy")["points"]["post1"]
        ground = terrain("urc_autonomy")
        x, y = post["x"], post["y"] + 16.0
        slope = math.atan2(ground.height(x, y - 1.0) - ground.height(x, y + 1.0), 2.0)
        with world_copy("urc_autonomy", rover=(x, y, -math.pi / 2), pitch=-slope) as world:
            path = Path(world)  # as fast as it runs: 40 s of driving take 14 s instead of 40
            path.write_text(path.read_text().replace("<real_time_factor>1</real_time_factor>",
                                                     "<real_time_factor>0</real_time_factor>"))
            state = simulate(40.0, world=world, cmd=(0.5, 0.0))
        bx, by, bz = state.poses["base_link"][:3]
        self.assertGreater(by - post["y"], 6.0)
        self.assertLess(bz, post["z"] - 3.0)

    def test_camera_sees_the_start_post(self):
        """The rover's camera, CAMERA_RANGE in front of one of the start
        post's faces and facing it, decodes ArUco 0 (rendered in a separate
        process: see world_copy).

        Why square on and 5 m: 1280 px over 1.5 rad (f = 686 px) put a
        2.5 cm tag cell on 3.6 px at 5 m from the post (the camera 4.65 m
        from the face), about what the old 640 px camera gave at 2.5 m, where
        it decoded every one of 14 post yaws square on but faces 52-60 deg
        away only by luck (measured on this world). So the rover stands on a
        face's normal, taken from the post's yaw in the sheet (props.ar_post:
        face k looks along yaw + 2 pi k / 3), and the test does not depend on
        the yaw a regeneration draws."""
        import subprocess
        post = sheet("urc_autonomy")["objects"]["route_start"]
        x, y = post["x"] + CAMERA_RANGE * math.cos(post["yaw"]), post["y"] + CAMERA_RANGE * math.sin(post["yaw"])
        with world_copy("urc_autonomy", rover=(x, y, post["yaw"] + math.pi), sensors=True) as world:
            code = CAMERA_CHECK.format(tests=str(SIM_DIR / "tests"), topic=gen_model.CAMERA_TOPIC + "/image",
                                       world=world)
            result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=300)
        line = next((l for l in result.stdout.splitlines() if l.startswith("IDS")), None)
        self.assertIsNotNone(line, result.stdout[-2000:] + result.stderr[-2000:])
        self.assertIn(0, json.loads(line[4:]))


class Lander(unittest.TestCase):
    def test_key_presses_become_text(self):
        from gz.msgs10.stringmsg_pb2 import StringMsg
        presses = []
        node = Node()
        node.subscribe(StringMsg, "/model/lander/presses", lambda m: presses.append(m.data))
        plan = sorted([(0.5, "key_s"), (0.8, "key_x"), (1.1, "key_backspace"), (1.4, "key_o"), (1.7, "key_l")])
        joints = {}

        def pre(info, ecm):
            if not joints:
                model = Model(World(world_entity(ecm)).model_by_name(ecm, "lander"))
                for _, name in plan:
                    joints[name] = Joint(model.joint_by_name(ecm, name))
            t = info.iterations / 1000
            for start, name in plan:
                if start <= t < start + 0.1:
                    joints[name].set_force(ecm, [2.0])

        with world_copy("urc_equipment_servicing") as world:
            fixture = TestFixture(world)
            fixture.on_pre_update(pre)
            fixture.finalize()
            fixture.server().run(True, 2200, False)
        time.sleep(0.3)
        self.assertEqual(presses, [name for _, name in plan])
        text = ""
        for name in presses:
            text = J.type_text(text, name)
        self.assertEqual(text, "sol")


class RefereeRun(unittest.TestCase):
    def test_route_target_scored_end_to_end(self):
        """Rover parked by Post 1 with the arrival LED: the referee scores it."""
        from gz.msgs10.stringmsg_pb2 import StringMsg
        import referee
        post = sheet("urc_autonomy")["points"]["post1"]
        with world_copy("urc_autonomy", rover=(post["x"], post["y"] - 1.2, math.pi / 2)) as world:
            fixture = TestFixture(world)
            fixture.finalize()
            server = fixture.server()
            server.run(False, 30000, False)  # in the background
            path = sheets.path("urc_autonomy")
            judge = J.make_judge("autonomy", sheets.load(path), path)
            ref = referee.Referee("autonomy", path, judge)
            led = Node().advertise(gen_model.LED_TOPIC, StringMsg)
            msg = StringMsg()
            msg.data = "green"
            deadline = time.time() + 60
            while "post1" not in judge.scores and time.time() < deadline:
                led.publish(msg)
                ref.tick()
                time.sleep(0.05)
            self.assertIn("post1", judge.scores, judge.events)
            self.assertNotIn("post2", judge.scores)


class DeliveryGround(unittest.TestCase):
    """The ground of Delivery's rule 1.c.ii with the physical drivetrain
    (design spec 11): the sand flat crossed straight, a spin in it digging in
    (the strong preset, the user's choice), the crate hill climbable round its
    clay flank, the scree chute up the steep mesa not."""

    @classmethod
    def setUpClass(cls):
        cls.sheet = sheet("urc_delivery")
        cls.zones = cls.sheet["terrain_zones"]

    def world(self, layout):
        """World (x, y) of a layout point of delivery.py (C2 at layout 0, 0)."""
        return layout[0] + self.sheet["c2"]["x"], layout[1] + self.sheet["c2"]["y"]

    def test_sand_flat_is_crossed_straight(self):
        """At 0.5 m/s, slipping about a quarter in the sand (slip x crr x the
        dig factor, design spec 5.6, 6.5) but never stuck."""
        x, y = xy(self.zones["sand_flat"]["center"])
        with fast_copy("urc_delivery", (x - 20.0, y, 0.0)) as world:
            s = simulate(130.0, world=world, cmd=(0.5, 0.0), trace_every=100)
        self.assertGreater(s.poses["base_link"][0], x + 16.0, s.poses["base_link"])
        t, along = s.trace[:, 0], s.trace[:, 1]
        in_sand = (along > x - 8.0) & (along < x + 8.0)
        speed = np.diff(along[in_sand]).sum() / np.diff(t[in_sand]).sum()
        self.assertTrue(0.25 < speed < 0.45, speed)

    def test_a_spin_in_the_sand_digs_in(self):
        """A sustained spin slows until the rover hardly turns (design spec
        6.9: strong preset), and the rover then drives out straight, slowly
        while its wheels are dug in (measured 0.22 m/s of a commanded 0.3)."""
        x, y = xy(self.zones["sand_flat"]["center"])
        cmd = [(0.0, 0.0, 0.0), (0.5, 0.0, 1.0), (10.5, 0.0, 0.0), (11.0, 0.3, 0.0)]
        with fast_copy("urc_delivery", (x, y, 0.0)) as world:
            s = simulate(40.0, world=world, cmd=cmd, trace_every=10)
        t, rate = s.trace[:, 0], s.trace[:, 7]
        fresh, dug = rate[(t >= 0.5) & (t < 1.5)].max(), rate[(t >= 8.5) & (t < 10.5)].mean()
        self.assertGreater(fresh, 0.8 * spin_ratio(terrains.SAND.traction))  # it did start turning
        self.assertLess(dug, 0.25 * fresh, (fresh, dug))
        out = np.hypot(s.trace[:, 1] - x, s.trace[:, 2] - y)
        self.assertGreater(out[-1] - out[t <= 30.0][-1], 1.5)  # still driving out, not stuck
        self.assertGreater(out[-1], 4.0)

    def test_crate_hill_is_climbable(self):
        """From its gentle side, off the clay flank, up to astronaut D (D3)."""
        top = xy(self.sheet["points"]["astronaut_d"])
        route = clear_route("urc_delivery", (top[0] + 30.0, top[1] + 20.0), top, 14.0)
        self.assertIsNotNone(route)
        d = drive_route("urc_delivery", route, 120.0, tolerance=2.5, skip=1.0)
        self.assertIsNotNone(d.arrived, f"{d.gap(top):.1f} m short")

    def test_scree_chute_is_not_climbable(self):
        """Straight up the ~26 deg chute (scree climbs 19 deg): the rover stays
        in its lower part."""
        chute = next(f for f in delivery.FEATURES if f.key == "scree_chute")
        (fx, fy), (tx, ty) = self.world(chute.foot), self.world(chute.top)
        yaw = math.atan2(ty - fy, tx - fx)
        with fast_copy("urc_delivery", (fx - 2.0 * math.cos(yaw), fy - 2.0 * math.sin(yaw), yaw)) as world:
            s = simulate(40.0, world=world, cmd=(0.5, 0.0))
        x, y = s.poses["base_link"][:2]
        climbed = (x - fx) * math.cos(yaw) + (y - fy) * math.sin(yaw)
        self.assertLess(climbed, 0.6 * math.dist((fx, fy), (tx, ty)), (x, y))


@SLOW
class MissionRoutes(unittest.TestCase):
    """Each mission's designed routes, driven by the physical rover with the
    strong dig-in default: pure pursuit on ground truth at 0.8 m/s, never
    turning in place (simulate.pursue; a spin in sand digs in, which is
    intended, so no route may need one). Routes the missions do not draw
    are the easiest clear ones between their places (clear_route)."""

    def test_autonomy_easy_route_to_post1(self):
        """The judges' easy route from the start post to within 3 m of Post 1
        in at most 6 minutes of sim time (design spec 11, A)."""
        easy = [xy(p) for p in sheet("urc_autonomy")["judges_only"]["easy_route"]["points"]]
        d = drive_route("urc_autonomy", easy, 360.0, speed=1.0, tolerance=3.0)
        self.assertIsNotNone(d.arrived, f"{d.gap(easy[-1]):.1f} m short of Post 1")

    def test_autonomy_route_to_post2(self):
        """Over the plain and the wash sand from the start post to Post 2."""
        s = sheet("urc_autonomy")
        route = clear_route("urc_autonomy", xy(s["points"]["route_start"]), xy(s["points"]["post2"]), 8.0)
        d = drive_route("urc_autonomy", route, 600.0)
        self.assertIsNotNone(d.arrived, f"{d.gap(route[-1]):.1f} m short of Post 2")

    def test_autonomy_astronaut_walk(self):
        """After the astronaut: the Follow! walk and on to the Stay! point."""
        s = sheet("urc_autonomy")["astronaut"]
        walk = [xy(p) for p in s["follow_path"]] + [xy(s["stay_to"])]
        d = drive_route("urc_autonomy", walk, 200.0)
        self.assertIsNotNone(d.arrived, f"{d.gap(walk[-1]):.1f} m short of the Stay! point")

    def test_delivery_course(self):
        """Every leg of the course in task order (D1-D6): to each object and
        on to its astronaut, over the ridge pass to astronaut C, out of the
        wash with the spectrometer."""
        s = sheet("urc_delivery")
        place = {**{k: xy(p) for k, p in s["points"].items()}, **{k: xy(o) for k, o in s["objects"].items()},
                 "rover_start": xy(s["rover_start"])}
        legs = [("rover_start", "toolbox"), ("toolbox", "astronaut_a"), ("astronaut_a", "water_jug"),
                ("water_jug", "astronaut_b"), ("astronaut_b", "supply_crate"), ("supply_crate", "astronaut_d"),
                ("astronaut_d", "field_sign"), ("field_sign", "instrument_case"), ("instrument_case", "first_aid_kit"),
                ("first_aid_kit", "astronaut_c"), ("spectrometer", "astronaut_c")]
        for a, b in legs:
            with self.subTest(leg=f"{a} -> {b}"):
                route = clear_route("urc_delivery", place[a], place[b], 16.0)
                self.assertIsNotNone(route)
                length = float(np.sum(np.hypot(*np.diff(np.asarray(route), axis=0).T)))
                d = drive_route("urc_delivery", route, 60.0 + 2.0 * length, tolerance=2.5)
                self.assertIsNotNone(d.arrived, f"{d.gap(place[b]):.1f} m short")

    def test_equipment_approach(self):
        """From the gate to the lander's front, the approach the rocks keep clear."""
        s = sheet("urc_equipment_servicing")
        gate, lander = xy(s["points"]["start_gate"]), xy(s["points"]["lander"])
        front = (lander[0] - 4.0, lander[1])  # 3 m off its face (x = 99 in the layout faces the gate)
        d = drive_route("urc_equipment_servicing", [gate, front], 240.0)
        self.assertIsNotNone(d.arrived, f"{d.gap(front):.1f} m short of the lander")

    def test_astrobiology_wash_and_gypsum(self):
        """From C2 into the dry wash (a sampling unit), out of it again up the
        gypsum mound, then back to C2 (1.b.vii)."""
        s = sheet("urc_astrobiology")
        stops = [xy(s["c2"]), xy(s["points"]["dry_wash"]), xy(s["points"]["gypsum"]), xy(s["c2"])]
        for a, b in zip(stops, stops[1:]):
            with self.subTest(leg=(a, b)):
                route = clear_route("urc_astrobiology", a, b, 16.0)
                self.assertIsNotNone(route)
                length = float(np.sum(np.hypot(*np.diff(np.asarray(route), axis=0).T)))
                d = drive_route("urc_astrobiology", route, 60.0 + 2.0 * length, tolerance=3.0, skip=4.0)
                self.assertIsNotNone(d.arrived, f"{d.gap(b):.1f} m short")


if __name__ == "__main__":
    unittest.main()
