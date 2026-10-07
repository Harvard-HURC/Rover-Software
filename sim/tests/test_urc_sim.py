#!/usr/bin/env python3
"""Headless tests of the URC worlds in Gazebo (pixi run sim-test)."""
import json
import math
import sys
import time
import unittest
from pathlib import Path

import numpy as np

from simulate import SIM_DIR, simulate
from worldfiles import sheet, terrain, world_copy

import gz.math7  # noqa: F401  (lets gz.sim8 return Pose3d values)
from gz.sim8 import Joint, Model, TestFixture, World, world_entity
from gz.transport13 import Node

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import geo  # noqa: E402
from urc import judge as J  # noqa: E402
from urc import sheet as sheets  # noqa: E402
from urc.missions import MISSIONS  # noqa: E402


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
CAMERA_RANGE = 2.5  # [m] rover centre to the post; the camera is 0.35 m ahead of it


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
        north face at Post 1, which bare ground lets it climb (without the
        rim's boulders this run ends 0.6 m from Post 1), the rover stops at
        the boulders 7 m short of it (measured 8.8 m)."""
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

        Why square on and 2.5 m: 640 px over 1.5 rad (f = 343 px) put a
        2.5 cm tag cell on 4.1 px at 2.5 m from the post (the camera 2.1 m
        from the face), and OpenCV's default detector misses a face turned
        52-60 deg away (the worst the post's random yaw gives: its three faces
        are 120 deg apart) even there. Square on, measured on this world with
        the post turned to 14 yaws (every 30 deg, 75 and 115 deg), it decoded
        every one at 2.5 m but 2 of 4 at 3 m. So the rover stands on a face's
        normal, taken from the post's yaw in the sheet (props.ar_post: face k
        looks along yaw + 2 pi k / 3), and the test does not depend on the yaw
        a regeneration draws. A team's HD camera reads the tag from further."""
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

if __name__ == "__main__":
    unittest.main()
