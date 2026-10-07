#!/usr/bin/env python3
"""Headless physics tests for the rover model (pixi run sim-test)."""
import math
import unittest

from simulate import gen_model, simulate
from worldfiles import world_copy

P = gen_model.Params()
STEP = 0.10
BLOCK_UNDER_FL = f"""
    <model name="block">
      <static>true</static>
      <pose>{P.wheel_dx} {P.pivot_y} {STEP / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.4 0.3 {STEP}</size></box></geometry></collision>
      </link>
    </model>"""


class Suspension(unittest.TestCase):
    def test_rests_level_on_flat_ground(self):
        s = simulate(2.0)
        _, _, z, roll, pitch, _ = s.poses["base_link"]
        self.assertAlmostEqual(z, 0.0, delta=0.02)
        self.assertLess(abs(roll), math.radians(1))
        self.assertLess(abs(pitch), math.radians(1))
        for joint, q in s.rockers.items():
            self.assertAlmostEqual(q, 0.0, delta=0.01, msg=joint)

    def test_differential_keeps_all_wheels_down(self):
        s = simulate(2.0, extra=BLOCK_UNDER_FL, spawn_z=STEP + 0.02)
        q_l, q_r = s.rockers["rocker_left_joint"], s.rockers["rocker_right_joint"]
        # The left rocker pitches front-up by atan(step / wheelbase); the
        # differential puts the body halfway, so each rocker sees half of it.
        self.assertLess(abs(q_l + q_r), 0.005)
        self.assertAlmostEqual(q_l, -math.atan(STEP / (2 * P.wheel_dx)) / 2, delta=0.01)
        # A rigid chassis would leave one wheel in the air.
        ground = {"wheel_fl": STEP, "wheel_fr": 0.0, "wheel_rl": 0.0, "wheel_rr": 0.0}
        for wheel, height in ground.items():
            self.assertAlmostEqual(s.poses[wheel][2], height + P.wheel_radius, delta=0.02, msg=wheel)


class Drive(unittest.TestCase):
    def test_drives_straight(self):
        s = simulate(4.0, cmd=(0.5, 0.0))
        x, y, _, _, _, yaw = s.poses["base_link"]
        self.assertTrue(1.7 <= x <= 2.1, f"x = {x}")
        self.assertLess(abs(y), 0.1)
        self.assertLess(abs(yaw), 0.05)

    def test_turns_in_place(self):
        s = simulate(4.0, cmd=(0.0, 0.5))
        x, y, _, _, _, yaw = s.poses["base_link"]
        # Commanded 2 rad; skid-steer scrub makes it turn slower (sim/README.md).
        self.assertTrue(1.2 <= yaw <= 2.05, f"yaw = {yaw}")
        self.assertLess(math.hypot(x, y), 0.15)


def frame_id(header):
    return next(d.value[0] for d in header.data if d.key == "frame_id")


class Topics(unittest.TestCase):
    def test_sensors_and_odometry_publish(self):
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.odometry_pb2 import Odometry

        topics = [(gen_model.IMU_TOPIC, IMU), (gen_model.JOINT_STATE_TOPIC, Model),
                  (gen_model.ODOM_TOPIC, Odometry), (gen_model.GROUND_TRUTH_TOPIC, Odometry)]
        s = simulate(1.0, subscribe=topics)
        for topic, _ in topics:
            self.assertTrue(s.messages[topic], f"nothing on {topic}")
        imu = s.messages[gen_model.IMU_TOPIC][-1]
        self.assertEqual(frame_id(imu.header), "base_link")
        a = imu.linear_acceleration
        self.assertAlmostEqual(math.sqrt(a.x**2 + a.y**2 + a.z**2), 9.8, delta=0.3)
        joints = {j.name for j in s.messages[gen_model.JOINT_STATE_TOPIC][-1].joint}
        self.assertEqual(joints, {"rocker_left_joint", "rocker_right_joint", "wheel_fl_joint",
                                  "wheel_rl_joint", "wheel_fr_joint", "wheel_rr_joint",
                                  gen_model.PAN_JOINT, gen_model.TILT_JOINT})


class DemoWorld(unittest.TestCase):
    def test_loads_and_rover_settles(self):
        with world_copy("rover_test") as world:  # without cameras: ogre2 starts once per process
            s = simulate(1.0, world=world)
        _, _, z, roll, pitch, _ = s.poses["base_link"]
        self.assertAlmostEqual(z, 0.0, delta=0.02)
        self.assertLess(max(abs(roll), abs(pitch)), math.radians(1))


if __name__ == "__main__":
    unittest.main()
