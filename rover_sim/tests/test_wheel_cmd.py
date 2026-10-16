#!/usr/bin/env python3
"""wheel_cmd.py, the ROS-free way to drive the simulated rover: the skid-steer formula and the ramp the drivetrain
applied before PR 3, what Driver and TwistDrive publish and when, and that the drive controller of the ROS stack ramps
each wheel at the same rate (rover_control/config/controllers.yaml). Nothing runs Gazebo; Task 10 of the PR 3 plan
compared whole runs bit for bit."""
import unittest

import yaml

from worldfiles import SIM_DIR  # noqa: F401  (puts rover_sim/ on the path)

import rover_config  # noqa: E402
import wheel_cmd  # noqa: E402
from rover_config import ROVER, SIM  # noqa: E402

TRACK, RADIUS, TOP = 2 * ROVER.rocker.pivot_y, ROVER.wheel.radius, ROVER.wheel.top_speed


class FakeNode:
    """advertise() as gz.transport13.Node has it, recording a copy of every message published as (topic, message)."""

    def __init__(self):
        self.sent = []

    def advertise(self, topic, kind):
        sent = self.sent

        class Publisher:
            def publish(self, msg):
                copy = kind()
                copy.CopyFrom(msg)
                sent.append((topic, copy))

        return Publisher()

    def of(self, topic):
        return [msg for t, msg in self.sent if t == topic]


class Setpoints(unittest.TestCase):
    def test_the_drivetrains_formula(self):
        vx, wz = 0.3, 0.2
        left, right = (vx - 1.0 * wz * TRACK / 2) / RADIUS, (vx - -1.0 * wz * TRACK / 2) / RADIUS
        self.assertEqual(wheel_cmd.setpoints(vx, wz), (left, left, right, right))
        self.assertEqual(len(set(wheel_cmd.setpoints(0.5, 0.0))), 1)  # straight: all alike
        fl, rl, fr, rr = wheel_cmd.setpoints(0.0, 1.0)
        self.assertEqual((fl, rl), (-fr, -rr))  # in place: the sides opposite
        self.assertAlmostEqual(fr, ROVER.rocker.pivot_y / RADIUS)
        self.assertEqual(wheel_cmd.WHEELS, ("fl", "rl", "fr", "rr"))


class RampSteps(unittest.TestCase):
    def test_at_most_accel_per_second(self):
        ramp = wheel_cmd.Ramp(accel=8.0)
        self.assertEqual(ramp.step(0.3, 0.0), (8.0 * 0.001,) * 4)  # 2 rad/s asked, 8 rad/s^2 for 1 ms
        for _ in range(248):
            values = ramp.step(0.3, 0.0)
        self.assertAlmostEqual(values[0], 249 * 0.008, places=9)
        for _ in range(10):
            values = ramp.step(0.3, 0.0)
        for value in values:
            self.assertAlmostEqual(value, 0.3 / RADIUS, places=12)  # arrived, and held

    def test_the_top_speed_first(self):
        ramp = wheel_cmd.Ramp(accel=0.0)
        for value in ramp.step(10.0, 0.0):  # 66.7 rad/s asked
            self.assertAlmostEqual(value, TOP, places=9)

    def test_without_accel_the_arithmetic_stays(self):
        """accel 0 is no limit, but the value still goes through SpeedLimiter's arithmetic, previous + (target -
        previous) / dt x dt, which may differ from the target in the last bit, as the drivetrain's did."""
        ramp = wheel_cmd.Ramp(accel=0.0)
        target = wheel_cmd.setpoints(0.37, 0.11)
        self.assertEqual(ramp.step(0.37, 0.11), tuple(0.0 + ((t - 0.0) / 0.001) * 0.001 for t in target))

    def test_one_rounding_as_libgz_math_computes_it(self):
        """previous + acceleration x dt is rounded once, as libgz-math7's SpeedLimiter, which ramped the drivetrain's
        setpoints until PR 3, computes it on osx-arm64 (a fused multiply-add; measured against the library,
        2026-10-10): a jump from a turn in place back to zero leaves the residue the drivetrain left, where two
        roundings give 0."""
        ramp = wheel_cmd.Ramp(accel=0.0)
        self.assertEqual(ramp.step(0.0, 1.0)[0].hex(), "-0x1.5555555555556p+1")
        fl, rl, fr, rr = ramp.step(0.0, 0.0)
        self.assertEqual((fl, rl), (float.fromhex("0x1.20c49ba5e354p-54"),) * 2)
        self.assertEqual((fr, rr), (-fl, -fl))

    def test_reset_starts_from_zero(self):
        ramp = wheel_cmd.Ramp(accel=8.0)
        ramp.step(0.3, 0.0)
        ramp.reset()
        self.assertEqual(ramp.values, (0.0,) * 4)


class DriverPublishes(unittest.TestCase):
    def test_nothing_before_the_first_command(self):
        node = FakeNode()
        driver = wheel_cmd.Driver(node)
        for _ in range(30):
            driver.step(force=True)
        self.assertEqual(node.sent, [])

    def test_every_change_and_the_beat(self):
        node = FakeNode()
        driver = wheel_cmd.Driver(node, accel=8.0)
        driver.command(0.3, 0.0)
        self.assertEqual([(m.linear.x, m.angular.z) for m in node.of(rover_config.CMD_VEL_TOPIC)], [(0.3, 0.0)])
        for _ in range(300):
            driver.step()
        wheels = node.of(rover_config.WHEEL_CMD_TOPIC)
        self.assertTrue(248 <= len(wheels) <= 255, len(wheels))  # the 250 steps of the ramp, not the held ones
        self.assertAlmostEqual(wheels[-1].data[0], 0.3 / RADIUS, places=9)
        driver.step(force=True)
        self.assertEqual(len(node.of(rover_config.WHEEL_CMD_TOPIC)), len(wheels) + 1)  # the beat repeats them

    def test_a_reset_waits_for_the_next_command(self):
        node = FakeNode()
        driver = wheel_cmd.Driver(node, accel=8.0)
        driver.command(0.3, 0.0)
        driver.step()
        driver.reset()
        count = len(node.sent)
        driver.step(force=True)
        self.assertEqual(len(node.sent), count)
        driver.command(0.3, 0.0)
        driver.step()
        self.assertEqual(node.of(rover_config.WHEEL_CMD_TOPIC)[-1].data[0], 8.0 * 0.001)  # from zero again


class TwistDriveSends(unittest.TestCase):
    def test_unramped_setpoints_and_the_twist(self):
        node = FakeNode()
        wheel_cmd.TwistDrive(node).twist(0.6, 0.8)
        self.assertEqual(tuple(node.of(rover_config.WHEEL_CMD_TOPIC)[0].data), wheel_cmd.setpoints(0.6, 0.8))
        self.assertEqual(node.of(rover_config.CMD_VEL_TOPIC)[0].angular.z, 0.8)


class DriveController(unittest.TestCase):
    def test_it_ramps_each_wheel_at_the_helpers_rate(self):
        """controllers.yaml's linear and angular acceleration limits are sim.yaml's wheel_cmd accel for a wheel
        driving straight and turning in place, and its kinematics have no track multiplier (1)."""
        params = yaml.safe_load(rover_config.CONTROLLERS_YAML.read_text())["drive_controller"]["ros__parameters"]
        accel, half = SIM.wheel_cmd.accel, ROVER.rocker.pivot_y
        self.assertAlmostEqual(params["linear.x.max_acceleration"] / RADIUS, accel, places=9)
        self.assertAlmostEqual(-params["linear.x.max_deceleration"] / RADIUS, accel, places=9)
        self.assertAlmostEqual(params["angular.z.max_acceleration"] * half / RADIUS, accel, places=9)
        self.assertAlmostEqual(-params["angular.z.max_deceleration"] * half / RADIUS, accel, places=9)
        self.assertEqual(params["wheel_separation_multiplier"], 1.0)


if __name__ == "__main__":
    unittest.main()
