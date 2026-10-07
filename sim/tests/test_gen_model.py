#!/usr/bin/env python3
"""Unit tests for sim/gen_model.py (no physics; pixi run sim-test)."""
import unittest
import xml.etree.ElementTree as ET

from worldfiles import gz_check, temp_sdf, vec

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import sdf  # noqa: E402

P = gen_model.Params()


class Inertia(unittest.TestCase):
    def test_box(self):
        self.assertEqual(sdf.box_inertia(12.0, (1.0, 2.0, 3.0)), (13.0, 10.0, 5.0))

    def test_wheel_axis_is_y(self):
        ixx, iyy, izz = gen_model.wheel_inertia(2.0, 0.5, 1.0)
        self.assertAlmostEqual(iyy, 2.0 * 0.5**2 / 2)
        self.assertAlmostEqual(ixx, 2.0 * (3 * 0.5**2 + 1.0**2) / 12)
        self.assertEqual(ixx, izz)

    def test_flat_rocker_is_one_rod(self):
        # Rods to (+-1, 0, 0) form one rod of length 2 about its middle.
        self.assertEqual(gen_model.rocker_inertia(2.0, 1.0, 0.0), (0.0, 2.0 * 4 / 12, 2.0 * 4 / 12))

    def test_vertical_rocker_is_one_rod(self):
        # Both rods straight down to (0, 0, -1) overlap: one rod of length 1.
        ixx, iyy, izz = gen_model.rocker_inertia(2.0, 0.0, -1.0)
        self.assertAlmostEqual(ixx, 2.0 / 12)
        self.assertAlmostEqual(iyy, 2.0 / 12)
        self.assertEqual(izz, 0.0)


class Structure(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = gen_model.build_sdf(P)
        cls.model = ET.fromstring(cls.sdf).find("model")

    def plugin(self, name):
        return self.model.find(f"plugin[@name='{name}']")

    def test_joint_tree(self):
        tree = {j.get("name"): (j.findtext("parent"), j.findtext("child")) for j in self.model.findall("joint")}
        self.assertEqual(tree, {
            "rocker_left_joint": ("base_link", "rocker_left"),
            "rocker_right_joint": ("base_link", "rocker_right"),
            "wheel_fl_joint": ("rocker_left", "wheel_fl"),
            "wheel_rl_joint": ("rocker_left", "wheel_rl"),
            "wheel_fr_joint": ("rocker_right", "wheel_fr"),
            "wheel_rr_joint": ("rocker_right", "wheel_rr"),
            "camera_pan_joint": ("base_link", "camera_pan_link"),
            "camera_tilt_joint": ("camera_pan_link", "camera_tilt_link"),
        })
        for joint in self.model.findall("joint"):
            axis = [0, 0, 1] if joint.get("name") == gen_model.PAN_JOINT else [0, 1, 0]
            self.assertEqual(vec(joint.findtext("axis/xyz")), axis, joint.get("name"))

    def test_wheel_centers_follow_params(self):
        z = P.pivot_z + P.wheel_dz
        expected = {"wheel_fl": (P.wheel_dx, P.pivot_y), "wheel_rl": (-P.wheel_dx, P.pivot_y),
                    "wheel_fr": (P.wheel_dx, -P.pivot_y), "wheel_rr": (-P.wheel_dx, -P.pivot_y)}
        for name, (x, y) in expected.items():
            pose = vec(self.model.find(f"link[@name='{name}']/pose").text)
            for got, want in zip(pose, (x, y, z, 0, 0, 0)):
                self.assertAlmostEqual(got, want, msg=name)

    def test_wheels_rest_on_the_ground(self):
        self.assertAlmostEqual(P.pivot_z + P.wheel_dz, P.wheel_radius)

    def test_total_mass(self):
        total = sum(float(m.text) for m in self.model.iter("mass"))
        self.assertAlmostEqual(total, P.chassis_mass + 2 * P.rocker_mass + 4 * P.wheel_mass + P.camera_pan_mass
                               + P.camera_tilt_mass)

    def test_camera_on_the_pan_tilt_head(self):
        sensor = self.model.find("link[@name='camera_tilt_link']/sensor[@name='camera']")
        self.assertEqual(sensor.findtext("topic"), gen_model.CAMERA_TOPIC)
        # The tilt axis runs through the camera, the pan axis vertically below it.
        tilt = vec(self.model.find("link[@name='camera_tilt_link']/pose").text)
        pan = vec(self.model.find("link[@name='camera_pan_link']/pose").text)
        self.assertEqual(tilt, [*P.camera_xyz, 0, 0, 0])
        self.assertEqual(pan[:2], tilt[:2])
        for joint, start in ((gen_model.PAN_JOINT, 0.0), (gen_model.TILT_JOINT, P.camera_pitch)):
            controller = self.model.find(f"plugin[joint_name='{joint}']")
            self.assertEqual(controller.get("name"), "gz::sim::systems::JointPositionController")
            self.assertEqual(controller.findtext("topic"), gen_model.HEAD_TOPIC.format(joint=joint))
            self.assertEqual(controller.findtext("use_velocity_commands"), "true")
            self.assertAlmostEqual(float(controller.findtext("initial_position")), start)
        lower, upper = P.camera_tilt_limits
        self.assertTrue(lower < P.camera_pitch < upper)

    def test_drive_is_tank(self):
        drive = self.plugin("gz::sim::systems::DiffDrive")
        self.assertEqual([e.text for e in drive.findall("left_joint")], ["wheel_fl_joint", "wheel_rl_joint"])
        self.assertEqual([e.text for e in drive.findall("right_joint")], ["wheel_fr_joint", "wheel_rr_joint"])
        self.assertAlmostEqual(float(drive.findtext("wheel_separation")), 2 * P.pivot_y)
        self.assertAlmostEqual(float(drive.findtext("wheel_radius")), P.wheel_radius)
        self.assertEqual(drive.findtext("topic"), gen_model.CMD_VEL_TOPIC)

    def test_differential_couples_the_rockers(self):
        diff = self.plugin("rover_sim::RockerDifferential")
        self.assertEqual(diff.get("filename"), "RockerDifferential")
        self.assertEqual(diff.findtext("left_joint"), "rocker_left_joint")
        self.assertEqual(diff.findtext("right_joint"), "rocker_right_joint")
        self.assertAlmostEqual(float(diff.findtext("stiffness")), P.diff_stiffness)
        self.assertIsNone(self.model.find(".//mimic"))

    def test_tire_friction_axes(self):
        for ode in self.model.iter("ode"):
            self.assertEqual(vec(ode.findtext("fdir1")), [0, 0, 1])  # the axle
            self.assertAlmostEqual(float(ode.findtext("mu")), P.mu_lateral)
            self.assertAlmostEqual(float(ode.findtext("mu2")), P.mu_longitudinal)
        self.assertEqual(len(list(self.model.iter("ode"))), 4)

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Valid", result.stdout)


class ChaseCameraModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = gen_model.build_chase_sdf(gen_model.ChaseParams())
        cls.model = ET.fromstring(cls.sdf).find("model")

    def test_floats_without_collisions(self):
        self.assertEqual(self.model.get("name"), gen_model.CHASE_MODEL)
        self.assertEqual(self.model.findtext("link/gravity"), "false")
        self.assertIsNone(self.model.find(".//collision"))

    def test_camera_and_plugin(self):
        c = gen_model.ChaseParams()
        sensor = self.model.find("link/sensor[@type='camera']")
        self.assertEqual(sensor.findtext("topic"), gen_model.CHASE_IMAGE_TOPIC)
        self.assertEqual((int(sensor.findtext("camera/image/width")), int(sensor.findtext("camera/image/height"))),
                         c.size)
        plugin = self.model.find("plugin[@name='rover_sim::ChaseCamera']")
        self.assertEqual(plugin.get("filename"), "ChaseCamera")
        self.assertEqual(plugin.findtext("target"), "rover")
        self.assertEqual(plugin.findtext("cmd_topic"), gen_model.CHASE_CMD_TOPIC)
        self.assertGreater(float(plugin.findtext("min_pitch")), 0)  # stays above the target

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class EyeCameraModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = gen_model.build_eye_sdf(gen_model.EyeParams(), P)
        cls.model = ET.fromstring(cls.sdf).find("model")
        cls.plugin = cls.model.find("plugin[@name='rover_sim::ChaseCamera']")

    def test_floats_without_collisions(self):
        self.assertEqual(self.model.get("name"), gen_model.EYE_MODEL)
        self.assertEqual(self.model.findtext("link/gravity"), "false")
        self.assertIsNone(self.model.find(".//collision"))

    def test_rides_at_the_camera_pivot(self):
        # Where the tilt axis runs through the RGB-D sensor and the pan axis meets it.
        rover = ET.fromstring(gen_model.build_sdf(P)).find("model")
        tilt = vec(rover.find("link[@name='camera_tilt_link']/pose").text)
        pan = vec(rover.find("link[@name='camera_pan_link']/pose").text)
        mount = vec(self.plugin.findtext("mount"))
        self.assertEqual(mount, tilt[:3])
        self.assertEqual(mount[:2], pan[:2])
        self.assertEqual(self.plugin.findtext("target"), "rover")

    def test_turns_like_the_head(self):
        value = lambda tag: float(self.plugin.findtext(tag))  # noqa: E731
        self.assertEqual((value("yaw"), value("pitch")), (0.0, P.camera_pitch))
        self.assertEqual((value("min_yaw"), value("max_yaw")), (-P.camera_pan_limit, P.camera_pan_limit))
        self.assertEqual((value("min_pitch"), value("max_pitch")), P.camera_tilt_limits)
        self.assertEqual(self.plugin.findtext("look_topic"), gen_model.EYE_LOOK_TOPIC)
        self.assertIsNone(self.plugin.find("cmd_topic"))  # an eye only turns, it does not orbit or zoom

    def test_camera(self):
        e = gen_model.EyeParams()
        sensor = self.model.find("link/sensor[@type='camera']")
        self.assertEqual(sensor.findtext("topic"), gen_model.EYE_IMAGE_TOPIC)
        self.assertEqual((int(sensor.findtext("camera/image/width")), int(sensor.findtext("camera/image/height"))),
                         (960, 540))
        self.assertEqual(float(sensor.findtext("update_rate")), 20.0)
        self.assertEqual(float(sensor.findtext("camera/horizontal_fov")), e.hfov)
        self.assertEqual(e.hfov, P.camera_hfov)

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
