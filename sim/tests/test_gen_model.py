#!/usr/bin/env python3
"""Unit tests for sim/gen_model.py (no physics; pixi run sim-test)."""
import dataclasses
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import numpy as np
from PIL import Image

from worldfiles import MODELS, gz_check, temp_sdf, vec

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import sdf, terrains  # noqa: E402

P = gen_model.Params()
DIFFDRIVE = dataclasses.replace(P, drive=gen_model.DriveParams(mode="diffdrive"))


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

    def test_differential_couples_the_rockers(self):
        diff = self.plugin("rover_sim::RockerDifferential")
        self.assertEqual(diff.get("filename"), "RockerDifferential")
        self.assertEqual(diff.findtext("left_joint"), "rocker_left_joint")
        self.assertEqual(diff.findtext("right_joint"), "rocker_right_joint")
        self.assertAlmostEqual(float(diff.findtext("stiffness")), P.diff_stiffness)
        self.assertIsNone(self.model.find(".//mimic"))

    def test_realism_camera(self):
        """RGB 1280x720 to 80 km, depth clipped at 0.1-40 m (design spec 7, D14; 640x480 read a 20 cm ArUco
        face only to ~2.5 m); no SDF noise, which aborts gz on Metal on an RGB-D camera."""
        camera = self.model.find("link[@name='camera_tilt_link']/sensor[@name='camera']/camera")
        self.assertEqual((int(camera.findtext("image/width")), int(camera.findtext("image/height"))), (1280, 720))
        self.assertEqual(float(camera.findtext("clip/far")), 80_000.0)
        depth = camera.find("depth_camera/clip")
        self.assertEqual((float(depth.findtext("near")), float(depth.findtext("far"))), P.camera_clip)
        self.assertIsNone(camera.find("noise"))

    def test_dust_emitters(self):
        """Behind each rear wheel, on its rocker (a wheel link spins): not emitting
        until the drivetrain says so, an explicit topic; a white diffuse (without
        one the particles render black) and no colour range or scatter ratio,
        which gz-rendering 8 does not apply: the sprite carries the dust's
        colour and opacity (DriveParams.dust_alpha)."""
        for side, s in (("left", "l"), ("right", "r")):
            emitter = self.model.find(f"link[@name='rocker_{side}']/particle_emitter[@name='dust_r{s}']")
            self.assertEqual(emitter.findtext("emitting"), "false")
            for dead in ("particle_scatter_ratio", "color_start", "color_end", "color_range_image"):
                self.assertIsNone(emitter.find(dead), dead)
            self.assertEqual(vec(emitter.findtext("material/diffuse")), [1.0, 1.0, 1.0, 1.0])
            topic = gen_model.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{s}")
            self.assertEqual(emitter.findtext("topic"), topic)
            x, _, z = vec(emitter.findtext("pose"))[:3]
            self.assertLess(x, -(P.wheel_dx + P.wheel_radius))  # behind the rear tyre
            self.assertAlmostEqual(z - (P.wheel_dz - P.wheel_radius), P.drive.dust_box / 2)  # on the ground
            sprite = emitter.findtext("material/pbr/metal/albedo_map")  # the soft puff, tracked with the model
            self.assertEqual(sprite, f"model://rover/{gen_model.DUST_SPRITE}")
            self.assertTrue((MODELS / "rover" / gen_model.DUST_SPRITE).is_file())
        with tempfile.TemporaryDirectory() as tmp:
            gen_model.write_dust_sprite(Path(tmp) / "puff.png")
            written = np.asarray(Image.open(Path(tmp) / "puff.png"))
        np.testing.assert_array_equal(np.asarray(Image.open(MODELS / "rover" / gen_model.DUST_SPRITE)), written)
        self.assertAlmostEqual(written[..., 3].max() / 255, 0.72 * P.drive.dust_alpha, delta=0.02)
        np.testing.assert_array_equal(written[64, 64, :3], np.round(np.array(terrains.DUST_RGB) * 255))

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Valid", result.stdout)


class TrackedModel(unittest.TestCase):
    def test_the_default_rover_is_physical_and_tracked(self):
        """The default rover (DriveParams.mode "physical", design spec D22) is
        the tracked models/rover/model.sdf, byte for byte."""
        self.assertEqual(P.drive.mode, "physical")
        self.assertEqual(gen_model.build_sdf(P), (MODELS / "rover" / "model.sdf").read_text())

    def test_unknown_modes_are_refused(self):
        with self.assertRaises(ValueError):
            gen_model.build_sdf(dataclasses.replace(P, drive=gen_model.DriveParams(mode="servo")))


class DiffDriveVariant(unittest.TestCase):
    """DriveParams(mode="diffdrive"): Gazebo's DiffDrive on anisotropic tyres,
    kept for A/B tests and cost comparisons (design spec D22); everything but
    the drive is the default rover's."""

    @classmethod
    def setUpClass(cls):
        cls.model = ET.fromstring(gen_model.build_sdf(DIFFDRIVE)).find("model")

    def test_drive_is_tank(self):
        drive = self.model.find("plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual([e.text for e in drive.findall("left_joint")], ["wheel_fl_joint", "wheel_rl_joint"])
        self.assertEqual([e.text for e in drive.findall("right_joint")], ["wheel_fr_joint", "wheel_rr_joint"])
        self.assertAlmostEqual(float(drive.findtext("wheel_separation")), 2 * P.pivot_y)
        self.assertAlmostEqual(float(drive.findtext("wheel_radius")), P.wheel_radius)
        self.assertEqual(drive.findtext("topic"), gen_model.CMD_VEL_TOPIC)
        self.assertIsNone(self.model.find("plugin[@filename='RoverDrivetrain']"))  # never both (D22)

    def test_tire_friction_axes(self):
        for ode in self.model.iter("ode"):
            self.assertEqual(vec(ode.findtext("fdir1")), [0, 0, 1])  # the axle
            self.assertAlmostEqual(float(ode.findtext("mu")), P.mu_lateral)
            self.assertAlmostEqual(float(ode.findtext("mu2")), P.mu_longitudinal)
        self.assertEqual(len(list(self.model.iter("ode"))), 4)
        for joint in self.model.findall("joint"):
            if joint.get("name").startswith("wheel_"):
                self.assertEqual(float(joint.findtext("axis/limit/effort")), P.wheel_effort)

    def test_only_the_drive_differs(self):
        """Camera, dust emitters, sensors and links are the default rover's."""
        default = ET.fromstring(gen_model.build_sdf(P)).find("model")
        for path in ("link[@name='camera_tilt_link']/sensor", "link[@name='rocker_left']/particle_emitter",
                     "link[@name='base_link']/sensor[@name='gnss']"):
            self.assertEqual(ET.tostring(self.model.find(path)), ET.tostring(default.find(path)), path)
        self.assertEqual([link.get("name") for link in self.model.findall("link")],
                         [link.get("name") for link in default.findall("link")])


class PhysicalVariant(unittest.TestCase):
    """The default rover's drivetrain, plugins/rover_drivetrain.cpp (design
    spec 6.2)."""

    @classmethod
    def setUpClass(cls):
        cls.params = P
        cls.sdf = gen_model.build_sdf(cls.params)
        cls.model = ET.fromstring(cls.sdf).find("model")
        cls.plugin = cls.model.find("plugin[@name='rover_sim::RoverDrivetrain']")

    def test_drivetrain_replaces_diffdrive(self):
        self.assertIsNone(self.model.find("plugin[@name='gz::sim::systems::DiffDrive']"))
        self.assertEqual(self.plugin.get("filename"), "RoverDrivetrain")
        d = self.params.drive
        expect = {"topic": gen_model.CMD_VEL_TOPIC, "odom_topic": gen_model.ODOM_TOPIC, "tf_topic": gen_model.TF_TOPIC,
                  "state_topic": gen_model.DRIVETRAIN_TOPIC, "frame_id": "odom", "child_frame_id": "base_link",
                  "cmd_timeout_clock": "wall"}
        for tag, value in expect.items():
            self.assertEqual(self.plugin.findtext(tag), value, tag)
        numbers = {"cmd_timeout": 0.5, "track": 2 * P.pivot_y, "radius": P.wheel_radius, "track_multiplier": 1.0,
                   "motor/gear": 50.0, "motor/efficiency": 0.8, "motor/current_limit": 20.0, "motor/voltage": 24.0,
                   "driveline/backlash": d.backlash, "controller/kp": 4.0, "controller/ki": 160.0,
                   "controller/accel": d.accel, "controller/max_speed": P.wheel_speed,
                   "contact/v_stribeck": d.v_stribeck, "contact/stick_perp_ratio": 0.3, "contact/perp_ratio": 0.0,
                   "dust_rule/max_rate": d.dust_max}
        for tag, value in numbers.items():
            self.assertAlmostEqual(float(self.plugin.findtext(tag)), value, msg=tag)
        self.assertEqual(self.plugin.findtext("contact/dig"), "true")
        self.assertEqual(self.plugin.findtext("contact/default_surface"), "regolith")
        self.assertEqual(self.plugin.findtext("contact/object_surface"), "manmade")
        wheels = [(w.findtext("name"), w.findtext("joint"), w.findtext("link"), w.findtext("side"))
                  for w in self.plugin.findall("wheel")]
        self.assertEqual(wheels, [(f"{e}{s}", f"wheel_{e}{s}_joint", f"wheel_{e}{s}", side)
                                  for side, s in (("left", "l"), ("right", "r")) for e in "fr"])

    def test_dig_in_switch(self):
        """Dig-in is on by default and can be switched off; its strength is the
        ground's (terrains.DIG), not scaled by the rover (the plugin's gains
        stay 1)."""
        def contact(dig):
            drive = gen_model.DriveParams(dig=dig)
            model = ET.fromstring(gen_model.build_sdf(dataclasses.replace(P, drive=drive))).find("model")
            return model.find("plugin[@filename='RoverDrivetrain']/contact")

        self.assertEqual(contact(True).findtext("dig"), "true")
        self.assertEqual(contact(False).findtext("dig"), "false")
        self.assertIsNone(self.plugin.find("contact/dig_rate_gain"))
        self.assertIsNone(self.plugin.find("contact/dig_max_gain"))

    def test_surface_rows_come_from_the_catalogue(self):
        """The default and object surfaces' traction under the catalogue's
        dig-in preset (terrains.DIG), as the worlds' ground.json, for worlds
        without a ground map."""
        model = ET.fromstring(gen_model.build_sdf(dataclasses.replace(
            P, drive=gen_model.DriveParams(default_surface="sand")))).find("model")
        for dig in ("strong", "mild"):
            with mock.patch.object(terrains, "DIG", dig):
                rows = {row["key"]: row for row in gen_model.surface_rows("sand", "manmade")}
            self.assertEqual(set(rows), {"sand", "manmade"})
            for key, row in rows.items():
                kind = terrains.TYPES[key]
                self.assertEqual({f.name: row[f.name] for f in dataclasses.fields(terrains.Traction)},
                                 dataclasses.asdict(terrains.traction(kind, dig)), (dig, key))
                self.assertEqual(row["dust"], kind.appearance.dust)
        written = {row.findtext("key"): row
                   for row in model.findall("plugin[@filename='RoverDrivetrain']/contact/surface")}
        self.assertEqual(float(written["sand"].findtext("dig_max")), terrains.STRONG_DIG[1])  # the default, strong

    def test_wheel_joints_and_tyres(self):
        """Effort 1000 N m (DART never clamps the torque), velocity limit 16 rad/s
        (never a hidden brake); isotropic tyre mu, the drivetrain sets every contact."""
        for joint in self.model.findall("joint"):
            if joint.get("name").startswith("wheel_"):
                self.assertEqual(float(joint.findtext("axis/limit/effort")), 1000.0)
                self.assertEqual(float(joint.findtext("axis/limit/velocity")), 16.0)
        for ode in self.model.iter("ode"):
            self.assertEqual((float(ode.findtext("mu")), float(ode.findtext("mu2"))), (1.0, 1.0))
            self.assertIsNone(ode.find("fdir1"))
        self.assertEqual(len(list(self.model.iter("ode"))), 4)

    def test_drivetrain_drives_the_dust(self):
        """The drivetrain commands the rear emitters on their topics."""
        for side, s in (("left", "l"), ("right", "r")):
            topic = gen_model.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{s}")
            self.assertIn((f"wheel_r{s}", topic), [(d.findtext("wheel"), d.findtext("topic"))
                                                   for d in self.plugin.findall("dust")])

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Valid", result.stdout)


class TyreCompliance(unittest.TestCase):
    """Params.tire_compliance (design spec 6.7, phase 2): each wheel hangs from its
    rocker through an axial and a radial sprung hub."""

    @classmethod
    def setUpClass(cls):
        cls.params = dataclasses.replace(P, tire_compliance=True)
        cls.sdf = gen_model.build_sdf(cls.params)
        cls.model = ET.fromstring(cls.sdf).find("model")

    def test_hub_chain(self):
        joints = {j.get("name"): j for j in self.model.findall("joint")}
        for side, s in (("left", "l"), ("right", "r")):
            for e in "fr":
                wheel = f"wheel_{e}{s}"
                axial, radial = joints[f"{wheel}_tire_axial"], joints[f"{wheel}_tire_radial"]
                self.assertEqual((axial.findtext("parent"), axial.findtext("child")),
                                 (f"rocker_{side}", f"{wheel}_hub_axial"))
                self.assertEqual((radial.findtext("parent"), radial.findtext("child")),
                                 (f"{wheel}_hub_axial", f"{wheel}_hub_radial"))
                self.assertEqual(joints[f"{wheel}_joint"].findtext("parent"), f"{wheel}_hub_radial")
                for joint, axis, (stiffness, damping) in ((axial, [0, 1, 0], P.tire_axial),
                                                         (radial, [0, 0, 1], P.tire_radial)):
                    self.assertEqual(joint.get("type"), "prismatic")
                    self.assertEqual(vec(joint.findtext("axis/xyz")), axis)
                    self.assertEqual(float(joint.findtext("axis/dynamics/spring_stiffness")), stiffness)
                    self.assertEqual(float(joint.findtext("axis/dynamics/damping")), damping)
                    self.assertEqual(float(joint.findtext("axis/limit/upper")), P.tire_travel)

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


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
