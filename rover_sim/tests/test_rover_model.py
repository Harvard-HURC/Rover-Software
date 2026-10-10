#!/usr/bin/env python3
"""Unit tests for tools/gen_rover.py, the rover's Gazebo model built from rover_description and config/sim.yaml
(no physics; pixi run sim-test). test_equivalence checks that it is the rover gen_model.py wrote."""
import copy
import dataclasses
import importlib.util
import json
import re
import shutil
import struct
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import numpy as np
import xacro
from PIL import Image

from worldfiles import MODELS, gz_check, temp_sdf, vec

import gen_rover  # noqa: E402  (worldfiles puts sim/ and sim/tools on the path)
import rover_config  # noqa: E402
import viewers  # noqa: E402
from rover_config import ROVER, SIM, Variant  # noqa: E402
from urc import sdf, terrains  # noqa: E402

V = Variant()
DIFFDRIVE = V.replace(drivetrain={"mode": "diffdrive"})
DUST = V.replace(dust={"enabled": True})  # the opt-in dust (sim.yaml dust)
DUST_PLUGIN_TAGS = ("dust_rate", "dust", "dust_rule")  # the drivetrain's dust elements
# The tyre's dig-in cues off (sim.yaml tread_tyre, dig_sink): the rover before them, but for the ruts (their own
# switch, sim.yaml ruts).
CUES_OFF = V.replace(tread_tyre=False, dig_sink={"enabled": False})
NO_RUTS = V.replace(ruts={"enabled": False})  # sim.yaml ruts off
WHEELS = [f"wheel_{e}{s}" for _, s, _ in rover_config.SIDES for _, e, _ in rover_config.ENDS]
SIM_DIR = rover_config.SIM_DIR
INERTIALS = rover_config.DESCRIPTION_DIR / "urdf" / "inertials.xacro"
WHEEL_MESH_GENERATOR = rover_config.DESCRIPTION_DIR / "meshes" / "gen_wheel_mesh.py"


def canonical(element):
    """An element's XML without its indentation, to compare models built differently."""
    return ET.canonicalize(ET.tostring(element), strip_text=True)


def inertia(macro, **params):
    """The principal moments (ixx, iyy, izz) that a macro of rover_description's urdf/inertials.xacro gives for
    `params` (xacro expressions as text)."""
    attributes = " ".join(f'{key}="{value}"' for key, value in params.items())
    with tempfile.TemporaryDirectory() as tmp:
        document = Path(tmp) / "inertia.xacro"
        document.write_text(f'<robot name="t" xmlns:xacro="http://www.ros.org/wiki/xacro">'
                            f'<xacro:include filename="{INERTIALS}"/>'
                            f'<link name="l"><xacro:{macro} {attributes}/></link></robot>')
        element = ET.fromstring(xacro.process_file(str(document)).toxml()).find("link/inertial/inertia")
    return tuple(float(element.get(key)) for key in ("ixx", "iyy", "izz"))


def wheel_mesh_generator():
    """rover_description/meshes/gen_wheel_mesh.py, imported from its file."""
    spec = importlib.util.spec_from_file_location("gen_wheel_mesh", WHEEL_MESH_GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Inertia(unittest.TestCase):
    def test_box(self):
        self.assertEqual(sdf.box_inertia(12.0, (1.0, 2.0, 3.0)), (13.0, 10.0, 5.0))
        self.assertEqual(inertia("box_inertial", mass="12.0", size="${[1.0, 2.0, 3.0]}"), (13.0, 10.0, 5.0))

    def test_wheel_axis_is_y(self):
        ixx, iyy, izz = inertia("wheel_inertial", mass="2.0", radius="0.5", width="1.0")
        self.assertAlmostEqual(iyy, 2.0 * 0.5**2 / 2)
        self.assertAlmostEqual(ixx, 2.0 * (3 * 0.5**2 + 1.0**2) / 12)
        self.assertEqual(ixx, izz)

    def test_flat_rocker_is_one_rod(self):
        # Rods to (+-1, 0, 0) form one rod of length 2 about its middle.
        self.assertEqual(inertia("rocker_inertial", mass="2.0", dx="1.0", dz="0.0", xyz="0 0 0"),
                         (0.0, 2.0 * 4 / 12, 2.0 * 4 / 12))

    def test_vertical_rocker_is_one_rod(self):
        # Both rods straight down to (0, 0, -1) overlap: one rod of length 1.
        ixx, iyy, izz = inertia("rocker_inertial", mass="2.0", dx="0.0", dz="-1.0", xyz="0 0 -0.5")
        self.assertAlmostEqual(ixx, 2.0 / 12)
        self.assertAlmostEqual(iyy, 2.0 / 12)
        self.assertEqual(izz, 0.0)


class Structure(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = gen_rover.build_sdf(V)
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
            axis = [0, 0, 1] if joint.get("name") == rover_config.PAN_JOINT else [0, 1, 0]
            self.assertEqual(vec(joint.findtext("axis/xyz")), axis, joint.get("name"))

    def test_wheel_centers_follow_params(self):
        """The wheel centres of rover.yaml: rocker pivots plus the wheel offsets."""
        rocker, wheel = ROVER.rocker, ROVER.wheel
        z = rocker.pivot_z + wheel.dz
        expected = {"wheel_fl": (wheel.dx, rocker.pivot_y), "wheel_rl": (-wheel.dx, rocker.pivot_y),
                    "wheel_fr": (wheel.dx, -rocker.pivot_y), "wheel_rr": (-wheel.dx, -rocker.pivot_y)}
        for name, (x, y) in expected.items():
            pose = vec(self.model.find(f"link[@name='{name}']/pose").text)
            for got, want in zip(pose, (x, y, z, 0, 0, 0)):
                self.assertAlmostEqual(got, want, msg=name)

    def test_wheels_rest_on_the_ground(self):
        self.assertAlmostEqual(ROVER.rocker.pivot_z + ROVER.wheel.dz, ROVER.wheel.radius)

    def test_total_mass(self):
        total = sum(float(m.text) for m in self.model.iter("mass"))
        self.assertAlmostEqual(total, ROVER.chassis.mass + 2 * ROVER.rocker.mass + 4 * ROVER.wheel.mass
                               + ROVER.head.pan_mass + ROVER.head.tilt_mass)

    def test_camera_on_the_pan_tilt_head(self):
        sensor = self.model.find("link[@name='camera_tilt_link']/sensor[@name='camera']")
        self.assertEqual(sensor.findtext("topic"), rover_config.CAMERA_TOPIC)
        # The tilt axis runs through the camera, the pan axis vertically below it.
        tilt = vec(self.model.find("link[@name='camera_tilt_link']/pose").text)
        pan = vec(self.model.find("link[@name='camera_pan_link']/pose").text)
        self.assertEqual(tilt, [*ROVER.camera.xyz, 0, 0, 0])
        self.assertEqual(pan[:2], tilt[:2])
        for joint, start in ((rover_config.PAN_JOINT, 0.0), (rover_config.TILT_JOINT, ROVER.head.home_tilt)):
            controller = self.model.find(f"plugin[joint_name='{joint}']")
            self.assertEqual(controller.get("name"), "gz::sim::systems::JointPositionController")
            self.assertEqual(controller.findtext("topic"), rover_config.HEAD_TOPIC.format(joint=joint))
            self.assertEqual(controller.findtext("use_velocity_commands"), "true")
            self.assertAlmostEqual(float(controller.findtext("initial_position")), start)
        lower, upper = ROVER.head.tilt_limits
        self.assertTrue(lower < ROVER.head.home_tilt < upper)

    def test_differential_couples_the_rockers(self):
        diff = self.plugin("rover_sim::RockerDifferential")
        self.assertEqual(diff.get("filename"), "RockerDifferential")
        self.assertEqual(diff.findtext("left_joint"), "rocker_left_joint")
        self.assertEqual(diff.findtext("right_joint"), "rocker_right_joint")
        self.assertAlmostEqual(float(diff.findtext("stiffness")), ROVER.differential.stiffness)
        self.assertIsNone(self.model.find(".//mimic"))

    def test_realism_camera(self):
        """RGB 1280x720 to 80 km, depth clipped at 0.1-40 m (design spec 7, D14; 640x480 read a 20 cm ArUco
        face only to ~2.5 m); no SDF noise, which aborts gz on Metal on an RGB-D camera."""
        camera = self.model.find("link[@name='camera_tilt_link']/sensor[@name='camera']/camera")
        self.assertEqual((int(camera.findtext("image/width")), int(camera.findtext("image/height"))), (1280, 720))
        self.assertEqual(float(camera.findtext("clip/far")), 80_000.0)
        depth = camera.find("depth_camera/clip")
        self.assertEqual((float(depth.findtext("near")), float(depth.findtext("far"))), ROVER.camera.clip)
        self.assertIsNone(camera.find("noise"))

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Valid", result.stdout)


class TrackedModel(unittest.TestCase):
    def test_the_default_rover_is_physical_and_tracked(self):
        """The default rover (sim.yaml drivetrain mode physical, design spec D22; no dust, the user's decision of
        2026-10-07) is the tracked models/rover/model.sdf, byte for byte."""
        self.assertEqual(SIM.drivetrain.mode, "physical")
        self.assertFalse(SIM.dust.enabled)
        self.assertEqual(gen_rover.build_sdf(V), (MODELS / "rover" / "model.sdf").read_text())

    def test_unknown_modes_are_refused(self):
        with self.assertRaises(ValueError):
            gen_rover.build_sdf(V.replace(drivetrain={"mode": "servo"}))

    def test_the_surfaces_are_the_catalogues(self):
        """The tracked config/surfaces.yaml is what gen_rover.py writes from urc/terrains.py."""
        self.assertEqual((SIM_DIR / "config" / "surfaces.yaml").read_text(), gen_rover.surfaces_text())

    def test_the_tread_mesh_is_the_descriptions(self):
        """The rover model's tread tyre is a copy of rover_description's (main() copies it)."""
        self.assertEqual((MODELS / "rover" / rover_config.WHEEL_MESH).read_bytes(),
                         gen_rover.DESCRIPTION_MESH.read_bytes())


class DiffDriveVariant(unittest.TestCase):
    """sim.yaml drivetrain mode diffdrive: Gazebo's DiffDrive on anisotropic tyres, kept for A/B tests and cost
    comparisons (design spec D22); everything but the drive is the default rover's."""

    @classmethod
    def setUpClass(cls):
        cls.model = ET.fromstring(gen_rover.build_sdf(DIFFDRIVE)).find("model")

    def test_drive_is_tank(self):
        drive = self.model.find("plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual([e.text for e in drive.findall("left_joint")], ["wheel_fl_joint", "wheel_rl_joint"])
        self.assertEqual([e.text for e in drive.findall("right_joint")], ["wheel_fr_joint", "wheel_rr_joint"])
        self.assertAlmostEqual(float(drive.findtext("wheel_separation")), 2 * ROVER.rocker.pivot_y)
        self.assertAlmostEqual(float(drive.findtext("wheel_radius")), ROVER.wheel.radius)
        self.assertEqual(drive.findtext("topic"), rover_config.CMD_VEL_TOPIC)
        self.assertIsNone(self.model.find("plugin[@filename='RoverDrivetrain']"))  # never both (D22)

    def test_tire_friction_axes(self):
        for ode in self.model.iter("ode"):
            self.assertEqual(vec(ode.findtext("fdir1")), [0, 0, 1])  # the axle
            self.assertAlmostEqual(float(ode.findtext("mu")), SIM.diffdrive.mu_lateral)
            self.assertAlmostEqual(float(ode.findtext("mu2")), SIM.diffdrive.mu_longitudinal)
        self.assertEqual(len(list(self.model.iter("ode"))), 4)
        for joint in self.model.findall("joint"):
            if joint.get("name").startswith("wheel_"):
                self.assertEqual(float(joint.findtext("axis/limit/effort")), SIM.diffdrive.wheel_effort)

    def test_only_the_drive_differs(self):
        """Camera, sensors and links are the default rover's, and so are the
        dust emitters: none by default, the same two with sim.yaml's dust on."""
        default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
        for path in ("link[@name='camera_tilt_link']/sensor", "link[@name='base_link']/sensor[@name='gnss']"):
            self.assertEqual(ET.tostring(self.model.find(path)), ET.tostring(default.find(path)), path)
        self.assertEqual([link.get("name") for link in self.model.findall("link")],
                         [link.get("name") for link in default.findall("link")])

        def emitters(variant):
            return [ET.tostring(e) for e in ET.fromstring(gen_rover.build_sdf(variant)).iter("particle_emitter")]

        self.assertEqual(emitters(DIFFDRIVE), [])
        dusty = emitters(DIFFDRIVE.replace(dust={"enabled": True}))
        self.assertEqual(len(dusty), 2)
        self.assertEqual(dusty, emitters(DUST))


class PhysicalVariant(unittest.TestCase):
    """The default rover's drivetrain, plugins/rover_drivetrain.cpp (design
    spec 6.2)."""

    @classmethod
    def setUpClass(cls):
        cls.sdf = gen_rover.build_sdf(V)
        cls.model = ET.fromstring(cls.sdf).find("model")
        cls.plugin = cls.model.find("plugin[@name='rover_sim::RoverDrivetrain']")

    def test_drivetrain_replaces_diffdrive(self):
        self.assertIsNone(self.model.find("plugin[@name='gz::sim::systems::DiffDrive']"))
        self.assertEqual(self.plugin.get("filename"), "RoverDrivetrain")
        expect = {"setpoint_topic": rover_config.WHEEL_CMD_TOPIC, "odom_topic": rover_config.ODOM_TOPIC,
                  "tf_topic": rover_config.TF_TOPIC, "state_topic": rover_config.DRIVETRAIN_TOPIC,
                  "frame_id": "odom", "child_frame_id": "base_link", "cmd_timeout_clock": "wall"}
        for tag, value in expect.items():
            self.assertEqual(self.plugin.findtext(tag), value, tag)
        numbers = {"cmd_timeout": 0.5, "track": 2 * ROVER.rocker.pivot_y, "radius": ROVER.wheel.radius,
                   "motor/gear": 50.0, "motor/efficiency": 0.8,
                   "motor/current_limit": 20.0, "motor/voltage": 48.0, "driveline/backlash": ROVER.driveline.backlash,
                   "controller/kp": 4.0, "controller/ki": 160.0,
                   "controller/max_speed": ROVER.wheel.top_speed, "contact/v_stribeck": SIM.contact.v_stribeck,
                   "contact/stick_perp_ratio": 0.3, "contact/perp_ratio": 0.0}
        for tag, value in numbers.items():
            self.assertAlmostEqual(float(self.plugin.findtext(tag)), value, msg=tag)
        for gone in ("topic", "track_multiplier", "controller/accel"):  # the drive controller's since PR 3
            self.assertIsNone(self.plugin.find(gone), gone)
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
            model = ET.fromstring(gen_rover.build_sdf(V.replace(contact={"dig": dig}))).find("model")
            return model.find("plugin[@filename='RoverDrivetrain']/contact")

        self.assertEqual(contact(True).findtext("dig"), "true")
        self.assertEqual(contact(False).findtext("dig"), "false")
        self.assertIsNone(self.plugin.find("contact/dig_rate_gain"))
        self.assertIsNone(self.plugin.find("contact/dig_max_gain"))

    def test_surface_rows_come_from_the_catalogue(self):
        """Every catalogue type's traction under the catalogue's dig-in
        preset (terrains.DIG), as the worlds' ground.json, and the model's
        rows for its default and object surfaces, for worlds without a ground
        map."""
        model = ET.fromstring(gen_rover.build_sdf(V.replace(contact={"default_surface": "sand"}))).find("model")
        for dig in ("strong", "mild"):
            with mock.patch.object(terrains, "DIG", dig):
                rows = gen_rover.surface_rows()
            self.assertEqual(set(rows), set(terrains.TYPES))
            for key, row in rows.items():
                kind = terrains.TYPES[key]
                self.assertEqual({f.name: row[f.name] for f in dataclasses.fields(terrains.Traction)},
                                 dataclasses.asdict(terrains.traction(kind, dig)), (dig, key))
                self.assertEqual(row["dust"], kind.appearance.dust)
        written = {row.findtext("key"): row
                   for row in model.findall("plugin[@filename='RoverDrivetrain']/contact/surface")}
        self.assertEqual(set(written), {"sand", "manmade"})
        self.assertEqual(float(written["sand"].findtext("dig_max")), terrains.STRONG_DIG[1])  # the default, strong

    def test_wheel_joints_and_tyres(self):
        """Effort 1000 N m (DART never clamps the torque), velocity limit 32 rad/s
        (never a hidden brake); isotropic tyre mu, the drivetrain sets every contact."""
        for joint in self.model.findall("joint"):
            if joint.get("name").startswith("wheel_"):
                self.assertEqual(float(joint.findtext("axis/limit/effort")), 1000.0)
                self.assertEqual(float(joint.findtext("axis/limit/velocity")), 32.0)
        for ode in self.model.iter("ode"):
            self.assertEqual((float(ode.findtext("mu")), float(ode.findtext("mu2"))), (1.0, 1.0))
            self.assertIsNone(ode.find("fdir1"))
        self.assertEqual(len(list(self.model.iter("ode"))), 4)

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Valid", result.stdout)


class Dust(unittest.TestCase):
    """sim.yaml dust, the one switch for the rover's wheel dust (design spec
    6.5, D15). Off by default (the user's decision of 2026-10-07): in
    gz-rendering 8.2.2 the depth image and point cloud see any visible
    particle (tests/test_render.py), so Q11 (depth does not see dust) is met
    by having no dust. On: the emitters, asking for a scatter ratio near
    none, and the drivetrain's dust elements, for when gz-rendering honours
    that ratio."""

    @classmethod
    def setUpClass(cls):
        cls.default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
        cls.tracked = ET.parse(MODELS / "rover" / "model.sdf").getroot().find("model")
        cls.sdf = gen_rover.build_sdf(DUST)
        cls.model = ET.fromstring(cls.sdf).find("model")
        cls.plugin = cls.model.find("plugin[@name='rover_sim::RoverDrivetrain']")

    def test_off_by_default(self):
        """The default rover, generated and tracked (model://rover), has no
        particle emitter in either drive mode, so no camera, depth image or
        point cloud can see dust; its drivetrain drives none (no dust
        elements), while the same rover with the switch on has both."""
        self.assertFalse(SIM.dust.enabled)
        diffdrive = ET.fromstring(gen_rover.build_sdf(DIFFDRIVE)).find("model")
        for name, model in (("generated", self.default), ("tracked", self.tracked), ("diffdrive", diffdrive)):
            self.assertEqual(list(model.iter("particle_emitter")), [], name)
        for name, model in (("generated", self.default), ("tracked", self.tracked)):
            plugin = model.find("plugin[@name='rover_sim::RoverDrivetrain']")
            self.assertIsNotNone(plugin, name)
            for tag in DUST_PLUGIN_TAGS:
                self.assertEqual(plugin.findall(tag), [], (name, tag))
        self.assertEqual(len(list(self.model.iter("particle_emitter"))), 2)
        for tag in DUST_PLUGIN_TAGS:
            self.assertTrue(self.plugin.findall(tag), tag)

    def test_switch_adds_only_the_dust(self):
        """With the switch on the model is the default one plus the two
        emitters and the drivetrain's dust elements, nothing else."""
        model = ET.fromstring(self.sdf).find("model")  # a copy to strip
        plugin = model.find("plugin[@name='rover_sim::RoverDrivetrain']")
        dust = [(link, e) for link in model.findall("link") for e in link.findall("particle_emitter")]
        dust += [(plugin, e) for tag in DUST_PLUGIN_TAGS for e in plugin.findall(tag)]
        self.assertEqual(len(dust), 2 + 1 + 2 + 1)  # emitters, dust_rate, <dust> per rear wheel, <dust_rule>
        for parent, element in dust:
            parent.remove(element)
        self.assertEqual(canonical(model), canonical(self.default))

    def test_dust_emitters(self):
        """Behind each rear wheel, on its rocker (a wheel link spins): not emitting
        until the drivetrain says so, an explicit topic; a white diffuse (without
        one the particles render black) and no colour range, which gz-rendering 8
        does not apply: the sprite carries the dust's colour and opacity
        (sim.yaml dust alpha). A scatter ratio near none but above 0: without
        one, or at 0, a gz-rendering that honours it applies 0.65 and depth sees
        the dust (8.2.2 honours none, tests/test_render.py). The sprite is
        written with the model whether or not the switch is on."""
        wheel = ROVER.wheel
        for side, s in (("left", "l"), ("right", "r")):
            emitter = self.model.find(f"link[@name='rocker_{side}']/particle_emitter[@name='dust_r{s}']")
            self.assertEqual(emitter.findtext("emitting"), "false")
            for dead in ("color_start", "color_end", "color_range_image"):
                self.assertIsNone(emitter.find(dead), dead)
            ratio = emitter.findtext("particle_scatter_ratio")
            self.assertIsNotNone(ratio, "particle_scatter_ratio")  # without one: SDF's 0.65
            # An SDF float: the converter writes its float32 value, which Gazebo reads as it read 1e-06.
            self.assertEqual(np.float32(ratio), np.float32(SIM.dust.scatter_ratio))
            self.assertTrue(0 < float(ratio) <= 1e-3, ratio)
            self.assertEqual(vec(emitter.findtext("material/diffuse")), [1.0, 1.0, 1.0, 1.0])
            topic = rover_config.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{s}")
            self.assertEqual(emitter.findtext("topic"), topic)
            x, _, z = vec(emitter.findtext("pose"))[:3]
            self.assertLess(x, -(wheel.dx + wheel.radius))  # behind the rear tyre
            self.assertAlmostEqual(z - (wheel.dz - wheel.radius), SIM.dust.box / 2)  # on the ground
            sprite = emitter.findtext("material/pbr/metal/albedo_map")  # the soft puff, tracked with the model
            self.assertEqual(sprite, f"model://rover/{rover_config.DUST_SPRITE}")
            self.assertTrue((MODELS / "rover" / rover_config.DUST_SPRITE).is_file())
        with tempfile.TemporaryDirectory() as tmp:
            gen_rover.write_dust_sprite(Path(tmp) / "puff.png")
            written = np.asarray(Image.open(Path(tmp) / "puff.png"))
        np.testing.assert_array_equal(np.asarray(Image.open(MODELS / "rover" / rover_config.DUST_SPRITE)), written)
        self.assertAlmostEqual(written[..., 3].max() / 255, 0.72 * SIM.dust.alpha, delta=0.02)
        np.testing.assert_array_equal(written[64, 64, :3], np.round(np.array(terrains.DUST_RGB) * 255))

    def test_drivetrain_drives_the_dust(self):
        """The drivetrain commands the rear emitters on their topics, at
        the dust rate, by the dust rule (sim.yaml's gains)."""
        d = DUST.sim.dust
        for side, s in (("left", "l"), ("right", "r")):
            topic = rover_config.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{s}")
            self.assertIn((f"wheel_r{s}", topic), [(e.findtext("wheel"), e.findtext("topic"))
                                                   for e in self.plugin.findall("dust")])
        numbers = {"dust_rate": d.rate, "dust_rule/speed_gain": d.rule.speed_gain,
                   "dust_rule/slip_gain": d.rule.slip_gain, "dust_rule/max_rate": d.rule.max_rate,
                   "dust_rule/min_speed": d.rule.min_speed}
        for tag, value in numbers.items():
            self.assertAlmostEqual(float(self.plugin.findtext(tag)), value, msg=tag)

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Valid", result.stdout)


class DigCues(unittest.TestCase):
    """The dig-in made visible (the user's decisions of 2026-10-07), each
    cue visual only and behind its own switch, both on by default:
    sim.yaml dig_sink, the drivetrain drawing a dug wheel's tyre sunk at
    true scale (<dig_sink>), and sim.yaml tread_tyre, the tyres drawn with
    the tread mesh (rover_config.WHEEL_MESH) instead of the plain cylinder."""

    @classmethod
    def setUpClass(cls):
        cls.default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
        cls.tracked = ET.parse(MODELS / "rover" / "model.sdf").getroot().find("model")

    @staticmethod
    def without(model, tyres=False, sink=False):
        """A copy of a model without its wheels' visuals and/or the drivetrain's <dig_sink>."""
        model = copy.deepcopy(model)
        for name in WHEELS if tyres else ():
            link = model.find(f"link[@name='{name}']")
            for visual in link.findall("visual"):
                link.remove(visual)
        plugin = model.find("plugin[@name='rover_sim::RoverDrivetrain']")
        for element in plugin.findall("dig_sink") if sink else ():
            plugin.remove(element)
        return model

    def test_on_by_default(self):
        """The default rover, generated and tracked (model://rover): each
        wheel's one visual is the tread mesh scaled to the wheel, with no SDF
        <material> (it would replace the mesh's own), while its collision is
        still the cylinder; the drivetrain gets <dig_sink> at true scale."""
        wheel, sink_params = ROVER.wheel, SIM.dig_sink
        self.assertTrue(SIM.tread_tyre)
        self.assertTrue(sink_params.enabled)
        self.assertEqual(sink_params.gain, 1.0)  # true scale, not the research's x3 (user decision)
        for label, model in (("generated", self.default), ("tracked", self.tracked)):
            for name in WHEELS:
                link = model.find(f"link[@name='{name}']")
                visuals = link.findall("visual")
                self.assertEqual([v.get("name") for v in visuals], ["tire_visual"], (label, name))
                mesh = visuals[0].find("geometry/mesh")
                self.assertEqual(mesh.findtext("uri"), f"model://rover/{rover_config.WHEEL_MESH}")
                self.assertEqual(vec(mesh.findtext("scale")), [wheel.radius, wheel.width, wheel.radius])
                self.assertEqual(vec(visuals[0].findtext("pose")), [0.0] * 6)
                self.assertIsNone(visuals[0].find("material"), (label, name))
                cylinder = link.find("collision/geometry/cylinder")
                self.assertEqual((float(cylinder.findtext("radius")), float(cylinder.findtext("length"))),
                                 (wheel.radius, wheel.width))
            sink = model.find("plugin[@name='rover_sim::RoverDrivetrain']/dig_sink")
            self.assertIsNotNone(sink, label)
            self.assertEqual({e.tag: float(e.text) for e in sink},
                             {"gain": sink_params.gain, "max": sink_params.max, "tau": sink_params.tau})
        self.assertTrue((MODELS / "rover" / rover_config.WHEEL_MESH).is_file())

    def test_each_switch_takes_away_only_its_cue(self):
        """tread_tyre off gives back the plain tyre, a black visual of the
        collision's own cylinder; dig_sink off takes away <dig_sink>; nothing
        else changes, in either drive mode. With both off, and the ruts
        (sim.yaml ruts, Ruts) too, the rover is the one before the cues
        (byte for byte the model.sdf of b124f7d, checked when they were built
        and again when they were merged, 2026-10-07)."""
        plain = ET.fromstring(gen_rover.build_sdf(V.replace(tread_tyre=False))).find("model")
        no_sink = ET.fromstring(gen_rover.build_sdf(V.replace(dig_sink={"enabled": False}))).find("model")
        before = ET.fromstring(gen_rover.build_sdf(CUES_OFF)).find("model")
        for name in WHEELS:
            link = plain.find(f"link[@name='{name}']")
            visual, collision = link.find("visual"), link.find("collision")
            self.assertEqual(ET.tostring(visual.find("geometry")), ET.tostring(collision.find("geometry")))
            self.assertEqual(visual.findtext("pose"), collision.findtext("pose"))
            self.assertEqual(vec(visual.findtext("material/diffuse")), list(ROVER.looks.colors.tyre))
        self.assertIsNone(no_sink.find("plugin[@name='rover_sim::RoverDrivetrain']/dig_sink"))
        self.assertIsNone(before.find("plugin[@name='rover_sim::RoverDrivetrain']/dig_sink"))
        self.assertEqual(canonical(self.without(plain, tyres=True)), canonical(self.without(self.default, tyres=True)))
        self.assertEqual(canonical(no_sink), canonical(self.without(self.default, sink=True)))
        self.assertEqual(canonical(self.without(before, tyres=True)),
                         canonical(self.without(self.default, tyres=True, sink=True)))
        for name in WHEELS:
            self.assertEqual(canonical(before.find(f"link[@name='{name}']")), canonical(plain.find(f"link[@name='{name}']")))
        for variant in (DIFFDRIVE, CUES_OFF):
            model = ET.fromstring(gen_rover.build_sdf(variant.replace(drivetrain={"mode": "diffdrive"}))).find("model")
            mesh = model.find("link[@name='wheel_fl']/visual/geometry/mesh")
            self.assertEqual(mesh is not None, variant.sim.tread_tyre)  # the look is the rover's, not the drive's

    def test_tread_mesh(self):
        """The rover model's tread tyre (rover_config.WHEEL_MESH, a copy of
        rover_description's meshes/wheel.glb) is what gen_wheel_mesh.py
        writes, byte for byte: a cylinder of radius 1 and width 1 round the
        axle (y), the same outline as the plain tyre's, in three materials,
        the plain tyre's dark, light (16 tread bars, hub, spokes) and the
        chassis' ochre, which is one spoke on each face at angle 0 and
        nowhere else. The GLB's materials are those colours."""
        tracked = MODELS / "rover" / rover_config.WHEEL_MESH
        generator = wheel_mesh_generator()
        dark, light, mark = generator.colors()
        with tempfile.TemporaryDirectory() as tmp:
            generator.write_glb(Path(tmp) / "wheel.glb", generator.wheel_parts(dark, light, mark))
            self.assertEqual((Path(tmp) / "wheel.glb").read_bytes(), tracked.read_bytes())
        blob = tracked.read_bytes()
        length, kind = struct.unpack("<II", blob[12:20])
        self.assertEqual(kind, 0x4E4F534A)  # "JSON"
        gltf = json.loads(blob[20:20 + length])
        colours = [tuple(m["pbrMetallicRoughness"]["baseColorFactor"]) for m in gltf["materials"]]
        np.testing.assert_allclose(colours, [dark + (1.0,), light + (1.0,), mark + (1.0,)])
        self.assertEqual(dark, ROVER.looks.colors.tyre[:3])
        self.assertEqual(mark, ROVER.looks.colors.chassis[:3])
        parts = generator.wheel_parts(dark, light, mark)
        V = np.concatenate([part[0] for part in parts])
        np.testing.assert_allclose(np.hypot(V[:, 0], V[:, 2]).max(), 1.0)
        self.assertEqual((V[:, 1].min(), V[:, 1].max()), (-0.5, 0.5))
        (_, dark_n), (light_v, light_n), (mark_v, mark_n) = ((part[0], part[2]) for part in parts)
        self.assertEqual(int(np.sum(np.abs(light_n[::3, 1]) < 1e-9)), 2 * 16)  # the bars: 16, two triangles each
        self.assertEqual(len(mark_v), 3 * 2 * 2 * 2)  # one spoke: two segments, two triangles each, on both faces
        self.assertEqual(set(np.sign(mark_v[:, 1])), {-1.0, 1.0})
        np.testing.assert_allclose(np.abs(mark_n[:, 1]), 1.0)  # on the faces, not the tread
        rim = mark_v[np.hypot(mark_v[:, 0], mark_v[:, 2]) > 0.99]
        self.assertTrue(np.all(np.abs(np.degrees(np.arctan2(rim[:, 2], rim[:, 0]))) <= 7.5 + 1e-9))
        self.assertTrue(len(dark_n))


class Ruts(unittest.TestCase):
    """sim.yaml ruts, the one switch for the ruts and pits behind the wheels
    (its tracks numbers, plugins/rover_tracks.hh): on by default (the
    user's decision of 2026-10-07), written as the drivetrain's <tracks>
    group; off, the model is the default one without it."""

    @classmethod
    def setUpClass(cls):
        cls.default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
        cls.tracked = ET.parse(MODELS / "rover" / "model.sdf").getroot().find("model")
        cls.off = ET.fromstring(gen_rover.build_sdf(NO_RUTS)).find("model")

    def test_on_by_default(self):
        """The default rover, generated and tracked (model://rover), has one
        <tracks> in its drivetrain with every number of sim.yaml's ruts tracks
        and the wheel's width; DiffDrive has no drivetrain and no ruts."""
        self.assertTrue(SIM.ruts.enabled)
        expected = {**SIM.ruts.tracks, "width": ROVER.wheel.width}
        for name, model in (("generated", self.default), ("tracked", self.tracked)):
            groups = model.findall("plugin[@name='rover_sim::RoverDrivetrain']/tracks")
            self.assertEqual(len(groups), 1, name)
            written = {e.tag: float(e.text) for e in groups[0]}
            self.assertEqual(written, {k: float(v) for k, v in expected.items()}, name)
        diffdrive = ET.fromstring(gen_rover.build_sdf(DIFFDRIVE)).find("model")
        self.assertEqual(list(diffdrive.iter("tracks")), [])

    def test_switch_removes_only_the_tracks(self):
        """With the switch off the model is the default one without <tracks>,
        nothing else: the plugin then lays, reads and draws nothing."""
        self.assertEqual(list(self.off.iter("tracks")), [])
        model = ET.fromstring(gen_rover.build_sdf(V)).find("model")  # a copy to strip
        plugin = model.find("plugin[@name='rover_sim::RoverDrivetrain']")
        plugin.remove(plugin.find("tracks"))
        self.assertEqual(canonical(model), canonical(self.off))

    def test_plugin_reads_every_element(self):
        """Every element of <tracks> is one the plugin reads
        (rover_tracks_render.hh ReadParams), and the defaults agree: a
        misspelt parameter would silently keep the C++ default."""
        source = (SIM_DIR / "plugins" / "rover_tracks_render.hh").read_text()
        header = (SIM_DIR / "plugins" / "rover_tracks.hh").read_text()
        for name, value in {**SIM.ruts.tracks, "width": ROVER.wheel.width}.items():
            self.assertIn(f'get("{name}", p.{name})', source, name)
            default = re.search(rf"\b(?:double|int) {name} = ([-\d.e]+);", header)
            self.assertIsNotNone(default, name)
            self.assertAlmostEqual(float(default.group(1)), float(value), msg=name)

    def test_gz_accepts_it(self):
        with temp_sdf(gen_rover.build_sdf(V)) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Valid", result.stdout)


class TyreCompliance(unittest.TestCase):
    """sim.yaml tyre_compliance (design spec 6.7, phase 2): each wheel hangs from its
    rocker through an axial and a radial sprung hub, with rover.yaml's tyre."""

    @classmethod
    def setUpClass(cls):
        cls.sdf = gen_rover.build_sdf(V.replace(tyre_compliance={"enabled": True}))
        cls.model = ET.fromstring(cls.sdf).find("model")

    def test_hub_chain(self):
        tyre = ROVER.tyre
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
                for joint, axis, spring in ((axial, [0, 1, 0], tyre.axial), (radial, [0, 0, 1], tyre.radial)):
                    self.assertEqual(joint.get("type"), "prismatic")
                    self.assertEqual(vec(joint.findtext("axis/xyz")), axis)
                    self.assertEqual(float(joint.findtext("axis/dynamics/spring_stiffness")), spring.stiffness)
                    self.assertEqual(float(joint.findtext("axis/dynamics/damping")), spring.damping)
                    self.assertEqual(float(joint.findtext("axis/limit/upper")), tyre.travel)

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class ChaseCameraModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = viewers.build_chase_sdf(viewers.ChaseParams())
        cls.model = ET.fromstring(cls.sdf).find("model")

    def test_floats_without_collisions(self):
        self.assertEqual(self.model.get("name"), viewers.CHASE_MODEL)
        self.assertEqual(self.model.findtext("link/gravity"), "false")
        self.assertIsNone(self.model.find(".//collision"))

    def test_camera_and_plugin(self):
        c = viewers.ChaseParams()
        sensor = self.model.find("link/sensor[@type='camera']")
        self.assertEqual(sensor.findtext("topic"), viewers.CHASE_IMAGE_TOPIC)
        self.assertEqual((int(sensor.findtext("camera/image/width")), int(sensor.findtext("camera/image/height"))),
                         c.size)
        plugin = self.model.find("plugin[@name='rover_sim::ChaseCamera']")
        self.assertEqual(plugin.get("filename"), "ChaseCamera")
        self.assertEqual(plugin.findtext("target"), "rover")
        self.assertEqual(plugin.findtext("cmd_topic"), viewers.CHASE_CMD_TOPIC)
        self.assertGreater(float(plugin.findtext("min_pitch")), 0)  # stays above the target

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class EyeCameraModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = viewers.build_eye_sdf(viewers.EyeParams(), ROVER)
        cls.model = ET.fromstring(cls.sdf).find("model")
        cls.plugin = cls.model.find("plugin[@name='rover_sim::ChaseCamera']")

    def test_floats_without_collisions(self):
        self.assertEqual(self.model.get("name"), viewers.EYE_MODEL)
        self.assertEqual(self.model.findtext("link/gravity"), "false")
        self.assertIsNone(self.model.find(".//collision"))

    def test_rides_at_the_camera_pivot(self):
        # Where the tilt axis runs through the RGB-D sensor and the pan axis meets it.
        rover = ET.fromstring(gen_rover.build_sdf(V)).find("model")
        tilt = vec(rover.find("link[@name='camera_tilt_link']/pose").text)
        pan = vec(rover.find("link[@name='camera_pan_link']/pose").text)
        mount = vec(self.plugin.findtext("mount"))
        self.assertEqual(mount, tilt[:3])
        self.assertEqual(mount[:2], pan[:2])
        self.assertEqual(self.plugin.findtext("target"), "rover")

    def test_turns_like_the_head(self):
        value = lambda tag: float(self.plugin.findtext(tag))  # noqa: E731
        head = ROVER.head
        self.assertEqual((value("yaw"), value("pitch")), (0.0, head.home_tilt))
        self.assertEqual((value("min_yaw"), value("max_yaw")), (-head.pan_limit, head.pan_limit))
        self.assertEqual((value("min_pitch"), value("max_pitch")), head.tilt_limits)
        self.assertEqual(self.plugin.findtext("look_topic"), viewers.EYE_LOOK_TOPIC)
        self.assertIsNone(self.plugin.find("cmd_topic"))  # an eye only turns, it does not orbit or zoom

    def test_camera(self):
        e = viewers.EyeParams()
        sensor = self.model.find("link/sensor[@type='camera']")
        self.assertEqual(sensor.findtext("topic"), viewers.EYE_IMAGE_TOPIC)
        self.assertEqual((int(sensor.findtext("camera/image/width")), int(sensor.findtext("camera/image/height"))),
                         (960, 540))
        self.assertEqual(float(sensor.findtext("update_rate")), 20.0)
        self.assertEqual(float(sensor.findtext("camera/horizontal_fov")), e.hfov)
        self.assertEqual(e.hfov, ROVER.camera.hfov)

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class Ros2ControlVariant(unittest.TestCase):
    """build_urdf and build_sdf with ros2_control: the rover sim.launch.py runs (design spec 2026-10-09, 6.1 and 6.3):
    rover_description's ros2_control block with the hardware interface RoverSimSystem, and the gz_ros2_control
    system with rover_control's controllers.yaml just before the drivetrain, so that its PreUpdate, where the
    controller manager writes the setpoints, comes first in each step (tests/test_ros2_control.py)."""

    @classmethod
    def setUpClass(cls):
        cls.urdf = gen_rover.build_urdf(V, ros2_control=True)
        cls.sdf = gen_rover.build_sdf(V, ros2_control=True)
        cls.model = ET.fromstring(cls.sdf).find("model")

    def test_the_block_names_rover_sim_system(self):
        block = ET.fromstring(self.urdf).find("ros2_control")
        self.assertEqual(block.findtext("hardware/plugin"), "rover_sim/RoverSimSystem")
        commanded = [j.get("name") for j in block.findall("joint") if j.find("command_interface") is not None]
        self.assertEqual(commanded, [f"{wheel}_joint" for wheel in WHEELS])
        self.assertEqual(len(block.findall("joint")), 8)

    def test_gz_ros2_control_runs_just_before_the_drivetrain(self):
        plugins = self.model.findall("plugin")
        names = [p.get("name") for p in plugins]
        here = names.index("gz_ros2_control::GazeboSimROS2ControlPlugin")
        self.assertEqual(names[here + 1], "rover_sim::RoverDrivetrain")
        self.assertEqual(plugins[here].get("filename"), "gz_ros2_control-system")
        self.assertEqual(plugins[here].findtext("parameters"), str(rover_config.CONTROLLERS_YAML))
        self.assertTrue(rover_config.CONTROLLERS_YAML.is_file())

    def test_without_it_the_rover_has_neither_and_is_the_same(self):
        default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
        self.assertNotIn("gz_ros2_control::GazeboSimROS2ControlPlugin", [p.get("name") for p in default.iter("plugin")])
        self.assertIsNone(ET.fromstring(gen_rover.build_urdf(V)).find("ros2_control"))
        ros = copy.deepcopy(self.model)
        ros.remove(ros.find("plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"))
        self.assertEqual(canonical(ros), canonical(default))

    def test_check_urdf_accepts_it(self):
        if shutil.which("check_urdf") is None:
            self.skipTest("check_urdf is not installed (liburdfdom-tools)")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "rover.urdf"
            path.write_text(self.urdf)
            result = subprocess.run(["check_urdf", str(path)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Successfully Parsed XML", result.stdout)

    def test_diffdrive_cannot_have_it(self):
        with self.assertRaises(Exception) as caught:
            gen_rover.build_urdf(DIFFDRIVE, ros2_control=True)
        self.assertIn("drivetrain mode physical", str(caught.exception))

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
