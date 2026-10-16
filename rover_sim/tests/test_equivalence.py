#!/usr/bin/env python3
"""The rover the simulation loads is the rover gen_model.py wrote, number for number (design spec 2026-10-09,
section 5): tools/gen_rover.py's model of the description, and of each variant the tests build, against the frozen
models in fixtures/rover_model (written by gen_model.py at import/simulation, 9208047; fixtures/rover_model/README.md
lists them), after ros2_control_boundary(), the one change since that is not physical. SdfEquivalence checks the
comparison itself; Drives runs the frozen and the generated rover side by side in Gazebo. When the rover changes on
purpose, refreeze the fixtures from gen_rover.py in the same commit and say why (fixtures/rover_model/README.md)."""
import contextlib
import copy
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from sdf_equivalence import differences
from simulate import simulate
from worldfiles import SIM_DIR

import gen_rover  # noqa: E402  (worldfiles puts sim/ and sim/tools on the path)
import rover_config  # noqa: E402
from rover_config import Variant  # noqa: E402
from urc import sdf  # noqa: E402

FIXTURES = SIM_DIR / "tests" / "fixtures" / "rover_model"
V = Variant()
# The frozen models and the variants of rover.yaml and sim.yaml that are the same rovers (README.md beside them).
VARIANTS = {
    "default": V,
    "diffdrive": V.replace(drivetrain={"mode": "diffdrive"}),
    "diffdrive_square": V.replace(drivetrain={"mode": "diffdrive"}, diffdrive={"mu_lateral": 1.0}),
    "diffdrive_dust": V.replace(drivetrain={"mode": "diffdrive"}, dust={"enabled": True}),
    "dust": V.replace(dust={"enabled": True}),
    "cues_off": V.replace(tread_tyre=False, dig_sink={"enabled": False}, ruts={"enabled": False}),
    "tyre_compliance": V.replace(tyre_compliance={"enabled": True}),
    "test_physical": V.replace(
        drivetrain={"cmd_timeout_clock": "sim", "state_rate": 1000.0, "cmd_timeout": 0.0},
        contact={"dig": False, "mu_noise": 0.0, "default_surface": "sand"}, driveline={"backlash": 0.0},
        motor={"current_limit": 10.0}, ruts={"tracks": {"report": 1.0}}),
    "same_surfaces": V.replace(contact={"default_surface": "manmade"}),
    "unknown_surface": V.replace(contact={"default_surface": "lunar_dust"}),
}
WALL_CLOCK = "<cmd_timeout_clock>wall</cmd_timeout_clock>"
SIM_CLOCK = "<cmd_timeout_clock>sim</cmd_timeout_clock>"


def fixture(name):
    return (FIXTURES / f"{name}.sdf").read_text()


def ros2_control_boundary(text):
    """A frozen model as the generator writes it since the ros2_control boundary (design spec 2026-10-09, section 6.1;
    PR 3): RoverDrivetrain no longer turns a twist into wheel setpoints, so its twist topic, track multiplier and ramp
    (<topic>, <track_multiplier>, <controller><accel>) are gone, and it takes setpoints on <setpoint_topic>. Nothing
    physical changes; a DiffDrive model is left as it is."""
    root = ET.fromstring(text)
    for plugin in root.iter("plugin"):
        if plugin.get("filename") != "RoverDrivetrain":
            continue
        for parent, tag in ((plugin, "topic"), (plugin, "track_multiplier"), (plugin.find("controller"), "accel")):
            parent.remove(parent.find(tag))
        sdf.sub(plugin, "setpoint_topic", rover_config.WHEEL_CMD_TOPIC)
    return sdf.document(root)


class SdfEquivalence(unittest.TestCase):
    """differences() finds what changes a model and nothing else."""

    @classmethod
    def setUpClass(cls):
        cls.reference = fixture("default")

    def changed(self, edit):
        root = ET.fromstring(self.reference)
        edit(root.find("model"))
        return differences(self.reference, sdf.document(root))

    def test_a_model_equals_itself(self):
        self.assertEqual(differences(self.reference, self.reference), [])

    def test_a_number_beyond_the_tolerance_differs(self):
        def heavier(model):
            mass = model.find("link[@name='wheel_fl']/inertial/mass")
            mass.text = sdf.fmt(float(mass.text) * (1 + 1e-6))
        self.assertEqual(len(self.changed(heavier)), 1)

    def test_rounding_within_the_tolerance_does_not(self):
        def rounded(model):
            ixx = model.find("link[@name='wheel_fl']/inertial/inertia/ixx")
            ixx.text = repr(float(ixx.text) * (1 + 1e-12))
        self.assertEqual(self.changed(rounded), [])

    def test_a_missing_plugin_parameter_differs(self):
        def without_ki(model):
            controller = model.find("plugin[@filename='RoverDrivetrain']/controller")
            controller.remove(controller.find("ki"))
        self.assertEqual(len(self.changed(without_ki)), 1)

    def test_the_plugins_order_counts(self):
        def swapped(model):
            plugins = model.findall("plugin")
            for plugin in plugins[2:4]:
                model.remove(plugin)
            model.extend([plugins[3], plugins[2]])
        self.assertTrue(any("plugin order" in line for line in self.changed(swapped)))

    def test_a_default_written_out_is_the_same(self):
        def explicit(model):
            axis = model.find("joint[@name='wheel_fl_joint']/axis")
            sdf.sub(sdf.sub(axis, "dynamics"), "spring_stiffness", 0.0)
        self.assertEqual(self.changed(explicit), [])

    def test_a_pose_compares_its_rotation_not_its_angles(self):
        path = "link[@name='rocker_left']/collision[@name='front_arm_collision']/pose"
        x, y, z, _, pitch, _ = (float(v) for v in ET.fromstring(self.reference).find(f"model/{path}").text.split())

        def turned(rpy):
            return lambda model: setattr(model.find(path), "text", sdf.fmt((x, y, z, *rpy)))

        self.assertEqual(self.changed(turned((np.pi, np.pi - pitch, np.pi))), [])  # the same rotation
        self.assertEqual(len(self.changed(turned((0.0, pitch + 1e-6, 0.0)))), 1)

    def test_an_element_only_one_side_has_differs(self):
        def extra(model):
            sdf.sub(model.find("link[@name='base_link']"), "self_collide", True)
        self.assertEqual(len(self.changed(extra)), 1)

    def test_a_name_two_elements_share_differs(self):
        """Elements pair by name, so a second one of the same name, the same or not, would hide behind the other."""
        def same_sensor(model):
            link = model.find("link[@name='base_link']")
            link.append(copy.deepcopy(link.find("sensor[@name='imu']")))

        def other_tyre(model):
            link = model.find("link[@name='wheel_fl']")
            tyre = copy.deepcopy(link.find("collision[@name='tire_collision']"))
            tyre.find("geometry/cylinder/radius").text = "9"
            link.insert(0, tyre)

        for edit in (same_sensor, other_tyre):
            with self.subTest(edit.__name__):
                self.assertEqual(len(self.changed(edit)), 1)


class Equivalence(unittest.TestCase):
    """Each generated variant is its frozen model: links' mass, inertia and pose; joints' type, axis, limits,
    damping and springs; collisions, visuals and materials; sensors; every plugin and parameter, in order."""

    def test_the_generated_rovers_are_the_frozen_ones(self):
        self.assertEqual(sorted(p.stem for p in FIXTURES.glob("*.sdf")), sorted(VARIANTS))
        for name, variant in VARIANTS.items():
            with self.subTest(name):
                self.assertEqual(differences(ros2_control_boundary(fixture(name)), gen_rover.build_sdf(variant)), [])

    def test_the_rover_is_the_tracked_model(self):
        """The tracked model://rover is the default rover (the comparison, not the bytes: gen_rover.py lays it out
        as the converter does)."""
        tracked = (SIM_DIR / "models" / "rover" / "model.sdf").read_text()
        self.assertEqual(differences(ros2_control_boundary(fixture("default")), tracked), [])

    def test_the_boundary_changes_only_the_drives_command_interface(self):
        """ros2_control_boundary changes four elements of a physical rover's drivetrain and nothing of a DiffDrive
        rover's."""
        self.assertEqual(len(differences(fixture("default"), ros2_control_boundary(fixture("default")))), 4)
        self.assertEqual(differences(fixture("diffdrive"), ros2_control_boundary(fixture("diffdrive"))), [])


@contextlib.contextmanager
def model_uri(text):
    """A temporary model directory holding the SDF document `text`, its command timeout on sim time (so runs repeat
    exactly)."""
    with tempfile.TemporaryDirectory(prefix="rover_equivalence_") as directory:
        model = Path(directory) / "rover"
        model.mkdir()
        assert text.count(WALL_CLOCK) <= 1
        (model / "model.sdf").write_text(text.replace(WALL_CLOCK, SIM_CLOCK))
        (model / "model.config").write_text(sdf.model_config("rover", "A rover to compare.",
                                                             "sim/tests/test_equivalence.py"))
        yield model.as_uri()


class Drives(unittest.TestCase):
    """The frozen and the generated rover drive the same 3 s schedule (forward, an arc, a turn in place) on flat
    ground to the same base_link trace, step for step, and the same rocker angles, bit for bit: the physical rover,
    the one with tyre compliance and DiffDrive's."""
    SCHEDULE = [(0.0, 0.0, 0.0), (0.5, 0.5, 0.3), (2.0, 0.0, 1.0)]

    def run_rover(self, text):
        with model_uri(text) as uri:
            return simulate(3.0, cmd=self.SCHEDULE, rover_uri=uri, trace_every=1)

    def test_bit_for_bit(self):
        for name in ("default", "tyre_compliance", "diffdrive"):
            with self.subTest(name):
                old = self.run_rover(fixture(name))
                new = self.run_rover(gen_rover.build_sdf(VARIANTS[name]))
                self.assertGreater(abs(old.poses["base_link"][0]), 0.3)  # it drove
                np.testing.assert_array_equal(old.trace, new.trace)
                self.assertEqual(old.rockers, new.rockers)
                self.assertEqual(old.poses, new.poses)


if __name__ == "__main__":
    unittest.main()
