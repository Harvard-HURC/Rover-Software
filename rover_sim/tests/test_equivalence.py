#!/usr/bin/env python3
"""The rover the simulation loads is the rover gen_model.py wrote, number for number (design spec 2026-10-09,
section 5): tools/gen_rover.py's model of the description, and of each variant the tests build, against the frozen
models in fixtures/rover_model (written by gen_model.py at import/simulation, 9208047; fixtures/rover_model/README.md
lists them). SdfEquivalence checks the comparison itself; Drives runs the frozen and the generated rover side by
side in Gazebo. When the rover changes on purpose, refreeze the fixtures from gen_rover.py in the same commit and say
why (fixtures/rover_model/README.md)."""
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from sdf_equivalence import differences
from worldfiles import SIM_DIR

from urc import sdf  # noqa: E402  (worldfiles puts sim/ on the path)

FIXTURES = SIM_DIR / "tests" / "fixtures" / "rover_model"


def fixture(name):
    return (FIXTURES / f"{name}.sdf").read_text()


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


if __name__ == "__main__":
    unittest.main()
