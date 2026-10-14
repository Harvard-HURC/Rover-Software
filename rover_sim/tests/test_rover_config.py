#!/usr/bin/env python3
"""rover_config.py, the simulation's one reader of rover.yaml and sim.yaml (no Gazebo)."""
import unittest

import yaml

import worldfiles  # noqa: F401  (puts sim/ on the path)

import rover_config  # noqa: E402
from rover_config import ROVER, SIM, Tree, Variant  # noqa: E402


class Files(unittest.TestCase):
    def test_the_files_share_no_section(self):
        """Variant.replace() finds a section by its name: a name in both files would be ambiguous."""
        self.assertEqual(set(ROVER) & set(SIM), set())

    def test_numbers_keep_their_type(self):
        """Every number that is not a count is written with a decimal point, so the tools read a float."""
        self.assertIsInstance(ROVER.wheel.top_speed, float)
        self.assertIsInstance(ROVER.rocker.limit, float)
        self.assertIsInstance(SIM.ruts.tracks.chunk, int)
        self.assertEqual(ROVER.camera.size, (1280, 720))

    def test_trees_read_like_the_files(self):
        self.assertEqual(ROVER.plain(), yaml.safe_load(rover_config.ROVER_YAML.read_text()))
        self.assertEqual(SIM.plain(), yaml.safe_load(rover_config.SIM_YAML.read_text()))
        self.assertEqual(ROVER["wheel"]["radius"], ROVER.wheel.radius)
        with self.assertRaises(AttributeError):
            ROVER.wheel.no_such_number  # noqa: B018

    def test_topics_are_the_gazebo_interface(self):
        self.assertEqual(rover_config.CMD_VEL_TOPIC, "/model/rover/cmd_vel")
        self.assertEqual(rover_config.HEAD_TOPIC.format(joint=rover_config.PAN_JOINT),
                         "/model/rover/joint/camera_pan_joint/0/cmd_pos")
        self.assertEqual(rover_config.DUST_TOPIC.format(link="rocker_left", emitter="dust_rl"),
                         "/model/rover/link/rocker_left/particle_emitter/dust_rl/cmd")


class Variants(unittest.TestCase):
    def test_replace_merges_into_the_sections(self):
        v = Variant().replace(contact={"dig": False}, motor={"current_limit": 10.0}, tread_tyre=False)
        self.assertFalse(v.sim.contact.dig)
        self.assertEqual(v.sim.contact.mu_noise, SIM.contact.mu_noise)
        self.assertEqual(v.rover.motor.current_limit, 10.0)
        self.assertEqual(v.rover.motor.kt, ROVER.motor.kt)
        self.assertFalse(v.sim.tread_tyre)
        self.assertEqual(Variant().sim, SIM)  # the default is the files' own, untouched

    def test_nested_sections_merge_too(self):
        v = Variant().replace(ruts={"tracks": {"report": 1.0}})
        self.assertEqual(v.sim.ruts.tracks.report, 1.0)
        self.assertEqual(v.sim.ruts.tracks.spacing, SIM.ruts.tracks.spacing)
        self.assertTrue(v.sim.ruts.enabled)

    def test_a_misspelt_name_is_refused(self):
        with self.assertRaises(KeyError):
            Variant().replace(contact={"mu_nosie": 0.0})
        with self.assertRaises(KeyError):
            Variant().replace(no_such_section={"x": 1.0})

    def test_variants_are_values(self):
        """Equal variants are equal and hash alike: gen_rover.build_sdf caches by variant."""
        a = Variant().replace(contact={"dig": False})
        b = Variant().replace(contact={"dig": False})
        self.assertEqual(a, b)
        self.assertEqual(hash(a), hash(b))
        self.assertNotEqual(a, Variant())
        self.assertIsInstance(a.sim.contact, Tree)


if __name__ == "__main__":
    unittest.main()
