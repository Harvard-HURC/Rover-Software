#!/usr/bin/env python3
"""The package's launch files (rover_sim/launch) describe a launch on every
platform; they are only loaded here, nothing is started."""
import contextlib
import importlib.util
import io
import sys
import unittest
from unittest import mock

from launch import LaunchDescription

from worldfiles import SIM_DIR

GAZEBO_LAUNCH = SIM_DIR / "launch" / "gazebo.launch.py"


def describe(path, platform):
    """generate_launch_description() of a launch file as `platform` (sys.platform) runs it."""
    spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with mock.patch.object(sys, "platform", platform), contextlib.redirect_stdout(io.StringIO()):
        return module.generate_launch_description()


class GazeboLaunch(unittest.TestCase):
    def test_macos_linux_and_windows_get_a_launch_description(self):
        for platform in ("darwin", "linux", "win32"):
            with self.subTest(platform):
                self.assertIsInstance(describe(GAZEBO_LAUNCH, platform), LaunchDescription)


if __name__ == "__main__":
    unittest.main()
