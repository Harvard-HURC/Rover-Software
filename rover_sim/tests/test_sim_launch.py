#!/usr/bin/env python3
"""launch/sim.launch.py (design spec 2026-10-09, 6.3): its arguments and what it starts (loaded, nothing started),
the world's copy with the rover swapped for the one with ros2_control, the sensor bridge's topics, and (SmokeTest) a
run of it headless."""
import importlib.util
import logging
import re
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import yaml

import rosstack
from worldfiles import SIM_DIR, WORLDS

import gzenv  # noqa: E402  (worldfiles puts rover_sim/ on the path)

SIM_LAUNCH = SIM_DIR / "launch" / "sim.launch.py"
# The topic contract (design spec 6.2, docs/topics.md) the simulation provides.
CONTRACT = {"/cmd_vel", "/odom", "/joint_states", "/tf", "/tf_static", "/robot_description", "/imu", "/gps/fix",
            "/camera/image", "/camera/depth", "/camera/camera_info", "/clock", "/ground_truth/odom"}


def load():
    """The launch file as a module."""
    spec = importlib.util.spec_from_file_location("sim_launch", SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def tearDownModule():
    rosstack.shutdown()


class Arguments(unittest.TestCase):
    def test_the_five_arguments_and_their_defaults(self):
        from launch.actions import DeclareLaunchArgument
        module = load()
        for no_display, gui in ((False, "true"), (True, "false")):
            with mock.patch.object(module, "headless", lambda: no_display):
                entities = module.generate_launch_description().entities
            defaults = {e.name: e.default_value[0].text for e in entities if isinstance(e, DeclareLaunchArgument)}
            self.assertEqual(defaults, {"world": "rover_test", "gui": gui, "station": "false",
                                        "controller": "drive_controller", "sensors": "true"})

    def test_the_source_tree_from_here_or_from_the_installed_marker(self):
        module = load()
        self.assertEqual(module.source_dir(SIM_LAUNCH), SIM_DIR)
        with tempfile.TemporaryDirectory() as tmp:
            share = Path(tmp) / "share" / "rover_sim"
            (share / "launch").mkdir(parents=True)
            (share / "source_dir").write_text(f"{SIM_DIR}\n")
            self.assertEqual(module.source_dir(share / "launch" / "sim.launch.py"), SIM_DIR)

    def test_the_installed_launch_files_find_this_tree(self):
        share = gzenv.INSTALL_DIR / "share" / "rover_sim"
        self.assertEqual((share / "source_dir").read_text().strip(), str(SIM_DIR))
        for path in ("launch/sim.launch.py", "urdf/rover_sim.urdf.xacro", "config/sim.yaml"):
            self.assertTrue((share / path).is_file(), path)


class WorldCopy(unittest.TestCase):
    def test_the_rover_include_points_at_the_ros_rover_with_its_pose(self):
        module = load()
        text = (WORLDS / "rover_test.sdf").read_text()
        copy = module.ros_world(text, "file:///tmp/rover", sensors=False)
        includes = [i for i in ET.fromstring(copy).iter("include") if i.findtext("uri") == "file:///tmp/rover"]
        self.assertEqual(len(includes), 1)
        original = [i for i in ET.fromstring(text).iter("include") if i.findtext("uri") == "model://rover"][0]
        self.assertEqual(includes[0].findtext("pose"), original.findtext("pose"))
        self.assertNotIn("gz-sim-sensors-system", copy)
        self.assertIn("gz-sim-sensors-system", module.ros_world(text, "file:///tmp/rover"))

    def test_a_world_without_exactly_one_rover_is_refused(self):
        module = load()
        with self.assertRaises(ValueError):
            module.ros_world('<sdf version="1.11"><world name="w"/></sdf>', "file:///tmp/rover")

    def test_prepare_writes_the_ros_rover_and_the_copy(self):
        module = load()
        with tempfile.TemporaryDirectory() as tmp:
            copy, urdf = module.prepare(WORLDS / "rover_test.sdf", True, Path(tmp))
            model = ET.parse(Path(tmp) / "rover" / "model.sdf").getroot().find("model")
            self.assertEqual(model.get("name"), "rover")
            self.assertIn("gz_ros2_control::GazeboSimROS2ControlPlugin", [p.get("name") for p in model.iter("plugin")])
            self.assertEqual(ET.fromstring(urdf).findtext("ros2_control/hardware/plugin"), "rover_sim/RoverSimSystem")
            self.assertIn((Path(tmp) / "rover").as_uri(), copy.read_text())
            self.assertEqual(module.world_name(copy.read_text()), "rover_test")

    def test_what_it_starts(self):
        """The server on the copy, robot_state_publisher, the controllers and the bridge; the GUI and the station
        only when asked, the station attached to the world by its name."""
        from launch import LaunchContext
        from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
        from launch_ros.actions import Node
        module = load()
        with tempfile.TemporaryDirectory() as tmp:
            for gui, station in (("false", "false"), ("true", "true")):
                run = Path(tmp) / gui
                with mock.patch.object(module, "mkdtemp", lambda prefix: str(run.mkdir() or run)):
                    context = LaunchContext()
                    context.launch_configurations.update(world="rover_test", gui=gui, station=station,
                                                         controller="drive_controller", sensors="false")
                    actions = module.launch_setup(context)

                def text(substitutions):
                    return "".join(s.perform(context) for s in substitutions)

                def command(action):
                    return [text(part) for part in action.cmd]

                # An ExecuteProcess's name and each part of its cmd are lists of substitutions (launch normalizes
                # them), not strings: performed, they are.
                processes = {text(a.name): command(a) for a in actions
                             if isinstance(a, ExecuteProcess) and not isinstance(a, Node)}
                expected = {"gz_server"} | ({"gz_gui", "station"} if gui == "true" else set())
                self.assertEqual(set(processes), expected)
                self.assertEqual(processes["gz_server"], ["gz", "sim", "-s", "-r", str(run / "rover_test.sdf")])
                self.assertEqual([a.node_executable for a in actions if isinstance(a, Node)],
                                 ["robot_state_publisher", "parameter_bridge"])
                groups = [a for a in actions if isinstance(a, GroupAction)]
                self.assertEqual(len(groups), 1)  # the controllers, their spawner's ROS_HOME this launch's own
                inside = groups[0].get_sub_entities()
                self.assertEqual(len([a for a in inside if isinstance(a, IncludeLaunchDescription)]), 1)
                ros_home = {text(a.name): text(a.value) for a in inside if isinstance(a, SetEnvironmentVariable)}
                self.assertEqual(ros_home["ROS_HOME"], str(run / "ros_home"))
                if station == "true":
                    self.assertEqual(processes["station"][-4:], ["--world-name", "rover_test", "--wait", "120"])


class Controllers(unittest.TestCase):
    """The controllers after a world reset, which reloads gz_ros2_control (it does not implement Gazebo's reset), and
    a spawner that fails."""

    LINE = (b"[INFO] [1791666490.415934000] [controller_manager.hardware_component.system.rover]: RoverSimSystem: "
            b"8 joints, 4 wheels driven through RoverDrivetrain's motor model\n")

    def test_the_line_they_wait_for_is_rover_sim_systems(self):
        source = (SIM_DIR / "ros2_control" / "rover_sim_system.cpp").read_text()
        self.assertIn(load().HARDWARE_UP.decode(), source)

    def test_they_are_spawned_again_each_time_the_hardware_comes_up_after_the_first(self):
        spawned = []
        respawn = load().RespawnAfterReset(lambda: spawned.append("spawn") or "spawn")

        def output(text, stdout=False):
            return respawn(SimpleNamespace(from_stdout=stdout, text=text))

        self.assertEqual(output(self.LINE), [])  # the start: controllers.launch.py's spawner is on its way
        self.assertEqual(output(b"[gz_server-1] [INFO] Loading controller_manager\n"), [])
        actions = output(self.LINE[:40]) + output(b"[some gz line on stdout]\n", stdout=True) + output(self.LINE[40:])
        self.assertEqual(actions[-1], "spawn")  # a reset, its line in two pieces of the server's output
        self.assertEqual(output(self.LINE)[-1], "spawn")  # another reset
        self.assertEqual(spawned, ["spawn", "spawn"])

    def test_a_spawner_that_fails_stops_the_launch_and_says_why(self):
        from launch.actions import Shutdown
        module = load()

        def exited(executable, code, stopping=False):
            event = SimpleNamespace(action=SimpleNamespace(node_executable=executable), returncode=code)
            return module.stop_if_a_spawner_fails(event, SimpleNamespace(is_shutdown=stopping))

        quiet = logging.getLogger("test_sim_launch.spawner")  # launch's own logger would print it in the suite
        with mock.patch.object(module.launch.logging, "get_logger", lambda name: quiet), \
                self.assertLogs(quiet, "ERROR") as logs:
            actions = exited("spawner", 1)
        self.assertEqual([type(a) for a in actions], [Shutdown])
        self.assertIn("the controllers did not start", logs.output[0])
        self.assertIsNone(exited("spawner", 0))
        self.assertIsNone(exited("spawner", -9, stopping=True))  # killed while the launch stops
        self.assertIsNone(exited("robot_state_publisher", 1))
        self.assertIsNone(module.stop_if_a_spawner_fails(SimpleNamespace(action=object(), returncode=1),
                                                         SimpleNamespace(is_shutdown=False)))  # Gazebo's server


class SensorBridge(unittest.TestCase):
    def test_the_contracts_sensor_topics_and_none_of_the_controllers(self):
        """bridge.yaml bridges the sensors and the simulation's own topics; /cmd_vel, /odom, /joint_states and /tf
        are the controllers' and robot_state_publisher's."""
        names = {entry["ros_topic_name"] for entry in yaml.safe_load((SIM_DIR / "bridge.yaml").read_text())}
        self.assertLessEqual({"/imu", "/gps/fix", "/camera/image", "/camera/depth", "/camera/camera_info", "/clock",
                              "/ground_truth/odom"}, names)
        self.assertEqual(names & {"/cmd_vel", "/odom", "/joint_states", "/tf"}, set())


class SmokeTest(unittest.TestCase):
    """sim.launch.py headless on rover_test (gui:=false sensors:=false: no GPU needed, as on CI's Linux runner), in this
    process's private gz partition and ROS domain (design spec 2026-10-09, 8: the smoke test): a /cmd_vel command moves
    /odom, the topic contract's topics are there, the drive controller's timeout brakes the rover, after a world reset
    (which reloads gz_ros2_control) the launch spawns the controllers again and /cmd_vel drives again, and SIGINT stops
    it all."""

    ACTIVATED = "Configured and activated drive_controller"  # the spawner's line

    def test_cmd_vel_moves_odom(self):
        with tempfile.TemporaryDirectory(prefix="rover_smoke_") as tmp:
            log = Path(tmp) / "launch.log"
            launch = rosstack.start(["ros2", "launch", SIM_LAUNCH, "world:=rover_test", "gui:=false", "sensors:=false"],
                                    log)
            probe = rosstack.Probe()
            try:
                probe.wait(lambda: probe.messages["/odom"] and probe.cmd.get_subscription_count() > 0, 180,
                           "/odom and a subscriber of /cmd_vel", [log])
                start = probe.messages["/odom"][-1].pose.pose.position.x
                # On sim time (/odom's stamps), so that a runner slower than real time drives as far as this Mac.
                probe.drive_for(0.5, 0.0, sim_seconds=5.0, timeout=90)
                driven = probe.messages["/odom"][-1].pose.pose.position.x - start
                # No command for 2 s of sim time: the drive controller's 0.5 s timeout, then its 0.42 s ramp, brake.
                probe.drive_for(0.0, 0.0, sim_seconds=2.0, timeout=60, publish=False)
                coasting = probe.messages["/odom"][-1].twist.twist.linear.x
                topics = probe.topics()
                reset = subprocess.run(["gz", "service", "-s", "/world/rover_test/control", "--reqtype",
                                        "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", "--timeout", "10000",
                                        "--req", "reset: {all: true}"], capture_output=True, text=True, timeout=60)
                probe.wait(lambda: log.read_text(errors="replace").count(self.ACTIVATED) == 2, 120,
                           "drive controller spawned again after the reset", [log])
                count = len(probe.messages["/odom"])  # the new drive controller's odometry, from the origin
                probe.wait(lambda: len(probe.messages["/odom"]) > count, 30, "/odom after the reset", [log])
                start_again = probe.messages["/odom"][-1].pose.pose.position.x
                probe.drive_for(0.5, 0.0, sim_seconds=3.0, timeout=60)
                driven_again = probe.messages["/odom"][-1].pose.pose.position.x - start_again
            finally:
                probe.close()
                rosstack.stop(launch)
                text = log.read_text(errors="replace")
        copy = re.search(r"world: (\S+) \(a copy of", text)
        self.assertIsNotNone(copy, text[-3000:])
        self.assertGreater(driven, 1.5, text[-3000:])  # 0.5 m/s for 5 s of sim time, less the ramp and the slip
        self.assertLess(abs(coasting), 0.05)
        self.assertLessEqual(CONTRACT, topics)
        self.assertIn("data: true", reset.stdout, reset.stdout + reset.stderr)
        self.assertIn("the world was reset: gz_ros2_control started a new controller manager", text)
        self.assertGreater(driven_again, 0.8, text[-3000:])  # 0.5 m/s for 3 s, less the ramp and the slip
        self.assertNotEqual(subprocess.run(["pgrep", "-f", copy.group(1)], capture_output=True).returncode, 0,
                            "Gazebo's server still runs")


if __name__ == "__main__":
    unittest.main()
