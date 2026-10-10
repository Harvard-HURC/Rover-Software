#!/usr/bin/env python3
"""RoverSimSystem (ros2_control/rover_sim_system.cpp) inside Gazebo with the rover's controllers (design spec
2026-10-09, section 6.1), as sim.launch.py runs it: `gz sim -s` with the rover built with ros2_control,
robot_state_publisher, the /clock bridge and rover_control's controllers (joint_state_broadcaster, drive_controller),
in this process's private ROS domain and gz partition (rosstack, simulate). One run on flat ground in real time,
with the drivetrain's state every 1 ms step (stamped with the step's sim time) and the drive controller's messages
(stamped with the controller manager's clock: ros2_control 4.47 gives the controllers and the hardware that clock,
sim time from the bridged /clock, a few ms behind the step); the tests read it:
- gz_ros2_control loads RoverSimSystem and the controllers drive the wheels: /odom and the wheels move;
- the setpoint the controller manager writes at the start of a step is the one the motor model uses in that step;
- the joints' state and the motor currents come back on /joint_states.
The description of that rover is checked in test_rover_model.Ros2ControlVariant."""
import json
import tempfile
import unittest
from pathlib import Path

import yaml

import rosstack
from simulate import physical, world_sdf

import gen_rover  # noqa: E402  (worldfiles, through simulate, puts sim/ and sim/tools on the path)
import rover_config  # noqa: E402
from rover_config import ROVER  # noqa: E402
from urc import sdf  # noqa: E402

from geometry_msgs.msg import TwistStamped  # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402
from gz.transport13 import Node  # noqa: E402

WHEELS = ("fl", "rl", "fr", "rr")
SIDE = {"fl": 0, "rl": 0, "fr": 1, "rr": 1}  # index into (left, right)
JOINTS = {f"wheel_{w}_joint" for w in WHEELS} | {"rocker_left_joint", "rocker_right_joint", rover_config.PAN_JOINT,
                                                 rover_config.TILT_JOINT}
CMD_VEL_OUT = "/drive_controller/cmd_vel_out"
DRIVE = (0.3, 0.5)  # [m/s], [rad/s]: an arc to the left, every wheel forward, the right ones faster
SEPARATION, RADIUS = 2 * ROVER.rocker.pivot_y, ROVER.wheel.radius  # controllers.yaml's (rover_control checks them)
UPDATE_STEPS = 10  # gz_ros2_control updates the controllers every 10th 1 ms step (100 Hz): steps 10, 20, 30 ...


def tearDownModule():
    rosstack.shutdown()


def ms(stamp):
    """A ROS stamp, or a sim time [s], in whole milliseconds: the physics step's grid."""
    if hasattr(stamp, "sec"):
        return round(stamp.sec * 1000 + stamp.nanosec / 1e6)
    return round(stamp * 1000)


def close(got, want):
    """Equal within the drivetrain state's 6 significant digits."""
    return abs(got - want) <= 1e-5 * abs(want) + 1e-6


class RoverSimSystemInGazebo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="rover_ros2_control_")
        tmp = Path(cls.tmp.name)
        variant = physical(drivetrain={"state_rate": 1000.0})
        urdf = gen_rover.build_urdf(variant, ros2_control=True)
        (tmp / "rover").mkdir()
        (tmp / "rover" / "model.sdf").write_text(gen_rover.convert(urdf, variant))
        (tmp / "rover" / "model.config").write_text(sdf.model_config("rover", "The rover with ros2_control.",
                                                                     "rover_sim/tests/test_ros2_control.py"))
        world = tmp / "world.sdf"
        world.write_text(world_sdf(rover_uri=(tmp / "rover").as_uri()).replace(
            "<real_time_factor>0</real_time_factor>", "<real_time_factor>1</real_time_factor>"))
        (tmp / "rsp.yaml").write_text(yaml.safe_dump(
            {"robot_state_publisher": {"ros__parameters": {"robot_description": urdf, "use_sim_time": True}}}))
        (tmp / "test.yaml").write_text(yaml.safe_dump(
            {"drive_controller": {"ros__parameters": {"publish_limited_velocity": True}}}))
        cls.states = []
        node = Node()
        node.subscribe(StringMsg, rover_config.DRIVETRAIN_TOPIC, lambda m: cls.states.append(json.loads(m.data)))
        logs = [tmp / name for name in ("gz.log", "rsp.log", "bridge.log", "spawner.log")]
        processes = [
            rosstack.start(["gz", "sim", "-s", "-r", world], logs[0]),
            rosstack.start(["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args",
                            "--params-file", tmp / "rsp.yaml"], logs[1]),
            rosstack.start(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], logs[2]),
            rosstack.start(rosstack.spawner(extra_params=[tmp / "test.yaml"]), logs[3]),
        ]
        probe = rosstack.Probe(topics=[(CMD_VEL_OUT, TwistStamped)])
        try:
            probe.wait(lambda: probe.messages[CMD_VEL_OUT] and probe.cmd.get_subscription_count() > 0, 180,
                       "update of the drive controller", logs)
            cls.drive(probe)
        finally:
            cls.messages = {topic: list(found) for topic, found in probe.messages.items()}
            probe.close()
            for process in reversed(processes):
                rosstack.stop(process)
            node.unsubscribe(rover_config.DRIVETRAIN_TOPIC)
        cls.log = logs[0].read_text(errors="replace")

    @classmethod
    def drive(cls, probe):
        probe.drive_for(*DRIVE, sim_seconds=3.0)
        probe.drive_for(0.0, 0.0, sim_seconds=1.5)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_the_controllers_drive_the_rover(self):
        self.assertIn("RoverSimSystem: 8 joints, 4 wheels driven", self.log)
        self.assertTrue(self.messages[CMD_VEL_OUT], "no cmd_vel_out: the drive controller never updated")
        odom = self.messages["/odom"]
        self.assertTrue(odom, "no /odom")
        self.assertGreater(odom[-1].pose.pose.position.x, 0.3)
        self.assertGreater(odom[-1].pose.pose.position.y, 0.05)  # the arc turned left
        joints = self.messages["/joint_states"][-1]
        self.assertEqual(set(joints.name), JOINTS)
        position = dict(zip(joints.name, joints.position))
        # The arc's wheel commands: left (0.3 - 0.5 x 0.4) / 0.15 = 0.67 rad/s, right 3.33 rad/s, for 3 s of sim time
        # less the ramps: some 1.9 and 9.6 rad.
        for w in WHEELS:
            self.assertGreater(position[f"wheel_{w}_joint"], 0.5 if SIDE[w] == 0 else 5.0, w)
        self.assertGreater(position["wheel_fr_joint"], 3 * position["wheel_fl_joint"])

    def test_a_setpoint_written_at_a_step_drives_the_motors_in_that_step(self):
        """gz_ros2_control runs the controller manager's update in its PostUpdate of every 10th step, counted on sim
        time from 0 (steps 10, 20, 30 ...), and hands the new wheel commands to RoverSimSystem in its PreUpdate of
        the next step; the drivetrain, whose plugin comes after gz_ros2_control's, uses them in that same step. So
        every change of a setpoint in the drivetrain's state (every step) is in a step 11, 21, 31 ...: written and
        used in the same step (one step later they would be in steps 12, 22, 32 ...). The stamps of cmd_vel_out and
        /joint_states cannot show it: they are the controller manager's clock, a few ms behind the step."""
        sp = {ms(s["t"]): tuple(s["wheels"][w]["sp"] for w in WHEELS) for s in self.states}
        changes = [t for t in sorted(sp) if t - 1 in sp and sp[t] != sp[t - 1]]
        self.assertGreater(len(changes), 20)  # the ramps up and down: some 50 updates that change a command
        self.assertEqual(sorted({t % UPDATE_STEPS for t in changes}), [1], changes[:20])

    def test_the_joint_state_and_the_motor_currents_come_back(self):
        """/joint_states (joint_state_broadcaster) carries each wheel's motor current as its effort: the drivetrain's
        current of an update step (read() runs in gz_ros2_control's PostUpdate of steps 10, 20, 30 ..., after the
        drivetrain's PreUpdate of that step wrote it). A message's stamp is the controller manager's clock, a few ms
        behind the step, so the step is found near it: the steps within 50 ms whose four currents all equal the
        efforts. Where that is one step (the currents change from step to step while the rover drives), it is an
        update step."""
        currents = {ms(s["t"]): [s["wheels"][w]["i"] for w in WHEELS] for s in self.states}
        pinned = drawn = 0
        for message in self.messages["/joint_states"]:
            effort = dict(zip(message.name, message.effort))
            got = [effort[f"wheel_{w}_joint"] for w in WHEELS]
            stamp = ms(message.header.stamp)
            if not any(t in currents for t in range(stamp - 50, stamp + 51)):
                continue  # before the drivetrain's first state, or after its last
            steps = [t for t in range(stamp - 50, stamp + 51)
                     if t in currents and all(close(g, c) for g, c in zip(got, currents[t]))]
            self.assertTrue(steps, (stamp, got))
            if len(steps) == 1:
                self.assertEqual(steps[0] % UPDATE_STEPS, 0, (stamp, steps, got))
                pinned += 1
            drawn += max(abs(g) for g in got) > 1.0
        self.assertGreater(pinned, 50)
        self.assertGreater(drawn, 0, "the motors drew current")


if __name__ == "__main__":
    unittest.main()
