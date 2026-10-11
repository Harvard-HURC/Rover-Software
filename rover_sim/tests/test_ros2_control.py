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
- the joints' state and the motor currents come back on /joint_states;
- when /clock stops, the drivetrain's watchdog stops the motors (the drive controller's timeout cannot: its clock
  stops with it). That run comes last, and the other tests read only what came before it.
The description of that rover is checked in test_rover_model.Ros2ControlVariant."""
import json
import os
import signal
import tempfile
import time
import unittest
from pathlib import Path

import yaml

import rosstack
from simulate import physical, world_sdf

import gen_rover  # noqa: E402  (worldfiles, through simulate, puts sim/ and sim/tools on the path)
import gzenv  # noqa: E402
import rover_config  # noqa: E402
import wheel_cmd  # noqa: E402
from rover_config import ROVER  # noqa: E402
from urc import sdf  # noqa: E402

from geometry_msgs.msg import TwistStamped  # noqa: E402
from gz.msgs10.double_v_pb2 import Double_V  # noqa: E402
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
CMD_TIMEOUT = 0.5  # [s] sim.yaml drivetrain cmd_timeout (the test world's watchdog runs on the sim clock: physical())


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
        cls.wheel_cmd = node.advertise(rover_config.WHEEL_CMD_TOPIC, Double_V)  # early: it connects meanwhile
        logs = [tmp / name for name in ("gz.log", "rsp.log", "bridge.log", "spawner.log")]
        processes = [
            rosstack.start(["gz", "sim", "-s", "-r", world], logs[0]),
            rosstack.start(["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args",
                            "--params-file", tmp / "rsp.yaml"], logs[1]),
            rosstack.start(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], logs[2]),
            # Its lock file in a ROS_HOME of this run's own: in ~/.ros every spawner on the machine shares it.
            rosstack.start(rosstack.spawner(extra_params=[tmp / "test.yaml"]), logs[3],
                           env=dict(os.environ, **gzenv.private_ros_home(tmp / "ros_home"))),
        ]
        probe = rosstack.Probe(topics=[(CMD_VEL_OUT, TwistStamped)])
        cls.cut = {}
        try:
            probe.wait(lambda: probe.messages[CMD_VEL_OUT] and probe.cmd.get_subscription_count() > 0, 180,
                       "update of the drive controller", logs)
            cls.drive(probe)
            cut = {topic: len(found) for topic, found in probe.messages.items()}
            time.sleep(0.3)  # the drivetrain's states of those messages' steps come in too
            cls.cut = {**cut, "states": len(cls.states)}
            cls.stop_the_clock(probe, processes[2])
        finally:
            # The other tests read what came before the clock stopped: the messages after it carry frozen stamps.
            cls.messages = {topic: list(found)[:cls.cut.get(topic)] for topic, found in probe.messages.items()}
            probe.close()
            for process in reversed(processes):
                rosstack.stop(process)
            node.unsubscribe(rover_config.DRIVETRAIN_TOPIC)
        cut = cls.cut.get("states", len(cls.states))
        cls.clock_stopped, cls.states = cls.states[cut:], cls.states[:cut]
        cls.log = logs[0].read_text(errors="replace")

    @classmethod
    def drive(cls, probe):
        probe.drive_for(*DRIVE, sim_seconds=1.5)
        for _ in range(3):  # the ROS-free setpoint topic, while ros2_control drives the wheels
            cls.wheel_cmd.publish(wheel_cmd.message((5.0,) * 4))
            probe.drive_for(*DRIVE, sim_seconds=0.1)
        probe.drive_for(*DRIVE, sim_seconds=1.2)
        probe.drive_for(0.0, 0.0, sim_seconds=1.5)

    @classmethod
    def stop_the_clock(cls, probe, bridge):
        """Drive the arc again, then stop the /clock bridge (SIGSTOP to its process group) while /cmd_vel keeps coming
        at 20 Hz, for 1.5 s of the drivetrain's sim time (its states keep coming over gz-transport); then let the
        bridge go on. cls.stopped_at: the drivetrain's sim time when the bridge stopped."""
        probe.drive_for(*DRIVE, sim_seconds=1.0)
        msg = TwistStamped()
        msg.twist.linear.x, msg.twist.angular.z = DRIVE
        os.killpg(bridge.pid, signal.SIGSTOP)
        try:
            cls.stopped_at = cls.states[-1]["t"]
            deadline = time.monotonic() + 30.0
            while cls.states[-1]["t"] < cls.stopped_at + 1.5 and time.monotonic() < deadline:
                probe.cmd.publish(msg)
                time.sleep(0.05)
        finally:
            os.killpg(bridge.pid, signal.SIGCONT)

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
        # The ramps up and down make some 50 updates that change a command. A change counts only where both its step
        # and the one before came in: a busy machine drops some of the 1 kHz states (CI's macOS runner, 2026-10-11).
        self.assertGreaterEqual(len(changes), 10, f"{len(sp)} states over {max(sp, default=0) - min(sp, default=0)} ms")
        self.assertEqual(sorted({t % UPDATE_STEPS for t in changes}), [1], changes[:20])

    def test_the_joint_state_and_the_motor_currents_come_back(self):
        """/joint_states (joint_state_broadcaster) carries each wheel's motor current as its effort: the drivetrain's
        current of an update step (read() runs in gz_ros2_control's PostUpdate of steps 10, 20, 30 ..., after the
        drivetrain's PreUpdate of that step wrote it). A message's stamp is the controller manager's clock, a few ms
        behind the step, so the step is found near it: the steps within 50 ms whose four currents all equal the
        efforts. Where that is one step (the currents change from step to step while the rover drives), it is an
        update step."""
        currents = {ms(s["t"]): [s["wheels"][w]["i"] for w in WHEELS] for s in self.states}
        pinned = drawn = undecided = 0
        for message in self.messages["/joint_states"]:
            effort = dict(zip(message.name, message.effort))
            got = [effort[f"wheel_{w}_joint"] for w in WHEELS]
            stamp = ms(message.header.stamp)
            window = range(stamp - 50, stamp + 51)
            if not any(t in currents for t in window):
                continue  # before the drivetrain's first state, or after its last
            if not all(t in currents for t in window):
                undecided += 1  # a busy machine dropped a state here, maybe the one that matches (1 kHz states)
                continue
            steps = [t for t in window if all(close(g, c) for g, c in zip(got, currents[t]))]
            self.assertTrue(steps, (stamp, got))
            if len(steps) == 1:
                self.assertEqual(steps[0] % UPDATE_STEPS, 0, (stamp, steps, got))
                pinned += 1
            drawn += max(abs(g) for g in got) > 1.0
        self.assertGreater(pinned, 10, f"{undecided} messages near a dropped state")  # some 100 with every state
        self.assertGreater(drawn, 0, "the motors drew current")

    def test_the_setpoint_topic_is_ignored(self):
        """While ros2_control drives the wheels RoverDrivetrain ignores its ROS-free setpoint topic, and says so once:
        the 5 rad/s sent there in the middle of the drive reach no motor."""
        self.assertEqual(self.log.count("is ignored: ros2_control's RoverSimSystem drives the wheels"), 1)
        self.assertEqual([s["t"] for s in self.states for w in WHEELS if abs(s["wheels"][w]["sp"] - 5.0) < 0.05], [])

    def test_the_drivetrain_stops_the_motors_when_the_clock_stops(self):
        """ros2_control 4.47 hands the hardware the controller manager's clock, sim time from the bridged /clock
        (spec section 12, item 5): with the bridge stopped, RoverSimSystem's stamps freeze while Gazebo runs on, and
        the drivetrain's watchdog zeroes every setpoint cmd_timeout after the last stamp, at once. The drive
        controller's own timeout cannot do it (its clock is frozen too, and it would ramp the wheels down, not drop
        them to zero); it keeps commanding the arc. Should a ros2_control update hand the hardware the step's time
        instead, the stamps would not freeze and this test would fail."""
        states = [(s["t"], [s["wheels"][w]["sp"] for w in WHEELS]) for s in self.clock_stopped
                  if self.stopped_at <= s["t"] <= self.stopped_at + 1.5]  # until the bridge went on
        self.assertTrue(states, "no drivetrain state after the clock stopped")
        stopped = [t for t, sp in states if sp == [0.0] * 4]
        self.assertTrue(stopped, f"the motors still had setpoints {states[-1]} at {states[-1][0]:.3f} s, "
                                 f"{states[-1][0] - self.stopped_at:.3f} s after the clock stopped")
        first = stopped[0]
        before = [sp for t, sp in states if t < first][-1]
        # The arc's wheels, left 0.67 rad/s and right 3.33 rad/s, up to the step before: zero at once, not a ramp.
        self.assertTrue(all(sp > 0.5 for sp in before), before)
        # The last stamp is from up to some 20 ms before the bridge stopped (the probe saw 1 to 16 ms).
        self.assertGreater(first - self.stopped_at, CMD_TIMEOUT - 0.05)
        self.assertLess(first - self.stopped_at, CMD_TIMEOUT + 0.05)
        self.assertTrue(all(sp == [0.0] * 4 for t, sp in states if t >= first), "a setpoint came back")


if __name__ == "__main__":
    unittest.main()
