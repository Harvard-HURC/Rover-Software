"""ROS 2 runs for the tests: processes in sessions of their own, stopped with everything they started, and a probe
node in this process that publishes /cmd_vel and records what comes back, all in this process's private ROS domain
and gz partition (simulate.py sets both, and gzenv's Gazebo environment, when it is imported)."""
import contextlib
import importlib.util
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import simulate  # noqa: F401  (sets GZ_PARTITION, GZ_IP, ROS_DOMAIN_ID and Gazebo's paths before rclpy starts)

import rclpy  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

import rover_config  # noqa: E402

STOP_TIMEOUT = 30.0  # [s] for a process group to end on SIGINT before what is left of it is killed


def start(cmd, log, env=None):
    """A process in a session of its own (so stop() reaches what it starts), its output in the file `log`."""
    with open(log, "w") as out:
        return subprocess.Popen([str(c) for c in cmd], stdout=out, stderr=subprocess.STDOUT, env=env or os.environ,
                                start_new_session=True)


def _alive(group):
    try:
        os.killpg(group, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def stop(process, timeout=STOP_TIMEOUT):
    """SIGINT to the process's group, SIGKILL to what is left of it after `timeout`; the process's exit code. A
    process that has ended (the spawner, once its controllers are active) is reaped first: macOS refuses a signal to a
    group whose only member is a zombie with EPERM, which therefore counts as a group that has ended."""
    group = process.pid
    process.poll()
    with contextlib.suppress(ProcessLookupError, PermissionError):
        os.killpg(group, signal.SIGINT)
    deadline = time.monotonic() + timeout
    while (process.poll() is None or _alive(group)) and time.monotonic() < deadline:
        time.sleep(0.2)
    if process.poll() is None or _alive(group):
        with contextlib.suppress(ProcessLookupError, PermissionError):
            os.killpg(group, signal.SIGKILL)
        process.wait()
    return process.returncode


def spawner(controller="drive_controller", extra_params=(), timeout=180):
    """The spawner that rover_control's launch/controllers.launch.py runs (its spawner_arguments), with rover_control's
    config/controllers.yaml and then extra_params, which override it."""
    spec = importlib.util.spec_from_file_location("controllers_launch",
                                                  rover_config.CONTROL_DIR / "launch" / "controllers.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    arguments = module.spawner_arguments(controller, str(rover_config.CONTROLLERS_YAML), str(timeout))
    arguments[4:4] = [arg for path in extra_params for arg in ("--param-file", str(path))]  # after controllers.yaml
    return ["ros2", "run", "controller_manager", "spawner", *arguments]


def shutdown():
    """rclpy off (a test module's tearDownModule)."""
    if rclpy.ok():
        rclpy.shutdown()


class Probe:
    """A node in this process: publishes TwistStamped commands on /cmd_vel with a zero stamp (the drive controller
    stamps them on arrival with its own clock, docs/topics.md) and records /odom, /joint_states and `topics`
    [(name, message type)] in `messages`."""

    def __init__(self, topics=()):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(f"rover_sim_test_{os.getpid()}")
        self.cmd = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.messages = {}
        for topic, kind in [("/odom", Odometry), ("/joint_states", JointState), *topics]:
            self.messages[topic] = []
            self.node.create_subscription(kind, topic, self.messages[topic].append, 100)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def sim_time(self):
        """The latest /odom's stamp [s]: sim time on the controller manager's clock (0 before the first)."""
        odom = self.messages["/odom"]
        return odom[-1].header.stamp.sec + odom[-1].header.stamp.nanosec * 1e-9 if odom else 0.0

    def drive_for(self, vx, wz, sim_seconds, timeout=60.0, rate=20.0, publish=True):
        """Publish the command `rate` times a second until /odom's stamps have advanced `sim_seconds` [s] of sim time,
        or `timeout` [s] of wall time has passed: a machine that runs the simulation slower than real time (CI's
        runners) drives as far as a fast one. publish=False only waits (the drive controller's timeout brakes)."""
        msg = TwistStamped()
        msg.twist.linear.x, msg.twist.angular.z = float(vx), float(wz)
        start, end = self.sim_time(), time.monotonic() + timeout
        while self.sim_time() - start < sim_seconds and time.monotonic() < end:
            if publish:
                self.cmd.publish(msg)
            time.sleep(1.0 / rate)

    def wait(self, condition, timeout, what, logs=()):
        """Wait until condition() is true; after `timeout` [s] fail with the tails of the files `logs`."""
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                tails = "\n".join(f"--- {log}\n{Path(log).read_text(errors='replace')[-3000:]}" for log in logs)
                raise AssertionError(f"no {what} within {timeout:.0f} s\n{tails}")
            time.sleep(0.2)

    def topics(self):
        """The names of the topics in the ROS graph."""
        return {name for name, _ in self.node.get_topic_names_and_types()}

    def close(self):
        self.executor.shutdown()
        self.thread.join(5)
        self.node.destroy_node()
