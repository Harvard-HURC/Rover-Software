"""rover.launch.py, the real rover's ROS 2 stack, with the hardware stub (design spec 2026-10-09, 6.3), and
controllers.launch.py, which it shares with the simulation: what they start, and a run in a private ROS domain: the
controllers come up, /joint_states reports zero motion, a /cmd_vel command reaches the stub, which logs what it would
send, and /odom stays at the origin. CTest runs it with this package's install on AMENT_PREFIX_PATH
(CMakeLists.txt)."""
import contextlib
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
JOINTS = {'wheel_fl_joint', 'wheel_rl_joint', 'wheel_fr_joint', 'wheel_rr_joint', 'rocker_left_joint',
          'rocker_right_joint', 'camera_pan_joint', 'camera_tilt_joint'}
DOMAIN = str(50 + os.getpid() % 50)  # a private ROS domain, as rover_sim's tests take (gzenv.private_ros_domain)


def stop(process, timeout=30.0):
    """SIGINT to the launch's process group, SIGKILL to what is left of it after `timeout` [s]. A launch that has
    already ended is reaped first: macOS refuses a signal to a group whose only member is a zombie with EPERM, which
    would hide the test's own failure (rover_sim's tests/rosstack.py stop() has the same rule)."""
    def alive():
        try:
            os.killpg(process.pid, 0)
            return True
        except (ProcessLookupError, PermissionError):
            return False

    process.poll()
    with contextlib.suppress(ProcessLookupError, PermissionError):
        os.killpg(process.pid, signal.SIGINT)
    deadline = time.monotonic() + timeout
    while (process.poll() is None or alive()) and time.monotonic() < deadline:
        time.sleep(0.2)
    if process.poll() is None or alive():
        with contextlib.suppress(ProcessLookupError, PermissionError):
            os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def load(path, name):
    """Import a launch file."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def defaults(description):
    from launch.actions import DeclareLaunchArgument
    return {e.name: e.default_value[0].text for e in description.entities
            if isinstance(e, DeclareLaunchArgument)}


class Description(unittest.TestCase):

    def test_rover_launch_starts_the_description_the_manager_and_the_controllers(self):
        from launch.actions import IncludeLaunchDescription
        from launch_ros.actions import Node
        description = load(PACKAGE / 'launch' / 'rover.launch.py', 'rover_launch').generate_launch_description()
        found = defaults(description)
        self.assertEqual(found['controller'], 'drive_controller')
        self.assertEqual(found['hardware'], 'rover_control/RoverHardwareStub')
        self.assertTrue(found['rover_yaml'].endswith(os.path.join('config', 'rover.yaml')))
        self.assertEqual([e.node_executable for e in description.entities if isinstance(e, Node)],
                         ['robot_state_publisher', 'ros2_control_node'])
        self.assertEqual(len([e for e in description.entities
                              if isinstance(e, IncludeLaunchDescription)]), 1)

    def test_controllers_launch_spawns_both_with_the_contracts_topics(self):
        module = load(PACKAGE / 'launch' / 'controllers.launch.py', 'controllers_launch')
        found = defaults(module.generate_launch_description())
        self.assertEqual(found['controller'], 'drive_controller')
        self.assertEqual(found['controllers'], str(PACKAGE / 'config' / 'controllers.yaml'))
        self.assertEqual(module.spawner_arguments('team_drive', '/c.yaml', '60'), [
            'joint_state_broadcaster', 'team_drive', '--param-file', '/c.yaml',
            '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60',
            '--controller-ros-args=-r /team_drive/cmd_vel:=/cmd_vel -r /team_drive/odom:=/odom'])


class Run(unittest.TestCase):
    """ros2 launch rover_control rover.launch.py in a private ROS domain, on this machine only."""

    def test_the_stub_drives_nothing_and_says_so(self):
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        os.environ.update(ROS_DOMAIN_ID=DOMAIN, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        got = {'/joint_states': [], '/odom': []}
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'launch.log'
            with open(log, 'w') as out:
                launch = subprocess.Popen(['ros2', 'launch', 'rover_control', 'rover.launch.py'],
                                          stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('test_rover_launch')
            node.create_subscription(JointState, '/joint_states', got['/joint_states'].append, 10)
            node.create_subscription(Odometry, '/odom', got['/odom'].append, 10)
            cmd = node.create_publisher(TwistStamped, '/cmd_vel', 10)
            try:
                deadline = time.monotonic() + 90
                while not (got['/joint_states'] and got['/odom'] and cmd.get_subscription_count()):
                    self.assertLess(time.monotonic(), deadline, log.read_text())
                    rclpy.spin_once(node, timeout_sec=0.1)
                msg = TwistStamped()
                msg.twist.linear.x = 0.5
                end = time.monotonic() + 2.5
                while time.monotonic() < end:
                    cmd.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.05)
            finally:
                node.destroy_node()
                rclpy.shutdown()
                stop(launch)
            text = log.read_text()
        self.assertIn('No motors are connected: RoverHardwareStub reports zero motion', text)
        self.assertIn('No motors are connected: would send wheel_fl_joint 3.33', text)  # 0.5 m/s / 0.15 m
        joints = got['/joint_states'][-1]
        self.assertEqual(set(joints.name), JOINTS)
        self.assertEqual(set(joints.position), {0.0})
        self.assertEqual(got['/odom'][-1].pose.pose.position.x, 0.0)


if __name__ == '__main__':
    unittest.main()
