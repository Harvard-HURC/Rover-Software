"""Exercise UDP feedback -> ROS on an ephemeral port, without hardware."""

import json
import socket
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from yam_ros_bridge import JOINT_NAMES, YamJointStateBridge


class FeedbackTests(unittest.TestCase):
    def test_feedback_and_bad_packets(self):
        rclpy.init(args=[])
        bridge = YamJointStateBridge(parameter_overrides=[Parameter("feedback_port", value=0)])
        peer = rclpy.create_node("yam_feedback_test")
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(peer)
        messages = []
        subscription = peer.create_subscription(JointState, "/yam/joint_states", messages.append, 10)
        try:
            deadline = time.monotonic() + 5
            while bridge.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            self.assertGreater(bridge.publisher.get_subscription_count(), 0)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                address = bridge.sock.getsockname()
                for payload in ([], {}, {"position": None}, {"position": [0] * 5},
                                {"position": [True] * 6}, {"position": [float("nan")] * 6}):
                    sender.sendto(json.dumps(payload).encode(), address)
                    bridge.receive_positions()
                self.assertEqual(messages, [])
                sender.sendto(b'{"position": [0, 1, 1, 0, 0, 0]}', address)
                deadline = time.monotonic() + 5
                while not messages and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.05)
                self.assertTrue(messages)
                self.assertEqual(list(messages[-1].name), JOINT_NAMES)
                self.assertEqual(list(messages[-1].position), [0, 1, 1, 0, 0, 0])
                self.assertGreater(messages[-1].header.stamp.sec, 0)
        finally:
            executor.shutdown()
            peer.destroy_subscription(subscription)
            peer.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
