import json
import math
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]


class YamJointStateBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("yam_joint_state_bridge", **kwargs)
        self.declare_parameter("feedback_port", 8765)

        self.publisher = self.create_publisher(
            JointState, "/yam/joint_states", 10
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", int(self.get_parameter("feedback_port").value)))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.02, self.receive_positions)
        self.get_logger().info(
            f"Listening for i2rt joint feedback on UDP port {self.sock.getsockname()[1]}"
        )

    def receive_positions(self):
        latest = None

        # Drain queued packets; publish only the newest feedback.
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                latest = json.loads(data.decode("utf-8"))
            except BlockingIOError:
                break
            except (ValueError, UnicodeDecodeError) as exc:
                self.get_logger().warning(f"Invalid feedback packet: {exc}")
                return

        if latest is None:
            return

        positions = latest.get("position") if isinstance(latest, dict) else None
        if (not isinstance(positions, list) or len(positions) != 6
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in positions)):
            self.get_logger().warning(
                "Expected 6 finite numeric joint positions"
            )
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(value) for value in positions]
        self.publisher.publish(msg)

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YamJointStateBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
