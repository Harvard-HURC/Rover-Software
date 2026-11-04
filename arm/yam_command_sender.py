"""ROS 2 node and interactive terminal for the YAM command bridge."""

import queue
import select
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from yam_command_help import COMMAND_SUMMARY, COMMAND_USAGE
from yam_commands import COMMAND_PORT, CommandClient, named_positions, parse_duration

HELP = COMMAND_SUMMARY


def parse_command(line: str) -> tuple[str, dict]:
    parts = line.split()
    if not parts:
        return "help", {}
    operation = parts[0].lower()
    timing = {}
    if operation in ("move", "jog") and len(parts) >= 3 and parts[-2] == "--duration":
        timing["duration"] = parse_duration(parts[-1])
        parts = parts[:-2]
    if operation in ("state", "hold", "commands", "help", "quit") and len(parts) == 1:
        return operation, {}
    if operation == "move" and len(parts) == 7:
        return "move", {"position": [float(value) for value in parts[1:]], **timing}
    if operation == "jog" and len(parts) == 3:
        return "jog", {"joint": parts[1], "delta": float(parts[2]), **timing}
    raise ValueError(HELP)


class YamCommandSender(Node):
    def __init__(self) -> None:
        super().__init__("yam_command_sender")
        self.declare_parameter("command_port", COMMAND_PORT)
        self.declare_parameter("interactive", True)
        self.declare_parameter("gui", False)
        self.declare_parameter("move_duration", 0.0)
        self.gui = bool(self.get_parameter("gui").value)
        self.client = CommandClient(int(self.get_parameter("command_port").value))
        self.interactive = bool(self.get_parameter("interactive").value) and not self.gui
        self.ui_events: queue.Queue = queue.Queue(maxsize=100)
        self.ui_commands: queue.Queue = queue.Queue(maxsize=1)
        self.finished = False
        self.owns_motion = False
        self.last_status = ""
        self.status_publisher = self.create_publisher(String, "/yam/command_status", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(JointState, "/yam/joint_commands", self.on_target, qos)
        self.hold_service = self.create_service(Trigger, "/yam/hold", self.on_hold)
        self.heartbeat = self.create_timer(0.2, self.poll_state)
        self.console = self.create_timer(0.05, self.poll_console) if self.interactive else None
        self.gui_timer = self.create_timer(0.02, self.poll_gui_commands) if self.gui else None
        self.get_logger().info(HELP)
        self.get_logger().info("Joint targets: /yam/joint_commands; hold: /yam/hold; status: /yam/command_status")

    def report(self, text: str) -> None:
        self.emit_ui({"kind": "log", "text": text})
        self.status_publisher.publish(String(data=text))
        self.get_logger().info(text)

    def command(self, operation: str, **fields: object) -> dict | None:
        try:
            # An acknowledgement may be lost after acceptance; still hold on exit.
            if operation in ("move", "jog"):
                self.owns_motion = True
            response = self.client.request(operation, **fields)
            self.emit_ui({"kind": "state", "data": response, "at": time.monotonic()})
            if operation == "hold":
                self.owns_motion = False
            timing = ""
            if operation in ("move", "jog") and "duration" in response:
                timing = f"; target duration: {response['duration']:.3f} s"
            self.report(f"{response['status']}{timing}; joints (rad): {response['position']}")
            return response
        except (ValueError, ConnectionError, OSError) as exc:
            self.report(f"Rejected/unconfirmed: {exc}")
            return None

    def on_target(self, message: JointState) -> None:
        try:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            # Physical hardware commands use wall time, even when other nodes use simulation time.
            age = time.time() - stamp
            if not 0 <= age <= 0.5:
                raise ValueError("Joint command needs a current wall-clock header stamp (at most 0.5 seconds old)")
            if message.velocity or message.effort:
                raise ValueError("Only position commands are supported; leave velocity and effort empty")
            self.command(
                "move",
                position=named_positions(list(message.name), list(message.position)),
                duration=self.get_parameter("move_duration").value,
            )
        except ValueError as exc:
            self.report(f"Rejected: {exc}")

    def on_hold(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        result = self.command("hold")
        response.success = result is not None
        response.message = "Holding current position" if result else "Hold not acknowledged; check sender"
        return response

    def poll_state(self) -> None:
        try:
            response = self.client.request("state")
            self.emit_ui({"kind": "state", "data": response, "at": time.monotonic()})
            status = response["status"]
        except (ValueError, ConnectionError, OSError) as exc:
            status = str(exc)
            self.emit_ui({"kind": "disconnected", "text": status})
        if status != self.last_status:
            self.report(status)
            self.last_status = status

    def emit_ui(self, event: dict) -> None:
        if not self.gui:
            return
        try:
            self.ui_events.put_nowait(event)
        except queue.Full:
            try:
                self.ui_events.get_nowait()
            except queue.Empty:
                pass
            self.ui_events.put_nowait(event)

    def poll_gui_commands(self) -> None:
        try:
            request = self.ui_commands.get_nowait()
        except queue.Empty:
            return
        if request["op"] != "hold" and time.monotonic() > request["expires"]:
            self.report("Rejected: GUI command expired before dispatch")
        else:
            self.command(request["op"], **request["fields"])
        self.emit_ui({"kind": "result"})

    def poll_console(self) -> None:
        if not select.select([sys.stdin], [], [], 0)[0]:
            return
        line = sys.stdin.readline()
        if not line:
            self.finished = True
            return
        try:
            operation, fields = parse_command(line)
            if operation in ("commands", "help"):
                self.report(COMMAND_USAGE)
            elif operation == "quit":
                self.finished = True
            else:
                self.command(operation, **fields)
        except ValueError as exc:
            self.report(str(exc))

    def close(self) -> None:
        if self.owns_motion:
            try:
                self.client.request("hold")
            except (ValueError, ConnectionError, OSError):
                # The sender's independent lease also holds when this process disappears.
                pass
        self.destroy_node()


def main() -> None:
    # rclpy parses ROS CLI parameters/remapping; no separate argument parser is needed.
    rclpy.init()
    node = None
    try:
        node = YamCommandSender()
        if node.gui:
            from yam_command_gui import run_gui

            run_gui(node)
        else:
            while rclpy.ok() and not node.finished:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
