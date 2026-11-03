"""Run in the ROS/Pixi environment; only starts the hardware sender with --sim."""

import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

try:
    import rclpy
    from PyQt5.QtWidgets import QApplication
except ImportError as exc:
    raise unittest.SkipTest("Run this integration test inside the ROS/Pixi environment") from exc

from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from yam_command_gui import CommandWindow
from yam_command_sender import YamCommandSender, parse_command
from yam_commands import JOINT_NAMES, CommandClient


class RosIntegrationTests(unittest.TestCase):
    def test_duration_command_syntax(self) -> None:
        self.assertEqual(
            parse_command("jog joint1 0.1 --duration 1"),
            ("jog", {"joint": "joint1", "delta": 0.1, "duration": 1.0}),
        )
        self.assertEqual(parse_command("move 0 1 1 0 0 0 --duration instant")[1]["duration"], 0)
        for line in ("jog joint1 0.1 --duration", "jog joint1 0.1 --duration nan", "state --duration 1"):
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_command(line)

    def test_ros_to_simulated_arm(self) -> None:
        root = Path(__file__).resolve().parent
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
            reserve.bind(("127.0.0.1", 0))
            port = reserve.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as feedback, tempfile.TemporaryFile() as log:
            feedback.bind(("127.0.0.1", 0))
            feedback.settimeout(5)
            env = os.environ.copy()
            env.pop("PYTHONPATH", None)
            process = subprocess.Popen(
                [
                    str(root / "vendor/i2rt/.venv/bin/python"),
                    str(root / "yam_state_sender.py"),
                    "--sim",
                    "--enable-commands",
                    "--command-port",
                    str(port),
                    "--feedback-port",
                    str(feedback.getsockname()[1]),
                ],
                cwd=root,
                env=env,
                stdout=log,
                stderr=log,
            )
            node = None
            peer = None
            executor = None
            window = None
            try:
                probe = CommandClient(port)
                deadline = time.monotonic() + 40
                while True:
                    try:
                        probe.request("state")
                        break
                    except (ConnectionError, OSError, ValueError):
                        # During model startup the bound socket may still contain expired probes.
                        if time.monotonic() > deadline or process.poll() is not None:
                            log.seek(0)
                            self.fail(log.read().decode())
                        time.sleep(0.1)
                self.assertIn(b'"position"', feedback.recv(4096))
                rclpy.init(
                    args=["--ros-args", "-p", "interactive:=false", "-p", "gui:=true", "-p", f"command_port:={port}"]
                )
                node = YamCommandSender()
                gui_app = QApplication.instance() or QApplication([])
                window = CommandWindow(node.ui_events, node.ui_commands, lambda: None)
                peer = rclpy.create_node("yam_command_test")
                executor = SingleThreadedExecutor()
                executor.add_node(node)
                executor.add_node(peer)

                def spin(seconds: float) -> None:
                    until = time.monotonic() + seconds
                    while time.monotonic() < until:
                        executor.spin_once(timeout_sec=0.02)
                        gui_app.processEvents()

                operation, fields = parse_command("jog joint1 0.1 --duration 1")
                self.assertEqual(node.command(operation, **fields)["duration"], 1)
                spin(1.5)
                self.assertAlmostEqual(probe.request("state")["position"][0], 0.1, places=4)
                publisher = peer.create_publisher(JointState, "/yam/joint_commands", 1)
                spin(1)
                self.assertGreater(publisher.get_subscription_count(), 0)
                target = probe.request("state")["position"]
                target[0] = 0.2
                msg = JointState(name=JOINT_NAMES, position=target)
                # Unstamped/old commands must never move the arm.
                publisher.publish(msg)
                spin(0.3)
                self.assertAlmostEqual(probe.request("state")["position"][0], 0.1, places=4)
                msg.header.stamp = peer.get_clock().now().to_msg()
                self.assertTrue(node.set_parameters([Parameter("move_duration", value=1.0)])[0].successful)
                publisher.publish(msg)
                spin(1.5)
                self.assertAlmostEqual(probe.request("state")["position"][0], 0.2, places=4)
                self.assertEqual(probe.request("state")["duration"], 1.0)

                self.assertIsNotNone(node.command("jog", joint="joint1", delta=0.3))
                hold = peer.create_client(Trigger, "/yam/hold")
                self.assertTrue(hold.wait_for_service(timeout_sec=3))
                future = hold.call_async(Trigger.Request())
                spin(0.4)
                self.assertTrue(future.done())
                self.assertTrue(future.result().success)
                before = probe.request("state")["position"][0]
                spin(0.5)
                self.assertAlmostEqual(probe.request("state")["position"][0], before)

                self.assertIsNotNone(node.command("jog", joint="joint1", delta=0.3))
                # Simulate node failure by stopping its executor/heartbeats.
                time.sleep(1.1)
                self.assertIn("disconnected", probe.request("state")["status"])
                spin(0.4)
                self.assertIn("SIMULATION", window.mode.text())
                before = probe.request("state")["position"][0]
                window.step.setCurrentText("0.025")
                window.duration.setCurrentText("0.5")
                window.jog_buttons[0][1].click()
                spin(1)
                self.assertAlmostEqual(probe.request("state")["position"][0], before + 0.025, places=4)
                window.copy_button.click()
                window.targets[0].setText(str(before + 0.05))
                window.send_button.click()
                spin(1)
                self.assertAlmostEqual(probe.request("state")["position"][0], before + 0.05, places=4)
                window.step.setCurrentText("0.3")
                window.duration.setCurrentText("instant")
                window.jog_buttons[0][1].click()
                spin(0.2)
                window.hold_button.click()
                spin(0.3)
                self.assertEqual(probe.request("state")["status"], "holding")
                console = subprocess.run(
                    [
                        sys.executable,
                        str(root / "yam_command_sender.py"),
                        "--ros-args",
                        "-r",
                        "__node:=yam_command_console_test",
                        "-p",
                        f"command_port:={port}",
                    ],
                    input="state\nhelp\nquit\n",
                    text=True,
                    capture_output=True,
                    timeout=30,
                    check=False,
                )
                self.assertEqual(console.returncode, 0, console.stdout + console.stderr)
                self.assertIn("joints (rad)", console.stdout + console.stderr)
            finally:
                if window:
                    window.close()
                if executor:
                    executor.shutdown()
                if node:
                    node.close()
                if peer:
                    peer.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
