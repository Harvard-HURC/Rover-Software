"""Run in ROS/Pixi with WSLg: python -m unittest -v test_yam_command_gui."""

import queue
import time
import unittest

try:
    import rclpy  # noqa: F401
    from PyQt5.QtWidgets import QApplication
except ImportError as exc:
    raise unittest.SkipTest("GUI tests require the ROS/Pixi environment with PyQt5") from exc

from yam_command_gui import CommandWindow
from yam_commands import load_limits


class GuiTests(unittest.TestCase):
    def setUp(self) -> None:
        self.app = QApplication.instance() or QApplication([])
        self.events = queue.Queue()
        self.commands = queue.Queue(maxsize=1)
        self.closed = False
        self.window = CommandWindow(self.events, self.commands, self.on_close)
        self.app.processEvents()

    def on_close(self) -> None:
        self.closed = True

    def tearDown(self) -> None:
        self.window.close()

    def connect(self) -> None:
        self.events.put(
            {
                "kind": "state",
                "at": time.monotonic(),
                "data": {
                    "mode": "simulation",
                    "status": "idle",
                    "position": [0, 1, 1, 0, 0, 0],
                    "max_delta": 0.5,
                    "speed": 0.25,
                    "limits": load_limits().tolist(),
                },
            }
        )
        self.window.timer.stop()
        self.window.refresh()
        self.app.processEvents()

    def test_startup_does_not_send_and_connect_populates_targets(self) -> None:
        self.assertTrue(not self.window.send_button.isEnabled())
        self.connect()
        self.assertIn("SIMULATION", self.window.mode.text())
        self.assertEqual([float(v.text()) for v in self.window.targets], [0, 1, 1, 0, 0, 0])
        self.assertTrue(self.commands.empty())

    def test_send_and_jog_are_explicit_and_bounded(self) -> None:
        self.connect()
        self.window.targets[0].setText("0.1")
        self.assertTrue(self.commands.empty())
        self.window.send_button.click()
        request = self.commands.get_nowait()
        self.assertEqual(request["op"], "move")
        self.assertEqual(request["fields"]["position"], [0.1, 1, 1, 0, 0, 0])
        self.assertTrue(not self.window.send_button.isEnabled())

    def test_invalid_values_and_stale_feedback_cannot_send(self) -> None:
        self.connect()
        for target in ("nan", "inf", "bad", "3.5", "0.6"):
            self.window.targets[0].setText(target)
            self.window.send_targets()
            self.assertTrue(self.commands.empty())
        self.window.step.setCurrentText("nan")
        self.window.jog(0, 1)
        self.assertTrue(self.commands.empty())
        self.window.received_at = time.monotonic() - 2
        self.window.step.setCurrentText("0.05")
        self.window.jog(0, 1)
        self.assertTrue(self.commands.empty())
        self.window.update_controls()
        self.assertTrue(not self.window.send_button.isEnabled())

    def test_hold_replaces_unsent_jog(self) -> None:
        self.connect()
        self.window.jog_buttons[1][0].click()
        self.window.hold_button.click()
        self.assertEqual(self.commands.get_nowait()["op"], "hold")
        self.assertTrue(self.commands.empty())

    def test_duration_applies_to_moves_and_jogs(self) -> None:
        self.connect()
        self.window.duration.setCurrentText("1")
        self.window.targets[0].setText("0.1")
        self.window.send_button.click()
        self.assertEqual(self.commands.get_nowait()["fields"]["duration"], 1)
        self.events.put({"kind": "result"})
        self.window.refresh()
        self.window.duration.setCurrentText("instant")
        self.window.jog_buttons[0][1].click()
        self.assertEqual(self.commands.get_nowait()["fields"]["duration"], 0)

    def test_invalid_duration_blocks_motion_but_not_hold(self) -> None:
        self.connect()
        self.window.targets[0].setText("0.1")
        for value in ("nan", "-1", "bad", "0.01"):
            self.window.duration.setCurrentText(value)
            self.window.send_button.click()
            self.window.jog_buttons[0][1].click()
            self.assertTrue(self.commands.empty())
        self.window.hold_button.click()
        self.assertEqual(self.commands.get_nowait()["op"], "hold")

    def test_feedback_does_not_overwrite_edits_and_close_signals_owner(self) -> None:
        self.connect()
        self.window.targets[0].setText("0.2")
        self.connect()
        self.assertEqual(self.window.targets[0].text(), "0.2")
        self.window.copy_button.click()
        self.assertEqual(float(self.window.targets[0].text()), 0)
        self.window.close()
        self.assertTrue(self.closed)


if __name__ == "__main__":
    unittest.main()
