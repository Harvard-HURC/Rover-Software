"""Hardware-free checks: python -m unittest -v test_yam_commands."""

import json
import socket
import time
import unittest

import numpy as np

from yam_commands import CommandServer, JointController, named_positions, parse_duration, positions


class FakeRobot:
    def __init__(self) -> None:
        self.position = np.array([0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.4])
        self.commands: list[np.ndarray] = []

    def get_joint_pos(self) -> np.ndarray:
        return self.position.copy()

    def command_joint_pos(self, value: np.ndarray) -> None:
        self.position = value.copy()
        self.commands.append(value.copy())


class ControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = FakeRobot()
        self.controller = JointController(self.robot)

    def request(self, op: str, now: float = 0.0, **fields: object) -> dict:
        return self.controller.handle(dict(fields, op=op, session="test"), now)

    def test_validation(self) -> None:
        for value in ([0] * 5, [0] * 7, [float("nan")] * 6, [float("inf")] * 6, [True] * 6, ["1"] * 6):
            with self.subTest(value=value), self.assertRaises(ValueError):
                positions(value)
        with self.assertRaises(ValueError):
            named_positions(["joint1"] * 6, [0] * 6)
        self.assertEqual(
            named_positions([f"joint{i}" for i in range(6, 0, -1)], [6, 5, 4, 3, 2, 1]),
            [1, 2, 3, 4, 5, 6],
        )

    def test_no_motion_on_start_or_query(self) -> None:
        self.request("state")
        self.controller.tick(1)
        self.assertEqual(self.robot.commands, [])

    def test_invalid_commands_never_move(self) -> None:
        for fields in (
            {"op": "jog", "joint": "joint7", "delta": 0.1},
            {"op": "jog", "joint": "joint1", "delta": 0.6},
            {"op": "jog", "joint": "joint1", "delta": float("nan")},
            {"op": "move", "position": [0, -0.1, 1, 0, 0, 0]},
            {"op": "bad"},
        ):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.request(**fields)
        self.assertEqual(self.robot.commands, [])

    def test_model_limit_rejection(self) -> None:
        self.robot.position[4] = 1.5
        with self.assertRaisesRegex(ValueError, "joint limits"):
            self.request("jog", joint="joint5", delta=0.2)

    def test_interpolation_speed_and_gripper(self) -> None:
        self.request("jog", joint="joint1", delta=0.3)
        previous = self.robot.position.copy()
        for index in range(1, 301):
            now = index * 0.01
            self.request("state", now=now)
            self.controller.tick(now)
            self.assertLessEqual(abs(self.robot.position[0] - previous[0]), 0.25 * 0.01 + 1e-9)
            self.assertEqual(self.robot.position[6], 0.4)
            previous = self.robot.position.copy()
        self.assertAlmostEqual(self.robot.position[0], 0.3)
        self.assertIsNone(self.controller.target)

    def test_reject_busy_and_hold(self) -> None:
        self.request("jog", joint="joint1", delta=0.3)
        with self.assertRaisesRegex(ValueError, "Motion in progress"):
            self.request("jog", joint="joint2", delta=0.1)
        self.controller.tick(0.1)
        before = self.robot.position.copy()
        self.request("hold", now=0.1)
        self.controller.tick(10)
        np.testing.assert_array_equal(self.robot.position, before)

    def test_requested_duration_reaches_midpoint_and_endpoint(self) -> None:
        result = self.request("jog", joint="joint1", delta=0.1, duration=1.0)
        self.assertEqual(result["duration"], 1.0)
        for index in range(1, 101):
            now = index * 0.01
            self.request("state", now=now)
            self.controller.tick(now)
            if index == 50:
                self.assertAlmostEqual(self.robot.position[0], 0.05)
                self.assertIsNotNone(self.controller.target)
        self.assertAlmostEqual(self.robot.position[0], 0.1)
        self.assertIsNone(self.controller.target)
        self.assertEqual(self.robot.position[6], 0.4)

    def test_invalid_or_too_short_duration_has_no_side_effects(self) -> None:
        for duration in (-1, float("nan"), float("inf"), True, "1", 0.1):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                self.request("jog", joint="joint1", delta=0.1, duration=duration)
            self.assertIsNone(self.controller.target)
            self.assertEqual(self.controller.status, "idle")
            self.assertEqual(self.robot.commands, [])
        # A rejected move must not leave the controller busy.
        self.request("jog", joint="joint1", delta=0.1, duration=1)

    def test_instant_keeps_speed_limit_and_long_moves_keep_lease(self) -> None:
        self.request("jog", joint="joint1", delta=0.1, duration=0)
        self.assertGreater(self.controller.duration, 0.6)
        self.controller.tick(0.01)
        self.assertLessEqual(self.robot.position[0], 0.25 * 0.01)
        self.request("hold", now=0.01)
        self.request("jog", now=0.01, joint="joint1", delta=0.1, duration=5)
        self.controller.tick(0.82)
        self.assertIn("disconnected", self.controller.status)

    def test_duration_text(self) -> None:
        self.assertIsNone(parse_duration("auto"))
        self.assertEqual(parse_duration(" Instant "), 0)
        self.assertEqual(parse_duration("1.5"), 1.5)
        for value in ("-1", "nan", "inf", "one", ""):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_duration(value)

    def test_disconnect_holds_and_other_client_cannot_renew(self) -> None:
        self.request("jog", joint="joint1", delta=0.3)
        self.controller.tick(0.1)
        before = self.robot.position.copy()
        self.controller.handle({"op": "state", "session": "other"}, 0.7)
        self.controller.tick(0.81)
        self.assertIn("disconnected", self.controller.status)
        np.testing.assert_array_equal(self.robot.position, before)

    def test_stall_does_not_jump_target(self) -> None:
        self.request("jog", joint="joint1", delta=0.3)
        self.request("state", now=0.7)
        self.controller.tick(0.7)
        self.assertLessEqual(self.robot.position[0], 0.25 * 0.05)

    def test_udp_rejects_expiry_duplicates_and_malformed_packets(self) -> None:
        server = CommandServer(0)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
                client.settimeout(1)
                address = server.socket.getsockname()

                def exchange(data: bytes) -> dict:
                    client.sendto(data, address)
                    server.poll(self.controller)
                    return json.loads(client.recv(4096))

                packet = dict(
                    op="jog", joint="joint1", delta=0.1, session="test", id="move", expires=time.monotonic() + 0.5
                )
                data = json.dumps(packet).encode()
                self.assertTrue(exchange(data)["ok"])
                self.assertIn("Duplicate", exchange(data)["error"])
                packet.update(id="expired", expires=time.monotonic() - 1)
                self.assertIn("expired", exchange(json.dumps(packet).encode())["error"])
                for data in (b"not json", b"[]", b"x" * 5000, b'{"id": []}'):
                    self.assertFalse(exchange(data)["ok"])
        finally:
            server.close()


if __name__ == "__main__":
    unittest.main()
