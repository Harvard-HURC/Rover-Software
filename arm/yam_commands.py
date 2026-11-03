"""Local command protocol and bounded motion shared by the ROS and hardware environments."""

import json
import math
import socket
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
COMMAND_PORT = 8766
MAX_PACKET = 4096
LEASE_SECONDS = 0.8


def parse_duration(value: str) -> float | None:
    """Parse user-facing seconds; auto keeps the default, instant requests the fastest ramp."""
    value = value.strip().lower()
    if value == "auto":
        return None
    if value == "instant":
        return 0.0
    try:
        seconds = float(value)
    except ValueError as exc:
        raise ValueError("Duration must be seconds >= 0, auto, or instant") from exc
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError("Duration must be finite seconds >= 0, auto, or instant")
    return seconds


def motion_duration(distance: float, speed: float, requested: object = None) -> float:
    """Resolve a half-cosine ramp without exceeding the configured peak target speed."""
    minimum = max(0.2, math.pi * distance / (2 * speed))
    if requested is None:
        return minimum
    if type(requested) not in (int, float) or not math.isfinite(requested) or requested < 0:
        raise ValueError("Duration must be finite seconds >= 0")
    if requested == 0:
        return minimum
    if requested < minimum:
        suggested = math.ceil(minimum * 1000) / 1000
        raise ValueError(f"Duration too short: use at least {suggested:.3f} seconds at the current speed limit")
    return float(requested)


def positions(values: Any, count: int = 6) -> np.ndarray:
    if not isinstance(values, (list, tuple, np.ndarray)) or len(values) != count:
        raise ValueError(f"Expected {count} joint positions")
    if any(isinstance(v, (bool, str)) for v in values):
        raise ValueError("Positions must be numbers")
    result = np.asarray(values, dtype=float)
    if result.shape != (count,) or not np.isfinite(result).all():
        raise ValueError("Positions must be finite numbers")
    return result


def named_positions(names: list[str], values: list[float]) -> list[float]:
    result = positions(values)
    if len(names) != 6 or set(names) != set(JOINT_NAMES):
        raise ValueError("Name each of joint1 through joint6 exactly once")
    return [float(result[names.index(name)]) for name in JOINT_NAMES]


def load_limits() -> np.ndarray:
    model = Path(__file__).parent / "vendor/i2rt/i2rt/robot_models/arm/yam/v1/yam.urdf"
    if not model.is_file():
        raise FileNotFoundError("YAM model is missing. Run: git submodule update --init --recursive")
    root = ET.parse(model).getroot()
    limits = []
    for name in JOINT_NAMES:
        limit = root.find(f"./joint[@name='{name}']/limit")
        if limit is None:
            raise ValueError(f"Missing model limits for {name}")
        limits.append([float(limit.attrib["lower"]), float(limit.attrib["upper"])])
    return np.asarray(limits)


class JointController:
    """Single-threaded controller; caller retains sole ownership of the robot."""

    def __init__(self, robot: Any, max_delta: float = 0.5, speed: float = 0.25, sim: bool = False) -> None:
        if not math.isfinite(max_delta) or not 0 < max_delta <= 0.5:
            raise ValueError("max_delta must be in (0, 0.5] radians")
        if not math.isfinite(speed) or not 0 < speed <= 0.25:
            raise ValueError("speed must be in (0, 0.25] radians/second")
        self.robot = robot
        self.max_delta = max_delta
        self.speed = speed
        self.mode = "simulation" if sim else "hardware"
        self.limits = load_limits()
        self.target: np.ndarray | None = None
        self.owner = ""
        self.last_contact = 0.0
        self.last_tick = time.monotonic()
        self.elapsed = 0.0
        self.duration = 0.0
        self.start = np.zeros(7)
        self.status = "idle"

    def current(self) -> np.ndarray:
        return positions(self.robot.get_joint_pos(), 7).copy()

    def hold(self, reason: str = "holding") -> None:
        self.robot.command_joint_pos(self.current())
        self.target = None
        self.status = reason

    def handle(self, request: dict, now: float) -> dict:
        operation = request.get("op")
        session = request.get("session")
        if not isinstance(session, str) or not session or len(session) > 64:
            raise ValueError("Invalid session")
        if operation == "hold":
            self.hold()
        elif operation == "state":
            if session == self.owner:
                self.last_contact = now
        elif operation in ("move", "jog"):
            if self.target is not None:
                raise ValueError("Motion in progress; hold before sending another target")
            current = self.current()
            target = current.copy()  # Preserve the gripper position throughout the move.
            if operation == "move":
                target[:6] = positions(request.get("position"))
            else:
                joint = request.get("joint")
                if joint not in JOINT_NAMES:
                    raise ValueError("Joint must be joint1 through joint6")
                target[JOINT_NAMES.index(joint)] += positions([request.get("delta")], 1)[0]
            if np.any(np.abs(target[:6] - current[:6]) > self.max_delta + 1e-9):
                raise ValueError(f"Maximum change per command is {self.max_delta} radians per joint")
            if np.any(target[:6] < self.limits[:, 0]) or np.any(target[:6] > self.limits[:, 1]):
                raise ValueError("Target exceeds YAM joint limits")
            if np.any(current[:6] < self.limits[:, 0] - 0.01) or np.any(current[:6] > self.limits[:, 1] + 0.01):
                raise ValueError("Current arm position is outside the model limits")
            # Half-cosine interpolation has peak speed pi*distance/(2*duration).
            distance = float(np.max(np.abs(target[:6] - current[:6])))
            duration = motion_duration(distance, self.speed, request.get("duration"))
            self.start = current
            self.target = target
            self.duration = duration
            self.elapsed = 0.0
            self.last_tick = now
            self.last_contact = now
            self.owner = session
            self.status = "moving"
        else:
            raise ValueError("Unknown command")
        return {
            "ok": True,
            "status": self.status,
            "position": self.current()[:6].tolist(),
            "mode": self.mode,
            "max_delta": self.max_delta,
            "speed": self.speed,
            "limits": self.limits.tolist(),
            "duration": self.duration,
            "elapsed": self.elapsed,
        }

    def tick(self, now: float) -> None:
        dt = max(0.0, min(now - self.last_tick, 0.05))
        self.last_tick = now
        if self.target is None:
            return
        if now - self.last_contact > LEASE_SECONDS:
            self.hold("holding: command node disconnected")
            return
        self.elapsed = min(self.duration, self.elapsed + dt)
        alpha = (1 - math.cos(math.pi * self.elapsed / self.duration)) / 2
        self.robot.command_joint_pos(self.start + alpha * (self.target - self.start))
        if self.elapsed >= self.duration:
            self.target = None
            self.status = "holding: target sent"


class CommandServer:
    def __init__(self, port: int = COMMAND_PORT) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(("127.0.0.1", port))
        except BaseException:
            self.socket.close()
            raise
        self.socket.setblocking(False)
        self.seen: dict[str, float] = {}

    def poll(self, controller: JointController) -> None:
        # Bound work per tick so incoming packets cannot starve motion/feedback.
        for _ in range(16):
            try:
                data, address = self.socket.recvfrom(MAX_PACKET + 1)
            except BlockingIOError:
                return
            request_id = None
            try:
                if len(data) > MAX_PACKET:
                    raise ValueError("Command packet too large")
                request = json.loads(data)
                if not isinstance(request, dict):
                    raise ValueError("Expected a command object")
                request_id = request.get("id")
                if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                    raise ValueError("Invalid command ID")
                deadline = request.get("expires")
                # Both processes share a WSL kernel; clock synchronization must not expire commands.
                now = time.monotonic()
                if type(deadline) not in (int, float) or not math.isfinite(deadline) or not now < deadline <= now + 1:
                    raise ValueError("Command expired or has an invalid deadline")
                self.seen = {key: expiry for key, expiry in self.seen.items() if expiry > now}
                if request_id in self.seen:
                    raise ValueError("Duplicate command; not repeated")
                self.seen[request_id] = deadline
                response = controller.handle(request, time.monotonic())
            except (ValueError, TypeError, OverflowError) as exc:
                response = {"ok": False, "error": str(exc)}
            response["id"] = request_id
            self.socket.sendto(json.dumps(response, allow_nan=False).encode(), address)

    def close(self) -> None:
        self.socket.close()


class CommandClient:
    def __init__(self, port: int = COMMAND_PORT) -> None:
        self.port = port
        self.session = uuid.uuid4().hex

    def request(self, operation: str, **fields: Any) -> dict:
        request_id = uuid.uuid4().hex
        request = dict(fields, op=operation, session=self.session, id=request_id, expires=time.monotonic() + 0.5)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(0.5)
            sock.connect(("127.0.0.1", self.port))
            sock.send(json.dumps(request, allow_nan=False).encode())
            try:
                response = json.loads(sock.recv(MAX_PACKET))
            except OSError as exc:
                raise ConnectionError("No acknowledgement from yam_state_sender.py --enable-commands") from exc
        if response.get("id") != request_id:
            raise ConnectionError("Unexpected command acknowledgement")
        if not response.get("ok"):
            raise ValueError(response.get("error", "Command rejected"))
        return response
