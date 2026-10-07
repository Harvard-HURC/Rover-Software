"""What the driver's keys and sticks mean: target speeds, acceleration
limits, the deadman and where the cameras look. No transport or web code
here, so it can be unit-tested (sim/tests/test_station.py)."""
import math
from dataclasses import dataclass

import gen_model

ROVER = gen_model.Params()
# Speed presets 1/2/3: (linear [m/s], angular [rad/s]) at full stick.
PRESETS = {1: (0.25, 0.4), 2: (0.6, 0.8), 3: (1.2, 1.2)}
FAST = 2.0  # Shift multiplies the preset, up to what the rover can do:
MAX_LINEAR = ROVER.wheel_speed * ROVER.wheel_radius  # [m/s] wheel joint velocity limit
MAX_ANGULAR = 1.5  # [rad/s] spinning faster than this is not drivable from a camera
# Acceleration limits [m/s^2, rad/s^2]: gentle speeding up, firmer slowing down.
ACCEL = (1.0, 2.0)
DECEL = (2.0, 4.0)
DEADMAN = 0.5  # [s] without input from the page -> zero twist
DEADZONE = 0.12  # of a gamepad stick
# KeyboardEvent.code -> action; the page sends the gamepad's fast (RT) and
# stop (B) buttons as GamepadFast and GamepadBrake.
KEYS = {"KeyW": "forward", "KeyS": "back", "KeyA": "left", "KeyD": "right", "Space": "brake",
        "ShiftLeft": "fast", "ShiftRight": "fast", "GamepadFast": "fast", "GamepadBrake": "brake"}


def stick(value):
    """A gamepad axis with its dead zone removed, rescaled to -1..1."""
    if abs(value) <= DEADZONE:
        return 0.0
    return math.copysign(min(1.0, (abs(value) - DEADZONE) / (1 - DEADZONE)), value)


@dataclass(frozen=True)
class Command:
    """One input message from the page: held key codes, and the left stick
    (throttle forward +, turn left +) if a gamepad is connected."""
    keys: frozenset = frozenset()
    axes: tuple = (0.0, 0.0)

    @property
    def actions(self):
        return {KEYS[k] for k in self.keys if k in KEYS}


def target(command, preset):
    """Target (vx [m/s], wz [rad/s]) for a command: W/S and A/D are full stick,
    keys and stick add up; positive turn is to the left (counter-clockwise)."""
    actions = command.actions
    if "brake" in actions:
        return 0.0, 0.0
    linear, angular = PRESETS[preset]
    if "fast" in actions:
        linear, angular = min(linear * FAST, MAX_LINEAR), min(angular * FAST, MAX_ANGULAR)
    throttle = ("forward" in actions) - ("back" in actions) + stick(command.axes[0])
    turn = ("left" in actions) - ("right" in actions) + stick(command.axes[1])
    clip = lambda v: max(-1.0, min(1.0, v))  # noqa: E731
    return clip(throttle) * linear, clip(turn) * angular


def slew(current, goal, dt, accel, decel):
    """Move current toward goal by at most accel * dt, or decel * dt while
    slowing down (toward zero or through it)."""
    slowing = current != 0 and (goal * current < 0 or abs(goal) < abs(current))
    step = (decel if slowing else accel) * dt
    return current + max(-step, min(step, goal - current))


class Drive:
    """The twist the station sends: the target of the latest command, reached
    under the acceleration limits; zero at once on brake or when the page
    has been silent for DEADMAN seconds (closed tab, lost focus)."""

    def __init__(self, preset=2):
        self.preset = preset
        self.command = Command()
        self.last_input = None  # time of the latest command
        self.last_step = None
        self.twist = (0.0, 0.0)
        self.deadman = True  # holding the rover (as of the last step)

    def set_preset(self, preset):
        if preset in PRESETS:
            self.preset = preset

    def input(self, command, now):
        self.command = command
        self.last_input = now

    def step(self, now):
        dt = 0.0 if self.last_step is None else min(max(now - self.last_step, 0.0), 0.2)
        self.last_step = now
        self.deadman = self.last_input is None or now - self.last_input > DEADMAN
        if self.deadman or "brake" in self.command.actions:
            self.twist = (0.0, 0.0)
            return self.twist
        goal = target(self.command, self.preset)
        self.twist = tuple(slew(c, g, dt, a, d) for c, g, a, d in zip(self.twist, goal, ACCEL, DECEL))
        return self.twist


class Look:
    """Where the operator looks from the rover: the rover eye's yaw and pitch,
    which are also the pan-tilt head's targets (pan = yaw, left +; tilt =
    pitch, down +), so both see the same. Kept inside the head's joint
    limits; centred is straight ahead at the head's starting tilt."""

    def __init__(self, rover=ROVER):
        self.rover = rover
        self.center()

    def center(self):
        self.pan, self.tilt = 0.0, self.rover.camera_pitch

    def nudge(self, pan, tilt):
        limit = self.rover.camera_pan_limit
        lower, upper = self.rover.camera_tilt_limits
        self.pan = max(-limit, min(limit, self.pan + pan))
        self.tilt = max(lower, min(upper, self.tilt + tilt))
