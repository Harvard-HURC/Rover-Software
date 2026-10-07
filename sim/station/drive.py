"""What the driver's keys and sticks mean: target speeds, acceleration
limits, the deadman and where the cameras look; in the Fly view, how the fly
camera moves and where its gotos put it. No transport or web code here, so it
can be unit-tested (sim/tests/test_station.py)."""
import math
from dataclasses import dataclass

import numpy as np

import gen_model
import viewers

ROVER = gen_model.Params()
FLY = viewers.FlyParams()
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


# --- Fly view (realism design 8.3; plugins/fly_camera.cpp flies the camera) ------------------

FLY_TURN_RATE = 1.0  # [rad/s] turning with the arrows, I/K/J/L or the right stick (A: the page's look rate)
# KeyboardEvent.code -> (axis, sign) of the fly camera's command: move forward, left and up in
# cruise speeds; turn yaw (left +) and pitch (down +).
FLY_KEYS = {"KeyW": ("forward", 1), "KeyS": ("forward", -1), "KeyA": ("left", 1), "KeyD": ("left", -1),
            "KeyE": ("up", 1), "KeyQ": ("up", -1),
            "ArrowLeft": ("yaw", 1), "ArrowRight": ("yaw", -1), "KeyJ": ("yaw", 1), "KeyL": ("yaw", -1),
            "ArrowUp": ("pitch", -1), "ArrowDown": ("pitch", 1), "KeyI": ("pitch", -1), "KeyK": ("pitch", 1)}
FLY_FAST = frozenset({"ShiftLeft", "ShiftRight"})  # moves at FlyParams.fast cruise speeds
# Gotos (realism design 8.3).
ROVER_VIEW = (6.0, 3.0)  # [m] behind and above the rover, looking at it
POINT_PITCH = 0.6  # [rad] down, looking at a clicked map point ...
POINT_DISTANCE = (15.0, 400.0)  # [m] ... from a quarter of the map's span on screen, within these
TOP_SPAN = 1.4  # straight down at span / TOP_SPAN above the ground: a 1.2 rad picture is then as wide as the map
PIXEL_STANDOFF = 15.0  # [m] a double-clicked spot: fly to this short of it
MARCH_STEP = 0.5  # [m] along a ray against the heightmap (the URC worlds sample it every 0.25-1 m)
MARCH_RANGE = 10_000.0  # [m] a ray that meets no ground within this is sky (A: 5x the largest terrain)


@dataclass(frozen=True)
class View:
    """A fly camera pose: where it is [m] and where it looks, yaw (left +)
    and pitch (down +) [rad], turned as the plugin turns it: Rz(yaw) Ry(pitch)."""
    x: float
    y: float
    z: float
    yaw: float
    pitch: float

    @classmethod
    def of(cls, state):
        """The pose in the fly camera's state message."""
        return cls(*(float(state[k]) for k in ("x", "y", "z", "yaw", "pitch")))

    def axes(self):
        """The camera's forward, left and up axes in the world, as rows."""
        cy, sy, cp, sp = math.cos(self.yaw), math.sin(self.yaw), math.cos(self.pitch), math.sin(self.pitch)
        return np.array(((cp * cy, cp * sy, -sp), (-sy, cy, 0.0), (sp * cy, sp * sy, cp)))

    def quaternion(self):
        """(w, x, y, z) of roll 0, pitch, yaw."""
        cp, sp = math.cos(self.pitch / 2), math.sin(self.pitch / 2)
        cy, sy = math.cos(self.yaw / 2), math.sin(self.yaw / 2)
        return cp * cy, -sp * sy, sp * cy, cp * sy


def fly_command(command, fast=FLY.fast):
    """((forward, left, up) [cruise speeds], (yaw rate, pitch rate) [rad/s])
    for a Fly view input: keys are full stick, the right stick (axes: x right
    +, y down +) turns, and Shift multiplies the move by `fast`."""
    totals = dict.fromkeys(("forward", "left", "up", "yaw", "pitch"), 0.0)
    for key in command.keys:
        if key in FLY_KEYS:
            axis, sign = FLY_KEYS[key]
            totals[axis] += sign
    totals["yaw"] -= stick(command.axes[0])
    totals["pitch"] += stick(command.axes[1])
    clip = lambda v: max(-1.0, min(1.0, v))  # noqa: E731
    boost = fast if command.keys & FLY_FAST else 1.0
    return (tuple(clip(totals[a]) * boost for a in ("forward", "left", "up")),
            (clip(totals["yaw"]) * FLY_TURN_RATE, clip(totals["pitch"]) * FLY_TURN_RATE))


class Fly:
    """The fly camera's controls: the command of the latest Fly view input
    while the page keeps sending, None once it has been silent for DEADMAN
    (the station then stops publishing, and the plugin's own deadman stops the
    camera); and the speed multiplier, kept here so that a camera spawned
    again (a restarted world) gets it back."""

    def __init__(self, params=FLY):
        self.params = params
        self.command = Command()
        self.last_input = None
        self.scale = 1.0

    def input(self, command, now):
        self.command = command
        self.last_input = now

    def step(self, now):
        """fly_command of the latest input, or None while the page is silent."""
        if self.last_input is None or now - self.last_input > DEADMAN:
            return None
        return fly_command(self.command, self.params.fast)

    def speed(self, factor):
        """Multiply the speed multiplier by factor (> 0), within FlyParams.speed_scales; returns it."""
        if not factor > 0:
            raise ValueError(f"speed factor {factor!r} is not positive")
        low, high = self.params.speed_scales
        self.scale = max(low, min(high, self.scale * factor))
        return self.scale


def rover_view(pose, ground, clearance=FLY.clearance, look_height=viewers.ChaseParams.look_height):
    """ROVER_VIEW behind and above the rover (pose: x, y, z, yaw), looking at
    a point look_height above its origin, at least clearance above
    ground(x, y)."""
    behind, above = ROVER_VIEW
    yaw = pose["yaw"]
    x, y = pose["x"] - behind * math.cos(yaw), pose["y"] - behind * math.sin(yaw)
    z = max(pose["z"] + above, float(ground(x, y)) + clearance)
    return View(x, y, z, yaw, math.atan2(z - pose["z"] - look_height, behind))


def point_view(x, y, yaw, span, ground, clearance=FLY.clearance):
    """Looking at the ground at (x, y) along `yaw`, POINT_PITCH down, from a
    quarter of `span` (the map's width on screen [m]) within POINT_DISTANCE;
    lifted where that is less than clearance above the ground, still looking
    at the point."""
    distance = max(POINT_DISTANCE[0], min(POINT_DISTANCE[1], span / 4))
    target = float(ground(x, y))
    back = distance * math.cos(POINT_PITCH)
    cx, cy = x - back * math.cos(yaw), y - back * math.sin(yaw)
    cz = max(target + distance * math.sin(POINT_PITCH), float(ground(cx, cy)) + clearance)
    return View(cx, cy, cz, yaw, math.atan2(cz - target, back))


def top_view(x, y, span, ground, clearance=FLY.clearance, yaw=math.pi / 2):
    """Straight down over (x, y), span / TOP_SPAN above the ground; yaw pi/2
    puts north at the top of the picture, as on the map."""
    return View(x, y, float(ground(x, y)) + max(span / TOP_SPAN, clearance), yaw, math.pi / 2)


def looks_down(view):
    """Whether the camera looks straight down (top-down or orthographic)."""
    return view.pitch > math.pi / 2 - 1e-3


def pan_view(view, x, y, ground, ortho=False, clearance=FLY.clearance):
    """`view` moved over (x, y), looking the same way: where a camera looking
    straight down goes. It stays as high above the ground as it is now; an
    orthographic one keeps its height, which with the height of the ground
    where orthographic began sets the window (plugins/fly_camera.cpp), so the
    scale holds. Never closer than clearance to the ground."""
    there = float(ground(x, y))
    z = view.z if ortho else there + view.z - float(ground(view.x, view.y))
    return View(float(x), float(y), max(z, there + clearance), view.yaw, view.pitch)


def pixel_ray(view, u, v, hfov, aspect, ortho=0.0):
    """(origin, unit direction) of the ray through the picture point (u, v)
    (0-1 from the left and from the top) of a camera at `view` with
    horizontal field of view hfov [rad] and width / height `aspect`; ortho:
    the orthographic window's width [m], 0 in perspective."""
    forward, left, up = view.axes()
    across, down = 2 * u - 1, 2 * v - 1
    origin = np.array((view.x, view.y, view.z), float)
    if ortho > 0:
        return origin - across * ortho / 2 * left - down * ortho / aspect / 2 * up, forward
    t = math.tan(hfov / 2)
    direction = forward - across * t * left - down * t / aspect * up
    return origin, direction / np.linalg.norm(direction)


def march(origin, direction, ground, step=MARCH_STEP, reach=MARCH_RANGE):
    """Distance along the ray to where it first meets ground(x, y)
    (vectorised), within 1 mm; None if it meets none within reach."""
    t = np.arange(0.0, reach + step, step)
    points = origin + t[:, None] * direction
    below = np.flatnonzero(points[:, 2] <= ground(points[:, 0], points[:, 1]))
    if not len(below):
        return None
    if below[0] == 0:
        return 0.0
    lo, hi = t[below[0] - 1], t[below[0]]
    while hi - lo > 1e-3:
        mid = (lo + hi) / 2
        p = origin + mid * direction
        lo, hi = (lo, mid) if p[2] <= float(ground(p[0], p[1])) else (mid, hi)
    return float(hi)


def pixel_view(view, u, v, hfov, aspect, ground, ortho=0.0, clearance=FLY.clearance):
    """Where a double-click on the picture point (u, v) flies the camera:
    PIXEL_STANDOFF short of where the ray meets the ground, looking along
    it. Looking straight down (top-down or orthographic) it moves over that
    point instead (pan_view). None for the sky."""
    origin, direction = pixel_ray(view, u, v, hfov, aspect, ortho)
    t = march(origin, direction, ground)
    if t is None:
        return None
    x, y, _ = origin + t * direction
    if ortho > 0 or looks_down(view):
        return pan_view(view, x, y, ground, ortho > 0, clearance)
    x, y, z = (float(c) for c in origin + max(t - PIXEL_STANDOFF, 0.0) * direction)
    return View(x, y, max(z, float(ground(x, y)) + clearance), math.atan2(direction[1], direction[0]),
                math.asin(-direction[2]))


def flat_ground(x, y):
    """The ground of a world without a heightmap: z = 0, as the fly camera takes it."""
    return np.zeros(np.shape(x))
