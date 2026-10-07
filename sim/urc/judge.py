"""Mission judges: the referee's rules, without Gazebo.

A judge reads the mission sheet, gets an Observation every referee tick and
returns Actions (move the astronaut, publish a command, ...). sim/referee.py
connects judges to a running simulation; the tests drive them with made-up
observations.

Scoring follows URC 2027 where the rules give points (Autonomy 1.e). Where
they do not (Equipment Servicing "points for every task", Delivery "partial
completion"), tasks share the mission's 100 points equally; the judges in
the field decide that at URC. Distances to the rover are measured from its
footprint (chassis plus wheels), "stopped" means under 5 cm/s for 2 s.
"""
import math
import random
import string
from dataclasses import dataclass, field

import gen_model  # the rover's geometry; sim/ is on sys.path wherever urc is used

from . import lander, props, rules, sheet as sheets

_ROVER = gen_model.Params()
ROVER_HALF_LENGTH = _ROVER.wheel_dx + _ROVER.wheel_radius  # [m] wheel centres plus the wheel radius
ROVER_HALF_WIDTH = _ROVER.pivot_y + _ROVER.wheel_width / 2
STOPPED_SPEED = 0.05  # [m/s]
STOPPED_TIME = 2.0  # [s]


# --- Inputs and outputs ----------------------------------------------------------------

@dataclass
class Observation:
    t: float  # sim time [s]
    rover: tuple  # (x, y, z, yaw), world frame
    led: str = "off"
    poses: dict = field(default_factory=dict)  # "model" or "model::link" -> (x, y, z, roll, pitch, yaw)
    joints: dict = field(default_factory=dict)  # "model::joint" -> position
    presses: list = field(default_factory=list)  # lander joint names pressed since the last observation


@dataclass
class SetPose:
    model: str
    pose: tuple  # x, y, z, yaw


@dataclass
class JointTargets:
    model: str
    targets: dict  # joint -> angle


@dataclass
class Say:
    topic: str  # "/astronaut/command", "/astronaut/speech", "/urc/display", ...
    text: str


# --- Shared helpers ------------------------------------------------------------------

def footprint_distance(rover, point):
    """Horizontal distance from a point to the rover's footprint rectangle."""
    x, y, _, yaw = rover
    dx, dy = point[0] - x, point[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    fx, fy = c * dx + s * dy, -s * dx + c * dy  # in the rover frame
    ex = max(abs(fx) - ROVER_HALF_LENGTH, 0.0)
    ey = max(abs(fy) - ROVER_HALF_WIDTH, 0.0)
    return math.hypot(ex, ey)


class Motion:
    """Tracks whether the rover has stopped."""

    def __init__(self):
        self.history = []

    def update(self, t, rover):
        self.history.append((t, rover[0], rover[1]))
        while self.history and self.history[0][0] < t - STOPPED_TIME:
            self.history.pop(0)

    @property
    def stopped(self):
        if len(self.history) < 2 or self.history[-1][0] - self.history[0][0] < STOPPED_TIME * 0.9:
            return False
        _, x0, y0 = self.history[-1]
        return max(math.hypot(x - x0, y - y0) for _, x, y in self.history) < STOPPED_SPEED * STOPPED_TIME


class Judge:
    """Base: holds the sheet, the score, the event log and rover motion."""

    def __init__(self, sheet, terrain=None):
        self.sheet = sheet
        self.terrain = terrain
        self.motion = Motion()
        self.scores = {}  # task id -> points
        self.max_points = {}
        self.events = []  # (t, text)
        self.t = 0.0

    def log(self, text):
        self.events.append((self.t, text))

    def award(self, task, points, text):
        if task not in self.scores:
            self.scores[task] = points
            self.log(f"{task}: {text} (+{points:g})")

    def point(self, key):
        p = self.sheet["points"][key]
        return p["x"], p["y"], p["z"]

    def step(self, obs):
        self.t = obs.t
        self.motion.update(obs.t, obs.rover)
        return self.update(obs)

    def update(self, obs):
        return []

    def summary(self):
        total = sum(self.scores.values())
        return {"mission": self.sheet["mission"], "t": round(self.t, 1), "points": round(total, 2),
                "max_points": round(sum(self.max_points.values()), 2), "tasks": dict(self.scores),
                "events": [f"[{t:7.1f}] {e}" for t, e in self.events]}


# --- Autonomy (1.e) -----------------------------------------------------------------

class RouteFinding(Judge):
    """Route targets: stop within 1 m and show the arrival LED (1.e.xvii)."""

    def __init__(self, sheet, terrain=None):
        super().__init__(sheet, terrain)
        self.targets = [t for t in sheet["tasks"] if t.get("subtask") == "route_finding"]
        for t in self.targets:
            self.max_points[t["id"]] = t["points"]

    def update(self, obs):
        for task in self.targets:
            target = self.point(task["target"])
            d = footprint_distance(obs.rover, target)
            if (task["id"] not in self.scores and self.motion.stopped and d <= task["tolerance_m"]
                    and obs.led == rules.LED_ARRIVED):
                self.award(task["id"], task["points"], f"reached {task['target']} ({d:.2f} m), LED green")
        return []


BECKON = [(0.0, {"shoulder_right_pitch": 1.5, "elbow_right": 0.3}), (0.5, {"elbow_right": 1.7}),
          (1.0, {"elbow_right": 0.3}), (1.5, {"elbow_right": 1.7}), (2.0, {"elbow_right": 0.3}),
          (2.5, {"elbow_right": 1.7}), (3.0, {"shoulder_right_pitch": 0.0, "elbow_right": 0.0})]
HALT = [(0.0, {"shoulder_right_pitch": 1.55, "elbow_right": 0.0}), (2.5, {"shoulder_right_pitch": 0.0})]
POINT_DOWN = [(0.0, {"shoulder_right_pitch": 0.7, "elbow_right": 0.0, "shoulder_right_roll": 0.3}),
              (3.0, {"shoulder_right_pitch": 0.0, "shoulder_right_roll": 0.0})]
BOTH_ARMS = [(0.0, {"shoulder_right_pitch": 1.4, "shoulder_left_pitch": 1.4}),
             (3.0, {"shoulder_right_pitch": 0.0, "shoulder_left_pitch": 0.0})]
SHOW_SIGN = [(0.0, props.POSE_SHOW_SIGN), (4.0, props.POSE_REST)]
GESTURES = {"follow": BECKON, "come": BECKON, "stay": HALT, "fetch": POINT_DOWN, "give": BOTH_ARMS}
COMMAND_METHODS = ("device", "sign", "speech", "gesture")
NEAR_ASTRONAUT = 10.0  # [m] the rover must be with the astronaut for Stay!
GIVE_RADIUS = 6.0  # [m] the hammer is given to the astronaut: dropped this close


class AstronautAssistance(Judge):
    """Go to the astronaut, then Follow!, Stay!, Fetch!, Come!, Give! (1.e.v-x).

    The astronaut (model "astronaut") is scripted here: the referee moves it
    along the sheet's path and gives each command by one method (Follow!
    scores by method, 1.e.vi): "device" publishes the word on
    /astronaut/command, "speech" on /astronaut/speech (stands in for audio),
    "sign" raises the sign (one sign for every command: the order is fixed,
    Q&A Autonomy 13), "gesture" uses a gesture per command (beckon for
    Follow!/Come!, palm out for Stay!, point down for Fetch!, both arms out
    for Give!).

    The astronaut waits at the GNSS point until the rover gets there, so the
    sub-missions can come in either order (1.e.i); each command after that
    times out after TIMEOUT. A rover that arrives under teleoperation (blue
    LED) scores 0 for that task, but the commands go on.
    """

    PHASES = ("goto_astronaut", "follow", "stay", "fetch", "come", "give")
    TIMEOUT = 240.0  # [s] per command before the referee moves on

    def __init__(self, sheet, terrain, method="device"):
        super().__init__(sheet, terrain)
        assert method in COMMAND_METHODS
        self.method = method
        self.tasks = {t["id"]: t for t in sheet["tasks"] if t.get("subtask") == "astronaut_assistance"}
        for task_id in self.PHASES:
            self.max_points[task_id] = self.tasks[task_id]["points"]
        a = sheet["astronaut"]
        self.path = [(p["x"], p["y"]) for p in a["follow_path"]]
        self.stay_to = (a["stay_to"]["x"], a["stay_to"]["y"])
        self.speed = a["walk_speed_mps"]
        self.astronaut = self.path[0]
        self.yaw = sheet["objects"]["astronaut"]["yaw"]
        self.phase = 0
        self.phase_start = 0.0
        self.walk = None  # (start t, polyline)
        self.gesture = None  # (start t, keyframes)
        self.pending = None  # a command to give once the astronaut stops walking
        self.anchor = None  # rover position when Stay! was given
        self.with_astronaut = True  # the rover was near the astronaut at Stay!
        self.lifted_at_command = False
        self.started = False

    # The astronaut ------------------------------------------------------------

    def _command(self, word, actions):
        self.log(f"astronaut: {word.capitalize()}! ({self.method})")
        self.phase_start = self.t
        if self.method == "device":
            actions.append(Say("/astronaut/command", word))
        elif self.method == "speech":
            actions.append(Say("/astronaut/speech", word))
        elif self.method == "sign":
            self.gesture = (self.t, SHOW_SIGN)
        else:
            self.gesture = (self.t, GESTURES[word])

    def _walk_to(self, polyline):
        self.walk = (self.t + 2.0, [self.astronaut] + list(polyline))

    def _walking(self):
        return self.walk is not None

    def _move_astronaut(self, actions):
        if self.walk is not None:
            start, line = self.walk
            s = max(0.0, self.t - start) * self.speed
            for p, q in zip(line[:-1], line[1:]):
                seg = math.hypot(q[0] - p[0], q[1] - p[1])
                if s <= seg:
                    f = s / seg if seg else 1.0
                    self.astronaut = (p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]))
                    self.yaw = math.atan2(q[1] - p[1], q[0] - p[0])
                    break
                s -= seg
            else:
                self.astronaut = line[-1]
                self.walk = None
        x, y = self.astronaut
        actions.append(SetPose("astronaut", (x, y, self.terrain.height(x, y), self.yaw)))
        if self.gesture is not None:
            start, frames = self.gesture
            current = [f for f in frames if f[0] <= self.t - start]
            if current:
                targets = {}
                for _, frame in current:
                    targets.update(frame)
                actions.append(JointTargets("astronaut", targets))
            if self.t - start > frames[-1][0]:
                self.gesture = None

    def _face_rover(self, rover):
        if self.walk is None:
            self.yaw = math.atan2(rover[1] - self.astronaut[1], rover[0] - self.astronaut[0])

    # The tasks ------------------------------------------------------------------

    def _next(self, actions, obs, lifted):
        self.phase += 1
        self.phase_start = self.t
        if self.phase >= len(self.PHASES):
            self.log("astronaut assistance finished")
            return
        name = self.PHASES[self.phase]
        apart = math.hypot(self.astronaut[0] - obs.rover[0], self.astronaut[1] - obs.rover[1])
        if name == "follow":
            self._command("follow", actions)
            self._walk_to(self.path[1:])
        elif name == "stay":
            self._command("stay", actions)
            self.anchor = obs.rover[:2]
            self.with_astronaut = apart <= NEAR_ASTRONAUT
            self._walk_to([self.stay_to])
        elif name == "fetch":
            self.lifted_at_command = lifted
            self._command("fetch", actions)
        elif name == "come" and apart < rules.COME_MIN_DISTANCE + 2:
            # Q&A Autonomy 7: the astronaut is at least 20 m away for Come!.
            ax, ay = self.astronaut
            dx, dy = (ax - obs.rover[0], ay - obs.rover[1]) if apart > 0.1 else (1.0, 0.0)
            norm = math.hypot(dx, dy)
            far = rules.COME_MIN_DISTANCE + 5
            self._walk_to([(obs.rover[0] + dx / norm * far, obs.rover[1] + dy / norm * far)])
            self.pending = "come"
        elif name == "come":
            self._command("come", actions)
        elif name == "give":
            self.lifted_at_command = lifted
            self._command("give", actions)

    def _fail(self, name, why, actions, obs, lifted):
        self.scores.setdefault(name, 0)
        self.log(f"{name}: {why} (0 points)")
        self._next(actions, obs, lifted)

    def update(self, obs):
        actions = []
        if not self.started:
            self.started = True
            self.log("astronaut waiting at the GNSS point (1.e.v)")
        name = self.PHASES[self.phase] if self.phase < len(self.PHASES) else None
        d = footprint_distance(obs.rover, self.astronaut)
        hammer = obs.poses.get("rock_pick_hammer")
        ground = self.terrain.height(*hammer[:2]) if hammer else 0.0
        lifted = hammer is not None and hammer[2] > ground + 0.15
        autonomous = obs.led in (rules.LED_AUTONOMOUS, rules.LED_ARRIVED)
        points = self.tasks[name]["points"] if name else 0
        if self.pending is not None:
            name = None  # the astronaut is still walking to where it gives the command
        if name == "goto_astronaut":
            target = self.point(self.tasks[name]["target"])
            dt = footprint_distance(obs.rover, target)
            if self.motion.stopped and dt <= rules.ASTRONAUT_TOLERANCE:
                if autonomous:
                    self.award(name, points, f"stopped {dt:.1f} m from the astronaut")
                    self._next(actions, obs, lifted)
                else:
                    self._fail(name, f"arrived with the LED {obs.led}, not autonomous", actions, obs, lifted)
        elif name == "follow":
            if not self._walking() and self.motion.stopped and d <= rules.ASTRONAUT_TOLERANCE:
                points = self.tasks[name]["command_points"][self.method]
                self.award(name, points, f"followed and stopped {d:.1f} m from the astronaut ({self.method})")
                self._next(actions, obs, lifted)
        elif name == "stay":
            moved = math.hypot(obs.rover[0] - self.anchor[0], obs.rover[1] - self.anchor[1])
            away = math.hypot(self.astronaut[0] - obs.rover[0], self.astronaut[1] - obs.rover[1])
            if moved > 0.5:
                self._fail(name, f"the rover moved {moved:.1f} m", actions, obs, lifted)
            elif not self._walking() and self.t - self.phase_start > 15.0:
                if not self.with_astronaut:
                    self._fail(name, "the rover was not with the astronaut", actions, obs, lifted)
                elif away <= rules.STAY_DISTANCE:
                    self._fail(name, f"the astronaut is only {away:.0f} m away", actions, obs, lifted)
                else:
                    self.award(name, points, f"stayed while the astronaut walked {away:.0f} m away")
                    self._next(actions, obs, lifted)
        elif name == "fetch":
            if self.lifted_at_command:
                self._fail(name, "the hammer was picked up before Fetch!", actions, obs, lifted)
            elif lifted:
                if obs.led == rules.LED_AUTONOMOUS:
                    self.award(name, points, "picked up the hammer autonomously")
                    self._next(actions, obs, lifted)
                else:
                    self._fail(name, "hammer picked up under teleoperation", actions, obs, lifted)
        elif name == "come":
            if self.motion.stopped and d <= rules.ASTRONAUT_TOLERANCE:
                self.award(name, points, f"came within {d:.1f} m")
                self._next(actions, obs, lifted)
        elif name == "give":
            near = hammer is not None and math.hypot(hammer[0] - self.astronaut[0],
                                                     hammer[1] - self.astronaut[1]) <= GIVE_RADIUS
            if not self.lifted_at_command:
                self._fail(name, "the rover was not holding the hammer", actions, obs, lifted)
            elif hammer is not None and not lifted:
                if near:
                    self.award(name, points, "hammer placed on the ground by the astronaut")
                    self._next(actions, obs, lifted)
                else:
                    self._fail(name, "hammer dropped away from the astronaut", actions, obs, lifted)
        if name not in (None, "goto_astronaut") and self.phase < len(self.PHASES) \
                and self.PHASES[self.phase] == name and self.t - self.phase_start > self.TIMEOUT:
            self._fail(name, "timed out", actions, obs, lifted)
        current = self.PHASES[self.phase] if self.phase < len(self.PHASES) else None
        if current in ("goto_astronaut", "fetch", "come", "give"):
            self._face_rover(obs.rover)
        self._move_astronaut(actions)
        if self.pending is not None and not self._walking():
            self._face_rover(obs.rover)
            self._command(self.pending, actions)
            self.pending = None
        return actions


class Autonomy(Judge):
    """Both sub-missions, in any order (1.e.i)."""

    def __init__(self, sheet, terrain, method="device"):
        super().__init__(sheet, terrain)
        self.parts = [AstronautAssistance(sheet, terrain, method), RouteFinding(sheet, terrain)]

    def step(self, obs):
        self.t = obs.t
        actions = []
        for part in self.parts:
            actions += part.step(obs)
        self.scores = {k: v for part in self.parts for k, v in part.scores.items()}
        self.max_points = {k: v for part in self.parts for k, v in part.max_points.items()}
        self.events = sorted(e for part in self.parts for e in part.events)
        return actions


# --- Equipment Servicing (1.d) -------------------------------------------------------

def launch_key(seed=None):
    rng = random.Random(seed)
    return "".join(rng.choice(string.ascii_lowercase) for _ in range(rng.randint(*rules.LAUNCH_KEY_LENGTH)))


def type_text(text, joint):
    """The e-paper display after one key press."""
    char = lander.KEYMAP.get(joint)
    if char in ("\b", "\x7f"):
        return text[:-1]
    if char is None or char == "\n":
        return text
    return text + char


class EquipmentServicing(Judge):
    TASKS = ("cache_sample", "deliver_cache", "drawer", "panel", "typing", "key", "hose", "valve", "controls")

    def __init__(self, sheet, terrain=None, key=None):
        super().__init__(sheet, terrain)
        self.key = key or launch_key()
        each = 100.0 / len(self.TASKS)
        self.max_points = {t: each for t in self.TASKS}
        self.each = each
        self.display = ""
        self.typed_all = ""
        self.drawer_was_open = False
        self.buttons = set()
        self.announced = False
        lp = sheet["lander"]
        self.lander_pose = (lp["pose"]["x"], lp["pose"]["y"], lp["pose"]["z"], lp["pose"]["yaw"])

    def to_lander(self, p):
        """World point -> lander frame."""
        x0, y0, z0, yaw = self.lander_pose
        dx, dy = p[0] - x0, p[1] - y0
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy, p[2] - z0

    def update(self, obs):
        actions = []
        if not self.announced:
            self.announced = True
            self.log(f"launch key: {self.key}")
            actions.append(Say("/urc/launch_key", self.key))
        j = obs.joints
        for joint in obs.presses:
            if joint.startswith("key_"):
                self.display = type_text(self.display, joint)
                self.typed_all += lander.KEYMAP.get(joint) or ""
                actions.append(Say("/urc/display", self.display))
            elif joint.startswith("button_"):
                self.buttons.add(joint)
        tube = obs.poses.get("sample_tube")
        cache = obs.poses.get("cache_container")
        if tube and cache and "cache_sample" not in self.scores:
            inside = math.hypot(tube[0] - cache[0], tube[1] - cache[1]) < 0.05 and cache[2] - 0.01 < tube[2] < cache[2] + 0.1
            if inside and j.get("cache_container::lid_hinge", 1) < 0.1 and j.get("cache_container::lock_joint", 0) > 1.3:
                self.award("cache_sample", self.each, "tube in the cache, lid closed, lock turned")
        if cache:
            lx, ly, lz = self.to_lander(cache)
            if 0.0 < lx < 2.5 and abs(ly) < 1.6:
                self.award("deliver_cache", self.each, "cache at the lander")
            drawer = j.get("lander::drawer", 0.0)
            self.drawer_was_open = self.drawer_was_open or drawer > 0.2
            wx, wy, wz = lander.DRAWER_WELL_CLOSED
            in_well = abs(lx - (wx + drawer)) < 0.03 and abs(ly - wy) < 0.03 and abs(lz - wz) < 0.05
            if self.drawer_was_open and in_well and drawer < 0.01:
                self.award("drawer", self.each, "cache in the drawer well, drawer closed")
        if j.get("lander::latch", 0) > lander.LATCH_OPEN and j.get("lander::door", 0) > 1.0:
            self.award("panel", self.each, "latch undone, panel open")
        if self.display == self.key:
            self.award("typing", self.each, f"typed the launch key '{self.key}'")
        key = obs.poses.get("key")
        if key:
            kx, ky, kz = self.to_lander(key)
            bx, by, bz = lander.lock_point("b", lander.KEY_TIP_DEPTH)
            if abs(ky - by) < 0.01 and abs(kz - bz) < 0.01 and kx < bx + 0.01 and j.get("lander::lock_b", 0) > 1.3:
                self.award("key", self.each, "key in lock B and turned")
        coupler = obs.poses.get("fuel_tank::coupler")
        if coupler:
            cx, cy, cz = self.to_lander(coupler)
            tx, ty, tz = lander.INLET_TIP
            # The coupler's origin is its hose end; its mouth is COUPLER length
            # further along. Pushed on, the inlet is at least 2 cm inside.
            if abs(cy - ty) < 0.012 and abs(cz - tz) < 0.012 and cx < tx + props.COUPLER["length"] - 0.02:
                self.award("hose", self.each, "hose coupler pushed onto the inlet")
        if j.get("lander::valve", 0) > 1.3:
            self.award("valve", self.each, "valve turned")
        switches = sum(1 for n in range(len(lander.SWITCHES)) if j.get(f"lander::switch_{n}", 0) > lander.SWITCH_FLIPPED)
        knobs = sum(1 for n in range(len(lander.KNOBS)) if abs(j.get(f"lander::knob_{n}", 0)) > lander.KNOB_TURNED)
        if self.buttons and switches and knobs:
            self.award("controls", self.each, f"{len(self.buttons)} button(s), {switches} switch(es), {knobs} knob(s)")
        return actions


# --- Delivery (1.c) ---------------------------------------------------------------------

class Delivery(Judge):
    """Each task: delivered when the object lies on the ground, at rest for
    REST_TIME, within the tolerance of the astronaut. Partial credit (a
    quarter) for opening the toolbox and for finding a stage-2 object; it
    counts towards, not on top of, the task's share."""

    REST_TIME = 1.0  # [s]
    ON_GROUND = 0.15  # [m] object origin (its bottom) above the terrain

    def __init__(self, sheet, terrain=None):
        super().__init__(sheet, terrain)
        self.tasks = sheet["tasks"]
        each = 100.0 / len(self.tasks)
        self.each = each
        self.max_points = {t["id"]: each for t in self.tasks}
        self.track = {}  # object -> [(t, pose)] over the last REST_TIME

    def _resting_on_ground(self, name, t, pose):
        track = self.track.setdefault(name, [])
        if not track or track[-1][1] != pose:  # repeated (stale) poses carry no news
            track.append((t, pose))
        while len(track) > 1 and track[1][0] <= t - self.REST_TIME:
            track.pop(0)
        if t - track[0][0] < self.REST_TIME or any(math.dist(p[:3], pose[:3]) > 0.02 for _, p in track):
            return False
        ground = self.terrain.height(*pose[:2]) if self.terrain else pose[2]
        return pose[2] - ground < self.ON_GROUND

    def update(self, obs):
        for task in self.tasks:
            item = obs.poses.get(task["object"])
            if item is None or self.scores.get(task["id"], 0) >= self.each:
                continue
            tx, ty, _ = self.point(task["deliver_to"])
            resting = self._resting_on_ground(task["object"], obs.t, item)
            if resting and math.hypot(item[0] - tx, item[1] - ty) <= task["tolerance_m"]:
                self.scores[task["id"]] = self.each
                self.log(f"{task['id']}: {task['object']} delivered (+{self.each:.2f} in all)")
                continue
            if task["object"] == "wrench" and obs.joints.get("toolbox::lid_hinge", 0) > 1.0:
                self._partial(task["id"], "toolbox opened")
            if footprint_distance(obs.rover, item[:2]) < 3.0 and task["stage"] == 2:
                self._partial(task["id"], f"found the {task['object']}")
        return []

    def _partial(self, task, text):
        if task not in self.scores:
            self.scores[task] = round(self.each / 4, 2)
            self.log(f"{task}: {text} (+{self.scores[task]:g} partial)")


# --- Astrobiology (1.b) -------------------------------------------------------------------

class Astrobiology(Judge):
    """Site visits: the rover stopped >= 30 s, >= 20 m from other sites and
    from the C2 station and the rover's start, within 0.5 km of C2; then the
    cache back within HOME_RADIUS of C2. Science is judged by people; this
    judge records what a team would document (1.b.iii)."""

    SITE_TIME = 30.0
    SITE_SPACING = 20.0
    HOME_RADIUS = 15.0  # [m] from the C2 station (the rover starts 10 m away)

    def __init__(self, sheet, terrain=None):
        super().__init__(sheet, terrain)
        self.sites = []
        self.dwell = None  # (t start, x, y)
        self.max_points = {"sites": 50.0, "returned": 50.0}
        self.units = [(k, p) for k, p in sheet["points"].items() if "radius_m" in p]

    def update(self, obs):
        c2 = self.sheet["c2"]
        x, y = obs.rover[:2]
        if self.motion.stopped:
            if self.dwell is None:
                self.dwell = (obs.t, x, y)
            elif obs.t - self.dwell[0] >= self.SITE_TIME and all(
                    math.hypot(x - sx, y - sy) >= self.SITE_SPACING for sx, sy, *_ in self.sites + [
                        (c2["x"], c2["y"]), (self.sheet["rover_start"]["x"], self.sheet["rover_start"]["y"])]):
                r = math.hypot(x - c2["x"], y - c2["y"])
                unit = next((k for k, p in self.units if math.hypot(x - p["x"], y - p["y"]) <= p["radius_m"]), None)
                if r <= rules.SITE_RADIUS:
                    self.sites.append((x, y, obs.rover[2], unit))
                    self.log(f"site {len(self.sites)} at ({x:.1f}, {y:.1f}), {r:.0f} m from C2, unit: {unit}")
                    if len(self.sites) >= rules.MIN_SITES:
                        self.award("sites", 50.0, f"{len(self.sites)} sites investigated")
                else:
                    self.log(f"stopped {r:.0f} m from C2: outside the 0.5 km site")
                self.dwell = (math.inf, x, y)
        else:
            self.dwell = None
        if "sites" in self.scores and math.hypot(x - c2["x"], y - c2["y"]) <= self.HOME_RADIUS:
            self.award("returned", 50.0, "back at C2 with the cache")
        return []


def make_judge(mission, sheet, sheet_path, **options):
    terrain = sheets.terrain(sheet, sheet_path)
    if mission == "autonomy":
        return Autonomy(sheet, terrain, options.get("method") or "device")
    if mission == "equipment_servicing":
        return EquipmentServicing(sheet, terrain, key=options.get("key"))
    if mission == "delivery":
        return Delivery(sheet, terrain)
    if mission == "astrobiology":
        return Astrobiology(sheet, terrain)
    raise ValueError(f"unknown mission {mission}")
