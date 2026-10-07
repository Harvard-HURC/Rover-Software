"""Run the rover headless in Gazebo for the tests.

Gazebo's Python TestFixture runs the server inside this process, so a run is
repeatable: commands go out over gz-transport every 20 ms of sim time, and
the state is read from the entity-component manager after the last step.
The rover is model://rover or one built from gen_model Params (the physical
drivetrain: physical()), on flat ground (world_sdf) or on a synthetic terrain
with a ground map (ground_world: heightmap, ground.png and ground.json as the
drivetrain reads them, design spec 9.1).

cpu_time_per_step() measures what a world costs instead (the realism design's
section 10.3): plain `gz sim -s -r --iterations N` processes, their CPU time
from the kernel minus a one-step start-up run, while twist_publisher drives
the rover from outside the server (no Python in its step).
"""
import contextlib
import dataclasses
import json
import math
import os
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image
from worldfiles import SIM_DIR, temp_sdf

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
import gzenv  # noqa: E402
from urc import sdf, terrain, terrains  # noqa: E402

# Set before Gazebo starts (gzenv): where model://rover and the plugins live,
# and a private transport partition so tests never talk to a running simulation.
os.environ.update(gzenv.environment(partition=f"rover_sim_test_{os.getpid()}",
                                    ip=os.environ.get("GZ_IP", "127.0.0.1")))

import gz.math7  # noqa: E402,F401  (lets gz.sim8 return Pose3d values)
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
from gz.msgs10.world_stats_pb2 import WorldStatistics  # noqa: E402
from gz.sim8 import Joint, Link, Model, TestFixture, World, world_entity  # noqa: E402
from gz.transport13 import Node  # noqa: E402

LINKS = ("base_link", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
ROCKERS = ("rocker_left_joint", "rocker_right_joint")
ROVER_URI = "model://rover"
SOLVERS = ("dantzig", "pgs")  # DART's LCP solvers (dantzig is its default)
# A drive for cost measurements, [(t [s], vx [m/s], wz [rad/s])] from t on:
# straight, turn in place, an arc, the turn back (the drivetrain prototype's
# exp_rtf.py schedule, sim/data/research/drive/prototype).
DRIVE_SCHEDULE = ((0.0, 0.0, 0.0), (1.0, 0.8, 0.0), (8.0, 0.0, 1.0), (12.0, 0.8, 0.2), (18.0, 0.0, -1.0),
                  (22.0, 0.0, 0.0))


@dataclass
class State:
    """The rover after the last step."""

    rockers: dict  # joint name -> angle [rad]
    poses: dict  # link name -> (x, y, z, roll, pitch, yaw) in the world
    rocker_peak: float = 0.0  # the largest |rocker angle| during the run [rad]
    messages: dict = field(default_factory=dict)  # topic -> messages received
    trace: np.ndarray = None  # with trace_every: rows (t, x, y, z, roll, pitch, yaw, yaw rate) of base_link


def world_sdf(extra="", spawn_z=0.02, rover_uri=ROVER_URI, default_surface=None, solver=None):
    """Flat ground (DART, 1 ms steps), the rover at the origin, plus `extra` SDF.
    default_surface: a terrains.TYPES key or TerrainType for the ground (its
    Coulomb mu in the SDF, which DART honours on a plane; the physical
    drivetrain takes a plane's ground from its own default surface instead,
    see world_file); solver: one of SOLVERS (None: DART's default)."""
    surface = ""
    if default_surface is not None:
        kind = terrains.TYPES[default_surface] if isinstance(default_surface, str) else default_surface
        mu = kind.traction.mu_k
        surface = f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>"
    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="test">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>{_solver_sdf(solver)}
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>{surface}
        </collision>
      </link>
    </model>
    {extra}
    <include><uri>{rover_uri}</uri><pose>0 0 {spawn_z} 0 0 0</pose></include>
  </world>
</sdf>
"""


def _solver_sdf(solver):
    if solver is None:
        return ""
    if solver not in SOLVERS:
        raise ValueError(f"solver {solver!r}: one of {SOLVERS}")
    return f"\n      <dart><solver><solver_type>{solver}</solver_type></solver></dart>"


def variant_sdf(text, rover_uri=None, solver=None):
    """A world document with its rover (model://rover) replaced by
    rover_uri and/or DART's solver set (one of SOLVERS)."""
    if rover_uri is not None:
        text, count = re.subn(r"<uri>model://rover</uri>", f"<uri>{rover_uri}</uri>", text)
        assert count == 1, f"{count} rovers in the world"
    if solver is not None:
        assert "<dart>" not in text, "the world sets DART options already"
        text, count = re.subn(r"(<physics[^>]*>)", lambda m: m.group(1) + _solver_sdf(solver), text, count=1)
        assert count == 1, "the world has no <physics>"
    return text


@contextlib.contextmanager
def rover_model(params):
    """URI of a temporary rover model built from gen_model Params `params`."""
    with tempfile.TemporaryDirectory(prefix="rover_model_") as directory:
        model = Path(directory) / "rover"
        model.mkdir()
        (model / "model.sdf").write_text(gen_model.build_sdf(params))
        (model / "model.config").write_text(sdf.model_config("rover", "A test variant of the rover.",
                                                             "sim/tests/simulate.py"))
        yield model.as_uri()


def physical(**drive):
    """Params of the rover with the physical drivetrain, its DriveParams
    changed by `drive`; the command timeout runs on sim time, so runs repeat
    exactly."""
    return gen_model.Params(drive=gen_model.DriveParams(mode="physical", cmd_timeout_clock="sim", **drive))


@contextlib.contextmanager
def world_file(world=None, extra="", spawn_z=0.02, rover_uri=None, params=None, default_surface=None, solver=None):
    """Path of the world to run: world_sdf(extra, spawn_z, ...) when world is
    None, else the SDF file `world` (a copy when its rover or solver change).
    The rover: rover_uri, or one built from Params `params`, or model://rover.
    A physical rover built from params takes default_surface as its
    drivetrain's default surface, which is what it feels on a plane."""
    with contextlib.ExitStack() as stack:
        if params is not None:
            assert rover_uri is None, "rover_uri or params, not both"
            if default_surface is not None and params.drive.mode == "physical":
                drive = dataclasses.replace(params.drive, default_surface=default_surface)
                params, default_surface = dataclasses.replace(params, drive=drive), None
            rover_uri = stack.enter_context(rover_model(params))
        if world is None:
            text = world_sdf(extra, spawn_z, rover_uri or ROVER_URI, default_surface, solver)
            yield stack.enter_context(temp_sdf(text))
            return
        assert default_surface is None, "default_surface applies to world_sdf's ground only"
        if rover_uri is None and solver is None:
            yield str(world)
        else:
            yield stack.enter_context(temp_sdf(variant_sdf(Path(world).read_text(), rover_uri, solver)))


def twist_at(schedule, t):
    """(vx, wz) of a schedule [(t_start, vx, wz), ...] at time t: the last
    entry started by then, (0, 0) before the first."""
    vx = wz = 0.0
    for start, v, w in schedule:
        if t >= start:
            vx, wz = v, w
    return vx, wz


def simulate(seconds, extra="", spawn_z=0.02, cmd=(0.0, 0.0), subscribe=(), world=None, rover_uri=None, params=None,
             default_surface=None, solver=None, trace_every=0, publish_until=None):
    """Run for `seconds` of sim time and return the final State.

    cmd: (vx [m/s], wz [rad/s]) sent from the start, or a schedule
    [(t [s], vx, wz), ...] (twist_at), sent every 20 ms from the start and,
    with publish_until [s], only until then.
    subscribe: (topic, message class) pairs to record during the run.
    world: an SDF file to run instead of world_sdf(extra, spawn_z).
    rover_uri, params, default_surface, solver: see world_file.
    trace_every: record base_link every that many steps (State.trace).
    """
    with world_file(world, extra, spawn_z, rover_uri, params, default_surface, solver) as world_path:
        return _run(seconds, cmd, subscribe, world_path, trace_every, publish_until)


def _run(seconds, cmd, subscribe, world_path, trace_every=0, publish_until=None):
    node = Node()
    messages = {topic: [] for topic, _ in subscribe}
    for topic, msg_type in subscribe:
        node.subscribe(msg_type, topic, messages[topic].append)
    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
    scripted = bool(cmd) and isinstance(cmd[0], (tuple, list))
    twist = Twist()
    if not scripted:
        twist.linear.x, twist.angular.z = cmd
    handles = {}
    last = {}
    peak = [0.0]
    trace = []

    def pre_update(info, ecm):
        if not handles:
            model = Model(World(world_entity(ecm)).model_by_name(ecm, "rover"))
            handles["rockers"] = {n: Joint(model.joint_by_name(ecm, n)) for n in ROCKERS}
            handles["links"] = {n: Link(model.link_by_name(ecm, n)) for n in LINKS}
            for joint in handles["rockers"].values():
                joint.enable_position_check(ecm, True)
            handles["links"]["base_link"].enable_velocity_checks(ecm, True)
        if publish_until is not None and info.iterations / 1000 > publish_until:
            return
        if info.iterations % 20 == 0:
            if scripted:
                twist.linear.x, twist.angular.z = twist_at(cmd, info.iterations / 1000)
                publisher.publish(twist)
            elif any(cmd):
                publisher.publish(twist)

    def post_update(info, ecm):
        last["rockers"] = {n: j.position(ecm) for n, j in handles["rockers"].items()}
        last["poses"] = {n: link.world_pose(ecm) for n, link in handles["links"].items()}
        peak[0] = max([peak[0]] + [abs(q[0]) for q in last["rockers"].values() if q])
        if trace_every and info.iterations % trace_every == 0:
            rate = handles["links"]["base_link"].world_angular_velocity(ecm)
            trace.append((info.iterations / 1000, *_xyzrpy(last["poses"]["base_link"]), rate.z() if rate else 0.0))

    fixture = TestFixture(world_path)
    fixture.on_pre_update(pre_update)
    fixture.on_post_update(post_update)
    fixture.finalize()
    fixture.server().run(True, round(seconds * 1000), False)
    if not last:
        raise RuntimeError(f"Gazebo did not step {world_path}; see the [Err] lines above")
    rockers = {n: q[0] for n, q in last["rockers"].items()}
    return State(rockers, {n: _xyzrpy(pose) for n, pose in last["poses"].items()}, peak[0], messages,
                 np.array(trace) if trace_every else None)


def _xyzrpy(pose):
    p, r = pose.pos(), pose.rot().euler()
    return (p.x(), p.y(), p.z(), r.x(), r.y(), r.z())


# --- Worlds with a ground map ----------------------------------------------------------------

def ground_row(index, key, traction, dust=0.0):
    """One row of ground.json's traction table (design spec 9.1): a type
    index in ground.png, its key and a terrains.Traction."""
    return dict(index=index, key=key, **dataclasses.asdict(traction), dust=dust)


def ground_json(hf, rows, default=None, collisions=None, prefixes=None, terrain_default=None, object_default=None):
    """The ground.json document of terrain hf (design spec 9.1): rows from
    ground_row; default: the index of samples the table lacks (the first
    row's); collisions, prefixes: the terrain's other shapes; terrain_default,
    object_default: keys for the terrain's unmapped shapes and for objects."""
    return {"format": "rover-ground/2", "size_m": hf.size, "samples": hf.n,
            "default": rows[0]["index"] if default is None else default, "types": rows,
            "collisions": collisions or {}, "prefixes": prefixes or {}, "terrain_default": terrain_default,
            "object_default": object_default}


def surface_pose(hf, x, y, yaw, lift=0.02):
    """(x, y, z, roll, pitch, yaw) that puts a model's base `lift` above the
    terrain hf at (x, y), tilted to its slope there and heading `yaw`."""
    d = 0.5
    gx = (hf.height(x + d, y) - hf.height(x - d, y)) / (2 * d)
    gy = (hf.height(x, y + d) - hf.height(x, y - d)) / (2 * d)
    n = np.array([-gx, -gy, 1.0]) / math.sqrt(gx * gx + gy * gy + 1)
    tilt = np.eye(3)
    angle = math.acos(n[2])  # the rotation from up to the normal
    if angle > 1e-9:
        k = np.cross([0.0, 0.0, 1.0], n) / math.sin(angle)
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        tilt = tilt + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K
    heading = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    position = np.array([x, y, hf.height(x, y)]) + lift * n
    return (*position, *sdf.matrix_to_rpy((tilt @ heading).tolist()))


@contextlib.contextmanager
def ground_world(hf, ground, rows, rover=(0.0, 0.0, 0.0), *, lift=0.02, ground_options=None, terrain_extra="",
                 extra="", rover_uri=ROVER_URI, params=None, solver=None, gravity=(0.0, 0.0, -9.8)):
    """Path of a world on a synthetic terrain with a ground map, in a
    temporary directory. hf: a terrain.Heightfield centred on the origin, the
    collision heightmap (shifted so its lowest sample is z = 0, as Gazebo
    needs; a flat one gets a 1 mm north-west corner sample, because Gazebo
    scales by the highest pixel). ground: (n, n) type indices, ground.png;
    rows and ground_options (ground_json's keywords): ground.json. The rover:
    model rover_uri, or one built from Params `params`, at rover = (x, y,
    yaw), `lift` above the terrain and tilted to its slope. terrain_extra:
    SDF of more shapes in the terrain's link; extra: SDF of more models;
    gravity [m/s^2] (SDF's default; tilted, flat ground stands in for a slope
    the rover starts on without a landing jolt)."""
    z = np.asarray(hf.z, dtype=float) - float(np.min(hf.z))
    if z.max() <= 0:
        z[0, 0] = 0.001
    surface = terrain.Heightfield(hf.size, hf.n, z)
    ground = np.asarray(ground, dtype=np.uint8)
    assert ground.shape == (hf.n, hf.n), "ground.png lies on the heightmap's grid"
    with contextlib.ExitStack() as stack:
        directory = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix="ground_world_")))
        if params is not None:
            assert rover_uri == ROVER_URI, "rover_uri or params, not both"
            rover_uri = stack.enter_context(rover_model(params))
        _, z_max = surface.write_png(directory / "heightmap.png", 0.0, float(z.max()))
        Image.fromarray(ground, mode="L").save(directory / "ground.png")
        (directory / "ground.json").write_text(json.dumps(ground_json(surface, rows, **(ground_options or {}))))
        pose = " ".join(f"{v:.9g}" for v in surface_pose(surface, *rover, lift))
        world = directory / "world.sdf"
        world.write_text(f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="ground_test">
    <gravity>{" ".join(f"{g:.9g}" for g in gravity)}</gravity>
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>{_solver_sdf(solver)}
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="heightmap_collision">
          <geometry><heightmap><uri>{(directory / "heightmap.png").as_uri()}</uri>
            <size>{hf.size} {hf.size} {z_max}</size><pos>0 0 0</pos></heightmap></geometry>
        </collision>
        {terrain_extra}
      </link>
    </model>
    {extra}
    <include><uri>{rover_uri}</uri><name>rover</name><pose>{pose}</pose></include>
  </world>
</sdf>
""")
        yield str(world)


# --- Cost: CPU time per step --------------------------------------------------------------

def world_name(path):
    """The name of the world in an SDF file."""
    return re.search(r'<world\s+name="([^"]+)"', Path(path).read_text()).group(1)


@contextlib.contextmanager
def twist_publisher(worlds, schedule=DRIVE_SCHEDULE, rate=20.0):
    """Drive the rover from a thread of this process: twist_at(schedule, sim
    time) on its cmd_vel topic `rate` times a second, while any of the named
    worlds runs in this partition. Sim time comes from the worlds' statistics
    (10 Hz): /clock, sent every step, would add to the cost being measured.
    Nothing is sent before the first statistics message."""
    node = Node()
    now = []

    def on_stats(msg):
        now[:] = [msg.sim_time.sec + msg.sim_time.nsec * 1e-9]

    for name in worlds:
        node.subscribe(WorldStatistics, f"/world/{name}/stats", on_stats)
    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
    stop = threading.Event()

    def loop():
        twist = Twist()
        while not stop.wait(1.0 / rate):
            if now:
                twist.linear.x, twist.angular.z = twist_at(schedule, now[0])
                publisher.publish(twist)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    try:
        yield
    finally:
        stop.set()
        thread.join()


@dataclass
class Cost:
    """What a world cost over `runs` server processes (cpu_time_per_step)."""

    per_step: float  # [s] median CPU time per physics step, start-up subtracted
    steps: list  # [s] per run
    startup_cpu: float  # [s] median CPU time of a one-step run (load, one step, exit)
    startup_wall: float  # [s] median wall time of a one-step run
    peak_rss: int  # [bytes] largest resident size of any run
    load: list  # 1-minute load average at the start of each run
    wall_per_step: float  # [s] median wall time per physics step, start-up subtracted

    @property
    def real_time_factor(self):
        """Sim seconds per CPU second at 1 ms steps, the design's measure (it
        holds on a loaded machine). A lower bound: the server's helper threads
        (gz-transport, /clock sent every step) add CPU time beside the step;
        on this machine it is 13-29 % below wall_real_time_factor
        (sim/data/research/gates.json, G5)."""
        return 0.001 / self.per_step

    @property
    def wall_real_time_factor(self):
        """Sim seconds per wall second at 1 ms steps: the real-time factor the
        server reaches, valid on an otherwise idle machine only."""
        return 0.001 / self.wall_per_step


def gz_run(world, iterations, env=None):
    """One `gz sim -s -r --iterations N` process: (CPU s, wall s, peak RSS
    bytes), CPU = user + sys of the process and the children it waited for."""
    gz = shutil.which("gz") or str(Path(sys.prefix) / "bin" / "gz")
    with tempfile.TemporaryFile() as log:
        start = time.monotonic()
        process = subprocess.Popen([gz, "sim", "-s", "-r", "--iterations", str(iterations), str(world)],
                                   env=env or os.environ, stdout=log, stderr=subprocess.STDOUT)
        _, status, usage = os.wait4(process.pid, 0)
        wall = time.monotonic() - start
        process.returncode = os.waitstatus_to_exitcode(status)
        if process.returncode != 0:
            log.seek(0)
            raise RuntimeError(f"gz sim exited with {process.returncode} on {world}:\n"
                               + log.read().decode(errors="replace")[-3000:])
    return usage.ru_utime + usage.ru_stime, wall, usage.ru_maxrss  # macOS: ru_maxrss in bytes


def cpu_time_per_step(worlds, iterations=20_000, runs=5, schedule=DRIVE_SCHEDULE, env=None):
    """{label: Cost} for the SDF files in worlds {label: path}, by the realism
    design's method (section 10.3): each run is `gz sim -s -r --iterations N`
    minus a one-step run of the same world, CPU time from the kernel, the
    rover driving `schedule` (twist_publisher, outside the server). The
    worlds' runs interleave, so drifting load affects them alike; ratios hold
    on a loaded machine, absolute numbers need a load average below 4."""
    samples = {label: [] for label in worlds}
    with twist_publisher({world_name(path) for path in worlds.values()}, schedule):
        for _ in range(runs):
            for label, path in worlds.items():
                load = os.getloadavg()[0]
                startup = gz_run(path, 1, env)
                full = gz_run(path, iterations, env)
                samples[label].append((load, startup, full))
    costs = {}
    for label, rows in samples.items():
        steps = [(full[0] - startup[0]) / (iterations - 1) for _, startup, full in rows]
        walls = [(full[1] - startup[1]) / (iterations - 1) for _, startup, full in rows]
        costs[label] = Cost(statistics.median(steps), steps, statistics.median(r[1][0] for r in rows),
                            statistics.median(r[1][1] for r in rows), max(r[2][2] for r in rows),
                            [r[0] for r in rows], statistics.median(walls))
    return costs
