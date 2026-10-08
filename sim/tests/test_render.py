"""Rendered pictures of the appearance assets (design spec section 11,
"Rendering"): the patched media's sky, Terra roughness and haze, the far
field, merged GLB clutter, what the haze leaves alone (depth), the
record of the rover's opt-in dust (gen_model.DriveParams.dust, off by
default since the user's decision of 2026-10-07): drawn, and seen by depth,
and the dig-in made visible (the user's decisions of 2026-10-07): the tyre
sink and the tread tyre seen by a camera on the driven rover (DigCues), the
ruts and pits behind the wheels as the cameras see them while the rover
drives in a generated world (Ruts).

Gazebo's ogre2 starts once per process, so every picture is taken in a
subprocess of its own (this file run with --render, in phases for
Renderer.drive, or with --drive for Renderer.drive_variant) from a small
world built here or a copy of a generated one, with the patched media (sim/tools/gz_media.py, made into a temporary build
directory) or Gazebo's stock media.
"""
import contextlib
import dataclasses
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from worldfiles import SIM_DIR

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
import gzenv  # noqa: E402
from urc import appearance as A  # noqa: E402
from urc import dem, farfield, geo, lighting, meshes, props, sdf, terrain, terrains  # noqa: E402
from urc.media import Media  # noqa: E402

sys.path.insert(0, str(SIM_DIR / "tools"))
import gz_media  # noqa: E402

STEPS = 1500  # [1 ms] per picture: ten frames at 10 Hz after the scene has loaded
COMMAND_STEPS, COMMAND_PERIOD = 20, 0.02  # a driven rover's command every 20 steps, 20 ms (as simulate sends them)
SUN = lighting.MISSION
PHYSICS_SYSTEM = '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'


def world(body, sky=True, particles=False):
    """A world document: DART at 1 ms as fast as it runs, the mission sun and
    sky, the Sensors system, and `body`."""
    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="render">
    <physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
    {PHYSICS_SYSTEM}{stage(sky, particles)}
    {body}
  </world>
</sdf>
"""


def stage(sky=True, particles=False):
    """What a world needs to be pictured: the SceneBroadcaster and Sensors
    systems, the mission sun and sky (and the particle emitters' system)."""
    f = lambda v: " ".join(f"{x:.6g}" for x in v)  # noqa: E731
    emitters = '<plugin filename="gz-sim-particle-emitter-system" name="gz::sim::systems::ParticleEmitter"/>'
    return f"""
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine></plugin>
    {emitters if particles else ''}
    <scene><ambient>{f(lighting.AMBIENT)} 1</ambient><background>{f(lighting.BACKGROUND)} 1</background>
      <grid>false</grid><shadows>true</shadows>{'<sky/>' if sky else ''}</scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 100 0 0 0</pose>
      <diffuse>{f(SUN.colour)} 1</diffuse><specular>{f(lighting.SUN_SPECULAR)} 1</specular>
      <intensity>{SUN.intensity}</intensity><direction>{f(SUN.direction)}</direction></light>"""


def camera(name, xyz, yaw, pitch, size=(640, 360), hfov=1.0, kind="camera", noise=None, far=80000.0, depth_far=40.0):
    """A static camera model looking along yaw (from +x, counter-clockwise),
    pitched down by `pitch` [rad], publishing on /render/<name> (an RGB-D
    camera: /render/<name>/image and /render/<name>/depth_image)."""
    noise_sdf = f"<noise><type>gaussian</type><mean>0</mean><stddev>{noise}</stddev></noise>" if noise else ""
    depth = f"<depth_camera><clip><near>0.1</near><far>{depth_far}</far></clip></depth_camera>" \
        if kind == "rgbd_camera" else ""
    fmt = "" if kind == "depth_camera" else "<format>R8G8B8</format>"
    return f"""
    <model name="cam_{name}"><static>true</static><pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 {pitch} {yaw}</pose>
      <link name="link"><sensor name="cam" type="{kind}"><always_on>true</always_on><update_rate>10</update_rate>
        <topic>/render/{name}</topic>
        <camera><horizontal_fov>{hfov}</horizontal_fov><image><width>{size[0]}</width><height>{size[1]}</height>{fmt}</image>
          <clip><near>0.1</near><far>{far}</far></clip>{depth}{noise_sdf}</camera>
      </sensor></link></model>"""


def model(models_dir, name, build):
    """Write models_dir/name with a static link that build(link) fills; returns its <include>."""
    root, m = sdf.model_root(name, static=True)
    build(sdf.link(m, "link"))
    sdf.write_model(models_dir, name, root, "Render test model.", generator="sim/tests/test_render.py")
    return f"<include><uri>model://{name}</uri><pose>0 0 0 0 0 0</pose></include>"


@dataclass
class Frames:
    """What a camera world showed after a phase of Renderer.drive: the last
    image of each topic {topic: image}, their stamps {topic: sim time [s]}
    and the drivetrain's last state (design spec 9.3; None without one)."""
    images: dict
    stamps: dict
    drivetrain: dict = None


class Renderer:
    """Takes pictures in subprocesses, with the patched media or stock."""

    def __init__(self, tmp):
        self.tmp = Path(tmp)
        self.models = self.tmp / "models"
        self.media = Media(self.models)
        self.patched = self.tmp / "patched"
        gz_media.build(gz_media.environment_media(), self.patched)  # raises: the patch must work here
        self.stock = self.tmp / "stock"
        self.stock.mkdir()
        self.count = 0

    def take(self, text, topics, patched=True):
        """{topic: last image} of world `text` (uint8 RGB or float32 depth)."""
        return self._run(text, topics, patched, {"steps": STEPS})[""].images

    def drive(self, text, topics, schedule, phases, patched=True):
        """The rover in world `text` driven by `schedule` [(t [s], vx, wz)]
        (simulate.twist_at) until the first reset, pictured after each of
        `phases` [(name, steps, reset)]: {name: Frames}. A phase runs its
        steps, then takes the pictures, then with `reset` resets the world."""
        from simulate import twist_at  # here: see _render_main
        steps = 0
        for _, n, reset in phases:
            steps += n
            if reset:
                break
        commands = [twist_at(schedule, k * COMMAND_PERIOD) for k in range(steps // COMMAND_STEPS + 1)]
        phases = [{"name": name, "steps": n, "reset": reset} for name, n, reset in phases]
        return self._run(text, topics, patched, {"commands": commands, "phases": phases})

    def _run(self, text, topics, patched, cfg):
        self.count += 1
        out = self.tmp / f"shot{self.count}"
        out.mkdir()
        path = out / "world.sdf"
        path.write_text(text)
        base = {k: v for k, v in os.environ.items() if k != "GZ_RENDERING_RESOURCE_PATH"}
        env = gzenv.environment(self.patched if patched else self.stock, partition=f"render_{os.getpid()}_{self.count}",
                                ip="127.0.0.1", base=base)
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([str(self.models), env["GZ_SIM_RESOURCE_PATH"]])
        cfg_file = out / "cfg.json"
        cfg_file.write_text(json.dumps({"world": str(path), "topics": topics, "out": str(out), **cfg}))
        result = subprocess.run([sys.executable, __file__, "--render", str(cfg_file)], env=env, capture_output=True,
                                text=True, timeout=300)
        frames = {}
        for phase in cfg.get("phases") or [{"name": ""}]:
            prefix = f"{phase['name']}_" if phase["name"] else ""
            images = {}
            for k, topic in enumerate(topics):
                file = out / f"{prefix}{k}.npy"
                if not file.exists():
                    raise AssertionError(f"no picture on {topic} ({phase['name'] or 'the end'}):\n"
                                         f"{result.stdout[-3000:]}\n{result.stderr[-3000:]}")
                images[topic] = np.load(file)
            info = json.loads((out / f"{prefix}info.json").read_text())
            frames[phase["name"]] = Frames(images, info["stamps"], info["drivetrain"])
        return frames

    def drive_variant(self, world_text, rover_sdf, cfg):
        """Drive a variant of the rover, the model `rover_sdf`, in world `world_text` (drive_world) in a subprocess
        (_drive_main); cfg: name (the world's), topics, schedule [(t, vx, wz)], seconds, grabs [(phase, t)],
        reset_steps (0: no reset).
        Returns (frames {(topic, grab index): image}, base_link trace rows (phase, t, x, y, z, yaw), the
        drivetrain's states [(phase, state)] (design spec 9.3), the subprocess's stderr, the frames' sim times
        {(topic, grab index): s})."""
        self.count += 1
        out = self.tmp / f"drive{self.count}"
        models = out / "models"
        (models / "rover_drive").mkdir(parents=True)
        (models / "rover_drive" / "model.sdf").write_text(rover_sdf)
        (models / "rover_drive" / "model.config").write_text(sdf.model_config(
            "rover_drive", "A test variant of the rover.", "sim/tests/test_render.py"))
        path = out / "world.sdf"
        path.write_text(world_text)
        base = {k: v for k, v in os.environ.items() if k != "GZ_RENDERING_RESOURCE_PATH"}
        env = gzenv.environment(self.patched, partition=f"render_{os.getpid()}_{self.count}", ip="127.0.0.1",
                                base=base)
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([str(models), str(self.models), env["GZ_SIM_RESOURCE_PATH"]])
        config = out / "cfg.json"
        config.write_text(json.dumps({**cfg, "world": str(path), "out": str(out)}))
        result = subprocess.run([sys.executable, __file__, "--drive", str(config)], env=env, capture_output=True,
                                text=True, timeout=900)
        frames = {}
        for k in range(len(cfg["grabs"])):
            for topic in cfg["topics"]:
                file = out / f"{topic.strip('/').replace('/', '_')}_{k}.npy"
                if not file.exists():
                    raise AssertionError(f"no frame {k} on {topic}:\n{result.stdout[-3000:]}\n{result.stderr[-3000:]}")
                frames[(topic, k)] = np.load(file)
        states = [(phase, json.loads(text)) for phase, text in json.loads((out / "states.json").read_text())]
        stamps = {(key.rsplit(" ", 1)[0], int(key.rsplit(" ", 1)[1])): t
                  for key, t in json.loads((out / "stamps.json").read_text()).items()}
        return frames, np.load(out / "trace.npy"), states, result.stderr, stamps


def aim(eye, target):
    """(yaw, pitch down) of a camera at `eye` looking at `target`."""
    d = np.subtract(target, eye, dtype=float)
    return math.atan2(d[1], d[0]), math.atan2(-d[2], math.hypot(d[0], d[1]))


def drive_world(world_name, rover, params, cameras, extra=""):
    """A copy of sim/worlds/<world_name>.sdf to drive in: as fast as it runs, the rover built from gen_model
    Params `params` (its model written beside, see Renderer.drive_variant) at rover = (x, y, yaw) 0.25 m above the
    terrain, the station's chase camera and rover eye (the models it spawns), and static cameras
    [(name, eye, target, kind, size)] (kind "camera" or "rgbd_camera"), eye and target `z` above the terrain."""
    from simulate import variant_sdf  # here: simulate sets the tests' Gazebo environment when imported
    from worldfiles import ROVER_POSE, WORLDS, terrain as world_terrain
    hf = world_terrain(world_name)
    text = (WORLDS / f"{world_name}.sdf").read_text()
    text = text.replace("<real_time_factor>1</real_time_factor>", "<real_time_factor>0</real_time_factor>")
    x, y, yaw = rover
    text, n = ROVER_POSE.subn(rf"\g<1>{x} {y} {hf.height(x, y) + 0.25} 0 0 {yaw}\g<2>", text)
    assert n == 1, world_name
    text = variant_sdf(text, rover_uri="model://rover_drive")
    body = ("<include><uri>model://chase_camera</uri><name>chase_camera</name>"
            f"<pose>{x + 3} {y} 3 0 0 0</pose></include>"
            f"<include><uri>model://eye_camera</uri><name>eye_camera</name><pose>{x} {y} 3 0 0 0</pose></include>")
    for name, eye, target, kind, size in cameras:
        eye = (eye[0], eye[1], eye[2] + hf.height(eye[0], eye[1]))
        target = (target[0], target[1], target[2] + hf.height(target[0], target[1]))
        body += camera(name, eye, *aim(eye, target), size=size, hfov=1.0, kind=kind)
    return text.replace("</world>", body + extra + "\n  </world>"), gen_model.build_sdf(params)


def _drive_main(cfg_path):
    """The subprocess of a drive (Renderer.drive_variant): run the world with the rover driving cfg's schedule (sim time,
    a command every 20 ms) for cfg's seconds, keep for each topic the first frame stamped at or after each grab
    time; with reset_steps, then request a world reset (the server is stopped, so it comes at that step) and run
    that many steps more; save the frames, the drivetrain's states and the base_link trace."""
    cfg = json.loads(Path(cfg_path).read_text())
    import gz.math7  # noqa: F401  (lets gz.sim8 return Pose3d values)
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.twist_pb2 import Twist
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.sim8 import Link, Model, TestFixture, World, world_entity
    from gz.transport13 import Node
    out = Path(cfg["out"])
    node = Node()
    phase = [0]  # 1 after the reset
    frames = {}  # (topic, grab index) -> (stamp, message): the earliest stamped at or after the grab time
    grabs = cfg["grabs"]  # [phase, t]

    def on_image(msg, topic):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec
        for k, (p, t) in enumerate(grabs):
            key = (topic, k)
            if p == phase[0] and t - 1e-6 <= stamp < t + 0.1 - 1e-6 and (key not in frames or stamp < frames[key][0]):
                frames[key] = (stamp, msg)

    for topic in cfg["topics"]:
        node.subscribe(Image, topic, lambda msg, t=topic: on_image(msg, t))
    states = []
    node.subscribe(StringMsg, gen_model.DRIVETRAIN_TOPIC, lambda m: states.append((phase[0], m.data)))
    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
    twist, handles, trace = Twist(), {}, []

    def pre_update(info, ecm):
        if not handles:
            handles["base"] = Link(Model(World(world_entity(ecm)).model_by_name(ecm, "rover")).link_by_name(
                ecm, "base_link"))
        if info.iterations % 20 == 0:
            vx = wz = 0.0
            for start, v, w in cfg["schedule"]:
                if info.iterations / 1000 >= start:
                    vx, wz = v, w
            twist.linear.x, twist.angular.z = vx, wz
            publisher.publish(twist)

    def post_update(info, ecm):
        if info.iterations % 10 == 0:
            p = handles["base"].world_pose(ecm)
            trace.append((phase[0], info.iterations / 1000, p.pos().x(), p.pos().y(), p.pos().z(),
                          p.rot().euler().z()))

    fixture = TestFixture(cfg["world"])
    fixture.on_pre_update(pre_update)
    fixture.on_post_update(post_update)
    fixture.finalize()
    fixture.server().run(True, round(cfg["seconds"] * 1000), False)
    time.sleep(0.5)
    if cfg.get("reset_steps"):
        request = WorldControl()
        request.reset.all = True
        ok, _ = node.request(f"/world/{cfg['name']}/control", request, WorldControl, Boolean, 3000)
        if not ok:
            print("RESET FAILED", flush=True)
        phase[0] = 1
        fixture.server().run(True, cfg["reset_steps"], False)
        time.sleep(0.5)
    stamps = {}
    for (topic, k), (stamp, msg) in list(frames.items()):
        stamps[f"{topic} {k}"] = round(stamp, 6)
        if len(msg.data) == msg.width * msg.height * 4:
            image = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        else:
            image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        np.save(out / f"{topic.strip('/').replace('/', '_')}_{k}.npy", image)
    np.save(out / "trace.npy", np.array(trace))
    (out / "states.json").write_text(json.dumps(states))
    (out / "stamps.json").write_text(json.dumps(stamps))


def _render_main(cfg_path):
    """The subprocess: run the world headless and, after each phase (default:
    one of cfg["steps"]), save each topic's last image (<phase>_<k>.npy) and
    <phase>_info.json, the images' stamps and the drivetrain's last state. A
    phase with "reset" then resets the world (WorldControl, through the gz
    command: a Python request with subscriptions active can lose its reply).
    cfg["commands"]: the rover's (vx, wz), one every COMMAND_PERIOD of sim
    time from the start until the first reset. This process must not import
    simulate: importing it sets the tests' Gazebo environment, the patched
    media among it, over the one Renderer gave the picture."""
    cfg = json.loads(Path(cfg_path).read_text())
    import gz.math7  # noqa: F401  (lets gz.sim8 return Pose3d values)
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.twist_pb2 import Twist
    from gz.sim8 import TestFixture
    from gz.transport13 import Node
    out = Path(cfg["out"])
    last, state = {}, {}
    node = Node()
    for topic in cfg["topics"]:
        node.subscribe(Image, topic, lambda msg, t=topic: last.__setitem__(t, msg))
    node.subscribe(StringMsg, gen_model.DRIVETRAIN_TOPIC, lambda msg: state.__setitem__("drivetrain", msg.data))
    fixture = TestFixture(cfg["world"])
    commands = cfg.get("commands")
    driving = [bool(commands)]
    if commands:
        publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
        twist = Twist()

        def pre_update(info, ecm):
            k, due = divmod(info.iterations, COMMAND_STEPS)
            if driving[0] and not due and k < len(commands):
                twist.linear.x, twist.angular.z = commands[k]
                publisher.publish(twist)

        fixture.on_pre_update(pre_update)
    fixture.finalize()
    for phase in cfg.get("phases") or [{"name": "", "steps": cfg["steps"]}]:
        fixture.server().run(True, phase["steps"], False)
        time.sleep(0.5)
        prefix = f"{phase['name']}_" if phase["name"] else ""
        stamps = {}
        for k, topic in enumerate(cfg["topics"]):
            if topic not in last:
                print(f"NO IMAGE {topic}", flush=True)
                continue
            msg = last[topic]
            if len(msg.data) == msg.width * msg.height * 4:
                image = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
            else:
                image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            np.save(out / f"{prefix}{k}.npy", image)
            stamps[topic] = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        drivetrain = json.loads(state["drivetrain"]) if "drivetrain" in state else None
        (out / f"{prefix}info.json").write_text(json.dumps({"stamps": stamps, "drivetrain": drivetrain}))
        if phase.get("reset"):
            driving[0] = False
            gz = shutil.which("gz") or str(Path(sys.prefix) / "bin" / "gz")
            name = re.search(r'<world\s+name="([^"]+)"', Path(cfg["world"]).read_text()).group(1)
            subprocess.run([gz, "service", "-s", f"/world/{name}/control", "--reqtype",
                            "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req",
                            "reset: {all: true}"], check=True, capture_output=True, timeout=30)
            last.clear()
            state.clear()


def residual(image, sigma=6):
    """Standard deviation of what a Gaussian blur removes: texture, clouds [DN]."""
    img = image.astype(np.float32)
    return float((img - cv2.GaussianBlur(img, (0, 0), sigma)).std())


class Render(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.r = Renderer(cls._tmp.name)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_procedural_sky_replaces_the_cumulus(self):
        """Stock cumulus sky gone: the patched sky is a smooth blue gradient."""
        text = world(camera("sky", (0, 0, 2), math.pi / 2, -0.45, hfov=1.2))
        stock = self.r.take(text, ["/render/sky"], patched=False)["/render/sky"]
        patched = self.r.take(text, ["/render/sky"])["/render/sky"]
        self.assertLess(residual(patched), 0.8)
        self.assertGreater(residual(stock), 3 * max(residual(patched), 0.3))
        top = patched[:60].reshape(-1, 3).mean(axis=0)
        self.assertGreater(top[2], top[0] + 40)  # blue overhead
        self.assertGreater(patched[-30:].mean(), patched[:30].mean())  # paler towards the horizon

    def test_no_glint_on_sunlit_terrain(self):
        """No specular glint on sunlit flat terrain: looking at the sun's
        mirror image on Terra, the brightest 0.1 % of ground pixels stay
        within p99 + 15 DN (stock: roughness 0, a mirror)."""
        tmp = self.r.models / "urc_glint"
        tmp.mkdir(parents=True, exist_ok=True)
        hf = terrain.Heightfield(64.0, 129)
        hf.z = np.linspace(0.0, 0.2, 129)[None, :].repeat(129, axis=0)  # nearly flat; the PNG needs a maximum
        hf.write_png(tmp / "heightmap.png")
        cv2.imwrite(str(tmp / "colour.png"), np.full((64, 64, 3), (120, 150, 190), np.uint8))  # BGR: tan

        def build(link):
            v = sdf.sub(link, "visual", name="terrain")
            g = sdf.sub(sdf.sub(v, "geometry"), "heightmap")
            sdf.sub(g, "use_terrain_paging", False)
            t = sdf.sub(g, "texture")
            sdf.sub(t, "diffuse", "model://urc_glint/colour.png")
            sdf.sub(t, "normal", self.r.media.flat_normal())
            sdf.sub(t, "size", 64.0)
            sdf.sub(g, "uri", "model://urc_glint/heightmap.png")
            sdf.sub(g, "size", (64.0, 64.0, 0.2))

        include = model(self.r.models, "urc_glint", build)
        s = SUN.toward
        yaw, pitch = math.atan2(s[1], s[0]), math.radians(SUN.elevation_deg)  # towards the mirrored sun
        eye = (-12 * s[0], -12 * s[1], 12 * s[2])
        text = world(include + camera("glint", eye, yaw, pitch, hfov=0.8))
        brightest = {}
        for patched in (False, True):
            ground = self.r.take(text, ["/render/glint"], patched=patched)["/render/glint"].astype(float).mean(axis=2)
            brightest[patched] = (np.percentile(ground, 99.9), np.percentile(ground, 99))
        p999, p99 = brightest[True]
        self.assertLessEqual(p999, p99 + 15, brightest)
        self.assertGreater(brightest[False][0], brightest[False][1] + 15, brightest)  # the stock glint

    def terra(self, name, hf, layers):
        """A heightmap visual of hf (its own frame, lowest point 0) with
        Terra layers [(diffuse, normal, size)] and blends; its <include>."""
        directory = self.r.models / name
        directory.mkdir(parents=True, exist_ok=True)
        _, z_max = hf.write_png(directory / "heightmap.png", 0.0)

        def build(link):
            g = sdf.sub(sdf.sub(sdf.sub(link, "visual", name="terrain"), "geometry"), "heightmap")
            sdf.sub(g, "use_terrain_paging", False)
            layers(g)
            sdf.sub(g, "uri", f"model://{name}/heightmap.png")
            sdf.sub(g, "size", (hf.size, hf.size, z_max))

        return model(self.r.models, name, build)

    def test_detail_layers_keep_the_colour_map(self):
        """Terra renders a compensated colour map under the shared detail
        layers (appearance.terra_layers) in the colour map's own colours:
        seen from 25 m, each half of a two-colour map averages within 3 DN
        of the plain colour map's render (the details are zero-mean; M:
        0.1 DN in the render prototype), and the details add texture."""
        hf = terrain.Heightfield(64.0, 129).noise(1.0, 30.0, 4)
        hf.z -= hf.z.min()
        colour = np.zeros((256, 256, 3), np.uint8)
        colour[:, :128], colour[:, 128:] = (200, 170, 130), (150, 150, 145)
        plain_dir = self.r.models / "urc_plain"
        plain_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(plain_dir / "colour.png"), cv2.cvtColor(colour, cv2.COLOR_RGB2BGR))
        layers = A.terra_layers(colour, hf, A.DEFAULT_DETAILS, self.r.media)
        detail_dir = self.r.models / "urc_detail"
        detail_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(detail_dir / "colour.png"), cv2.cvtColor(layers.layer0, cv2.COLOR_RGB2BGR))

        def plain(g):
            t = sdf.sub(g, "texture")
            sdf.sub(t, "diffuse", "model://urc_plain/colour.png")
            sdf.sub(t, "normal", self.r.media.flat_normal())
            sdf.sub(t, "size", hf.size)

        cam = camera("top", (0, 0, 25.0), 0.0, math.pi / 2, (480, 480), 0.9)
        shots = {}
        for name, fill in (("urc_plain", plain),
                           ("urc_detail", lambda g: layers.write(g, "model://urc_detail/colour.png",
                                                                 self.r.media.flat_normal(), hf.size))):
            shots[name] = self.r.take(world(self.terra(name, hf, fill) + cam), ["/render/top"])["/render/top"]
        # Looking straight down, the image's top is east: the west half is the bottom rows.
        for rows in (slice(300, 460), slice(20, 180)):
            a = shots["urc_plain"][rows, 40:440].reshape(-1, 3).astype(float).mean(axis=0)
            b = shots["urc_detail"][rows, 40:440].reshape(-1, 3).astype(float).mean(axis=0)
            self.assertLess(np.abs(a - b).max(), 3.0, (a, b))
        self.assertGreater(residual(shots["urc_detail"], 2), residual(shots["urc_plain"], 2) + 1.0)

    def test_aruco_through_haze_and_noise(self):
        """ArUco 0 still decoded at 2.5 m with the haze and camera noise
        (stddev 0.06, ~2 DN, design spec 7): today's rover camera, 640 x 480
        over 1.5 rad, square on to a 20 cm tag."""
        tag = self.r.media.aruco(0)
        include = model(self.r.models, "urc_tag", lambda link: props.picture(
            link, self.r.media, "tag", tag, 0.2, 0.2, (0, 0, 0.6, 0, 0, 0)))
        text = world(include + camera("tag", (2.5, 0, 0.6), math.pi, 0.0, (640, 480), 1.5, noise=0.06))
        image = self.r.take(text, ["/render/tag"])["/render/tag"]
        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        _, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
        self.assertIsNotNone(ids)
        self.assertIn(0, ids.ravel().tolist())

    def boxes(self):
        """Boxes at 3-30 m in front of a camera at the origin, on a ground plane."""
        out = ['<model name="ground"><static>true</static><link name="link"><visual name="v"><geometry><plane>'
               '<normal>0 0 1</normal><size>200 200</size></plane></geometry></visual></link></model>']
        for k, (x, y) in enumerate(((3, -1), (6, 1.5), (12, -3), (30, 4))):
            out.append(f'<model name="box{k}"><static>true</static><pose>{x} {y} 0.5 0 0 0.3</pose><link name="link">'
                       f'<visual name="v"><geometry><box><size>1 1 1</size></box></geometry></visual></link></model>')
        return "".join(out)

    RGBD = ["/render/d/depth_image", "/render/d/image"]

    def rgbd(self, extra="", patched=True):
        """An RGB-D camera (the rover's kind: depth clipped at 40 m) looking
        at the boxes, with `extra` in the world."""
        cam = camera("d", (0, 0, 0.6), 0.0, 0.05, (320, 240), 1.2, kind="rgbd_camera")
        return self.r.take(world(self.boxes() + cam + extra, particles=True), self.RGBD, patched)

    @staticmethod
    def dust():
        """The rover's own dust emitter (gen_model._add_dust_emitter, which the rover has only with
        DriveParams.dust; its sprite from sim/models/rover) emitting at 400 /s, ten times the drivetrain's most,
        2 m in front of the camera."""
        link = ET.Element("link", name="link")
        gen_model._add_dust_emitter(link, gen_model.Params(), "e")
        emitter = link.find("particle_emitter")
        emitter.find("emitting").text, emitter.find("rate").text = "true", "400"
        emitter.find("pose").text = "0 0 0 0 0 0"
        return (f'<model name="dust"><static>true</static><pose>2 0 0.3 0 0 0</pose>'
                f'{ET.tostring(link, encoding="unicode")}</model>')

    @staticmethod
    def same_depth(a, b):
        finite = np.isfinite(a) & np.isfinite(b)
        return np.array_equal(np.isfinite(a), np.isfinite(b)) and float(np.abs(a[finite] - b[finite]).max()) < 1e-3

    def test_depth_unchanged_by_haze(self):
        """The haze patch changes the colour image, never the depth image."""
        stock, patched = self.rgbd(patched=False), self.rgbd()
        self.assertTrue(self.same_depth(stock[self.RGBD[0]], patched[self.RGBD[0]]))
        self.assertGreater(np.abs(stock[self.RGBD[1]].astype(int) - patched[self.RGBD[1]].astype(int)).max(), 5)

    def test_dust_is_drawn(self):
        """The rover's opt-in dust (DriveParams.dust, off by default) shows in
        the colour image as dust: pale tan, its sprite's colour
        (terrains.DUST_RGB), not black (an emitter material without a diffuse
        draws black smoke, measured). Kept for when the switch comes back on."""
        plain, dusty = self.rgbd(), self.rgbd(self.dust())
        before, after = plain[self.RGBD[1]].astype(int), dusty[self.RGBD[1]].astype(int)
        changed = np.abs(after - before).max(axis=-1) > 8
        self.assertGreater(changed.mean(), 0.02)
        r, g, b = after[changed].mean(axis=0)
        self.assertGreater(r, before[changed].mean() + 20, (r, g, b))  # lighter than the dark boxes behind it
        self.assertGreater(r, b + 10, (r, g, b))  # tan

    @unittest.expectedFailure
    def test_depth_unchanged_by_dust(self):
        """Design spec D15/Q11 (depth does not see dust): the depth image
        with the rover's dust emitter at full rate in view is the one without
        it. Fails in gz-sim 8.10 / gz-rendering 8.2.2 (M, 2026-10-07): the
        depth shader (depth_camera_fs.metal) takes every pixel of a particle
        with any red (particle.x > 0) for a return at a fixed scatter ratio:
        <particle_scatter_ratio> 1e-6, 0.1 and 1 changed 8,710, 8,640 and
        8,601 of 76,800 pixels, and so did ratios sent on the emitter's
        topic. Only a particle material without a diffuse stays out of the
        depth image, and it renders black. Hence the user's decision of
        2026-10-07: the rover has no dust by default (DriveParams.dust off,
        test_gen_model.Dust), and Q11 is met by its absence. Kept as an
        expected failure, the record for the opt-in switch: the emitter asks
        for a scatter ratio near none (DriveParams.dust_scatter_ratio; with
        none a gz-rendering that honours it would apply 0.65), so the test
        starts passing when that ratio is honoured, and then the dust can
        come back on."""
        plain, dusty = self.rgbd(), self.rgbd(self.dust())
        self.assertTrue(self.same_depth(plain[self.RGBD[0]], dusty[self.RGBD[0]]))

    def test_glb_clutter_stands_z_up(self):
        """A merged two-colour GLB (the clutter's format) renders upright
        where its vertices are: a red slab on the left, a blue one on the right."""
        def slabs(path):
            V, F = meshes.slab(0)
            V = V * (1.0, 1.0, 2.0)
            meshes.write_glb_parts(path, [(V + (5, 1.2, 0), F, None, None, meshes.Material((0.8, 0.05, 0.05))),
                                          (V + (5, -1.2, 0), F, None, None, meshes.Material((0.05, 0.05, 0.8)))])

        uri = self.r.media.glb("test_slabs", slabs)
        include = model(self.r.models, "urc_slabs", lambda link: sdf.sub(
            sdf.sub(sdf.sub(sdf.sub(link, "visual", name="v"), "geometry"), "mesh"), "uri", uri))
        image = self.r.take(world(include + camera("glb", (0, 0, 0.4), 0.0, 0.0, (640, 360), 1.0)),
                            ["/render/glb"])["/render/glb"].astype(int)
        # 1.2 m left and right at 5 m: columns 320 -+ 320 * 1.2 / 5 / tan(0.5) ~ 179 and 461; 0.4 m up: the middle row
        left, right = image[170:190, 160:200].reshape(-1, 3).mean(0), image[170:190, 440:480].reshape(-1, 3).mean(0)
        self.assertGreater(left[0], left[2] + 30, (left, right))
        self.assertGreater(right[2], right[0] + 30, (left, right))

    def test_far_field_on_the_horizon(self):
        """In the Autonomy area, pixels towards the Henry Mountains (Mount
        Ellen, ~35 km south) and Factory Butte (~10 km west-north-west) are
        not sky: the far field puts them there."""
        far = dem.read_geotiff(farfield.FAR_DEM)
        lat, lon = lighting.MISSION_SITE
        origin = geo.Origin(lat, lon, far.height(lat, lon) + dem.NAVD88_TO_WGS84)
        farfield.build(self.r.models, self.r.media, "urc_far_render", origin, terrain.Heightfield(64.0, 65))
        eye = 2.0
        cams, aims = "", {}
        peaks = (("ellen", ((38.09, 38.13), (-110.84, -110.79))), ("butte", ((38.46, 38.48), (-110.91, -110.88))))
        for name, box in peaks:
            (s, n), (w, e) = box
            r0, c0 = (int(v) for v in np.floor(far.pixel(n, w)))
            r1, c1 = (int(v) for v in np.ceil(far.pixel(s, e)))
            r, c = np.unravel_index(np.argmax(far.z[r0:r1, c0:c1]), (r1 - r0, c1 - c0))
            plat, plon = far.lat0 - (r0 + r + 0.5) * far.dlat, far.lon0 + (c0 + c + 0.5) * far.dlon
            x, y, _ = geo.wgs84_to_enu(origin, plat, plon)
            dist = math.hypot(x, y)
            z = float(far.z[r0 + r, c0 + c]) + dem.NAVD88_TO_WGS84 - origin.alt
            z -= (1 - farfield.REFRACTION) * dist * dist / (2 * farfield.EARTH_RADIUS)
            elevation = math.atan2(z - eye, dist)
            aims[name] = elevation
            cams += camera(name, (0, 0, eye), math.atan2(y, x), 0.0, (640, 360), 0.6)
        include = "<include><uri>model://urc_far_render</uri><pose>0 0 0 0 0 0</pose></include>"
        topics = ["/render/ellen", "/render/butte"]
        with_far = self.r.take(world(include + cams), topics)
        sky_only = self.r.take(world(cams), topics)
        focal = 320 / math.tan(0.3)
        for name, topic in zip(("ellen", "butte"), topics):
            row = int(round(180 - focal * math.tan(aims[name] - math.radians(0.4))))  # a little below the top
            a = with_far[topic][row - 2:row + 3, 316:325].reshape(-1, 3).mean(0)
            b = sky_only[topic][row - 2:row + 3, 316:325].reshape(-1, 3).mean(0)
            self.assertGreater(np.abs(a - b).max(), 20, (name, a, b, math.degrees(aims[name])))


# --- The dig-in made visible (the user's decisions of 2026-10-07) ----------------------------------------------------

ROVER = gen_model.Params()
SIDE = "/render/side"  # side_view_world's camera on the rover
SIDE_IMAGE, SIDE_DEPTH = f"{SIDE}/image", f"{SIDE}/depth_image"
SIDE_SIZE, SIDE_HFOV = (960, 540), 0.9
SIDE_POSE = (0.0, 1.95, 0.25, 0.0, 0.0, -math.pi / 2)  # in base_link: level, looking right (-y) at the left tyres
SIDE_FOCAL = SIDE_SIZE[0] / 2 / math.tan(SIDE_HFOV / 2)  # [px]
FACE = SIDE_POSE[1] - (ROVER.pivot_y + ROVER.wheel_width / 2)  # [m] the left tyres' outer faces from the camera
SPIN = [(0.0, 0.0, 0.0), (0.5, 0.0, 1.0), (5.0, 0.0, 0.0)]  # turn in place at 1 rad/s from 0.5 s to 5 s


@contextlib.contextmanager
def side_view_world(params, key):
    """The text of a world to picture (stage()): flat ground of catalogue
    type `key` (terrains.TYPES, under the strong dig-in preset) with the
    rover built from gen_model Params `params` at the origin, and a camera
    on its base_link 1.5 m beyond its left tyres (SIDE, SIDE_POSE), level. A
    sensor has no mass, so the rover drives as it does without one, and its
    tyres stay where they are in the pictures however it turns. The ground
    is drawn the type's static sinkage above its collision surface, as the
    worlds draw it. The rover's directory is not on the resource path, so
    model://rover/meshes is the tracked model's (the tread tyre)."""
    from simulate import ground_row, ground_world  # here: see _render_main
    kind = terrains.TYPES[key]
    rows = [ground_row(1, key, kind.traction, kind.appearance.dust),
            ground_row(2, "manmade", terrains.MANMADE.traction, 0.0)]
    rgb = " ".join(f"{c / 255:.4g}" for c in kind.rgb)
    surface = (f'<visual name="ground_visual"><pose>0 0 {kind.traction.sinkage_m - 0.05:.6g} 0 0 0</pose>'
               f'<geometry><box><size>32 32 0.1</size></box></geometry>'
               f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>')
    root = ET.fromstring(gen_model.build_sdf(params))
    sensor = sdf.sub(root.find("model/link[@name='base_link']"), "sensor", name="side", type="rgbd_camera")
    sdf.pose(sensor, SIDE_POSE)
    sdf.sub(sensor, "always_on", True)
    sdf.sub(sensor, "update_rate", 10)
    sdf.sub(sensor, "topic", SIDE)
    sdf.camera(sensor, SIDE_HFOV, SIDE_SIZE, (0.1, 100.0), image_format="R8G8B8", depth_clip=(0.1, 40.0))
    hf = terrain.Heightfield(32.0, 65)
    with tempfile.TemporaryDirectory(prefix="side_view_") as directory:
        model = Path(directory) / "rover"
        model.mkdir()
        (model / "model.sdf").write_text(sdf.document(root))
        (model / "model.config").write_text(sdf.model_config("rover", "The rover with a side camera.",
                                                             "sim/tests/test_render.py"))
        with ground_world(hf, np.ones((hf.n, hf.n), np.uint8), rows, terrain_extra=surface,
                          ground_options=dict(terrain_default=key, object_default="manmade"),
                          rover_uri=model.as_uri()) as path:
            yield Path(path).read_text().replace(PHYSICS_SYSTEM, PHYSICS_SYSTEM + stage())


def side_px(x, y, z):
    """(u, v) [px] of a point (x, y, z) [m] of base_link in the side camera's picture."""
    depth = SIDE_POSE[1] - y
    return SIDE_SIZE[0] / 2 - SIDE_FOCAL * x / depth, SIDE_SIZE[1] / 2 - SIDE_FOCAL * (z - SIDE_POSE[2]) / depth


def depth_changed(a, b, tolerance=1e-3):
    """Where two depth images differ by more than `tolerance` [m] (a return against none counts)."""
    finite = np.isfinite(a) & np.isfinite(b)
    with np.errstate(invalid="ignore"):
        return (np.isfinite(a) != np.isfinite(b)) | (finite & (np.abs(np.where(finite, a - b, 0.0)) > tolerance))


def top_edge(depth, u, rows):
    """The first row of `rows` (a range, top down) in column u whose depth is within 3 m: a tyre's top edge
    against the sky (side_view_world)."""
    near = np.flatnonzero(np.nan_to_num(depth[rows, u], nan=np.inf, posinf=np.inf) < 3.0)
    return rows.start + int(near[0]) if len(near) else None


class DigCues(unittest.TestCase):
    """The dig-in made visible, each cue behind its own switch and visual
    only (the user's decisions of 2026-10-07): the tyre sink
    (DriveParams.dig_sink: a dug wheel's tyre drawn (D - 1) x the ground's
    static sinkage lower, true scale) and the tread tyre (Params.tread_tyre:
    spokes, one of them ochre). The rover turns in place at 1 rad/s on
    strong-preset sand (test_drivetrain.LooseSand): its wheels dig in to
    sand's D_max 2.0 within 3 s, and it spins its wheels in place without
    turning. A camera on its base_link (side_view_world) sees the left tyres
    side on. Each run is one subprocess; the same physics, so the pictures
    differ only by the cues."""

    SAND = [("spin_a", 4900, False), ("spin_b", 100, False), ("dug", 500, True), ("reset", 1500, False)]
    ROCK = [("rock", 3500, False)]

    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.r = Renderer(cls._tmp.name)
        from simulate import physical  # here: see _render_main
        cues, no_sink = physical(), physical(dig_sink=False)  # the defaults: both cues on
        plain = dataclasses.replace(no_sink, tread_tyre=False)  # the rover before the cues
        rock_spin = [(0.0, 0.0, 0.0), (0.5, 0.0, 1.0), (3.0, 0.0, 0.0)]
        cls.runs = {}
        for name, params, key, schedule, phases in (("cues", cues, "sand", SPIN, cls.SAND),
                                                    ("no_sink", no_sink, "sand", SPIN, cls.SAND),
                                                    ("plain", plain, "sand", SPIN, cls.SAND[:2]),
                                                    ("rock", cues, "rock", rock_spin, cls.ROCK),
                                                    ("rock_no_sink", no_sink, "rock", rock_spin, cls.ROCK)):
            with side_view_world(params, key) as text:
                cls.runs[name] = cls.r.drive(text, [SIDE_IMAGE, SIDE_DEPTH], schedule, phases)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_tyres_sink_in_sand(self):
        """Dug in to D 2.0, the left tyres' top edges are (D - 1) x sand's 2 cm
        lower in the side camera's depth image than without the sink, +-1.5
        px (13.3 px at 1.5 m): the true scale (DriveParams.dig_sink_gain 1).
        The physics is the same (the drivetrain states are equal)."""
        sunk, flat = self.runs["cues"]["dug"], self.runs["no_sink"]["dug"]
        self.assertEqual(sunk.drivetrain, flat.drivetrain)
        s = terrains.SAND.traction.sinkage_m
        for wheel, x in (("fl", ROVER.wheel_dx), ("rl", -ROVER.wheel_dx)):
            dig = sunk.drivetrain["wheels"][wheel]["dig"]
            self.assertGreater(dig, 1.95, wheel)
            u, v = side_px(x, SIDE_POSE[1] - FACE, ROVER.wheel_radius)
            top = round(v - SIDE_FOCAL * ROVER.wheel_radius / FACE)
            rows = range(top - 20, round(v))
            drops = []
            for column in range(round(u) - 15, round(u) + 16):
                edges = [top_edge(frame.images[SIDE_DEPTH], column, slice(rows.start, rows.stop))
                         for frame in (sunk, flat)]
                self.assertNotIn(None, edges, (wheel, column))
                drops.append(edges[0] - edges[1])
            expected = SIDE_FOCAL * (dig - 1) * s / FACE
            self.assertAlmostEqual(float(np.median(drops)), expected, delta=1.5, msg=(wheel, drops))

    def test_only_the_tyres_change(self):
        """With the sink on, the depth image changes only where a tyre moved
        down: pixels get farther only where a tyre was (the band it left at
        its top), nearer only where a tyre now is (its lower outline, over
        the ground behind it), all within each wheel's outline and 3 cm
        round it. Nothing floats. The colour image changes on the tyres and,
        off them, only on the ground (their shadows)."""
        sunk, flat = self.runs["cues"]["dug"], self.runs["no_sink"]["dug"]
        on, off = sunk.images[SIDE_DEPTH], flat.images[SIDE_DEPTH]
        changed = depth_changed(on, off)
        boxes = np.zeros(changed.shape, bool)
        r, w, m = ROVER.wheel_radius, ROVER.wheel_width, 0.03
        for x in (ROVER.wheel_dx, -ROVER.wheel_dx):
            for y in (ROVER.pivot_y, -ROVER.pivot_y):
                corners = [side_px(x + dx, y + dy, r + dz) for dx in (-r - m, r + m) for dy in (-w / 2 - m, w / 2 + m)
                           for dz in (-r - m, r + m)]
                (u0, v0), (u1, v1) = np.min(corners, axis=0), np.max(corners, axis=0)
                boxes[max(0, int(v0)):int(v1) + 1, max(0, int(u0)):int(u1) + 1] = True
        self.assertGreater(changed.sum(), 500)
        self.assertEqual(int((changed & ~boxes).sum()), 0)

        def tyre(depth):  # pixels at a tyre's depth: the near tyres' outer faces and tread, the far ones'
            near, far = (SIDE_POSE[1] + side * ROVER.pivot_y for side in (-1, 1))
            with np.errstate(invalid="ignore"):
                return (np.abs(depth - near) < w / 2 + 0.01) | (np.abs(depth - far) < w / 2 + 0.01)

        with np.errstate(invalid="ignore"):
            farther = changed & ~(on < off)
            nearer = changed & (on < off)
        self.assertGreater(farther.sum(), 100)
        self.assertGreater(nearer.sum(), 100)
        self.assertEqual(int((farther & ~tyre(off)).sum()), 0)
        self.assertEqual(int((nearer & ~tyre(on)).sum()), 0)
        rgb = np.abs(sunk.images[SIDE_IMAGE].astype(int) - flat.images[SIDE_IMAGE].astype(int)).max(axis=2) > 10
        vv, uu = np.nonzero(rgb & ~boxes)
        self.assertTrue(len(vv))  # the tyres' shadows moved
        height = SIDE_POSE[2] - (vv - SIDE_SIZE[1] / 2) * off[vv, uu] / SIDE_FOCAL  # [m] above base_link's origin
        np.testing.assert_allclose(height, terrains.SAND.traction.sinkage_m, atol=0.01)  # on the drawn ground

    def test_nothing_sinks_on_rock(self):
        """Rock has no static sinkage and never digs: after the same spin the
        pictures with the sink on are those without it, bit for bit (they are
        deterministic)."""
        on, off = self.runs["rock"]["rock"], self.runs["rock_no_sink"]["rock"]
        self.assertEqual({w["surface"] for w in on.drivetrain["wheels"].values()}, {"rock"})
        self.assertEqual({w["dig"] for w in on.drivetrain["wheels"].values()}, {1.0})
        for topic in (SIDE_IMAGE, SIDE_DEPTH):
            np.testing.assert_array_equal(on.images[topic], off.images[topic], topic)

    def test_reset_restores_the_tyres(self):
        """A world reset clears the dig-in and puts the tyres back: 1.5 s after
        it the pictures with the sink on are those without it, bit for bit,
        where just before it they differed."""
        for topic in (SIDE_IMAGE, SIDE_DEPTH):
            self.assertFalse(np.array_equal(self.runs["cues"]["dug"].images[topic],
                                            self.runs["no_sink"]["dug"].images[topic]), topic)
            np.testing.assert_array_equal(self.runs["cues"]["reset"].images[topic],
                                          self.runs["no_sink"]["reset"].images[topic], topic)
        self.assertLess(max(w["dig"] for w in self.runs["cues"]["reset"].drivetrain["wheels"].values()), 1.01)

    def test_tread_tyre_shows_the_spin(self):
        """The tread tyre (Params.tread_tyre) on a wheel spinning in place, the
        rover dug in and still: its face is not uniform (light hub and spokes
        on the dark tyre), and between two frames 0.1 s apart its one ochre
        spoke turns by the wheel's own angle, +-30 %. The plain tyre is
        uniform, has no ochre, and looks the same in both frames."""
        tread, plain = self.runs["no_sink"], self.runs["plain"]  # no sink: the tyres where they are drawn plain
        u, v = side_px(ROVER.wheel_dx, SIDE_POSE[1] - FACE, ROVER.wheel_radius)
        radius = SIDE_FOCAL * ROVER.wheel_radius / FACE
        vv, uu = np.mgrid[:SIDE_SIZE[1], :SIDE_SIZE[0]]
        ground = side_px(0.0, SIDE_POSE[1] - FACE, terrains.SAND.traction.sinkage_m + 0.02)[1]
        disc = (np.hypot(uu - u, vv - v) < 0.85 * radius) & (vv < ground)

        def look(frame):
            image = frame.images[SIDE_IMAGE].astype(float)
            r, g, b = image[..., 0], image[..., 1], image[..., 2]
            ochre = disc & (r > b + 30)
            return image.mean(axis=2), ochre

        lum_a, ochre_a = look(tread["spin_a"])
        lum_b, ochre_b = look(tread["spin_b"])
        flat_a, plain_ochre = look(plain["spin_a"])
        flat_b, _ = look(plain["spin_b"])
        self.assertGreater(lum_a[disc].std(), 4 * max(flat_a[disc].std(), 1.0))
        self.assertGreater(np.abs(lum_a - lum_b)[disc].mean(), 2.0)
        self.assertLess(np.abs(flat_a - flat_b)[disc].mean(), 0.2)
        self.assertEqual(int(plain_ochre.sum()), 0)
        self.assertGreater(min(ochre_a.sum(), ochre_b.sum()), 50)

        def angle(ochre):  # of the ochre spoke, from the image's right towards its top: the wheel's own sense
            return math.atan2(v - vv[ochre].mean(), uu[ochre].mean() - u)

        turned = math.remainder(angle(ochre_b) - angle(ochre_a), 2 * math.pi)
        speed = np.mean([frame.drivetrain["wheels"]["fl"]["w"] for frame in (tread["spin_a"], tread["spin_b"])])
        dt = tread["spin_b"].stamps[SIDE_IMAGE] - tread["spin_a"].stamps[SIDE_IMAGE]
        self.assertAlmostEqual(dt, 0.1, delta=1e-6)
        self.assertGreater(abs(speed), 2.0)  # spinning in place
        self.assertAlmostEqual(turned, speed * dt, delta=0.3 * abs(speed * dt))


def back_project(depth, eye, target, hfov=1.0):
    """World points (h, w, 3) of a depth image from camera() at `eye` looking at `target` (aim): gz-rendering's
    depth is along the optical axis; the camera looks along its x, y left, z up."""
    h, w = depth.shape
    yaw, pitch = aim(eye, target)
    f = (w / 2) / math.tan(hfov / 2)
    v, u = np.mgrid[:h, :w].astype(float)
    cam = np.stack([depth, -(u - (w - 1) / 2) * depth / f, -(v - (h - 1) / 2) * depth / f], axis=-1)
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    rotation = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return cam @ rotation.T + np.asarray(eye)


class Ruts(unittest.TestCase):
    """The ruts and pits behind the wheels (gen_model.DriveParams.ruts, on by
    default since the user's decisions of 2026-10-07; plugins/rover_tracks.hh,
    rover_tracks_render.hh) as camera sensors see them: the station's chase
    camera and rover eye, static close-ups and an RGB-D. The proving ground's
    sand pit, the synthesis' drive: 2 m west at 0.5 m/s, a 6 s spin at 1 rad/s
    (the strong dig-in takes every wheel to D = 2), back out 5 s, then a world
    reset. Each drive is a subprocess (Renderer.drive_variant); the same
    drive on the slickrock slab (sinkage 0) beside it."""

    WORLD = "proving_ground"
    SAND = (19.5, 0.0, math.pi)  # the sand pit's flat, heading west
    ROCK = (20.5, 20.0, math.pi)  # the slickrock slab's flat
    SCHEDULE = [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0), (5.0, 0.0, 1.0), (11.0, 0.0, 0.0), (11.5, -0.5, 0.0),
                (16.5, 0.0, 0.0)]
    GRABS = [(0, 5.0), (0, 11.0), (0, 16.9), (1, 0.9)]  # driven 1.5 m, dug in, backed out; 0.9 s after the reset
    DEPTH = ((22.8, -1.6, 1.5), (17.5, 0.0, 0.0))  # the RGB-D's eye and target, above the terrain
    CAMERAS = [("top", (18.5, -2.6, 3.2), (18.5, 0.0, 0.0), "camera", (640, 360)),
               ("side", (18.0, -2.0, 0.45), (18.0, 0.0, 0.12), "camera", (640, 360)),
               ("d", *DEPTH, "rgbd_camera", (640, 360))]
    TOPICS = [gen_model.CHASE_IMAGE_TOPIC, gen_model.EYE_IMAGE_TOPIC, "/render/top", "/render/side",
              "/render/d/image", "/render/d/depth_image"]

    @classmethod
    def setUpClass(cls):
        from simulate import physical  # noqa: E402  (the Gazebo environment of the tests)
        from worldfiles import terrain as world_terrain
        cls._tmp = tempfile.TemporaryDirectory()
        cls.r = Renderer(cls._tmp.name)
        cls.hf = world_terrain(cls.WORLD)
        cls.runs = {}
        for label, pose, ruts in (("on", cls.SAND, True), ("again", cls.SAND, True), ("off", cls.SAND, False),
                                  ("rock_on", cls.ROCK, True), ("rock_off", cls.ROCK, False)):
            dy = pose[1] - cls.SAND[1]
            cameras = [(n, (e[0], e[1] + dy, e[2]), (t[0], t[1] + dy, t[2]), k, s) for n, e, t, k, s in cls.CAMERAS]
            text, rover = drive_world(cls.WORLD, pose, physical(ruts=ruts), cameras)
            rock = label.startswith("rock")
            cls.runs[label] = cls.r.drive_variant(text, rover, dict(
                name=cls.WORLD, topics=cls.TOPICS, schedule=cls.SCHEDULE, seconds=17.0,
                grabs=cls.GRABS[:3] if rock else cls.GRABS, reset_steps=0 if rock else 1000))

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def frames(self, label):
        return self.runs[label][0]

    def changed(self, topic, grab, threshold=8, a="off", b="on"):
        """Pixels of `topic` at `grab` that differ by more than `threshold` DN between runs a and b."""
        x, y = self.frames(a)[(topic, grab)].astype(int), self.frames(b)[(topic, grab)].astype(int)
        return np.abs(y - x).max(axis=-1) > threshold

    def depth_change(self, grab):
        """(on - off depth, the on run's world points) of the RGB-D at `grab`."""
        off, on = (self.frames(k)[("/render/d/depth_image", grab)] for k in ("off", "on"))
        eye, target = ((p[0], p[1], p[2] + self.hf.height(p[0], p[1])) for p in self.DEPTH)
        with np.errstate(invalid="ignore"):  # no return in either: inf - inf
            return on - off, back_project(on, eye, target)

    def wheel_paths(self, phase=0):
        """World (x, y) of the four wheel centres along the run, every 10 ms."""
        trace = self.runs["on"][1]
        trace = trace[trace[:, 0] == phase]
        p = gen_model.Params()
        out = []
        for dx, dy in ((p.wheel_dx, p.pivot_y), (p.wheel_dx, -p.pivot_y), (-p.wheel_dx, p.pivot_y),
                       (-p.wheel_dx, -p.pivot_y)):
            c, s = np.cos(trace[:, 5]), np.sin(trace[:, 5])
            out.append(np.stack([trace[:, 2] + c * dx - s * dy, trace[:, 3] + s * dx + c * dy], axis=-1))
        return np.concatenate(out)

    def test_physics_unchanged(self):
        """The ruts read the drivetrain's state and write nothing: the same
        base_link trace and drivetrain states with them on and off, on sand
        (PGS, the proving ground's solver) and on rock; the strong dig-in
        took the wheels to D = 2 on sand, so the pits were dug."""
        for on, off in (("on", "off"), ("rock_on", "rock_off")):
            np.testing.assert_array_equal(self.runs[on][1], self.runs[off][1])
            self.assertEqual(self.runs[on][2], self.runs[off][2])
        dug = max(w["dig"] for phase, s in self.runs["on"][2] if phase == 0 and s["t"] <= 11.0
                  for w in s["wheels"].values())
        self.assertGreater(dug, 1.95)

    def test_ruts_behind_the_rover_on_sand(self):
        """After 1.5 m of driving on sand the static views and the station's
        chase view show the track (at the strong preset's D 1.25: the faint
        floor and 7 mm berms): mostly darker pixels, between the wheels' start
        and where they are."""
        for topic, share in (("/render/top", 0.02), ("/render/side", 0.02), (gen_model.CHASE_IMAGE_TOPIC, 0.005)):
            self.assertGreater(self.changed(topic, 0).mean(), share, topic)
        off, on = (self.frames(k)[("/render/top", 0)].astype(int).sum(axis=-1) for k in ("off", "on"))
        changed = self.changed("/render/top", 0)
        self.assertGreater(np.mean(on[changed] < off[changed]), 0.8)  # the floor overlay darkens
        # Where the RGB-D sees changes, they lie on the wheels' paths, behind the rover.
        _, points = self.depth_change(0)
        moved = self.changed("/render/d/image", 0)
        self.assertGreater(moved.sum(), 500)
        x = points[moved][:, 0]
        trace = self.runs["on"][1]
        here = trace[(trace[:, 0] == 0) & (trace[:, 1] <= 5.0)][-1, 2]  # base_link's x; the front wheels 0.45 m on
        self.assertGreater(np.percentile(x, 5), here - 0.8)  # not ahead of the rover
        self.assertLess(np.percentile(x, 95), self.SAND[0] + 0.8)

    def test_darker_and_higher_after_digging_in(self):
        """Dug in to D = 2 by the spin, the wheels stand in pits: the RGB-D
        sees rims up to 2.6 cm x 1.35 above the surface (measured 2.9 cm; the
        berms of driving at D 1.25 reach 1.2 cm, at most 1.6 cm below D 1.3),
        and round the wheels 1.5 times as many pixels are darker, and by more,
        than there at the end of the track before the spin."""
        heights = []
        for grab in (0, 1):
            change, points = self.depth_change(grab)
            closer = np.nan_to_num(change) < -0.002
            heights.append(float(np.max(points[closer][:, 2] - self.hf.height(points[closer][:, 0],
                                                                                points[closer][:, 1]))))
        self.assertLess(heights[0], 0.017, heights)
        self.assertGreater(heights[1], 0.022, heights)
        trace = self.runs["on"][1]
        row = trace[(trace[:, 0] == 0) & (trace[:, 1] <= 11.0)][-1]  # where the wheels dug in
        p = gen_model.Params()
        c, s = math.cos(row[5]), math.sin(row[5])
        wheels = np.array([(row[2] + c * dx - s * dy, row[3] + s * dx + c * dy)
                           for dx in (p.wheel_dx, -p.wheel_dx) for dy in (p.pivot_y, -p.pivot_y)])
        eye, target = ((q[0], q[1], q[2] + self.hf.height(q[0], q[1])) for q in self.DEPTH)
        dark = []
        for grab in (0, 1):
            depth = self.frames("off")[("/render/d/depth_image", grab)]
            ground = back_project(depth, eye, target)
            near = np.isfinite(depth) & (np.min(np.hypot(ground[..., None, 0] - wheels[:, 0],
                                                         ground[..., None, 1] - wheels[:, 1]), axis=-1) < 0.35)
            off, on = (self.frames(k)[("/render/d/image", grab)].astype(int).sum(axis=-1) for k in ("off", "on"))
            diff = (on - off)[near]
            dark.append((int(np.sum(diff < -8)), float(diff[diff < -8].mean())))
        self.assertGreater(dark[1][0], 1.5 * dark[0][0], dark)
        self.assertLess(dark[1][1], dark[0][1] - 10, dark)  # sum of R, G, B [DN]

    def test_depth_only_closer_and_on_the_ground(self):
        """Depth images only get closer (berms, rims), never farther, by at
        most 15 cm along the rays; every changed pixel is on the ground (within
        -2 / +9 cm of the terrain the sheet describes) and within 0.45 m of a
        wheel's path: nothing floats. The overlay floor writes no depth."""
        paths = self.wheel_paths()
        for grab in (0, 1, 2):
            with self.subTest(grab=grab):
                change, points = self.depth_change(grab)
                off = self.frames("off")[("/render/d/depth_image", grab)]
                on = self.frames("on")[("/render/d/depth_image", grab)]
                self.assertTrue(np.array_equal(np.isfinite(off), np.isfinite(on)))
                change = np.nan_to_num(change)
                self.assertLessEqual(change.max(), 1e-4)  # never farther
                self.assertGreaterEqual(change.min(), -0.15)
                moved = change < -0.002
                self.assertGreater(moved.sum(), 500)
                p = points[moved]
                above = p[:, 2] - self.hf.height(p[:, 0], p[:, 1])
                self.assertGreaterEqual(above.min(), -0.02)
                self.assertLessEqual(above.max(), 0.09)
                gap = np.min(np.hypot(p[:, None, 0] - paths[None, ::5, 0], p[:, None, 1] - paths[None, ::5, 1]), axis=1)
                self.assertLessEqual(gap.max(), 0.45)
        # The back-projection is right: the ground pixels of the run without ruts lie on the sheet's terrain.
        eye, target = ((p[0], p[1], p[2] + self.hf.height(p[0], p[1])) for p in self.DEPTH)
        ground = back_project(self.frames("off")[("/render/d/depth_image", 0)], eye, target)[300:]
        ground = ground[np.isfinite(ground[..., 2])]
        self.assertLess(np.median(np.abs(ground[:, 2] - self.hf.height(ground[:, 0], ground[:, 1]))), 0.01)

    def test_eye_sees_the_dig_site_after_backing_out(self):
        """The rover eye (the station's main view) never sees the wheels while
        they dig in place (the head looks ahead, 0.12 rad down: the
        synthesis' documented limit); once the rover has backed out the pits
        and ruts lie ahead of it and the eye shows them."""
        self.assertFalse(self.changed(gen_model.EYE_IMAGE_TOPIC, 1, 0).any())
        self.assertGreater(self.changed(gen_model.EYE_IMAGE_TOPIC, 2).mean(), 0.01)

    def test_reset_clears_them(self):
        """A world reset clears every rut and pit: 0.9 s after it every view
        is the one without ruts, pixel for pixel (before it they differed)."""
        for topic in self.TOPICS:
            with self.subTest(topic=topic):
                before = [self.frames(k)[(topic, 2)] for k in ("off", "on")]
                after = [self.frames(k)[(topic, 3)] for k in ("off", "on")]
                self.assertFalse(np.array_equal(*before, equal_nan=True))
                self.assertTrue(np.array_equal(*after, equal_nan=True))

    def test_two_runs_are_identical(self):
        """The rendering thread draws, for each frame, exactly the ruts laid by
        the frame's sim time (rover_tracks.hh, the snapshot rule): two runs
        give the same colour and depth images, bit for bit, frame for frame
        (the same sim times)."""
        self.assertEqual(self.runs["on"][4], self.runs["again"][4])
        for (topic, grab), image in self.frames("on").items():
            with self.subTest(topic=topic, grab=grab):
                self.assertTrue(np.array_equal(image, self.frames("again")[(topic, grab)], equal_nan=True))

    def test_on_a_plane(self):
        """A world without a heightmap or an albedo map (rover_test's kind: a
        plane, its ground the drivetrain's default surface, here sand): the
        ruts lie on the plane at the contacts' height, in the default colour;
        depth only closer, every changed pixel within 9 cm above the plane."""
        from simulate import physical  # noqa: E402
        eye, target = (3.5, -2.5, 1.6), (1.0, 0.0, 0.0)
        body = ('<model name="ground"><static>true</static><link name="link"><collision name="c"><geometry><plane>'
                '<normal>0 0 1</normal><size>100 100</size></plane></geometry></collision><visual name="v"><geometry>'
                '<plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></visual></link></model>'
                '<include><uri>model://rover_drive</uri><name>rover</name><pose>2 0 0.02 0 0 3.14159265</pose>'
                '</include>'
                + camera("d", eye, *aim(eye, target), kind="rgbd_camera"))
        topics = ["/render/d/image", "/render/d/depth_image"]
        shots = {}
        for ruts in (False, True):
            rover = gen_model.build_sdf(physical(ruts=ruts, default_surface="sand"))
            shots[ruts], trace, _, _, _ = self.r.drive_variant(world(body), rover, dict(
                name="render", topics=topics, schedule=[(0.0, 0.0, 0.0), (0.5, 0.5, 0.0), (4.5, 0.0, 1.0)],
                seconds=7.0, grabs=[(0, 6.5)], reset_steps=0))
        colour = np.abs(shots[True][(topics[0], 0)].astype(int) - shots[False][(topics[0], 0)].astype(int)).max(-1)
        self.assertGreater((colour > 8).mean(), 0.005)
        off, on = shots[False][(topics[1], 0)], shots[True][(topics[1], 0)]
        with np.errstate(invalid="ignore"):  # no return in either: inf - inf
            change = np.nan_to_num(on - off)
        self.assertLessEqual(change.max(), 1e-4)
        moved = change < -0.002
        self.assertGreater(moved.sum(), 100)
        above = back_project(on, eye, target)[moved][:, 2]
        self.assertGreaterEqual(above.min(), -0.005)
        self.assertLessEqual(above.max(), 0.09)

    def test_none_on_rock(self):
        """On the slickrock slab (sinkage 0) the same drive and spin leave
        nothing: every view is the one without ruts, pixel for pixel."""
        yaw = np.unwrap(self.runs["rock_on"][1][:, 5])
        self.assertGreater(np.ptp(yaw), 1.0)  # it turned
        for key, image in self.frames("rock_on").items():
            with self.subTest(topic=key[0], grab=key[1]):
                self.assertTrue(np.array_equal(image, self.frames("rock_off")[key], equal_nan=True))


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--render":
        _render_main(sys.argv[2])
    elif len(sys.argv) == 3 and sys.argv[1] == "--drive":
        _drive_main(sys.argv[2])
    else:
        unittest.main()
