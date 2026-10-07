#!/usr/bin/env python3
"""Tests of the fly camera (plugins/fly_camera.cpp, viewers.FlyParams) and the
offline orthophoto map (tools/render_map.py).

Flights run the FlyCamera system headless in this process (no Sensors
system, so nothing renders; the deadman on sim time) and read the camera's
pose from the entity-component manager after every step. The rendering
tests serve one world each in a `gz sim` subprocess (render_map.FlyServer):
ogre2 renders once per process, on its main thread.
"""
import contextlib
import io
import json
import math
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
from PIL import Image as PILImage

from simulate import gen_model, world_sdf
from worldfiles import SIM_DIR, temp_sdf

import viewers  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import terrain as terrains  # noqa: E402

sys.path.insert(0, str(SIM_DIR / "tools"))
import render_map  # noqa: E402

import gz.math7  # noqa: E402,F401  (lets gz.sim8 return Pose3d values)
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.double_pb2 import Double  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
from gz.msgs10.vector3d_pb2 import Vector3d  # noqa: E402
from gz.msgs10.world_control_pb2 import WorldControl  # noqa: E402
from gz.sim8 import Link, Model, TestFixture, World, world_entity  # noqa: E402
from gz.transport13 import Node  # noqa: E402

FLY = replace(viewers.FlyParams(), deadman_clock="sim")
TAN = math.tan(FLY.hfov / 2)
HILL = dict(size=256.0, samples=129, height=30.0, sigma=30.0)  # a Gaussian hill at the origin


# --- Messages and worlds -------------------------------------------------------------------

def twist(forward=0.0, left=0.0, up=0.0, yaw_rate=0.0, pitch_rate=0.0):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.linear.z = forward, left, up
    msg.angular.z, msg.angular.y = yaw_rate, pitch_rate
    return msg


def text(data):
    msg = StringMsg()
    msg.data = data
    return msg


def double(value):
    msg = Double()
    msg.data = value
    return msg


def look(yaw, pitch):
    msg = Vector3d()
    msg.x, msg.y = yaw, pitch
    return msg


def goto(x, y, z, yaw, pitch, jump=False):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = x, y, z
    q = gz.math7.Quaterniond(0.0, pitch, yaw)
    msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = q.w(), q.x(), q.y(), q.z()
    if jump:
        msg.header.data.add(key=viewers.FLY_JUMP_KEY)
    return msg


def bare_world(extra):
    """DART at 1 ms, real-time factor 0, and `extra` SDF."""
    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="test">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    {extra}
  </world>
</sdf>
"""


def write_hill(directory, size, samples, height, sigma, centre=(0.0, 0.0)):
    """A 16-bit heightmap PNG of a Gaussian hill (its top the image's highest
    pixel, so Gazebo scales it to `height`) in directory; returns (path,
    Heightfield of the same samples)."""
    c = np.linspace(-size / 2, size / 2, samples)
    X, Y = np.meshgrid(c, c[::-1])
    z = np.exp(-((X - centre[0]) ** 2 + (Y - centre[1]) ** 2) / (2 * sigma ** 2))
    pixels = np.round(z / z.max() * 65535).astype(np.uint16)
    path = Path(directory) / "heightmap.png"
    PILImage.fromarray(pixels).save(path)
    return path, terrains.Heightfield(size, samples, pixels / 65535.0 * height)


def heightmap_model(path, size, height, texture=None):
    """A static terrain with a visual heightmap (the FlyCamera reads the
    visual one); texture: (diffuse, normal) PNG paths for ogre2."""
    material = ""
    if texture is not None:
        material = (f"<texture><diffuse>file://{texture[0]}</diffuse><normal>file://{texture[1]}</normal>"
                    "<size>8</size></texture>")
    return f"""<model name="terrain"><static>true</static><link name="link"><visual name="visual"><geometry>
      <heightmap><uri>file://{path}</uri><size>{size} {size} {height}</size>{material}</heightmap>
    </geometry></visual></link></model>"""


# --- Flights in this process ------------------------------------------------------------------

def fly(world, seconds, events=(), holds=(), track=(), sleeps=(), fixture_runs=None):
    """Run `world` (SDF text) for `seconds` of sim time in this process, then
    fixture_runs(fixture) if given. events: (t, topic, message) published
    once at sim time t; holds: (t0, t1, topic, message) published every 20 ms
    from t0 until before t1; neither after a world reset. track: other models
    whose canonical link's position to record. sleeps: (t, s) wall-clock
    pauses at sim time t. Returns
    {"t": [s], "camera": steps x 6 (x, y, z, roll, pitch, yaw), name: steps x 3,
    "states": [fly camera state dicts]}."""
    node = Node()
    publishers = {}
    for _, topic, message in [*events, *[(t0, topic, m) for t0, _, topic, m in holds]]:
        if topic not in publishers:
            publishers[topic] = node.advertise(topic, type(message))
    states = []
    node.subscribe(StringMsg, viewers.FLY_STATE_TOPIC, lambda m: states.append(json.loads(m.data)))
    links = {}
    out = {"t": [], "camera": [], **{name: [] for name in track}}
    last = [0]

    def pre_update(info, ecm):
        if not links:
            world_ = World(world_entity(ecm))
            for name in (viewers.FLY_MODEL, *track):
                links[name] = Link(Model(world_.model_by_name(ecm, name)).canonical_link(ecm))
        if info.iterations < last[0] or last[0] < 0:  # the world was reset: nothing more to send
            last[0] = -1
            return
        last[0] = info.iterations
        t = info.iterations / 1000
        for when, topic, message in events:
            if abs(t - when) < 5e-4:
                publishers[topic].publish(message)
        if info.iterations % 20 == 0:
            for t0, t1, topic, message in holds:
                if t0 - 5e-4 < t < t1 - 5e-4:
                    publishers[topic].publish(message)
        for when, pause in sleeps:
            if abs(t - when) < 5e-4:
                time.sleep(pause)

    def post_update(info, ecm):
        out["t"].append(info.iterations / 1000)
        pose = links[viewers.FLY_MODEL].world_pose(ecm)
        p, r = pose.pos(), pose.rot().euler()
        out["camera"].append((p.x(), p.y(), p.z(), r.x(), r.y(), r.z()))
        for name in track:
            q = links[name].world_pose(ecm).pos()
            out[name].append((q.x(), q.y(), q.z()))

    with temp_sdf(world) as path:
        fixture = TestFixture(path)
        fixture.on_pre_update(pre_update)
        fixture.on_post_update(post_update)
        fixture.finalize()
        fixture.server().run(True, round(seconds * 1000), False)
        if fixture_runs is not None:
            fixture_runs(fixture)
    result = {key: np.array(value) for key, value in out.items()}
    result["states"] = states
    return result


def at(result, t, key="camera"):
    """The row of `key` recorded after the step that ends at sim time t."""
    return result[key][int(np.argmin(np.abs(result["t"] - t)))]


class Model_(unittest.TestCase):
    """The generated model (no Gazebo)."""

    def test_model_carries_the_params(self):
        f = viewers.FlyParams()
        model = ET.fromstring(viewers.build_fly_sdf(f)).find("model")
        self.assertEqual(model.get("name"), viewers.FLY_MODEL)
        link = model.find("link")
        self.assertEqual(link.findtext("gravity"), "false")
        self.assertIsNone(link.find("collision"))
        self.assertIsNone(link.find("visual"))
        sensor = link.find("sensor")
        self.assertEqual(sensor.findtext("topic"), viewers.FLY_IMAGE_TOPIC)
        self.assertEqual((int(sensor.findtext("camera/image/width")), int(sensor.findtext("camera/image/height"))),
                         f.size)
        self.assertEqual(float(sensor.findtext("camera/horizontal_fov")), f.hfov)
        self.assertEqual((float(sensor.findtext("camera/clip/near")), float(sensor.findtext("camera/clip/far"))),
                         (0.1, 80_000.0))
        plugin = model.find("plugin")
        self.assertEqual((plugin.get("filename"), plugin.get("name")), ("FlyCamera", "rover_sim::FlyCamera"))
        values = {e.tag: e.text for e in plugin}
        for tag, value in (("target", f.target), ("clearance", f.clearance), ("time_constant", f.time_constant),
                           ("look_time_constant", f.look_time_constant), ("deadman", f.deadman),
                           ("deadman_clock", f.deadman_clock), ("speed_per_agl", f.speed_per_agl),
                           ("min_speed", f.speed_limits[0]), ("max_speed", f.speed_limits[1]), ("fast", f.fast),
                           ("min_scale", f.speed_scales[0]), ("max_scale", f.speed_scales[1]),
                           ("max_altitude", f.max_altitude), ("margin", f.margin)):
            self.assertEqual(values[tag], str(value) if isinstance(value, str) else f"{value:.9g}", tag)
        for tag, topic in (("cmd_topic", viewers.FLY_CMD_TOPIC), ("speed_topic", viewers.FLY_SPEED_TOPIC),
                           ("look_topic", viewers.FLY_LOOK_TOPIC), ("goto_topic", viewers.FLY_GOTO_TOPIC),
                           ("mode_topic", viewers.FLY_MODE_TOPIC), ("state_topic", viewers.FLY_STATE_TOPIC)):
            self.assertEqual(values[tag], topic)

    def test_write_all_writes_the_checked_in_model(self):
        with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()):
            viewers.write_all(d)
            for file in ("model.sdf", "model.config"):
                self.assertEqual((Path(d) / viewers.FLY_MODEL / file).read_text(),
                                 (SIM_DIR / "models" / viewers.FLY_MODEL / file).read_text())


class Flight(unittest.TestCase):
    """The FlyCamera system in Gazebo, pose only (nothing renders)."""

    def test_constant_command_eases_in_to_the_cruise_speed(self):
        # 20 m above flat ground (no heightmap: z = 0): cruise 20 m/s, x 0.5 multiplier.
        world = bare_world(render_map.fly_model(FLY, (0, 0, 20, 0, 0.3, 0)))
        r = fly(world, 8.0, events=[(0.1, viewers.FLY_SPEED_TOPIC, double(0.5))],
                holds=[(0.5, 3.5, viewers.FLY_CMD_TOPIC, twist(1.0)),
                       (4.0, 6.0, viewers.FLY_CMD_TOPIC, twist(10.0))])  # beyond `fast`: clamped to 4

        def x(t):
            return at(r, t)[0]

        v = 20.0 * 0.5
        tau = FLY.time_constant
        for s in (0.1, 0.2, 0.4):  # first-order ease-in from rest
            expected = v * (s - tau * (1 - math.exp(-s / tau)))
            self.assertAlmostEqual(x(0.5 + s), expected, delta=0.02 * expected + 0.011, msg=f"{s} s in")
        self.assertAlmostEqual(x(3.5) - x(2.5), v, delta=0.01 * v)
        self.assertAlmostEqual(x(6.0) - x(5.0), v * FLY.fast, delta=0.01 * v * FLY.fast)
        # After the deadman (6.3 s) the speed decays with the time constant: 40 m/s x e^-8.5 by 8 s.
        self.assertLess(abs(x(8.0) - x(7.9)), 2e-3, "stopping after the commands end")
        self.assertLess(np.abs(r["camera"][:, 1]).max(), 1e-9)
        self.assertLess(np.abs(r["camera"][:, 2] - 20.0).max(), 1e-6)
        self.assertAlmostEqual(at(r, 1.0)[4], 0.3, delta=1e-6)

    def test_deadman_stops_it(self):
        world = bare_world(render_map.fly_model(FLY, (0, 0, 10, 0, 0, 0)))
        r = fly(world, 3.0, events=[(0.5, viewers.FLY_CMD_TOPIC, twist(1.0))])
        # A first-order lag keeps the integral: the camera covers cruise x deadman.
        self.assertAlmostEqual(at(r, 3.0)[0], 10.0 * FLY.deadman, delta=0.02)
        self.assertLess(abs(at(r, 3.0)[0] - at(r, 2.5)[0]), 1e-3, "stopped: 10 m/s x e^-8.5 left")

    def test_wall_clock_deadman(self):
        """The default: a command lives 0.3 s of wall time, however fast the
        sim runs. Here the 100 steps after it take a few ms of wall time, then
        the sim stalls for 0.5 s: the camera stops after 0.1 s of sim time,
        where the sim-clock deadman would have let it fly 0.3 s."""
        fly_params = replace(FLY, deadman_clock="wall")
        world = bare_world(render_map.fly_model(fly_params, (0, 0, 10, 0, 0, 0)))
        r = fly(world, 3.0, events=[(0.5, viewers.FLY_CMD_TOPIC, twist(1.0))], sleeps=[(0.6, 0.5)])
        self.assertAlmostEqual(at(r, 3.0)[0], 10.0 * 0.1, delta=0.02)

    def test_flight_into_a_hill_keeps_clearance_and_bounds(self):
        fly_params = replace(FLY, max_altitude=60.0)
        with tempfile.TemporaryDirectory() as d:
            path, hf = write_hill(d, **HILL)
            start = (-110.0, 0.0, hf.height(-110.0, 0.0) + 2.0)
            world = bare_world(heightmap_model(path, HILL["size"], HILL["height"])
                               + render_map.fly_model(fly_params, (*start, 0, 0.2, 0)))
            # Full speed: 4 x the multiplier 4 x cruise, 32 m/s at 2 m up, faster as it climbs.
            r = fly(world, 12.0, events=[(0.1, viewers.FLY_SPEED_TOPIC, double(4.0))],
                    holds=[(0.2, 8.0, viewers.FLY_CMD_TOPIC, twist(fly_params.fast)),
                           (8.0, 12.0, viewers.FLY_CMD_TOPIC, twist(up=fly_params.fast))])
        x, y, z = r["camera"][:, 0], r["camera"][:, 1], r["camera"][:, 2]
        ground = hf.height(x, y)
        self.assertGreaterEqual((z - ground).min(), 0.3 - 1e-5, "hard floor")  # float32 heights in the plugin
        self.assertGreaterEqual((z - ground).min(), fly_params.clearance - 0.05, "clearance, looking ahead")
        crest = int(np.argmin(np.abs(x)))
        self.assertGreater(z[crest], HILL["height"] + fly_params.clearance - 0.05, "over the top")
        edge = HILL["size"] / 2 + fly_params.margin
        self.assertAlmostEqual(at(r, 8.0)[0], edge, delta=1e-6, msg="stops at the terrain edge + margin")
        self.assertAlmostEqual(at(r, 12.0)[2], HILL["height"] + fly_params.max_altitude, delta=1e-6,
                               msg="stops at max_altitude above the highest terrain")
        self.assertLessEqual(x.max(), edge + 1e-9)

    def test_follow_keeps_the_offset(self):
        world = world_sdf(render_map.fly_model(FLY, (-6, 0, 3, 0, 0.4, 0)))
        r = fly(world, 9.0, events=[(1.0, viewers.FLY_MODE_TOPIC, text("follow")),
                                    (6.0, viewers.FLY_MODE_TOPIC, text("free"))],
                holds=[(0.0, 9.0, gen_model.CMD_VEL_TOPIC, twist(0.6))], track=["rover"])

        def offset(t):
            return np.array(at(r, t)[:3]) - at(r, t, "rover")

        self.assertGreater(np.linalg.norm(at(r, 5.9, "rover") - at(r, 1.1, "rover")), 2.0, "the rover drove")
        self.assertLess(np.linalg.norm(offset(5.9) - offset(1.1)), 0.01)
        self.assertLess(np.linalg.norm(np.array(at(r, 9.0)[:3]) - at(r, 6.1)[:3]), 1e-3, "free: it stays")
        self.assertEqual(next(s["mode"] for s in r["states"] if s["t"] > 3.0), "follow")

    def test_goto_jump_look_modes_and_orthographic_state(self):
        world = bare_world(render_map.fly_model(FLY, (0, 0, 10, 0, 0, 0)))
        flight = math.dist((0, 0, 10), (40, 30, 30)) / 50.0  # [s] distance / 50 m/s within 0.4-1.5 s
        r = fly(world, 10.5, events=[
            (0.5, viewers.FLY_GOTO_TOPIC, goto(40, 30, 30, 1.0, 0.5)),
            (2.0, viewers.FLY_LOOK_TOPIC, look(0.5, 0.3)),
            (3.0, viewers.FLY_MODE_TOPIC, text("top")),
            (4.0, viewers.FLY_MODE_TOPIC, text("level")),
            (5.0, viewers.FLY_GOTO_TOPIC, goto(-20, 10, 15, -2.0, 0.1, jump=True)),
            (6.5, viewers.FLY_MODE_TOPIC, text("stop")),
            (7.5, viewers.FLY_MODE_TOPIC, text("ortho")),
            (7.8, viewers.FLY_LOOK_TOPIC, look(0.0, -0.5)),
            (8.5, viewers.FLY_MODE_TOPIC, text("ortho 50")),
            (9.5, viewers.FLY_MODE_TOPIC, text("perspective")),
            (9.6, viewers.FLY_LOOK_TOPIC, look(0.0, -0.5))],
            holds=[(6.0, 6.5, viewers.FLY_CMD_TOPIC, twist(1.0))])
        middle = at(r, 0.5 + flight / 2)  # smoothstep: half way at half time
        np.testing.assert_allclose(middle[:3], (20, 15, 20), atol=0.05)
        self.assertAlmostEqual(middle[5], 0.5, delta=0.01)
        self.assertAlmostEqual(middle[4], 0.25, delta=0.01)
        np.testing.assert_allclose(at(r, 0.55 + flight), (40, 30, 30, 0, 0.5, 1.0), atol=1e-3)
        # A look delta is smoothed (0.08 s) towards yaw 1.5, pitch 0.8.
        self.assertAlmostEqual(at(r, 2.0 + FLY.look_time_constant)[5], 1.0 + 0.5 * (1 - math.exp(-1)), delta=0.01)
        np.testing.assert_allclose(at(r, 2.9)[4:], (0.8, 1.5), atol=1e-3)
        self.assertAlmostEqual(at(r, 3.9)[4], math.pi / 2, delta=1e-3)
        self.assertAlmostEqual(at(r, 4.9)[4], 0.0, delta=1e-3)
        np.testing.assert_allclose(at(r, 5.003), (-20, 10, 15, 0, 0.1, -2.0), atol=1e-6, err_msg="jump: at once")
        self.assertGreater(math.dist(at(r, 6.5)[:3], at(r, 6.4)[:3]), 1.0, "15 m/s")
        self.assertLess(math.dist(at(r, 7.4)[:3], at(r, 6.503)[:3]), 1e-6, "stop: halts at once")

        def state(t):
            return next(s for s in r["states"] if s["t"] >= t)

        self.assertAlmostEqual(state(7.7)["ortho"], 2 * 15 * TAN, delta=1e-3)
        self.assertAlmostEqual(at(r, 8.2)[4], math.pi / 2, delta=1e-6, msg="orthographic looks straight down")
        self.assertAlmostEqual(at(r, 9.4)[2], 50 / (2 * TAN), delta=1e-3, msg="ortho <width> sets the height")
        self.assertAlmostEqual(state(9.4)["ortho"], 50.0, delta=1e-3)
        self.assertEqual(state(9.7)["ortho"], 0.0)
        self.assertAlmostEqual(at(r, 10.5)[4], math.pi / 2 - 0.5, delta=1e-3, msg="perspective: the pitch is free")

    def test_reset_returns_to_the_start(self):
        start = (5.0, -3.0, 12.0, 0.0, 0.2, 0.7)
        world = bare_world(render_map.fly_model(FLY, start))
        after = {}

        def reset_and_run(fixture):
            request = WorldControl()
            request.reset.all = True
            # The reply may be lost while this process subscribes (README "Gazebo
            # lessons"); the server acts on the request anyway, so the pose decides.
            Node().request("/world/test/control", request, WorldControl, Boolean, 2000)
            fixture.server().run(True, 500, False)
            after["done"] = True

        r = fly(world, 1.0, events=[(0.1, viewers.FLY_MODE_TOPIC, text("ortho"))],
                holds=[(0.2, 1.0, viewers.FLY_CMD_TOPIC, twist(1.0, 0.5, 0.0, 0.3))], fixture_runs=reset_and_run)
        self.assertTrue(after)
        self.assertGreater(np.linalg.norm(at(r, 1.0)[:3] - np.array(start[:3])), 5.0, "it flew away")
        t = r["t"]
        restart = int(np.argmax(np.diff(t) < 0)) + 1
        self.assertGreater(restart, 1, "the world was reset")
        np.testing.assert_allclose(r["camera"][-1], start, atol=1e-6)
        last = r["states"][-1]
        self.assertEqual((last["mode"], last["ortho"], last["speed"], last["goto"]), ("free", 0.0, 1.0, False))


# --- The map tool --------------------------------------------------------------------------

def synthetic_tiles(hf, size, pixels, max_tile_m, colour):
    """Tile pictures as render_map.capture would get them over terrain hf
    whose ground has colour(x, y) -> (..., 3): each pixel's ray from the tile
    camera followed down to the terrain (fixed point: the slopes are gentle)."""
    k, tile, px, hfov = render_map.layout(size, pixels, max_tile_m)
    cam_z = float(hf.z.max()) + render_map.ABOVE
    f = (px / 2) / math.tan(hfov / 2)
    u, v = np.meshgrid(np.arange(px) + 0.5 - px / 2, np.arange(px) + 0.5 - px / 2)
    tiles = []
    for cx, cy in render_map.tile_centres(size, k):
        X, Y = cx + u / f * render_map.ABOVE, cy - v / f * render_map.ABOVE
        for _ in range(20):
            d = cam_z - hf.height(X, Y)
            X, Y = cx + u / f * d, cy - v / f * d
        tiles.append((cx, cy, np.clip(colour(X, Y), 0, 255).astype(np.uint8)))
    return tile, cam_z, hfov, tiles


class MapTool(unittest.TestCase):
    """render_map without Gazebo: tiling, reprojection, inputs and staleness."""

    def test_layout_of_the_urc_worlds(self):
        # The prototype's measured tilings (sim/data/research/flycam/urc_*_ortho.json).
        k, tile, px, hfov = render_map.layout(256.0, 4096)
        self.assertEqual((k, tile, px), (4, 64.0, 1270))
        self.assertAlmostEqual(hfov, 0.26300670, places=6)
        k, tile, px, hfov = render_map.layout(2048.0, 4096)
        self.assertEqual((k, tile, px), (8, 256.0, 635))
        self.assertAlmostEqual(hfov, 0.97325929, places=6)
        k, tile, px, hfov = render_map.layout(20.0, 512)  # smaller than the narrowest view
        self.assertEqual((k, tile), (1, 20.0))
        self.assertAlmostEqual(hfov, render_map.MIN_HFOV)
        self.assertEqual(px, round(2 * render_map.ABOVE * math.tan(render_map.MIN_HFOV / 2) / (20.0 / 512)))
        with self.assertRaises(ValueError):
            render_map.layout(20.0, 4096)

    def test_assemble_reprojects_the_ground(self):
        size, pixels = 128.0, 512
        with tempfile.TemporaryDirectory() as d:
            _, hf = write_hill(d, size, 65, 12.0, 25.0, centre=(15.0, -10.0))

        def colour(x, y):
            return np.stack([127 + 100 * np.sin(x / 7.0) * np.cos(y / 5.0), 127 + 100 * np.cos(x / 4.0 + 1.0),
                             127 + 100 * np.sin(y / 6.0 - 0.5)], axis=-1)

        tile, cam_z, hfov, tiles = synthetic_tiles(hf, size, pixels, 64.0, colour)
        self.assertEqual(len(tiles), 4)
        image = render_map.assemble(hf.z, size, pixels, tile, cam_z, hfov, tiles).astype(float)
        c = -size / 2 + (np.arange(pixels) + 0.5) * size / pixels
        X, Y = np.meshgrid(c, c[::-1])
        expected = np.clip(colour(X, Y), 0, 255)
        self.assertLess(np.abs(image - expected).max(), 3.0, "rounding and interpolation only, seams included")
        # Without the terrain height the hill's flanks land up to 1.5 m off.
        flat = render_map.assemble(np.zeros_like(hf.z), size, pixels, tile, cam_z, hfov, tiles).astype(float)
        self.assertGreater(np.percentile(np.abs(flat - expected), 99), 8.0)

    def test_feather_weights_sum_to_one(self):
        m, feather = 64, render_map.FEATHER_PX
        index = np.arange(-feather, 2 * m + feather)
        total = render_map._ramp(index, 0, m, feather) + render_map._ramp(index, m, m, feather)
        np.testing.assert_allclose(total[(index >= feather // 2) & (index < 2 * m - feather // 2)], 1.0)
        self.assertEqual(render_map._ramp(np.array([m // 2]), 0, m, feather)[0], 1.0)

    def test_status_follows_the_inputs(self):
        world = SIM_DIR / "worlds" / "urc_equipment_servicing.sdf"
        found = render_map.inputs(world)
        self.assertIn("worlds/urc_equipment_servicing.sdf", found)
        self.assertIn("models/urc_terrain_equipment_servicing/heightmap.png", found)
        self.assertTrue(any(k.startswith("models/urc_media/textures/") for k in found))
        self.assertFalse(any("models/rover/" in k for k in found), "the rover is not on the map")
        self.assertNotIn("missing", found.values())
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(render_map.status(world, d), ("missing", []))
            image, meta = render_map.map_paths(world, d)
            image.write_bytes(b"")
            meta.write_text(json.dumps({"format": render_map.FORMAT, "inputs": found}))
            self.assertEqual(render_map.status(world, d), ("current", []))
            key = "models/urc_terrain_equipment_servicing/heightmap.png"
            meta.write_text(json.dumps({"format": render_map.FORMAT, "inputs": {**found, key: "0" * 40}}))
            self.assertEqual(render_map.status(world, d), ("stale", [key]))


# --- Rendering, one world per gz sim subprocess ----------------------------------------------------

LIT_WORLD = """<?xml version="1.0"?>
<sdf version="1.11">
  <world name="{name}">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.5 0.6 0.8 1</background><grid>false</grid></scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <direction>-0.3 0.2 -0.9</direction>
      <diffuse>1 1 1 1</diffuse>
    </light>
    {extra}
  </world>
</sdf>
"""


def marker(name, xyz, rgb, size):
    """A static, self-lit box (a disc would do as well) of `size` (x, y, z) [m]."""
    colour = " ".join(map(str, rgb))
    return f"""<model name="{name}"><static>true</static><pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0</pose>
      <link name="link"><visual name="visual"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>
      <material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse><emissive>{colour} 1</emissive></material>
      </visual></link></model>"""


def colour_mask(rgb, channel):
    """Pixels where `channel` (0 red, 1 green, 2 blue) dominates."""
    rgb = rgb.astype(int)
    others = [c for c in range(3) if c != channel]
    return (rgb[..., channel] > 150) & (rgb[..., others[0]] < 90) & (rgb[..., others[1]] < 90)


class Rendering(unittest.TestCase):
    """The camera's pictures (GPU; each world in its own gz sim process)."""

    def test_orthographic_projection(self):
        f = replace(FLY, size=(640, 480))
        extra = (marker("pole", (0, 0, 5), (1, 0, 0), (2, 2, 10))
                 + """<model name="ground"><static>true</static><link name="link"><visual name="visual">
                   <geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
                   <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
                   </visual></link></model>"""
                 + render_map.fly_model(f, (0, 0, 40, 0, math.pi / 2, 0)))
        focal = 320 / TAN

        def width(server, state):
            _, rgb = server.frame_after(state["t"] + 0.2)
            columns = np.where(colour_mask(rgb, 0).any(axis=0))[0]
            return columns.max() - columns.min() + 1

        with temp_sdf(LIT_WORLD.format(name="ortho", extra=extra)) as path, render_map.FlyServer(path) as server:
            perspective = width(server, server.state())
            ortho = width(server, server.mode("ortho", lambda s: s["ortho"] > 0))
            wide = server.mode("ortho 120", lambda s: abs(s["ortho"] - 120) < 1e-3 and not s["goto"])
            ortho_wide = width(server, wide)
            back = server.mode("perspective", lambda s: s["ortho"] == 0)
            perspective_high = width(server, back)
        self.assertAlmostEqual(perspective, 2 * focal / 30, delta=1.5, msg="perspective: the top, 30 m off")
        self.assertAlmostEqual(ortho, 2 * 640 / (2 * 40 * TAN), delta=1.5, msg="orthographic, 40 m up")
        self.assertAlmostEqual(wide["z"], 120 / (2 * TAN), delta=1e-2)
        self.assertAlmostEqual(ortho_wide, 2 * 640 / 120, delta=1.5, msg="orthographic, 120 m wide")
        self.assertAlmostEqual(perspective_high, 2 * focal / (wide["z"] - 10), delta=1.5, msg="perspective again")

    def test_map_of_a_hilly_world(self):
        size, height = 256.0, 20.0
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            path, hf = write_hill(d, size, 129, height, 40.0, centre=(20.0, -10.0))
            rng = np.random.default_rng(1)
            cv2.imwrite(str(d / "diffuse.png"), rng.integers(120, 160, (64, 64, 3), dtype=np.uint8))
            cv2.imwrite(str(d / "normal.png"), np.full((64, 64, 3), (255, 128, 128), np.uint8))  # BGR: flat
            # Self-lit markers on tile seams (x = 0, y = 0), inside a tile and on the hill's flank, 4 m x 4 m.
            places = {"red": (0.0, 40.0, 0), "green": (50.0, 0.0, 1), "blue": (-60.0, -70.0, 2),
                      "flank": (40.0, -30.0, 0)}
            extra = heightmap_model(path, size, height, (d / "diffuse.png", d / "normal.png"))
            for name, (x, y, channel) in places.items():
                rgb = [1 if c == channel else 0 for c in range(3)]
                extra += marker(name, (x, y, hf.height(x, y) + 0.2), rgb, (4, 4, 0.4))
            world = d / "hill.sdf"
            world.write_text(LIT_WORLD.format(name="hill", extra=extra))
            (d / "hill.json").write_text(json.dumps({"terrain": {"heightmap": path.name, "size_m": size,
                                                                 "z_max": height, "samples": 129}}))
            meta = render_map.render(world, pixels=1024, out_dir=d / "out", max_tile_m=128.0)
            image = cv2.cvtColor(cv2.imread(str(d / "out" / "hill_map.jpg")), cv2.COLOR_BGR2RGB)
            state = render_map.status(world, d / "out")
        self.assertEqual(image.shape, (1024, 1024, 3))
        self.assertEqual((meta["tiles_per_side"], meta["gsd_m"]), (2, 0.25))
        self.assertEqual(state, ("current", []))
        for name, (x, y, channel) in places.items():
            mask = colour_mask(image, channel)
            if name == "red":
                mask[:, 600:] = False  # the flank marker is red too, south-east of it
            elif name == "flank":
                mask[:, :600] = False
            rows, cols = np.nonzero(mask)
            self.assertGreater(len(rows), 0.6 * 16 / 0.25 ** 2, name)
            self.assertLess(len(rows), 1.4 * 16 / 0.25 ** 2, f"{name}: one marker, not two")
            mx, my = -size / 2 + (cols.mean() + 0.5) * 0.25, size / 2 - (rows.mean() + 0.5) * 0.25
            self.assertLess(math.hypot(mx - x, my - y), 0.5, f"{name} at {mx:.2f}, {my:.2f}, not {x}, {y}")


if __name__ == "__main__":
    unittest.main()
