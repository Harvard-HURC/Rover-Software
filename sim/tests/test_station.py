#!/usr/bin/env python3
"""Tests of the driver station (sim/station) and its cameras
(plugins/chase_camera.cpp): the drive and look mappings, the web app against
a fake transport, the chase camera flying after a driving rover in Gazebo,
and the rover eye riding on it."""
import asyncio
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from aiohttp import WSMsgType
from aiohttp.test_utils import AioHTTPTestCase

from simulate import gen_model, world_sdf
from worldfiles import SENSORS, SIM_DIR, WORLDS, temp_sdf

from station import drive, minimap, server, video  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import sheet as sheets  # noqa: E402

import gz.math7  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
from gz.msgs10.vector3d_pb2 import Vector3d  # noqa: E402
from gz.sim8 import Joint, Link, Model, TestFixture, World, world_entity  # noqa: E402
from gz.transport13 import Node  # noqa: E402

from station import link as gz_link  # noqa: E402

P = gen_model.Params()


def command(*keys, axes=(0.0, 0.0)):
    return drive.Command(frozenset(keys), axes)


class DriveMapping(unittest.TestCase):
    def test_keys_are_full_stick_of_the_preset(self):
        linear, angular = drive.PRESETS[2]
        self.assertEqual(drive.target(command("KeyW"), 2), (linear, 0.0))
        self.assertEqual(drive.target(command("KeyS", "KeyD"), 2), (-linear, -angular))
        self.assertEqual(drive.target(command("KeyA"), 1), (0.0, drive.PRESETS[1][1]))
        self.assertEqual(drive.target(command("KeyW", "KeyS"), 2), (0.0, 0.0))

    def test_shift_doubles_up_to_the_rover_limits(self):
        linear, angular = drive.PRESETS[1]
        self.assertEqual(drive.target(command("KeyW", "KeyA", "ShiftLeft"), 1), (2 * linear, 2 * angular))
        vx, wz = drive.target(command("KeyW", "KeyA", "ShiftRight"), 3)
        self.assertEqual((vx, wz), (min(2 * drive.PRESETS[3][0], drive.MAX_LINEAR),
                                    min(2 * drive.PRESETS[3][1], drive.MAX_ANGULAR)))
        self.assertLessEqual(vx, P.wheel_speed * P.wheel_radius)
        self.assertEqual(drive.target(command("KeyW", "GamepadFast"), 1)[0], 2 * linear)

    def test_brake_wins(self):
        self.assertEqual(drive.target(command("KeyW", "Space"), 3), (0.0, 0.0))
        self.assertEqual(drive.target(command("GamepadBrake", axes=(1.0, 1.0)), 3), (0.0, 0.0))

    def test_sticks_have_a_dead_zone_and_add_to_keys(self):
        self.assertEqual(drive.stick(drive.DEADZONE * 0.9), 0.0)
        self.assertEqual(drive.stick(-1.0), -1.0)
        self.assertAlmostEqual(drive.stick(0.5), (0.5 - drive.DEADZONE) / (1 - drive.DEADZONE))
        linear, angular = drive.PRESETS[2]
        vx, wz = drive.target(command(axes=(0.5, -1.0)), 2)
        self.assertAlmostEqual(vx, drive.stick(0.5) * linear)
        self.assertAlmostEqual(wz, -angular)
        self.assertEqual(drive.target(command("KeyW", axes=(1.0, 0.0)), 2)[0], linear)  # clipped

    def test_unknown_keys_do_nothing(self):
        self.assertEqual(drive.target(command("KeyQ", "ArrowUp"), 2), (0.0, 0.0))

    def test_slew_limits_acceleration_and_braking(self):
        self.assertAlmostEqual(drive.slew(0.0, 1.0, 0.1, 1.0, 2.0), 0.1)
        self.assertAlmostEqual(drive.slew(1.0, 0.0, 0.1, 1.0, 2.0), 0.8)  # slowing: decel
        self.assertAlmostEqual(drive.slew(0.5, -0.5, 0.1, 1.0, 2.0), 0.3)  # reversing slows first
        self.assertAlmostEqual(drive.slew(-0.1, -0.5, 0.1, 1.0, 2.0), -0.2)
        self.assertEqual(drive.slew(0.45, 0.5, 0.1, 1.0, 2.0), 0.5)  # no overshoot


class DriveLoop(unittest.TestCase):
    def run_steps(self, d, start, end, dt=0.05, command_=None):
        t = start
        while t < end - 1e-9:
            if command_ is not None:
                d.input(command_, t)
            d.step(t)
            t += dt
        return t

    def test_silent_page_holds_the_rover(self):
        d = drive.Drive()
        self.assertEqual(d.step(0.0), (0.0, 0.0))
        self.assertTrue(d.deadman)

    def test_ramps_up_at_the_acceleration_limit(self):
        d = drive.Drive()
        t = self.run_steps(d, 0.0, 0.3, command_=command("KeyW", "KeyA"))
        d.input(command("KeyW", "KeyA"), t)
        vx, wz = d.step(t)
        self.assertFalse(d.deadman)
        self.assertAlmostEqual(vx, drive.ACCEL[0] * t, places=6)
        self.assertAlmostEqual(wz, drive.ACCEL[1] * t, places=6)
        self.run_steps(d, t, 3.0, command_=command("KeyW", "KeyA"))
        self.assertEqual(d.twist, drive.PRESETS[2])

    def test_deadman_after_half_a_second_of_silence(self):
        d = drive.Drive()
        t = self.run_steps(d, 0.0, 2.0, command_=command("KeyW"))
        self.assertGreater(d.twist[0], 0)
        last_input = t - 0.05
        d.step(last_input + drive.DEADMAN - 0.05)
        self.assertGreater(d.twist[0], 0)
        self.assertEqual(d.step(last_input + drive.DEADMAN + 0.05), (0.0, 0.0))
        self.assertTrue(d.deadman)

    def test_space_stops_at_once(self):
        d = drive.Drive()
        t = self.run_steps(d, 0.0, 2.0, command_=command("KeyW"))
        d.input(command("KeyW", "Space"), t)
        self.assertEqual(d.step(t), (0.0, 0.0))

    def test_taking_control_back_keeps_the_preset(self):
        station = server.Station(FakeLink(), "test")
        station.handle({"t": "preset", "n": 1}, 0.0)
        station.set_control(False)
        station.set_control(True)
        self.assertEqual(station.drive.preset, 1)
        self.run_steps(station.drive, 0.0, 3.0, command_=command("KeyW"))
        self.assertEqual(station.drive.twist[0], drive.PRESETS[1][0])

    def test_numbers_must_be_finite(self):
        """json.loads takes NaN, Infinity and 1e309; one NaN would stick in the chase camera's view."""
        link = FakeLink()
        station = server.Station(link, "test")
        for msg in ({"t": "chase", "yaw": float("inf")}, {"t": "chase", "pitch": float("nan")},
                    {"t": "look", "pan": float("-inf")}, {"t": "input", "keys": [], "axes": [float("nan"), 0]}):
            with self.assertRaises(ValueError, msg=msg):
                station.handle(msg, 0.0)
        self.assertEqual(link.sent, [])
        self.assertEqual((station.look.pan, station.look.tilt), (0.0, P.camera_pitch))


class LookMapping(unittest.TestCase):
    """The look: the rover eye's yaw and pitch, which the camera head follows."""

    def test_nudges_add_up_inside_the_head_limits(self):
        look = drive.Look()
        self.assertEqual((look.pan, look.tilt), (0.0, P.camera_pitch))
        look.nudge(0.3, 0.1)
        look.nudge(-0.1, 0.05)
        self.assertAlmostEqual(look.pan, 0.2)
        self.assertAlmostEqual(look.tilt, P.camera_pitch + 0.15)
        look.nudge(10.0, -10.0)
        self.assertEqual((look.pan, look.tilt), (P.camera_pan_limit, P.camera_tilt_limits[0]))
        look.nudge(-20.0, 20.0)
        self.assertEqual((look.pan, look.tilt), (-P.camera_pan_limit, P.camera_tilt_limits[1]))
        look.center()
        self.assertEqual((look.pan, look.tilt), (0.0, P.camera_pitch))

    def test_the_eye_model_starts_and_stops_like_the_head(self):
        plugin = ET.fromstring(gen_model.build_eye_sdf(gen_model.EyeParams(), P)).find("model/plugin")
        value = lambda tag: float(plugin.findtext(tag))  # noqa: E731
        look = drive.Look()
        self.assertEqual((value("yaw"), value("pitch")), (look.pan, look.tilt))
        look.nudge(10.0, 10.0)
        self.assertEqual((value("max_yaw"), value("max_pitch")), (look.pan, look.tilt))
        look.nudge(-20.0, -20.0)
        self.assertEqual((value("min_yaw"), value("min_pitch")), (look.pan, look.tilt))

    def test_the_eye_always_the_head_only_in_control(self):
        link = FakeLink()
        station = server.Station(link, "test")
        station.handle({"t": "look", "pan": 0.5, "tilt": -0.2}, 0.0)
        self.assertEqual(link.sent, [("eye", 0.5, P.camera_pitch - 0.2), ("head", 0.5, P.camera_pitch - 0.2)])
        station.set_control(False)
        link.sent.clear()
        station.handle({"t": "look", "pan": 0.1}, 0.1)
        station.handle({"t": "look_center"}, 0.2)
        self.assertEqual(link.sent, [("eye", 0.6, P.camera_pitch - 0.2), ("eye", 0.0, P.camera_pitch)])

    def test_resent_every_period_for_late_subscribers(self):
        link = FakeLink()
        station = server.Station(link, "test", control=False)
        for k in range(61):  # 3 s of control ticks
            station.tick(k * server.CONTROL_PERIOD)
        self.assertEqual(link.of("eye"), [("eye", 0.0, P.camera_pitch)] * 4)  # at 0, 1, 2 and 3 s
        self.assertEqual(link.of("head") + link.of("twist"), [])  # released: the head is autonomy's


def image(pixel_format, width=32, height=24):
    msg = Image()
    msg.width, msg.height, msg.pixel_format_type = width, height, pixel_format
    if pixel_format == video.RGB_INT8:
        rgb = np.zeros((height, width, 3), np.uint8)
        rgb[..., 0] = 200  # red
        msg.data = rgb.tobytes()
    else:
        depth = np.linspace(0.05, 50.0, width * height, dtype=np.float32)
        depth[0] = np.inf
        msg.data = depth.tobytes()
    msg.step = len(msg.data) // height
    return msg


class Video(unittest.TestCase):
    def test_depth_is_coloured_and_no_return_is_black(self):
        bgr = video.to_bgr(image(video.R_FLOAT32), P.camera_clip)
        self.assertEqual(bgr.shape, (24, 32, 3))
        self.assertEqual(bgr[0, 0].tolist(), [0, 0, 0])  # inf: no return
        self.assertEqual(bgr[-1, -1].tolist(), [0, 0, 0])  # beyond the far clip
        self.assertNotEqual(bgr[0, 1].tolist(), bgr[12, 16].tolist())  # near and far differ

    def test_one_subscription_and_the_first_picture_dropped(self):
        subscribed = []
        feed = video.Feed("rgb", "/test/rgb", subscribed.append, subscribed.remove)
        feed.watch()
        feed.watch()
        self.assertEqual(subscribed, [feed])
        black = image(video.RGB_INT8)
        black.data = bytes(len(black.data))
        feed.on_image(black)
        feed.on_image(image(video.RGB_INT8))
        self.assertEqual(feed.seq, 1)
        seq, jpeg = feed.jpeg()
        picture = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(seq, 1)
        self.assertGreater(picture[..., 2].mean(), 150)  # the red test picture, not the black one
        self.assertIs(feed.jpeg()[1], jpeg)  # encoded once
        feed.unwatch()
        feed.unwatch()
        self.assertEqual(subscribed, [])


class FakeLink:
    """GzLink's interface without Gazebo: records what the station sends and
    feeds cameras with test pictures every 20 ms while someone watches."""

    def __init__(self):
        self.sent = []
        self.watching = set()
        self.state = {"stats": {"sim_time": 0.0, "rtf": 1.0, "paused": False}}
        self.others = []
        self.feeds = {name: video.Feed(name, f"/test/{name}", self._watch, self._unwatch)
                      for name in gz_link.CAMERAS}

    def _watch(self, feed):
        self.watching.add(feed.name)
        msg = image(video.R_FLOAT32 if feed.name == "depth" else video.RGB_INT8)

        def pump():
            while feed.name in self.watching:
                feed.on_image(msg)
                time.sleep(0.02)

        threading.Thread(target=pump, daemon=True).start()

    def _unwatch(self, feed):
        self.watching.discard(feed.name)

    def twist(self, vx, wz):
        self.sent.append(("twist", vx, wz))

    def led(self, colour):
        self.sent.append(("led", colour))

    def chase(self, yaw, pitch, zoom):
        self.sent.append(("chase", yaw, pitch, zoom))

    def chase_mode(self, mode):
        self.sent.append(("chase_mode", mode))

    def eye(self, yaw, pitch):
        self.sent.append(("eye", yaw, pitch))

    def head(self, pan, tilt):
        self.sent.append(("head", pan, tilt))

    def announce(self, url):
        self.sent.append(("announce", url))

    def other_stations(self):
        return list(self.others)

    def online(self):
        return "stats" in self.state

    def ensure_cameras(self):
        self.sent.append(("ensure_cameras",))
        return ["eye_camera: spawned", "chase_camera: spawned"]

    def snapshot(self):
        return dict(self.state)

    def of(self, kind):
        return [s for s in self.sent if s[0] == kind]


class Telemetry(unittest.TestCase):
    def test_rockers_and_wheels(self):
        link = FakeLink()
        link.state["joints"] = {"rocker_left_joint": (0.03, 0.0), "rocker_right_joint": (-0.028, 0.0),
                                "wheel_fl_joint": (1.0, 2.0), gen_model.PAN_JOINT: (0.1, 0.0),
                                gen_model.TILT_JOINT: (0.2, 0.0)}
        t = server.Station(link, "test").telemetry(0.0)
        self.assertEqual(t["rockers"]["left"], 0.03)
        self.assertAlmostEqual(t["rockers"]["error"], 0.002)  # what the differential keeps at 0
        self.assertEqual(t["wheels"], {"fl": 2.0 * P.wheel_radius})  # ground speed of the wheel
        self.assertEqual(t["head"], {"pan": 0.1, "tilt": 0.2})
        self.assertEqual(t["look"], {"pan": 0.0, "tilt": P.camera_pitch})  # what the station commands
        json.dumps(t)

    def test_nothing_known_yet(self):
        t = server.Station(FakeLink(), "test").telemetry(0.0)
        self.assertIsNone(t["rockers"])
        self.assertIsNone(t["wheels"])
        self.assertIsNone(t["head"])
        self.assertTrue(t["control"])
        self.assertEqual(t["others"], [])

    def test_announces_itself_and_reports_other_stations(self):
        link = FakeLink()
        station = server.Station(link, "test")
        station.url = "http://127.0.0.1:8765/"
        link.others = [{"id": "host:1", "world": "test", "url": "http://127.0.0.1:8766/"},
                       {"id": "host:2", "world": "test", "url": None}]
        for k in range(21):  # 1 s of control ticks
            station.tick(k * server.CONTROL_PERIOD)
        self.assertEqual(link.of("announce"), [("announce", station.url)] * 3)  # at 0, 0.5 and 1 s
        self.assertEqual(station.telemetry(1.0)["others"], ["http://127.0.0.1:8766/", "host:2"])

    def test_world_returned_once_after_a_gap(self):
        link = FakeLink()
        station = server.Station(link, "test")
        seen = []
        for online in (True, True, False, False, True, True, False, True):
            link.state = {"stats": {}} if online else {}
            seen.append(station.world_returned())
        self.assertEqual(seen, [False, False, False, False, True, False, False, True])


class Unplugged:
    """A gz-transport Node that is not connected: GzLink's bookkeeping
    without subscriptions that would outlive the test."""

    def subscribe(self, *args):
        return True

    def advertise(self, topic, msg_type):
        return None

    def topic_list(self):
        return []


class GzLinkState(unittest.TestCase):
    """GzLink's bookkeeping, fed by hand."""

    def setUp(self):
        self.link = gz_link.GzLink("nowhere", Unplugged())

    def test_nothing_while_the_simulation_is_gone(self):
        link = self.link
        link._put("pose", {"x": 1.0})
        link._put("led", "blue")
        self.assertEqual(link.snapshot(), {})  # no statistics: no simulation
        self.assertFalse(link.online())
        link._put("stats", {"sim_time": 1.0})
        self.assertEqual(link.snapshot()["led"], "blue")
        self.assertTrue(link.online())
        link._seen["stats"] -= gz_link.SIM_TIMEOUT + 0.1
        self.assertEqual(link.snapshot(), {})
        self.assertFalse(link.online())

    def test_camera_states_and_referee_expire(self):
        link = self.link
        link._put("stats", {})
        link._put("chase", {"mode": "follow"})
        link._put("score", {"points": 3})
        link._put("led", "red")
        for key in ("chase", "score", "led"):
            link._seen[key] -= 5.0
        self.assertEqual(set(link.snapshot()), {"stats", "led"})

    def test_other_stations_from_their_announcements(self):
        link = self.link
        link._on_station(_string(json.dumps({"id": link.id, "world": "nowhere", "url": None})))  # itself
        link._on_station(_string(json.dumps({"id": "elsewhere:7", "world": "nowhere", "url": "http://x/"})))
        self.assertEqual([o["id"] for o in link.other_stations()], ["elsewhere:7"])
        info, seen = link._stations["elsewhere:7"]
        link._stations["elsewhere:7"] = (info, seen - gz_link.STATION_TIMEOUT - 0.1)
        self.assertEqual(link.other_stations(), [])


def _string(text):
    msg = StringMsg()
    msg.data = text
    return msg


class Minimap(unittest.TestCase):
    def test_hillshade_and_places(self):
        path = WORLDS / "urc_equipment_servicing.json"
        sheet = sheets.load(path)
        png = minimap.hillshade_png(sheet, path)
        picture = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
        self.assertLessEqual(max(picture.shape[:2]), minimap.MAX_PIXELS)
        self.assertGreater(picture.std(), 2)  # shaded, not flat
        info = minimap.map_info(sheet)
        names = {p["name"] for p in info["places"]}
        self.assertTrue({"start_gate", "lander", "c2", "rover_start"} <= names, names)
        self.assertEqual(sheets.find("urc_equipment_servicing"), path)
        self.assertIsNone(sheets.find("rover_test"))


class App(AioHTTPTestCase):
    async def get_application(self):
        self.link = FakeLink()
        path = WORLDS / "urc_equipment_servicing.json"
        self.station = server.Station(self.link, "urc_equipment_servicing", sheets.load(path), path)
        return server.make_app(self.station)

    async def test_page_and_assets(self):
        for path, kind in (("/", "text/html"), ("/static/station.js", "javascript"), ("/static/station.css", "text/css")):
            async with self.client.get(path) as response:
                self.assertEqual(response.status, 200, path)
                self.assertIn(kind, response.headers["Content-Type"])
        async with self.client.get("/") as response:
            self.assertIn("/static/station.js", await response.text())

    async def test_info(self):
        async with self.client.get("/api/info") as response:
            info = await response.json()
        self.assertEqual(info["world"], "urc_equipment_servicing")
        self.assertEqual(info["map"]["size"], [256.0, 256.0])
        self.assertEqual(info["rover"]["wheel_dx"], P.wheel_dx)
        self.assertEqual(sorted(info["presets"]), ["1", "2", "3"])
        self.assertEqual(info["hfov"], {"eye": gen_model.EyeParams().hfov, "chase": gen_model.ChaseParams().hfov,
                                        "rgb": P.camera_hfov, "depth": P.camera_hfov})

    async def test_minimap(self):
        async with self.client.get("/minimap.png") as response:
            self.assertEqual(response.status, 200)
            self.assertTrue((await response.read()).startswith(b"\x89PNG"))

    async def test_frames_are_jpegs_and_stop_when_nobody_watches(self):
        for camera in ("eye", "chase", "rgb", "depth"):
            async with self.client.get(f"/frame/{camera}") as response:
                self.assertEqual(response.status, 200, camera)
                self.assertEqual(response.headers["Content-Type"], "image/jpeg")
                picture = cv2.imdecode(np.frombuffer(await response.read(), np.uint8), cv2.IMREAD_COLOR)
            self.assertEqual(picture.shape, (24, 32, 3))
        self.assertEqual(self.link.watching, set())
        async with self.client.get("/frame/thermal") as response:
            self.assertEqual(response.status, 404)

    async def test_mjpeg_stream(self):
        async with self.client.get("/stream/rgb") as response:
            self.assertEqual(response.status, 200)
            self.assertTrue(response.headers["Content-Type"].startswith("multipart/x-mixed-replace"))
            data = b""
            while data.count(b"--frame") < 3:
                data += await response.content.read(4096)
            self.assertEqual(self.link.watching, {"rgb"})
        part = data.split(b"--frame")[1]
        jpeg = part.split(b"\r\n\r\n", 1)[1]
        self.assertTrue(jpeg.startswith(b"\xff\xd8"))
        for _ in range(100):  # the server notices the closed connection
            if not self.link.watching:
                break
            await asyncio.sleep(0.02)
        self.assertEqual(self.link.watching, set())

    async def test_shutdown_ends_streams(self):
        async with self.client.get("/stream/chase") as response:
            await response.content.read(100)
            await self.app.shutdown()  # what Ctrl-C does before aiohttp waits for open requests
            await asyncio.wait_for(response.content.read(), 3.0)  # to the end of the stream
        self.assertEqual(self.link.watching, set())

    async def drive_for(self, ws, seconds, keys):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            await ws.send_json({"t": "input", "keys": keys, "axes": [0, 0]})
            await asyncio.sleep(0.05)

    async def test_drive_led_and_deadman(self):
        async with self.client.ws_connect("/ws") as ws:
            first = await ws.receive_json()
            self.assertEqual(first["t"], "telemetry")
            await self.drive_for(ws, 0.5, ["KeyW", "KeyA"])
            vx, wz = self.link.of("twist")[-1][1:]
            self.assertTrue(0.2 < vx <= drive.PRESETS[2][0], vx)
            self.assertTrue(0.0 < wz <= drive.PRESETS[2][1], wz)
            self.assertIn(("led", "blue"), self.link.sent)  # teleoperation, rule 1.e.ii
            await asyncio.sleep(drive.DEADMAN + 0.2)
            self.assertEqual(self.link.of("twist")[-1], ("twist", 0.0, 0.0))
            deadline = time.monotonic() + 1.0  # skip telemetry queued before the deadman
            deadman = False
            while not deadman and time.monotonic() < deadline:
                msg = await ws.receive_json(timeout=1.0)
                deadman = msg["t"] == "telemetry" and msg["deadman"]
            self.assertTrue(deadman)

    async def test_cameras_from_the_same_socket(self):
        async with self.client.ws_connect("/ws") as ws:
            await ws.send_json({"t": "chase", "yaw": 0.1, "zoom": -0.2})
            await ws.send_json({"t": "chase_mode", "mode": "orbit"})
            await ws.send_json({"t": "chase_mode", "mode": "sideways"})  # ignored
            await ws.send_json({"t": "look", "pan": 0.3, "tilt": 0.1})
            await ws.send_json({"t": "look_center"})
            await ws.send_json({"t": "preset", "n": 3})
            await asyncio.sleep(0.2)
        self.assertEqual(self.link.of("chase"), [("chase", 0.1, 0.0, -0.2)])
        self.assertEqual(self.link.of("chase_mode"), [("chase_mode", "orbit")])
        for kind in ("eye", "head"):  # the head follows the look
            sent = self.link.of(kind)
            self.assertIn((kind, 0.3, P.camera_pitch + 0.1), sent)
            self.assertEqual(sent[-1], (kind, 0.0, P.camera_pitch))
        self.assertEqual(self.station.drive.preset, 3)

    async def test_cameras_come_back_with_the_world(self):
        """When the world stops and starts again (pixi run sim), the page sees
        no simulation in between and the station spawns its cameras again."""
        async with self.client.ws_connect("/ws") as ws:
            self.link.state = {}  # the world stopped
            await asyncio.sleep(server.WATCH_PERIOD + 0.2)
            deadline = time.monotonic() + 1.0
            while (msg := await ws.receive_json(timeout=1.0))["stats"] is not None:
                self.assertLess(time.monotonic(), deadline)
            self.assertIsNone(msg["pose"])
            self.assertEqual(self.link.of("ensure_cameras"), [])
            self.link.state = {"stats": {"sim_time": 0.0, "rtf": 1.0, "paused": False}}
            await asyncio.sleep(server.WATCH_PERIOD + 0.2)
        self.assertEqual(self.link.of("ensure_cameras"), [("ensure_cameras",)])

    async def test_release_to_autonomy(self):
        rover = lambda: [s for s in self.link.sent if s[0] in ("twist", "led", "head")]  # noqa: E731
        async with self.client.ws_connect("/ws") as ws:
            await self.drive_for(ws, 0.3, ["KeyW"])
            await ws.send_json({"t": "control", "on": False})
            await asyncio.sleep(0.1)
            count = len(rover())
            self.assertEqual(rover()[-1], ("twist", 0.0, 0.0))  # handed over stopped
            await self.drive_for(ws, 0.3, ["KeyW"])
            await ws.send_json({"t": "look", "pan": 0.2})
            await asyncio.sleep(0.1)
            self.assertEqual(len(rover()), count)  # cmd_vel, LED and head left alone
            self.assertEqual(self.link.of("eye")[-1], ("eye", 0.2, P.camera_pitch))  # the eye still looks
            await ws.send_json({"t": "control", "on": True})
            await asyncio.sleep(0.1)
            self.assertIn(("led", "blue"), rover()[count:])
            self.assertIn(("head", 0.2, P.camera_pitch), rover()[count:])  # the head catches up

    async def test_bad_messages_are_answered_not_fatal(self):
        async with self.client.ws_connect("/ws") as ws:
            await ws.send_str("{not json")
            await ws.send_json({"t": "preset"})
            await ws.send_str('{"t": "chase", "yaw": 1e309}')  # what JSON.stringify never sends
            errors = []
            while len(errors) < 3:
                msg = await ws.receive()
                self.assertEqual(msg.type, WSMsgType.TEXT)
                data = json.loads(msg.data)
                if data["t"] == "error":
                    errors.append(data)
            await ws.send_json({"t": "preset", "n": 1})
            await asyncio.sleep(0.1)
        self.assertEqual(self.station.drive.preset, 1)
        self.assertEqual(self.link.of("chase"), [])


class NoMap(AioHTTPTestCase):
    async def get_application(self):
        return server.make_app(server.Station(FakeLink(), "rover_test"))

    async def test_no_sheet(self):
        async with self.client.get("/minimap.png") as response:
            self.assertEqual(response.status, 404)
        async with self.client.get("/api/info") as response:
            self.assertIsNone((await response.json())["map"])


def unit(x, y, z):
    n = math.sqrt(x * x + y * y + z * z)
    return x / n, y / n, z / n


class ChaseCameraFlight(unittest.TestCase):
    """The ChaseCamera plugin keeps its model on the commanded sphere around
    a rover that drives in a circle (headless: no Sensors system, so the
    camera itself does not render)."""

    def test_follows_orbits_and_stays_above(self):
        c = gen_model.ChaseParams()
        extra = (f"<include><uri>model://{gen_model.CHASE_MODEL}</uri><name>{gen_model.CHASE_MODEL}</name>"
                 "<pose>-5 0 2 0 0 0</pose></include>")
        node = Node()
        twist_pub = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
        cmd_pub = node.advertise(gen_model.CHASE_CMD_TOPIC, Vector3d)
        mode_pub = node.advertise(gen_model.CHASE_MODE_TOPIC, StringMsg)
        twist = Twist()
        twist.linear.x, twist.angular.z = 0.6, 0.35
        zoom = math.log(1.5)
        # (sim time [s], action): commands go out once (cmd deltas add up); the
        # one at 5 s is not finite and must be dropped.
        plan = [(4.0, ("cmd", 0.8, 0.2, zoom)), (5.0, ("cmd", math.nan, math.inf, -math.inf)),
                (7.0, ("mode", "orbit")), (10.0, ("cmd", 0.0, -3.0, 0.0))]
        samples = {}
        models = {}

        def wait_for(publisher):
            deadline = time.monotonic() + 10
            while not publisher.has_connections() and time.monotonic() < deadline:
                time.sleep(0.01)

        def pre_update(info, ecm):
            if not models:
                world = World(world_entity(ecm))
                models["rover"] = Model(world.model_by_name(ecm, "rover"))
                models["camera"] = Model(world.model_by_name(ecm, gen_model.CHASE_MODEL))
            if info.iterations % 20 == 0:
                twist_pub.publish(twist)
            t = info.iterations / 1000
            for when, action in plan:
                if abs(t - when) < 5e-4:
                    if action[0] == "cmd":
                        wait_for(cmd_pub)
                        msg = Vector3d()
                        msg.x, msg.y, msg.z = action[1:]
                        cmd_pub.publish(msg)
                    else:
                        wait_for(mode_pub)
                        msg = StringMsg()
                        msg.data = action[1]
                        mode_pub.publish(msg)
                    time.sleep(0.05)  # let the plugin's transport thread take it before the next step

        def post_update(info, ecm):
            t = round(info.iterations / 1000, 3)
            if t in (3.9, 6.9, 7.5, 9.9, 11.0):
                samples[t] = {name: _pose(m, ecm) for name, m in models.items()}

        with temp_sdf(world_sdf(extra)) as world:
            fixture = TestFixture(world)
            fixture.on_pre_update(pre_update)
            fixture.on_post_update(post_update)
            fixture.finalize()
            fixture.server().run(True, 11000, False)
        self.assertEqual(sorted(samples), [3.9, 6.9, 7.5, 9.9, 11.0])

        def view(t):
            """(azimuth relative to behind the rover, world azimuth, elevation,
            range, look error [rad]) of the camera at time t."""
            rover, camera = samples[t]["rover"], samples[t]["camera"]
            look = (rover[0], rover[1], rover[2] + c.look_height)
            d = [camera[k] - look[k] for k in range(3)]
            rng = math.sqrt(sum(v * v for v in d))
            azimuth = math.atan2(d[1], d[0])
            relative = math.remainder(azimuth - rover[5] - math.pi, 2 * math.pi)
            # The camera's x axis (roll 0, pitch, yaw) should point at the look-at point.
            _, pitch, yaw = camera[3:]
            axis = (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch))
            to_look = unit(*[-v for v in d])
            look_error = math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(axis, to_look)))))
            return relative, azimuth, math.asin(d[2] / rng), rng, look_error

        # The rover turns at 0.14-0.3 rad/s (the physical drivetrain achieves
        # ~40 % of the commanded 0.35 rad/s on regolith), so the smoothed view
        # trails by about rate x time constant; the tolerances allow for that.
        relative, _, elevation, rng, look_error = view(3.9)
        self.assertLess(abs(relative - c.yaw), 0.15, "follow: behind the rover")
        self.assertAlmostEqual(elevation, c.pitch, delta=0.03)
        self.assertAlmostEqual(rng, c.distance, delta=0.2)
        self.assertLess(look_error, 0.06)

        relative, _, elevation, rng, look_error = view(6.9)
        self.assertLess(abs(relative - (c.yaw + 0.8)), 0.15, "follow: commanded yaw offset")
        self.assertAlmostEqual(elevation, c.pitch + 0.2, delta=0.03)
        self.assertAlmostEqual(rng, c.distance * 1.5, delta=0.3)
        self.assertLess(look_error, 0.06)

        # Orbit: the world azimuth holds while the rover keeps turning.
        _, azimuth_a, _, _, _ = view(7.5)
        _, azimuth_b, _, _, _ = view(9.9)
        turned = math.remainder(samples[9.9]["rover"][5] - samples[7.5]["rover"][5], 2 * math.pi)
        self.assertGreater(abs(turned), 0.25)  # measured 0.33 rad with the physical rover
        self.assertLess(abs(math.remainder(azimuth_b - azimuth_a, 2 * math.pi)), 0.05)

        # Pitch is clamped above the horizon: the camera never dips below the target.
        _, _, elevation, _, _ = view(11.0)
        self.assertAlmostEqual(elevation, c.pitch_limits[0], delta=0.02)
        self.assertGreater(samples[11.0]["camera"][2], samples[11.0]["rover"][2] + c.look_height)


class EyeCameraRide(unittest.TestCase):
    """The rover eye (the ChaseCamera plugin's eye mode) stays at the rover's
    camera pivot while the station drives the rover over a plank and round a
    turn, and looking only turns it there; the camera head follows the look.
    Station and GzLink run as in pixi run drive, against a headless world
    (no Sensors system, so nothing renders)."""

    def test_stays_at_the_pivot_and_turns_with_the_look(self):
        # A 6 cm plank under the left wheels rolls the body (x 1.0-1.4 m).
        extra = ("<model name='plank'><static>true</static><link name='link'><pose>1.2 0.4 0.03 0 0 0</pose>"
                 "<collision name='c'><geometry><box><size>0.4 0.3 0.06</size></box></geometry></collision>"
                 "</link></model>"
                 f"<include><uri>model://{gen_model.EYE_MODEL}</uri><name>{gen_model.EYE_MODEL}</name>"
                 "<pose>0 0 3 0 0 0</pose></include>")
        station = server.Station(gz_link.GzLink("test", Node()), "test")
        # Its callbacks would otherwise keep the node, and its subscriptions, alive for the other tests.
        self.addCleanup(lambda node=station.link.node: [node.unsubscribe(t) for t in node.subscribed_topics()])
        raw_look = Node().advertise(gen_model.EYE_LOOK_TOPIC, Vector3d)  # one advertiser per node and topic
        mount = gz.math7.Pose3d(gz.math7.Vector3d(*P.camera_xyz), gz.math7.Quaterniond())
        # (sim time [s], message to the station); keys W from 0.5 s, W + A (turn left) from 4.5 s.
        looks = {1.0: {"t": "look", "pan": 0.8, "tilt": 0.3}, 4.0: {"t": "look", "pan": -1.5, "tilt": -0.6},
                 7.0: {"t": "look_center"}}
        expected = {3.0: (0.8, P.camera_pitch + 0.3), 6.0: (-0.7, P.camera_pitch - 0.3),
                    8.5: (0.0, P.camera_pitch)}
        handles = {}
        worst = {"offset": 0.0, "roll": 0.0}
        samples = {}

        def pre_update(info, ecm):
            if not handles:
                world = World(world_entity(ecm))
                rover = Model(world.model_by_name(ecm, "rover"))
                handles["base"] = Link(rover.link_by_name(ecm, "base_link"))
                handles["eye"] = Link(Model(world.model_by_name(ecm, gen_model.EYE_MODEL)).canonical_link(ecm))
                handles["joints"] = [Joint(rover.joint_by_name(ecm, j)) for j in (gen_model.PAN_JOINT,
                                                                                 gen_model.TILT_JOINT)]
                for joint in handles["joints"]:
                    joint.enable_position_check(ecm, True)
            if info.iterations % 50:
                return
            t = info.iterations / 1000
            if t in looks:
                station.handle(looks[t], t)
            keys = ["KeyW", "KeyA"] if t >= 4.5 else ["KeyW"] if t >= 0.5 else []
            station.handle({"t": "input", "keys": keys}, t)
            station.tick(t)
            if t in (8.6, 8.7):  # straight to the plugin (the station's next re-send is at 9 s)
                msg = Vector3d()
                msg.x, msg.y = (5.0, -2.0) if t == 8.6 else (math.nan, math.inf)  # past the limits, not finite
                raw_look.publish(msg)
            time.sleep(0.002)  # let transport threads deliver before the next step

        def post_update(info, ecm):
            t = round(info.iterations / 1000, 3)
            base, eye = handles["base"].world_pose(ecm), handles["eye"].world_pose(ecm)
            if t >= 0.1:  # once the plugin has moved it from its spawn pose
                worst["offset"] = max(worst["offset"], (eye.pos() - (base * mount).pos()).length())
            worst["roll"] = max(worst["roll"], abs(base.rot().euler().x()))
            if t in (*expected, 4.5, 8.9):
                rel = (base.rot().inverse() * eye.rot()).euler()  # the eye in the rover's frame
                samples[t] = {"look": (rel.x(), rel.y(), rel.z()), "base": (base.pos().x(), base.pos().y(),
                                                                            base.rot().euler().z()),
                              "joints": [j.position(ecm)[0] for j in handles["joints"]]}

        with temp_sdf(world_sdf(extra)) as world:
            fixture = TestFixture(world)
            fixture.on_pre_update(pre_update)
            fixture.on_post_update(post_update)
            fixture.finalize()
            fixture.server().run(True, 8950, False)
        self.assertEqual(sorted(samples), [3.0, 4.5, 6.0, 8.5, 8.9])
        # At the pivot all along (one 1 ms step behind the rover), and the body really rolled.
        self.assertLess(worst["offset"], 0.01)
        self.assertGreater(worst["roll"], 0.03)
        for t, (pan, tilt) in expected.items():
            roll, pitch, yaw = samples[t]["look"]
            self.assertAlmostEqual(roll, 0.0, delta=0.01, msg=t)  # body-fixed: no roll of its own
            self.assertAlmostEqual(pitch, tilt, delta=0.01, msg=t)
            self.assertAlmostEqual(yaw, pan, delta=0.01, msg=t)
            for got, want in zip(samples[t]["joints"], (pan, tilt)):
                self.assertAlmostEqual(got, want, delta=0.02, msg=f"head at {t} s")
        # Out of range: the plugin clamps to the head's limits (and drops the
        # look that is not finite); the head was not commanded.
        _, pitch, yaw = samples[8.9]["look"]
        self.assertAlmostEqual(yaw, P.camera_pan_limit, delta=0.01)
        self.assertAlmostEqual(pitch, P.camera_tilt_limits[0], delta=0.01)
        for got, want in zip(samples[8.9]["joints"], (0.0, P.camera_pitch)):
            self.assertAlmostEqual(got, want, delta=0.02)
        # The rover did drive and turn.
        x0, y0, yaw0 = samples[4.5]["base"]
        x1, y1, yaw1 = samples[8.9]["base"]
        self.assertGreater(x0, 1.5)
        self.assertGreater(abs(math.remainder(yaw1 - yaw0, 2 * math.pi)), 1.0)


class StationProcess(unittest.TestCase):
    """pixi run drive as a process, in a private transport partition: it
    stops the world it started however it is stopped (DiffDrive would keep
    the last twist forever), one world has one station, and a world that
    cannot start ends the wait at once."""

    def station(self, *args):
        """A running station process; its output collects in .output."""
        env = dict(os.environ, GZ_PARTITION=f"station_test_{os.getpid()}_{self._testMethodName}",
                   GZ_IP="127.0.0.1", PATH=os.pathsep.join((os.path.join(sys.prefix, "bin"), os.environ["PATH"])))
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        process = subprocess.Popen([sys.executable, str(SIM_DIR / "station" / "__main__.py"), *args, "--no-browser",
                                    "--port", str(port)], env=env, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        process.output = []

        def read():  # until the station and the world it started (which shares the pipe) are gone
            process.output.extend(process.stdout)
            process.stdout.close()

        reader = threading.Thread(target=read, daemon=True)
        reader.start()
        self.addCleanup(reader.join, 10)
        self.addCleanup(lambda: process.poll() is None and process.kill())
        return process

    def wait_for(self, process, text, timeout):
        deadline = time.monotonic() + timeout
        while text not in "".join(process.output):
            self.assertLess(time.monotonic(), deadline, f"no {text!r} in {''.join(process.output)!r}")
            self.assertIsNone(process.poll(), "".join(process.output))
            time.sleep(0.1)

    def world(self):
        """rover_test without the Sensors system (nothing renders), under a
        name only this test's gz sim has on its command line."""
        text = SENSORS.sub("", (WORLDS / "rover_test.sdf").read_text())
        context = temp_sdf(text)
        path = context.__enter__()
        self.addCleanup(context.__exit__, None, None, None)
        self.addCleanup(lambda: subprocess.run(["pkill", "-KILL", "-f", path]))  # in case the test fails
        return path

    def assert_world_stops(self, path, timeout=15.0):
        deadline = time.monotonic() + timeout
        while subprocess.run(["pgrep", "-f", path], capture_output=True).returncode == 0:
            self.assertLess(time.monotonic(), deadline, f"gz sim {path} still runs")
            time.sleep(0.2)

    def test_closing_the_terminal_stops_the_world_and_one_station_per_world(self):
        path = self.world()
        first = self.station(path)
        self.wait_for(first, "driver station on", 90)
        self.assertIn("eye_camera: spawned", "".join(first.output))
        self.assertIn("chase_camera: spawned", "".join(first.output))
        self.assertEqual(subprocess.run(["pgrep", "-f", path], capture_output=True).returncode, 0)
        second = self.station()  # attaches to the running world
        second.wait(30)
        self.assertNotEqual(second.returncode, 0)
        self.assertIn("another driver station is attached to rover_test", "".join(second.output))
        first.send_signal(signal.SIGHUP)
        self.assertEqual(first.wait(30), 0, "".join(first.output))  # stopped by the station, not by the signal
        self.assert_world_stops(path)

    def test_stopped_while_the_world_loads(self):
        path = self.world()
        process = self.station(path)
        self.wait_for(process, "headless", 30)
        process.send_signal(signal.SIGTERM)
        self.assertEqual(process.wait(30), 0, "".join(process.output))
        self.assert_world_stops(path)

    def test_a_world_that_does_not_load(self):
        text = (WORLDS / "rover_test.sdf").read_text()
        text, count = re.subn(r"<box><size>[^<]*</size>", "<box><size>1 2</size>", text, count=1)  # wants 3 values
        self.assertEqual(count, 1)
        with temp_sdf(text) as path:
            start = time.monotonic()
            process = self.station(path)
            process.wait(60)
        self.assertLess(time.monotonic() - start, 30)
        self.assertNotEqual(process.returncode, 0)
        self.assertIn("before rover_test came up", "".join(process.output))


def _pose(model, ecm):
    pose = Link(model.canonical_link(ecm)).world_pose(ecm)
    p, r = pose.pos(), pose.rot().euler()
    return (p.x(), p.y(), p.z(), r.x(), r.y(), r.z())


if __name__ == "__main__":
    unittest.main()
