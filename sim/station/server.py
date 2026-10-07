"""The driver station's web app: one page that drives the rover, moves the
cameras and shows the telemetry, so a single keyboard focus does it all.

The page sends its inputs over a WebSocket (/ws) and gets telemetry back at
10 Hz; cameras are MJPEG streams (/stream/<camera>) or single JPEGs
(/frame/<camera>). Station turns inputs into gz commands at 20 Hz. Besides
the driving views the page has two inspection views that only a simulation
has (realism design 8.3): Fly, the fly camera's picture, and Map, the
world's orthophoto (/map.jpg, tools/render_map.py) or its hillshade
(/minimap.png).
"""
import asyncio
import json
import math
import sys
import time
from pathlib import Path

from aiohttp import WSMsgType, web

import gen_model
import viewers
from urc import rules
from urc import sheet as sheets

from . import drive, minimap

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import render_map  # noqa: E402  (the orthophoto maps and whether they are current)

STATIC_DIR = Path(__file__).resolve().parent / "static"
CONTROL_PERIOD = 1 / 20  # [s] twist commands
TELEMETRY_PERIOD = 1 / 10  # [s]
LED_PERIOD = 0.5  # [s] teleoperation LED, re-sent so a late referee sees it
LOOK_PERIOD = 1.0  # [s] the look, re-sent for late subscribers (a camera spawned later)
ANNOUNCE_PERIOD = 0.5  # [s] tell other driver stations about this one, and look for them
WATCH_PERIOD = 1.0  # [s] check whether the world came back (its cameras then need spawning again)
FRAME_TIMEOUT = 5.0  # [s] /frame waits this long for a picture
CHASE_MODES = ("follow", "orbit", "reset")
FLY = viewers.FlyParams()
VIEWS = ("eye", "chase", "rgb", "depth", "fly", "map")  # the page's main views
INSPECTION = ("fly", "map")  # sim-only views; both want the fly camera (Map shows where it is)
FLY_SPAWN_RETRY = 10.0  # [s] before the station tries again to spawn a fly camera that does not report
DEFAULT_SPAN = 100.0  # [m] map width on screen for a goto that names none (A)


def finite(value):
    """float(value), refusing NaN and the infinities (json.loads accepts NaN,
    Infinity and 1e309, and one NaN would stick in a camera's view)."""
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{value!r} is not a finite number")
    return number


def command(msg):
    """The drive.Command of an input or fly message: held key codes and two stick axes."""
    keys = frozenset(str(k) for k in msg.get("keys", ()))
    axes = tuple(finite(a) for a in msg.get("axes", (0.0, 0.0)))[:2]
    return drive.Command(keys, axes if len(axes) == 2 else (0.0, 0.0))


class Station:
    """What the page controls. link: GzLink, or a fake with the same
    methods (twist, led, chase, chase_mode, eye, head, fly, fly_speed,
    fly_look, fly_goto, fly_mode, announce, other_stations, online,
    ensure_cameras, ensure_fly, snapshot, feeds). maps_dir: where the
    orthophoto map lies (default: next to the world, render_map.map_paths)."""

    def __init__(self, link, world, sheet=None, sheet_path=None, control=True, maps_dir=None):
        self.link = link
        self.world = world
        self.sheet = sheet
        self.sheet_path = sheet_path
        self.maps_dir = maps_dir
        self.url = None  # the page's address, once served
        self.drive = drive.Drive()
        self.look = drive.Look()
        self.fly = drive.Fly()
        self.view = "eye"  # the page's main view (VIEWS)
        self.fly_note = None  # why the fly camera did not spawn, for the page
        # Teleoperation; False leaves cmd_vel, the LED and the camera head to the autonomy stack.
        self.control = control
        self.others = []  # announcements of other driver stations in this partition
        self._led_sent = -math.inf
        self._look_sent = -math.inf
        self._announced = -math.inf
        self._fly_speed_checked = -math.inf
        self._fly_spawned = -math.inf
        self._world_gone = False
        self._minimap = None
        self._terrain = None

    def handle(self, msg, now):
        """Apply one message from the page (a dict, see static/station.js)."""
        kind = msg.get("t")
        if kind == "input":
            self.drive.input(command(msg), now)
        elif kind == "view" and msg.get("view") in VIEWS:
            self.view = msg["view"]
        elif kind == "fly":
            self.fly.input(command(msg), now)
        elif kind == "fly_look":
            self.link.fly_look(finite(msg.get("yaw", 0)), finite(msg.get("pitch", 0)))
        elif kind == "fly_speed":
            self.link.fly_speed(self.fly.speed(finite(msg["factor"])))
        elif kind == "fly_mode" and msg.get("mode") in viewers.FLY_MODES:
            self.link.fly_mode(msg["mode"])
        elif kind == "fly_goto":
            view = self.goto(msg)
            if view is not None:
                self.link.fly_goto(view)
        elif kind == "preset":
            self.drive.set_preset(int(msg["n"]))
        elif kind == "chase":
            self.link.chase(finite(msg.get("yaw", 0)), finite(msg.get("pitch", 0)), finite(msg.get("zoom", 0)))
        elif kind == "chase_mode" and msg.get("mode") in CHASE_MODES:
            self.link.chase_mode(msg["mode"])
        elif kind == "look":
            self.look.nudge(finite(msg.get("pan", 0)), finite(msg.get("tilt", 0)))
            self._send_look(now)
        elif kind == "look_center":
            self.look.center()
            self._send_look(now)
        elif kind == "control":
            self.set_control(bool(msg["on"]))

    def set_control(self, on):
        if on == self.control:
            return
        self.control = on
        self.drive = drive.Drive(self.drive.preset)  # a fresh start, at the operator's speed
        if not on:
            self.link.twist(0.0, 0.0)  # hand the rover over stopped
        self._led_sent = self._look_sent = -math.inf

    def tick(self, now):
        """Every CONTROL_PERIOD: the fly camera's command while the page
        flies it (released too: the operator may watch autonomy from the
        air), the look, the fly speed and the announcement now and then;
        while in control also the twist and the LED."""
        twist = self.drive.step(now)
        fly = self.fly.step(now)
        if fly is not None:
            self.link.fly(*fly)
        if now - self._look_sent >= LOOK_PERIOD:
            self._send_look(now)
        if now - self._fly_speed_checked >= LOOK_PERIOD:  # a fly camera spawned again starts at 1
            state = self.link.snapshot().get("fly")
            if state is not None and abs(state["speed"] - self.fly.scale) > 2e-3:  # the state has 3 decimals
                self.link.fly_speed(self.fly.scale)
            self._fly_speed_checked = now
        if now - self._announced >= ANNOUNCE_PERIOD:
            self.link.announce(self.url)
            self.others = self.link.other_stations()
            self._announced = now
        if not self.control:
            return
        self.link.twist(*twist)
        if now - self._led_sent >= LED_PERIOD:
            self.link.led(rules.LED_TELEOP)
            self._led_sent = now

    def world_returned(self):
        """True once each time the world comes back after it stopped (pixi
        run sim started it again): its cameras are gone."""
        online = self.link.online()
        returned = online and self._world_gone
        self._world_gone = not online
        if returned:
            self._fly_spawned = -math.inf  # spawn it again as soon as a view wants it
        return returned

    # --- Fly camera ----------------------------------------------------------------

    def fly_spawn_due(self, now):
        """Whether to spawn the fly camera now (spawn_fly, off the event loop):
        a Fly or Map view wants it, the world runs without it, and the last
        try was FLY_SPAWN_RETRY ago."""
        if self.view not in INSPECTION or now - self._fly_spawned < FLY_SPAWN_RETRY or not self.link.online():
            return False
        if "fly" in self.link.snapshot():
            return False
        self._fly_spawned = now
        return True

    def spawn_fly(self):
        """Spawn the fly camera behind the rover (or over the terrain centre
        while the rover's pose is unknown); its console line. Blocks."""
        pose = self.link.snapshot().get("pose")
        start = drive.rover_view(pose, self.ground) if pose else drive.top_view(0.0, 0.0, DEFAULT_SPAN, self.ground)
        line = self.link.ensure_fly(start)
        self.fly_note = None if line.endswith(("spawned", "already in the world")) else line
        return line

    def goto(self, msg):
        """The drive.View that a fly_goto message asks for: kind "rover"
        (behind it), "point" or "top" (x, y and the map's span on screen [m]),
        or "pixel" (u, v of the fly picture, 0-1 from its top left); None for
        a pixel in the sky. ValueError while what it needs is not known."""
        s = self.link.snapshot()
        kind = msg.get("kind")
        if kind == "rover":
            if "pose" not in s:
                raise ValueError("the rover's pose is not known yet")
            return drive.rover_view(s["pose"], self.ground)
        if kind in ("point", "top"):
            x, y, span = finite(msg["x"]), finite(msg["y"]), finite(msg.get("span", DEFAULT_SPAN))
            if span <= 0:
                raise ValueError(f"span {span} is not positive")
            if kind == "top":
                return drive.top_view(x, y, span, self.ground)
            yaw = s["fly"]["yaw"] if "fly" in s else s["pose"]["yaw"] if "pose" in s else math.pi / 2
            return drive.point_view(x, y, yaw, span, self.ground)
        if kind == "pixel":
            if "fly" not in s:
                raise ValueError("the fly camera is not in the world yet")
            u, v = finite(msg["u"]), finite(msg["v"])
            if not (0 <= u <= 1 and 0 <= v <= 1):
                raise ValueError(f"picture point {u}, {v} is outside 0-1")
            width, height = FLY.size
            return drive.pixel_view(drive.View.of(s["fly"]), u, v, FLY.hfov, width / height, self.ground,
                                    s["fly"]["ortho"])
        raise ValueError(f"unknown goto {kind!r} (rover, point, top, pixel)")

    def terrain(self):
        """The world's terrain from its sheet (a Heightfield, read once), or None."""
        if self._terrain is None and self.sheet is not None:
            self._terrain = sheets.terrain(self.sheet, self.sheet_path)
        return self._terrain

    def ground(self, x, y):
        """Ground height at world (x, y), vectorised: the sheet's heightmap,
        or flat at z = 0 (as the fly camera takes a world without one)."""
        hf = self.terrain()
        return drive.flat_ground(x, y) if hf is None else hf.height(x, y)

    def shutdown(self):
        if self.control:
            self.link.twist(0.0, 0.0)

    def _send_look(self, now):
        """The rover eye always (it is the operator's own view); the head,
        which belongs to the rover, only while in control."""
        self.link.eye(self.look.pan, self.look.tilt)
        if self.control:
            self.link.head(self.look.pan, self.look.tilt)
        self._look_sent = now

    def telemetry(self, now):
        s = self.link.snapshot()
        joints = s.get("joints", {})
        rockers = None
        if "rocker_left_joint" in joints and "rocker_right_joint" in joints:
            left, right = joints["rocker_left_joint"][0], joints["rocker_right_joint"][0]
            # The differential keeps left + right at zero (plugins/rocker_differential.cpp).
            rockers = {"left": left, "right": right, "error": left + right}
        wheels = {w: joints[f"wheel_{w}_joint"][1] * drive.ROVER.wheel_radius
                  for w in ("fl", "fr", "rl", "rr") if f"wheel_{w}_joint" in joints} or None
        head = None
        if gen_model.PAN_JOINT in joints and gen_model.TILT_JOINT in joints:
            head = {"pan": joints[gen_model.PAN_JOINT][0], "tilt": joints[gen_model.TILT_JOINT][0]}
        return {
            "t": "telemetry", "stats": s.get("stats"), "pose": s.get("pose"),
            "rockers": rockers, "wheels": wheels, "head": head, "gnss": s.get("gnss"), "led": s.get("led"),
            "score": s.get("score"), "radio": s.get("radio"), "chase": s.get("chase"),
            "look": {"pan": self.look.pan, "tilt": self.look.tilt},
            "control": self.control, "deadman": self.drive.deadman, "preset": self.drive.preset,
            "others": [o["url"] or o["id"] for o in self.others],
            "cmd": list(self.drive.twist), "cameras": {name: feed.fps(now) for name, feed in self.link.feeds.items()},
            # The fly camera's state (null until it is spawned) and the
            # physical drivetrain's (design spec 9.3; null for a DiffDrive rover).
            "fly": s.get("fly"), "fly_note": self.fly_note, "drivetrain": s.get("drivetrain"),
        }

    def info(self):
        """Static facts for the page: map (and whether its orthophoto is
        current), rover geometry for the side view, speeds, the drivetrain's
        current limit, the fly camera, and each camera's horizontal field of
        view (dragging turns the view by the angle under the pointer). Reads
        files: call it off the event loop."""
        r = drive.ROVER
        info = {
            "world": self.world,
            "map": minimap.map_info(self.sheet),
            "rover": {"pivot_z": r.pivot_z, "wheel_dx": r.wheel_dx, "wheel_dz": r.wheel_dz,
                      "wheel_radius": r.wheel_radius, "chassis_size": r.chassis_size, "chassis_z": r.chassis_z,
                      "rocker_limit": r.rocker_limit},
            "hfov": {"eye": gen_model.EyeParams().hfov, "chase": gen_model.ChaseParams().hfov,
                     "rgb": r.camera_hfov, "depth": r.camera_hfov, "fly": FLY.hfov},
            "fly": {"aspect": FLY.size[0] / FLY.size[1], "fast": FLY.fast, "speed_scales": FLY.speed_scales},
            "drivetrain": {"current_limit": r.drive.current_limit},
            "presets": drive.PRESETS, "max": [drive.MAX_LINEAR, drive.MAX_ANGULAR], "deadman": drive.DEADMAN,
        }
        if info["map"] is not None:
            info["map"]["photo"] = self.photo()
        return info

    def world_path(self):
        """The world's .sdf beside its sheet, or None."""
        path = self.sheet_path and Path(self.sheet_path).with_suffix(".sdf")
        return path if path and path.is_file() else None

    def photo(self):
        """The world's orthophoto map (tools/render_map.py): {"status":
        current, stale or missing, "changed": the first inputs that changed
        since it was rendered, "url"}; None for a world without a file."""
        world_path = self.world_path()
        if world_path is None:
            return None
        status, changed = render_map.status(world_path, self.maps_dir)
        image = render_map.map_paths(world_path, self.maps_dir)[0]
        url = None if status == "missing" else f"/map.jpg?v={image.stat().st_mtime_ns}"
        return {"status": status, "changed": changed[:3], "url": url}

    def photo_line(self):
        """One console line on the orthophoto map; reads files."""
        photo = self.photo()
        if photo is None:
            return "map: no photo map (the world has no sheet or no file)"
        if photo["status"] == "missing":
            return f"map: no photo map yet, the page shows the relief (pixi run sim-maps {self.world} renders it)"
        if photo["status"] == "stale":
            return (f"map: the photo map is out of date ({', '.join(photo['changed'])}), the page says so "
                    f"(pixi run sim-maps {self.world} renders it again)")
        return "map: photo map current"

    def minimap_png(self):
        if self._minimap is None and self.sheet is not None:
            self._minimap = minimap.hillshade_png(self.terrain())
        return self._minimap


STATION = web.AppKey("station", Station)
CLIENTS = web.AppKey("clients", set)
LOOPS = web.AppKey("loops", list)
STOPPING = web.AppKey("stopping", asyncio.Event)  # set on shutdown: ends the endless MJPEG streams


def make_app(station):
    app = web.Application()
    app[STATION] = station
    app[CLIENTS] = set()
    app.router.add_get("/", _index)
    app.router.add_get("/api/info", _info)
    app.router.add_get("/minimap.png", _minimap)
    app.router.add_get("/map.jpg", _photo_map)
    app.router.add_get("/stream/{camera}", _stream)
    app.router.add_get("/frame/{camera}", _frame)
    app.router.add_get("/ws", _websocket)
    app.router.add_static("/static", STATIC_DIR)
    app.on_response_prepare.append(_revalidate)
    app.on_startup.append(_start_loops)
    # On shutdown, before aiohttp waits for open requests (streams, sockets) to end.
    app.on_shutdown.append(_stop_loops)
    return app


async def _revalidate(request, response):
    """The page's files change while the station is in use: browsers must ask."""
    if request.path == "/" or request.path.startswith("/static/"):
        response.headers["Cache-Control"] = "no-cache"


async def _index(request):
    return web.FileResponse(STATIC_DIR / "index.html")


async def _info(request):
    # Hashing the map's inputs takes ~0.5 s in the biggest world: not on the event loop.
    info = await asyncio.get_running_loop().run_in_executor(None, request.app[STATION].info)
    return web.json_response(info)


async def _minimap(request):
    station = request.app[STATION]
    if station.sheet is None:
        raise web.HTTPNotFound(text=f"world {station.world} has no mission sheet")
    png = await asyncio.get_running_loop().run_in_executor(None, station.minimap_png)
    return web.Response(body=png, content_type="image/png", headers={"Cache-Control": "max-age=3600"})


async def _photo_map(request):
    """The orthophoto (its URL in /api/info names its version, so it may be cached)."""
    station = request.app[STATION]
    world_path = station.world_path()
    path = world_path and render_map.map_paths(world_path, station.maps_dir)[0]
    if not path or not path.is_file():
        raise web.HTTPNotFound(text=f"world {station.world} has no photo map: pixi run sim-maps {station.world}")
    return web.FileResponse(path, headers={"Cache-Control": "max-age=86400"})


def _feed(request):
    feed = request.app[STATION].link.feeds.get(request.match_info["camera"])
    if feed is None:
        raise web.HTTPNotFound(text=f"cameras: {', '.join(request.app[STATION].link.feeds)}")
    return feed


async def _stream(request):
    """MJPEG: one JPEG per new frame for as long as the client stays."""
    feed = _feed(request)
    response = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame",
                                           "Cache-Control": "no-cache"})
    await response.prepare(request)
    loop = asyncio.get_running_loop()
    feed.watch()
    sent = 0
    try:
        stopping = request.app[STOPPING]
        while request.transport is not None and not request.transport.is_closing() and not stopping.is_set():
            await feed.next_frame(sent, timeout=1.0)
            seq, jpeg = await loop.run_in_executor(None, feed.jpeg)
            if jpeg is None or seq == sent:
                continue
            await response.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n%s\r\n"
                                 % (len(jpeg), jpeg))
            sent = seq
    except ConnectionResetError:
        pass
    finally:
        feed.unwatch()
    return response


async def _frame(request):
    """The next frame as one JPEG (for scripts and tests)."""
    feed = _feed(request)
    feed.watch()
    try:
        start = feed.seq
        deadline = time.monotonic() + FRAME_TIMEOUT
        while feed.seq == start and time.monotonic() < deadline:
            await feed.next_frame(start, timeout=deadline - time.monotonic())
        if feed.seq == start:
            raise web.HTTPServiceUnavailable(text=f"no picture from {feed.topic} in {FRAME_TIMEOUT:.0f} s")
        _, jpeg = await asyncio.get_running_loop().run_in_executor(None, feed.jpeg)
    finally:
        feed.unwatch()
    return web.Response(body=jpeg, content_type="image/jpeg", headers={"Cache-Control": "no-cache"})


async def _websocket(request):
    station = request.app[STATION]
    ws = web.WebSocketResponse(heartbeat=5.0)
    await ws.prepare(request)
    request.app[CLIENTS].add(ws)
    await ws.send_str(json.dumps(station.telemetry(time.monotonic())))
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                station.handle(json.loads(msg.data), time.monotonic())
            except (ValueError, KeyError, TypeError) as e:
                await ws.send_str(json.dumps({"t": "error", "text": f"bad message {msg.data[:80]!r}: {e}"}))
    finally:
        request.app[CLIENTS].discard(ws)
    return ws


async def _every(period, action):
    next_time = time.monotonic()
    while True:
        await action(time.monotonic())
        next_time += period
        await asyncio.sleep(max(0.0, next_time - time.monotonic()))


async def _start_loops(app):
    station = app[STATION]
    app[STOPPING] = asyncio.Event()

    async def spawn_fly():
        print(await asyncio.get_running_loop().run_in_executor(None, station.spawn_fly), flush=True)

    async def control(now):
        station.tick(now)
        if station.fly_spawn_due(now):
            loops = app[LOOPS]  # changed in place: a running app's keys are frozen
            loops[:] = [task for task in loops if not task.done()] + [asyncio.create_task(spawn_fly())]

    async def cameras(now):
        if station.world_returned():
            lines = await asyncio.get_running_loop().run_in_executor(None, station.link.ensure_cameras)
            print(f"{station.world} is running again: {'; '.join(lines)}", flush=True)

    async def telemetry(now):
        if app[CLIENTS]:
            text = json.dumps(station.telemetry(now))
            for ws in list(app[CLIENTS]):
                try:
                    await ws.send_str(text)
                except ConnectionResetError:
                    app[CLIENTS].discard(ws)

    app[LOOPS] = [asyncio.create_task(_every(CONTROL_PERIOD, control)),
                  asyncio.create_task(_every(TELEMETRY_PERIOD, telemetry)),
                  asyncio.create_task(_every(WATCH_PERIOD, cameras))]


async def _stop_loops(app):
    app[STOPPING].set()
    for task in app[LOOPS]:
        task.cancel()
    await asyncio.gather(*app[LOOPS], return_exceptions=True)
    for ws in list(app[CLIENTS]):
        await ws.close()
    app[STATION].shutdown()
