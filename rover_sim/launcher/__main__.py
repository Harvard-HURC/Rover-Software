"""Simulation launcher: pixi run launcher [--port N] [--no-browser].

One page with a tile for every world in sim/worlds. A click starts that world
with its driver station (sim/station, the world headless) and shows the
station inside the page; Exit, or closing the page, stops both. One
simulation at a time, in a GZ_PARTITION of its own (on GZ_IP 127.0.0.1 unless
set, as the tests run), so it never meets a world started from a terminal or
by the tests.
"""
import argparse
import asyncio
import contextlib
import errno
import functools
import io
import json
import os
import re
import signal
import socket
import sys
import webbrowser
from collections import deque
from pathlib import Path

from aiohttp import WSMsgType, web
from PIL import Image

HERE = Path(__file__).resolve().parent
SIM_DIR = HERE.parent
WORLDS = SIM_DIR / "worlds"
STATION = SIM_DIR / "station" / "__main__.py"
PORT = 8790
PORT_TRIES = 10
GRACE = 5.0  # [s] with no page open before the simulation stops (a reload reconnects sooner)
STOP_TIMEOUT = 25.0  # [s] for the station to stop its world (it kills gz after 10 s) before it is killed
THUMB_M = 600.0  # [m] the square around the rover start that a tile shows (at most the whole map)
THUMB_PX = 480
LOG_LINES = 60  # the station's last lines the page shows while a world starts
READY = re.compile(r"driver station on (\S+)")
COLOUR = re.compile(r"\x1b\[[0-9;]*m")  # gz colours its warnings for a terminal
STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def _sheet(world):
    path = WORLDS / f"{world}.json"
    return json.loads(path.read_text()) if path.is_file() else {}


def _version(path):
    return path.stat().st_mtime_ns // 1_000_000 if path.is_file() else None  # ms: exact in JavaScript


def _minutes(limit):
    if limit is None:
        return None
    low, high = (limit, limit) if isinstance(limit, (int, float)) else limit
    return f"{low / 60:.0f} min" if low == high else f"{low / 60:.0f}–{high / 60:.0f} min"


def worlds():
    """The tiles: the URC missions in rule order, then the courses, then the other worlds."""
    tiles = []
    for sdf in WORLDS.glob("*.sdf"):
        if sdf.stem.startswith("tmp"):  # a test's world, there while it runs
            continue
        sheet = _sheet(sdf.stem)
        if "rules" in sheet:
            order, subtitle = (0, sheet["rules"]), " · ".join(
                filter(None, (sheet["rules"], _minutes(sheet.get("time_limit_s")))))
        elif sheet:
            order, subtitle = (1, ""), "Test course"
        else:
            order, subtitle = (2, ""), "Test world"
        tiles.append((order, {
            "id": sdf.stem,
            "title": sheet.get("mission") or sdf.stem.replace("_", " ").capitalize(),
            "subtitle": subtitle,
            "thumb": _version(WORLDS / f"{sdf.stem}_map.json"),  # in the thumbnail's URL: a new map, a new URL
        }))
    return [tile for _, tile in sorted(tiles, key=lambda pair: (pair[0], pair[1]["id"]))]


@functools.lru_cache(maxsize=32)
def thumbnail(world, version=None):
    """A JPEG of the world's orthophoto map (tools/render_map.py) around the rover start."""
    meta = json.loads((WORLDS / f"{world}_map.json").read_text())
    start = _sheet(world).get("rover_start", {"x": 0.0, "y": 0.0})
    image = Image.open(WORLDS / meta["image"])
    image.draft("RGB", (image.width // 4, image.height // 4))  # decodes the 4096 px JPEG at 1/4 scale
    per_px = meta["size_m"] / image.width  # the map is centred on the world origin, row 0 north
    half = min(THUMB_M, meta["size_m"]) / 2 / per_px  # the whole map of a small world
    cx = min(max((start["x"] + meta["size_m"] / 2) / per_px, half), image.width - half)
    cy = min(max((meta["size_m"] / 2 - start["y"]) / per_px, half), image.height - half)
    tile = image.convert("RGB").crop((round(cx - half), round(cy - half), round(cx + half), round(cy + half)))
    out = io.BytesIO()
    tile.resize((THUMB_PX, THUMB_PX), Image.LANCZOS).save(out, "JPEG", quality=82)
    return out.getvalue()


def free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class Launcher:
    """The one simulation: a driver station that started its world."""

    def __init__(self):
        self.partition = f"rover_launcher_{os.getpid()}"
        self.process = None
        self.reader = None
        self.world = None
        self.url = None
        self.status = "idle"  # idle, starting, running, stopping, failed
        self.log = deque(maxlen=LOG_LINES)
        self.clients = set()
        self.lock = asyncio.Lock()
        self.dirty = asyncio.Event()
        self.idle_timer = None
        self.tasks = set()

    def state(self):
        return {"status": self.status, "world": self.world, "url": self.url, "log": list(self.log)}

    def changed(self):
        self.dirty.set()

    async def broadcast(self):
        """The state to every page, at most ~7 times a second (gz can print a lot)."""
        while True:
            await self.dirty.wait()
            self.dirty.clear()
            message = json.dumps(self.state())
            for ws in list(self.clients):
                with contextlib.suppress(ConnectionError, RuntimeError):
                    await ws.send_str(message)
            await asyncio.sleep(0.15)

    def spawn(self, coroutine):
        task = asyncio.create_task(coroutine)
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)

    async def launch(self, world):
        async with self.lock:
            if self.process is not None and self.world == world:
                return
            await self._stop()
            self.world, self.url, self.status = world, None, "starting"
            self.log.clear()
            self.changed()
            print(f"starting {world} (GZ_PARTITION={self.partition})", flush=True)
            self.process = await asyncio.create_subprocess_exec(
                sys.executable, "-u", str(STATION), str(WORLDS / f"{world}.sdf"),
                "--no-browser", "--port", str(free_port()),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT, limit=1 << 20,
                env={"GZ_IP": "127.0.0.1", **os.environ, "GZ_PARTITION": self.partition, "PYTHONUNBUFFERED": "1"},
                start_new_session=True)  # the terminal's Ctrl-C reaches the launcher, which stops it
            self.reader = asyncio.create_task(self._read(self.process))

    async def _read(self, process):
        with contextlib.suppress(ValueError):  # a line over the limit ends the log, not the world
            async for raw in process.stdout:
                line = COLOUR.sub("", raw.decode(errors="replace").rstrip())
                print(f"  | {line}", flush=True)
                self.log.append(line)
                ready = READY.search(line)
                if ready and process is self.process and self.status == "starting":
                    self.url, self.status = ready.group(1), "running"
                self.changed()
        code = await process.wait()
        if process is self.process and self.status in ("starting", "running"):
            self.process, self.status = None, "failed"
            self.log.append(f"the simulation exited (code {code})")
            self.changed()

    async def stop(self):
        async with self.lock:
            await self._stop()

    async def _stop(self):
        if self.idle_timer is not None:
            self.idle_timer.cancel()
            self.idle_timer = None
        process = self.process
        if process is not None:
            self.status = "stopping"
            self.changed()
            print(f"stopping {self.world}", flush=True)
            if process.returncode is None:
                with contextlib.suppress(ProcessLookupError):
                    process.send_signal(signal.SIGINT)  # the station stops its world, then exits
                try:
                    await asyncio.wait_for(process.wait(), STOP_TIMEOUT)
                except TimeoutError:
                    with contextlib.suppress(ProcessLookupError):
                        os.killpg(process.pid, signal.SIGKILL)
                    await process.wait()
            await self.reader
            print("stopped", flush=True)
        self.process = self.reader = self.world = self.url = None
        self.status = "idle"
        self.changed()

    def joined(self, ws):
        self.clients.add(ws)
        if self.idle_timer is not None:
            self.idle_timer.cancel()
            self.idle_timer = None

    def left(self, ws):
        """The last page closed: stop the simulation unless one comes back within GRACE."""
        self.clients.discard(ws)
        if not self.clients and self.process is not None and self.idle_timer is None:
            self.idle_timer = asyncio.get_running_loop().call_later(GRACE, lambda: self.spawn(self.stop()))


def make_app(launcher):
    names = lambda: {tile["id"] for tile in worlds()}  # noqa: E731  (worlds can be regenerated meanwhile)

    async def index(request):
        return web.FileResponse(HERE / "index.html", headers={"Cache-Control": "no-cache"})

    async def world_list(request):
        return web.json_response(worlds())

    async def thumb(request):
        world = request.match_info["world"]
        if world not in names() or not (WORLDS / f"{world}_map.json").is_file():
            raise web.HTTPNotFound()
        body = await asyncio.get_running_loop().run_in_executor(
            None, thumbnail, world, _version(WORLDS / f"{world}_map.json"))
        return web.Response(body=body, content_type="image/jpeg", headers={"Cache-Control": "max-age=31536000, immutable"})

    async def launch(request):
        world = (await request.json()).get("world")
        if world not in names():
            raise web.HTTPNotFound(text=f"no world {world!r}")
        launcher.spawn(launcher.launch(world))
        return web.json_response({"ok": True}, status=202)

    async def stop(request):
        launcher.spawn(launcher.stop())
        return web.json_response({"ok": True}, status=202)

    async def socket_(request):
        ws = web.WebSocketResponse(heartbeat=2.0)
        await ws.prepare(request)
        launcher.joined(ws)
        await ws.send_str(json.dumps(launcher.state()))
        try:
            async for message in ws:
                if message.type == WSMsgType.ERROR:
                    break
        finally:
            launcher.left(ws)
        return ws

    async def broadcasting(app):
        task = asyncio.create_task(launcher.broadcast())
        yield
        task.cancel()

    app = web.Application()
    app.cleanup_ctx.append(broadcasting)
    app.router.add_get("/", index)
    app.router.add_get("/api/worlds", world_list)
    app.router.add_get("/thumb/{world}", thumb)
    app.router.add_post("/api/launch", launch)
    app.router.add_post("/api/stop", stop)
    app.router.add_get("/ws", socket_)
    return app


async def serve(host, ports, open_browser):
    launcher = Launcher()
    runner = web.AppRunner(make_app(launcher))
    await runner.setup()
    for port in ports:
        try:
            await web.TCPSite(runner, host, port).start()
            break
        except OSError as e:
            if e.errno != errno.EADDRINUSE or port == ports[-1]:
                raise
            print(f"port {port} is in use", flush=True)
    url = f"http://{host}:{port}/"
    print(f"simulation launcher on {url} (Ctrl-C to stop)", flush=True)
    if open_browser:
        webbrowser.open(url)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in STOP_SIGNALS:
        loop.add_signal_handler(sig, stop.set)
    await stop.wait()
    await launcher.stop()
    for ws in list(launcher.clients):
        await ws.close()
    await runner.cleanup()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, help=f"(default {PORT}, or the next free one)")
    parser.add_argument("--no-browser", action="store_true", help="do not open the page")
    args = parser.parse_args()
    ports = [args.port] if args.port else list(range(PORT, PORT + PORT_TRIES))
    asyncio.run(serve(args.host, ports, not args.no_browser))


if __name__ == "__main__":
    main()
