"""Driver station: pixi run drive [world] [--port N] [--no-browser].

Drive the rover and look from it (the rover eye at its camera pivot), follow
it with the chase camera and see its own camera from one browser page. With a world (a path to an .sdf, or a name in
sim/worlds such as urc_autonomy) and no simulation running, it starts the
world headless (`gz sim -s -r`, the environment of sim/run.sh), stops it on
exit (Ctrl-C, kill, or closing the terminal) and exits with it; without one
it attaches to the running simulation, and spawns its cameras again when
that is restarted. One world per GZ_PARTITION and one station per world: the
rover's topics are not world-scoped.
"""
import argparse
import asyncio
import contextlib
import errno
import os
import shutil
import signal
import subprocess
import sys
import webbrowser
import xml.etree.ElementTree as ET
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))

from aiohttp import web  # noqa: E402

import gzenv  # noqa: E402
from station import link, server  # noqa: E402
from urc import sheet as sheets  # noqa: E402

DISCOVERY = 3.0  # [s] to find a running simulation
STARTUP = 120.0  # [s] for a started world to come up (big heightmaps load slowly)
STOP_TIMEOUT = 10.0  # [s] for a started world to exit on SIGINT before it is killed
STATION_WAIT = 1.5  # [s] to hear from another driver station (they announce themselves at 2 Hz)
PORT = 8765
PORT_TRIES = 10  # without --port, the next ports are tried while one is in use (another world's station)
# Ctrl-C, kill, the terminal closing. Each stops the station the same way: the
# rover gets a zero twist (DiffDrive keeps the last one forever) and a world
# the station started is stopped (it runs in its own session, so the terminal
# does not reach it). A signal ignored from the start (nohup) stays ignored.
STOP_SIGNALS = tuple(sig for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
                     if signal.getsignal(sig) is not signal.SIG_IGN)


def resolve_world(arg):
    """(path, world name) for a path or a name in sim/worlds."""
    path = Path(arg)
    if not path.is_file():
        path = SIM_DIR / "worlds" / f"{arg}.sdf"
    if not path.is_file():
        sys.exit(f"no world {arg!r}: give an .sdf path or one of "
                 f"{', '.join(sorted(p.stem for p in (SIM_DIR / 'worlds').glob('*.sdf')))}")
    return path.resolve(), ET.parse(path).getroot().find("world").get("name")


def start_simulation(path):
    """The world headless, in sim/run.sh's environment (gzenv)."""
    gz = shutil.which("gz") or str(Path(sys.prefix) / "bin" / "gz")
    print(f"starting {path.name} headless: gz sim -s -r", flush=True)
    return subprocess.Popen([gz, "sim", "-s", "-r", str(path)], env=gzenv.environment(), start_new_session=True)


def interrupt(signum, frame):
    """Outside serve(), every stop signal is a Ctrl-C."""
    raise KeyboardInterrupt


@contextlib.contextmanager
def signals_held():
    """Stop signals wait until the block is done: one that cut Popen short
    would leave the world started, unknown to the station and running."""
    held = []
    for sig in STOP_SIGNALS:
        signal.signal(sig, lambda signum, frame: held.append(signum))
    try:
        yield
    finally:
        for sig in STOP_SIGNALS:
            signal.signal(sig, interrupt)
        if held:
            raise KeyboardInterrupt


def stop_simulation(process):
    """SIGINT to the started world's process group; SIGKILL after
    STOP_TIMEOUT, or at once on another stop signal (a second Ctrl-C)."""
    if process.poll() is not None:
        return

    def kill(*_):
        with contextlib.suppress(ProcessLookupError):  # already gone
            os.killpg(process.pid, signal.SIGKILL)

    for sig in STOP_SIGNALS:
        signal.signal(sig, kill)
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(STOP_TIMEOUT)
    except subprocess.TimeoutExpired:  # seen once with cameras rendering
        kill()
        process.wait()


async def serve(app, host, ports, open_browser, process=None):
    """Serve on the first of `ports` that is free until a stop signal, or
    until `process` (the world this station started) exits."""
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in STOP_SIGNALS:
        loop.add_signal_handler(sig, stop.set)
    runner = web.AppRunner(app)
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
    app[server.STATION].url = url
    print(f"driver station on {url} (Ctrl-C to stop)", flush=True)
    if open_browser:
        webbrowser.open(url)
    while not stop.is_set():
        if process is not None and process.poll() is not None:
            print(f"gz sim exited (code {process.returncode}): the station stops too", flush=True)
            break
        with contextlib.suppress(TimeoutError):
            await asyncio.wait_for(stop.wait(), 1.0)
    await runner.cleanup()  # zero twist (Station.shutdown)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("world", nargs="?", help="world to start if no simulation is running")
    parser.add_argument("--world-name", help="attach to this running world (only one world per GZ_PARTITION "
                        "works: the rover's topics are not world-scoped)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, help=f"(default {PORT}, or the next free one)")
    parser.add_argument("--no-browser", action="store_true", help="do not open the page")
    parser.add_argument("--released", action="store_true",
                        help="start released to autonomy: no cmd_vel or LED until you take control on the page")
    args = parser.parse_args()

    for sig in STOP_SIGNALS:
        signal.signal(sig, interrupt)
    node = link.Node()
    path, name = resolve_world(args.world) if args.world else (None, args.world_name)
    process = None
    try:
        running = link.wait_for_world(node, DISCOVERY)
        if running is None and path is None:
            sys.exit("no simulation is running: start one (pixi run sim urc_autonomy) or give a world "
                     "(pixi run drive urc_autonomy)")
        if running is not None:
            worlds = link.running_worlds(node)
            name = name or running
            if name not in worlds:
                sys.exit(f"world {name} is not running (running: {', '.join(worlds)})")
            if len(worlds) > 1:
                print(f"warning: worlds {', '.join(worlds)} share this GZ_PARTITION, and with it the rover's "
                      "topics: run each world in its own partition (GZ_PARTITION=<name>)", flush=True)
        else:
            with signals_held():
                process = start_simulation(path)
            if link.wait_for_world(node, STARTUP, name, process) is None:
                code = process.poll()
                sys.exit(f"{path.name} did not come up within {STARTUP:.0f} s" if code is None else
                         f"gz sim exited (code {code}) before {name} came up: see its errors above")
        print(f"world: {name}", flush=True)
        gz = link.GzLink(name, node)
        others = gz.other_stations(wait=STATION_WAIT)
        if others:  # the rover's topics are shared by every world in the partition
            other = others[0]
            sys.exit(f"another driver station is attached to {other['world']} ({other['url'] or other['id']}): "
                     "use that one, or stop it first")
        for line in gz.ensure_cameras():
            print(line, flush=True)
        sheet_path = sheets.find(name, path)
        sheet = sheets.load(sheet_path) if sheet_path else None
        station = server.Station(gz, name, sheet, sheet_path, control=not args.released)
        ports = [args.port] if args.port else list(range(PORT, PORT + PORT_TRIES))
        asyncio.run(serve(server.make_app(station), args.host, ports, not args.no_browser, process))
    except KeyboardInterrupt:
        pass
    finally:
        if process is not None:
            stop_simulation(process)


if __name__ == "__main__":
    main()
