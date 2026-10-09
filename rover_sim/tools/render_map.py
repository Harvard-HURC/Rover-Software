#!/usr/bin/env python3
"""Orthophoto maps of the worlds for the driver station's Map view, rendered
by Gazebo itself (realism design section 8.2, decision D17; a port of
sim/data/research/flycam/prototype/exp5_orthomap.py).

    pixi run sim-maps                          # every world with a sheet whose map is missing or stale
    python sim/tools/render_map.py urc_autonomy [--pixels 4096] [--force]
    python sim/tools/render_map.py --check     # list missing and stale maps; exit 1 if any

A world can be mapped when it has a sheet (urc/sheet.py) giving its terrain:
the URC worlds and the proving ground.

Capture: a copy of the world without the rover (the station draws it live)
runs headless with a FlyCamera (viewers.FlyParams with a square picture and
no smoothing). It jumps over each tile in turn, ABOVE m over the highest
terrain, looking straight down with north up, and each tile's picture is the
first frame stamped SETTLE s after the jump. From 300 m cast shadows still
render (they vanish by ~590 m, and an orthographic camera renders none), and
the field of view stays above the 0.1 rad SDF minimum.
Assembly: every map pixel is projected through its tile's camera at its
terrain height (the world's own heightmap, decoded by the sheet), so the
ground meets across tiles without parallax steps; neighbours cross-fade over
FEATHER_PX in their overlap. Things that stand up (rocks, the lander) lean
away from their tile's centre, as in any orthophoto from a perspective camera.
Output: <world>_map.jpg next to the world (row 0 north, column 0 west: the
terrain square centred on the world origin) and <world>_map.json (size,
ground sample distance, frame, and the SHA1 of every input: the world SDF,
every file its models reference, the media patch). It needs the GPU, so it is
not part of sim-test; status(world path) tells the station whether a map is
current.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
sys.path.insert(0, str(SIM_DIR / "tests"))
import gz_media  # noqa: E402  (the media patch and the light it is filled with)
import gzenv  # noqa: E402
import viewers  # noqa: E402
import worldfiles  # noqa: E402  (temporary world copies, as the tests make them)
from urc import sheet as sheets  # noqa: E402

WORLDS = SIM_DIR / "worlds"
FORMAT = "rover-map/1"
PIXELS = 4096  # map side: 6.25 cm/px over 256 m, 0.25 m over 1 km, 0.5 m over 2 km (section 8.2)
# [m] camera above the highest terrain: cast shadows render up to at least 400 m
# from the camera and are gone by 590 m (measurements/projection_and_shadow_tests.json).
ABOVE = 300.0
MARGIN = 0.12  # tile overlap on each side, fraction of the tile (prototype: seams continuous)
# [m] largest tile: from 300 m its corners are seen 0.98 rad off vertical (the
# prototype's Autonomy tile), which bounds how far tall things lean (A).
MAX_TILE_M = 256.0
MAX_TILE_PX = 1280  # largest camera picture side (A: the size the fly camera renders anyway)
MIN_HFOV = 0.1  # [rad] SDF rejects a smaller horizontal field of view (measured)
# Cross-fade width at tile seams [map px]: without one, the brightness step
# at a seam was 1.0-1.9x the adjacent-pixel difference (prototype); narrow,
# so leaning objects ghost over few pixels (A).
FEATHER_PX = 32
SETTLE = 0.15  # [s] sim time between the jump and the first usable frame (prototype)
RATE = 10.0  # [Hz] tile camera (prototype)
JPEG_QUALITY = 88  # 2-3.3 MB at 4096 px (prototype)
MEDIA_DIFF = SIM_DIR / "patches" / "gz-rendering8-ogre2-media.diff"  # the media patch (gz_media.py)
ROVER_INCLUDE = re.compile(r"<include>\s*<uri>model://rover</uri>.*?</include>", re.S)
# What only physics reads, which no picture shows: collisions (collision heightmaps and meshes, surface
# friction) and the world's physics settings.
PHYSICS_ONLY = re.compile(r"<collision\b.*?</collision>|<physics\b.*?</physics>", re.S)
MODEL_URI = re.compile(r"model://([^<>\s\"']+)")
FILE_URI = re.compile(r"file://(/[^<>\s\"']+)")


# --- Inputs and staleness ------------------------------------------------------------------

def map_paths(world_path, out_dir=None):
    """(<world>_map.jpg, <world>_map.json) in out_dir, by default next to the world."""
    world_path = Path(world_path)
    directory = Path(out_dir) if out_dir else world_path.parent
    return directory / f"{world_path.stem}_map.jpg", directory / f"{world_path.stem}_map.json"


def terrain(world_path):
    """The world's terrain from the sheet next to it (urc/sheet.py): a
    Heightfield centred on the world origin, as Gazebo draws it."""
    world_path = Path(world_path)
    sheet_path = sheets.find(world_path.stem, world_path)
    if sheet_path is None:
        raise ValueError(f"{world_path} has no sheet, so no terrain to map")
    return sheets.terrain(sheets.load(sheet_path), sheet_path)


def _resource_dirs():
    env = gzenv.environment()
    return [Path(p) for p in env["GZ_SIM_RESOURCE_PATH"].split(os.pathsep) if p]


def _resolve(uri_path, dirs):
    """model://<uri_path> as a file (a bare model name: its model.sdf); None if not found."""
    name, _, rest = uri_path.partition("/")
    for d in dirs:
        candidate = d / name / (rest or "model.sdf")
        if candidate.is_file():
            return candidate
    return None


def _sha1(path):
    h = hashlib.sha1()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _key(path):
    path = Path(path).resolve()
    return str(path.relative_to(SIM_DIR)) if path.is_relative_to(SIM_DIR) else str(path)


def _shown(sdf_text):
    """An SDF document without what only physics reads (PHYSICS_ONLY)."""
    return PHYSICS_ONLY.sub("", sdf_text)


def inputs(world_path):
    """{file: SHA1} of everything the map shows: the world SDF and every
    file referenced through model:// or file:// from it and from the models
    it includes (heightmaps, colour maps, meshes, textures), except the rover,
    which the map leaves out, and what only physics reads (PHYSICS_ONLY: a
    collision heightmap or mesh, surface friction or the solver changes no
    picture, measured stale after a collision-only change); each SDF hashed
    without those. Plus "media": which gz-rendering media render it (the
    patch's SHA1 with the sun, sky and haze it is filled with, gz_media.tokens,
    or "stock")."""
    dirs = _resource_dirs()
    world_text = ROVER_INCLUDE.sub("", Path(world_path).read_text())
    out = {_key(world_path): hashlib.sha1(_shown(world_text).encode()).hexdigest()}
    pending = [_shown(world_text)]
    seen = set()
    while pending:
        text = pending.pop()
        uris = [(f"model://{u}", _resolve(u, dirs)) for u in MODEL_URI.findall(text)]
        uris += [(f"file://{u}", Path(u) if Path(u).is_file() else None) for u in FILE_URI.findall(text)]
        for uri, path in uris:
            if uri in seen:
                continue
            seen.add(uri)
            if path is None:
                out[uri] = "missing"
                continue
            if path.suffix == ".sdf":
                shown = _shown(path.read_text())
                out[_key(path)] = hashlib.sha1(shown.encode()).hexdigest()
                pending.append(shown)
            else:
                out[_key(path)] = _sha1(path)
    patched = "GZ_RENDERING_RESOURCE_PATH" in gzenv.environment()
    if patched:
        diff = MEDIA_DIFF.read_bytes() if MEDIA_DIFF.is_file() else b"patched"
        out["media"] = hashlib.sha1(diff + json.dumps(gz_media.tokens(), sort_keys=True).encode()).hexdigest()
    else:
        out["media"] = "stock"
    return out


def status(world_path, out_dir=None):
    """("missing" | "stale" | "current", [inputs added, changed or gone]) of
    the world's map (map_paths): a map is stale when any input differs from
    the ones it was rendered from."""
    image_path, meta_path = map_paths(world_path, out_dir)
    if not (meta_path.is_file() and image_path.is_file()):
        return "missing", []
    meta = json.loads(meta_path.read_text())
    then = meta.get("inputs", {}) if meta.get("format") == FORMAT else {}
    now = inputs(world_path)
    changed = sorted(k for k in then.keys() | now.keys() if then.get(k) != now.get(k))
    return ("stale" if changed else "current"), changed


# --- Tiles ------------------------------------------------------------------------------

def layout(size, pixels, max_tile_m=MAX_TILE_M, max_tile_px=MAX_TILE_PX):
    """(tiles per side, tile [m], camera picture side [px], hfov [rad]) for a
    square of `size` m mapped at `pixels` px: the fewest tiles within both
    maxima, a whole number of map pixels each. The camera sees its tile plus
    MARGIN on every side, or more where that would be narrower than MIN_HFOV
    (a world smaller than ~24 m)."""
    gsd = size / pixels
    need = max(math.ceil(size / max_tile_m), math.ceil(size * (1 + 2 * MARGIN) / gsd / max_tile_px), 1)
    k = next(k for k in range(need, pixels + 1) if pixels % k == 0)
    tile = size / k
    footprint = max(tile * (1 + 2 * MARGIN), 2 * ABOVE * math.tan(MIN_HFOV / 2))
    px = round(footprint / gsd)
    if px > max_tile_px:
        raise ValueError(f"a {size:g} m world at {pixels} px needs {px} px tile pictures; map it with fewer pixels")
    if FEATHER_PX / 2 > MARGIN * pixels / k:
        raise ValueError(f"{k} tiles of {pixels // k} px leave no room for a {FEATHER_PX} px feather")
    return k, tile, px, 2 * math.atan(footprint / 2 / ABOVE)


def tile_centres(size, k):
    """World (x, y) of each tile's centre, north row first, west to east."""
    return [(-size / 2 + (i + 0.5) * size / k, size / 2 - (j + 0.5) * size / k) for j in range(k) for i in range(k)]


def look_down(yaw=math.pi / 2):
    """(w, x, y, z) of RPY(0, pi/2, yaw): looking straight down; yaw pi/2 puts north at the top."""
    h = math.sqrt(0.5)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return h * cy, -h * sy, h * cy, h * sy


def fly_model(fly, pose):
    """A fly camera of FlyParams `fly` at `pose` (x, y, z, roll, pitch, yaw)
    as a <model> element to put in a world."""
    model = ET.fromstring(viewers.build_fly_sdf(fly)).find("model")
    ET.SubElement(model, "pose").text = " ".join(f"{v:.9g}" for v in pose)
    return ET.tostring(model, encoding="unicode")


def camera_world(world_text, fly, pose):
    """A world document with its rover removed and fly_model(fly, pose) added."""
    head, tail = ROVER_INCLUDE.sub("", world_text).rsplit("</world>", 1)
    return head + fly_model(fly, pose) + "\n  </world>" + tail


class FlyServer:
    """A world served headless by `gz sim -s -r`, with a fly camera in it:
    jump the camera, set its mode, read its state and its frames. The server
    inherits this process's environment, which main() (or tests/simulate.py)
    set with gzenv, so both share a transport partition. Only publishing and
    subscribing, no service calls (README "Gazebo lessons": blocking requests
    stall)."""

    def __init__(self, world_path, start_timeout=300.0):
        self.world_path = Path(world_path)
        self.start_timeout = start_timeout
        self.frames = deque(maxlen=4)  # (stamp [s], RGB array)
        self.latest = {}
        self.lock = threading.Lock()

    def __enter__(self):
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node
        self._pose_type, self._string_type = Pose, StringMsg
        self.node = Node()
        self.node.subscribe(StringMsg, viewers.FLY_STATE_TOPIC, self._on_state)
        self.node.subscribe(Image, viewers.FLY_IMAGE_TOPIC, self._on_image)
        self._goto = self.node.advertise(viewers.FLY_GOTO_TOPIC, Pose)
        self._mode = self.node.advertise(viewers.FLY_MODE_TOPIC, StringMsg)
        self.log = tempfile.TemporaryFile()
        gz = shutil.which("gz") or str(Path(sys.prefix) / "bin" / "gz")
        self.process = subprocess.Popen([gz, "sim", "-s", "-r", "-v", "2", str(self.world_path)],
                                        stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            self._wait(lambda: self.latest, self.start_timeout, "the fly camera's state")
            self._wait(lambda: self.frames, 60.0, "a frame")
        except BaseException:
            self.__exit__()
            raise
        return self

    def __exit__(self, *exc):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.log.close()

    def _on_state(self, msg):
        with self.lock:
            self.latest = json.loads(msg.data)

    def _on_image(self, msg):
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        with self.lock:
            self.frames.append((msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9, rgb))

    def _poll(self, ready, timeout):
        """ready()'s first true value within timeout [s], under the lock; None if none."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and self.process.poll() is None:
            with self.lock:
                value = ready()
            if value:
                return value
            time.sleep(0.02)
        return None

    def _wait(self, ready, timeout, what):
        """_poll, or a RuntimeError with the end of the server's log."""
        value = self._poll(ready, timeout)
        if value:
            return value
        self.log.seek(0)
        tail = self.log.read().decode(errors="replace")[-3000:]
        code = self.process.poll()
        raise RuntimeError(f"no {what} from {self.world_path.name} within {timeout:.0f} s "
                           f"({'gz sim exited with ' + str(code) if code is not None else 'gz sim runs'}):\n{tail}")

    def state(self):
        with self.lock:
            return dict(self.latest)

    def jump(self, x, y, z, quaternion=look_down(), timeout=30.0):
        """Put the camera at (x, y, z) at once, looking along `quaternion`
        (w, x, y, z); returns the first state that reports it there."""
        msg = self._pose_type()
        msg.position.x, msg.position.y, msg.position.z = x, y, z
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = quaternion
        msg.header.data.add(key=viewers.FLY_JUMP_KEY)
        with self.lock:
            self.latest = {}

        def there():
            s = self.latest
            return s if s and abs(s["x"] - x) < 1e-3 and abs(s["y"] - y) < 1e-3 and abs(s["z"] - z) < 1e-3 else None

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and self.process.poll() is None:
            self._goto.publish(msg)  # again until it lands: the first may beat the subscription
            state = self._poll(there, 0.25)
            if state:
                return state
        return self._wait(there, 0.0, f"state at the jump target {x:.1f}, {y:.1f}, {z:.1f}")

    def mode(self, text, check, timeout=30.0, every=3.0):
        """Send mode `text` until a state passes check(state); returns that
        state. Sent again only every `every` s: "ortho <width>" starts a
        flight of up to 1.5 s, which each copy would start over."""
        msg = self._string_type()
        msg.data = text

        def passed():
            return self.latest if self.latest and check(self.latest) else None

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and self.process.poll() is None:
            self._mode.publish(msg)
            state = self._poll(passed, min(every, deadline - time.monotonic()))
            if state:
                return dict(state)
        return dict(self._wait(passed, 0.0, f"state after mode {text!r}"))

    def frame_after(self, stamp, timeout=60.0):
        """The first frame stamped after `stamp` [s of sim time]: (stamp, RGB)."""
        return self._wait(lambda: next((f for f in self.frames if f[0] > stamp), None), timeout,
                          f"frame after t = {stamp:.2f} s")


def capture(world_path, hf, pixels=PIXELS, max_tile_m=MAX_TILE_M):
    """Fly over every tile of the world, whose terrain is Heightfield hf:
    (geometry, [(cx, cy, RGB)])."""
    k, tile, px, hfov = layout(hf.size, pixels, max_tile_m)
    z_top = float(hf.z.max())
    cam_z = z_top + ABOVE
    fly = replace(viewers.FlyParams(), clearance=0.0, time_constant=0.0, look_time_constant=0.0,
                  deadman_clock="sim", rate=RATE, size=(px, px), hfov=hfov,
                  clip=(max(1.0, ABOVE - 100.0), ABOVE + z_top - float(hf.z.min()) + 400.0))
    centres = tile_centres(hf.size, k)
    world_path = Path(world_path)
    text = camera_world(world_path.read_text(), fly, (*centres[0], cam_z, 0.0, math.pi / 2, math.pi / 2))
    geometry = {"size_m": hf.size, "pixels": pixels, "tiles_per_side": k, "tile_m": tile,
                "camera": {"px": px, "hfov": hfov, "z": cam_z, "above_highest_terrain": ABOVE}}
    tiles = []
    with worldfiles.temp_sdf(text, world_path.parent) as path, FlyServer(path) as server:
        for cx, cy in centres:
            state = server.jump(cx, cy, cam_z)
            _, rgb = server.frame_after(state["t"] + SETTLE)
            tiles.append((cx, cy, rgb))
    return geometry, tiles


def _ramp(index, start, m, feather_px):
    """Weight of map pixels `index` in a tile spanning [start, start + m):
    1 inside, ramping to 0 across a feather_px band centred on its edges, so
    that the weights of two neighbours sum to 1 in the band."""
    inside = np.minimum(index - start, start + m - 1 - index)
    return np.clip((inside + feather_px // 2 + 0.5) / feather_px, 0, 1)


def assemble(heights, size, pixels, tile_m, cam_z, hfov, tiles, feather_px=FEATHER_PX):
    """The orthophoto (pixels x pixels x 3, uint8): each map pixel projected
    through the tile cameras at its terrain height (heights: the heightmap,
    row 0 north, spanning `size`), tiles cross-fading over feather_px."""
    gsd = size / pixels
    n = heights.shape[0]
    heights = heights.astype(np.float32)
    total = np.zeros((pixels, pixels, 3), np.float32)
    weight = np.zeros((pixels, pixels), np.float32)
    m = round(tile_m / gsd)
    half = feather_px // 2
    for cx, cy, rgb in tiles:
        side = rgb.shape[0]
        f = (side / 2) / math.tan(hfov / 2)
        j0 = round((cx - tile_m / 2 + size / 2) / gsd)
        i0 = round((size / 2 - (cy + tile_m / 2)) / gsd)
        rows = np.arange(max(0, i0 - half), min(pixels, i0 + m + half))
        cols = np.arange(max(0, j0 - half), min(pixels, j0 + m + half))
        X, Y = np.meshgrid(-size / 2 + (cols + 0.5) * gsd, size / 2 - (rows + 0.5) * gsd)
        Z = cv2.remap(heights, ((X + size / 2) / size * (n - 1)).astype(np.float32),
                      ((size / 2 - Y) / size * (n - 1)).astype(np.float32), cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_REPLICATE)
        d = cam_z - Z
        u = side / 2 + f * (X - cx) / d - 0.5
        v = side / 2 - f * (Y - cy) / d - 0.5
        part = cv2.remap(rgb, u.astype(np.float32), v.astype(np.float32), cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REFLECT)
        w = np.outer(_ramp(rows, i0, m, feather_px), _ramp(cols, j0, m, feather_px)).astype(np.float32)
        total[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1] += part * w[..., None]
        weight[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1] += w
    return np.clip(total / weight[..., None] + 0.5, 0, 255).astype(np.uint8)


def render(world_path, pixels=PIXELS, out_dir=None, max_tile_m=MAX_TILE_M):
    """Render, assemble and write the world's map (map_paths); returns its metadata."""
    started = time.monotonic()
    world_inputs = inputs(world_path)
    hf = terrain(world_path)
    geometry, tiles = capture(world_path, hf, pixels, max_tile_m)
    rendered = time.monotonic() - started
    image = assemble(hf.z, hf.size, pixels, geometry["tile_m"], geometry["camera"]["z"], geometry["camera"]["hfov"],
                     tiles)
    image_path, meta_path = map_paths(world_path, out_dir)
    image_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(image_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    meta = {"format": FORMAT, "world": Path(world_path).stem, "image": image_path.name, **geometry,
            "gsd_m": hf.size / pixels, "frame": "row 0 north, column 0 west; the terrain square centred on the world origin, x east, y north",
            "feather_px": FEATHER_PX, "rover": "left out (drawn live)",
            "shadows": f"cast shadows as seen from {ABOVE:.0f} m above the highest terrain",
            "rendered": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
            "seconds": {"render": round(rendered, 1), "total": round(time.monotonic() - started, 1)},
            "inputs": world_inputs}
    meta_path.write_text(json.dumps(meta, indent=1) + "\n")
    return meta


def mapped_worlds():
    """Every world in sim/worlds with a sheet: the ones with a terrain to map."""
    return sorted(p.stem for p in WORLDS.glob("*.sdf")
                  if not p.name.startswith("tmp") and p.with_suffix(".json").is_file())


def main():
    parser = argparse.ArgumentParser(description="Render orthophoto maps of the worlds (needs the GPU).")
    parser.add_argument("worlds", nargs="*", help="world names in sim/worlds (default: every world with a sheet)")
    parser.add_argument("--pixels", type=int, default=PIXELS, help=f"map side [px] (default {PIXELS})")
    parser.add_argument("--force", action="store_true", help="render even if the map is current")
    parser.add_argument("--check", action="store_true", help="only report missing and stale maps (exit 1 if any)")
    parser.add_argument("--build-dir", help="plugin build directory (default sim/build)")
    args = parser.parse_args()
    # Before any gz-transport node exists: the server and this process share a private partition.
    os.environ.update(gzenv.environment(args.build_dir, partition=f"render_map_{os.getpid()}", ip="127.0.0.1"))
    worlds = args.worlds or mapped_worlds()
    unknown = [w for w in worlds if w not in mapped_worlds()]
    if unknown:
        parser.error(f"no world with a sheet named {', '.join(unknown)} in {WORLDS}")
    outdated = 0
    for world in worlds:
        path = WORLDS / f"{world}.sdf"
        state, changed = status(path)
        if args.check or (state == "current" and not args.force):
            print(f"{world}: {state}" + (f" ({', '.join(changed[:5])}{' ...' if len(changed) > 5 else ''})"
                                         if changed else ""))
            outdated += state != "current"
            continue
        meta = render(path, args.pixels)
        print(f"{world}: {meta['tiles_per_side'] ** 2} tiles of {meta['tile_m']:g} m "
              f"({meta['camera']['px']} px, hfov {meta['camera']['hfov']:.3f} rad) in {meta['seconds']['render']} s, "
              f"{meta['seconds']['total']} s in all -> {map_paths(path)[0].relative_to(SIM_DIR.parent)}")
    return 1 if args.check and outdated else 0


if __name__ == "__main__":
    sys.exit(main())
