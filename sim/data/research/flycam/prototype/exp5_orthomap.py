"""Experiment 5: an offline orthophoto of a sim world, rendered by Gazebo.

A square perspective camera looks straight down (north up) from ~300 m
above each tile (telephoto, so cast shadows still render: they vanish past
~500 m, and an orthographic camera renders none, see exp1/exp2). Each tile is
then orthorectified with the world's own heightmap (every map pixel is
projected through the tile camera at its terrain height) and the tiles are
stitched by nearest tile centre. The rover is left out of the world (the
station draws it live).

usage: exp5_orthomap.py <world> <metres per pixel> <tile size m> [out dir]"""
import json
import math
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

import gzrun
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.stringmsg_pb2 import StringMsg

T = gzrun.T
world_key = sys.argv[1]
GSD = float(sys.argv[2])
TILE = float(sys.argv[3])
OUT = Path(sys.argv[4]) if len(sys.argv) > 4 else T / "exp5"
OUT.mkdir(parents=True, exist_ok=True)
SAVE_RAW = False
MARGIN = 0.12  # tile overlap on each side, fraction of the tile
PITCH = 1.5707  # the plugin clamps pitch here
ABOVE = 300.0  # [m] camera above the highest terrain: cast shadows render within ~500 m

sheet_path = T / "worlds" / f"{world_key}.json"
sheet = json.loads(sheet_path.read_text())
terrain = sheet["terrain"]
size = float(terrain["size_m"]) if np.isscalar(terrain["size_m"]) else float(terrain["size_m"][0])
hm = cv2.imread(str(sheet_path.parent / terrain["heightmap"]), cv2.IMREAD_UNCHANGED).astype(np.float32)
if hm.ndim == 3:
    hm = hm[..., 0]
heights = hm / 65535.0 * float(terrain["z_max"])
n = heights.shape[0]
zmax = float(heights.max())

PX = int(round(TILE * (1 + 2 * MARGIN) / GSD))  # camera image side [px]
CAM_Z = zmax + ABOVE
footprint = TILE * (1 + 2 * MARGIN)
HFOV = 2 * math.atan(footprint / 2 / ABOVE)  # at the highest terrain; lower ground is covered with more margin
print(f"{world_key}: {size:.0f} m, z 0..{zmax:.1f} m, tile {TILE} m -> camera {PX}x{PX} px, hfov {HFOV:.3f} rad, "
      f"camera z {CAM_Z:.0f} m", flush=True)

plugin = ('<plugin filename="FlyCamProto" name="rover_sim::FlyCamProto">'
          f'<start>0 0 {CAM_Z} 0 1.5707 1.5707963</start><clearance>0</clearance><time_constant>0</time_constant>'
          '<ortho_hook>false</ortho_hook></plugin>')
models = [gzrun.camera_model("mapcam", pose=f"0 0 {CAM_Z} 0 1.5707 1.5707963", topic="/mapcam/image",
                             plugin=plugin, hfov=HFOV, w=PX, h=PX, near=max(1.0, ABOVE - 100), far=ABOVE + zmax + 400,
                             rate=10)]
world = gzrun.make_world(T / "worlds" / f"{world_key}.sdf", T / f"exp5_{world_key}.sdf", models, strip_rover=True)
name = gzrun.world_name(world)
node = gzrun.Node()
state = {}
lock = threading.Lock()


def on_state(m):
    with lock:
        state.update(json.loads(m.data))


node.subscribe(StringMsg, "/flycam/state", on_state)
proc = gzrun.start(world, OUT / f"server_{world_key}.log")
t_start = time.monotonic()
try:
    assert gzrun.wait_world(node, name, 300), "world did not start"
    grab = gzrun.Grabber(node, "/mapcam/image")
    assert grab.wait(3, 300), "no frames"
    print(f"world up and rendering after {time.monotonic() - t_start:.1f} s", flush=True)
    goto = node.advertise("/flycam/goto", Pose)
    time.sleep(1.0)
    k = int(round(size / TILE))
    centers = [(-size / 2 + (i + 0.5) * TILE, size / 2 - (j + 0.5) * TILE) for j in range(k) for i in range(k)]
    tiles = []
    t_render = time.monotonic()
    for cx, cy in centers:
        p = Pose()
        p.position.x, p.position.y, p.position.z = cx, cy, CAM_Z
        # RPY(0, PITCH, pi/2): optical axis down, image top = north (PITCH just short of pi/2,
        # where the plugin's Euler angles are still unique)
        cp, sp, cw, sw = math.cos(PITCH / 2), math.sin(PITCH / 2), math.cos(math.pi / 4), math.sin(math.pi / 4)
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = cp * cw, -sp * sw, sp * cw, cp * sw
        t_tile = time.monotonic()
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            goto.publish(p)
            time.sleep(0.05)
            with lock:
                s = dict(state)
            if s and abs(s["x"] - cx) < 1e-3 and abs(s["y"] - cy) < 1e-3:
                break
        t_pose = time.monotonic() - t_tile
        frame = grab.latest_after(s["t"] + 0.15, 60)
        assert frame is not None, f"no frame for tile {cx}, {cy}"
        print(f"tile {cx:.0f},{cy:.0f}: pose {t_pose:.2f} s (state x {s.get('x')}, y {s.get('y')}, t {s.get('t')}), "
              f"frame stamp {frame[0]:.2f} after {time.monotonic() - t_tile:.2f} s, mean {frame[2].mean():.0f}", flush=True)
        if SAVE_RAW: cv2.imwrite(str(OUT / f"raw_{world_key}_{cx:.0f}_{cy:.0f}.jpg"), cv2.cvtColor(frame[2], cv2.COLOR_RGB2BGR))
        tiles.append((cx, cy, frame[2]))
        with grab.lock:
            grab.frames = [f for f in grab.frames if f[0] > s["t"]]
    render_s = time.monotonic() - t_render
    print(f"{len(tiles)} tiles in {render_s:.1f} s", flush=True)
finally:
    gzrun.stop(proc)

# --- Orthorectify and stitch -------------------------------------------------------
t0 = time.monotonic()
N = int(round(size / GSD))
f = (PX / 2) / math.tan(HFOV / 2)
mosaic = np.zeros((N, N, 3), np.uint8)
xs = -size / 2 + (np.arange(N) + 0.5) * GSD
for cx, cy, rgb in tiles:
    # Map pixels whose nearest tile centre is this one (a TILE x TILE square).
    j0 = int(round((cx - TILE / 2 + size / 2) / GSD))
    i0 = int(round((size / 2 - (cy + TILE / 2)) / GSD))
    m = int(round(TILE / GSD))
    X, Y = np.meshgrid(xs[j0:j0 + m], (size / 2 - (np.arange(i0, i0 + m) + 0.5) * GSD))
    # Terrain height at each map pixel, bilinear in the heightmap (row 0 north, col 0 west).
    col = (X + size / 2) / size * (n - 1)
    row = (size / 2 - Y) / size * (n - 1)
    Z = cv2.remap(heights, col.astype(np.float32), row.astype(np.float32), cv2.INTER_LINEAR)
    d = CAM_Z - Z
    u = PX / 2 + f * (X - cx) / d - 0.5
    v = PX / 2 - f * (Y - cy) / d - 0.5
    mosaic[i0:i0 + m, j0:j0 + m] = cv2.remap(rgb, u.astype(np.float32), v.astype(np.float32), cv2.INTER_LINEAR,
                                             borderMode=cv2.BORDER_REFLECT)
print(f"orthorectified {N}x{N} px in {time.monotonic() - t0:.1f} s", flush=True)
bgr = cv2.cvtColor(mosaic, cv2.COLOR_RGB2BGR)
cv2.imwrite(str(OUT / f"{world_key}_ortho_{GSD:g}m.jpg"), bgr, [cv2.IMWRITE_JPEG_QUALITY, 88])
small = cv2.resize(bgr, (1024, 1024), interpolation=cv2.INTER_AREA)
cv2.imwrite(str(OUT / f"{world_key}_ortho_1024.jpg"), small, [cv2.IMWRITE_JPEG_QUALITY, 90])
meta = {"world": name, "size_m": size, "gsd_m": GSD, "pixels": N, "tile_m": TILE, "tiles": len(tiles),
        "camera": {"px": PX, "hfov": HFOV, "z": CAM_Z, "above_highest_terrain": ABOVE},
        "frame": "row 0 north, column 0 west, centred on the world origin; x east, y north",
        "render_seconds": round(render_s, 1), "rover": "left out (drawn live)"}
(OUT / f"{world_key}_ortho.json").write_text(json.dumps(meta, indent=1))
print(json.dumps(meta))
