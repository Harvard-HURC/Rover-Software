"""Experiment 8: what the inspection camera shows, in the Autonomy world
(real 3DEP terrain): a few poses from low oblique to top-down perspective to
orthographic, saved as a contact sheet; also checks the weak_ptr fix (no crash
on shutdown after using the orthographic projection)."""
import json
import math
import threading
import time

import cv2
import numpy as np

import gzrun
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.stringmsg_pb2 import StringMsg

T = gzrun.T
import sys as _s
out = T / ("exp8" if len(_s.argv) < 2 else f"exp8_{_s.argv[1]}")
out.mkdir(exist_ok=True)
START = "150 90 70 0 0.3 1.2"
plugin = ('<plugin filename="FlyCamProto" name="rover_sim::FlyCamProto">'
          f'<start>{START}</start><clearance>1.0</clearance><hook_event>sceneupdate</hook_event></plugin>')
models = [gzrun.camera_model("flycam", pose=START, topic="/flycam/image", plugin=plugin, w=1280, h=720, far=6000)]
import sys
WORLD = sys.argv[1] if len(sys.argv) > 1 else "urc_autonomy"
world = gzrun.make_world(T / "worlds" / f"{WORLD}.sdf", T / f"exp8_{WORLD}.sdf", models)
name = gzrun.world_name(world)
node = gzrun.Node()
state = {}
lock = threading.Lock()


def on_state(m):
    with lock:
        state.update(json.loads(m.data))


node.subscribe(StringMsg, "/flycam/state", on_state)
proc = gzrun.start(world, out / "server.log")


def pose_msg(x, y, z, yaw, pitch):
    q = Pose()
    q.position.x, q.position.y, q.position.z = x, y, z
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    q.orientation.w, q.orientation.x, q.orientation.y, q.orientation.z = cp * cy, -sp * sy, sp * cy, cp * sy
    return q


SHOTS = [  # label, x, y, z, yaw, pitch, ortho width
    ("1 behind the rover, 3 m up", 160, 112, 61, 1.45, 0.25, 0),
    ("2 over C2 toward the mesa, 40 m up", 150, 60, 100, 1.75, 0.35, 0),
    ("3 route-finding mesa, oblique", 260, 420, 140, 2.2, 0.45, 0),
    ("4 top-down, perspective, 600 m", 120, 300, 660, 1.5707963, 1.5707, 0),
    ("5 top-down, orthographic 900 m wide", 120, 300, 660, 1.5707963, 1.5707, 900),
    ("6 top-down, orthographic 120 m wide (posts)", 190, 550, 300, 1.5707963, 1.5707, 120),
]
try:
    assert gzrun.wait_world(node, name, 300)
    grab = gzrun.Grabber(node, "/flycam/image")
    assert grab.wait(3, 300)
    goto = node.advertise("/flycam/goto", Pose)
    ortho = node.advertise("/flycam/ortho", Double)
    time.sleep(1)
    tiles = []
    for label, x, y, z, yaw, pitch, width in SHOTS:
        m = Double(); m.data = float(width)
        ortho.publish(m)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            goto.publish(pose_msg(x, y, z, yaw, pitch))
            time.sleep(0.05)
            with lock:
                s = dict(state)
            if s and abs(s["x"] - x) < 1e-3 and abs(s["y"] - y) < 1e-3:
                break
        f = grab.latest_after(s["t"] + 0.4)
        im = cv2.cvtColor(f[2], cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out / f"shot{label[0]}.jpg"), im, [cv2.IMWRITE_JPEG_QUALITY, 88])
        t = cv2.resize(im, (640, 360), interpolation=cv2.INTER_AREA)
        cv2.rectangle(t, (0, 0), (640, 30), (0, 0, 0), -1)
        cv2.putText(t, f"{label}  (agl {s['z'] - s['ground']:.0f} m)", (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(t)
    sheet = np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)])
    cv2.imwrite(str(out / "flycam_contact_sheet.jpg"), sheet, [cv2.IMWRITE_JPEG_QUALITY, 88])
finally:
    gzrun.stop(proc)
crashed = "~FlyCamProto" in (out / "server.log").read_text(errors="ignore")
print("shutdown crash:", crashed)
