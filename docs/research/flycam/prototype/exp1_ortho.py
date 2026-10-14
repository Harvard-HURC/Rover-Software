"""Experiment 1: does a camera sensor honour an orthographic projection?
(a) SDF <projection_type>orthographic</projection_type> inside <camera>
(b) SDF <lens><type>orthographic</type></lens>
(c) the prototype plugin switching the rendering camera to CPT_ORTHOGRAPHIC
Each looks straight down at the lander from 40 m and 80 m: a perspective
picture halves in scale, an orthographic one does not."""
import sys
import time

import cv2
import numpy as np

import gzrun
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_pb2 import Pose

T = gzrun.T
out = T / "exp1"
out.mkdir(exist_ok=True)
X, Y = 49, 0
models = []
for h in (40, 80):
    models.append(gzrun.camera_model(f"persp{h}", pose=f"{X} {Y} {h} 0 1.5707963 0", static="true"))
    models.append(gzrun.camera_model(f"projtype{h}", pose=f"{X} {Y} {h} 0 1.5707963 0", static="true",
                                     extra="<projection_type>orthographic</projection_type>"))
    models.append(gzrun.camera_model(f"lens{h}", pose=f"{X} {Y} {h} 0 1.5707963 0", static="true",
                                     extra="<lens><type>orthographic</type><scale_to_hfov>true</scale_to_hfov></lens>"))
plugin = ('<plugin filename="FlyCamProto" name="rover_sim::FlyCamProto">'
          f'<start>{X} {Y} 40 0 1.5707963 0</start><clearance>1.0</clearance></plugin>')
models.append(gzrun.camera_model("flycam", pose=f"{X} {Y} 40 0 1.5707963 0", topic="/flycam/image", plugin=plugin))
world = gzrun.make_world(T / "worlds/urc_equipment_servicing.sdf", T / "exp1_world.sdf", models)
name = gzrun.world_name(world)
node = gzrun.Node()
proc = gzrun.start(world, out / "server.log")
try:
    assert gzrun.wait_world(node, name), "world did not start"
    grabs = {m: gzrun.Grabber(node, f"/{m}/image") for h in (40, 80) for m in (f"persp{h}", f"projtype{h}", f"lens{h}")}
    fly = gzrun.Grabber(node, "/flycam/image")
    for g in list(grabs.values()) + [fly]:
        ok = g.wait(3, 120)
        print(g.topic, "frames" if ok else "NO FRAMES", len(g.frames), flush=True)
    for m, g in grabs.items():
        cv2.imwrite(str(out / f"{m}.png"), cv2.cvtColor(g.frames[-1][2], cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / "fly_persp40.png"), cv2.cvtColor(fly.frames[-1][2], cv2.COLOR_RGB2BGR))
    ortho_pub = node.advertise("/flycam/ortho", Double)
    goto_pub = node.advertise("/flycam/goto", Pose)
    time.sleep(1.0)
    for width, h in ((60.0, 40), (60.0, 80), (60.0, 400)):
        msg = Double(); msg.data = width
        ortho_pub.publish(msg)
        p = Pose(); p.position.x, p.position.y, p.position.z = X, Y, h
        # pitch +90 deg about y: (w, x, y, z) of RPY(0, pi/2, 0)
        p.orientation.w, p.orientation.y = np.cos(np.pi / 4), np.sin(np.pi / 4)
        goto_pub.publish(p)
        stamp = fly.frames[-1][0]
        time.sleep(1.5)
        f = fly.latest_after(stamp + 1.0)
        print("ortho", width, "h", h, "frame", f is not None, flush=True)
        cv2.imwrite(str(out / f"fly_ortho{int(width)}_h{h}.png"), cv2.cvtColor(fly.frames[-1][2], cv2.COLOR_RGB2BGR))
    msg = Double(); msg.data = 0.0
    ortho_pub.publish(msg)
    time.sleep(1.5)
    cv2.imwrite(str(out / "fly_back_to_persp_h400.png"), cv2.cvtColor(fly.frames[-1][2], cv2.COLOR_RGB2BGR))
finally:
    gzrun.stop(proc)
