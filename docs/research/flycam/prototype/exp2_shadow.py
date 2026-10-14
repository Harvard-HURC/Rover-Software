"""Experiment 2: do cast shadows survive a high, narrow (telephoto) perspective
camera looking straight down? Footprint 60 m wide over the lander at several
heights; the lander's shadow falls to its south-west."""
import math

import cv2
import numpy as np

import gzrun

T = gzrun.T
out = T / "exp2"
out.mkdir(exist_ok=True)
X, Y = 49, 0
HEIGHTS = (40, 100, 200, 400, 590, 1500)
models = []
for h in HEIGHTS:
    hfov = max(0.1, 2 * math.atan(30 / h))
    models.append(gzrun.camera_model(f"tele{h}", pose=f"{X} {Y} {h} 0 1.5707963 0", static="true", hfov=hfov,
                                     near=max(0.1, h - 100), far=h + 100))
world = gzrun.make_world(T / "worlds/urc_equipment_servicing.sdf", T / "exp2_world.sdf", models)
name = gzrun.world_name(world)
node = gzrun.Node()
proc = gzrun.start(world, out / "server.log")
try:
    assert gzrun.wait_world(node, name), "world did not start"
    grabs = {h: gzrun.Grabber(node, f"/tele{h}/image") for h in HEIGHTS}
    for h, g in grabs.items():
        print(h, g.wait(3, 120), flush=True)
    tiles = []
    for h, g in grabs.items():
        im = cv2.cvtColor(g.frames[-1][2], cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out / f"tele{h}.png"), im)
        # Shadow darkness: darkest 1% of a box south-west of the lander vs the ground median.
        c = im[200:340, 400:560].astype(np.float32).mean(axis=2)
        print(f"h {h}: crop min {c.min():.0f}, p1 {np.percentile(c, 1):.0f}, median {np.median(c):.0f}", flush=True)
        t = cv2.resize(im[200:340, 400:560], (320, 280), interpolation=cv2.INTER_NEAREST)
        cv2.putText(t, f"{h} m", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        tiles.append(t)
    cv2.imwrite(str(out / "shadows.jpg"), np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:])]))
finally:
    gzrun.stop(proc)
