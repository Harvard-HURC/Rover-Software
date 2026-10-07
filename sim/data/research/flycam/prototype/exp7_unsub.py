"""Does a camera sensor cost anything while nobody subscribes to it?
Unthrottled sim speed (sim s per wall s) over 12 s: no extra camera, a
1280x720 20 Hz camera nobody watches, the same camera watched."""
import json, subprocess, sys, time
import numpy as np
import gzrun
from gz.msgs10.world_stats_pb2 import WorldStatistics
T = gzrun.T
config = sys.argv[1]
models = [] if config == "none" else [gzrun.camera_model("cam", pose="0 -60 30 0 0.6 1.57", topic="/cam/image",
                                                         w=1280, h=720, static="true")]
world = gzrun.make_world(T / "worlds/urc_equipment_servicing.sdf", T / f"exp7_{config}.sdf", models, rtf="0")
name = gzrun.world_name(world)
node = gzrun.Node()
stats = []
node.subscribe(WorldStatistics, f"/world/{name}/stats", lambda m: stats.append((time.monotonic(), m.sim_time.sec + m.sim_time.nsec * 1e-9)))
proc = gzrun.start(world, T / f"exp7_{config}.log")
try:
    assert gzrun.wait_world(node, name)
    if config == "sub":
        g = gzrun.Grabber(node, "/cam/image", keep=False)
        assert g.wait(3, 60)
    time.sleep(4)
    t0 = time.monotonic()
    time.sleep(12)
    s = [x for x in stats if x[0] >= t0]
    print(json.dumps({"config": config, "sim_per_wall": round((s[-1][1] - s[0][1]) / (s[-1][0] - s[0][0]), 3)}))
finally:
    gzrun.stop(proc)
