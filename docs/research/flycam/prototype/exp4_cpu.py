"""Experiment 4: what does a camera that the plugin moves every physics step
cost? Server CPU and real-time factor over 15 s with RTF unthrottled (0) so
the step rate shows the cost directly, for:
  static   - a static camera (nothing moves)
  idle     - the plugin camera, no command (it still sets its pose every step)
  flying   - the plugin camera flying at 4 m/s
A subscriber keeps the camera rendering in every case."""
import json
import subprocess
import sys
import time

import numpy as np

import gzrun
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.world_stats_pb2 import WorldStatistics

T = gzrun.T
out = T / "exp4"
out.mkdir(exist_ok=True)
POSE = "0 -60 26 0 1.5707963 1.5707963"
config = sys.argv[1]
rtf_target = sys.argv[2] if len(sys.argv) > 2 else "0"
# config: static | dynamic (free body, no plugin) | <method>_<idle|flying>[_onchange]
parts = config.split("_")
if config in ("static", "dynamic"):
    plugin = ""
else:
    plugin = ('<plugin filename="FlyCamProto" name="rover_sim::FlyCamProto">'
              f'<start>{POSE}</start><method>{parts[0]}</method>'
              f'<only_on_change>{"true" if "onchange" in parts else "false"}</only_on_change>'
              f'<ortho_hook>{"false" if "nohook" in parts else "true"}</ortho_hook>'
              f'<hook_event>{next((x.split("-")[1] for x in parts if x.startswith("hook-")), "prerender")}</hook_event>'
              '</plugin>')
static = "true" if config == "static" or parts[0] == "pose" else "false"
models = [gzrun.camera_model("cam", pose=POSE, topic="/cam/image", plugin=plugin, static=static)]
world = gzrun.make_world(T / "worlds/urc_equipment_servicing.sdf", T / f"exp4_{config}.sdf", models, rtf=rtf_target)
name = gzrun.world_name(world)
node = gzrun.Node()
proc = gzrun.start(world, out / f"server_{config}.log")
stats = []
node.subscribe(WorldStatistics, f"/world/{name}/stats",
               lambda m: stats.append((time.monotonic(), m.real_time_factor, m.sim_time.sec + m.sim_time.nsec * 1e-9,
                                       m.iterations)))
try:
    assert gzrun.wait_world(node, name)
    grab = gzrun.Grabber(node, "/cam/image", keep=False)
    assert grab.wait(5, 120)
    pub = node.advertise("/flycam/cmd", Twist)
    time.sleep(3)
    t0 = time.monotonic()
    cpu = []
    while time.monotonic() - t0 < 15:
        if "flying" in config:
            tw = Twist()
            tw.linear.x = 4.0 if int(time.monotonic() - t0) % 6 < 3 else -4.0  # back and forth over the same ground
            pub.publish(tw)
        r = subprocess.run(["ps", "-o", "%cpu=", "-p", str(proc.pid)], capture_output=True, text=True).stdout.strip()
        if r:
            cpu.append(float(r))
        time.sleep(0.05)
    s = [x for x in stats if x[0] >= t0]
    sim_rate = (s[-1][2] - s[0][2]) / (s[-1][0] - s[0][0])
    frames = [f for f in grab.frames if f[1] >= t0]
    result = {"config": config, "rtf_target": rtf_target, "sim_seconds_per_wall_second": round(sim_rate, 3),
              "server_cpu_pct_median": float(np.median(cpu)), "frames_per_wall_s": len(frames) / 15}
    print(json.dumps(result))
finally:
    gzrun.stop(proc)
