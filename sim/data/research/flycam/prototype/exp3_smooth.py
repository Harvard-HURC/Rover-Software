"""Experiment 3: how smooth is a flying camera when (A) the plugin integrates
a velocity command every physics step, vs (B) Python moves the model with the
/world/<w>/set_pose service at a fixed wall-clock rate?

The camera looks straight down (north up) from 25 m and flies north at
4 m/s; consecutive frames are phase-correlated, so the measured image shift
per frame is what a viewer sees. Smooth = the same shift every frame."""
import json
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

import gzrun
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.world_stats_pb2 import WorldStatistics

T = gzrun.T
out = T / "exp3"
out.mkdir(exist_ok=True)
SPEED = 4.0
X0, Y0, Z0 = 0.0, -60.0, 26.0
DURATION = 12.0
REQ_TIMEOUT = 2  # [ms] fire and forget: a blocking request deadlocks with Python subscribers until it times out
mode = sys.argv[1]  # plugin | setpose20 | setpose60
rate = {"plugin": 20, "setpose20": 20, "setpose60": 60}[mode]
q = (np.cos(np.pi / 4), 0, 0, 0)  # placeholder
POSE = f"{X0} {Y0} {Z0} 0 1.5707963 1.5707963"
if mode == "plugin":
    plugin = ('<plugin filename="FlyCamProto" name="rover_sim::FlyCamProto">'
              f'<start>{POSE}</start><clearance>1.0</clearance><time_constant>0.2</time_constant></plugin>')
else:
    plugin = ""
models = [gzrun.camera_model("cam", pose=POSE, topic="/cam/image", plugin=plugin)]
world = gzrun.make_world(T / "worlds/urc_equipment_servicing.sdf", T / f"exp3_{mode}.sdf", models)
name = gzrun.world_name(world)
node = gzrun.Node()
proc = gzrun.start(world, out / f"server_{mode}.log")
stats = []
node.subscribe(WorldStatistics, f"/world/{name}/stats",
               lambda m: stats.append((time.monotonic(), m.real_time_factor, m.sim_time.sec + m.sim_time.nsec * 1e-9)))
try:
    assert gzrun.wait_world(node, name), "world did not start"
    grab = gzrun.Grabber(node, "/cam/image")
    assert grab.wait(5, 120), "no frames"
    time.sleep(2.0)
    with grab.lock:
        grab.frames.clear()
    latencies = []
    cpu = []
    t_start = time.monotonic()
    next_t = t_start
    if mode == "plugin":
        pub = node.advertise("/flycam/cmd", Twist)
        time.sleep(0.5)
        t_start = next_t = time.monotonic()
    pose_msg = Pose()
    pose_msg.name = "cam"
    # RPY(0, pi/2, pi/2) as a quaternion (w, x, y, z)
    from math import cos, sin
    cr, sr, cp, sp, cy, sy = 1, 0, cos(np.pi / 4), sin(np.pi / 4), cos(np.pi / 4), sin(np.pi / 4)
    pose_msg.orientation.w = cr * cp * cy + sr * sp * sy
    pose_msg.orientation.x = sr * cp * cy - cr * sp * sy
    pose_msg.orientation.y = cr * sp * cy + sr * cp * sy
    pose_msg.orientation.z = cr * cp * sy - sr * sp * cy
    while time.monotonic() - t_start < DURATION:
        now = time.monotonic()
        if mode == "plugin":
            tw = Twist()
            tw.linear.x = SPEED
            pub.publish(tw)
        else:
            pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = X0, Y0 + SPEED * (now - t_start), Z0
            t0 = time.perf_counter()
            ok, rep = node.request(f"/world/{name}/set_pose", pose_msg, Pose, Boolean, REQ_TIMEOUT)
            latencies.append((time.perf_counter() - t0) * 1000)
        if int((now - t_start) * 2) != int((now - t_start - 1 / rate) * 2):
            try:
                cpu.append(float(subprocess.run(["ps", "-o", "%cpu=", "-p", str(proc.pid)], capture_output=True,
                                                text=True).stdout.strip() or 0))
            except ValueError:
                pass
        next_t += 1 / rate
        time.sleep(max(0, next_t - time.monotonic()))
    time.sleep(0.5)
    with grab.lock:
        frames = list(grab.frames)
    # Phase correlation between consecutive frames (grayscale, Hanning window).
    win = cv2.createHanningWindow(frames[0][2].shape[1::-1], cv2.CV_32F)
    shifts, dstamp, dwall = [], [], []
    prev = None
    for stamp, wall, rgb in frames:
        g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)
        if prev is not None:
            (dx, dy), resp = cv2.phaseCorrelate(prev[2], g, win)
            shifts.append((dx, dy, resp))
            dstamp.append(stamp - prev[0])
            dwall.append(wall - prev[1])
        prev = (stamp, wall, g)
    s = np.array(shifts)
    # Use the middle (steady) part: skip the first 1.5 s of acceleration.
    steady = s[30:-5] if len(s) > 60 else s
    dy = steady[:, 1]
    rtf = [r for t, r, _ in stats if t > t_start]
    result = {
        "mode": mode, "frames": len(frames), "rtf_mean": float(np.mean(rtf)) if rtf else None,
        "shift_px_mean": float(np.mean(dy)), "shift_px_std": float(np.std(dy)),
        "cv": float(np.std(dy) / abs(np.mean(dy))) if np.mean(dy) else None,
        "zero_frames": int(np.sum(np.abs(dy) < 0.25 * abs(np.mean(dy)))),
        "double_frames": int(np.sum(np.abs(dy) > 1.6 * abs(np.mean(dy)))),
        "steady_n": int(len(dy)), "dx_std": float(np.std(steady[:, 0])),
        "stamp_dt_ms": [float(np.mean(dstamp) * 1000), float(np.std(dstamp) * 1000)],
        "wall_dt_ms": [float(np.mean(dwall) * 1000), float(np.std(dwall) * 1000)],
        "setpose_latency_ms": [float(np.median(latencies)), float(np.percentile(latencies, 95)),
                               float(np.max(latencies))] if latencies else None,
        "server_cpu_pct": float(np.median(cpu)) if cpu else None,
        "shifts_first40": [round(float(v), 2) for v in s[:40, 1]],
        "shifts_steady_sample": [round(float(v), 2) for v in dy[:40]],
    }
    print(json.dumps(result, indent=1))
    (out / f"result_{mode}.json").write_text(json.dumps(result, indent=1))
    cv2.imwrite(str(out / f"frame_{mode}.png"), cv2.cvtColor(frames[len(frames) // 2][2], cv2.COLOR_RGB2BGR))
finally:
    gzrun.stop(proc)
