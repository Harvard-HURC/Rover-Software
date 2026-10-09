"""Experiment 6, several checks with the prototype plugin hooked on SceneUpdate:
  ortho  - the SceneUpdate hook can switch the camera to orthographic
  look   - per-frame rotation when 20 Hz look deltas (a dragged mouse) are
           applied at once vs smoothed (<look_tau>)
  pause  - does a camera sensor render while the world is paused?
  ground - the plugin's terrain height (heightmap from the ECM) vs the
           mission sheet's heightmap, at random points (clearance check)
usage: exp6_checks.py <world> <look_tau> <check>[,<check>...]"""
import json
import math
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

import gzrun
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.vector3d_pb2 import Vector3d

T = gzrun.T
world_key, LOOK_TAU, checks = sys.argv[1], float(sys.argv[2]), sys.argv[3].split(",")
out = T / "exp6"
out.mkdir(exist_ok=True)
START = "0 -60 30 0 1.5707 1.5707963"
plugin = ('<plugin filename="FlyCamProto" name="rover_sim::FlyCamProto">'
          f'<start>{START}</start><clearance>1.0</clearance><time_constant>0.2</time_constant>'
          f'<look_tau>{LOOK_TAU}</look_tau><hook_event>sceneupdate</hook_event></plugin>')
models = [gzrun.camera_model("flycam", pose=START, topic="/flycam/image", plugin=plugin)]
world = gzrun.make_world(T / "worlds" / f"{world_key}.sdf", T / f"exp6_{world_key}.sdf", models, strip_rover=True)
name = gzrun.world_name(world)
node = gzrun.Node()
state = {}
lock = threading.Lock()
node.subscribe(StringMsg, "/flycam/state", lambda m: (lock.acquire(), state.update(json.loads(m.data)), lock.release()))
proc = gzrun.start(world, out / f"server_{world_key}_{LOOK_TAU}.log")
results = {"world": world_key, "look_tau": LOOK_TAU}


def pose_msg(x, y, z, yaw, pitch):
    q = Pose()
    q.position.x, q.position.y, q.position.z = x, y, z
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    q.orientation.w, q.orientation.x, q.orientation.y, q.orientation.z = cp * cy, -sp * sy, sp * cy, cp * sy
    return q


def goto_and_wait(pub, x, y, z, yaw, pitch, tol=1e-3):
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        pub.publish(pose_msg(x, y, z, yaw, pitch))
        time.sleep(0.05)
        with lock:
            s = dict(state)
        if s and abs(s["x"] - x) < tol and abs(s["y"] - y) < tol:
            return s
    raise RuntimeError("goto not applied")


try:
    assert gzrun.wait_world(node, name, 300)
    grab = gzrun.Grabber(node, "/flycam/image")
    assert grab.wait(3, 300)
    goto = node.advertise("/flycam/goto", Pose)
    ortho = node.advertise("/flycam/ortho", Double)
    look = node.advertise("/flycam/look", Vector3d)
    time.sleep(1)

    if "ortho" in checks:
        sizes = {}
        for label, width, h in (("persp", 0.0, 40), ("persp", 0.0, 80), ("ortho", 60.0, 40), ("ortho", 60.0, 80)):
            m = Double(); m.data = width
            ortho.publish(m)
            s = goto_and_wait(goto, 49, 0, h, math.pi / 2, 1.5707)
            f = grab.latest_after(s["t"] + 0.3)
            img = f[2]
            # Lander size: dark-olive pixels (the lander top) around the image centre.
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            c = hsv[img.shape[0] // 2 - 150:img.shape[0] // 2 + 150, img.shape[1] // 2 - 150:img.shape[1] // 2 + 150]
            mask = (c[..., 2] < 120)
            sizes[f"{label}_{h}"] = int(mask.sum())
            cv2.imwrite(str(out / f"ortho_check_{label}_{h}.png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        results["ortho_dark_pixels"] = sizes
        m = Double(); m.data = 0.0
        ortho.publish(m)

    if "look" in checks:
        s = goto_and_wait(goto, 0, -60, 3.0, 0.0, 0.25)
        time.sleep(1.0)
        with grab.lock:
            grab.frames.clear()
        t0 = time.monotonic()
        nxt = t0
        while time.monotonic() - t0 < 6.0:  # a mouse drag: 0.05 rad per 50 ms page tick = 1 rad/s
            v = Vector3d(); v.x = 0.05
            look.publish(v)
            nxt += 0.05
            time.sleep(max(0, nxt - time.monotonic()))
        time.sleep(0.5)
        with grab.lock:
            frames = list(grab.frames)
        win = cv2.createHanningWindow(frames[0][2].shape[1::-1], cv2.CV_32F)
        g = [cv2.cvtColor(f[2], cv2.COLOR_RGB2GRAY).astype(np.float32) for f in frames]
        # The upper half shows the distant ground and sky edge: yaw moves it sideways.
        dx = [cv2.phaseCorrelate(a[:270], b[:270], win[:270])[0][0] for a, b in zip(g[:-1], g[1:])]
        dx = np.array(dx[20:-15])
        results["look"] = {"frames": len(dx), "dx_mean": float(dx.mean()), "dx_std": float(dx.std()),
                           "cv": float(dx.std() / abs(dx.mean())),
                           "stalls": int(np.sum(np.abs(dx) < 0.25 * abs(dx.mean()))),
                           "doubles": int(np.sum(np.abs(dx) > 1.6 * abs(dx.mean()))),
                           "sample": [round(float(v), 1) for v in dx[:30]]}

    if "pause" in checks:
        gz = str(gzrun.PREFIX / "bin/gz")
        req = lambda p: subprocess.run([gz, "service", "-s", f"/world/{name}/control", "--reqtype", "gz.msgs.WorldControl",
                                        "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f"pause: {p}"],
                                       capture_output=True, text=True).stdout.strip()
        print("pause:", req("true"), flush=True)
        time.sleep(1.0)
        n0 = len(grab.frames)
        time.sleep(3.0)
        paused_frames = len(grab.frames) - n0
        print("unpause:", req("false"), flush=True)
        time.sleep(1.0)
        n0 = len(grab.frames)
        time.sleep(3.0)
        results["pause"] = {"frames_in_3s_paused": paused_frames, "frames_in_3s_running": len(grab.frames) - n0}

    if "ground" in checks:
        sheet = json.loads((T / "worlds" / f"{world_key}.json").read_text())
        terrain = sheet["terrain"]
        size = float(terrain["size_m"])
        hm = cv2.imread(str((T / "worlds" / terrain["heightmap"])), cv2.IMREAD_UNCHANGED).astype(np.float64)
        hm = hm[..., 0] if hm.ndim == 3 else hm
        z = hm / 65535.0 * float(terrain["z_max"])
        nn = z.shape[0]

        def sheet_height(x, y):  # bilinear, row 0 north, column 0 west
            c = (x + size / 2) / size * (nn - 1)
            r = (size / 2 - y) / size * (nn - 1)
            c0, r0 = int(c), int(r)
            a, b = c - c0, r - r0
            return ((1 - a) * (1 - b) * z[r0, c0] + a * (1 - b) * z[r0, c0 + 1] + (1 - a) * b * z[r0 + 1, c0]
                    + a * b * z[r0 + 1, c0 + 1])

        rng = np.random.default_rng(1)
        errs, errs_flipped = [], []
        for x, y in rng.uniform(-0.45 * size, 0.45 * size, (12, 2)):
            s = goto_and_wait(goto, float(x), float(y), -500.0, 0.0, 0.3)
            time.sleep(0.15)
            with lock:
                s = dict(state)
            errs.append(s["ground"] - sheet_height(x, y))
            errs_flipped.append(s["ground"] - sheet_height(x, -y))
            assert abs(s["z"] - (s["ground"] + 1.0)) < 1e-3, s
        results["ground"] = {"max_abs_err_m": float(np.max(np.abs(errs))),
                             "max_abs_err_if_flipped_m": float(np.max(np.abs(errs_flipped))),
                             "z_range_m": float(z.max() - z.min())}
    print(json.dumps(results, indent=1))
    (out / f"result_{world_key}_{LOOK_TAU}_{'_'.join(checks)}.json").write_text(json.dumps(results, indent=1))
finally:
    gzrun.stop(proc)
