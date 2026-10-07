"""Headless camera renders of a Gazebo world (one process per call: ogre2 starts once per process).

python render.py config.json

config: {"world": path to an SDF, "out": dir, "seconds": sim seconds, "views": [{"name", "pose": [x y z r p y],
          "size": [w, h], "hfov", "far", "rate", "extra": sensor xml}], "rtf0": bool, "strip_rover_camera": bool}
Writes <out>/<name>.png (last frame) and prints a JSON line "RESULT {...}" with timings.
"""
import json
import os
import re
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
os.environ.setdefault("GZ_SIM_RESOURCE_PATH", str(HERE / "models"))
os.environ.setdefault("GZ_SIM_SYSTEM_PLUGIN_PATH", str(HERE / "build"))
os.environ.setdefault("GZ_PARTITION", f"vis_{os.getpid()}")
os.environ.setdefault("GZ_IP", "127.0.0.1")
os.environ.setdefault("OGRE2_RESOURCE_PATH", str(Path(sys.prefix) / "lib" / "OGRE-Next"))

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

import gz.math7  # noqa: E402,F401
from gz.msgs10.image_pb2 import Image as ImageMsg  # noqa: E402
from gz.sim8 import TestFixture  # noqa: E402
from gz.transport13 import Node  # noqa: E402


def camera_model(view):
    name = view["name"]
    w, h = view.get("size", (960, 540))
    pose = " ".join(str(v) for v in view["pose"])
    return f"""
    <model name="viscam_{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <always_on>true</always_on>
          <update_rate>{view.get("rate", 5)}</update_rate>
          <topic>/viscam/{name}</topic>
          <camera>
            <horizontal_fov>{view.get("hfov", 1.2)}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>{view.get("near", 0.1)}</near><far>{view.get("far", 3000)}</far></clip>
            {view.get("extra", "")}
          </camera>
          {view.get("sensor_extra", "")}
        </sensor>
      </link>
    </model>"""


def main(cfg_path):
    cfg = json.loads(Path(cfg_path).read_text())
    text = Path(cfg["world"]).read_text()
    if cfg.get("rtf0", True):
        text = re.sub(r"<real_time_factor>[^<]*</real_time_factor>", "<real_time_factor>0</real_time_factor>", text)
    for pattern, repl in cfg.get("replace", []):
        text, n = re.subn(pattern, repl, text, flags=re.S)
        if n == 0:
            print(f"WARN replace pattern not found: {pattern[:80]}", flush=True)
    cams = "".join(camera_model(v) for v in cfg["views"])
    text = text.replace("</world>", cams + cfg.get("extra_world", "") + "\n  </world>")
    out = Path(cfg["out"])
    out.mkdir(parents=True, exist_ok=True)
    world = tempfile.NamedTemporaryFile("w", suffix=".sdf", dir=HERE / "worlds", delete=False)
    world.write(text)
    world.close()
    node = Node()
    frames = {v["name"]: [] for v in cfg["views"]}
    stamps = {v["name"]: [] for v in cfg["views"]}
    for v in cfg["views"]:
        def cb(msg, name=v["name"]):
            frames[name][:] = [msg]
            stamps[name].append(time.perf_counter())
        node.subscribe(ImageMsg, f"/viscam/{v['name']}", cb)
    t0 = time.perf_counter()
    fixture = TestFixture(world.name)
    fixture.finalize()
    t1 = time.perf_counter()
    steps = round(cfg.get("seconds", 1.0) * 1000)
    fixture.server().run(True, steps, False)
    t2 = time.perf_counter()
    time.sleep(0.5)
    os.unlink(world.name)
    result = {"load_s": round(t1 - t0, 2), "run_s": round(t2 - t1, 2), "steps": steps, "frames": {}}
    for name, msgs in frames.items():
        result["frames"][name] = len(stamps[name])
        if not msgs:
            print(f"WARN no frame from {name}", flush=True)
            continue
        m = msgs[-1]
        rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        Image.fromarray(rgb).save(out / f"{name}.png")
        s = stamps[name]
        if len(s) > 4:
            dt = np.diff(s[2:])
            result.setdefault("frame_dt_ms", {})[name] = round(float(np.median(dt)) * 1000, 2)
    print("RESULT " + json.dumps(result), flush=True)


if __name__ == "__main__":
    main(sys.argv[1])
