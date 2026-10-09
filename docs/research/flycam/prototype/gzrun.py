"""Helpers for the flycam research prototypes: start a world headless in a
private partition, grab camera frames, stop it."""
import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path

T = Path(__file__).resolve().parent
ROVER = Path("/Users/alarion239/Desktop/Rover")
PREFIX = ROVER / ".pixi/envs/default"
PARTITION = f"flycam_research_{os.getpid()}"
os.environ.update({
    "GZ_SIM_RESOURCE_PATH": str(T / "models"),
    "GZ_SIM_SYSTEM_PLUGIN_PATH": f"{T / 'pbuild'}:{T / 'build'}",
    "OGRE2_RESOURCE_PATH": str(PREFIX / "lib/OGRE-Next"),
    "OGRE_RESOURCE_PATH": str(PREFIX / "lib/OGRE"),
    "GZ_IP": "127.0.0.1",
    "GZ_PARTITION": PARTITION,
})

import numpy as np  # noqa: E402
from gz.transport13 import Node  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402

CAMERA_MODEL = """
    <model name="{name}">
      <pose>{pose}</pose>
      <static>{static}</static>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>0.1</mass><inertia><ixx>1e-3</ixx><iyy>1e-3</iyy><izz>1e-3</izz></inertia></inertial>
        <sensor name="camera" type="camera">
          <always_on>true</always_on>
          <update_rate>{rate}</update_rate>
          <topic>{topic}</topic>
          <camera>
            {extra}
            <horizontal_fov>{hfov}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>{near}</near><far>{far}</far></clip>
          </camera>
        </sensor>
      </link>
      {plugin}
    </model>
"""


def camera_model(name, pose="0 0 40 0 1.5707963 0", static="false", rate=20, topic=None, extra="", hfov=1.2,
                 w=960, h=540, near=0.1, far=3000, plugin=""):
    return CAMERA_MODEL.format(name=name, pose=pose, static=static, rate=rate, topic=topic or f"/{name}/image",
                               extra=extra, hfov=hfov, w=w, h=h, near=near, far=far, plugin=plugin)


def make_world(src, dst, models, strip_rover=False, rtf=None):
    text = Path(src).read_text()
    if strip_rover:
        text = re.sub(r"<include>\s*<uri>model://rover</uri>.*?</include>", "", text, flags=re.S)
    if rtf is not None:
        text = re.sub(r"<real_time_factor>[^<]*</real_time_factor>", f"<real_time_factor>{rtf}</real_time_factor>",
                      text, count=1)
    text = text.replace("</world>", "".join(models) + "\n  </world>")
    Path(dst).write_text(text)
    return Path(dst)


def world_name(path):
    return re.search(r'<world name="([^"]+)"', Path(path).read_text()).group(1)


def start(world, log):
    gz = str(PREFIX / "bin/gz")
    proc = subprocess.Popen([gz, "sim", "-s", "-r", "-v", "3", str(world)], stdout=open(log, "w"),
                            stderr=subprocess.STDOUT, start_new_session=True)
    return proc


def wait_world(node, name, timeout=180):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if f"/world/{name}/clock" in node.topic_list():
            return True
        time.sleep(0.25)
    return False


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


class Grabber:
    """Keeps every frame (stamp, wall time, rgb array) of an image topic."""

    def __init__(self, node, topic, keep=True):
        self.frames = []
        self.lock = threading.Lock()
        self.keep = keep
        self.topic = topic
        node.subscribe(Image, topic, self.on)

    def on(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy() if self.keep else None
        with self.lock:
            self.frames.append((stamp, time.monotonic(), rgb))

    def wait(self, n, timeout=60):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if len(self.frames) >= n:
                    return True
            time.sleep(0.05)
        return False

    def latest_after(self, stamp, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                for f in self.frames:
                    if f[0] > stamp:
                        return f
            time.sleep(0.02)
        return None
