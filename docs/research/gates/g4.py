"""G4: do alpha textures from SDF cut out (shrub cards) and feather (decal edges) in ogre2 on Metal?

Three 1 m quads in front of an emissive blue wall, seen by one camera: a red disc on transparent green
texels (a cut-out shows blue around the disc), a red texture whose alpha ramps 0 -> 1 across the quad
(feathering shows a red-to-blue gradient, an alpha test a hard edge), and the same ramp with SDF
<transparency> 0.001, the one blending switch SDF has."""
import json
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

D = G.SCRATCH / "g4"
D.mkdir(exist_ok=True)
n = 256
yy, xx = np.mgrid[0:n, 0:n]
disc = np.zeros((n, n, 4), np.uint8)  # BGRA for cv2
disc[..., 1] = 255  # transparent texels are green: a missing cut-out shows green
inside = (xx - n / 2) ** 2 + (yy - n / 2) ** 2 < (0.35 * n) ** 2
disc[inside] = (0, 0, 255, 255)  # opaque red disc
disc[~inside, 3] = 0
cv2.imwrite(str(D / "disc.png"), disc)
ramp = np.zeros((n, n, 4), np.uint8)
ramp[..., 2] = 255  # red
ramp[..., 3] = np.round(xx / (n - 1) * 255).astype(np.uint8)  # alpha 0 -> 1 along u
cv2.imwrite(str(D / "ramp.png"), ramp)


def quad(name, y, z, texture, transparency=None):
    extra = f"<transparency>{transparency}</transparency>" if transparency is not None else ""
    return f"""<model name="{name}"><static>true</static><pose>4 {y} {z} 0 0 0</pose><link name="l">
  <visual name="v"><geometry><plane><normal>-1 0 0</normal><size>1 1</size></plane></geometry>{extra}
    <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
      <pbr><metal><albedo_map>{texture}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr>
    </material></visual></link></model>"""


world = f"""<?xml version="1.0"?>
<sdf version="1.11"><world name="g4">
  <physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <scene><ambient>0.6 0.6 0.6 1</ambient><background>0 0 0 1</background><grid>false</grid></scene>
  <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><diffuse>1 1 1 1</diffuse><direction>1 0 -0.3</direction></light>
  <model name="backdrop"><static>true</static><pose>8 0 1 0 0 0</pose><link name="l"><visual name="v">
    <geometry><box><size>0.2 20 20</size></box></geometry>
    <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse><emissive>0 0 0.8 1</emissive></material></visual></link></model>
  {quad("cutout", 0.8, 1.0, D / "disc.png")}
  {quad("feather", -0.8, 1.0, D / "ramp.png")}
  {quad("feather_blend", 0.0, 0.0, D / "ramp.png", transparency=0.001)}
  <model name="cam"><static>true</static><pose>0 0 1 0 0 0</pose><link name="l">
    <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>10</update_rate><topic>/g4/camera</topic>
      <camera><horizontal_fov>1.0</horizontal_fov><image><width>640</width><height>480</height></image>
      <clip><near>0.1</near><far>100</far></clip></camera></sensor></link></model>
</world></sdf>"""
path = D / "g4.sdf"
path.write_text(world)
out = D / "g4.png"
r = subprocess.run([sys.executable, str(Path(__file__).with_name("render_once.py")), str(path), "/g4/camera", str(out), "1.5"],
                   capture_output=True, text=True, timeout=300)
print(r.stdout[-500:], r.stderr[-1500:] if r.returncode else "")
img = cv2.imread(str(out))[..., ::-1].astype(int)  # RGB
f = 320 / np.tan(0.5)


def px(y, z):
    return int(round(240 - f * (z - 1) / 4)), int(round(320 - f * y / 4))


def patch(y, z, r=2):
    v, u = px(y, z)
    return img[v - r:v + r + 1, u - r:u + r + 1].reshape(-1, 3).mean(axis=0).round(1).tolist()


result = {"backdrop": patch(0.0, 2.2), "cutout_disc_centre": patch(0.8, 1.0),
          "cutout_corner_transparent": patch(1.22, 1.42), "cutout_corner_2": patch(0.38, 0.58)}
green = result["cutout_corner_transparent"]
result["cutout_works"] = bool(green[2] > green[1] and green[2] > 60)


def ramp_profile(y, z):
    """The quad's red fraction r / (r + b), averaged across and profiled along whichever screen axis
    it varies on (the texture's u axis lands on a screen axis by the plane's orientation)."""
    v0, u0 = px(y + 0.42, z + 0.42)
    v1, u1 = px(y - 0.42, z - 0.42)
    block = img[min(v0, v1):max(v0, v1), min(u0, u1):max(u0, u1)]
    frac = block[..., 0] / np.maximum(block[..., 0] + block[..., 2], 1)
    profiles = [frac.mean(axis=0), frac.mean(axis=1)]
    profile = max(profiles, key=lambda p: p.max() - p.min())
    steps = np.abs(np.diff(profile))
    return dict(profile=profile[:: max(1, len(profile) // 24)].round(2).tolist(), max_step=round(float(steps.max()), 3),
                levels=int(len(np.unique(np.round(profile, 1)))),
                feathered=bool(len(np.unique(np.round(profile, 1))) >= 5 and steps.max() < 0.3))


result["feather"] = ramp_profile(-0.8, 1.0)
result["feather_with_transparency"] = ramp_profile(0.0, 0.0)
print(json.dumps(result, indent=1))
G.save("g4", result)
