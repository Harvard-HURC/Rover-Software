"""G6: driver-station frame rate with full-config render loads, in Delivery and Equipment Servicing.

The full render config (WS-A, WS-T2) does not exist yet, so its loads are stood in for with the spec's
budgets (section 7, D10, D11, D14): 1.48 M shrub triangles within 400 m and 0.8 M pebble triangles
within 35 m of the rover (merged GLB chunks), a far-field grid 65 x 80 km at 120 m, a 4096^2 colour
map as terrain layer 0, cameras clipped at 80 km. The rover is the drivetrain prototype (physics cost
sets RTF); eye (960x540) and fly (1280x720, FlyCamProto, SceneUpdate hook) cameras watched at 20 Hz,
the rover's RGB-D subscribed (image and depth). RTF 1, the rover driving.

Usage: python g6.py <world> <variant> [seconds]
variant: rgbd1280 | rgbd640 | chase | chase_flare | fallback, and <variant>_dd for the same with today's
DiffDrive rover (a cheaper physics step: what the render alone allows). The rover's prototype takes its
wheel loads from G6_LOAD_SOURCE (default joint, gate G8's pick). The server's output goes through a
pseudo-terminal, so its messages (LensFlare's) reach the log line by line.
"""
import json
import math
import os
import pty
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402
import glb  # noqa: E402
from urc import sheet as sheets  # noqa: E402

world_key, variant = sys.argv[1], sys.argv[2]
diffdrive = variant.endswith("_dd")
base_variant = variant.removesuffix("_dd")
seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
D = G.SCRATCH / "g6"
MODELS = D / "models"
MODELS.mkdir(parents=True, exist_ok=True)
FLY_BUILD = G.FLY_BUILD
sheet_path = G.REPO / "sim" / "worlds" / f"{world_key}.json"
sheet = sheets.load(sheet_path)
hf = sheets.terrain(sheet, sheet_path)
start = sheet["rover_start"]
x0, y0, z0, yaw0 = start["x"], start["y"], start["z"], start["yaw"]
half = hf.size / 2
rng = np.random.default_rng(6)


def write_model(name, links_xml):
    d = MODELS / name
    d.mkdir(exist_ok=True)
    (d / "model.sdf").write_text(f'<?xml version="1.0"?><sdf version="1.11"><model name="{name}"><static>true</static>'
                                 f'{links_xml}</model></sdf>')
    (d / "model.config").write_text(f'<?xml version="1.0"?><model><name>{name}</name><version>1</version>'
                                    '<sdf version="1.11">model.sdf</sdf></model>')


def scatter_chunks(name, count, radius, unit, scale, chunks, color):
    V0, F0 = unit
    xy = []
    while len(xy) < count:
        r, a = radius * math.sqrt(rng.uniform()), rng.uniform(0, 2 * math.pi)
        x, y = x0 + r * math.cos(a), y0 + r * math.sin(a)
        if abs(x) < half - 1 and abs(y) < half - 1:
            xy.append((x, y))
    visuals = []
    per = int(math.ceil(count / chunks))
    for c in range(chunks):
        parts, faces, off = [], [], 0
        for x, y in xy[c * per:(c + 1) * per]:
            s = scale * rng.uniform(0.6, 1.4)
            parts.append(V0 * [s, s, 0.6 * s] + [x, y, hf.height(x, y)])
            faces.append(F0 + off)
            off += len(V0)
        path = MODELS / name / f"{name}_{c}.glb"
        path.parent.mkdir(exist_ok=True)
        glb.write_glb(path, np.concatenate(parts), np.concatenate(faces), color=color)
        visuals.append(f'<visual name="{name}_{c}"><geometry><mesh><uri>{path}</uri></mesh></geometry></visual>')
    return len(xy) * len(F0)


fallback = base_variant == "fallback"  # the spec's G6 fallback: fewer shrubs, no pebbles, fly 960x540
key = f"{world_key}_fallback" if fallback else f"{world_key}"
assets = MODELS / f"g6_assets_{key}"
if not (assets / "model.sdf").exists():
    sphere1 = glb.icosphere(1)  # 80 triangles
    shrub = (np.concatenate([sphere1[0], sphere1[0] * 0.7 + [0.25, 0.1, 0.1]]),
             np.concatenate([sphere1[1], sphere1[1] + len(sphere1[0])]))  # 160 triangles
    sphere0 = glb.icosphere(0)  # 20 triangles
    pebble = (np.concatenate([sphere0[0], sphere0[0] * 0.8 + [0.5, 0, 0]]),
              np.concatenate([sphere0[1], sphere0[1] + len(sphere0[0])]))  # 40 triangles
    tris = scatter_chunks(f"g6_shrubs_{key}", 4604 if fallback else 9208, 400.0, shrub, 0.35, 16, (0.42, 0.47, 0.33, 1))
    tris_p = 0 if fallback else scatter_chunks(f"g6_pebbles_{key}", 20000, 35.0, pebble, 0.03, 4, (0.55, 0.47, 0.40, 1))
    # far field: 65 x 80 km, 120 m grid, 4 m below the terrain's lowest point, a hole under the terrain
    xs, ys = np.arange(-32500, 32501, 120.0), np.arange(-40000, 40001, 120.0)
    X, Y = np.meshgrid(xs, ys)
    V = np.stack([X.ravel(), Y.ravel(), np.full(X.size, -4.0)], axis=1)
    nx = len(xs)
    i, j = np.meshgrid(np.arange(len(ys) - 1), np.arange(nx - 1), indexing="ij")
    a = (i * nx + j).ravel()
    keep = ~((np.abs(X[:-1, :-1].ravel()) < half) & (np.abs(Y[:-1, :-1].ravel()) < half))
    a = a[keep]
    F = np.concatenate([np.stack([a, a + 1, a + nx + 1], 1), np.stack([a, a + nx + 1, a + nx], 1)])
    far = assets / "far.glb"
    assets.mkdir(exist_ok=True)
    glb.write_glb(far, V, F, color=(0.75, 0.62, 0.5, 1))
    links = []
    for group in (f"g6_shrubs_{key}", f"g6_pebbles_{key}"):
        for p in sorted((MODELS / group).glob("*.glb")):
            links.append(f'<visual name="{p.stem}"><geometry><mesh><uri>{p}</uri></mesh></geometry></visual>')
    links.append(f'<visual name="far"><cast_shadows>false</cast_shadows><geometry><mesh><uri>{far}</uri></mesh></geometry></visual>')
    write_model(f"g6_assets_{key}", '<link name="l">' + "".join(links) + "</link>")
    (assets / "counts.json").write_text(json.dumps(dict(shrub_tris=tris, pebble_tris=tris_p, far_tris=len(F))))

# terrain copy with a 4096^2 colour map as layer 0 (size = the terrain)
terrain_name = re.search(r"model://(urc_terrain_\w+)", (G.REPO / "sim/worlds" / f"{world_key}.sdf").read_text()).group(1)
tcopy = MODELS / f"{terrain_name}_g6"
if not tcopy.exists():
    shutil.copytree(G.REPO / "sim/models" / terrain_name, tcopy, ignore=shutil.ignore_patterns("dem.tif"))
    noise = rng.integers(120, 200, (512, 512, 3), dtype=np.uint8)
    PILImage.fromarray(noise).resize((4096, 4096), PILImage.BILINEAR).save(tcopy / "colour_map.png")
    text = (tcopy / "model.sdf").read_text().replace(f"model://{terrain_name}/", f"model://{terrain_name}_g6/")
    text = text.replace(f'<model name="{terrain_name}">', f'<model name="{terrain_name}_g6">')
    first = re.search(r"<texture>\s*<diffuse>[^<]*</diffuse>\s*<normal>[^<]*</normal>\s*<size>[^<]*</size>", text)
    text = text.replace(first.group(0), f"<texture><diffuse>model://{terrain_name}_g6/colour_map.png</diffuse>"
                        f"<normal>{re.search(r'<normal>([^<]*)</normal>', first.group(0)).group(1)}</normal>"
                        f"<size>{hf.size}</size>", 1)
    (tcopy / "model.sdf").write_text(text)
    (tcopy / "model.config").write_text((tcopy / "model.config").read_text().replace(terrain_name, f"{terrain_name}_g6"))

# rover: drivetrain prototype, RGB-D at the variant's size, RGB clip 80 km, depth 40 m
rgbd = (640, 480) if base_variant == "rgbd640" else (1280, 720)
uri = G.proto_rover(f"g6_{variant}", load_source=os.environ.get("G6_LOAD_SOURCE", "joint"))
rover_sdf = Path(uri.removeprefix("file://")) / "model.sdf"
text = G.gen_model.build_sdf(G.gen_model.Params()) if diffdrive else rover_sdf.read_text()
cam = re.search(r'<sensor name="camera" type="rgbd_camera">.*?</sensor>', text, re.S).group(0)
new = re.sub(r"<width>\d+</width>", f"<width>{rgbd[0]}</width>", cam)
new = re.sub(r"<height>\d+</height>", f"<height>{rgbd[1]}</height>", new)
new = new.replace("<far>40</far>", "<far>80000</far>")
new = new.replace("</camera>", "<depth_camera><clip><near>0.1</near><far>40</far></clip></depth_camera></camera>")
rover_sdf.write_text(text.replace(cam, new))

# viewers
eye = (G.REPO / "sim/models/eye_camera/model.sdf").read_text().replace("<far>2000</far>", "<far>80000</far>")
eye = eye.split("\n", 1)[1].replace('<sdf version="1.11">', "").replace("</sdf>", "").replace(
    '<model name="eye_camera">', '<model name="eye_camera"><pose>0 0 -100 0 0 0</pose>')
chase = ""
if base_variant.startswith("chase"):
    chase = (G.REPO / "sim/models/chase_camera/model.sdf").read_text().replace("<far>2000</far>", "<far>80000</far>")
    chase = chase.split("\n", 1)[1].replace('<sdf version="1.11">', "").replace("</sdf>", "").replace(
        '<model name="chase_camera">', '<model name="chase_camera"><pose>0 0 -100 0 0 0</pose>')
    if base_variant == "chase_flare":
        chase = chase.replace("</camera>", "</camera><plugin filename=\"gz-sim-lens-flare-system\" "
                              "name=\"gz::sim::systems::LensFlare\"><scale>0.6</scale><color>1.0 0.95 0.9</color></plugin>")
c, s = math.cos(yaw0), math.sin(yaw0)
fly_pose = f"{x0 - 6 * c} {y0 - 6 * s} {z0 + 3} 0 0.4 {yaw0}"
fly_model = f"""<model name="fly_camera"><pose>{fly_pose}</pose><link name="link"><gravity>false</gravity>
  <inertial><mass>0.1</mass><inertia><ixx>1e-3</ixx><iyy>1e-3</iyy><izz>1e-3</izz></inertia></inertial>
  <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>20</update_rate><topic>/fly_camera/image</topic>
    <camera><horizontal_fov>1.2</horizontal_fov><image><width>{960 if fallback else 1280}</width><height>{540 if fallback else 720}</height><format>R8G8B8</format></image>
    <clip><near>0.1</near><far>80000</far></clip></camera></sensor></link>
  <plugin filename="FlyCamProto" name="rover_sim::FlyCamProto"><start>{fly_pose}</start><hook_event>sceneupdate</hook_event></plugin>
</model>"""
src = (G.REPO / "sim/worlds" / f"{world_key}.sdf").read_text()
src = src.replace(f"<uri>model://{terrain_name}</uri>", f"<uri>model://{terrain_name}_g6</uri>")
src = G.S.variant_sdf(src, rover_uri=uri)
src = src.replace("</world>", f"<include><uri>model://g6_assets_{key}</uri><name>g6_assets</name><pose>0 0 0 0 0 0</pose></include>"
                  f"{eye}{chase}{fly_model}</world>", 1)
world = D / f"g6_{world_key}_{variant}.sdf"
world.write_text(src)

env = G.env_for(models_dir=MODELS, plugin_dirs=[G.PROTO_BUILD, FLY_BUILD])
log = D / f"g6_{world_key}_{variant}.log"
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.msgs10.world_stats_pb2 import WorldStatistics  # noqa: E402
from gz.transport13 import Node  # noqa: E402

topics = ["/eye_camera/image", "/fly_camera/image", "/model/rover/camera/image", "/model/rover/camera/depth_image"]
if base_variant.startswith("chase"):
    topics.append("/chase_camera/image")
stamps = {t: [] for t in topics}
rtf = []
node = Node()
for t in topics:
    node.subscribe(Image, t, lambda m, t=t: stamps[t].append(time.monotonic()))
name = G.S.world_name(world)
node.subscribe(WorldStatistics, f"/world/{name}/stats", lambda m: rtf.append((time.monotonic(), m.real_time_factor)))
master, slave = pty.openpty()  # a terminal: the server flushes every line, also when it is killed


def copy_output():
    with open(log, "wb") as f:
        while True:
            try:
                chunk = os.read(master, 65536)
            except OSError:  # the server closed its end
                return
            if not chunk:
                return
            f.write(chunk)


copier = threading.Thread(target=copy_output, daemon=True)
copier.start()
with G.S.twist_publisher({name}):
    proc = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "3", str(world)], env=env, stdout=slave, stderr=slave)
    os.close(slave)
    t0 = time.monotonic()
    while not stamps["/eye_camera/image"] and time.monotonic() - t0 < 120:
        time.sleep(0.2)
    first = time.monotonic() - t0
    time.sleep(5.0)
    a = time.monotonic()
    time.sleep(seconds)
    b = time.monotonic()
    proc.send_signal(signal.SIGINT)  # a clean stop flushes the server log
    try:
        proc.wait(15)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
copier.join(10)
fps = {t: round(sum(a <= x <= b for x in v) / (b - a), 2) for t, v in stamps.items()}
rtfs = [r for t, r in rtf if a <= t <= b]
text = log.read_text(errors="replace")
out = dict(world=world_key, variant=variant, seconds=seconds, first_frame_s=round(first, 1), fps=fps,
           real_time_factor=round(float(np.median(rtfs)), 3) if rtfs else None,
           lens_flare_pass_added="LensFlare Render pass added" in text,
           lens_flare_sensor_missing=text.count("Unable to find sensor"), exit_code=proc.returncode,
           # the OGRE plugin path error is printed once even though the fallback path then works (README)
           errors=[l for l in text.splitlines() if "[Err]" in l and "Unable to load Ogre Plugin" not in l][:5],
           counts=json.loads((assets / "counts.json").read_text()), rgbd=list(rgbd))
print("RESULT", json.dumps(out))
G.save(f"g6_{world_key}_{variant}", out)
