"""Rendered pictures of the appearance assets (design spec section 11,
"Rendering"): the patched media's sky, Terra roughness and haze, the far
field, merged GLB clutter, what the haze leaves alone (depth), and the
record of the rover's opt-in dust (gen_model.DriveParams.dust, off by
default since the user's decision of 2026-10-07): drawn, and seen by depth.

Gazebo's ogre2 starts once per process, so every picture is taken in a
subprocess of its own (this file run with --render) from a small world built
here, with the patched media (sim/tools/gz_media.py, made into a temporary
build directory) or Gazebo's stock media.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

from worldfiles import SIM_DIR

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
import gzenv  # noqa: E402
from urc import appearance as A  # noqa: E402
from urc import dem, farfield, geo, lighting, meshes, props, sdf, terrain  # noqa: E402
from urc.media import Media  # noqa: E402

sys.path.insert(0, str(SIM_DIR / "tools"))
import gz_media  # noqa: E402

STEPS = 1500  # [1 ms] per picture: ten frames at 10 Hz after the scene has loaded
SUN = lighting.MISSION


def world(body, sky=True, particles=False):
    """A world document: DART at 1 ms as fast as it runs, the mission sun and
    sky, the Sensors system, and `body`."""
    f = lambda v: " ".join(f"{x:.6g}" for x in v)  # noqa: E731
    emitters = '<plugin filename="gz-sim-particle-emitter-system" name="gz::sim::systems::ParticleEmitter"/>'
    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="render">
    <physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine></plugin>
    {emitters if particles else ''}
    <scene><ambient>{f(lighting.AMBIENT)} 1</ambient><background>{f(lighting.BACKGROUND)} 1</background>
      <grid>false</grid><shadows>true</shadows>{'<sky/>' if sky else ''}</scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 100 0 0 0</pose>
      <diffuse>{f(SUN.colour)} 1</diffuse><specular>{f(lighting.SUN_SPECULAR)} 1</specular>
      <intensity>{SUN.intensity}</intensity><direction>{f(SUN.direction)}</direction></light>
    {body}
  </world>
</sdf>
"""


def camera(name, xyz, yaw, pitch, size=(640, 360), hfov=1.0, kind="camera", noise=None, far=80000.0, depth_far=40.0):
    """A static camera model looking along yaw (from +x, counter-clockwise),
    pitched down by `pitch` [rad], publishing on /render/<name> (an RGB-D
    camera: /render/<name>/image and /render/<name>/depth_image)."""
    noise_sdf = f"<noise><type>gaussian</type><mean>0</mean><stddev>{noise}</stddev></noise>" if noise else ""
    depth = f"<depth_camera><clip><near>0.1</near><far>{depth_far}</far></clip></depth_camera>" \
        if kind == "rgbd_camera" else ""
    fmt = "" if kind == "depth_camera" else "<format>R8G8B8</format>"
    return f"""
    <model name="cam_{name}"><static>true</static><pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 {pitch} {yaw}</pose>
      <link name="link"><sensor name="cam" type="{kind}"><always_on>true</always_on><update_rate>10</update_rate>
        <topic>/render/{name}</topic>
        <camera><horizontal_fov>{hfov}</horizontal_fov><image><width>{size[0]}</width><height>{size[1]}</height>{fmt}</image>
          <clip><near>0.1</near><far>{far}</far></clip>{depth}{noise_sdf}</camera>
      </sensor></link></model>"""


def model(models_dir, name, build):
    """Write models_dir/name with a static link that build(link) fills; returns its <include>."""
    root, m = sdf.model_root(name, static=True)
    build(sdf.link(m, "link"))
    sdf.write_model(models_dir, name, root, "Render test model.", generator="sim/tests/test_render.py")
    return f"<include><uri>model://{name}</uri><pose>0 0 0 0 0 0</pose></include>"


class Renderer:
    """Takes pictures in subprocesses, with the patched media or stock."""

    def __init__(self, tmp):
        self.tmp = Path(tmp)
        self.models = self.tmp / "models"
        self.media = Media(self.models)
        self.patched = self.tmp / "patched"
        gz_media.build(gz_media.environment_media(), self.patched)  # raises: the patch must work here
        self.stock = self.tmp / "stock"
        self.stock.mkdir()
        self.count = 0

    def take(self, text, topics, patched=True):
        """{topic: last image} of world `text` (uint8 RGB or float32 depth)."""
        self.count += 1
        out = self.tmp / f"shot{self.count}"
        out.mkdir()
        path = out / "world.sdf"
        path.write_text(text)
        base = {k: v for k, v in os.environ.items() if k != "GZ_RENDERING_RESOURCE_PATH"}
        env = gzenv.environment(self.patched if patched else self.stock, partition=f"render_{os.getpid()}_{self.count}",
                                ip="127.0.0.1", base=base)
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([str(self.models), env["GZ_SIM_RESOURCE_PATH"]])
        cfg = out / "cfg.json"
        cfg.write_text(json.dumps({"world": str(path), "topics": topics, "steps": STEPS, "out": str(out)}))
        result = subprocess.run([sys.executable, __file__, "--render", str(cfg)], env=env, capture_output=True,
                                text=True, timeout=300)
        shots = {}
        for k, topic in enumerate(topics):
            file = out / f"{k}.npy"
            if not file.exists():
                raise AssertionError(f"no picture on {topic}:\n{result.stdout[-3000:]}\n{result.stderr[-3000:]}")
            shots[topic] = np.load(file)
        return shots


def _render_main(cfg_path):
    """The subprocess: run the world headless and save each topic's last image."""
    cfg = json.loads(Path(cfg_path).read_text())
    import gz.math7  # noqa: F401  (lets gz.sim8 return Pose3d values)
    from gz.msgs10.image_pb2 import Image
    from gz.sim8 import TestFixture
    from gz.transport13 import Node
    last = {}
    node = Node()
    for topic in cfg["topics"]:
        node.subscribe(Image, topic, lambda msg, t=topic: last.__setitem__(t, msg))
    fixture = TestFixture(cfg["world"])
    fixture.finalize()
    fixture.server().run(True, cfg["steps"], False)
    time.sleep(0.5)
    for k, topic in enumerate(cfg["topics"]):
        if topic not in last:
            print(f"NO IMAGE {topic}", flush=True)
            continue
        msg = last[topic]
        if len(msg.data) == msg.width * msg.height * 4:
            image = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        else:
            image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        np.save(Path(cfg["out"]) / f"{k}.npy", image)


def residual(image, sigma=6):
    """Standard deviation of what a Gaussian blur removes: texture, clouds [DN]."""
    img = image.astype(np.float32)
    return float((img - cv2.GaussianBlur(img, (0, 0), sigma)).std())


class Render(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.r = Renderer(cls._tmp.name)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_procedural_sky_replaces_the_cumulus(self):
        """Stock cumulus sky gone: the patched sky is a smooth blue gradient."""
        text = world(camera("sky", (0, 0, 2), math.pi / 2, -0.45, hfov=1.2))
        stock = self.r.take(text, ["/render/sky"], patched=False)["/render/sky"]
        patched = self.r.take(text, ["/render/sky"])["/render/sky"]
        self.assertLess(residual(patched), 0.8)
        self.assertGreater(residual(stock), 3 * max(residual(patched), 0.3))
        top = patched[:60].reshape(-1, 3).mean(axis=0)
        self.assertGreater(top[2], top[0] + 40)  # blue overhead
        self.assertGreater(patched[-30:].mean(), patched[:30].mean())  # paler towards the horizon

    def test_no_glint_on_sunlit_terrain(self):
        """No specular glint on sunlit flat terrain: looking at the sun's
        mirror image on Terra, the brightest 0.1 % of ground pixels stay
        within p99 + 15 DN (stock: roughness 0, a mirror)."""
        tmp = self.r.models / "urc_glint"
        tmp.mkdir(parents=True, exist_ok=True)
        hf = terrain.Heightfield(64.0, 129)
        hf.z = np.linspace(0.0, 0.2, 129)[None, :].repeat(129, axis=0)  # nearly flat; the PNG needs a maximum
        hf.write_png(tmp / "heightmap.png")
        cv2.imwrite(str(tmp / "colour.png"), np.full((64, 64, 3), (120, 150, 190), np.uint8))  # BGR: tan

        def build(link):
            v = sdf.sub(link, "visual", name="terrain")
            g = sdf.sub(sdf.sub(v, "geometry"), "heightmap")
            sdf.sub(g, "use_terrain_paging", False)
            t = sdf.sub(g, "texture")
            sdf.sub(t, "diffuse", "model://urc_glint/colour.png")
            sdf.sub(t, "normal", self.r.media.flat_normal())
            sdf.sub(t, "size", 64.0)
            sdf.sub(g, "uri", "model://urc_glint/heightmap.png")
            sdf.sub(g, "size", (64.0, 64.0, 0.2))

        include = model(self.r.models, "urc_glint", build)
        s = SUN.toward
        yaw, pitch = math.atan2(s[1], s[0]), math.radians(SUN.elevation_deg)  # towards the mirrored sun
        eye = (-12 * s[0], -12 * s[1], 12 * s[2])
        text = world(include + camera("glint", eye, yaw, pitch, hfov=0.8))
        brightest = {}
        for patched in (False, True):
            ground = self.r.take(text, ["/render/glint"], patched=patched)["/render/glint"].astype(float).mean(axis=2)
            brightest[patched] = (np.percentile(ground, 99.9), np.percentile(ground, 99))
        p999, p99 = brightest[True]
        self.assertLessEqual(p999, p99 + 15, brightest)
        self.assertGreater(brightest[False][0], brightest[False][1] + 15, brightest)  # the stock glint

    def terra(self, name, hf, layers):
        """A heightmap visual of hf (its own frame, lowest point 0) with
        Terra layers [(diffuse, normal, size)] and blends; its <include>."""
        directory = self.r.models / name
        directory.mkdir(parents=True, exist_ok=True)
        _, z_max = hf.write_png(directory / "heightmap.png", 0.0)

        def build(link):
            g = sdf.sub(sdf.sub(sdf.sub(link, "visual", name="terrain"), "geometry"), "heightmap")
            sdf.sub(g, "use_terrain_paging", False)
            layers(g)
            sdf.sub(g, "uri", f"model://{name}/heightmap.png")
            sdf.sub(g, "size", (hf.size, hf.size, z_max))

        return model(self.r.models, name, build)

    def test_detail_layers_keep_the_colour_map(self):
        """Terra renders a compensated colour map under the shared detail
        layers (appearance.terra_layers) in the colour map's own colours:
        seen from 25 m, each half of a two-colour map averages within 3 DN
        of the plain colour map's render (the details are zero-mean; M:
        0.1 DN in the render prototype), and the details add texture."""
        hf = terrain.Heightfield(64.0, 129).noise(1.0, 30.0, 4)
        hf.z -= hf.z.min()
        colour = np.zeros((256, 256, 3), np.uint8)
        colour[:, :128], colour[:, 128:] = (200, 170, 130), (150, 150, 145)
        plain_dir = self.r.models / "urc_plain"
        plain_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(plain_dir / "colour.png"), cv2.cvtColor(colour, cv2.COLOR_RGB2BGR))
        layers = A.terra_layers(colour, hf, A.DEFAULT_DETAILS, self.r.media)
        detail_dir = self.r.models / "urc_detail"
        detail_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(detail_dir / "colour.png"), cv2.cvtColor(layers.layer0, cv2.COLOR_RGB2BGR))

        def plain(g):
            t = sdf.sub(g, "texture")
            sdf.sub(t, "diffuse", "model://urc_plain/colour.png")
            sdf.sub(t, "normal", self.r.media.flat_normal())
            sdf.sub(t, "size", hf.size)

        cam = camera("top", (0, 0, 25.0), 0.0, math.pi / 2, (480, 480), 0.9)
        shots = {}
        for name, fill in (("urc_plain", plain),
                           ("urc_detail", lambda g: layers.write(g, "model://urc_detail/colour.png",
                                                                 self.r.media.flat_normal(), hf.size))):
            shots[name] = self.r.take(world(self.terra(name, hf, fill) + cam), ["/render/top"])["/render/top"]
        # Looking straight down, the image's top is east: the west half is the bottom rows.
        for rows in (slice(300, 460), slice(20, 180)):
            a = shots["urc_plain"][rows, 40:440].reshape(-1, 3).astype(float).mean(axis=0)
            b = shots["urc_detail"][rows, 40:440].reshape(-1, 3).astype(float).mean(axis=0)
            self.assertLess(np.abs(a - b).max(), 3.0, (a, b))
        self.assertGreater(residual(shots["urc_detail"], 2), residual(shots["urc_plain"], 2) + 1.0)

    def test_aruco_through_haze_and_noise(self):
        """ArUco 0 still decoded at 2.5 m with the haze and camera noise
        (stddev 0.06, ~2 DN, design spec 7): today's rover camera, 640 x 480
        over 1.5 rad, square on to a 20 cm tag."""
        tag = self.r.media.aruco(0)
        include = model(self.r.models, "urc_tag", lambda link: props.picture(
            link, self.r.media, "tag", tag, 0.2, 0.2, (0, 0, 0.6, 0, 0, 0)))
        text = world(include + camera("tag", (2.5, 0, 0.6), math.pi, 0.0, (640, 480), 1.5, noise=0.06))
        image = self.r.take(text, ["/render/tag"])["/render/tag"]
        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        _, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
        self.assertIsNotNone(ids)
        self.assertIn(0, ids.ravel().tolist())

    def boxes(self):
        """Boxes at 3-30 m in front of a camera at the origin, on a ground plane."""
        out = ['<model name="ground"><static>true</static><link name="link"><visual name="v"><geometry><plane>'
               '<normal>0 0 1</normal><size>200 200</size></plane></geometry></visual></link></model>']
        for k, (x, y) in enumerate(((3, -1), (6, 1.5), (12, -3), (30, 4))):
            out.append(f'<model name="box{k}"><static>true</static><pose>{x} {y} 0.5 0 0 0.3</pose><link name="link">'
                       f'<visual name="v"><geometry><box><size>1 1 1</size></box></geometry></visual></link></model>')
        return "".join(out)

    RGBD = ["/render/d/depth_image", "/render/d/image"]

    def rgbd(self, extra="", patched=True):
        """An RGB-D camera (the rover's kind: depth clipped at 40 m) looking
        at the boxes, with `extra` in the world."""
        cam = camera("d", (0, 0, 0.6), 0.0, 0.05, (320, 240), 1.2, kind="rgbd_camera")
        return self.r.take(world(self.boxes() + cam + extra, particles=True), self.RGBD, patched)

    @staticmethod
    def dust():
        """The rover's own dust emitter (gen_model._add_dust_emitter, which the rover has only with
        DriveParams.dust; its sprite from sim/models/rover) emitting at 400 /s, ten times the drivetrain's most,
        2 m in front of the camera."""
        link = ET.Element("link", name="link")
        gen_model._add_dust_emitter(link, gen_model.Params(), "e")
        emitter = link.find("particle_emitter")
        emitter.find("emitting").text, emitter.find("rate").text = "true", "400"
        emitter.find("pose").text = "0 0 0 0 0 0"
        return (f'<model name="dust"><static>true</static><pose>2 0 0.3 0 0 0</pose>'
                f'{ET.tostring(link, encoding="unicode")}</model>')

    @staticmethod
    def same_depth(a, b):
        finite = np.isfinite(a) & np.isfinite(b)
        return np.array_equal(np.isfinite(a), np.isfinite(b)) and float(np.abs(a[finite] - b[finite]).max()) < 1e-3

    def test_depth_unchanged_by_haze(self):
        """The haze patch changes the colour image, never the depth image."""
        stock, patched = self.rgbd(patched=False), self.rgbd()
        self.assertTrue(self.same_depth(stock[self.RGBD[0]], patched[self.RGBD[0]]))
        self.assertGreater(np.abs(stock[self.RGBD[1]].astype(int) - patched[self.RGBD[1]].astype(int)).max(), 5)

    def test_dust_is_drawn(self):
        """The rover's opt-in dust (DriveParams.dust, off by default) shows in
        the colour image as dust: pale tan, its sprite's colour
        (terrains.DUST_RGB), not black (an emitter material without a diffuse
        draws black smoke, measured). Kept for when the switch comes back on."""
        plain, dusty = self.rgbd(), self.rgbd(self.dust())
        before, after = plain[self.RGBD[1]].astype(int), dusty[self.RGBD[1]].astype(int)
        changed = np.abs(after - before).max(axis=-1) > 8
        self.assertGreater(changed.mean(), 0.02)
        r, g, b = after[changed].mean(axis=0)
        self.assertGreater(r, before[changed].mean() + 20, (r, g, b))  # lighter than the dark boxes behind it
        self.assertGreater(r, b + 10, (r, g, b))  # tan

    @unittest.expectedFailure
    def test_depth_unchanged_by_dust(self):
        """Design spec D15/Q11 (depth does not see dust): the depth image
        with the rover's dust emitter at full rate in view is the one without
        it. Fails in gz-sim 8.10 / gz-rendering 8.2.2 (M, 2026-10-07): the
        depth shader (depth_camera_fs.metal) takes every pixel of a particle
        with any red (particle.x > 0) for a return at a fixed scatter ratio:
        <particle_scatter_ratio> 1e-6, 0.1 and 1 changed 8,710, 8,640 and
        8,601 of 76,800 pixels, and so did ratios sent on the emitter's
        topic. Only a particle material without a diffuse stays out of the
        depth image, and it renders black. Hence the user's decision of
        2026-10-07: the rover has no dust by default (DriveParams.dust off,
        test_gen_model.Dust), and Q11 is met by its absence. Kept as an
        expected failure, the record for the opt-in switch: the emitter asks
        for a scatter ratio near none (DriveParams.dust_scatter_ratio; with
        none a gz-rendering that honours it would apply 0.65), so the test
        starts passing when that ratio is honoured, and then the dust can
        come back on."""
        plain, dusty = self.rgbd(), self.rgbd(self.dust())
        self.assertTrue(self.same_depth(plain[self.RGBD[0]], dusty[self.RGBD[0]]))

    def test_glb_clutter_stands_z_up(self):
        """A merged two-colour GLB (the clutter's format) renders upright
        where its vertices are: a red slab on the left, a blue one on the right."""
        def slabs(path):
            V, F = meshes.slab(0)
            V = V * (1.0, 1.0, 2.0)
            meshes.write_glb_parts(path, [(V + (5, 1.2, 0), F, None, None, meshes.Material((0.8, 0.05, 0.05))),
                                          (V + (5, -1.2, 0), F, None, None, meshes.Material((0.05, 0.05, 0.8)))])

        uri = self.r.media.glb("test_slabs", slabs)
        include = model(self.r.models, "urc_slabs", lambda link: sdf.sub(
            sdf.sub(sdf.sub(sdf.sub(link, "visual", name="v"), "geometry"), "mesh"), "uri", uri))
        image = self.r.take(world(include + camera("glb", (0, 0, 0.4), 0.0, 0.0, (640, 360), 1.0)),
                            ["/render/glb"])["/render/glb"].astype(int)
        # 1.2 m left and right at 5 m: columns 320 -+ 320 * 1.2 / 5 / tan(0.5) ~ 179 and 461; 0.4 m up: the middle row
        left, right = image[170:190, 160:200].reshape(-1, 3).mean(0), image[170:190, 440:480].reshape(-1, 3).mean(0)
        self.assertGreater(left[0], left[2] + 30, (left, right))
        self.assertGreater(right[2], right[0] + 30, (left, right))

    def test_far_field_on_the_horizon(self):
        """In the Autonomy area, pixels towards the Henry Mountains (Mount
        Ellen, ~35 km south) and Factory Butte (~10 km west-north-west) are
        not sky: the far field puts them there."""
        far = dem.read_geotiff(farfield.FAR_DEM)
        lat, lon = lighting.MISSION_SITE
        origin = geo.Origin(lat, lon, far.height(lat, lon) + dem.NAVD88_TO_WGS84)
        farfield.build(self.r.models, self.r.media, "urc_far_render", origin, terrain.Heightfield(64.0, 65))
        eye = 2.0
        cams, aims = "", {}
        peaks = (("ellen", ((38.09, 38.13), (-110.84, -110.79))), ("butte", ((38.46, 38.48), (-110.91, -110.88))))
        for name, box in peaks:
            (s, n), (w, e) = box
            r0, c0 = (int(v) for v in np.floor(far.pixel(n, w)))
            r1, c1 = (int(v) for v in np.ceil(far.pixel(s, e)))
            r, c = np.unravel_index(np.argmax(far.z[r0:r1, c0:c1]), (r1 - r0, c1 - c0))
            plat, plon = far.lat0 - (r0 + r + 0.5) * far.dlat, far.lon0 + (c0 + c + 0.5) * far.dlon
            x, y, _ = geo.wgs84_to_enu(origin, plat, plon)
            dist = math.hypot(x, y)
            z = float(far.z[r0 + r, c0 + c]) + dem.NAVD88_TO_WGS84 - origin.alt
            z -= (1 - farfield.REFRACTION) * dist * dist / (2 * farfield.EARTH_RADIUS)
            elevation = math.atan2(z - eye, dist)
            aims[name] = elevation
            cams += camera(name, (0, 0, eye), math.atan2(y, x), 0.0, (640, 360), 0.6)
        include = "<include><uri>model://urc_far_render</uri><pose>0 0 0 0 0 0</pose></include>"
        topics = ["/render/ellen", "/render/butte"]
        with_far = self.r.take(world(include + cams), topics)
        sky_only = self.r.take(world(cams), topics)
        focal = 320 / math.tan(0.3)
        for name, topic in zip(("ellen", "butte"), topics):
            row = int(round(180 - focal * math.tan(aims[name] - math.radians(0.4))))  # a little below the top
            a = with_far[topic][row - 2:row + 3, 316:325].reshape(-1, 3).mean(0)
            b = sky_only[topic][row - 2:row + 3, 316:325].reshape(-1, 3).mean(0)
            self.assertGreater(np.abs(a - b).max(), 20, (name, a, b, math.degrees(aims[name])))


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--render":
        _render_main(sys.argv[2])
    else:
        unittest.main()
