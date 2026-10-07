"""G3: GLB meshes as DART collisions (rover and sphere rest/roll on them like on OBJ), and GLB vs OBJ
visual memory in a rendering server. Usage: python g3.py collide | memory [runs]"""
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402
import glb  # noqa: E402

D = G.SCRATCH / "g3"
D.mkdir(exist_ok=True)
mode = sys.argv[1]


def mesh_model(name, path, pose, collide=True, visual=True):
    geometry = f"<geometry><mesh><uri>{path}</uri></mesh></geometry>"
    return (f'<model name="{name}"><static>true</static><pose>{pose}</pose><link name="l">'
            + (f'<collision name="c">{geometry}</collision>' if collide else "")
            + (f'<visual name="v">{geometry}<material><diffuse>0.6 0.5 0.4 1</diffuse></material></visual>'
               if visual else "")
            + "</link></model>")


if mode == "collide":
    V, F = glb.box((3.0, 3.0, 0.3))
    W, E = glb.wedge(4.0, 2.0, 20.0)
    for kind, writer in (("glb", glb.write_glb), ("obj", glb.write_obj)):
        writer(D / f"slab.{kind}", V, F)
        writer(D / f"wedge.{kind}", W, E)
    results = {}
    from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402,F401
    for kind in ("glb", "obj"):
        extra = (mesh_model("slab", D / f"slab.{kind}", "0 0 0 0 0 0")
                 + mesh_model("wedge", D / f"wedge.{kind}", "0 6 0 0 0 0")
                 + '<model name="ball"><pose>0.8 6 1.6 0 0 0</pose><link name="l"><inertial><mass>1</mass>'
                   '<inertia><ixx>0.016</ixx><iyy>0.016</iyy><izz>0.016</izz></inertia></inertial>'
                   '<collision name="c"><geometry><sphere><radius>0.2</radius></sphere></geometry></collision>'
                   '</link></model>')
        code = f"""
import sys, json
sys.path.insert(0, {str(G.REPO / 'sim' / 'tests')!r})
import simulate as S
from gz.sim8 import Link, Model, World, world_entity, TestFixture
import gz.math7
seen = {{}}
def pre(info, ecm):
    pass
def post(info, ecm):
    w = World(world_entity(ecm))
    if info.iterations in (1, 3000):
        rover = Model(w.model_by_name(ecm, "rover"))
        base = Link(rover.link_by_name(ecm, "base_link")); base.enable_velocity_checks(ecm, True)
        ball = Model(w.model_by_name(ecm, "ball")); bl = Link(ball.canonical_link(ecm))
        p, q = base.world_pose(ecm), bl.world_pose(ecm)
        seen[info.iterations] = dict(rover=[p.pos().x(), p.pos().y(), p.pos().z()], ball=[q.pos().x(), q.pos().y(), q.pos().z()])
with S.temp_sdf(S.world_sdf({extra!r}, spawn_z=0.32)) as path:
    fx = TestFixture(path); fx.on_post_update(post); fx.finalize(); fx.server().run(True, 3000, False)
print("RESULT", json.dumps(seen))
"""
        r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=300)
        line = next((l for l in r.stdout.splitlines() if l.startswith("RESULT")), None)
        if line is None:
            print(r.stdout[-2000:], r.stderr[-3000:])
            raise SystemExit(1)
        seen = json.loads(line[7:])
        end = seen["3000"]
        results[kind] = dict(rover_base_z_after_3s=round(end["rover"][2], 4),
                             ball_start=[round(v, 3) for v in seen["1"]["ball"]],
                             ball_after_3s=[round(v, 3) for v in end["ball"]])
    # Expected: rover base_link (on the ground between the wheels) rests on the slab top, z = 0.3;
    # the ball (r 0.2) dropped on the 20 deg wedge rolls down towards -x.
    for kind, r in results.items():
        r["rover_on_slab"] = abs(r["rover_base_z_after_3s"] - 0.3) < 0.02
        r["ball_rolled_downhill"] = r["ball_after_3s"][0] < r["ball_start"][0] - 0.3
    results["same_as_obj"] = (abs(results["glb"]["rover_base_z_after_3s"] - results["obj"]["rover_base_z_after_3s"]) < 1e-3
                              and abs(results["glb"]["ball_after_3s"][0] - results["obj"]["ball_after_3s"][0]) < 0.05)
    print(json.dumps(results, indent=1))
    G.save("g3_collide", results)

elif mode == "memory":
    runs = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    # 16 chunk meshes of 100 k triangles each (1.6 M, like the 1.48 M shrub triangles of spec D10),
    # visual only, all in the camera's view.
    S0, F0 = glb.icosphere(5)  # 20480 triangles
    rng = np.random.default_rng(3)
    for kind, writer in (("glb", glb.write_glb), ("obj", glb.write_obj)):
        for c in range(16):
            parts, faces, off = [], [], 0
            for k in range(5):
                parts.append(S0 * rng.uniform(0.3, 0.8) + [rng.uniform(-4, 4), rng.uniform(-4, 4), 0.5])
                faces.append(F0 + off)
                off += len(S0)
            writer(D / f"chunk_{c}.{kind}", np.concatenate(parts), np.concatenate(faces))
    sizes = {kind: sum((D / f"chunk_{c}.{kind}").stat().st_size for c in range(16)) for kind in ("glb", "obj")}

    def world(kind):
        models = "" if kind == "none" else "".join(
            mesh_model(f"chunk_{c}", D / f"chunk_{c}.{kind}", f"{12 + 9 * (c // 4)} {-13.5 + 9 * (c % 4)} 0 0 0 0",
                       collide=False) for c in range(16))
        text = f"""<?xml version="1.0"?>
<sdf version="1.11"><world name="g3mem">
  <physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><diffuse>1 1 1 1</diffuse><direction>0.5 0.2 -0.8</direction></light>
  {models}
  <model name="cam"><static>true</static><pose>0 0 6 0 0.35 0</pose><link name="l">
    <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>20</update_rate><topic>/g3/camera</topic>
      <camera><horizontal_fov>1.4</horizontal_fov><image><width>1280</width><height>720</height></image>
      <clip><near>0.1</near><far>500</far></clip></camera></sensor></link></model>
</world></sdf>"""
        path = D / f"mem_{kind}.sdf"
        path.write_text(text)
        return path

    # A subscriber makes the camera render (an unwatched camera renders nothing).
    from gz.msgs10.image_pb2 import Image  # noqa: E402
    from gz.transport13 import Node  # noqa: E402
    frames = {"n": 0}
    node = Node()
    node.subscribe(Image, "/g3/camera", lambda m: frames.__setitem__("n", frames["n"] + 1))
    out = {kind: [] for kind in ("none", "obj", "glb")}
    for _ in range(runs):
        for kind in out:
            frames["n"] = 0
            cpu, wall, rss = G.S.gz_run(world(kind), 4000, G.env_for())
            out[kind].append(dict(peak_rss_mb=round(rss / 2**20, 1), wall_s=round(wall, 2), frames=frames["n"]))
            print(kind, out[kind][-1], flush=True)
    summary = {kind: dict(peak_rss_mb_median=float(np.median([r["peak_rss_mb"] for r in v])),
                          wall_s_median=float(np.median([r["wall_s"] for r in v])),
                          frames=[r["frames"] for r in v]) for kind, v in out.items()}
    summary["files_mb"] = {k: round(v / 2**20, 1) for k, v in sizes.items()}
    summary["triangles"] = 16 * 5 * len(F0)
    print(json.dumps(summary, indent=1))
    G.save("g3_memory", dict(runs=out, summary=summary))
