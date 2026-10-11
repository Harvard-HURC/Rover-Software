"""TEMPORARY diagnostic (CI, 2026-10-11): render urc_autonomy's start-post camera view layer by layer.

GitHub's macOS runner (a virtual GPU) drew Autonomy's meshes as spikes while an M4 draws them right. This renders
the view of test_urc_sim.Worlds.test_camera_sees_the_start_post with only some layers present, so the frames show
which meshes break, and writes the meshes' sizes, the GPU and Ogre's log beside them. Usage:
python diag_autonomy_layers.py <out dir>. It exits 1 at the end on purpose, so CI reports when the frames are ready.
"""
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_urc_sim as T  # noqa: E402
from worldfiles import MODELS, SIM_DIR, world_copy  # noqa: E402

import gen_model  # noqa: E402

TERRAIN = "urc_terrain_autonomy"
CLASSES = ("rocks", "slabs", "risers", "shrubs", "pebbles")
PROPS = ("urc_ar_post_0", "urc_ar_post_1", "urc_ar_post_2", "urc_astronaut", "urc_c2_station", "urc_landing_pad",
         "urc_rock_pick_hammer")
FARFIELD = "urc_farfield_autonomy"


def drop_includes(text, models):
    for m in models:
        text = re.sub(rf"<include>(?:(?!</include>).)*?<uri>model://{m}</uri>.*?</include>", "", text, flags=re.S)
    return text


def terrain_variant(models_dir, name, keep):
    """A copy of the terrain model named `name` with only the visuals of `keep` (and the heightmap's)."""
    d = models_dir / name
    d.mkdir(parents=True)
    src = MODELS / TERRAIN
    sdf = (src / "model.sdf").read_text()
    drop = [c for c in CLASSES if c not in keep]
    if drop:
        sdf = re.sub(rf'<visual name="(?:{"|".join(drop)})_[^"]*">.*?</visual>', "", sdf, flags=re.S)
    sdf = sdf.replace(f'<model name="{TERRAIN}"', f'<model name="{name}"', 1)
    (d / "model.sdf").write_text(sdf)
    (d / "model.config").write_text((src / "model.config").read_text().replace(TERRAIN, name))


def glb_vertices(path):
    data = path.read_bytes()
    length = struct.unpack_from("<I", data, 12)[0]
    doc = json.loads(data[20:20 + length])
    count = 0
    for mesh in doc.get("meshes", []):
        for prim in mesh.get("primitives", []):
            count += doc["accessors"][prim["attributes"]["POSITION"]]["count"]
    indices = [doc["accessors"][p["indices"]]["componentType"] for m in doc.get("meshes", []) for p in m["primitives"]
               if "indices" in p]
    return count, sorted(set(indices))


def mesh_sizes():
    out = {}
    for c in CLASSES:
        rows = []
        for p in sorted((MODELS / TERRAIN / "meshes").glob(f"{c}_*")):
            if p.suffix == ".glb":
                n, idx = glb_vertices(p)
                rows.append({"file": p.name, "vertices": n, "index_types": idx})
            elif p.suffix == ".obj":
                rows.append({"file": p.name, "vertices": sum(1 for line in p.open() if line.startswith("v "))})
        out[c] = {"files": len(rows), "max_vertices": max((r["vertices"] for r in rows), default=0), "rows": rows}
    return out


def main(out):
    out.mkdir(parents=True, exist_ok=True)
    (out / "meshes.json").write_text(json.dumps(mesh_sizes(), indent=1))
    if sys.platform == "darwin":
        (out / "gpu.txt").write_text(subprocess.run(["system_profiler", "SPDisplaysDataType"], capture_output=True,
                                                    text=True).stdout)
    post = T.sheet("urc_autonomy")["objects"]["route_start"]
    x = post["x"] + T.CAMERA_RANGE * math.cos(post["yaw"])
    y = post["y"] + T.CAMERA_RANGE * math.sin(post["yaw"])
    variants = {"terrain": ((), False, False)}
    variants.update({f"terrain+{c}": ((c,), False, False) for c in CLASSES})
    variants["terrain+farfield"] = ((), True, False)
    variants["terrain+props"] = ((), False, True)
    variants["full"] = (CLASSES, True, True)
    results = {}
    with tempfile.TemporaryDirectory() as tmp:
        models = Path(tmp) / "models"
        for name, (keep, farfield, props) in variants.items():
            model = f"{TERRAIN}_{name.replace('+', '_')}"
            terrain_variant(models, model, keep)
            with world_copy("urc_autonomy", rover=(x, y, post["yaw"] + math.pi), sensors=True) as world:
                text = Path(world).read_text().replace(f"model://{TERRAIN}<", f"model://{model}<")
                if not farfield:
                    text = drop_includes(text, [FARFIELD])
                if not props:
                    text = drop_includes(text, PROPS)
                Path(world).write_text(text)
                env = dict(os.environ, ROVER_TEST_ARTIFACTS=str(out / name),
                           GZ_SIM_RESOURCE_PATH=os.pathsep.join([str(models), os.environ.get("GZ_SIM_RESOURCE_PATH", "")]))
                code = T.CAMERA_CHECK.format(tests=str(SIM_DIR / "tests"), topic=gen_model.CAMERA_TOPIC + "/image",
                                             world=world)
                r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=300, env=env)
                results[name] = [l for l in r.stdout.splitlines() if l.startswith(("IDS", "FRAMES"))] or \
                    [r.stderr[-1500:]]
                print(name, results[name], flush=True)
    (out / "results.json").write_text(json.dumps(results, indent=1))
    for log in Path.home().glob(".gz/rendering/*.log"):
        shutil.copy(log, out / log.name)
    print("diagnostic done: inspect", out)
    return 1  # on purpose: CI reports a failure when the frames are ready


if __name__ == "__main__":
    sys.exit(main(Path(sys.argv[1] if len(sys.argv) > 1 else "diag-frames")))
