"""Check sim/plugins/terrain_heightmap.hh against urc.sheet's terrain: the HmCheck plugin (hm_check.cpp,
built by build.sh) samples FindTerrainHeightmap's visual and collision heightmaps at 2000 random points,
the corners, the centre and 1 m outside; Python samples the sheet's Heightfield and the nearest-sample
formula of design spec 9.1. Usage: python check_heightmap.py [world ...]"""
import json
import os
import re
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402
from urc import sheet as sheets  # noqa: E402

OUT = G.SCRATCH / "hmcheck"
results = {}
for world in sys.argv[1:] or ["urc_autonomy", "urc_delivery", "proving_ground"]:
    sheet_path = G.REPO / "sim" / "worlds" / f"{world}.json"
    sheet = sheets.load(sheet_path)
    hf = sheets.terrain(sheet, sheet_path)
    rng = np.random.default_rng(1)
    half = hf.size / 2
    pts = np.concatenate([rng.uniform(-half, half, (2000, 2)),
                          [[-half, -half], [half, half], [-half, half], [half, -half], [0, 0], [half + 1, 0]]])
    (OUT / f"{world}_in.txt").write_text("\n".join(f"{float(x)!r} {float(y)!r}" for x, y in pts))
    text = (G.REPO / "sim" / "worlds" / f"{world}.sdf").read_text()
    text = G.SENSORS.sub("", text)
    plugin = (f'<plugin filename="HmCheck" name="check::HmCheck"><in>{OUT / (world + "_in.txt")}</in>'
              f'<out>{OUT / (world + "_out.txt")}</out></plugin>')
    text = text.replace("</world>", plugin + "\n</world>", 1)
    with G.S.temp_sdf(text) as path:
        subprocess.run(["gz", "sim", "-s", "-r", "--iterations", "3", path],
                       env=G.env_for(plugin_dirs=[OUT / "build"]), check=True, capture_output=True)
    lines = (OUT / f"{world}_out.txt").read_text().splitlines()
    rows = np.array([[float(v) for v in line.split()] for line in lines[2:]])
    inside = np.isfinite(rows[:, 2])
    want = np.array([hf.height(x, y) for x, y in rows[inside, :2]])
    n = hf.n
    col = np.rint((rows[inside, 0] + half) / hf.size * (n - 1))
    row = np.rint((half - rows[inside, 1]) / hf.size * (n - 1))
    results[world] = dict(header=lines[1], points=int(inside.sum()), outside_returns_minus_inf=int((~inside).sum()),
                          max_err_visual_m=float(np.abs(rows[inside, 2] - want).max()),
                          max_err_collision_m=float(np.abs(rows[inside, 3] - want).max()),
                          nearest_matches_python=bool(np.all((row == rows[inside, 5]) & (col == rows[inside, 6]))))
print(json.dumps(results, indent=1))
G.save("check_terrain_heightmap", results)
