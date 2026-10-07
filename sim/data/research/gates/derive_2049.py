"""G1: the 2049^2 twin of the 4097^2 lidar Autonomy world: same models and clutter, the heightmap
subsampled every other sample (renormalised to its own maximum, as Gazebo scales by it)."""
import os
import re
import shutil
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

SCRATCH = G.SCRATCH
src = SCRATCH / "trees" / "g1_lidar4097" / "sim"
dst_tree = SCRATCH / "trees" / "g1_lidar2049"
shutil.rmtree(dst_tree, ignore_errors=True)
dst = dst_tree / "sim"
shutil.copytree(src, dst, symlinks=True, ignore=shutil.ignore_patterns("dem.tif"))
terrain = dst / "models" / "urc_terrain_autonomy"
with Image.open(terrain / "heightmap.png") as img:
    z16 = np.asarray(img, dtype=np.float64)
sdf = (terrain / "model.sdf").read_text()
z_max = float(re.search(r"<size>2048 2048 ([0-9.eE+-]+)</size>", sdf).group(1))
z = z16 / 65535.0 * z_max
sub = z[::2, ::2]
new_max = float(sub.max())
Image.fromarray(np.round(sub / new_max * 65535).astype(np.uint16)).save(terrain / "heightmap.png")
sdf, count = re.subn(r"<size>2048 2048 [0-9.eE+-]+</size>", f"<size>2048 2048 {new_max:.9g}</size>", sdf)
assert count == 2, count
(terrain / "model.sdf").write_text(sdf)
print("4097 ->", sub.shape, "z_max", z_max, "->", new_max, "min", sub.min())
