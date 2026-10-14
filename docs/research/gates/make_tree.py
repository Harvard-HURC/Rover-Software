"""A scratch copy of the repo's sim/ with patched mission resolutions, generated (gates G1, G7), in
GATES_SCRATCH/trees/<name>.

Usage: python make_tree.py <name> <world> [<world> ...] [--patch file:old:new ...]
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

WT, SCRATCH = G.REPO, G.SCRATCH

parser = argparse.ArgumentParser()
parser.add_argument("name")
parser.add_argument("worlds", nargs="+")
parser.add_argument("--patch", action="append", default=[], help="relative/file.py|old|new")
args = parser.parse_args()

tree = SCRATCH / "trees" / args.name
shutil.rmtree(tree, ignore_errors=True)
sim = tree / "sim"
shutil.copytree(WT / "sim", sim, symlinks=True,
                ignore=shutil.ignore_patterns("data", "build", "worlds", "models", "__pycache__", "*.pyc"))
(sim / "data").symlink_to(WT / "sim" / "data")
(sim / "build").symlink_to(WT / "sim" / "build")
(sim / "worlds").mkdir()
shutil.copy(WT / "sim" / "worlds" / "rover_test.sdf", sim / "worlds")
(sim / "models").mkdir()
for name in ("rover", "chase_camera", "eye_camera"):
    shutil.copytree(WT / "sim" / "models" / name, sim / "models" / name)
for spec in args.patch:
    rel, old, new = spec.split("|")
    path = sim / rel
    text = path.read_text()
    assert text.count(old) == 1, (rel, old)
    path.write_text(text.replace(old, new))
    print(f"patched {rel}: {old!r} -> {new!r}")
start = time.time()
out = subprocess.run([sys.executable, str(sim / "gen_worlds.py"), *args.worlds], capture_output=True, text=True)
print(out.stdout, out.stderr[-3000:])
total = time.time() - start
times = {}
for line in out.stdout.splitlines():
    if line.startswith("wrote "):
        world = line.split()[1].split("/")[-1].removesuffix(".sdf")
        times[world] = float(line.rsplit("(", 1)[1].split(" s)")[0])
(tree / "generation.json").write_text(json.dumps({"seconds": times, "total_with_startup_s": round(total, 1),
                                                   "returncode": out.returncode}, indent=1))
print(json.dumps(times), "total", round(total, 1))
sys.exit(out.returncode)
