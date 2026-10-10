#!/usr/bin/env python3
"""Generate the URC 2027 mission worlds and test courses: pixi run sim-worlds [world ...].

Writes sim/worlds/urc_<mission>.sdf with its mission sheet urc_<mission>.json
(a course: sim/worlds/<course>.sdf and .json), and the models they use
(sim/models/urc_*). Generation is deterministic; edit the layouts in
sim/urc/missions/, not the output. Shared textures and meshes
(sim/models/urc_media) are made once and kept while current; afterwards
whatever no world uses any more is removed (prune). Runs may overlap (each
regenerating its own worlds): they build under a shared lock and prune
under an exclusive one (lock).
"""
import argparse
import contextlib
import fcntl
import re
import shutil
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))
from urc.media import Media  # noqa: E402
from urc.missions import COURSES, MISSIONS  # noqa: E402

MODELS_DIR = SIM_DIR / "models"
WORLDS_DIR = SIM_DIR / "worlds"
DATA_DIR = SIM_DIR / "data"  # the terrain rasters, git-ignored (pixi run fetch-data)
URI = re.compile(r"model://([\w.-]+)((?:/[\w.-]+)*)")
LOCK = ".gen_worlds.lock"  # in the models directory


@contextlib.contextmanager
def lock(models_dir, exclusive=False):
    """Hold the models directory's generator lock: shared while a run builds
    worlds (any number of runs at once), exclusive while one prunes. A run
    writes its media and models before its world file, and prune judges what
    is unused by the world files on disk: pruning during another run's build
    would delete what that run's world is about to use. The lock goes when
    the file is closed, also when the process dies."""
    Path(models_dir).mkdir(parents=True, exist_ok=True)
    with open(Path(models_dir) / LOCK, "a") as f:
        fcntl.flock(f, fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH)
        yield


def prune(models_dir, worlds_dir):
    """Delete the generated models (urc_*) that no world in worlds_dir uses,
    and the shared media files that neither those worlds nor their models
    use (left behind when a generator or its parameters changed, or half
    written by a run that died). References are read from the files on
    disk, so worlds not regenerated in this run keep what they use. Holds
    the exclusive lock, so it waits for runs still building (a caller must
    not hold the shared one). Returns the deleted paths."""
    with lock(models_dir, exclusive=True):
        texts = []
        for path in sorted(Path(worlds_dir).glob("*.sdf")):
            with contextlib.suppress(FileNotFoundError):  # a test's temporary world copy may be gone already
                texts.append(path.read_text())
        used, todo = set(), {name for text in texts for name, _ in URI.findall(text)}
        while todo:
            name = todo.pop()
            used.add(name)
            model = Path(models_dir) / name / "model.sdf"
            if model.is_file():
                text = model.read_text()
                texts.append(text)
                todo |= {other for other, _ in URI.findall(text)} - used
        files = {rest.lstrip("/") for text in texts for name, rest in URI.findall(text) if name == Media.NAME}
        removed = []
        for directory in sorted(Path(models_dir).glob("urc_*")):
            if directory.name not in used and directory.name != Media.NAME:
                shutil.rmtree(directory)
                removed.append(directory)
        media = Path(models_dir) / Media.NAME
        for kind in Media.SUFFIX:
            for path in sorted((media / kind).iterdir()):
                if f"{kind}/{path.name}" not in files:
                    path.unlink()
                    removed.append(path)
        return removed


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    worlds = {**MISSIONS, **COURSES}
    parser.add_argument("worlds", nargs="*", choices=[[]] + sorted(worlds), help="default: all")
    args = parser.parse_args()
    with lock(MODELS_DIR):
        media = Media(MODELS_DIR)
        for key in args.worlds or sorted(worlds):
            start = time.time()
            try:
                world, sheet = worlds[key].build(MODELS_DIR, WORLDS_DIR, media)
            except FileNotFoundError as e:  # a raster that was never fetched: say how to get it
                if e.filename is None or not Path(e.filename).is_relative_to(DATA_DIR):
                    raise
                sys.exit(f"{key}: {Path(e.filename).relative_to(SIM_DIR.parent)} is missing; pixi run fetch-data "
                         "fetches the terrain rasters (README, Simulation)")
            print(f"wrote {world.relative_to(SIM_DIR.parent)} and {sheet.name} ({time.time() - start:.1f} s)")
    for path in prune(MODELS_DIR, WORLDS_DIR):
        print(f"removed {path.relative_to(SIM_DIR.parent)} (unused)")


if __name__ == "__main__":
    main()
