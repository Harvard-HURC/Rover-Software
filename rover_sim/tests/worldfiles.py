"""World and model files as the tests read them, without Gazebo: the
generated worlds' sheets and terrain (urc/sheet.py), their models and merged
rocks, copies of a world to run, temporary SDF files and `gz sdf -k`; and
the skips of the tests that need a world or a raster that is not here
(needs_world, needs_raster). Importing it puts sim/ on the path."""
import contextlib
import os
import re
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
sys.path.insert(0, str(SIM_DIR / "tools"))
import fetch_data  # noqa: E402
from urc import sheet as sheets  # noqa: E402

MODELS = SIM_DIR / "models"
WORLDS = SIM_DIR / "worlds"
SENSORS = re.compile(r'<plugin filename="gz-sim-sensors-system".*?</plugin>', re.S)
ROVER_POSE = re.compile(r"(<include>\s*<uri>model://rover</uri>\s*<name>rover</name>\s*<pose>)[^<]*(</pose>)")


def world_missing(world):
    """Why sim/worlds/<world>.sdf (urc_<mission>, proving_ground) is not
    generated here, or None: a raster it reads that fetch-data cannot rebuild
    yet is not here (rasters.json's worlds), so gen_worlds skipped it."""
    return fetch_data.world_missing(world)


def raster_missing(path):
    """Why a test that reads the raster at path (under sim/data) cannot run here, or None."""
    path = Path(path)
    if path.exists():
        return None
    entry = fetch_data.entry_of(path)
    how = "pixi run fetch-data" if entry is None or entry["official"] else fetch_data.cannot_fetch(entry)
    return f"{fetch_data.shown(path)} is not here: {how}"


def needs_world(world):
    """Skips a test or a test class that reads the generated world when it is not here (world_missing)."""
    why = world_missing(world)
    return unittest.skipIf(why is not None, why)


def skip_if_missing(test, world):
    """In a loop over worlds (in a subTest): skips the world when it is not here (world_missing)."""
    why = world_missing(world)
    if why is not None:
        test.skipTest(why)


def needs_raster(path):
    """Skips a test or a test class that reads the raster at path when it is not here (raster_missing)."""
    why = raster_missing(path)
    return unittest.skipIf(why is not None, why)


def sheet(world):
    """The mission sheet of sim/worlds/<world>.sdf (world: urc_<mission>, proving_ground)."""
    return sheets.load(sheets.path(world))


def terrain(world):
    """The world's terrain from its sheet: a Heightfield in world coordinates."""
    return sheets.terrain(sheet(world), sheets.path(world))


def model_root(name):
    """The <model> element of sim/models/<name>/model.sdf."""
    return ET.parse(MODELS / name / "model.sdf").getroot().find("model")


def vec(text):
    """The numbers in an SDF value ("0 0 1", a pose)."""
    return [float(v) for v in text.split()]


def rock_vertices(world):
    """World (x, y, z) of every vertex of the world's merged rock collision meshes."""
    name = re.search(r"<uri>model://(urc_terrain_\w+)</uri>", (WORLDS / f"{world}.sdf").read_text()).group(1)
    out = [np.zeros((0, 3))]
    for c in model_root(name).iter("collision"):
        if re.fullmatch(r"rocks_-?\d+_-?\d+_collision", c.get("name")):
            path = MODELS / name / "meshes" / Path(c.findtext("geometry/mesh/uri")).name
            out.append(np.array([vec(line[2:]) for line in path.read_text().splitlines() if line.startswith("v ")])
                       + vec(c.findtext("pose"))[:3])
    return np.concatenate(out)


@contextlib.contextmanager
def temp_sdf(text, directory=None):
    """A temporary .sdf file holding `text`, deleted afterwards."""
    f = tempfile.NamedTemporaryFile("w", suffix=".sdf", dir=directory, delete=False)
    try:
        with f:
            f.write(text)
        yield f.name
    finally:
        os.unlink(f.name)


@contextlib.contextmanager
def world_copy(world, rover=None, pitch=0.0, lift=0.3, sensors=False):
    """A temporary copy of sim/worlds/<world>.sdf to run. Gazebo's ogre2
    cannot start twice in one process, so the copy drops the Sensors system
    (cameras) unless sensors=True. rover=(x, y, yaw) respawns the rover there
    (world coordinates), `lift` above the sheet's terrain and pitched by
    `pitch`."""
    text = (WORLDS / f"{world}.sdf").read_text()
    if not sensors:
        text = SENSORS.sub("", text)
    if rover is not None:
        x, y, yaw = rover
        z = terrain(world).height(x, y) + lift
        text, count = ROVER_POSE.subn(rf"\g<1>{x} {y} {z} 0 {pitch} {yaw}\g<2>", text)
        assert count == 1, world
    with temp_sdf(text, WORLDS) as path:
        yield path


def gz_check(path):
    """`gz sdf -k` on an SDF file, with sim/models on the model path."""
    env = {"SDF_PATH": str(MODELS), "GZ_SIM_RESOURCE_PATH": str(MODELS), "PATH": str(Path(sys.prefix) / "bin")}
    return subprocess.run(["gz", "sdf", "-k", str(path)], capture_output=True, text=True, env=env)
