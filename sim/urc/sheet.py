"""Mission sheets: the judges' hand-out and ground truth that WorldBuilder
writes next to each world (sim/worlds/<world>.json).

The one reader of a sheet and its terrain, for the referee and judges, the
driver station's map and the tests: terrain() decodes the sheet's heightmap
into a terrain.Heightfield in world coordinates, so every reader queries
heights and radio line of sight the same way the world builder did;
ground() reads the ground map next to it (ground.png and ground.json,
design spec 9.1), the same lookup the drivetrain makes.
"""
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

from . import terrain as _terrain

WORLDS_DIR = Path(__file__).resolve().parents[1] / "worlds"
ROVER_ANTENNA_HEIGHT = 1.0  # [m] above the ground under the rover, roughly its mast, for radio line of sight


def path(world):
    """sim/worlds/<world>.json (world: urc_<mission>, proving_ground, ...)."""
    return WORLDS_DIR / f"{world}.json"


def find(world, world_path=None):
    """The sheet next to a world file, else sim/worlds/<world>.json; None if
    the world has none (rover_test)."""
    candidates = [Path(world_path).with_suffix(".json")] if world_path else []
    candidates.append(path(world))
    return next((p for p in candidates if p.is_file()), None)


def load(sheet_path):
    return json.loads(Path(sheet_path).read_text())


def terrain(sheet, sheet_path, collision=False):
    """The sheet's heightmap as a Heightfield in world coordinates: centred on
    the world origin, row 0 north, column 0 west, z = pixel / 65535 * z_max.
    collision: the surface wheels and objects rest on, where the world has a
    carved collision heightmap (sinkage), else the same."""
    t = sheet["terrain"]
    png, z_max = ((t["collision_heightmap"], t["z_max_collision"]) if collision and "collision_heightmap" in t
                  else (t["heightmap"], t["z_max"]))
    with Image.open(Path(sheet_path).parent / png) as img:
        z = np.asarray(img, dtype=float) / 65535.0 * z_max
    return _terrain.Heightfield(t["size_m"], z.shape[0], z)


class Ground:
    """A world's ground map (design spec 9.1): which ground type lies at a
    world (x, y), by its nearest sample of ground.png, and that type's entry
    of ground.json (traction, dust). Outside the map: ground.json's default."""

    def __init__(self, png, info):
        with Image.open(png) as img:
            self.raster = np.asarray(img)
        self.info = info
        self.png = Path(png)
        self.types = {t["index"]: t for t in info["types"]}
        self.size, self.samples = info["size_m"], info["samples"]
        assert self.raster.shape == (self.samples, self.samples), (png, self.raster.shape)

    def type(self, x, y):
        """ground.json's entry for the type at world (x, y)."""
        res = self.size / (self.samples - 1)
        col, row = round((x + self.size / 2) / res), round((self.size / 2 - y) / res)
        inside = 0 <= col < self.samples and 0 <= row < self.samples and math.isfinite(x + y)
        return self.types[int(self.raster[row, col]) if inside else self.info["default"]]

    def __call__(self, x, y):
        """The key of the ground type at world (x, y)."""
        return self.type(x, y)["key"]


def ground(sheet, sheet_path):
    """The world's ground map: ground.png and ground.json next to its
    heightmap, where the drivetrain finds them too; None if it has none."""
    directory = (Path(sheet_path).parent / sheet["terrain"]["heightmap"]).parent
    if not (directory / "ground.json").is_file():
        return None
    return Ground(directory / "ground.png", json.loads((directory / "ground.json").read_text()))


def radio_los(hf, antenna, x, y, ground):
    """Line of sight from the C2 antenna (x, y, z) to a rover antenna
    ROVER_ANTENNA_HEIGHT above `ground` at (x, y), all in hf's frame."""
    return hf.line_of_sight(antenna, (x, y, ground + ROVER_ANTENNA_HEIGHT))
