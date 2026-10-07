"""Mission sheets: the judges' hand-out and ground truth that WorldBuilder
writes next to each world (sim/worlds/<world>.json).

The one reader of a sheet and its terrain, for the referee and judges, the
driver station's map and the tests: terrain() decodes the sheet's heightmap
into a terrain.Heightfield in world coordinates, so every reader queries
heights and radio line of sight the same way the world builder did.
"""
import json
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


def terrain(sheet, sheet_path):
    """The sheet's heightmap as a Heightfield in world coordinates: centred on
    the world origin, row 0 north, column 0 west, z = pixel / 65535 * z_max."""
    t = sheet["terrain"]
    with Image.open(Path(sheet_path).parent / t["heightmap"]) as img:
        z = np.asarray(img, dtype=float) / 65535.0 * t["z_max"]
    return _terrain.Heightfield(t["size_m"], z.shape[0], z)


def radio_los(hf, antenna, x, y, ground):
    """Line of sight from the C2 antenna (x, y, z) to a rover antenna
    ROVER_ANTENNA_HEIGHT above `ground` at (x, y), all in hf's frame."""
    return hf.line_of_sight(antenna, (x, y, ground + ROVER_ANTENNA_HEIGHT))
