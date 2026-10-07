"""The minimap: a hillshade of the world's terrain from its mission sheet
(sim/worlds/<world>.json, read by urc/sheet.py) and the sheet's named places.

Map frame = world frame: x east, y north, origin at the heightmap centre.
"""
import cv2
import numpy as np

from urc import sheet as sheets

MAX_PIXELS = 768
SUN_AZIMUTH = np.radians(315)  # from the north-west, as on printed maps
SUN_ELEVATION = np.radians(40)
# Hypsometric tint, low to high (RGB), multiplied by the shade.
TINT = np.array([(70, 84, 104), (128, 118, 110), (196, 164, 116), (226, 208, 172)], np.float32)


def places(sheet):
    """Named places to mark: the sheet's points, C2 and the rover start."""
    marks = [{"name": name, "x": p["x"], "y": p["y"], "kind": "point"} for name, p in sheet.get("points", {}).items()]
    for name, kind in (("c2", "c2"), ("rover_start", "start")):
        if name in sheet:
            marks.append({"name": name, "x": sheet[name]["x"], "y": sheet[name]["y"], "kind": kind})
    return marks


def hillshade_png(sheet, sheet_path):
    """PNG bytes of the shaded, tinted terrain, north up, at most MAX_PIXELS a side."""
    hf = sheets.terrain(sheet, sheet_path)
    z = hf.z.astype(np.float32)
    scale = min(1.0, MAX_PIXELS / max(z.shape))
    if scale < 1:
        z = cv2.resize(z, (round(z.shape[1] * scale), round(z.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    rows, cols = z.shape
    # Image rows run south, so north is -row.
    dz_drow, dz_dcol = np.gradient(z, hf.size / (rows - 1), hf.size / (cols - 1))
    dz_dx, dz_dy = dz_dcol, -dz_drow
    normal = np.dstack((-dz_dx, -dz_dy, np.ones_like(z)))
    normal /= np.linalg.norm(normal, axis=2, keepdims=True)
    sun = np.array([np.sin(SUN_AZIMUTH) * np.cos(SUN_ELEVATION), np.cos(SUN_AZIMUTH) * np.cos(SUN_ELEVATION),
                    np.sin(SUN_ELEVATION)])
    shade = np.clip(normal @ sun, 0, 1) * 0.75 + 0.25
    span = float(z.max() - z.min()) or 1.0
    level = (z - z.min()) / span * (len(TINT) - 1)
    low = np.clip(np.floor(level).astype(int), 0, len(TINT) - 2)
    frac = (level - low)[..., None]
    rgb = (TINT[low] * (1 - frac) + TINT[low + 1] * frac) * shade[..., None]
    ok, data = cv2.imencode(".png", cv2.cvtColor(np.clip(rgb, 0, 255).astype(np.uint8), cv2.COLOR_RGB2BGR))
    return data.tobytes()


def map_info(sheet):
    """What the page needs to draw the map (null when the world has no sheet)."""
    if sheet is None:
        return None
    size = float(sheet["terrain"]["size_m"])
    return {"size": (size, size), "places": places(sheet), "mission": sheet.get("mission")}
