"""Proving-ground test worlds (snapshot of sim/worlds/proving_ground.sdf, sensors stripped)."""
import re, math, json
import numpy as np, cv2
from pathlib import Path
HERE = Path(__file__).resolve().parent
TM = HERE / "terrain_models"
_hm = None
def height(x, y):
    global _hm
    if _hm is None:
        img = cv2.imread(str(TM / "urc_terrain_proving_ground/heightmap.png"), cv2.IMREAD_UNCHANGED).astype(float)
        _hm = img / 65535.0 * 8.25395703
    n = _hm.shape[0]
    c = (x + 128) / 256 * (n - 1); r = (128 - y) / 256 * (n - 1)
    c0, r0 = int(c), int(r)
    return float(_hm[r0:r0+2, c0:c0+2].max())

def world(name, x, y, yaw, physics_extra="", dz=0.08):
    src = (HERE / "worlds/proving_ground.sdf").read_text()
    for sysname in ("Sensors", "NavSat", "SceneBroadcaster", "UserCommands"):
        src = re.sub(r'<plugin filename="[^"]+" name="gz::sim::systems::%s"\s*(/>|>.*?</plugin>)' % sysname, "", src, flags=re.S)
    src = re.sub(r"<include>\s*<uri>model://(?!urc_terrain_proving_ground)[^<]+</uri>.*?</include>", "", src, flags=re.S)
    src = src.replace("<real_time_factor>1</real_time_factor>", "<real_time_factor>0</real_time_factor>" + physics_extra)
    # rover: spawn with the wheels just above the highest ground under the footprint
    zs = [height(x + dx * math.cos(yaw) - dy * math.sin(yaw), y + dx * math.sin(yaw) + dy * math.cos(yaw))
          for dx in (-0.6, 0, 0.6) for dy in (-0.5, 0, 0.5)]
    z = max(zs) + dz
    inc = f"<include><uri>model://rover</uri><name>rover</name><pose>{x} {y} {z} 0 0 {yaw}</pose></include>\n  </world>"
    src = src.replace("</world>", inc)
    p = HERE / "worlds" / f"{name}.sdf"
    p.write_text(src)
    return str(p)
