#!/usr/bin/env python3
"""The realism report of the design (docs/superpowers/specs/2026-10-06-urc-realism-design.md,
section 11): writes sim/data/research/realism_report.json and
realism_contact_sheet.jpg (300 m windows of the rendered maps over the
hillshade of their terrain, in the layout of mdrs_surface_types_crops.jpg,
which shows the real ground the same way).

The report holds, each with its target and whether it is met:
- roughness per ground type and per synthetic world against
  terrain_targets.json; slab size-frequency; shrub density; palette CIE76
  (urc/realism.py, which tests/test_realism.py also checks);
- Autonomy's colour map against boosted NAIP 2024, by slope;
- each rendered map (tools/render_map.py, pixi run sim-maps) against its
  world's colour map, and Autonomy's against raw NAIP for information, both
  by slope, with the overall exposure the sun and ambient give the ground;
- the spin-in-place ratio per catalogue ground, measured with the physical
  rover on flat ground (headless Gazebo, tests/simulate.py), against the
  closed form of design 5.6 on both of DART's solvers, and the dig-in of a sand spin under both presets.

Usage: python sim/tools/realism_report.py [--no-physics] [--no-render]
(about 3 minutes; the worlds must be generated, and the maps rendered and
current for the render rows, which are skipped otherwise).
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
sys.path.insert(0, str(SIM_DIR / "tools"))
from urc import appearance, geo, realism, terrain, terrains, textures  # noqa: E402
from urc import sheet as sheets  # noqa: E402
from urc.missions import autonomy  # noqa: E402

RESEARCH = SIM_DIR / "data" / "research"
REPORT = RESEARCH / "realism_report.json"
CONTACT_SHEET = RESEARCH / "realism_contact_sheet.jpg"
URC_WORLDS = ("urc_delivery", "urc_astrobiology", "urc_equipment_servicing", "urc_autonomy")
WORLD_TOLERANCE = 0.30  # design 1
SLAB_TOLERANCE, COVER_TOLERANCE = 0.30, 0.25  # design 11
PALETTE_DELTA_E = 10.0  # design 11
DELTA_E = 5.0  # design 1: colour map vs boosted NAIP, render vs colour map (smoothed medians)
SPIN_TOLERANCE = 0.04  # design 6.9
WINDOW_M = 300.0  # the real crops' window
TILE_PX = 400  # each crop on the sheet, as the real sheet's
ROUGH = ("badland_slope", "block_field", "sand_sheet", "clay_crust")  # what the second window of a world shows


def roughness():
    out = {}
    for world in realism.SYNTHETIC:
        types = realism.type_roughness(world)
        for scales in types.values():
            for r in scales.values():
                r["met"] = r["target"][0] <= r["median"] <= r["target"][2]
        whole = realism.world_roughness(world)
        for r in whole.values():
            r["error"] = [r["p50"] / r["real"][0] - 1, r["p90"] / r["real"][1] - 1]
            r["met"] = all(abs(e) < WORLD_TOLERANCE for e in r["error"])
        out[world] = {"types": types, "natural_ground": whole}
    return out


def clutter():
    slabs = realism.slab_pools()
    for key, pool in slabs.items():
        area = pool["area_m2"]
        for d, (observed, expected) in list(pool["per_100m2"].items()):
            sigma = math.sqrt(expected * area / 100) / area * 100
            pool["per_100m2"][d] = {"observed": observed, "expected": expected,
                                    "met": abs(observed - expected) <= max(SLAB_TOLERANCE * expected, 3 * sigma)}
    slabs["block_field"]["cover_met"] = abs(slabs["block_field"]["cover_1_7"] / realism.BLOCK_COVER - 1) \
        < COVER_TOLERANCE
    shrubs = {world: {key: {"count": c, "expected": e, "met": abs(c - e) <= 4 * math.sqrt(e) + 1}
                      for key, (c, e) in realism.shrub_densities(world).items()} for world in realism.SYNTHETIC}
    imaged = realism.sheet("urc_autonomy")["imaged_shrubs"]
    shrubs["urc_autonomy"] = {"detected_per_ha": imaged["detected"] / (autonomy.SIZE ** 2 / 1e4),
                              "meshed": imaged["meshed"]}
    return {"slabs": slabs, "shrubs": shrubs}


def colour():
    palettes = {world: {key: {"delta_e": de, "met": de < PALETTE_DELTA_E}
                        for key, de in realism.palette_delta_e(world).items()} for world in realism.SYNTHETIC}
    de, slope = realism.autonomy_against_naip()
    bands = realism.by_slope(de, slope)
    return {"palettes": palettes, "autonomy_vs_boosted_naip": {
        "by_slope": bands, "met": bands["0-5"]["median"] < DELTA_E,
        "note": "smoothed over 2 m; judged on ground under 5 deg, which de-shading leaves alone"}}


def renders():
    import render_map  # noqa: E402  (gz-transport only inside its FlyServer)
    out = {}
    for world in URC_WORLDS:
        world_path = SIM_DIR / "worlds" / f"{world}.sdf"
        state, changed = render_map.status(world_path)
        image_path, _ = render_map.map_paths(world_path)
        if state == "missing":
            out[world] = {"map": state}
            continue
        picture = np.asarray(Image.open(image_path).convert("RGB"))
        reference, _ = realism.colour_map(world)
        de, slope = realism.render_against(world, picture, reference)
        bands = realism.by_slope(de, slope)
        entry = {"map": state, "changed": changed[:10], "vs_colour_map": {
            "median": float(np.median(de)), "p90": float(np.percentile(de, 90)), "by_slope": bands,
            "exposure": realism.exposure(picture, reference), "met": float(np.median(de)) < DELTA_E}}
        if world == "urc_autonomy":
            hf = realism.world_terrain(world)
            s = realism.sheet(world)
            origin = geo.Origin(s["origin"]["lat"], s["origin"]["lon"], s["origin"]["alt"])
            naip = appearance.resample_raster(autonomy.NAIP_PATH, origin, hf.size, picture.shape[0] // 2)
            raw = np.moveaxis(textures.srgb_to_linear(naip[:3]), 0, -1)
            de, slope = realism.render_against(world, picture, raw)
            entry["vs_raw_naip"] = {"median": float(np.median(de)), "p90": float(np.percentile(de, 90)),
                                    "by_slope": realism.by_slope(de, slope),
                                    "exposure": realism.exposure(picture, raw), "note": "for information"}
        out[world] = entry
    return out


def spins():
    """Fresh spin ratio per catalogue ground (wz 1 rad/s, mean over 3-6 s) on DART's Dantzig ("fresh") and PGS
    ("fresh_pgs": the solver of four of the five shipped worlds), and a 10 s sand spin's ratio per second under
    each dig-in preset."""
    sys.path.insert(0, str(SIM_DIR / "tests"))
    import simulate  # noqa: E402  (sets the Gazebo environment)
    hf = terrain.Heightfield(32.0, 65)
    keys = [k for k, t in terrains.TYPES.items() if t.traction is not None]
    rows = [simulate.ground_row(i, k, terrains.traction(terrains.TYPES[k]), terrains.TYPES[k].appearance.dust)
            for i, k in enumerate(keys)]

    def run(seconds, key, params, row=None, solver=None):
        index = keys.index(key)
        table = [row if r["key"] == key and row else r for r in rows]
        raster = np.full((hf.n, hf.n), index, np.uint8)
        with simulate.ground_world(hf, raster, table, params=params, solver=solver) as world:
            s = simulate.simulate(seconds, world=world, cmd=[(0.0, 0.0, 0.0), (0.5, 0.0, 1.0)], trace_every=10)
        return s.trace

    def fresh_spins(solver):
        out = {}
        for key in keys:
            trace = run(6.0, key, simulate.physical(dig=False), solver=solver)
            measured = float(trace[(trace[:, 0] >= 3.0) & (trace[:, 0] < 6.0), 7].mean())
            expected = simulate.spin_ratio(terrains.TYPES[key].traction)
            out[key] = {"measured": measured, "closed_form": expected,
                        "met": abs(measured - expected) <= SPIN_TOLERANCE}
        return out

    fresh, fresh_pgs = fresh_spins(None), fresh_spins("pgs")
    dig = {}
    for preset in ("strong", "mild"):
        row = simulate.ground_row(keys.index("sand"), "sand", terrains.traction(terrains.SAND, preset), 0.8)
        trace = run(10.5, "sand", simulate.physical(), row)
        dig[preset] = [float(trace[(trace[:, 0] >= t) & (trace[:, 0] < t + 1), 7].mean())
                       for t in np.arange(0.5, 10.5, 1.0)]
    return {"fresh": fresh, "fresh_pgs": fresh_pgs, "sand_dig_in": dig,
            "note": "flat ground of one type, mu noise on, Dantzig (fresh_pgs: PGS); dig_in: yaw ratio in each "
                    "second of a spin (Dantzig)"}


def hillshade(hf, window, n):
    """Grey hillshade (sun from the north-west at 45 deg, as the real sheet's) of a window [x0, y0, size]."""
    x0, y0, size = window
    xs = x0 + (np.arange(n) + 0.5) / n * size
    ys = y0 + size - (np.arange(n) + 0.5) / n * size
    X, Y = np.meshgrid(xs, ys)
    z = hf.height(X, Y)
    d = size / n
    gy, gx = np.gradient(z, -d, d)
    nx, ny, nz = -gx, -gy, np.ones_like(z)
    sun = np.array([-1.0, 1.0, math.sqrt(2.0)]) / 2.0
    shade = (nx * sun[0] + ny * sun[1] + nz * sun[2]) / np.sqrt(nx * nx + ny * ny + 1)
    return np.clip(shade * 255, 0, 255).astype(np.uint8)


def windows(world, hf):
    """Two windows [x0, y0, size] of a world: around the rover start, and the one richest in rough ground."""
    size = min(WINDOW_M, hf.size)
    half = hf.size / 2
    start = realism.sheet(world)["rover_start"]

    def clamp(x):
        return float(np.clip(x, -half, half - size))

    out = [(clamp(start["x"] - size / 2), clamp(start["y"] - size / 2), size)]
    if size < hf.size:
        ground = sheets.ground(realism.sheet(world), sheets.path(world))
        keys = {t["key"]: t["index"] for t in ground.info["types"]}
        rough = np.isin(ground.raster, [keys[k] for k in ROUGH if k in keys]).astype(np.float32)
        k = max(1, int(round(size / hf.res)))
        score = cv2.boxFilter(rough, -1, (k, k), normalize=True)
        i, j = np.unravel_index(np.argmax(score), score.shape)
        out.append((clamp(-half + j * hf.res - size / 2), clamp(half - i * hf.res - size / 2), size))
    return out


def contact_sheet():
    import render_map  # noqa: E402
    columns = []
    for world in URC_WORLDS:
        world_path = SIM_DIR / "worlds" / f"{world}.sdf"
        image_path, _ = render_map.map_paths(world_path)
        if not image_path.is_file():
            continue
        picture = Image.open(image_path).convert("RGB")
        hf = realism.world_terrain(world)
        scale = picture.size[0] / hf.size
        for window in windows(world, hf):
            x0, y0, size = window
            box = (int((x0 + hf.size / 2) * scale), int((hf.size / 2 - y0 - size) * scale),
                   int((x0 + size + hf.size / 2) * scale), int((hf.size / 2 - y0) * scale))
            top = picture.crop(box).resize((TILE_PX, TILE_PX), Image.LANCZOS)
            bottom = Image.fromarray(hillshade(hf, window, TILE_PX)).convert("RGB")
            ImageDraw.Draw(top).text((6, 4), f"{world.removeprefix('urc_')} ({x0:.0f}, {y0:.0f}) {size:.0f} m",
                                     fill=(255, 0, 0))
            columns.append((top, bottom))
    if not columns:
        return None
    rows = 2 * math.ceil(len(columns) / 5)
    sheet = Image.new("RGB", (5 * TILE_PX, rows * TILE_PX + 30), (255, 255, 255))
    for k, (top, bottom) in enumerate(columns):
        x, y = (k % 5) * TILE_PX, (k // 5) * 2 * TILE_PX
        sheet.paste(top, (x, y))
        sheet.paste(bottom, (x, y + TILE_PX))
    ImageDraw.Draw(sheet).text((6, rows * TILE_PX + 8), "Rendered maps (tools/render_map.py) over the hillshade of "
                               "the same window; windows 300 m (Equipment Servicing: the whole 256 m). Compare "
                               "mdrs_surface_types_crops.jpg.", fill=(0, 0, 0))
    sheet.save(CONTACT_SHEET, quality=88)
    return CONTACT_SHEET


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--no-physics", action="store_true", help="skip the spin runs (headless Gazebo)")
    parser.add_argument("--no-render", action="store_true", help="skip the rendered-map comparisons")
    args = parser.parse_args()
    started = time.time()
    report = {"format": "rover-realism/1", "date": time.strftime("%Y-%m-%d"), "dig_preset": terrains.DIG,
              "roughness": roughness(), "clutter": clutter(), "colour": colour()}
    if not args.no_render:
        report["render"] = renders()
        sheet = contact_sheet()
        report["contact_sheet"] = sheet and str(sheet.relative_to(SIM_DIR))
    if not args.no_physics:
        report["spin"] = spins()
    report["seconds"] = round(time.time() - started, 1)
    REPORT.write_text(json.dumps(report, indent=1, default=float) + "\n")
    print(f"wrote {REPORT.relative_to(SIM_DIR.parent)} in {report['seconds']} s")


if __name__ == "__main__":
    main()
