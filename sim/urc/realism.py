"""The realism measures of the design (docs/superpowers/specs/2026-10-06-urc-realism-design.md,
sections 1, 5.4-5.7 and 11) on generated worlds: ground roughness against
the MDRS lidar, slab size-frequency, shrub density, colour against the
palettes and NAIP, and a rendered orthophoto against its colour map. One
implementation for the checks (tests/test_realism.py) and the report
(tools/realism_report.py), which reads the same numbers into
sim/data/research/realism_report.json.
"""
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from . import appearance, geo, landscape, terrain, terrains, textures
from . import sheet as sheets
from .missions import astrobiology, autonomy, delivery, equipment

TARGETS = json.loads((landscape.RELIEF_DIR.parent / "research" / "terrain_targets.json").read_text())
SCALES = (4, 8, 16)  # [m] the window sizes judged (design 5.4: 2 m is lidar noise, 32 m macro shape)
SYNTHETIC = {"urc_delivery": delivery, "urc_astrobiology": astrobiology, "urc_equipment_servicing": equipment}
SLAB_RECIPES = {"block_field": terrains.BLOCK_SLABS, "badland_slope": terrains.BADLAND_SLABS}
BLOCK_COVER = 0.086  # cover by 1-7 m blocks in the block fields (M, design 1)
NAIP_SMOOTH_M = 2.0  # [m] smoothing of colour comparisons (A: a few texels; NAIP's pixels are 0.6 m)
SLOPE_BANDS = ((0.0, 5.0), (5.0, 10.0), (10.0, 20.0), (20.0, 90.0))  # [deg]


def models_dir(world):
    return sheets.path(world).parent.parent / "models"


def sheet(world):
    return sheets.load(sheets.path(world))


def world_terrain(world):
    """The world's terrain from its sheet: a Heightfield in world coordinates."""
    return sheets.terrain(sheet(world), sheets.path(world))


def natural_ground(world):
    """(terrain, ground raster, legend keys by index, natural mask) of a
    synthetic world: its heightmap on the layout grid (the mission's
    CENTER), its ground.png, and the samples its relief was not kept off
    (landscape.keep_flat of the mission's RELIEF: pads, routes, engineered
    features)."""
    mission = SYNTHETIC[world]
    s = sheet(world)
    hf = world_terrain(world)
    layout = terrain.Heightfield(hf.size, hf.n, hf.z, mission.CENTER)
    ground = sheets.ground(s, sheets.path(world))
    keys = {t["index"]: t["key"] for t in ground.info["types"]}
    return layout, ground.raster, keys, landscape.keep_flat(layout, **mission.RELIEF) >= 0.999


def window_stats(hf, raster, natural, scale):
    """Window RMS [cm], the windows wholly on natural ground, and a
    function giving each window's share of a raster index."""
    rms = landscape.window_rms(hf.z, hf.res, scale) * 100
    whole = landscape.window_share(natural.astype(np.uint8), 1, hf.res, scale) == 1.0
    return rms, whole, lambda index: landscape.window_share(raster, index, hf.res, scale)


def type_roughness(world):
    """{type: {scale: {median, windows, target [p25, p50, p75]}}}: every
    ground type of a synthetic world with a real counterpart, over windows
    that are at least terrain_targets' window_share that type and wholly
    natural, where there are min_windows of them."""
    hf, raster, keys, natural = natural_ground(world)
    out = {}
    for scale in SCALES:
        rms, whole, share = window_stats(hf, raster, natural, scale)
        for index, key in keys.items():
            target = TARGETS["types"].get(key, {}).get(str(scale))
            if target is None:
                continue
            inside = whole & (share(index) >= TARGETS["window_share"])
            if inside.sum() < TARGETS["min_windows"]:
                continue
            out.setdefault(key, {})[scale] = {"median": float(np.median(rms[inside])), "windows": int(inside.sum()),
                                              "target": list(target[:3])}
    return out


def world_roughness(world):
    """{scale: {p50, p90, real [p50, p90]}}: a synthetic world's natural
    ground against the real Autonomy square's (design 1: within 30 %)."""
    hf, raster, _, natural = natural_ground(world)
    out = {}
    for scale in SCALES:
        rms, whole, _ = window_stats(hf, raster, natural, scale)
        p50, p90 = np.percentile(rms[whole], [50, 90])
        real = TARGETS["world"]["whole"][str(scale)]
        out[scale] = {"p50": float(p50), "p90": float(p90), "real": [real[1], real[3]]}
    return out


def slab_pools(worlds=tuple(SYNTHETIC)):
    """The recipe slabs pooled over worlds, per type with a slab table:
    {type: {area_m2, cover_1_7, per_100m2: {d: (observed, expected)}}}.
    Expected: the measured table less the features of 7 m and more, which
    are ledges and macro shape (design 1)."""
    pools = {key: {"area": 0.0, "cover": 0.0, "counts": {}} for key in SLAB_RECIPES}
    for world in worlds:
        for key, entry in sheet(world)["slabs"]["slabs"]["by_type"].items():
            if key not in SLAB_RECIPES:
                continue
            pool = pools[key]
            pool["area"] += entry["area_m2"]
            pool["cover"] += entry["cover_1_7"] * entry["area_m2"]
            for d, density in entry["per_100m2"].items():
                pool["counts"][d] = pool["counts"].get(d, 0.0) + density * entry["area_m2"] / 100
    out = {}
    for key, pool in pools.items():
        recipe, area = SLAB_RECIPES[key], pool["area"]
        if area <= 0:
            continue
        out[key] = {"area_m2": area, "cover_1_7": pool["cover"] / area, "per_100m2": {
            d: (count / area * 100,
                float(landscape.slab_count(recipe, float(d)) - landscape.slab_count(recipe, recipe.d_max)))
            for d, count in pool["counts"].items()}}
    return out


def shrub_densities(world):
    """{type: (count, expected)}: recipe shrubs against the recipe's density over the type's ground."""
    return {key: (entry["count"], terrains.TYPES[key].clutter.shrubs.per_ha * entry["area_m2"] / 1e4)
            for key, entry in sheet(world)["shrub_density"].items()}


def colour_map(world):
    """A world's colour map as the render shows it (linear RGB, n x n x 3,
    float32) and the world-frame terrain: Terra's layer 0 is pre-compensated
    for the detail layers over it (appearance.terra_layers), so the
    compensation is undone from the terrain model's own layers and blends
    and the detail textures' means: O = a_0 O' + sum_i a_i m_i."""
    s, models = sheet(world), models_dir(world)
    hf = world_terrain(world)
    layer0 = textures.srgb_to_linear(np.asarray(Image.open(sheets.path(world).parent / s["terrain"]["colour_map"])
                                                .convert("RGB")))
    name = Path(s["terrain"]["colour_map"]).parent.name
    visual = ET.parse(models / name / "model.sdf").getroot().find(".//visual/geometry/heightmap")
    heights = appearance.at_texels(hf, layer0.shape[0])
    acc = np.zeros_like(layer0)
    a0 = np.ones(heights.shape, np.float32)
    for texture, blend in zip(visual.findall("texture")[1:], visual.findall("blend")):
        diffuse = models / texture.findtext("diffuse").removeprefix("model://")
        mean = textures.srgb_to_linear(np.asarray(Image.open(diffuse).convert("RGB"))).reshape(-1, 3).mean(axis=0)
        low, fade = float(blend.findtext("min_height")), float(blend.findtext("fade_dist"))
        w = terrain.smoothstep(low, low + fade, heights).astype(np.float32)
        acc = acc * (1 - w[..., None]) + w[..., None] * mean
        a0 *= 1 - w
    return a0[..., None] * layer0 + acc, hf


def texel_types(world, n):
    """The ground raster's index at every texel of an n x n colour map
    (nearest sample, as the colour map is baked) and the legend keys."""
    ground = sheets.ground(sheet(world), sheets.path(world))
    m = ground.raster.shape[0]
    index = np.clip(np.floor((np.arange(n) + 0.5) / n * (m - 1) + 0.5).astype(int), 0, m - 1)
    return ground.raster[np.ix_(index, index)], {t["index"]: t["key"] for t in ground.info["types"]}


def palette_delta_e(world, min_share=0.01):
    """{type: CIE76}: each ground type covering at least min_share of a
    world, its median colour-map texel against its palette (the banded
    badland, coloured by strata, aside)."""
    rgb, _ = colour_map(world)
    types, keys = texel_types(world, rgb.shape[0])
    out = {}
    for index, key in keys.items():
        texels = types == index
        if texels.mean() < min_share or key == "badland_slope":
            continue
        median = textures.linear_to_srgb(np.median(rgb[texels], axis=0))
        out[key] = float(appearance.delta_e(median, terrains.TYPES[key].appearance.palette.base))
    return out


def smoothed(rgb, size, smooth_m=NAIP_SMOOTH_M):
    """Linear RGB blurred over smooth_m (Gaussian sigma), float32."""
    return cv2.GaussianBlur(rgb.astype(np.float32), (0, 0), smooth_m / (size / rgb.shape[0]))


def by_slope(de, slope):
    """{"<lo>-<hi> deg": {median, p90, share}} of a CIE76 field by slope band."""
    out = {}
    for lo, hi in SLOPE_BANDS:
        band = (slope >= lo) & (slope < hi)
        if band.any():
            out[f"{lo:g}-{hi:g}"] = {"median": float(np.median(de[band])), "p90": float(np.percentile(de[band], 90)),
                                     "share": float(band.mean())}
    return out


def autonomy_against_naip(step=4):
    """Autonomy's colour map against the boosted NAIP 2024 it drapes, both
    smoothed over NAIP_SMOOTH_M, every step-th texel: (CIE76 field, slope
    [deg] at those texels). De-shading changes slopes on purpose, so judge
    gentle ground (design 1: colour map vs boosted NAIP)."""
    rgb, hf = colour_map("urc_autonomy")
    n = rgb.shape[0]
    s = sheet("urc_autonomy")
    origin = geo.Origin(s["origin"]["lat"], s["origin"]["lon"], s["origin"]["alt"])
    naip = appearance.resample_raster(autonomy.NAIP_PATH, origin, hf.size, n)
    boosted = appearance.NAIP2024_BOOST.apply(np.moveaxis(textures.srgb_to_linear(naip[:3]), 0, -1))
    ours, theirs = smoothed(rgb, hf.size), smoothed(np.clip(boosted, 0, 1), hf.size)
    de = appearance.delta_e(textures.linear_to_srgb(ours[::step, ::step]),
                            textures.linear_to_srgb(theirs[::step, ::step]))
    return de, appearance.at_texels(hf, n, hf.slope_map())[::step, ::step]


def render_against(world, picture, reference, step=4):
    """A rendered orthophoto (sRGB uint8, row 0 north, the terrain square,
    tools/render_map.py) against a reference of the same square (linear RGB,
    any size), both resampled to the picture's grid and smoothed over
    NAIP_SMOOTH_M: (CIE76 field, slope [deg]) every step-th pixel."""
    hf = world_terrain(world)
    n = picture.shape[0]
    reference = cv2.resize(reference.astype(np.float32), (n, n), interpolation=cv2.INTER_AREA)
    ours = smoothed(textures.srgb_to_linear(picture), hf.size)
    theirs = smoothed(reference, hf.size)
    de = appearance.delta_e(textures.linear_to_srgb(ours[::step, ::step]),
                            textures.linear_to_srgb(theirs[::step, ::step]))
    return de, appearance.at_texels(hf, n, hf.slope_map())[::step, ::step]


def exposure(picture, reference):
    """The single gain on linear RGB that best maps a reference onto a
    rendered picture (least squares on the luminance medians): how much the
    sun and ambient brighten or darken the ground overall, for information."""
    ours = textures.luminance(textures.srgb_to_linear(picture))
    theirs = textures.luminance(cv2.resize(reference.astype(np.float32), picture.shape[1::-1],
                                           interpolation=cv2.INTER_AREA))
    return float(np.median(ours) / max(np.median(theirs), 1e-6))

