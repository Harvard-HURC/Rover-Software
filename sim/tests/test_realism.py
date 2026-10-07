#!/usr/bin/env python3
"""The realism checks of the design (docs/superpowers/specs/2026-10-06-urc-realism-design.md,
sections 1 and 11) on the generated worlds (pixi run sim-worlds; no Gazebo, no GPU):

- Roughness: each synthetic world's ground types against the same types of
  the real MDRS lidar, and its natural ground as a whole against the real
  Autonomy square (sim/data/research/terrain_targets.json; plane-detrended
  RMS height in square windows of 4, 8 and 16 m).
- Clutter: the block fields' and badland edges' slab size-frequency against
  the measured tables, shrub densities against their recipes.
- Colour: each synthetic world's colour map against the palettes of its
  ground; Autonomy's against the boosted NAIP orthophoto it drapes, and its
  ground against the soil map.

Rendered pictures against the colour maps need the GPU: that is the realism
report's (design 11, WS-V).
"""
import json
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import cv2
import numpy as np
from PIL import Image

from worldfiles import MODELS, WORLDS, sheet, terrain as world_terrain

from urc import appearance, geo, landscape, terrain, terrains, textures  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import sheet as sheets  # noqa: E402
from urc.missions import astrobiology, autonomy, delivery, equipment  # noqa: E402

TARGETS = json.loads((landscape.RELIEF_DIR.parent / "research" / "terrain_targets.json").read_text())
SCALES = (4, 8, 16)  # [m] the window sizes judged (design 5.4: 2 m is lidar noise, 32 m macro shape)
SYNTHETIC = {"urc_delivery": delivery, "urc_astrobiology": astrobiology, "urc_equipment_servicing": equipment}
WORLD_TOLERANCE = 0.30  # a world's natural-ground p50 and p90 within 30 % of the real square's (design 1)
FLAT_SITE_CM = 2.0  # [cm] Equipment Servicing's 4 m p50 at most ("relatively flat", design 5.2)
PALETTE_DELTA_E = 10.0  # a type's median colour within this CIE76 of its palette (design 11)
NAIP_DELTA_E = 5.0  # Autonomy's colour map against boosted NAIP, smoothed, outside de-shaded ground (design 1)
NAIP_SMOOTH_M = 2.0  # [m] the smoothing of that comparison (A: a few texels; NAIP's pixels are 0.6 m)
FLAT_DEG = 5.0  # [deg] ground this gentle is barely de-shaded: there the colour map is NAIP's own
SLAB_TOLERANCE = 0.30  # slab counts within 30 % of the table (design 11), or two Poisson sigma on small areas
COVER_TOLERANCE = 0.25  # cover by 1-7 m slabs within 25 % of the table's (design 11)
SOIL_SHARE = 0.02  # Autonomy's ground shares within 2 points of the soil map painted on the real square


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


class Roughness(unittest.TestCase):
    """Design 1 and 5.4: the synthetic ground is as rough as MDRS's own."""

    def test_each_type_as_rough_as_its_real_counterpart(self):
        """Every ground type with a real counterpart, on at least
        terrain_targets' min_windows windows that are 80 % or more that type
        and wholly natural: its median RMS within the real p25-p75."""
        for world in SYNTHETIC:
            hf, raster, keys, natural = natural_ground(world)
            for scale in SCALES:
                rms, whole, share = window_stats(hf, raster, natural, scale)
                for index, key in keys.items():
                    target = TARGETS["types"].get(key, {}).get(str(scale))
                    if target is None:
                        continue
                    inside = whole & (share(index) >= TARGETS["window_share"])
                    if inside.sum() < TARGETS["min_windows"]:
                        continue
                    median = float(np.median(rms[inside]))
                    with self.subTest(world=world, type=key, scale=scale):
                        self.assertTrue(target[0] <= median <= target[2],
                                        f"{median:.2f} cm not in [{target[0]}, {target[2]}] ({inside.sum()} windows)")

    def test_worlds_as_rough_as_the_real_square(self):
        """Delivery's and Astrobiology's natural ground: p50 and p90 at 4, 8
        and 16 m within 30 % of the real Autonomy square's (design 1)."""
        for world in ("urc_delivery", "urc_astrobiology"):
            hf, raster, _, natural = natural_ground(world)
            for scale in SCALES:
                rms, whole, _ = window_stats(hf, raster, natural, scale)
                p50, p90 = np.percentile(rms[whole], [50, 90])
                real = TARGETS["world"]["whole"][str(scale)]
                for ours, theirs, label in ((p50, real[1], "p50"), (p90, real[3], "p90")):
                    with self.subTest(world=world, scale=scale, percentile=label):
                        self.assertLess(abs(ours / theirs - 1), WORLD_TOLERANCE, f"{ours:.2f} cm against {theirs}")

    def test_engineered_ground_kept_flat(self):
        """The relief leaves the pads, the graded ways and the engineered
        features (a mission's RELIEF: the keep-flat mask of design 5.4) within
        2 cm of the macro shape the mission designed."""
        for world, mission in SYNTHETIC.items():
            with mock.patch.object(mission, "add_relief", lambda hf, *args, **kwargs: hf):
                macro = mission.make_terrain()
            hf, _, _, _ = natural_ground(world)
            flat = landscape.keep_flat(macro, **mission.RELIEF) == 0.0
            change = np.abs((hf.z - hf.z.min()) - (macro.z - macro.z.min()))
            offset = np.median(change[flat])  # the heightmaps' zeros differ by the relief's lowest point
            with self.subTest(world=world):
                self.assertGreater(flat.sum(), 1000)
                self.assertLess(np.percentile(np.abs(change[flat] - offset), 99.9), 0.02)

    def test_equipment_site_relatively_flat(self):
        """Rule 1.d's "relatively flat" site: 4 m roughness p50 at most 2 cm,
        about the real clay crust's median (design 5.2)."""
        hf, raster, _, natural = natural_ground("urc_equipment_servicing")
        rms, whole, _ = window_stats(hf, raster, natural, 4)
        self.assertLessEqual(float(np.median(rms[whole])), FLAT_SITE_CM)


class Clutter(unittest.TestCase):
    """Design 5.5 and 11: the clutter the recipes placed in each world."""

    def test_slab_size_frequency(self):
        """Block fields and badland edges, pooled over the synthetic worlds:
        N(>=1), N(>=2), N(>=4 m) per 100 m2 as the measured tables less the
        features of 7 m and more (ledges and macro shape), within 30 % or
        three Poisson sigma on a small pool; and the block fields' cover by
        1-7 m slabs within 25 % of the table's 8.6 %."""
        recipes = {"block_field": terrains.BLOCK_SLABS, "badland_slope": terrains.BADLAND_SLABS}
        pools = {key: {"area": 0.0, "cover": 0.0, "counts": {}} for key in recipes}
        for world in SYNTHETIC:
            for key, entry in sheet(world)["slabs"]["slabs"]["by_type"].items():
                if key not in recipes:
                    continue
                pool = pools[key]
                pool["area"] += entry["area_m2"]
                pool["cover"] += entry["cover_1_7"] * entry["area_m2"]
                for d, density in entry["per_100m2"].items():
                    pool["counts"][d] = pool["counts"].get(d, 0.0) + density * entry["area_m2"] / 100
        for key, pool in pools.items():
            recipe, area = recipes[key], pool["area"]
            self.assertGreater(area, 10_000, key)
            for d, count in pool["counts"].items():
                expected = float(landscape.slab_count(recipe, float(d)) - landscape.slab_count(recipe, recipe.d_max))
                observed = count / area * 100
                sigma = math.sqrt(expected * area / 100) / area * 100  # Poisson, per 100 m2
                with self.subTest(type=key, d=d):
                    self.assertLessEqual(abs(observed - expected), max(SLAB_TOLERANCE * expected, 3 * sigma),
                                         f"N(>={d}) {observed:.4f} per 100 m2 against {expected:.4f}")
        cover = pools["block_field"]["cover"] / pools["block_field"]["area"]
        self.assertLess(abs(cover / 0.086 - 1), COVER_TOLERANCE, cover)

    def test_shrub_densities(self):
        """Recipe shrubs per type: as many as the recipe's density over the
        type's ground (Poisson)."""
        for world in ("urc_delivery", "urc_astrobiology", "urc_equipment_servicing"):
            for key, entry in sheet(world)["shrub_density"].items():
                expected = terrains.TYPES[key].clutter.shrubs.per_ha * entry["area_m2"] / 1e4
                with self.subTest(world=world, type=key):
                    self.assertLessEqual(abs(entry["count"] - expected), 4 * math.sqrt(expected) + 1,
                                         f"{entry['count']} shrubs against {expected:.0f}")

    def test_autonomy_shrubs_from_naip(self):
        """Autonomy's shrubs are NAIP's (D10): 38 per hectare were found over
        the square (WS-A, M), meshes only where the rover works."""
        s = sheet("urc_autonomy")["imaged_shrubs"]
        density = s["detected"] / (autonomy.SIZE ** 2 / 1e4)
        self.assertTrue(25.0 < density < 60.0, density)
        self.assertLess(s["meshed"], s["detected"])


def colour_map(world):
    """A world's colour map as the render shows it (linear RGB, n x n x 3,
    float32) and the world-frame terrain: Terra's layer 0 is pre-compensated
    for the detail layers over it (appearance.terra_layers), so the
    compensation is undone from the terrain model's own layers and blends
    and the detail textures' means: O = a_0 O' + sum_i a_i m_i."""
    s = sheet(world)
    hf = world_terrain(world)
    layer0 = textures.srgb_to_linear(np.asarray(Image.open(WORLDS / s["terrain"]["colour_map"]).convert("RGB")))
    name = Path(s["terrain"]["colour_map"]).parent.name
    visual = ET.parse(MODELS / name / "model.sdf").getroot().find(".//visual/geometry/heightmap")
    n = layer0.shape[0]
    heights = appearance.at_texels(hf, n)
    acc = np.zeros_like(layer0)
    a0 = np.ones(heights.shape, np.float32)
    for texture, blend in zip(visual.findall("texture")[1:], visual.findall("blend")):
        diffuse = MODELS / texture.findtext("diffuse").removeprefix("model://")
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


class Colour(unittest.TestCase):
    """Design 5.7 and 11: the ground's colours."""

    def test_synthetic_colours_follow_the_palettes(self):
        """Each ground type covering at least 1 % of a synthetic world: its
        median texel within CIE76 10 of its palette (the banded badland,
        coloured by strata, aside)."""
        for world in SYNTHETIC:
            rgb, _ = colour_map(world)
            types, keys = texel_types(world, rgb.shape[0])
            for index, key in keys.items():
                texels = types == index
                if texels.mean() < 0.01 or key == "badland_slope":
                    continue
                median = textures.linear_to_srgb(np.median(rgb[texels], axis=0))
                with self.subTest(world=world, type=key):
                    self.assertLess(float(appearance.delta_e(median, terrains.TYPES[key].appearance.palette.base)),
                                    PALETTE_DELTA_E, median)

    def test_autonomy_is_naip(self):
        """Autonomy's colour map is the boosted NAIP 2024 orthophoto: on
        gentle ground (where de-shading leaves it), smoothed over
        NAIP_SMOOTH_M, the median CIE76 to boosted NAIP is within 5 (design 1;
        shadows and the spots under 3D shrubs are inpainted, so not every
        texel)."""
        rgb, hf = colour_map("urc_autonomy")
        n = rgb.shape[0]
        s = sheet("urc_autonomy")
        origin = geo.Origin(s["origin"]["lat"], s["origin"]["lon"], s["origin"]["alt"])
        naip = appearance.resample_raster(autonomy.NAIP_PATH, origin, hf.size, n)
        boosted = appearance.NAIP2024_BOOST.apply(np.moveaxis(textures.srgb_to_linear(naip[:3]), 0, -1))
        sigma = NAIP_SMOOTH_M / (hf.size / n)
        ours = cv2.GaussianBlur(rgb.astype(np.float32), (0, 0), sigma)
        theirs = cv2.GaussianBlur(np.clip(boosted, 0, 1).astype(np.float32), (0, 0), sigma)
        gentle = appearance.at_texels(hf, n, hf.slope_map()) < FLAT_DEG
        step = 4  # every 4th texel each way: enough for a median, a quarter of the memory
        de = appearance.delta_e(textures.linear_to_srgb(ours[::step, ::step]),
                                textures.linear_to_srgb(theirs[::step, ::step]))[gentle[::step, ::step]]
        self.assertLess(float(np.median(de)), NAIP_DELTA_E, np.percentile(de, [50, 90]))

    def test_autonomy_ground_is_the_soil_map(self):
        """Autonomy's ground raster: the soil map by slope on the real
        square (terrain_targets.json's cover of the Autonomy square), the
        mission's zones besides."""
        share = sheet("urc_autonomy")["terrain"]["ground_share"]
        for key, real in TARGETS["cover"]["autonomy_square"].items():
            with self.subTest(type=key):
                self.assertAlmostEqual(share.get(key, 0.0), real, delta=SOIL_SHARE)


if __name__ == "__main__":
    unittest.main()
