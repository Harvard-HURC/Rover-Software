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

The measures are urc/realism.py's, which tools/realism_report.py also
reports, with rendered pictures against the colour maps (GPU).
"""
import math
import unittest
from unittest import mock

import numpy as np

import worldfiles  # noqa: F401  (puts sim/ on the path)
from urc import landscape, realism  # noqa: E402
from urc.missions import autonomy  # noqa: E402

WORLD_TOLERANCE = 0.30  # a world's natural-ground p50 and p90 within 30 % of the real square's (design 1)
FLAT_SITE_CM = 2.0  # [cm] Equipment Servicing's 4 m p50 at most ("relatively flat", design 5.2)
PALETTE_DELTA_E = 10.0  # a type's median colour within this CIE76 of its palette (design 11)
NAIP_DELTA_E = 5.0  # Autonomy's colour map against boosted NAIP, smoothed, outside de-shaded ground (design 1)
FLAT_DEG = 5.0  # [deg] ground this gentle is barely de-shaded: there the colour map is NAIP's own
SLAB_TOLERANCE = 0.30  # slab counts within 30 % of the table (design 11), or three Poisson sigma on small areas
COVER_TOLERANCE = 0.25  # cover by 1-7 m slabs within 25 % of the table's 8.6 % (design 11)
SOIL_SHARE = 0.02  # Autonomy's ground shares within 2 points of the soil map painted on the real square


class Roughness(unittest.TestCase):
    """Design 1 and 5.4: the synthetic ground is as rough as MDRS's own."""

    def test_each_type_as_rough_as_its_real_counterpart(self):
        """Every ground type with a real counterpart, on at least
        terrain_targets' min_windows windows that are 80 % or more that type
        and wholly natural: its median RMS within the real p25-p75."""
        for world in realism.SYNTHETIC:
            for key, scales in realism.type_roughness(world).items():
                for scale, r in scales.items():
                    low, _, high = r["target"]
                    with self.subTest(world=world, type=key, scale=scale):
                        self.assertTrue(low <= r["median"] <= high,
                                        f"{r['median']:.2f} cm not in [{low}, {high}] ({r['windows']} windows)")

    def test_worlds_as_rough_as_the_real_square(self):
        """Delivery's and Astrobiology's natural ground: p50 and p90 at 4, 8
        and 16 m within 30 % of the real Autonomy square's (design 1)."""
        for world in ("urc_delivery", "urc_astrobiology"):
            for scale, r in realism.world_roughness(world).items():
                for ours, theirs, label in ((r["p50"], r["real"][0], "p50"), (r["p90"], r["real"][1], "p90")):
                    with self.subTest(world=world, scale=scale, percentile=label):
                        self.assertLess(abs(ours / theirs - 1), WORLD_TOLERANCE, f"{ours:.2f} cm against {theirs}")

    def test_engineered_ground_kept_flat(self):
        """The relief leaves the pads, the graded ways and the engineered
        features (a mission's RELIEF: the keep-flat mask of design 5.4) within
        2 cm of the macro shape the mission designed."""
        for world, mission in realism.SYNTHETIC.items():
            with mock.patch.object(mission, "add_relief", lambda hf, *args, **kwargs: hf):
                macro = mission.make_terrain()
            hf, _, _, _ = realism.natural_ground(world)
            flat = landscape.keep_flat(macro, **mission.RELIEF) == 0.0
            change = np.abs((hf.z - hf.z.min()) - (macro.z - macro.z.min()))
            offset = np.median(change[flat])  # the heightmaps' zeros differ by the relief's lowest point
            with self.subTest(world=world):
                self.assertGreater(flat.sum(), 1000)
                self.assertLess(np.percentile(np.abs(change[flat] - offset), 99.9), 0.02)

    def test_equipment_site_relatively_flat(self):
        """Rule 1.d's "relatively flat" site: 4 m roughness p50 at most 2 cm,
        about the real clay crust's median (design 5.2)."""
        hf, raster, _, natural = realism.natural_ground("urc_equipment_servicing")
        rms, whole, _ = realism.window_stats(hf, raster, natural, 4)
        self.assertLessEqual(float(np.median(rms[whole])), FLAT_SITE_CM)


class Clutter(unittest.TestCase):
    """Design 5.5 and 11: the clutter the recipes placed in each world."""

    def test_slab_size_frequency(self):
        """Block fields and badland edges, pooled over the synthetic worlds:
        N(>=1), N(>=2), N(>=4 m) per 100 m2 as the measured tables less the
        features of 7 m and more (ledges and macro shape), within 30 % or
        three Poisson sigma on a small pool; and the block fields' cover by
        1-7 m slabs within 25 % of the table's 8.6 %."""
        pools = realism.slab_pools()
        for key, pool in pools.items():
            area = pool["area_m2"]
            self.assertGreater(area, 10_000, key)
            for d, (observed, expected) in pool["per_100m2"].items():
                sigma = math.sqrt(expected * area / 100) / area * 100  # Poisson, per 100 m2
                with self.subTest(type=key, d=d):
                    self.assertLessEqual(abs(observed - expected), max(SLAB_TOLERANCE * expected, 3 * sigma),
                                         f"N(>={d}) {observed:.4f} per 100 m2 against {expected:.4f}")
        cover = pools["block_field"]["cover_1_7"]
        self.assertLess(abs(cover / realism.BLOCK_COVER - 1), COVER_TOLERANCE, cover)

    def test_shrub_densities(self):
        """Recipe shrubs per type: as many as the recipe's density over the
        type's ground (Poisson)."""
        for world in realism.SYNTHETIC:
            for key, (count, expected) in realism.shrub_densities(world).items():
                with self.subTest(world=world, type=key):
                    self.assertLessEqual(abs(count - expected), 4 * math.sqrt(expected) + 1,
                                         f"{count} shrubs against {expected:.0f}")

    def test_autonomy_shrubs_from_naip(self):
        """Autonomy's shrubs are NAIP's (D10): 38 per hectare were found over
        the square (WS-A, M), meshes only where the rover works."""
        s = realism.sheet("urc_autonomy")["imaged_shrubs"]
        density = s["detected"] / (autonomy.SIZE ** 2 / 1e4)
        self.assertTrue(25.0 < density < 60.0, density)
        self.assertLess(s["meshed"], s["detected"])


class Colour(unittest.TestCase):
    """Design 5.7 and 11: the ground's colours."""

    def test_synthetic_colours_follow_the_palettes(self):
        """Each ground type covering at least 1 % of a synthetic world: its
        median texel within CIE76 10 of its palette (the banded badland,
        coloured by strata, aside)."""
        for world in realism.SYNTHETIC:
            for key, de in realism.palette_delta_e(world).items():
                with self.subTest(world=world, type=key):
                    self.assertLess(de, PALETTE_DELTA_E)

    def test_autonomy_is_naip(self):
        """Autonomy's colour map is the boosted NAIP 2024 orthophoto: on
        gentle ground (where de-shading leaves it), smoothed over
        realism.NAIP_SMOOTH_M, the median CIE76 to boosted NAIP is within 5
        (design 1; shadows and the spots under 3D shrubs are inpainted, so
        not every texel)."""
        de, slope = realism.autonomy_against_naip()
        gentle = de[slope < FLAT_DEG]
        self.assertLess(float(np.median(gentle)), NAIP_DELTA_E, np.percentile(gentle, [50, 90]))

    def test_autonomy_ground_is_the_soil_map(self):
        """Autonomy's ground raster: the soil map by slope on the real
        square (terrain_targets.json's cover of the Autonomy square), the
        mission's zones besides."""
        share = realism.sheet("urc_autonomy")["terrain"]["ground_share"]
        for key, real in realism.TARGETS["cover"]["autonomy_square"].items():
            with self.subTest(type=key):
                self.assertAlmostEqual(share.get(key, 0.0), real, delta=SOIL_SHARE)


if __name__ == "__main__":
    unittest.main()
