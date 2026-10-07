#!/usr/bin/env python3
"""The performance budgets of the realism design (docs/superpowers/specs/
2026-10-06-urc-realism-design.md, section 10.3), by its method: CPU time per
1 ms physics step of plain `gz sim -s -r --iterations N` processes, minus a
one-step start-up run, the rover driving simulate.DRIVE_SCHEDULE from outside
the server (simulate.cpu_time_per_step). Cameras are stripped and the worlds
run as fast as they can (real_time_factor 0); CPU time does not depend on it.

Slow (several minutes) and sensitive to load: it runs only with ROVER_PERF=1
(pixi run sim-perf). Absolute numbers need a 1-minute load average below 4
(section 10.3); above that a world is skipped, with its numbers, not failed.
"""
import os
import re
import statistics
import unittest

from simulate import cpu_time_per_step
from worldfiles import SENSORS, WORLDS, temp_sdf

WORLD_NAMES = ("proving_ground", "urc_autonomy", "urc_astrobiology", "urc_delivery", "urc_equipment_servicing")
FLOOR = 1.1  # x real time, every world (section 10.3)
TARGET = 1.3  # x real time, reported only
STARTUP_S = 30.0  # wall time to the first step (a one-step run: load, step, exit)
MEMORY_B = 3.5 * 2**30  # peak resident size of a server
MAX_LOAD = 4.0
ITERATIONS = 20_000  # 20 s of sim time, the schedule's straight, turns and arc
RUNS = 5


@unittest.skipUnless(os.environ.get("ROVER_PERF") == "1", "performance budgets: ROVER_PERF=1 (pixi run sim-perf)")
class Budgets(unittest.TestCase):
    def test_every_world(self):
        for name in WORLD_NAMES:
            with self.subTest(world=name):
                text = SENSORS.sub("", (WORLDS / f"{name}.sdf").read_text())
                text = re.sub(r"<real_time_factor>[^<]*</real_time_factor>", "<real_time_factor>0</real_time_factor>",
                              text, count=1)
                with temp_sdf(text, WORLDS) as path:  # beside the original, as worldfiles.world_copy does
                    cost = cpu_time_per_step({name: path}, ITERATIONS, RUNS)[name]
                rtf, load = cost.real_time_factor, statistics.median(cost.load)
                print(f"{name}: {cost.per_step * 1e3:.3f} ms/step = {rtf:.2f}x real time "
                      f"({'meets' if rtf >= TARGET else 'misses'} the {TARGET}x target), start-up "
                      f"{cost.startup_wall:.1f} s, peak {cost.peak_rss / 2**20:.0f} MB, load {load:.1f}", flush=True)
                self.assertLess(cost.startup_wall, STARTUP_S)
                self.assertLess(cost.peak_rss, MEMORY_B)
                if load >= MAX_LOAD:
                    self.skipTest(f"{name}: load average {load:.1f} >= {MAX_LOAD}, {rtf:.2f}x not judged")
                self.assertGreaterEqual(rtf, FLOOR, f"{name}: {rtf:.2f}x real time at load {load:.1f}")


if __name__ == "__main__":
    unittest.main()
