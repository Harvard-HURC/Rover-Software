#!/usr/bin/env python3
"""The performance budgets of the realism design (docs/superpowers/specs/
2026-10-06-urc-realism-design.md, section 10.3): every world's physics at
least 1.1x real time (target 1.3x), start-up under 30 s, a server under
3.5 GB. Measured by simulate.cpu_time_per_step: plain `gz sim -s -r
--iterations N` processes minus a one-step start-up run, the rover driving
simulate.DRIVE_SCHEDULE from outside the server, cameras stripped,
real_time_factor 0.

The floor is judged on the wall-clock real-time factor, which an otherwise
idle machine shows: the design's CPU-time measure also counts the server's
helper threads and reads 13-29 % lower (sim/data/research/gates.json, G5);
it is printed beside it. Slow (several minutes) and sensitive to load: it
runs only with ROVER_PERF=1 (pixi run sim-perf), and a world measured while
other processes kept the 1-minute load average at 4 or more (the load minus
the cores the measured server itself keeps busy, CPU over wall time, about
1.3) is skipped with its numbers, not failed.
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
MAX_LOAD = 4.0  # from other processes
ITERATIONS = 20_000  # 20 s of sim time: the schedule's straight, turns and arc
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
                rtf = cost.wall_real_time_factor
                load = statistics.median(cost.load) - cost.per_step / cost.wall_per_step  # others' share
                print(f"{name}: {rtf:.2f}x real time ({'meets' if rtf >= TARGET else 'misses'} the {TARGET}x "
                      f"target; CPU time {cost.per_step * 1e3:.3f} ms/step = {cost.real_time_factor:.2f}x), "
                      f"start-up {cost.startup_wall:.1f} s, peak {cost.peak_rss / 2**20:.0f} MB, other load {load:.1f}",
                      flush=True)
                self.assertLess(cost.startup_wall, STARTUP_S)
                self.assertLess(cost.peak_rss, MEMORY_B)
                if load >= MAX_LOAD:
                    self.skipTest(f"{name}: other load {load:.1f} >= {MAX_LOAD}, {rtf:.2f}x not judged")
                self.assertGreaterEqual(rtf, FLOOR, f"{name}: {rtf:.2f}x real time at other load {load:.1f}")


if __name__ == "__main__":
    unittest.main()
