#!/usr/bin/env python3
"""The physical drivetrain (plugins/rover_drivetrain.cpp) against the
calibration targets of the realism design
(docs/superpowers/specs/2026-10-06-urc-realism-design.md, section 6.9).

The rover is built with DriveParams(mode="physical") into temporary
directories (simulate.physical) and mostly stands on synthetic ground maps
(simulate.ground_world) whose traction rows are the design's section 5.6
table. Each target's kind: physics (follows from the model's mechanics and
could fail), plumbing (restates its input parameters), (A) (an assumed
sub-model). Runs are headless TestFixture runs, a few seconds each."""
import dataclasses
import json
import math
import os
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from simulate import gen_model, ground_row, ground_world, physical, simulate, world_sdf
from worldfiles import temp_sdf

from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402  (after simulate set the environment)
from urc import meshes, terrain, terrains  # noqa: E402

P = gen_model.Params()
A, C = P.wheel_dx, P.pivot_y  # half wheelbase, half track [m]
T = terrains.Traction
# The ground types of design spec 5.6 the tests stand on, as ground.json rows: index, traction, dust. Test data:
# the same rows WS-T1 puts into terrains.TYPES and the worlds' ground.json.
GROUND = {
    "regolith": (3, T(mu_s=0.62, mu_k=0.52, crr=0.10, slip=0.3, sinkage_m=0.005), 0.4),
    "sand": (7, T(mu_s=0.52, mu_k=0.52, crr=0.20, bulldoze=0.06, slip=1.0, sinkage_m=0.02, dig_rate=0.01,
                  dig_max=1.25), 0.8),
    "wash_sand": (8, T(mu_s=0.52, mu_k=0.52, crr=0.25, bulldoze=0.10, slip=1.2, sinkage_m=0.03, dig_rate=0.012,
                       dig_max=1.15), 0.6),
    "rock": (12, T(mu_s=1.0, mu_k=0.85, crr=0.015, slip=0.05), 0.1),
    "manmade": (20, T(mu_s=0.80, mu_k=0.70, crr=0.015, slip=0.05), 0.0),
    "test_mu020": (30, T.coulomb(0.2), 0.0),
    "test_mu080": (31, T.coulomb(0.8), 0.0),
    "test_mu095": (32, T.coulomb(0.95), 0.0),
    "test_mu100": (33, T.coulomb(1.0), 0.0),
    "slab": (40, T(mu_s=1.0, mu_k=0.8, crr=0.015, slip=0.05), 0.1),  # the judder target's "mu 0.8 slab"
}
ROWS = [ground_row(index, key, traction, dust) for key, (index, traction, dust) in GROUND.items()]
OPTIONS = dict(terrain_default="rock", object_default="manmade")
FLAT = terrain.Heightfield(32.0, 65)  # 0.5 m samples
WEIGHT = 9.81 * (P.chassis_mass + 2 * P.rocker_mass + 4 * P.wheel_mass + P.camera_pan_mass + P.camera_tilt_mass)
LOAD = WEIGHT / 4  # [N] a wheel's share on flat ground, 113 N
WHEELS = ("fl", "rl", "fr", "rr")
SPIN = [(0.0, 0.0, 0.0), (0.5, 0.0, 1.0)]  # turn in place at 1 rad/s from 0.5 s


def kind(key):
    return GROUND[key][1]


def everywhere(key, n=FLAT.n):
    return np.full((n, n), GROUND[key][0], dtype=np.uint8)


def spin_ratio(t, dig=1.0):
    """The fresh spin-in-place yaw ratio on ground t (design spec 5.6): the
    root r of the quasi-static moment balance mu_k (c sx - a sy) / |s| =
    crr D c + bulldoze D^2 a, with sx = c (1 - r), sy = a r; 0 if it stalls."""
    resist = t.crr * dig * C + t.bulldoze * dig * dig * A

    def surplus(r):
        return t.mu_k * (C * C * (1 - r) - A * A * r) / math.hypot(C * (1 - r), A * r) - resist

    if surplus(0.0) <= 0:
        return 0.0
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if surplus(mid) > 0 else (lo, mid)
    return lo


@dataclass
class Run:
    """A run's base_link trace (simulate.State.trace) and drivetrain states (design spec 9.3)."""
    state: object
    states: list

    def window(self, t0, t1):
        tr = self.state.trace
        return tr[(tr[:, 0] >= t0) & (tr[:, 0] < t1)]

    def yaw_rate(self, t0, t1):
        return float(self.window(t0, t1)[:, 7].mean())

    def turned(self, t0, t1):
        yaw = np.unwrap(self.window(t0, t1)[:, 6])
        return float(yaw[-1] - yaw[0])

    def wheel(self, key, t0, t1, wheels=WHEELS):
        """(samples, wheels) of a drivetrain state value."""
        return np.array([[s["wheels"][w][key] for w in wheels] for s in self.states if t0 <= s["t"] < t1])


def drive(seconds, cmd, ground="regolith", params=None, solver=None, hf=FLAT, raster=None, rover=(0.0, 0.0, 0.0),
          lift=0.02, extra="", terrain_extra="", trace_every=10, subscribe=(), publish_until=None,
          gravity=(0.0, 0.0, -9.8)):
    """Run the physical rover on terrain hf with ground map `raster` (default: `ground` everywhere)."""
    raster = everywhere(ground, hf.n) if raster is None else raster
    with ground_world(hf, raster, ROWS, rover, lift=lift, ground_options=OPTIONS, terrain_extra=terrain_extra,
                      extra=extra, params=params or physical(dig="off"), solver=solver, gravity=gravity) as world:
        s = simulate(seconds, world=world, cmd=cmd, trace_every=trace_every, publish_until=publish_until,
                     subscribe=[(gen_model.DRIVETRAIN_TOPIC, StringMsg), *subscribe])
    return Run(s, [json.loads(m.data) for m in s.messages[gen_model.DRIVETRAIN_TOPIC]])


class SpinInPlace(unittest.TestCase):
    """Turning in place on flat ground of each kind, fresh (no dig-in): the
    yaw ratio is the closed-form root of design spec 5.6, +- 0.04 (physics)."""

    def test_ratio_per_ground(self):
        for key in ("rock", "regolith", "sand", "wash_sand", "test_mu095"):
            with self.subTest(ground=key):
                run = drive(6.0, SPIN, key)
                expected = spin_ratio(kind(key))
                self.assertAlmostEqual(run.yaw_rate(3.0, 6.0), expected, delta=0.04)
                self.assertEqual({s for row in run.states[-5:] for s in [row["wheels"]["fl"]["surface"]]}, {key})

    def test_wheel_torque_on_mu_lanes(self):
        """Mean wheel torque while spinning on Coulomb ground: mu N r x 0.748,
        12.7 mu N m +- 25 % (physics; the 0.748 is the slip direction's
        projection on the tread, the closed form's)."""
        for key, mu in (("test_mu020", 0.2), ("test_mu095", 0.95)):
            with self.subTest(ground=key):
                torque = np.abs(drive(6.0, SPIN, key).wheel("tau", 3.0, 6.0)).mean()
                self.assertAlmostEqual(torque, mu * LOAD * P.wheel_radius * 0.748, delta=0.25 * 12.7 * mu)


class SpinOnObjects(unittest.TestCase):
    """Every wheel contact gets the friction circle, not only the terrain's
    (today's DART rule cannot turn in place on box friction): the landing
    pad (an object without SDF friction: manmade), a step top and a mu 0.2
    friction tile (terrain shapes, by ground.json's collision map), an object
    whose SDF sets mu 0.5 (mu_s = mu_k = 0.5, crr 0.015, slip 0.05). Each turns
    at its surface's closed-form ratio +- 0.04 (physics, regression of D1)."""

    BOX = ('<collision name="{name}"><pose>0 0 0.05 0 0 0</pose><geometry><box><size>4 4 0.1</size></box>'
           '</geometry>{surface}</collision>')

    def spin_on(self, extra="", terrain_extra=""):
        return drive(6.0, SPIN, "regolith", lift=0.12, extra=extra, terrain_extra=terrain_extra).yaw_rate(3.0, 6.0)

    def model(self, surface=""):
        return (f'<model name="pad"><static>true</static><link name="link">'
                f'{self.BOX.format(name="pad_collision", surface=surface)}</link></model>')

    def test_landing_pad(self):
        self.assertAlmostEqual(self.spin_on(extra=self.model()), spin_ratio(kind("manmade")), delta=0.04)

    def test_object_with_sdf_friction(self):
        surface = "<surface><friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction></surface>"
        expected = spin_ratio(T(mu_s=0.5, mu_k=0.5, crr=0.015, slip=0.05))
        self.assertAlmostEqual(self.spin_on(extra=self.model(surface)), expected, delta=0.04)

    def test_terrain_shapes_by_collision_map(self):
        options = dict(OPTIONS, collisions={"zone_test_3": "test_mu020"})
        for name, key in (("step_top_collision", "rock"), ("zone_test_3", "test_mu020")):
            with self.subTest(shape=name):
                shape = self.BOX.format(name=name, surface="")
                with ground_world(FLAT, everywhere("regolith"), ROWS, lift=0.12, ground_options=options,
                                  terrain_extra=shape, params=physical(dig="off")) as world:
                    s = simulate(6.0, world=world, cmd=SPIN, trace_every=10)
                rate = Run(s, []).yaw_rate(3.0, 6.0)
                self.assertAlmostEqual(rate, spin_ratio(kind(key)), delta=0.04)


class DiagonalLoads(unittest.TestCase):
    """While spinning, the longitudinal friction moments about the rocker
    pivots unload one diagonal (design spec 6.8): dN/sum N = h mu 0.748 / a.
    PGS gives each wheel mu times its own load, which this needs (D4). The
    targets are analytic, so the runs leave out the spatial mu noise: averaged
    over the metre or two a wheel travels here, it moves single wheels'
    numbers by up to 10 % (measured: a light wheel at 3.9 A instead of 4.4)."""

    @classmethod
    def setUpClass(cls):
        cls.runs = {key: drive(6.0, SPIN, key, solver="pgs", params=physical(dig="off", mu_noise=0.0))
                    for key in ("test_mu100", "test_mu080")}

    def test_load_split(self):
        """Loads within +-15 % of 179/47 N at mu 1 and 166/60 N at mu 0.8; the
        loaded wheels' torque at least twice the light ones' (physics)."""
        for key, (heavy, light) in (("test_mu100", (179.0, 47.0)), ("test_mu080", (166.0, 60.0))):
            with self.subTest(ground=key):
                run = self.runs[key]
                loads = run.wheel("load", 3.0, 6.0).mean(axis=0)  # fl, rl, fr, rr
                torques = np.abs(run.wheel("tau", 3.0, 6.0)).mean(axis=0)
                # Turning left, the front-left and rear-right wheels carry the load.
                for got, want in zip(loads, (heavy, light, light, heavy)):
                    self.assertAlmostEqual(got, want, delta=0.15 * want)
                self.assertGreaterEqual(min(torques[[0, 3]]) / max(torques[[1, 2]]), 2.0)

    def test_currents_on_a_mu_08_slab(self):
        """Loaded wheels 9.7 +- 1.5 A: (14.9 N m friction + 2.3 N m motor
        friction) / 1.78 N m/A; light wheels 4-6 A (physics, with the (A) free
        current)."""
        currents = np.abs(self.runs["test_mu080"].wheel("i", 3.0, 6.0)).mean(axis=0)
        for got in currents[[0, 3]]:
            self.assertAlmostEqual(got, 9.7, delta=1.5)
        for got in currents[[1, 2]]:
            self.assertTrue(4.0 <= got <= 6.0, currents)


class Stall(unittest.TestCase):
    """A current limit below what the loaded diagonal needs (9.7 A on mu 0.8)
    stalls the turn (physics, PGS)."""

    @classmethod
    def setUpClass(cls):
        def turned(limit):
            run = drive(6.0, [(0.0, 0.0, 0.0), (1.0, 0.0, 1.0)], "test_mu080", solver="pgs",
                        params=physical(dig="off", current_limit=limit))
            return math.degrees(run.turned(1.0, 6.0))
        cls.yaw = {limit: turned(limit) for limit in (6.0, 6.8, 8.0, 10.0, 12.0, 12.6, 14.0, 20.0)}

    def test_stall_and_no_stall(self):
        """0.7 x 9.7 = 6.8 A: under 15 % of the unlimited yaw in 5 s (20 A is
        unlimited here); 1.3 x 9.7 = 12.6 A: at least 85 %."""
        self.assertLess(self.yaw[6.8], 0.15 * self.yaw[20.0], self.yaw)
        self.assertGreaterEqual(self.yaw[12.6], 0.85 * self.yaw[20.0], self.yaw)

    def test_monotonic_in_the_limit(self):
        """More current never turns less (+-2 deg): the sticking box is aligned
        with the load, so breakaway follows the friction circle (design spec 6.4)."""
        yaws = [self.yaw[limit] for limit in (6.0, 8.0, 10.0, 12.0, 14.0)]
        for low, high in zip(yaws, yaws[1:]):
            self.assertGreaterEqual(high, low - 2.0, self.yaw)


class Response(unittest.TestCase):
    """Rise to 90 % of the steady yaw rate after a step to wz = 1 rad/s on
    rock, and the stop after a step back to 0."""

    @staticmethod
    def times(params):
        run = drive(4.0, [(0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (3.0, 0.0, 0.0)], "rock", params=params, trace_every=1)
        tr = run.window(0.0, 4.0)
        t, rate = tr[:, 0], tr[:, 7]
        steady = rate[(t > 2.5) & (t < 3.0)].mean()
        rise = t[(t > 1.0) & (rate >= 0.9 * steady)][0] - 1.0
        stop = t[(t > 3.0) & (np.abs(rate) < 0.01)][0] - 3.0
        return rise, stop

    def test_with_the_ramp(self):
        """rise = setpoint / accel +- 20 %, 0.33 s at wz 1 (plumbing: the ramp
        is the controller's, an assumption, design spec 6.3)."""
        rise, _ = self.times(physical(dig="off"))
        setpoint = 1.0 * C / P.wheel_radius  # [rad/s] each wheel
        self.assertAlmostEqual(rise, setpoint / P.drive.accel, delta=0.2 * setpoint / P.drive.accel)

    def test_stop_without_the_ramp(self):
        """The motor, contact and tyre model alone stop the turn in < 40 ms (physics)."""
        self.assertLess(self.times(physical(dig="off", accel=0.0))[1], 0.040)

    @unittest.expectedFailure
    def test_rise_without_the_ramp(self):
        """Design spec 6.9: rise < 60 ms without the ramp (prototype 24 ms). The
        prototype had tyre compliance, which overshoots to 0.95-1.0 of steady
        at 30 ms (measured here 26-28 ms with Params.tire_compliance). Without
        it the yaw rate reaches 0.77 of steady in 40 ms and 90 % only after
        113 ms: the PI's integral (time constant kp / ki = 0.1 s) has to supply
        the motors' IR drop under load (measured 2026-10-07)."""
        self.assertLess(self.times(physical(dig="off", accel=0.0))[0], 0.060)


class SlowTurn(unittest.TestCase):
    """A slow turn in place (0.15 rad/s) on a firm slab should stick and slip;
    in loose sand (mu_s = mu_k) it must not (physics with the (A) Stribeck curve)."""

    @staticmethod
    def judder(ground, tire_compliance=False):
        params = dataclasses.replace(physical(dig="off", accel=4.0), tire_compliance=tire_compliance)
        run = drive(9.0, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.15)], ground, trace_every=1, params=params)
        rate = run.window(3.0, 9.0)[:, 7]
        spectrum = np.abs(np.fft.rfft((rate - rate.mean()) * np.hanning(len(rate)))) ** 2
        f = np.fft.rfftfreq(len(rate), 0.001)
        spectrum[f < 0.3] = 0
        return np.ptp(rate) / abs(rate.mean()), f[np.argmax(spectrum)]

    @unittest.expectedFailure
    def test_stick_slip_on_a_slab(self):
        """Design spec 6.9: yaw-rate peak-to-peak / mean >= 0.8, dominant 0.5-5 Hz,
        without tyre compliance (spec: prototype 1.26 at 0.8 Hz; the
        prototype's own results_summary.json has 0.001 for its no-tyre slow
        turn). A rigid rover turning in place slips at the yaw ratio where the
        slip's magnitude is least, r = c^2 / (a^2 + c^2), so the Stribeck
        curve's negative slope has no first-order effect and the turn stays
        smooth. Measured 2026-10-07: 0.011 (Dantzig); 0.60 on PGS, all solver
        noise at 333 Hz. With Params.tire_compliance: 0.41 (Dantzig) and 0.78
        (PGS), at the 22-24 Hz wheel hop, not 0.5-5 Hz. (Before the contact
        rule read the wheel spin through the speed filter, tyres and PGS gave
        1.10 at 2.7 Hz, but parked rovers crept; see rover_drivetrain.cpp.)"""
        ratio, peak = self.judder("slab")
        self.assertGreaterEqual(ratio, 0.8)
        self.assertTrue(0.5 <= peak <= 5.0, peak)

    def test_smooth_in_sand(self):
        self.assertLess(self.judder("sand")[0], 0.5)
        self.assertLess(self.judder("sand", tire_compliance=True)[0], 0.5)


class LooseSand(unittest.TestCase):
    """Driving and digging in loose sand (design spec 6.5)."""

    def test_straight_slip_and_torque(self):
        """Straight at 0.5 m/s: slip = slip x crr = 20 % +- 3 %, wheel torque =
        crr N r = 3.4 N m +- 15 % (plumbing: restates the parameters)."""
        run = drive(8.0, [(0.0, 0.0, 0.0), (0.5, 0.5, 0.0)], "sand", rover=(-6.0, 0.0, 0.0))
        tr = run.window(3.0, 8.0)
        speed = (tr[-1, 1] - tr[0, 1]) / (tr[-1, 0] - tr[0, 0])
        surface = np.abs(run.wheel("w", 3.0, 8.0)).mean() * P.wheel_radius
        sand = kind("sand")
        self.assertAlmostEqual(1 - speed / surface, sand.slip * sand.crr, delta=0.03)
        torque = np.abs(run.wheel("tau", 3.0, 8.0)).mean()
        self.assertAlmostEqual(torque, sand.crr * LOAD * P.wheel_radius, delta=0.15 * sand.crr * LOAD * P.wheel_radius)

    @staticmethod
    def sandpit(seconds, cmd, dig):
        """Sand within 3 m of the origin, rock around it; the rover at the origin."""
        X, Y = FLAT.grid()
        raster = np.where(np.hypot(X, Y) < 3.0, GROUND["sand"][0], GROUND["rock"][0]).astype(np.uint8)
        return drive(seconds, cmd, raster=raster, params=physical(dig=dig))

    @staticmethod
    def spin_ratios(run):
        """Yaw ratio in each second of the spin (it starts at 0.5 s)."""
        return [run.yaw_rate(t, t + 1.0) for t in np.arange(1.0, 10.5, 1.0)]

    def test_strong_dig_in(self):
        """The default (strong preset, the user's choice): a sustained spin in
        loose sand digs in until the rover cannot turn; it then drives out
        straight at 0.3 m/s, and a metre of rolling heals the dig to D < 1.05.
        Rock never digs. (A)"""
        run = self.sandpit(32.0, [(0.0, 0.0, 0.0), (0.5, 0.0, 1.0), (10.5, 0.0, 0.0), (11.0, 0.3, 0.0)], "strong")
        ratios = self.spin_ratios(run)
        self.assertGreater(run.window(0.5, 1.5)[:, 7].max(), 0.8 * spin_ratio(kind("sand")))  # it did start turning
        self.assertLess(max(ratios[-3:]), 0.1 * spin_ratio(kind("sand")), ratios)
        self.assertGreater(run.wheel("dig", 9.0, 10.5).min(), 1.95)  # strong: sand's D_max 2.0
        tr = run.window(11.0, 32.0)
        x, y = tr[:, 1], tr[:, 2]
        out = np.flatnonzero(np.hypot(x, y) > 3.0 + math.hypot(A, C) + P.wheel_radius)  # every wheel on the rock
        self.assertTrue(len(out), "the rover did not drive out of the sand")
        later = out[0] + np.flatnonzero(np.hypot(x[out[0]:] - x[out[0]], y[out[0]:] - y[out[0]]) > 1.0)
        self.assertTrue(len(later), "the rover did not roll 1 m on the rock")
        self.assertLess(run.wheel("dig", tr[later[0], 0], tr[later[0], 0] + 0.1).max(), 1.05)
        on_rock = np.array([[s["wheels"][w]["dig"] for w in WHEELS] for s in run.states if s["t"] >= tr[out[0], 0]])
        self.assertTrue(np.all(np.diff(on_rock, axis=0) <= 1e-12), "rock dug a wheel in")

    def test_mild_dig_in(self):
        """The mild preset: a 10 s spin slows monotonically to 0.6-0.8 x of the
        fresh ratio (0.72 x computed at D_max 1.25) and never stops. (A)"""
        ratios = self.spin_ratios(self.sandpit(10.5, [(0.0, 0.0, 0.0), (0.5, 0.0, 1.0)], "mild"))
        fresh = spin_ratio(kind("sand"))
        self.assertAlmostEqual(ratios[0], fresh, delta=0.04)
        for earlier, later in zip(ratios, ratios[1:]):
            self.assertLessEqual(later, earlier + 0.01, ratios)
        self.assertTrue(0.6 * fresh <= ratios[-1] <= 0.8 * fresh, ratios)

    def test_no_dig_on_firm_ground(self):
        run = drive(5.0, SPIN, "regolith", params=physical())  # the strong preset
        self.assertEqual(run.wheel("dig", 0.0, 5.0).max(), 1.0)


class Washboard(unittest.TestCase):
    """Over corrugations every 0.8 m, +-4 cm, at 0.5 m/s (DiffDrive's velocity
    servo: torque std 12.8 N m, spikes to its +-30 N m limit).

    The corrugations are a mesh in the terrain link (regolith by the collision
    map): on a corrugated heightmap DART's wheels sink 5-15 cm and stall,
    DiffDrive's too (measured here and on the proving ground's washboard)."""

    @classmethod
    def setUpClass(cls):
        with tempfile.TemporaryDirectory() as directory:
            x = np.arange(-4.0, 4.0001, 0.05)
            vertices = [(u, v, 0.04 * (1 - math.cos(2 * math.pi * (u + 4.0) / 0.8))) for u in x for v in (-2.0, 2.0)]
            faces = [face for i in range(0, 2 * len(x) - 2, 2) for face in ((i, i + 2, i + 1), (i + 1, i + 2, i + 3))]
            mesh = Path(directory) / "washboard.obj"
            meshes.write_obj(mesh, vertices, faces)
            shape = f'<collision name="washboard"><geometry><mesh><uri>{mesh.as_uri()}</uri></mesh></geometry></collision>'
            with ground_world(FLAT, everywhere("regolith"), ROWS, (-7.0, 0.0, 0.0), terrain_extra=shape,
                              ground_options=dict(OPTIONS, collisions={"washboard": "regolith"}),
                              params=physical(dig="off", state_rate=1000.0)) as world:
                s = simulate(20.0, world=world, cmd=[(0.0, 0.0, 0.0), (0.5, 0.5, 0.0)], trace_every=10,
                             subscribe=[(gen_model.DRIVETRAIN_TOPIC, StringMsg)])
        cls.trip = Run(s, [json.loads(m.data) for m in s.messages[gen_model.DRIVETRAIN_TOPIC]])
        tr = cls.trip.window(0.0, 20.0)
        over = tr[(tr[:, 1] > -3.0) & (tr[:, 1] < 3.0), 0]  # every wheel on the corrugations
        cls.span = over[0], over[-1]
        cls.torque = cls.trip.wheel("tau", *cls.span)

    def test_crosses_on_its_ground(self):
        self.assertGreater(self.span[1] - self.span[0], 10.0)
        touched = {w["surface"] for st in self.trip.states if self.span[0] <= st["t"] < self.span[1]
                   for w in st["wheels"].values()}
        self.assertEqual(touched - {""}, {"regolith"})  # "": a wheel off the ground for a step

    def test_no_torque_ripple(self):
        """Above 5 Hz the wheel torque's std stays under 1.5 N m: no chatter or
        spikes (physics)."""
        torque = self.torque - self.torque.mean(axis=0)
        spectrum = np.fft.rfft(torque, axis=0)
        spectrum[np.fft.rfftfreq(len(torque), 0.001) < 5.0] = 0
        self.assertLess(np.fft.irfft(spectrum, len(torque), axis=0).std(axis=0).max(), 1.5)

    @unittest.expectedFailure
    def test_wheel_torque_std(self):
        """Design spec 6.9: wheel-torque std < 1.5 N m (prototype 0.2-0.8). A
        speed-controlled wheel has to lift the rover over every crest: the body
        heaves +-4 cm at 0.63 Hz, which takes up to 71 W, +-5.3 N m per wheel
        (computed), std ~3.7 N m. Measured 2026-10-07: 3.9 N m, nearly all of it
        at the crest frequency."""
        self.assertLess(self.torque.std(axis=0).max(), 1.5)


def slope(degrees, size=32.0, n=65):
    """Ground rising to the north at `degrees`."""
    hf = terrain.Heightfield(size, n)
    _, Y = hf.grid()
    hf.z = math.tan(math.radians(degrees)) * (Y + size / 2)
    return hf


class Slopes(unittest.TestCase):
    """Parked, turning and traversing on slopes rising to the north (+y)."""

    def test_spin_on_a_side_slope(self):
        """2.5 s of turning in place on 20 deg regolith slides the rover 0.4-1.0 m
        downhill (physics, re-baselined for the slip semantics, design spec 6.9)."""
        run = drive(3.5, [(0.0, 0.0, 0.0), (1.0, 0.0, 1.0)], "regolith", hf=slope(20.0))
        tr = run.window(0.9, 3.5)
        self.assertTrue(0.4 <= tr[0, 2] - tr[-1, 2] <= 1.0, tr[0, 2] - tr[-1, 2])

    def test_drift_across_a_side_slope(self):
        """3.5 m across 20 deg at 0.5 m/s: downhill drift = slip tan 20 deg 3.5 m
        +- 30 % (rock 0.06 m, regolith 0.38 m, sand 1.27 m; the bulldozing in sand
        keeps it lower) and under 0.1 m on rock (plumbing and direction)."""
        for key in ("rock", "regolith", "sand"):
            with self.subTest(ground=key):
                run = drive(11.0, [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0)], key, hf=slope(20.0), rover=(-3.0, 0.0, 0.0))
                tr = run.window(0.9, 11.0)
                travelled = np.flatnonzero(tr[:, 1] - tr[0, 1] >= 3.5)
                self.assertTrue(len(travelled), f"{key}: did not get 3.5 m across")
                drift = tr[0, 2] - tr[travelled[0], 2]
                expected = kind(key).slip * math.tan(math.radians(20.0)) * 3.5
                self.assertAlmostEqual(drift, expected, delta=0.3 * expected)
                if key == "rock":
                    self.assertLess(drift, 0.1)

    def test_parked_rover_does_not_creep(self):
        """cmd 0 for 60 s on 15 deg regolith and 20 deg sand: moves < 1 cm. A
        sticking contact has no slip compliance (D23)."""
        for key, degrees in (("regolith", 15.0), ("sand", 20.0)):
            with self.subTest(ground=key):
                run = drive(61.0, (0.0, 0.0), key, hf=slope(degrees), trace_every=100)
                start, end = run.window(1.0, 1.1)[0], run.window(60.9, 61.0)[-1]
                self.assertLess(math.dist(start[1:4], end[1:4]), 0.01)

    def test_hold_angle_does_not_depend_on_heading(self):
        """Regolith (mu_s 0.62) holds a parked rover at atan(mu_s) - 2 deg and
        lets it slide at + 2 deg, at headings 0, 30, 45 and 90 deg: the
        sticking box is aligned with the load (design spec 6.4).

        Between atan(mu_k) and atan(mu_s) only a contact that never slid
        holds, so the rover must start without a jolt: flat ground under
        tilted gravity (a spawn on a sloped heightmap lands it on its uphill
        wheels first), no spatial mu noise (it moves the local limit by up to
        20 %), and no driveline backlash. With the 1.5 deg backlash a rover
        released at 30 deg to the fall line rolls through the dead band and
        the catch starts the slide (measured at 29.8 deg); a rover that drove
        there and stopped has its backlash taken up already."""
        limit = math.degrees(math.atan(kind("regolith").mu_s))
        for degrees, holds in ((limit - 2.0, True), (limit + 2.0, False)):
            tilt = math.radians(degrees)
            gravity = (0.0, -9.8 * math.sin(tilt), -9.8 * math.cos(tilt))  # downhill: south
            for heading in (0.0, 30.0, 45.0, 90.0):
                with self.subTest(slope=round(degrees, 1), heading=heading):
                    run = drive(4.0, (0.0, 0.0), "regolith", rover=(0.0, 0.0, math.radians(heading)), lift=0.0,
                                gravity=gravity, params=physical(dig="off", mu_noise=0.0, backlash=0.0))
                    moved = math.dist(run.window(1.0, 1.1)[0][1:4], run.window(3.9, 4.0)[-1][1:4])
                    if holds:
                        self.assertLess(moved, 0.02)
                    else:
                        self.assertGreater(moved, 0.2)

    def test_calibration_lanes(self):
        """Parked on 20 deg: the mu 0.2 lane slides, the mu 0.95 lane holds (physics)."""
        moved = {}
        for key in ("test_mu020", "test_mu095"):
            run = drive(4.0, (0.0, 0.0), key, hf=slope(20.0))
            moved[key] = math.dist(run.window(1.0, 1.1)[0][1:4], run.window(3.9, 4.0)[-1][1:4])
        self.assertGreater(moved["test_mu020"], 1.0, moved)
        self.assertLess(moved["test_mu095"], 0.02, moved)


if __name__ == "__main__":
    unittest.main()
