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
import re
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from simulate import (cpu_time_per_step, diffdrive, gen_model, ground_row, ground_world, physical, simulate,
                      spin_ratio, world_file)
from worldfiles import WORLDS, temp_sdf, world_copy

from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402  (after simulate set the environment)
from gz.msgs10.odometry_pb2 import Odometry  # noqa: E402
from gz.msgs10.particle_emitter_pb2 import ParticleEmitter  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
from gz.msgs10.world_control_pb2 import WorldControl  # noqa: E402
from gz.sim8 import TestFixture  # noqa: E402
from gz.transport13 import Node  # noqa: E402
from urc import meshes, terrain, terrains  # noqa: E402

P = gen_model.Params()
A, C = P.wheel_dx, P.pivot_y  # half wheelbase, half track [m]
T = terrains.Traction
# The ground types the tests stand on, as ground.json rows: index, traction, dust. The catalogue's types are
# the worlds' ground.json rows (terrains.TYPES, design spec 5.6, the strong dig-in preset).
GROUND = {
    "regolith": (3, terrains.REGOLITH.traction, 0.4),
    "sand": (7, terrains.SAND.traction, 0.8),
    "wash_sand": (8, terrains.WASH_SAND.traction, 0.6),
    "rock": (12, terrains.ROCK.traction, 0.1),
    "manmade": (20, terrains.MANMADE.traction, 0.0),
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
          gravity=(0.0, 0.0, -9.8), rows=ROWS):
    """Run the physical rover on terrain hf with ground map `raster` (default: `ground` everywhere)."""
    raster = everywhere(ground, hf.n) if raster is None else raster
    with ground_world(hf, raster, rows, rover, lift=lift, ground_options=OPTIONS, terrain_extra=terrain_extra,
                      extra=extra, params=params or physical(dig=False), solver=solver, gravity=gravity) as world:
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
    (DART's own rule cannot turn in place on box friction): the landing
    pad (an object without SDF friction: manmade), a step top and a mu 0.2
    box (terrain shapes, by ground.json's collision map), an object
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
                                  terrain_extra=shape, params=physical(dig=False)) as world:
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
        cls.runs = {key: drive(6.0, SPIN, key, solver="pgs", params=physical(dig=False, mu_noise=0.0))
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
                        params=physical(dig=False, current_limit=limit))
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
        rise, _ = self.times(physical(dig=False))
        setpoint = 1.0 * C / P.wheel_radius  # [rad/s] each wheel
        self.assertAlmostEqual(rise, setpoint / P.drive.accel, delta=0.2 * setpoint / P.drive.accel)

    def test_stop_without_the_ramp(self):
        """The motor, contact and tyre model alone stop the turn in < 40 ms (physics)."""
        self.assertLess(self.times(physical(dig=False, accel=0.0))[1], 0.040)

    def test_rise_without_the_ramp(self):
        """Without the ramp the turn rises to 90 % in < 60 ms (physics; design
        spec 6.9). The PI's integral supplies the motors' IR drop under load:
        with the prototype's ki 40 (integral time kp / ki = 0.1 s) it took
        118 ms, with ki 160 (25 ms) 53 ms (measured 2026-10-07)."""
        self.assertLess(self.times(physical(dig=False, accel=0.0))[0], 0.060)


class SlowTurn(unittest.TestCase):
    """A slow turn in place (0.15 rad/s) on a firm slab should stick and slip;
    in loose sand (mu_s = mu_k) it must not (physics with the (A) Stribeck curve)."""

    @staticmethod
    def judder(ground, tire_compliance=False):
        params = dataclasses.replace(physical(dig=False, accel=4.0), tire_compliance=tire_compliance)
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
        """Sand within 3 m of the origin, rock around it; the rover at the origin. dig: the catalogue's dig-in
        preset the sand's row is written with (terrains.traction)."""
        X, Y = FLAT.grid()
        raster = np.where(np.hypot(X, Y) < 3.0, GROUND["sand"][0], GROUND["rock"][0]).astype(np.uint8)
        index, _, dust = GROUND["sand"]
        sand = ground_row(index, "sand", terrains.traction(terrains.SAND, dig), dust)
        return drive(seconds, cmd, raster=raster, params=physical(), rows=[sand if r["key"] == "sand" else r
                                                                             for r in ROWS])

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
        run = drive(5.0, SPIN, "regolith", params=physical())  # dig-in on
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
            shape = (f'<collision name="washboard"><geometry><mesh><uri>{mesh.as_uri()}</uri></mesh></geometry>'
                     '</collision>')
            with ground_world(FLAT, everywhere("regolith"), ROWS, (-7.0, 0.0, 0.0), terrain_extra=shape,
                              ground_options=dict(OPTIONS, collisions={"washboard": "regolith"}),
                              params=physical(dig=False, state_rate=1000.0)) as world:
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

    def test_wheel_torque_is_the_climb(self):
        """Each wheel's torque std is what rolling its load over the crests
        takes, N r x the corrugation's steepest slope / sqrt 2 (3.8 N m),
        +-30 % (physics). Re-baselined from design spec 6.9's < 1.5 N m: the
        prototype's 0.2-0.8 N m came from the proving ground's corrugated
        heightmap, into which wheels sink 5-15 cm and so barely climb
        (measured 2026-10-07); on this mesh they ride the crests, and the body
        heaves +-4 cm at 0.63 Hz. The chatter the target was after is the
        ripple above 5 Hz (test_no_torque_ripple). DiffDrive: 12.8 N m."""
        steepest = 0.04 * 2 * math.pi / 0.8  # d/dx of 0.04 (1 - cos(2 pi x / 0.8))
        expected = LOAD * P.wheel_radius * steepest / math.sqrt(2)
        for std in self.torque.std(axis=0):
            self.assertAlmostEqual(std, expected, delta=0.3 * expected)


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
        """3.5 m across 20 deg at 0.5 m/s: downhill drift = slip tan 20 deg x the
        wheels' rolled distance, 3.5 m / (1 - forward slip), +- 30 % (rock
        0.06 m, regolith 0.39 m, sand 1.59 m), and under 0.1 m on rock
        (plumbing and direction). Design spec 6.9 multiplies by 3.5 m, but the
        slip law scales with the wheel's speed, which is 25 % above the hub's
        in sand. Sand's bulldozing takes 16 % of the side load; the rover also
        yaws a little downhill (measured 0.16 rad), which adds about as much."""
        for key in ("rock", "regolith", "sand"):
            with self.subTest(ground=key):
                run = drive(11.0, [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0)], key, hf=slope(20.0), rover=(-3.0, 0.0, 0.0))
                tr = run.window(0.9, 11.0)
                travelled = np.flatnonzero(tr[:, 1] - tr[0, 1] >= 3.5)
                self.assertTrue(len(travelled), f"{key}: did not get 3.5 m across")
                drift = tr[0, 2] - tr[travelled[0], 2]
                ground = kind(key)
                expected = ground.slip * math.tan(math.radians(20.0)) * 3.5 / (1 - ground.slip * ground.crr)
                self.assertAlmostEqual(drift, expected, delta=0.3 * expected)
                if key == "rock":
                    self.assertLess(drift, 0.1)

    def test_parked_rover_does_not_creep(self):
        """cmd 0 for 60 s on 15 deg regolith and 20 deg sand: moves < 1 cm. A
        stopped wheel has no slip compliance (D23)."""
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
                                gravity=gravity, params=physical(dig=False, mu_noise=0.0, backlash=0.0))
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


def frame_ids(header):
    return {d.key: d.value[0] for d in header.data}


class Interfaces(unittest.TestCase):
    """Topics, commands, the ground lookup, reset and repeatability."""

    def test_odometry_and_tf(self):
        """DiffDrive's topics, types and frames: no-slip odometry from the wheel
        angles, odom -> base_link (design spec 6.8)."""
        topics = [(gen_model.ODOM_TOPIC, Odometry), (gen_model.TF_TOPIC, Pose_V)]
        run = drive(4.0, [(0.0, 0.0, 0.0), (0.5, 0.5, 0.0)], "rock", subscribe=topics, rover=(-3.0, 0.0, 0.0))
        odom, tf = run.state.messages[gen_model.ODOM_TOPIC][-1], run.state.messages[gen_model.TF_TOPIC][-1]
        self.assertEqual(frame_ids(odom.header), {"frame_id": "odom", "child_frame_id": "base_link"})
        self.assertEqual(frame_ids(tf.pose[0].header), {"frame_id": "odom", "child_frame_id": "base_link"})
        self.assertAlmostEqual(tf.pose[0].position.x, odom.pose.position.x, delta=0.05)
        travelled = run.state.poses["base_link"][0] + 3.0
        self.assertAlmostEqual(odom.pose.position.x, travelled, delta=0.03 * travelled)  # rock: 0.1 % slip
        self.assertAlmostEqual(odom.twist.linear.x, 0.5, delta=0.02)
        self.assertGreater(len(run.state.messages[gen_model.ODOM_TOPIC]), 0.8 * 50 * 4.0)  # 50 Hz

    def test_odometry_overreports_a_spin(self):
        """Odometry yaw / true yaw while spinning = 1 / ratio of the ground +- 15 %
        (plumbing; with track_multiplier 1, as a real rover's odometry, design spec 6.8)."""
        run = drive(6.0, SPIN, "regolith", subscribe=[(gen_model.ODOM_TOPIC, Odometry)])
        odom = [m.twist.angular.z for m in run.state.messages[gen_model.ODOM_TOPIC]
                if 3.0 <= m.header.stamp.sec + m.header.stamp.nsec * 1e-9 < 6.0]
        expected = 1 / spin_ratio(kind("regolith"))
        self.assertAlmostEqual(np.mean(odom) / run.yaw_rate(3.0, 6.0), expected, delta=0.15 * expected)

    def test_state_message(self):
        """/model/rover/drivetrain (design spec 9.3): every wheel's setpoint, speed,
        current, torque, voltage, saturation, slip, load, surface and dig factor, 50 Hz."""
        run = drive(2.0, [(0.0, 0.0, 0.0), (0.5, 0.3, 0.2)], "sand")
        last = run.states[-1]
        self.assertEqual(set(last), {"t", "cmd", "wheels"})
        self.assertEqual(last["cmd"], [0.3, 0.2])
        self.assertEqual(set(last["wheels"]), set(WHEELS))
        for wheel in last["wheels"].values():
            self.assertEqual(set(wheel), {"sp", "w", "i", "tau", "u", "sat", "slip", "load", "surface", "dig"})
            self.assertEqual(wheel["surface"], "sand")
            self.assertIsInstance(wheel["sat"], bool)
        self.assertAlmostEqual(last["wheels"]["fl"]["sp"], (0.3 - 0.2 * C) / P.wheel_radius, delta=1e-4)
        self.assertTrue(90 <= len(run.states) <= 101, len(run.states))

    def test_command_timeout(self):
        """A command older than cmd_timeout counts as zero (0.5 s, the user's
        decision); cmd_timeout 0 holds the last command, as Gazebo's GUI Teleop
        needs. Sim clock here; the default clock is wall time."""
        cmd = [(0.0, 0.0, 0.0), (0.2, 0.5, 0.0)]
        stops = drive(4.0, cmd, "rock", publish_until=1.0)
        holds = drive(4.0, cmd, "rock", publish_until=1.0, params=physical(dig=False, cmd_timeout=0.0))
        self.assertEqual(stops.states[-1]["cmd"], [0.0, 0.0])
        self.assertEqual(holds.states[-1]["cmd"], [0.5, 0.0])
        self.assertLess(stops.state.poses["base_link"][0], 0.75)  # 1.0 s + 0.5 s of the last command, the ramps
        self.assertGreater(holds.state.poses["base_link"][0], 1.6)

    def test_ground_map_lookup(self):
        """The drivetrain reads ground.png at each contact, nearest sample (design
        spec 9.1): driving north across a boundary (sand north of y = 0, rock
        south), every wheel's reported ground is the one under its hub."""
        X, Y = FLAT.grid()
        raster = np.where(Y > 0.0, GROUND["sand"][0], GROUND["rock"][0]).astype(np.uint8)
        run = drive(14.0, [(0.0, 0.0, 0.0), (0.5, 0.4, 0.0)], raster=raster, rover=(0.0, -3.0, math.pi / 2))
        tr = run.window(0.0, 14.0)
        res = FLAT.size / (FLAT.n - 1)
        checked = 0
        for state in run.states:
            row = tr[np.argmin(np.abs(tr[:, 0] - state["t"]))]
            x, y, yaw = row[1], row[2], row[6]
            for name, (dx, dy) in zip(WHEELS, ((A, C), (-A, C), (A, -C), (-A, -C))):
                wx, wy = x + dx * math.cos(yaw) - dy * math.sin(yaw), y + dx * math.sin(yaw) + dy * math.cos(yaw)
                if abs(wy) < 0.4 or not state["wheels"][name]["surface"]:
                    continue  # within the contact's reach of the boundary
                i = int(round((FLAT.size / 2 - wy) / res))
                j = int(round((wx + FLAT.size / 2) / res))
                expected = next(k for k, (index, _, _) in GROUND.items() if index == raster[i, j])
                self.assertEqual(state["wheels"][name]["surface"], expected, (state["t"], name, wx, wy))
                checked += 1
        self.assertGreater(checked, 1500)

    def test_plane_is_the_default_surface(self):
        """Without a ground map a plane is ground of the drivetrain's default
        surface, whose traction comes from the catalogue (terrains.TYPES)."""
        for key in ("regolith", "sand"):
            with self.subTest(surface=key):
                s = simulate(6.0, cmd=SPIN, params=physical(dig=False), default_surface=key, trace_every=10)
                self.assertAlmostEqual(Run(s, []).yaw_rate(3.0, 6.0), spin_ratio(terrains.TYPES[key].traction),
                                       delta=0.04)

    def test_dust(self):
        """The rear emitters get a rate from speed, slip and the ground's dust
        factor (design spec 6.5): nothing while parked, emitting while driving,
        more on sand than on rock, and one "off" when the rover stops."""
        topic = gen_model.DUST_TOPIC.format(link="rocker_left", emitter="dust_rl")
        rates = {}
        for key in ("sand", "rock"):
            run = drive(6.0, [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0), (4.0, 0.0, 0.0)], key, rover=(-4.0, 0.0, 0.0),
                        subscribe=[(topic, ParticleEmitter)])
            messages = run.state.messages[topic]
            self.assertTrue(messages and messages[0].emitting.data)
            self.assertFalse(messages[-1].emitting.data)  # stopped
            self.assertEqual(sum(not m.emitting.data for m in messages), 1)
            rates[key] = max(m.rate.data for m in messages)
            self.assertLessEqual(rates[key], P.drive.dust_max)
        expected = GROUND["sand"][2] * (P.drive.dust_speed_gain * 0.4 + P.drive.dust_slip_gain * 0.1)
        self.assertAlmostEqual(rates["sand"], expected, delta=0.25 * expected)  # 0.5 m/s at 20 % slip
        self.assertGreater(rates["sand"], 3 * rates["rock"])

    def test_reset(self):
        """A world reset (ISystemReset) clears the drivetrain: the last command
        (held here: cmd_timeout 0), its odometry, motors and ramps. The rover
        stands still afterwards and its odometry starts again at 0."""
        with ground_world(FLAT, everywhere("rock"), ROWS, ground_options=OPTIONS,
                          params=physical(dig=False, cmd_timeout=0.0)) as world:
            fixture = TestFixture(world)
            node = Node()
            publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
            twist = Twist()
            twist.linear.x = 0.5
            commanding = [True]
            fixture.on_pre_update(lambda info, ecm: commanding[0] and info.iterations % 20 == 0
                                  and publisher.publish(twist))
            fixture.finalize()
            before, after = [], []
            node.subscribe(Odometry, gen_model.ODOM_TOPIC, before.append)
            fixture.server().run(True, 3000, False)
            commanding[0] = False
            node.unsubscribe(gen_model.ODOM_TOPIC)  # a request with a subscription active can lose its reply
            request = WorldControl()
            request.reset.all = True
            ok, _ = node.request("/world/ground_test/control", request, WorldControl, Boolean, 3000)
            self.assertTrue(ok)
            node.subscribe(Odometry, gen_model.ODOM_TOPIC, after.append)
            fixture.server().run(True, 1000, False)
        self.assertGreater(before[-1].pose.position.x, 1.0)  # it drove before the reset
        self.assertTrue(after)
        self.assertLess(max(abs(m.pose.position.x) for m in after), 0.01, [m.pose.position.x for m in after])
        self.assertLess(max(abs(m.twist.linear.x) for m in after), 0.01)

    def test_repeatable(self):
        """The same run twice gives the same poses."""
        first, second = (drive(3.0, [(0.0, 0.0, 0.0), (0.5, 0.4, 0.6)], "sand", params=physical()) for _ in range(2))
        self.assertEqual(first.state.poses, second.state.poses)


@unittest.skipUnless(os.environ.get("ROVER_PERF"), "slow (minutes): set ROVER_PERF=1")
class Cost(unittest.TestCase):
    def test_at_most_a_quarter_dearer_than_diffdrive(self):
        """CPU time per step with the physical drivetrain against DiffDrive, the
        same world, the rover driving simulate.DRIVE_SCHEDULE from outside the
        server: at most +25 % (design spec 10.3; interleaved runs, so the ratio
        holds on a loaded machine), the worlds uncapped and without cameras.
        Worlds not generated are skipped."""
        for world in ("rover_test", "urc_delivery"):
            if not (WORLDS / f"{world}.sdf").exists():
                continue
            with self.subTest(world=world), world_copy(world) as copy:
                text = re.sub(r"<real_time_factor>[^<]*</real_time_factor>", "<real_time_factor>0</real_time_factor>",
                              Path(copy).read_text())
                with temp_sdf(text, WORLDS) as plain, world_file(plain, params=diffdrive()) as servo, \
                        world_file(plain, params=physical(cmd_timeout=0.0)) as torque:
                    costs = cpu_time_per_step({"diffdrive": servo, "physical": torque}, iterations=20_000, runs=5)
                ratio = costs["physical"].per_step / costs["diffdrive"].per_step
                print(f"{world}: physical / DiffDrive CPU time per step {ratio:.3f} ({costs})")
                self.assertLessEqual(ratio, 1.25)


if __name__ == "__main__":
    unittest.main()
