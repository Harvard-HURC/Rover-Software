#!/usr/bin/env python3
"""Tests for the mission judges, sim/urc/judge.py, on made-up observations."""
import math
import unittest

import worldfiles

from urc import judge as J  # noqa: E402  (worldfiles puts sim/ on the path)
from urc import lander, rules  # noqa: E402


def sheet(mission):
    return worldfiles.sheet(f"urc_{mission}")


def run(judge, t0, t1, rover, dt=0.1, **obs):
    """Step a judge with the same observation from t0 to t1; returns all actions."""
    actions = []
    t = t0
    while t < t1:
        actions += judge.step(J.Observation(t, rover, **obs))
        t += dt
    return actions


class Geometry(unittest.TestCase):
    def test_footprint_distance(self):
        self.assertAlmostEqual(J.footprint_distance((0, 0, 0, 0), (1.0, 0)), 0.4)
        self.assertAlmostEqual(J.footprint_distance((0, 0, 0, 0), (0, 1.0)), 0.55)
        self.assertAlmostEqual(J.footprint_distance((0, 0, 0, math.pi / 2), (0, 1.0)), 0.4)
        self.assertEqual(J.footprint_distance((5, 5, 0, 1.0), (5.2, 5.1)), 0.0)

    def test_stopped(self):
        m = J.Motion()
        for k in range(30):
            m.update(k * 0.1, (0.0, 0.0, 0, 0))
        self.assertTrue(m.stopped)
        m = J.Motion()
        for k in range(30):
            m.update(k * 0.1, (0.3 * k * 0.1, 0.0, 0, 0))
        self.assertFalse(m.stopped)

    def test_terrain_matches_the_sheet(self):
        s = sheet("autonomy")
        terrain = worldfiles.terrain("urc_autonomy")
        for key in ("route_start", "post1", "post2", "astronaut_wait"):
            p = s["points"][key]
            self.assertAlmostEqual(terrain.height(p["x"], p["y"]), p["z"], delta=0.02, msg=key)


class RouteFinding(unittest.TestCase):
    def setUp(self):
        self.sheet = sheet("autonomy")
        self.post = self.sheet["points"]["post1"]

    def near_post(self, d):
        return (self.post["x"] - J.ROVER_HALF_LENGTH - d, self.post["y"], self.post["z"], 0.0)

    def test_needs_the_arrival_led(self):
        j = J.RouteFinding(self.sheet)
        run(j, 0, 5, self.near_post(0.5), led="red")
        self.assertNotIn("post1", j.scores)
        run(j, 5, 8, self.near_post(0.5), led="green")
        self.assertEqual(j.scores["post1"], 25)

    def test_needs_to_be_within_a_metre(self):
        j = J.RouteFinding(self.sheet)
        run(j, 0, 5, self.near_post(1.3), led="green")
        self.assertNotIn("post1", j.scores)


class AstronautAssistance(unittest.TestCase):
    def setUp(self):
        self.sheet = sheet("autonomy")
        self.terrain = worldfiles.terrain("urc_autonomy")
        self.hammer = self.sheet["objects"]["rock_pick_hammer"]

    def judge(self, method="device"):
        return J.AstronautAssistance(self.sheet, self.terrain, method=method)

    def hammer_at(self, x, y, held):
        z = self.terrain.height(x, y) + (0.5 if held else 0.0)
        return {"rock_pick_hammer": (x, y, z, 0, 0, 0)}

    def test_all_commands(self):
        """A perfect rover: it shadows the astronaut and carries the hammer."""
        j = self.judge()
        rover = (self.sheet["rover_start"]["x"], self.sheet["rover_start"]["y"], 0, 0)
        hx, hy, held = self.hammer["x"], self.hammer["y"], False
        said = []
        t = 0.0
        while t < 900 and j.phase < len(j.PHASES):
            phase = j.PHASES[j.phase]
            ax, ay = j.astronaut
            if j.pending is None and (phase in ("goto_astronaut", "come") or (phase == "follow" and not j._walking())):
                rover = (ax - 2.5, ay, 0, 0)
            if phase == "fetch" and t - j.phase_start > 3:
                held = True
            if phase == "give" and t - j.phase_start > 5:
                held = False
            if held:
                hx, hy = rover[:2]
            actions = j.step(J.Observation(t, rover, "red", self.hammer_at(hx, hy, held)))
            said += [a.text for a in actions if isinstance(a, J.Say)]
            t += 0.1
        self.assertEqual(said, ["follow", "stay", "fetch", "come", "give"])
        self.assertEqual(j.scores, {"goto_astronaut": 5, "follow": 5, "stay": 5, "fetch": 10, "come": 5, "give": 5})

    def test_route_finding_first_costs_nothing(self):
        """The astronaut waits: no timeouts, no points, until the rover comes."""
        j = self.judge()
        post = self.sheet["points"]["post1"]
        run(j, 0, 1300, (post["x"], post["y"] - 1, 0, 0), dt=1.0, led="red",
            poses=self.hammer_at(self.hammer["x"], self.hammer["y"], False))
        self.assertEqual(j.scores, {})
        self.assertEqual(j.phase, 0)

    def test_teleoperated_arrival_scores_nothing_but_goes_on(self):
        j = self.judge()
        ax, ay = j.astronaut
        run(j, 0, 4, (ax - 2.5, ay, 0, 0), led="blue")
        self.assertEqual(j.scores["goto_astronaut"], 0)
        self.assertEqual(j.PHASES[j.phase], "follow")

    def advance_to(self, j, phase, rover, poses=None):
        j.phase = j.PHASES.index(phase) - 1
        j._next([], J.Observation(j.t, rover, poses=poses or {}), False)

    def test_stay_fails_if_the_rover_moves(self):
        j = self.judge()
        x, y = j.astronaut
        self.advance_to(j, "stay", (x - 2, y, 0, 0))
        run(j, 0, 3, (x - 2, y, 0, 0))
        run(j, 3, 6, (x - 4, y, 0, 0))
        self.assertEqual(j.scores["stay"], 0)

    def test_stay_needs_the_rover_with_the_astronaut(self):
        j = self.judge()
        x, y = j.astronaut
        far = (x - 40, y, 0, 0)
        self.advance_to(j, "stay", far)
        run(j, 0, 120, far)
        self.assertEqual(j.scores["stay"], 0)

    def test_come_is_given_from_20_m(self):
        j = self.judge()
        x, y = j.astronaut
        rover = (x - 3, y, 0, 0)
        self.advance_to(j, "come", rover)
        self.assertEqual(j.pending, "come")
        actions = run(j, 0, 60, rover)
        self.assertIn("come", [a.text for a in actions if isinstance(a, J.Say)])
        ax, ay = j.astronaut
        self.assertGreaterEqual(math.hypot(ax - rover[0], ay - rover[1]), rules.COME_MIN_DISTANCE)
        self.assertNotIn("come", j.scores)

    def test_fetch_counts_only_after_the_command(self):
        j = self.judge()
        x, y = j.astronaut
        rover = (x - 3, y, 0, 0)
        j.phase = j.PHASES.index("fetch") - 1
        j._next([], J.Observation(0, rover), True)  # already holding it
        run(j, 0, 2, rover, led="red", poses=self.hammer_at(x - 3, y, True))
        self.assertEqual(j.scores["fetch"], 0)

    def test_give_needs_the_hammer_held_and_dropped_by_the_astronaut(self):
        j = self.judge()
        x, y = j.astronaut
        rover = (x - 2.5, y, 0, 0)
        self.advance_to(j, "give", rover)  # not holding the hammer at Give!
        run(j, 0, 2, rover, poses=self.hammer_at(x - 2.5, y, False))
        self.assertEqual(j.scores["give"], 0)
        j = self.judge()
        j.phase = j.PHASES.index("give") - 1
        j._next([], J.Observation(0, rover), True)
        run(j, 0, 2, rover, poses=self.hammer_at(x - 30, y, False))  # dropped far away
        self.assertEqual(j.scores["give"], 0)

    def test_gestures_differ_by_command(self):
        j = self.judge(method="gesture")
        targets = {}
        for word in ("follow", "stay", "fetch", "give"):
            j._command(word, [])
            actions = run(j, j.t, j.t + 4, (0, 0, 0, 0))
            targets[word] = [a.targets for a in actions if isinstance(a, J.JointTargets)]
            self.assertTrue(targets[word], word)
        self.assertTrue(any(t.get("elbow_right", 0) > 1.5 for t in targets["follow"]))
        self.assertTrue(any(t.get("shoulder_left_pitch", 0) > 1.0 for t in targets["give"]))
        self.assertNotEqual(targets["stay"][0], targets["fetch"][0])


class EquipmentServicing(unittest.TestCase):
    def test_typing_with_backspace(self):
        text = ""
        for key in ("key_a", "key_x", "key_backspace", "key_b", "key_lshift", "key_c"):
            text = J.type_text(text, key)
        self.assertEqual(text, "abc")

    def test_launch_key(self):
        for seed in range(20):
            key = J.launch_key(seed)
            self.assertTrue(3 <= len(key) <= 6 and key.isalpha() and key.islower())
        self.assertEqual(J.launch_key(4), J.launch_key(4))

    def test_typing_and_lander_tasks(self):
        s = sheet("equipment_servicing")
        j = J.EquipmentServicing(s, key="sol")
        rover = (0, 0, 0, 0)
        j.step(J.Observation(0, rover, presses=["key_s", "key_p", "key_backspace", "key_o", "key_l"]))
        self.assertEqual(j.display, "sol")
        self.assertIn("typing", j.scores)
        joints = {"lander::latch": 1.5, "lander::door": 1.4, "lander::valve": 1.5, "lander::switch_1": 0.8,
                  "lander::knob_0": -1.6}
        j.step(J.Observation(1, rover, joints=joints, presses=["button_2"]))
        for task in ("panel", "valve", "controls"):
            self.assertIn(task, j.scores)

    def test_cache_in_the_drawer(self):
        s = sheet("equipment_servicing")
        j = J.EquipmentServicing(s, key="abc")
        lx, ly, lz, yaw = j.lander_pose

        def world(p):
            c, si = math.cos(yaw), math.sin(yaw)
            return (lx + c * p[0] - si * p[1], ly + si * p[0] + c * p[1], lz + p[2], 0, 0, 0)

        well = lander.DRAWER_WELL_CLOSED
        open_well = world((well[0] + 0.3, well[1], well[2]))
        j.step(J.Observation(0, (0, 0, 0, 0), joints={"lander::drawer": 0.3}, poses={"cache_container": open_well}))
        self.assertNotIn("drawer", j.scores)
        j.step(J.Observation(1, (0, 0, 0, 0), joints={"lander::drawer": 0.0}, poses={"cache_container": world(well)}))
        self.assertIn("drawer", j.scores)


class Delivery(unittest.TestCase):
    def setUp(self):
        self.sheet = sheet("delivery")
        self.terrain = worldfiles.terrain("urc_delivery")
        self.each = 100 / len(self.sheet["tasks"])

    def at(self, name, point, dx=1.0, lift=0.0):
        x, y = point["x"] + dx, point["y"]
        return {name: (x, y, self.terrain.height(x, y) + lift, 0, 0, 0)}

    def test_delivered_when_resting_at_the_astronaut(self):
        j = J.Delivery(self.sheet, self.terrain)
        a = self.sheet["points"]["astronaut_a"]
        run(j, 0, 2, (0, 0, 0, 0), poses=self.at("wrench", a))
        self.assertAlmostEqual(j.scores["D1"], self.each)
        run(j, 2, 4, (0, 0, 0, 0), poses=self.at("water_jug", a, dx=30))
        self.assertNotIn("D2", j.scores)

    def test_partial_credit_counts_towards_the_share(self):
        j = J.Delivery(self.sheet, self.terrain)
        c = self.sheet["points"]["astronaut_c"]
        spectro = self.sheet["objects"]["spectrometer"]
        run(j, 0, 1, (spectro["x"] - 1.5, spectro["y"], 0, 0), poses=self.at("spectrometer", spectro, dx=0))
        self.assertAlmostEqual(j.scores["D6"], self.each / 4, places=1)
        run(j, 1, 3, (0, 0, 0, 0), poses=self.at("spectrometer", c))
        self.assertAlmostEqual(j.scores["D6"], self.each)
        self.assertAlmostEqual(j.summary()["points"], self.each, places=2)
        self.assertAlmostEqual(j.scores["D6"], self.each)  # summary() leaves the scores alone

    def test_held_object_is_not_delivered(self):
        """Still in the gripper (off the ground) next to the astronaut, with the
        same stale pose arriving tick after tick: not delivered."""
        j = J.Delivery(self.sheet, self.terrain)
        a = self.sheet["points"]["astronaut_a"]
        run(j, 0, 3, (0, 0, 0, 0), poses=self.at("wrench", a, lift=0.6))
        self.assertNotIn("D1", j.scores)


class Astrobiology(unittest.TestCase):
    def test_two_sites_then_home(self):
        s = sheet("astrobiology")
        j = J.Astrobiology(s)
        c2 = s["c2"]
        run(j, 0, 35, (c2["x"] + 50, c2["y"], 0, 0))
        run(j, 35, 70, (c2["x"] + 100, c2["y"], 0, 0))
        self.assertEqual(len(j.sites), 2)
        self.assertIn("sites", j.scores)
        run(j, 70, 72, (c2["x"] + 3, c2["y"], 0, 0))
        self.assertIn("returned", j.scores)

    def test_idling_at_the_start_is_not_a_site_and_coming_back_counts(self):
        s = sheet("astrobiology")
        j = J.Astrobiology(s)
        start, c2 = s["rover_start"], s["c2"]
        run(j, 0, 40, (start["x"], start["y"], 0, 0))
        self.assertEqual(j.sites, [])
        run(j, 40, 75, (c2["x"] + 50, c2["y"], 0, 0))
        run(j, 75, 110, (c2["x"] + 100, c2["y"], 0, 0))
        run(j, 110, 113, (start["x"], start["y"], 0, 0))
        self.assertEqual(set(j.scores), {"sites", "returned"})

    def test_sites_must_be_inside_half_a_kilometre(self):
        s = sheet("astrobiology")
        j = J.Astrobiology(s)
        c2 = s["c2"]
        run(j, 0, 35, (c2["x"] + rules.SITE_RADIUS + 20, c2["y"], 0, 0))
        self.assertEqual(j.sites, [])


if __name__ == "__main__":
    unittest.main()
