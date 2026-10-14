#!/usr/bin/env python3
"""Tests for driver/tools/viz.py (ctest runs this as viz_py).

Uses the rover_state named by ROVER_STATE_EXE, like viz.py itself.
"""
import sys
import tempfile
import unicodedata
import unittest
import unittest.mock
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import to_rgba  # noqa: E402
from matplotlib.patches import FancyArrow  # noqa: E402
from matplotlib.transforms import Bbox  # noqa: E402
from matplotlib.widgets import Slider  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Path3DCollection, Poly3DCollection  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import viz  # noqa: E402

DEFAULTS = {flag: initial for flag, _, _, _, initial in viz.SLIDERS}
# The plan's 3D snapshot: left rocker front lowered, right rocker front raised.
TILTED = dict(DEFAULTS, vx=0.4, ql=0.15, qr=-0.15, dql=-0.4)
# Rockers twisted, rover still: no wheel has a defined steering or contact angle.
STILL_TWISTED = dict(DEFAULTS, vx=0.0, vy=0.0, wz=0.0, ql=0.3, qr=-0.3)
# Finite flags whose kinematics overflow: rover_state prints null for those values.
OVERFLOW = dict(DEFAULTS, vx=1e300, gx=1e300)


def rot_y(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def contact_points(s):
    r = s["geometry"]["wheel_radius"]
    points = []
    for w in s["wheels"]:
        c, f, u = (np.array(w[k]) for k in ("center", "forward", "up"))
        eta = w["contact_angle"]
        points.append(c + r * (np.sin(eta) * f - np.cos(eta) * u))
    return np.array(points)


class FigureTest(unittest.TestCase):
    def tearDown(self):
        plt.close("all")


class QueryTest(unittest.TestCase):
    def test_failure_reports_rover_state_reason(self):
        with self.assertRaisesRegex(RuntimeError, r"invalid value 'nan' for --vx"):
            viz.query("3d", {"vx": float("nan")})

    def test_null_fields_lists_paths(self):
        self.assertEqual(viz.null_fields({"a": [1.0, None], "b": {"c": None, "d": "x"}}),
                         ["a[1]", "b.c"])
        self.assertEqual(viz.null_fields({"a": [0.0, False]}), [])

    def test_overflow_is_reported_not_drawn(self):
        with self.assertRaisesRegex(RuntimeError, r"command too large.*null for wheels\["):
            viz.query("3d", OVERFLOW)


class MainTest(FigureTest):
    def run_main(self, *args):
        with tempfile.TemporaryDirectory() as tmp:
            png = Path(tmp) / "out.png"
            argv = ["viz.py", *args, "--snapshot", str(png)]
            with unittest.mock.patch.object(sys, "argv", argv):
                viz.main()
            return png.exists()

    def test_snapshot(self):
        self.assertTrue(self.run_main("--mode", "3d"))

    def test_overflow_exits_with_reason(self):
        # Used to die in a label's format spec with a TypeError traceback.
        with self.assertRaisesRegex(SystemExit, r"command too large"):
            self.run_main("--vx", "1e300", "--gx", "1e300")


class TopViewTest(FigureTest):
    def draw(self, mode, values):
        fig, ax = plt.subplots()
        s = viz.query(mode, values)
        viz.draw_top(ax, s)
        fig.canvas.draw()  # also parses the mathtext labels
        return fig, ax, s

    def test_default_turning_center_is_in_view(self):
        _, ax, _ = self.draw("2d", DEFAULTS)
        icr = (-DEFAULTS["vy"] / DEFAULTS["wz"], DEFAULTS["vx"] / DEFAULTS["wz"])  # (0, 1.67)
        (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
        self.assertTrue(x0 < icr[0] < x1 and y0 < icr[1] < y1, (ax.get_xlim(), ax.get_ylim()))

    def test_distant_turning_center_keeps_default_view(self):
        _, ax, _ = self.draw("2d", dict(DEFAULTS, wz=0.01))  # turning center 50 m out
        np.testing.assert_allclose(ax.get_xlim(), (-1.5, 1.5))
        np.testing.assert_allclose(ax.get_ylim(), (-1.5, 1.5))

    def test_labels_have_no_combining_characters(self):
        # matplotlib does not place combining marks: "θ̇" (U+0307) rendered as "θ".
        _, ax, _ = self.draw("2d", DEFAULTS)
        labels = [t.get_text() for t in ax.texts]
        self.assertEqual(len(labels), 4)
        for label in labels:
            self.assertFalse(any(unicodedata.combining(ch) for ch in label), label)
            self.assertIn(r"$\dot{\theta}$", label)

    def test_arrows_drawn_above_outline_lines_and_labels(self):
        # Rear arrows run along the outline when driving straight; a label box
        # can sit on an arrowhead in a turn. The arrows must stay on top.
        for values in (dict(DEFAULTS, wz=0.0), dict(DEFAULTS, wz=0.5)):
            _, ax, _ = self.draw("2d", values)
            arrows = [p for p in ax.patches if isinstance(p, FancyArrow)]
            wheels = [p for p in ax.patches if not isinstance(p, FancyArrow)]
            self.assertEqual(len(arrows), 4)
            self.assertEqual(len(wheels), 4)
            below = [a.get_zorder() for a in [*ax.lines, *ax.texts, *wheels]]
            self.assertGreater(min(a.get_zorder() for a in arrows), max(below))
            outline = max(line.get_zorder() for line in ax.lines)
            self.assertGreater(min(w.get_zorder() for w in wheels), outline)

    def test_labels_stay_off_the_chassis(self):
        fig, ax, s = self.draw("3d", TILTED)
        renderer = fig.canvas.get_renderer()
        centers = np.array([w["center"][:2] for w in s["wheels"]])
        chassis = Bbox(ax.transData.transform([centers.min(axis=0), centers.max(axis=0)]))
        for text in ax.texts:
            self.assertFalse(text.get_window_extent(renderer).overlaps(chassis),
                             text.get_text().splitlines()[0])


class View3dTest(FigureTest):
    def draw(self, mode, values):
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        s = viz.query(mode, values)
        viz.draw_3d(ax, s)
        return ax, s

    def ground_planes(self, ax):
        return [c for c in ax.collections if isinstance(c, Poly3DCollection)]

    def test_title_in_2d_mode_says_rockers_level(self):
        ax, s = self.draw("2d", dict(DEFAULTS, ql=0.3, gx=0.5))
        self.assertEqual(s["suspension"]["q_left"], 0)  # rover_state ignores ql in 2d
        self.assertIn("rockers level", ax.get_title())
        self.assertNotIn("rockers at q", ax.get_title())

    def test_title_in_3d_mode_says_rockers_at_q(self):
        ax, _ = self.draw("3d", TILTED)
        self.assertIn("rockers at q", ax.get_title())

    def test_ground_plane_when_rockers_level(self):
        ax, _ = self.draw("3d", DEFAULTS)
        self.assertEqual(len(self.ground_planes(ax)), 1)
        self.assertEqual(ax.get_zlim()[0], 0.0)

    def contact_markers(self, ax):
        return [c for c in ax.collections if isinstance(c, Path3DCollection)]

    def test_contact_marker_only_where_contact_angle_is_defined(self):
        ax, s = self.draw("3d", TILTED)
        self.assertTrue(all(w["steer_defined"] for w in s["wheels"]))
        self.assertEqual(len(self.contact_markers(ax)), 4)
        # A still wheel has no contact angle (0 is a placeholder): a marker along
        # the tilted rocker's -z would look like a contact-angle error.
        ax, s = self.draw("3d", STILL_TWISTED)
        self.assertFalse(any(w["steer_defined"] for w in s["wheels"]))
        self.assertEqual(self.contact_markers(ax), [])

    def test_view_angle_survives_redraw(self):
        fig = plt.figure()
        _, ax = viz.make_view_axes(fig)
        for values in (DEFAULTS, TILTED):
            viz.draw_3d(ax, viz.query("3d", values))
            self.assertEqual((ax.elev, ax.azim), (viz.VIEW_ELEV, viz.VIEW_AZIM))

    def test_no_ground_plane_under_tilted_rockers(self):
        ax, s = self.draw("3d", TILTED)
        contacts = contact_points(s)
        self.assertLess(contacts[:, 2].min(), -0.05)  # FL and RR stand below z = 0
        self.assertEqual(self.ground_planes(ax), [])
        self.assertLess(ax.get_zlim()[0], contacts[:, 2].min())


class RoverStateJsonTest(unittest.TestCase):
    def test_wheel_centers_follow_geometry(self):
        s = viz.query("3d", TILTED)
        geometry, q = s["geometry"], s["suspension"]
        self.assertEqual(len(geometry["wheel_offset"]), 4)
        for w, offset in zip(s["wheels"], geometry["wheel_offset"]):
            side = viz.SIDE[w["name"]]
            angle = q["q_left"] if side == 0 else q["q_right"]
            expected = np.array(geometry["pivot"][side]) + rot_y(angle) @ np.array(offset)
            # rover_state prints 9 significant digits.
            np.testing.assert_allclose(w["center"], expected, rtol=0.0, atol=1e-8,
                                       err_msg=w["name"])

    def test_fk_contact_angles_in_3d_mode_only(self):
        # Round trip on consistent data: FK recovers the IK contact angles up to
        # the contact-angle prior's pull, ~(1e-3 / wheel speed)^2 rad.
        s = viz.query("3d", TILTED)
        ik = [w["contact_angle"] for w in s["wheels"]]
        np.testing.assert_allclose(s["fk"]["contact_angle"], ik, atol=1e-4)
        self.assertGreater(max(abs(a) for a in ik), 0.1)  # not trivially zero
        self.assertNotIn("contact_angle", viz.query("2d", DEFAULTS)["fk"])


class SummaryTest(unittest.TestCase):
    def test_2d_shows_only_what_forward_2d_estimates(self):
        text = viz.summary(viz.query("2d", DEFAULTS))
        fk_line = text.splitlines()[1]
        self.assertTrue(fk_line.startswith("FK est.:"), text)
        self.assertIn("wz=", fk_line)
        for absent in ("vz=", "converged", "iterations", "eta", "gyro"):
            self.assertNotIn(absent, text)

    def test_3d_marks_gyro_wz_and_compares_contact_angles(self):
        s = viz.query("3d", TILTED)
        lines = viz.summary(s).splitlines()
        self.assertTrue(lines[1].startswith("FK est.:"), lines)
        self.assertRegex(lines[1], r"vz=[-+]\d\.\d{3}  wz=[-+]\d\.\d{3} \(gyro\)$")
        ik_line, fk_line = lines[2], lines[3]
        self.assertTrue(ik_line.startswith("eta IK:") and fk_line.startswith("eta FK:"), lines)
        for w, fk_eta in zip(s["wheels"], s["fk"]["contact_angle"]):
            self.assertIn(f"{w['name']}={np.degrees(w['contact_angle']):+6.1f}°", ik_line)
            self.assertIn(f"{w['name']}={np.degrees(fk_eta):+6.1f}°", fk_line)
        self.assertIn("iterations=", lines[4])

    def test_3d_ik_contact_angle_na_without_steering(self):
        lines = viz.summary(viz.query("3d", STILL_TWISTED)).splitlines()
        self.assertEqual(lines[2].count("n/a"), 4, lines[2])


class SliderTest(FigureTest):
    def test_2d_mode_greys_exactly_the_ignored_sliders(self):
        # A flag is ignored in 2d mode if changing it leaves rover_state's output unchanged.
        base = viz.query("2d", DEFAULTS)
        ignored = {flag for flag in DEFAULTS
                   if viz.query("2d", dict(DEFAULTS, **{flag: DEFAULTS[flag] + 0.1})) == base}
        self.assertEqual(ignored, {"gx", "gy", "ql", "qr", "dql", "dqr"})

        fig = plt.figure()
        sliders = {flag: Slider(fig.add_axes([0.3, 0.05 + 0.1 * k, 0.5, 0.05]), label, lo, hi,
                                valinit=init)
                   for k, (flag, label, lo, hi, init) in enumerate(viz.SLIDERS)}
        normal = to_rgba(plt.rcParams["text.color"])
        viz.mark_ignored_sliders(sliders, "2d")
        for flag, slider in sliders.items():
            for text in (slider.label, slider.valtext):
                self.assertEqual(to_rgba(text.get_color()) != normal, flag in ignored, flag)
        viz.mark_ignored_sliders(sliders, "3d")
        for flag, slider in sliders.items():
            for text in (slider.label, slider.valtext):
                self.assertEqual(to_rgba(text.get_color()), normal, flag)


if __name__ == "__main__":
    unittest.main(verbosity=2)
