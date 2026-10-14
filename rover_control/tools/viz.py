#!/usr/bin/env python3
"""Interactive 2D/3D view of the rover driver kinematics.

Calls the C++ `rover_state` tool for every slider change and draws a top view
and a 3D view. `--snapshot out.png` renders once and exits (no window).
"""
import argparse
import json
import os
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import RadioButtons, Slider

# colcon builds rover_state in the workspace's build/rover_control (pixi run build).
EXE = Path(
    os.environ.get(
        "ROVER_STATE_EXE",
        Path(__file__).resolve().parents[3] / "build" / "rover_control" / "rover_state",
    )
)

# (flag, label, min, max, initial)
SLIDERS = [
    ("vx", "vx [m/s]", -1.5, 1.5, 0.5),
    ("vy", "vy [m/s]", -1.5, 1.5, 0.0),
    ("wz", "wz [rad/s]", -2.0, 2.0, 0.3),
    ("gx", "gyro wx [rad/s]", -1.0, 1.0, 0.0),
    ("gy", "gyro wy [rad/s]", -1.0, 1.0, 0.0),
    ("ql", "q left [rad]", -0.5, 0.5, 0.0),
    ("qr", "q right [rad]", -0.5, 0.5, 0.0),
    ("dql", "dq left [rad/s]", -1.0, 1.0, 0.0),
    ("dqr", "dq right [rad/s]", -1.0, 1.0, 0.0),
]
ONLY_3D = {"gx", "gy", "ql", "qr", "dql", "dqr"}  # rover_state ignores these in 2d mode
ARROW_SECONDS = 0.5  # velocity arrows show the displacement over this time
SIDE = {"FL": 0, "RL": 0, "FR": 1, "RR": 1}
TOP_VIEW_HALF_WIDTH = 1.5  # [m]
ICR_VIEW_MAX = 3.0  # [m] the top view grows to show a turning center up to this far out
# Top-view drawing order: the velocity arrows are the data, so they go above the
# outline and turning-center lines (zorder 2) and the wheel labels (text, 3).
WHEEL_ZORDER = 2.5
ARROW_ZORDER = 4
# 3D view: lower than matplotlib's default elevation of 30 deg, so rocker tilt
# and wheel heights read more easily.
VIEW_ELEV, VIEW_AZIM = 15.0, -70.0


def null_fields(value, path=""):
    """Paths of the null (None) entries in parsed rover_state JSON."""
    if value is None:
        return [path]
    if isinstance(value, dict):
        items = ((f"{path}.{k}" if path else k, v) for k, v in value.items())
    elif isinstance(value, list):
        items = ((f"{path}[{i}]", v) for i, v in enumerate(value))
    else:
        return []
    return [p for sub_path, v in items for p in null_fields(v, sub_path)]


def query(mode, values):
    cmd = [str(EXE), "--mode", mode]
    for flag, value in values.items():
        cmd += [f"--{flag}", f"{value:.6f}"]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"{' '.join(cmd)} exited with status {proc.returncode}:\n{proc.stderr.strip()}"
        )
    s = json.loads(proc.stdout)
    # rover_state prints null for a value that overflowed (JSON has no inf/nan).
    missing = null_fields(s)
    if missing:
        shown = ", ".join(missing[:3]) + (", ..." if len(missing) > 3 else "")
        raise RuntimeError(
            f"command too large: the kinematics overflowed (rover_state printed null for {shown})"
        )
    return s


def turning_center(cmd):
    """Instantaneous center of rotation (x, y) of the commanded twist; None if wz is 0."""
    if abs(cmd["wz"]) <= 1e-6:
        return None
    return np.array([-cmd["vy"] / cmd["wz"], cmd["vx"] / cmd["wz"]])


def draw_top(ax, s):
    ax.clear()
    ax.set_title(f"Top view ({s['mode']})")
    ax.set_aspect("equal")
    icr = turning_center(s["command"])
    half = TOP_VIEW_HALF_WIDTH
    if icr is not None and np.abs(icr).max() <= ICR_VIEW_MAX:
        half = max(half, np.abs(icr).max() + 0.25)
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.grid(alpha=0.3)
    ax.set_xlabel("x forward [m]")
    ax.set_ylabel("y left [m]")

    wheels = {w["name"]: w for w in s["wheels"]}
    outline = np.array([wheels[n]["center"][:2] for n in ("FL", "FR", "RR", "RL", "FL")])
    ax.plot(outline[:, 0], outline[:, 1], color="0.6")
    for p in s["geometry"]["pivot"]:
        ax.plot(p[0], p[1], "ks", ms=5)

    r = s["geometry"]["wheel_radius"]
    for w in s["wheels"]:
        c = np.array(w["center"][:2])
        f = np.array(w["forward"][:2])
        a = np.array(w["axle"][:2])
        corners = np.array(
            [c + r * f + 0.25 * r * a, c + r * f - 0.25 * r * a,
             c - r * f - 0.25 * r * a, c - r * f + 0.25 * r * a]
        )
        ax.fill(corners[:, 0], corners[:, 1],
                color="C0" if w["steer_defined"] else "0.7", alpha=0.8, zorder=WHEEL_ZORDER)
        v = np.array(w["velocity"][:2]) * ARROW_SECONDS
        if np.linalg.norm(v) > 1e-6:
            ax.arrow(c[0], c[1], v[0], v[1], width=0.01, color="C3",
                     length_includes_head=True, zorder=ARROW_ZORDER)
        label = "\n".join([
            w["name"],
            rf"$\psi$={np.degrees(w['steer_angle']):+.1f}°",
            rf"$\dot{{\theta}}$={w['wheel_rate']:+.2f} rad/s",
            rf"$\eta$={np.degrees(w['contact_angle']):+.1f}°",
        ])
        # Put the label off the wheel's outer corner, away from the chassis, on a
        # translucent box so ICR lines passing under it keep it legible. The
        # velocity arrows go above it (ARROW_ZORDER), so it never hides one.
        out = np.where(c >= 0.0, 1.0, -1.0)
        ax.annotate(
            label, c, xytext=10.0 * out, textcoords="offset points", fontsize=7,
            ha="left" if out[0] > 0 else "right", va="bottom" if out[1] > 0 else "top",
            bbox=dict(boxstyle="round,pad=0.2", fc="w", alpha=0.7, lw=0),
        )

    if icr is not None:
        ax.plot(icr[0], icr[1], "mx", ms=10)
        for w in s["wheels"]:
            ax.plot([w["center"][0], icr[0]], [w["center"][1], icr[1]], "m:", lw=0.8)


def draw_3d(ax, s):
    ax.clear()
    if s["mode"] == "2d":
        ax.set_title("3D view (2d mode: rockers level, gyro wx/wy ignored)")
    else:
        ax.set_title("3D view (body level; rockers at q)")
    r = s["geometry"]["wheel_radius"]
    pivots = np.array(s["geometry"]["pivot"])
    centers = {w["name"]: np.array(w["center"]) for w in s["wheels"]}

    xs = [c[0] for c in centers.values()]
    x0, x1 = min(xs), max(xs)
    zb = pivots[:, 2].mean()
    yl, yr = pivots[0, 1], pivots[1, 1]
    body = np.array([[x1, yl, zb], [x1, yr, zb], [x0, yr, zb], [x0, yl, zb], [x1, yl, zb]])
    ax.plot(*body.T, color="0.4", lw=2)

    t = np.linspace(0.0, 2.0 * np.pi, 40)
    lowest = 0.0
    for w in s["wheels"]:
        c = centers[w["name"]]
        p = pivots[SIDE[w["name"]]]
        ax.plot(*np.array([p, c]).T, color="C1", lw=3)
        f = np.array(w["forward"])
        u = np.array(w["up"])
        rim = c[:, None] + r * (np.outer(f, np.cos(t)) + np.outer(u, np.sin(t)))
        ax.plot(*rim, color="C0" if w["steer_defined"] else "0.6")
        # The contact direction, at eta from the module's -z toward its +x. Without
        # a defined steering (wheel not moving) eta is undefined (0), so none.
        if w["steer_defined"]:
            eta = w["contact_angle"]
            contact = c + r * (np.sin(eta) * f - np.cos(eta) * u)
            ax.plot(*np.array([c, contact]).T, color="C2")
            ax.scatter(*contact, color="C2", s=12)
        lowest = min(lowest, rim[2].min())  # the contact point lies on the rim
        v = np.array(w["velocity"]) * ARROW_SECONDS
        if np.linalg.norm(v) > 1e-6:
            ax.quiver(*c, *v, color="C3")

    q = s["suspension"]
    if abs(q["q_left"]) < 1e-9 and abs(q["q_right"]) < 1e-9:
        # Level rockers: all four wheels stand on the ground plane z = 0. With the
        # body drawn level and a rocker turned there is no single ground plane.
        gx, gy = np.meshgrid([-1.0, 1.0], [-1.0, 1.0])
        ax.plot_surface(gx, gy, np.zeros_like(gx), alpha=0.08, color="0.5")
    z0 = 0.0 if lowest > -1e-6 else lowest - 0.05  # keep lowered wheels above the box floor
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(z0, z0 + 1.0)
    ticks = np.linspace(-1.0, 1.0, 5)  # the default 9 crowd the flat viewing angle
    ax.set_xticks(ticks)
    ax.set_yticks(ticks)
    ax.set_box_aspect((2, 2, 1))
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")


def summary(s):
    cmd, fk = s["command"], s["fk"]
    lines = [f"command:  vx={cmd['vx']:+.3f}  vy={cmd['vy']:+.3f}  wz={cmd['wz']:+.3f}"]
    if s["mode"] == "2d":
        # forward_2d: closed-form least squares for (vx, vy, wz), no iterations.
        lines += [
            f"FK est.:  vx={fk['vx']:+.3f}  vy={fk['vy']:+.3f}  wz={fk['wz']:+.3f}",
            f"residual={fk['residual_rms']:.2e} m/s",
        ]
    else:
        # forward_3d estimates the body velocity and the contact angles; wz is the
        # gyro's. IK has no contact angle for a wheel whose steering is undefined.
        def angle(a, defined=True):
            return f"{np.degrees(a):+6.1f}°" if defined else "   n/a "

        wheels = s["wheels"]
        ik_eta = "  ".join(f"{w['name']}={angle(w['contact_angle'], w['steer_defined'])}"
                           for w in wheels)
        fk_eta = "  ".join(f"{w['name']}={angle(a)}" for w, a in zip(wheels, fk["contact_angle"]))
        lines += [
            f"FK est.:  vx={fk['vx']:+.3f}  vy={fk['vy']:+.3f}  vz={fk['vz']:+.3f}  "
            f"wz={fk['wz']:+.3f} (gyro)",
            f"eta IK:   {ik_eta}",
            f"eta FK:   {fk_eta}",
            f"residual={fk['residual_rms']:.2e} m/s  converged={fk['converged']}  "
            f"iterations={fk['iterations']}",
        ]
    return "\n".join(lines)


def mark_ignored_sliders(sliders, mode):
    """Grey out the sliders ({flag: Slider}) that rover_state ignores in `mode`."""
    for flag, slider in sliders.items():
        color = "0.6" if mode == "2d" and flag in ONLY_3D else plt.rcParams["text.color"]
        slider.label.set_color(color)
        slider.valtext.set_color(color)


def make_view_axes(fig):
    """The top-view and 3D axes of the viz figure."""
    ax_top = fig.add_axes([0.04, 0.38, 0.42, 0.58])
    ax_3d = fig.add_axes([0.5, 0.38, 0.48, 0.58], projection="3d")
    ax_3d.view_init(elev=VIEW_ELEV, azim=VIEW_AZIM)  # ax.clear() in draw_3d keeps it
    return ax_top, ax_3d


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--snapshot", type=Path, help="render once to this PNG and exit")
    parser.add_argument("--mode", choices=["2d", "3d"], default="3d")
    for flag, _, _, _, initial in SLIDERS:
        parser.add_argument(f"--{flag}", type=float, default=initial)
    args = parser.parse_args()
    if not EXE.exists():
        raise SystemExit(f"{EXE} not found - run `pixi run build` first")
    if args.snapshot:
        plt.switch_backend("Agg")

    fig = plt.figure(figsize=(13, 8))
    ax_top, ax_3d = make_view_axes(fig)
    text = fig.text(0.55, 0.30, "", family="monospace", fontsize=9, va="top")
    values = {flag: getattr(args, flag) for flag, *_ in SLIDERS}
    state = {"mode": args.mode}
    sliders = {}

    def redraw():
        s = query(state["mode"], values)
        draw_top(ax_top, s)
        draw_3d(ax_3d, s)
        text.set_text(summary(s))
        mark_ignored_sliders(sliders, s["mode"])
        fig.canvas.draw_idle()

    widgets = []  # keep references so the widgets stay alive
    if not args.snapshot:
        for k, (flag, label, lo, hi, _) in enumerate(SLIDERS):
            slider_ax = fig.add_axes([0.14, 0.30 - k * 0.03, 0.3, 0.02])
            slider = Slider(slider_ax, label, lo, hi, valinit=values[flag])

            def on_change(value, flag=flag):
                values[flag] = value
                redraw()

            slider.on_changed(on_change)
            widgets.append(slider)
            sliders[flag] = slider
        radio = RadioButtons(fig.add_axes([0.55, 0.05, 0.08, 0.12]), ("2d", "3d"),
                             active=0 if args.mode == "2d" else 1)

        def on_mode(label):
            state["mode"] = label
            redraw()

        radio.on_clicked(on_mode)
        widgets.append(radio)

    try:
        redraw()
    except RuntimeError as e:  # e.g. --vx nan: show rover_state's reason, not a traceback
        raise SystemExit(str(e)) from None
    if args.snapshot:
        fig.savefig(args.snapshot, dpi=110)
        return
    plt.show()


if __name__ == "__main__":
    main()
