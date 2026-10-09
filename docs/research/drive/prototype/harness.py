"""Headless 1 kHz logger for rover drivetrain experiments (research prototype, not repo code).

One run per process:  python harness.py <config.json>
config keys:
  world        path to an SDF world (must include model://rover named 'rover')
  models       list of resource dirs (prepended to GZ_SIM_RESOURCE_PATH)
  seconds      sim time
  schedule     list of [t_start, vx, wz]   (piecewise-constant body command)
  mode         'diffdrive' | 'motor' | 'motor_cpp'
  motor        dict of motor-model parameters (mode 'motor')
  out          output .npz path
"""
import json
import math
import os
import sys
from pathlib import Path

cfg = json.load(open(sys.argv[1]))
REPO = Path("/Users/alarion239/Desktop/Rover")
os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(cfg.get("models", []) + [str(REPO / "sim/models")])
os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(cfg.get("plugins", []) + [str(REPO / "sim/build")])
os.environ["GZ_PARTITION"] = f"drivetrain_research_{os.getpid()}"
os.environ["GZ_IP"] = "127.0.0.1"
os.environ.setdefault("OGRE2_RESOURCE_PATH", str(Path(sys.prefix) / "lib" / "OGRE-Next"))

import numpy as np  # noqa: E402
import gz.math7  # noqa: E402,F401
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
from gz.sim8 import Joint, Link, Model, TestFixture, World, world_entity  # noqa: E402
from gz.transport13 import Node  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from motor import Drivetrain  # noqa: E402

WHEELS = ("fl", "rl", "fr", "rr")  # left side first
SIDE = {"fl": +1, "rl": +1, "fr": -1, "rr": -1}  # +1 left
TRACK = 0.8
R = 0.15


def command(t):
    vx = wz = 0.0
    for t0, v, w in cfg["schedule"]:
        if t >= t0:
            vx, wz = v, w
    return vx, wz


node = Node()
pub = node.advertise("/model/rover/cmd_vel", Twist)
H = {}
log = []
mode = cfg["mode"]
drive = Drivetrain(cfg.get("motor", {}), dt=0.001) if mode == "motor" else None
ext = {}  # extra per-step values from the motor model


def setup(ecm):
    model = Model(World(world_entity(ecm)).model_by_name(ecm, "rover"))
    H["wj"] = [Joint(model.joint_by_name(ecm, f"wheel_{w}_joint")) for w in WHEELS]
    H["rj"] = [Joint(model.joint_by_name(ecm, n)) for n in ("rocker_left_joint", "rocker_right_joint")]
    H["base"] = Link(model.link_by_name(ecm, "base_link"))
    H["wl"] = [Link(model.link_by_name(ecm, f"wheel_{w}")) for w in WHEELS]
    for j in H["wj"] + H["rj"]:
        j.enable_position_check(ecm, True)
        j.enable_velocity_check(ecm, True)
    for link in [H["base"]] + H["wl"]:
        link.enable_velocity_checks(ecm, True)
        link.enable_acceleration_checks(ecm, True)


def pre_update(info, ecm):
    if not H:
        setup(ecm)
    t = info.sim_time.total_seconds() if hasattr(info.sim_time, "total_seconds") else info.iterations * 1e-3
    vx, wz = command(t)
    if mode == "diffdrive":
        if info.iterations % 20 == 0:
            tw = Twist()
            tw.linear.x, tw.angular.z = vx, wz
            pub.publish(tw)
    elif mode == "motor":
        w = []
        for j in H["wj"]:
            v = j.velocity(ecm)
            w.append(v[0] if v else 0.0)
        # wheel speed setpoints from the body twist (ideal diff-drive kinematics)
        sp = [(vx - SIDE[n] * wz * TRACK / 2) / R for n in WHEELS]
        tau = drive.step(sp, w)
        for j, tq in zip(H["wj"], tau):
            j.set_force(ecm, [tq])
    elif mode == "motor_cpp":
        if info.iterations % 20 == 0:
            tw = Twist()
            tw.linear.x, tw.angular.z = vx, wz
            pub.publish(tw)


def post_update(info, ecm):
    if not H:
        return
    t = info.iterations * 1e-3
    p = H["base"].world_pose(ecm)
    pos, rot = p.pos(), p.rot()
    e = rot.euler()
    av = H["base"].world_angular_velocity(ecm)
    lv = H["base"].world_linear_velocity(ecm)
    la = H["base"].world_linear_acceleration(ecm)
    row = [t, pos.x(), pos.y(), pos.z(), e.x(), e.y(), e.z()]
    row += [av.x(), av.y(), av.z()] if av else [0, 0, 0]
    # body-frame linear velocity / acceleration
    if lv:
        bv = rot.rotate_vector_reverse(lv)
        row += [bv.x(), bv.y(), bv.z()]
    else:
        row += [0, 0, 0]
    if la:
        ba = rot.rotate_vector_reverse(la)
        row += [ba.x(), ba.y(), ba.z()]
    else:
        row += [0, 0, 0]
    for j in H["wj"]:
        q = j.position(ecm)
        v = j.velocity(ecm)
        tq = float("nan")
        row += [q[0] if q else 0, v[0] if v else 0, tq]
    for link in H["wl"]:
        wp = link.world_pose(ecm)
        v = link.world_linear_velocity(ecm)
        if v:
            ax = wp.rot().rotate_vector(gz.math7.Vector3d(0, 1, 0))  # axle in world
            fw = ax.cross(gz.math7.Vector3d(0, 0, 1)); fw = fw.normalized() if fw.length() > 1e-9 else fw
            row += [v.dot(fw), v.dot(ax), wp.pos().z()]  # hub velocity: forward, along axle
        else:
            row += [0, 0, wp.pos().z()]
    for j in H["rj"]:
        q = j.position(ecm)
        row += [q[0] if q else 0]
    if drive is not None:
        row += list(drive.last_current) + list(drive.last_tau)
    log.append(row)


fixture = TestFixture(cfg["world"])
fixture.on_pre_update(pre_update)
fixture.on_post_update(post_update)
fixture.finalize()
fixture.server().run(True, round(cfg["seconds"] * 1000), False)
cols = ["t", "x", "y", "z", "roll", "pitch", "yaw", "wx", "wy", "wz", "vbx", "vby", "vbz", "abx", "aby", "abz"]
for w in WHEELS:
    cols += [f"q_{w}", f"w_{w}", f"tq_{w}"]
for w in WHEELS:
    cols += [f"hvx_{w}", f"hvy_{w}", f"hz_{w}"]
cols += ["rock_l", "rock_r"]
if drive is not None:
    cols += [f"i_{w}" for w in WHEELS] + [f"tau_{w}" for w in WHEELS]
np.savez(cfg["out"], data=np.array(log), cols=np.array(cols), cfg=json.dumps(cfg))
print("saved", cfg["out"], len(log), "rows")
