"""G8 (part 1): wheel loads from JointTransmittedWrench vs ContactSensorData on a 15 deg slope.

Usage: python g8_loads.py <case> <solver> <joint_sign>   (one TestFixture run per process)
case: up (parked heading uphill), across (parked across the slope), drive (0.2 m/s uphill)
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

case, solver, sign = sys.argv[1], sys.argv[2], float(sys.argv[3])
SLOPE = math.radians(15.0)
yaw = {"up": 0.0, "across": math.pi / 2, "drive": 0.0}[case]
log = G.SCRATCH / "results" / f"g8_{case}_{solver}.bin"
uri = G.proto_rover(f"g8_both_{case}_{solver}", load_source="both", log=str(log), joint_sign=sign)
os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] += os.pathsep + str(G.PROTO_BUILD)
# A 40 x 20 m plane rising towards +x at 15 deg; the rover's base_link (between the wheels, on the
# ground) at its centre, tilted with it.
R = np.array([[math.cos(SLOPE), 0, -math.sin(SLOPE)], [0, 1, 0], [math.sin(SLOPE), 0, math.cos(SLOPE)]])  # pitch -15
Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
from urc import sdf as U  # noqa: E402
roll, pitch, yaw_ = U.matrix_to_rpy((R @ Rz).tolist())
normal = R @ np.array([0, 0, 1.0])
spawn = 0.02 * normal
world = f"""<?xml version="1.0"?>
<sdf version="1.11"><world name="g8slope">
  <physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor>
    {'<dart><solver><solver_type>pgs</solver_type></solver></dart>' if solver == 'pgs' else ''}</physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <model name="slope"><static>true</static><pose>0 0 0 0 {-SLOPE} 0</pose><link name="l">
    <collision name="c"><pose>0 0 -0.1 0 0 0</pose><geometry><box><size>40 20 0.2</size></box></geometry></collision></link></model>
  <include><uri>{uri}</uri><name>rover</name><pose>{spawn[0]} {spawn[1]} {spawn[2]} {roll} {pitch} {yaw_}</pose></include>
</world></sdf>"""
cmd = [(0.0, 0.0, 0.0), (1.0, 0.2, 0.0)] if case == "drive" else (0.0, 0.0)
with G.S.temp_sdf(world) as path:
    state = G.S.simulate(4.0, world=path, cmd=cmd)
rec = np.fromfile(log, dtype=np.float64)
cols = 1 + 4 * 11 + 1
rec = rec[: len(rec) // cols * cols].reshape(-1, cols)
t = rec[:, 0]
window = (t >= 3.0) & (t <= 4.0)
out = {"case": case, "solver": solver, "joint_sign": sign, "wheels": {}}
for k, w in enumerate(("fl", "rl", "fr", "rr")):
    base = 1 + 11 * k
    fn, fnj = rec[window, base + 6], rec[window, base + 10]
    out["wheels"][w] = dict(contact_N=round(float(fn.mean()), 2), joint_N=round(float(fnj.mean()), 2),
                            ratio=round(float(fnj.mean() / fn.mean()), 4) if fn.mean() else None,
                            contact_std=round(float(fn.std()), 2), joint_std=round(float(fnj.std()), 2))
tot_c = sum(v["contact_N"] for v in out["wheels"].values())
tot_j = sum(v["joint_N"] for v in out["wheels"].values())
out["sum_contact_N"], out["sum_joint_N"] = round(tot_c, 2), round(tot_j, 2)
out["rover_weight_normal_N"] = round(9.81 * (30 + 2 * 3 + 4 * 2.5 + 0.04) * math.cos(SLOPE), 2)
out["final_pose"] = [round(v, 3) for v in state.poses["base_link"]]
print(json.dumps(out))
G.save(f"g8_loads_{case}_{solver}", out)
log.unlink()
