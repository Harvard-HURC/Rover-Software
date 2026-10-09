"""Blocking service request from Python while a Python subscriber receives
messages: what do ok / reply say, and does the request take effect?"""
import time, threading, json
import gzrun
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
T = gzrun.T
world = T / "exp3_setpose20.sdf"
name = gzrun.world_name(world)
node = gzrun.Node()
proc = gzrun.start(world, T / "exp3" / "server_gil2.log")
try:
    assert gzrun.wait_world(node, name)
    time.sleep(3)
    poses = {}
    node.subscribe(Pose_V, f"/world/{name}/pose/info", lambda m: poses.update({p.name: p.position.z for p in m.pose}))
    time.sleep(1)
    for z, timeout in ((40, 1000), (50, 3000)):
        p = Pose(); p.name = "cam"; p.position.z = z; p.orientation.w = 1
        t0 = time.perf_counter()
        ok, rep = node.request(f"/world/{name}/set_pose", p, Pose, Boolean, timeout)
        dt = 1000 * (time.perf_counter() - t0)
        time.sleep(0.5)
        print(f"timeout {timeout} ms: returned after {dt:.0f} ms, ok={ok}, reply={rep.data}, cam z now {poses.get('cam')}")
finally:
    gzrun.stop(proc)
