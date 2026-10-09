"""Does a Python subscriber callback stall Python's blocking service request?"""
import time
import gzrun
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.image_pb2 import Image
from gz.msgs10.clock_pb2 import Clock

T = gzrun.T
world = T / "exp3_setpose20.sdf"
name = gzrun.world_name(world)
node = gzrun.Node()
proc = gzrun.start(world, T / "exp3" / "server_gil.log")
def timed(label, n=5):
    p = Pose(); p.name = "cam"; p.position.z = 30; p.orientation.w = 1
    out = []
    for k in range(n):
        t0 = time.perf_counter()
        ok, rep = node.request(f"/world/{name}/set_pose", p, Pose, Boolean, 1000)
        out.append(round(1000 * (time.perf_counter() - t0), 1))
        time.sleep(0.05)
    print(label, out, flush=True)
try:
    assert gzrun.wait_world(node, name)
    time.sleep(3)
    timed("no subscriptions")
    clocks = []
    node.subscribe(Clock, f"/world/{name}/clock", lambda m: clocks.append(1))
    time.sleep(1)
    timed("with /clock subscriber (1 kHz?)")
    print("clock msgs/s", len(clocks) / 1.0, flush=True)
    n2 = gzrun.Node()
    imgs = []
    n2.subscribe(Image, "/cam/image", lambda m: imgs.append(1))
    time.sleep(2)
    timed("with /clock + image subscribers")
finally:
    gzrun.stop(proc)
