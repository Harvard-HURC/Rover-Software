"""G2: PGS in Equipment Servicing, Delivery and the proving ground (one TestFixture run per process).

python g2.py keypress <solver>                 the lander key-press test of test_urc_sim
python g2.py settle <world> <solver> <seconds> every model's links and joints, start vs end
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

import gz.math7  # noqa: E402,F401
from gz.sim8 import Joint, Link, Model, TestFixture, World, world_entity  # noqa: E402
from gz.transport13 import Node  # noqa: E402
from urc import judge as J  # noqa: E402

mode, rest = sys.argv[1], sys.argv[2:]


def copy(world, solver):
    return G.world_variant(G.REPO / "sim" / "worlds" / f"{world}.sdf", f"g2_{world}_{solver}",
                           solver=None if solver == "dantzig" else solver, rtf=0)


if mode == "keypress":
    solver = rest[0]
    from gz.msgs10.stringmsg_pb2 import StringMsg
    presses = []
    node = Node()
    node.subscribe(StringMsg, "/model/lander/presses", lambda m: presses.append(m.data))
    plan = sorted([(0.5, "key_s"), (0.8, "key_x"), (1.1, "key_backspace"), (1.4, "key_o"), (1.7, "key_l")])
    joints = {}

    def pre(info, ecm):
        if not joints:
            model = Model(World(world_entity(ecm)).model_by_name(ecm, "lander"))
            for _, name in plan:
                joints[name] = Joint(model.joint_by_name(ecm, name))
        t = info.iterations / 1000
        for start, name in plan:
            if start <= t < start + 0.1:
                joints[name].set_force(ecm, [2.0])

    fixture = TestFixture(str(copy("urc_equipment_servicing", solver)))
    fixture.on_pre_update(pre)
    fixture.finalize()
    fixture.server().run(True, 2200, False)
    time.sleep(0.3)
    text = ""
    for name in presses:
        text = J.type_text(text, name)
    out = dict(solver=solver, presses=presses, expected=[n for _, n in plan], text=text,
               passes=presses == [n for _, n in plan] and text == "sol")
    print("RESULT", json.dumps(out))
    G.save(f"g2_keypress_{solver}", out)

elif mode == "settle":
    world, solver, seconds = rest[0], rest[1], float(rest[2])
    snap = {}
    marks = (500, round(seconds * 1000))

    def post(info, ecm):
        if info.iterations not in marks:
            return
        w = World(world_entity(ecm))
        state = {}
        for entity in w.models(ecm):
            model = Model(entity)
            if model.static(ecm):
                continue
            name = model.name(ecm)
            links = {}
            for le in model.links(ecm):
                link = Link(le)
                p = link.world_pose(ecm)
                links[link.name(ecm)] = (p.pos().x(), p.pos().y(), p.pos().z(), p.rot().w(), p.rot().x(),
                                         p.rot().y(), p.rot().z())
            joints = {}
            for je in model.joints(ecm):
                j = Joint(je)
                q = j.position(ecm)
                if q:
                    joints[j.name(ecm)] = q[0]
            state[name] = dict(links=links, joints=joints)
        snap[info.iterations] = state

    fixture = TestFixture(str(copy(world, solver)))
    fixture.on_post_update(post)
    fixture.finalize()
    start = time.time()

    def enable(info, ecm):
        if info.iterations == 1:
            w = World(world_entity(ecm))
            for entity in w.models(ecm):
                for je in Model(entity).joints(ecm):
                    Joint(je).enable_position_check(ecm, True)
    fixture.on_pre_update(enable)
    fixture.server().run(True, marks[1], False)
    wall = time.time() - start
    a, b = snap[marks[0]], snap[marks[1]]
    moves = {}
    for name in a:
        dmax = 0.0
        amax = 0.0
        for link, pa in a[name]["links"].items():
            pb = b[name]["links"][link]
            dmax = max(dmax, math.dist(pa[:3], pb[:3]))
            dot = abs(sum(x * y for x, y in zip(pa[3:], pb[3:])))
            amax = max(amax, 2 * math.degrees(math.acos(min(1.0, dot))))
        jmax = max([abs(b[name]["joints"][k] - v) for k, v in a[name]["joints"].items()] or [0.0])
        moves[name] = dict(max_link_move_m=round(dmax, 5), max_link_turn_deg=round(amax, 3),
                           max_joint_change=round(jmax, 5), joints=len(a[name]["joints"]))
    out = dict(world=world, solver=solver, seconds=seconds, wall_s=round(wall, 1), models=moves)
    print("RESULT", json.dumps(out))
    G.save(f"g2_settle_{world}_{solver}", out)
