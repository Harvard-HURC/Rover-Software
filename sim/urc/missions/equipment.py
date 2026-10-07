"""Equipment Servicing Mission (rule 1.d).

Near MDRS on relatively flat ground. Layout metres from the C2 station, x
east, y north. The rover starts at the gate and drives 90 m east (rule:
up to 100 m) to the mock lander, whose front faces the gate. A sample stand
by the gate holds the sample tube (in a rack) and the cache container; the
fuel tank stands beside the lander with its hose on the ground; the tubular
key is in lock A. The rover works on a gravel apron in front of the lander
(a zone of gravel ground, urc/terrains.py); rocks and a rock garden lie off
the approach, which stays clear. The ground round it is the dry clay crust of
the shale pediment with patches of crusted sand sheet (PAINT), their MDRS
micro-relief at a low amplitude (4 m roughness about 2 cm, design 5.2: the
rule's "relatively flat" site) and their gravel and shrubs.
"""
import math

from .. import lander, landscape, props, rules, terrain, terrains
from ..world import WorldBuilder, add_relief, site

KEY = "equipment_servicing"
TITLE = "Equipment Servicing"
SITE = (38.4072, -110.7895)  # lat, lon of C2 (altitude: world.site)
SIZE, SAMPLES, CENTER = 256.0, 1025, (50.0, 0.0)
# Dantzig: with PGS the lander's 101 joints run below real time (gates G2, G5: 0.86x by CPU time), so per-wheel
# friction is approximate here (the user's choice, Q2).
SOLVER = "dantzig"

C2 = (0.0, 0.0, 0.0)
GATE = (9.0, 0.0)
ROVER = (6.5, 0.0, 0.0)
SAMPLE_STAND = (11.0, -3.5, math.pi / 2)  # x, y, yaw
LANDER = (99.0, 0.0, math.pi)  # front face at x = 99, facing the gate (west)
TANK_IN_LANDER = (1.0, 3.3, -math.pi / 2)  # hose runs along the lander's -y, in front of the face
GRAVEL_APRON = (92.5, 0.0, 12.0, 14.0)  # x, y, length (east), width: up to 0.5 m from the lander's face
ROCK_GARDEN = (50.0, 16.0, 20.0, 10.0, 0.18)  # x, y, length, width, rock size: beside the approach

PAINT = [landscape.Base("clay_crust"), landscape.Noise("sand_sheet", feature_m=60.0, cover=0.35)]
PADS = [(*C2[:2], 10.0), (*GATE, 6.0), (*SAMPLE_STAND[:2], 2.0), (*LANDER[:2], 9.0)]  # x, y, radius: kept flat
APPROACH_CLEAR = 7.0  # [m] no gravel this close to the drive from the gate to the lander (the apron's half-width)


def make_terrain():
    hf = terrain.Heightfield(SIZE, SAMPLES, center=CENTER)
    hf.noise(0.5, 70.0, seed=31, octaves=3)
    hf.z -= hf.height(0, 0)
    hf.flatten(*C2[:2], 10.0, 10.0, z=0.0)
    hf.flatten(*GATE, 6.0, 8.0)
    hf.flatten(*SAMPLE_STAND[:2], 2.0, 3.0)
    hf.flatten(*LANDER[:2], 9.0, 8.0)
    return add_relief(hf, PAINT, pads=PADS, seed=13)


def lander_point(xyz):
    """Lander-frame point -> layout (x, y, z above the lander's ground)."""
    x, y, z = lander.to_world((LANDER[0], LANDER[1], 0.0, LANDER[2]), xyz)
    return x, y, z


def build(models_dir, worlds_dir, media):
    hf = make_terrain()
    w = WorldBuilder(KEY, TITLE, "1.d", site(*SITE), hf, models_dir, worlds_dir, media, seed=13, solver=SOLVER)
    w.sheet["time_limit_s"] = rules.EQUIPMENT_TIME
    w.paint(PAINT)
    w.terrain()
    w.zone_rect("gravel_apron", terrains.GRAVEL, *GRAVEL_APRON)  # first: the fuel tank stands on its ground
    w.c2(*C2)
    w.place(props.start_gate(models_dir, media), "start_gate", *GATE)
    w.point("start_gate", *GATE, rule="1.d.i")
    w.rover(*ROVER)

    # Sample stand: tube in the rack, cache beside it.
    sx, sy, syaw = SAMPLE_STAND
    stand_z = w.place(props.sample_stand(models_dir), "sample_stand", sx, sy, syaw)
    c, s = math.cos(syaw), math.sin(syaw)
    for name, model, (dx, dy, dz), extra in (
            ("sample_tube", props.sample_tube(models_dir), props.SAMPLE_STAND_TUBE, {}),
            ("cache_container", props.cache_container(models_dir), props.SAMPLE_STAND_CACHE,
             {"handle_length_m": props.CACHE["handle_length"], "handle_diameter_m": props.CACHE["handle_diameter"],
              "mass_kg": props.CACHE["mass"]})):
        x, y = sx + c * dx - s * dy, sy + s * dx + c * dy
        w.include(model, name, (x, y, stand_z + dz + 0.001), syaw)
        w.sheet["objects"][name] = dict(w.geo(x, y, stand_z + dz), model=model, rule="1.d.ii", **extra)

    # The lander, the key in lock A, and the fuel tank.
    lx, ly, lyaw = LANDER
    ground = w.height(lx, ly)
    w.include(lander.lander(models_dir, media), "lander", (lx, ly, ground), lyaw)
    w.sheet["objects"]["lander"] = dict(w.geo(lx, ly, ground), model=lander.NAME, yaw=lyaw, rule="1.d")
    kx, ky, kz = lander_point(lander.KEY_START[:3])
    w.include(lander.tubular_key(models_dir, media), "key", (kx, ky, ground + kz), lyaw)
    w.sheet["objects"]["key"] = dict(w.geo(kx, ky, ground + kz), model=lander.KEY_NAME, yaw=lyaw, rule="1.d.ii")
    tx, ty, _ = lander_point((TANK_IN_LANDER[0], TANK_IN_LANDER[1], 0.0))
    w.place(props.fuel_tank(models_dir), "fuel_tank", tx, ty, lyaw + TANK_IN_LANDER[2], record={"rule": "1.d.ii"})
    def at_lander(p):
        x, y, z = lander_point(p)
        return w.geo(x, y, ground + z)

    w.sheet["lander"] = {
        "model": "lander", "pose": dict(w.geo(lx, ly, ground), yaw=lyaw),
        "lock_a": at_lander(lander.lock_point("a")), "lock_b": at_lander(lander.lock_point("b")),
        "inlet_tip": at_lander(lander.INLET_TIP), "drawer_well_closed": at_lander(lander.DRAWER_WELL_CLOSED),
        "state_topic": "/model/lander/state", "presses_topic": "/model/lander/presses",
        "keyboard_tag_ids": rules.KEYBOARD_TAG_IDS, "key_tag_ids": list(rules.KEY_TAG_IDS)}
    w.point("lander", lx, ly, rule="1.d.i", distance_from_gate_m=round(math.hypot(lx - GATE[0], ly - GATE[1]), 1))

    for task in (
            dict(id="cache_sample", title="Put the sample tube in the cache, close the lid and turn the lock",
                 objects=["sample_tube", "cache_container"]),
            dict(id="deliver_cache", title="Carry the cache container to the lander", objects=["cache_container"]),
            dict(id="drawer", title="Open the drawer, put the cache in its well, close the drawer",
                 joints=["drawer"]),
            dict(id="panel", title="Undo the latch and open the hinged panel", joints=["latch", "door"]),
            dict(id="typing", title="Autonomously type the 3-6 letter launch key",
                 note="the referee draws the key; presses on /model/lander/presses"),
            dict(id="key", title="Move the key from lock A to lock B and turn it", joints=["lock_b"],
                 objects=["key"]),
            dict(id="hose", title="Push the fuel hose coupler onto the lander inlet", objects=["fuel_tank"]),
            dict(id="valve", title="Turn the quarter-turn valve", joints=["valve"]),
            dict(id="controls", title="Push buttons, flip switches, turn knobs",
                 joints=[f"button_{n}" for n in range(len(lander.BUTTONS))]
                 + [f"switch_{n}" for n in range(len(lander.SWITCHES))]
                 + [f"knob_{n}" for n in range(len(lander.KNOBS))])):
        w.task(rule="1.d.ii", **task)

    keep_clear = w.keep_clear([[GATE, (lx - 3, ly)]], step=5.0)  # and the approach
    w.rock_field("scattered", w.scatter(260, CENTER, 120.0, (0.03, 0.25), avoid=keep_clear, clearance=4.0))
    w.rock_field("boulders", w.scatter(40, CENTER, 120.0, (0.3, 0.8), avoid=keep_clear, clearance=12.0))
    w.rock_garden("rock_garden", *ROCK_GARDEN, avoid=keep_clear, clearance=3.0)
    approach = w.near(paths=[([GATE, (lx, ly)], APPROACH_CLEAR)])
    w.clutter(avoid=keep_clear, clearance=4.0, rock_sizes=(0.15, 0.3), rocks_within=~approach)
    w.pebbles([(*ROVER[:2], 35.0), (sx, sy, 6.0), (lx - 6.0, ly, 6.0)])
    return w.write()
