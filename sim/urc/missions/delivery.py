"""Delivery Mission (rule 1.c): a staged course of finding, picking up and
delivering objects for astronauts over terrain of increasing difficulty.

URC publishes the exact course script shortly before the competition
(1.c.i); this course uses every task type the rules list (1.c.iii). Layout
metres from the C2 station, x east, y north; the course runs north-east.

Stage 1 (line of sight, easy to moderate ground):
  D1 open the toolbox at TOOLBOX and deliver its wrench to astronaut A
  D2 astronaut B holds a sign naming what to bring (the water jug)
  D3 carry the supply crate up a ~11 deg hill to astronaut D
Stage 2 (rugged, out to ~900 m, later tasks behind a ridge out of radio
line of sight):
  D4 read a field sign giving a search area; find the instrument case in it
  D5 carry the first-aid kit past the boulder field, over the ridge pass and
     down the ledges to astronaut C behind the ridge
  D6 find the spectrometer lost in the wash in the radio shadow; bring it to C

The ground (1.c.ii: "soft sandy areas, gravel, rough stony areas, rock and
boulder fields, vertical drops and steep loosely consolidated slopes"): a
soft sand flat in stage 1, a gravel plain on the way out, bentonite clay on
the crate hill's flank, sand along the wash floor and a loose scree chute up
the steep mesa are friction zones (urc/terrains.py, urc/features.py); two
rough stony areas (rock gardens), the boulder field and the ledges.
"""
import math

from .. import features, geo, props, rules, terrain, terrains
from ..world import Layer, WorldBuilder

KEY = "delivery"
TITLE = "Delivery"
ORIGIN = geo.Origin(38.4040, -110.7935, 1374.3)  # altitude: USGS 3DEP at this point
SIZE, SAMPLES, CENTER = 1024.0, 1025, (430.0, 390.0)

C2 = (0.0, 0.0, 0.0)
GATE = (12.0, 8.0)
ROVER = (9.0, 5.0, math.radians(35))
DELIVERY_TOLERANCE = 2.0  # [m] judged here as "delivered" (the rules leave it to the judges)

TOOLBOX = (70.0, 40.0)
ASTRONAUT_A = (125.0, 12.0)
ASTRONAUT_B = (165.0, 95.0)
WATER_JUG = (108.0, 132.0)
SUPPLY_CRATE = (195.0, 55.0)
CRATE_HILL = (240.0, 125.0)  # astronaut D on top
FIELD_SIGN = (330.0, 290.0)
SEARCH_CENTER, SEARCH_RADIUS = (430.0, 405.0), 40.0
INSTRUMENT_CASE = (452.0, 381.0)
FIRST_AID = (468.0, 222.0)
BOULDER_FIELD = (505.0, 265.0, 38.0)
STEEP_MESA = (330.0, 175.0)  # 25-30 deg loose slopes, off the main route
RIDGE = [(360.0, 720.0), (520.0, 570.0), (640.0, 430.0), (770.0, 270.0), (900.0, 110.0)]
RIDGE_PASS = [(560.0, 420.0), (680.0, 540.0)]
PASS_YAW = math.atan2(RIDGE_PASS[1][1] - RIDGE_PASS[0][1], RIDGE_PASS[1][0] - RIDGE_PASS[0][0])
LEDGES = [features.Ledge(f"ledge_{k}", x, y, PASS_YAW, drop)  # down the far side of the pass
          for k, (x, y, drop) in enumerate(((700.0, 552.0, 0.3), (712.0, 564.0, 0.6), (724.0, 576.0, 1.0)))]
ASTRONAUT_C = (760.0, 610.0)
WASH = [(160.0, 560.0), (330.0, 470.0), (480.0, 360.0), (610.0, 400.0), (720.0, 420.0), (820.0, 330.0),
        (950.0, 320.0)]
SPECTROMETER = (845.0, 333.0)

LANDFORMS = [features.Mesa("crate_hill", *CRATE_HILL, 10.0, 6.0, 30.0, seed=44, irregularity=0.1),
             features.Mesa("steep_mesa", *STEEP_MESA, 18.0, 11.0, 20.0, seed=45)]
FEATURES = [
    features.Patch("sand_flat", terrains.SAND, 150.0, 62.0, 14.0),  # among astronauts A, B and the supply crate
    features.Patch("gravel_plain", terrains.GRAVEL, 285.0, 222.0, 18.0),  # on the way out to the field sign
    # On the crate hill's approach, ~15 deg: steeper than clay holds (14 deg).
    features.Patch("clay_flank", terrains.CLAY, 225.0, 102.0, 9.0),
    # Sand on the wash floor, but not where the wash cuts through the ridge (21 deg).
    features.Wash("wash", WASH, depth=3.5, sand_step=90.0, skip=(7,)),
    features.Slope("scree_chute", terrains.SCREE, (295.0, 175.0), (318.0, 175.0), 11.0),  # ~26 deg up the steep mesa
    *LEDGES,
]
STONY_GROUNDS = [(385.0, 330.0, 34.0, 20.0, 0.12),  # x, y, length, width, rock size: rough stony areas
                 (425.0, 250.0, 30.0, 14.0, 0.22)]  # on the way to the first-aid kit

LAYERS = [Layer("regolith"), Layer("pavement", start=3.0, fade=1.0), Layer("mudstone", start=8.0, fade=1.5),
          Layer("bentonite", start=14.0, fade=1.5)]


def make_terrain():
    hf = terrain.Heightfield(SIZE, SAMPLES, center=CENTER)
    # Ground gets rougher with distance from the start (1.c.ii).
    rough = terrain.smoothstep(80.0, 650.0, hf.radial(0.0, 0.0))
    hf.z += (1.0 + 4.0 * rough) * terrain.fbm(SAMPLES, SIZE, 180.0, seed=41, octaves=3)
    hf.z += (0.2 + 1.2 * rough) * terrain.fbm(SAMPLES, SIZE, 35.0, seed=42, octaves=3)
    hf.noise(0.05, 4.0, seed=43, octaves=2)
    hf.z -= hf.height(0, 0)
    features.shape(hf, LANDFORMS)
    hf.ridge(RIDGE, 18.0, 16.0, 30.0)
    hf.ramp(RIDGE_PASS, half_width=6.0, falloff=8.0)
    features.shape(hf, FEATURES)
    hf.flatten(*C2[:2], 30.0, 40.0, z=0.0)
    hf.flatten(*TOOLBOX, 4.0, 6.0)
    hf.flatten(*ASTRONAUT_A, 3.0, 4.0)
    hf.flatten(*ASTRONAUT_B, 3.0, 4.0)
    return hf


def sign_lines(w):
    s = w.geo(*SEARCH_CENTER)
    return ["FIND THE ORANGE CASE", f"N {s['lat']:.5f}  W {-s['lon']:.5f}", f"WITHIN {SEARCH_RADIUS:.0f} m"]


def build(models_dir, worlds_dir, media):
    hf = make_terrain()
    w = WorldBuilder(KEY, TITLE, "1.c", ORIGIN, hf, models_dir, worlds_dir, media, seed=17)
    w.sheet["time_limit_s"] = list(rules.DELIVERY_TIME)
    w.terrain(LAYERS)
    features.dress(w, FEATURES)  # first: objects placed on a zone rest on its tiles
    w.c2(*C2)
    w.place(props.start_gate(models_dir, media), "start_gate", *GATE, math.radians(35))
    w.point("start_gate", *GATE)
    w.rover(*ROVER)

    def astronaut(key, x, y, lines, face=None):
        sign = media.sign(f"astronaut_{key}", lines, size=(512, 512))
        model = props.astronaut(models_dir, media, f"urc_astronaut_{key}", sign_texture=sign, static=True,
                                arm_pose=props.POSE_SHOW_SIGN)
        face = math.atan2(-y, -x) if face is None else face  # towards C2 by default
        w.place(model, f"astronaut_{key}", x, y, face)
        return w.point(f"astronaut_{key}", x, y, rule="1.c.iii")

    def obj(name, model, x, y, yaw=0.0, dz=0.002, **info):
        w.place(model, name, x, y, yaw, dz=dz, record=info)

    # --- Stage 1 ---
    obj("toolbox", props.toolbox(models_dir), *TOOLBOX, 0.4, task="D1")
    tz = w.height(*TOOLBOX)
    w.include(props.wrench(models_dir), "wrench", (TOOLBOX[0], TOOLBOX[1], tz + 0.009), 0.4)
    w.sheet["objects"]["wrench"] = dict(w.geo(TOOLBOX[0], TOOLBOX[1], tz), model="urc_wrench", task="D1",
                                        note="inside the toolbox")
    astronaut("a", *ASTRONAUT_A, ["ASTRONAUT", "A"])
    astronaut("b", *ASTRONAUT_B, ["BRING:", "WATER JUG"])
    obj("water_jug", props.water_jug(models_dir), *WATER_JUG, task="D2")
    obj("supply_crate", props.supply_crate(models_dir, media), *SUPPLY_CRATE, 1.0, task="D3")
    astronaut("d", *CRATE_HILL, ["ASTRONAUT", "D"])

    # --- Stage 2 ---
    yaw_sign = math.atan2(-FIELD_SIGN[1], -FIELD_SIGN[0])  # readable on the way out
    w.place(props.field_sign(models_dir, media, "search", sign_lines(w)), "field_sign", *FIELD_SIGN, yaw_sign,
            record={"text": sign_lines(w), "task": "D4"})
    w.point("search_area", *SEARCH_CENTER, radius_m=SEARCH_RADIUS, rule="1.c.iii")
    obj("instrument_case", props.instrument_case(models_dir), *INSTRUMENT_CASE, 2.2, task="D4")
    obj("first_aid_kit", props.first_aid_kit(models_dir, media), *FIRST_AID, 0.3, task="D5")
    astronaut("c", *ASTRONAUT_C, ["ASTRONAUT", "C"], face=math.atan2(-1, -1))
    obj("spectrometer", props.spectrometer(models_dir), *SPECTROMETER, 1.2, task="D6")
    w.point("ridge_pass", *RIDGE_PASS[0], note="graded saddle through the ridge")

    deliveries = [
        ("D1", 1, "Open the toolbox (hinged lid) and deliver the wrench to astronaut A", "wrench", "astronaut_a"),
        ("D2", 1, "Read astronaut B's sign and bring what it asks for", "water_jug", "astronaut_b"),
        ("D3", 1, "Carry the supply crate up the hill to astronaut D", "supply_crate", "astronaut_d"),
        ("D4", 2, "Read the field sign; find the instrument case in the search area and bring it to astronaut C",
         "instrument_case", "astronaut_c"),
        ("D5", 2, "Deliver the first-aid kit to astronaut C behind the ridge", "first_aid_kit", "astronaut_c"),
        ("D6", 2, "Find the spectrometer lost in the wash (no radio) and bring it to astronaut C", "spectrometer",
         "astronaut_c")]
    for task_id, stage, title, item, target in deliveries:
        o = w.sheet["objects"][item]
        t = w.sheet["points"][target]
        w.task(id=task_id, stage=stage, rule="1.c.iii", title=title, object=item, deliver_to=target,
               tolerance_m=DELIVERY_TOLERANCE, distance_m=round(math.hypot(o["x"] - t["x"], o["y"] - t["y"]), 1),
               radio_los_at_pickup=w.radio_los(*w.to_layout(o["x"], o["y"])),
               radio_los_at_delivery=w.radio_los(*w.to_layout(t["x"], t["y"])))

    # --- Terrain clutter: sparse near the start, dense far out, off the objects ---
    keep, keep_pass = w.keep_clear(), w.keep_clear([RIDGE_PASS], step=5.0)
    w.rock_field("scattered", w.scatter(400, (150.0, 120.0), 200.0, (0.03, 0.2), avoid=keep, clearance=4.0)
                 + w.scatter(900, (520.0, 450.0), 380.0, (0.05, 0.45), avoid=keep_pass, clearance=4.0))
    w.rock_field("boulder_field", w.scatter(260, BOULDER_FIELD[:2], BOULDER_FIELD[2], (0.25, 1.1), avoid=keep,
                                            clearance=4.0))
    w.rock_field("search_area", w.scatter(120, SEARCH_CENTER, SEARCH_RADIUS + 10, (0.1, 0.5), avoid=keep,
                                          clearance=3.0))
    for k, (x, y, length, width, size) in enumerate(STONY_GROUNDS):
        w.rock_garden(f"stony_{k}", x, y, length, width, size, avoid=keep, clearance=3.0)
    w.shrubs(w.scatter_points(45, (400.0, 350.0), 450.0, avoid=keep_pass, clearance=6.0))
    w.sheet["judges_only"] = {"ridge": [w.geo(x, y) for x, y in RIDGE], "ridge_pass": [w.geo(x, y) for x, y in RIDGE_PASS],
                              "wash": [w.geo(x, y) for x, y in WASH], "steep_mesa": w.geo(*STEEP_MESA),
                              "boulder_field": dict(w.geo(*BOULDER_FIELD[:2]), radius_m=BOULDER_FIELD[2]),
                              "stony_ground": [dict(w.geo(x, y), length_m=length, width_m=width, rock_size_m=size)
                                               for x, y, length, width, size in STONY_GROUNDS]}
    return w.write()
