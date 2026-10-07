"""Autonomy Mission (rule 1.e): Astronaut Assistance + Autonomous Route-Finding,
on the real terrain of the Utah state-owned square mile (1.e.xiii).

Terrain: the USGS 3DEP 1 m DEM (sim/data/dem, fetched by sim/tools/fetch_dem.py),
2048 m square around the square mile's centre. Layout metres x east, y north
from the C2 station, which stands on the flat central rise 200 m north-east of
that centre; z is the elevation above C2's. Altitudes (the origin, the sheet,
NavSat) are WGS84 ellipsoidal heights: the DEM's NAVD88 elevations plus
dem.NAVD88_TO_WGS84.

- Route-Finding, north-west: START_POST (ArUco 0) on flat ground 412 m from
  C2, in radio line of sight (1.e.xii). POST1 (ArUco 1) on the crest of a
  narrow butte, 15 m above the start. The crest is gentle (< 8 deg) for its
  last 85 m, but its west end is reached over an 8 m rim of 18-24 deg
  ground, which the easy route switchbacks up at 16 deg. Driving straight at
  Post 1 the grade is 26-44 deg from 22 of 24 compass bearings, steeper than
  a rover climbs loose slopes; but the bare ground here grips like the
  heightmap it is (terrains.HEIGHTMAP_MU), and the rover climbs it up to
  about 45 deg (measured: straight up a 40 deg face to Post 1). So boulders
  of the broken caprock line the butte's rim (routes.rim: round the ground
  reached from Post 1 on slopes up to RIM_SLOPE, kept clear of the easy
  route along the crest), too tall to climb (0.3 m rocks stop the rover on
  the proving ground) and too close together to pass between, everywhere
  but where the easy route climbs onto the butte: that climb is the only way
  up (1.e.xv: not every approach is navigable). POST2 (ArUco 2) on the plain
  71 m north of Post 1, out of the C2 antenna's line of sight behind the
  butte (1.e.xvi). judges_only holds the easy route, the butte's rim and its
  approach grades.
- Astronaut Assistance, south-east of C2 on gentle ground (< 6 deg), all in
  radio line of sight: the astronaut waits at ASTRONAUT_WAIT (1.e.v); Follow!
  along FOLLOW_PATH (1.e.vi); the rock pick hammer lies at HAMMER near its end;
  Stay! the astronaut walks on to STAY_TO, more than 20 m away (1.e.vii);
  Fetch! (1.e.viii); Come! (1.e.ix); Give! (1.e.x).

The ground follows the DEM (urc/terrains.py, urc/features.py): soft sand on
the floors of the washes that drain past the butte (the north one runs
between the start, Post 1 and Post 2), loose scree on the butte's north face
where the straight line from the start crosses it, gravel on the aprons at its
feet, bentonite clay on the plain the rover crosses from C2 and bare
slickrock on its caprock crest. Rocks lie where the rover drives: a stony
plain, two rock gardens, rubble, talus on the butte's faces, the boulders on
its rim, and boulders below its cliff and the knoll by the astronaut. Rocks
keep ROUTE_CLEARANCE from the easy route, the astronaut's walk and every
target, the friction zones further still; slickrock, which grips like the
bare ground, lies on the crest the easy route follows.
"""
import json
import math
from pathlib import Path

import numpy as np

from .. import dem, features, geo, props, routes, rules, terrain, terrains
from ..world import Layer, WorldBuilder

KEY = "autonomy"
TITLE = "Autonomy"
DEM_PATH = Path(__file__).resolve().parents[2] / "data" / "dem" / "route_area_3dep.tif"
SQUARE_MILE_CENTER = tuple((a + b) / 2 for a, b in zip(*rules.ROUTE_AREA))  # (lat, lon)
C2_FROM_CENTER = (160.0, 120.0)  # [m] east, north of the square mile's centre
SIZE, SAMPLES = 2048.0, 2049  # 1 m samples
CENTER = (-C2_FROM_CENTER[0], -C2_FROM_CENTER[1])  # the terrain is centred on the square mile

C2 = (0.0, 0.0, 0.0)  # x, y, yaw
ROVER = (6.0, 6.0, math.pi / 2)

ASTRONAUT_WAIT = (38.0, -82.0)  # 90 m from C2
FOLLOW_PATH = [ASTRONAUT_WAIT, (46.0, -103.0), (52.0, -126.0)]  # 46 m
HAMMER = (55.0, -129.0)
STAY_TO = (60.0, -157.0)  # 32 m beyond the end of the walk

START_POST = (-160.0, 380.0)
POST1 = (30.0, 400.0)  # on the crest 9 m west of the butte's top, where it is flattest
POST2 = (40.0, 470.0)
LANDING_PAD = (-150.0, 392.0)  # 16 m from the start post
EASY_ROUTE_MAX_SLOPE = 16.0  # [deg] the gentlest limit with a way onto the butte
APPROACH_BEARINGS = range(0, 360, 15)  # [deg] compass
APPROACH_RADII = (6.0, 80.0)  # [m] from Post 1
ROUTE_CLEARANCE = 6.0  # [m] rock centres from the easy route, the astronaut's walk and every placed model
# The butte's rim (routes.rim) and its boulders: RIM_SLOPE is the steepest
# ground of the hilltop (its 18-24 deg west rim is where the easy route climbs
# on); along the crest the rim keeps RIM_WIDTH from the easy route, so its
# boulders keep ROUTE_CLEARANCE (the outline runs within a cell of the
# width). One boulder every RIM_STEP, within RIM_JITTER of the rim, long
# half-axis RIM_SIZES: at least 0.78 m across (meshes.rock_variant: the short
# horizontal half-axis is 0.65-0.95 of the long one), so no gap between two
# reaches 0.55 m, far less than the rover's 0.9 m width.
RIM_SLOPE = 20.0  # [deg]
RIM_WIDTH = ROUTE_CLEARANCE + 1.5  # [m]
RIM_STEP = 0.8  # [m]
RIM_JITTER = 0.25  # [m]
RIM_SIZES = (0.6, 1.0)  # [m]

# Washes: thalwegs traced from the DEM's flow lines (D8 flow accumulation), sand every WASH_STEP.
NORTH_WASH = [(-90.0, 462.0), (-50.0, 474.0), (0.0, 440.0), (40.0, 448.0), (100.0, 450.0), (130.0, 384.0),
              (180.0, 344.0)]  # between the start, Post 1 and Post 2, then south-east past the butte
CLIFF_WASH = [(0.0, 342.0), (40.0, 350.0), (80.0, 352.0), (120.0, 366.0)]  # 15-20 m out from the foot of the cliff
WASH_STEP = 45.0  # [m]
SLABS = [(-85.0, 342.0, 12.0), (6.0, 397.0, 4.0), (44.0, 402.0, 3.5)]  # bare caprock on the butte (x, y, radius)

FEATURES = [
    features.Wash("wash", NORTH_WASH, sand_step=WASH_STEP, sand_radius=7.0),  # in the DEM already: no channel
    features.Wash("cliff_wash", CLIFF_WASH, sand_step=WASH_STEP, sand_radius=6.0),
    # The north face where the straight line from the start crosses it: 32 deg, far steeper than scree holds.
    features.Patch("north_face_scree", terrains.SCREE, -52.0, 392.0, 6.0, falloff=3.0),
    features.Patch("gravel_apron_north", terrains.GRAVEL, -125.0, 397.0, 8.0),  # by the start, off the line to Post 1
    features.Patch("gravel_apron_south", terrains.GRAVEL, -30.0, 315.0, 8.0),  # below the cliff
    features.Patch("clay_flat", terrains.CLAY, -120.0, 250.0, 12.0),  # beside the drive from C2 to the start
    *[features.Patch(f"caprock_slab_{k}", terrains.SLICKROCK, x, y, r, level=False)
      for k, (x, y, r) in enumerate(SLABS)],
]

# Rock scatter (x, y, radius): where the rover drives in this mission. Rocks
# outside these discs would cost visuals and add nothing to drive over.
ROUTE_FIELD = (-60.0, 220.0, 330.0)  # from C2 out past the butte
ASTRONAUT_FIELD = (50.0, -115.0, 110.0)
RUBBLE = (-80.0, 190.0, 20.0)  # on the direct line from C2 to the start post
BUTTE = (-12.0, 360.0, 130.0)  # talus: kept only on its faces
# Rock gardens (x, y, length, width, rock size, yaw): 0.2 m rocks on the way to
# Post 2, which the rover crosses; 0.3 m rocks beside the drive from C2, which
# stop it (measured on the proving ground), so it must go round.
ROCK_GARDENS = [(-105.0, 428.0, 28.0, 12.0, 0.2, 0.6), (-30.0, 215.0, 20.0, 10.0, 0.3, 1.2)]
BOULDER_APRONS = [(-85.0, 298.0, 10.0), (-42.0, 318.0, 10.0), (-28.0, 343.0, 9.0), (-10.0, 356.0, 9.0),
                  (15.0, 362.0, 9.0), (45.0, 366.0, 9.0), (75.0, 374.0, 8.0),  # the foot of the butte's cliff
                  (84.0, -142.0, 10.0)]  # x, y, radius; the last below the knoll south-east of the astronaut

# Elevation bands above C2's (the terrain spans 56 m below it to 30 m above;
# the route-finding plain lies 6-10 m below, the butte's crest 8 m above):
# packed regolith in the low ground, maroon mudstone, grey bentonite,
# sandstone caprock on the heights.
LAYERS = [Layer("regolith"), Layer("mudstone", start=-30.0, fade=3.0), Layer("bentonite", start=-14.0, fade=3.0),
          Layer("caprock", start=3.0, fade=1.5)]


def make_site():
    """The C2 station's WGS84 origin (ellipsoidal altitude from the DEM) and
    the DEM resampled into layout coordinates (before the features)."""
    d = dem.read_geotiff(DEM_PATH)
    center = geo.Origin(*SQUARE_MILE_CENTER, d.height(*SQUARE_MILE_CENTER))
    lat, lon, _ = geo.enu_to_wgs84(center, *C2_FROM_CENTER)
    origin = geo.Origin(lat, lon, d.height(lat, lon))  # NAVD88, as the DEM
    hf = dem.to_heightfield(d, origin, SIZE, SAMPLES, CENTER)
    return geo.Origin(lat, lon, origin.alt + dem.NAVD88_TO_WGS84), hf


def make_terrain():
    """The world's terrain: the DEM with the features' levelled zones, and how
    far [m] that moved it from the DEM at most."""
    origin, hf = make_site()
    measured = hf.z.copy()
    features.shape(hf, FEATURES)
    return origin, hf, float(np.abs(hf.z - measured).max())


def build(models_dir, worlds_dir, media):
    origin, hf, edit = make_terrain()
    w = WorldBuilder(KEY, TITLE, "1.e", origin, hf, models_dir, worlds_dir, media, seed=7)
    w.sheet["time_limit_s"] = rules.AUTONOMY_TIME
    w.terrain(LAYERS)
    features.dress(w, FEATURES)  # first: objects placed on a zone rest on its tiles
    w.c2(*C2)
    w.rover(*ROVER)
    w.place(props.landing_pad(models_dir, media), "landing_pad", *LANDING_PAD)
    w.point("landing_pad", *LANDING_PAD, rule="1.e.xiv")

    # --- Astronaut Assistance (50 points) ---
    astronaut = props.astronaut(models_dir, media)
    yaw_to_c2 = math.atan2(-ASTRONAUT_WAIT[1], -ASTRONAUT_WAIT[0])
    w.place(astronaut, "astronaut", *ASTRONAUT_WAIT, yaw_to_c2, record={"rule": "1.e.iv"})
    w.place(props.rock_pick_hammer(models_dir), "rock_pick_hammer", *HAMMER, 0.7, dz=0.002,
            record={"rule": "1.e.viii"})
    w.point("astronaut_wait", *ASTRONAUT_WAIT, rule="1.e.v", given_to_teams=True)
    w.point("follow_end", *FOLLOW_PATH[-1], rule="1.e.vi", given_to_teams="during set-up")
    w.point("stay_to", *STAY_TO, rule="1.e.vii")
    w.sheet["astronaut"] = {"model": "astronaut", "walk_speed_mps": 1.1,
                            "follow_path": [w.geo(x, y) for x, y in FOLLOW_PATH],
                            "stay_to": w.geo(*STAY_TO), "commands": list(rules.ASTRONAUT_COMMANDS)}
    for task in (
            dict(id="suit", rule="1.e.iv", points=5, title="EVA suit with camera and microphone", judged="not simulated"),
            dict(id="goto_astronaut", rule="1.e.v", points=5, title="Drive autonomously to the astronaut's GNSS location",
                 target="astronaut_wait", tolerance_m=rules.ASTRONAUT_TOLERANCE),
            dict(id="follow", rule="1.e.vi", points=15, title="Follow! the astronaut and stop within 3 m when they stop",
                 target="follow_end", tolerance_m=rules.ASTRONAUT_TOLERANCE,
                 command_points={"device": 5, "sign": 5, "speech": 10, "gesture": 15}),
            dict(id="stay", rule="1.e.vii", points=5, title="Stay! while the astronaut walks more than 20 m away",
                 target="stay_to", min_distance_m=rules.STAY_DISTANCE),
            dict(id="fetch", rule="1.e.viii", points=10, title="Fetch! pick up the rock pick hammer autonomously",
                 target="rock_pick_hammer"),
            dict(id="come", rule="1.e.ix", points=5, title="Come! drive to the astronaut and stop within 3 m",
                 target="stay_to", tolerance_m=rules.ASTRONAUT_TOLERANCE),
            dict(id="give", rule="1.e.x", points=5, title="Give! place the hammer on the ground", target="rock_pick_hammer")):
        w.task(subtask="astronaut_assistance", **task)

    # --- Route-Finding (50 points) ---
    for key, (x, y), tag, rule in (("route_start", START_POST, rules.AR_START_ID, "1.e.xii"),
                                   ("post1", POST1, rules.AR_POST1_ID, "1.e.xv"),
                                   ("post2", POST2, rules.AR_POST2_ID, "1.e.xvi")):
        yaw = float(w.rng.uniform(0, 2 * math.pi / 3))
        w.place(props.ar_post(models_dir, media, tag), key, x, y, yaw, record={"aruco_id": tag})
        w.point(key, x, y, rule=rule, aruco_id=tag, radio_los=w.radio_los(x, y), given_to_teams=True)
    for key, rule in (("post1", "1.e.xv"), ("post2", "1.e.xvi")):
        w.task(subtask="route_finding", id=key, rule=rule, points=rules.ROUTE_POINTS_PER_TARGET,
               title=f"Reach {key} autonomously, stop within 1 m and signal arrival (LED flashing green)",
               target=key, tolerance_m=rules.ROUTE_TOLERANCE, led=rules.LED_ARRIVED)
    easy_route, grade = routes.easy_route(hf, START_POST, POST1, EASY_ROUTE_MAX_SLOPE)
    approach = routes.approach_grades(hf, POST1, APPROACH_BEARINGS, *APPROACH_RADII)
    rim, entry = routes.rim(hf, POST1, RIM_SLOPE, easy_route, RIM_WIDTH)
    w.sheet["judges_only"] = {
        "easy_route": {"from": "route_start", "to": "post1", "max_grade_deg": round(grade, 1),
                       "grade_base_m": routes.GRADE_BASE, "search_max_slope_deg": EASY_ROUTE_MAX_SLOPE,
                       "clearance_m": ROUTE_CLEARANCE, "points": [w.geo(x, y) for x, y in easy_route]},
        "rim": {"note": "boulders line the butte's rim, except within clearance_m of the easy route where it "
                        "climbs onto the butte (entry): the only way up",
                "max_slope_deg": RIM_SLOPE, "boulder_step_m": RIM_STEP, "entry": w.geo(*entry),
                "points": [w.geo(x, y) for x, y in rim]},
        "approach_grades": {"around": "post1", "radii_m": list(APPROACH_RADII), "grade_base_m": routes.GRADE_BASE,
                            "note": "steepest grade driving straight at Post 1 from each compass bearing; the bare "
                                    "ground grips to about 45 deg, so the rim's boulders, not these grades, stop "
                                    "the rover",
                            "deg_by_bearing": {str(b): round(g, 1) for b, g in zip(APPROACH_BEARINGS, approach)}},
        "washes": {"north": [w.geo(x, y) for x, y in NORTH_WASH], "cliff": [w.geo(x, y) for x, y in CLIFF_WASH]},
        "terrain_source": dict(json.loads(DEM_PATH.with_suffix(".json").read_text()),
                               edits=f"levelled to a plane under each friction zone, at most {edit:.2f} m off "
                                     "the DEM; the DEM everywhere else",
                               altitudes=f"WGS84 ellipsoidal: the DEM's NAVD88 elevations {dem.NAVD88_TO_WGS84:+.2f} m "
                                         "(GEOID18 and NAD83(2011) to WGS84(G2139) at epoch 2027.4, NOAA VDatum); "
                                         "the world's dem.tif holds them too")}

    # --- Ground clutter: rocks where the rover drives, but not on the targets and routes ---
    clear = dict(avoid=w.keep_clear([easy_route, FOLLOW_PATH + [STAY_TO]]), clearance=ROUTE_CLEARANCE)
    w.rock_field("stony_plain", w.scatter(900, ROUTE_FIELD[:2], ROUTE_FIELD[2], (0.04, 0.4), **clear)
                 + w.scatter(200, ASTRONAUT_FIELD[:2], ASTRONAUT_FIELD[2], (0.04, 0.3), **clear))
    w.rock_field("rubble", w.scatter(110, RUBBLE[:2], RUBBLE[2], (0.1, 0.45), **clear))
    w.rock_field("talus", w.scatter(250, BUTTE[:2], BUTTE[2], (0.15, 0.7), min_slope=15.0, **clear))
    w.rock_field("boulders", w.scatter_each(BOULDER_APRONS, 14, (0.4, 1.2), **clear))
    for k, garden in enumerate(ROCK_GARDENS):
        w.rock_garden(f"rock_garden_{k}", *garden, **clear)
    rim_spots = [(x, y, RIM_JITTER) for x, y in terrain.resample(rim, RIM_STEP)]
    w.rock_field("rim_boulders", w.scatter_each(rim_spots, 1, RIM_SIZES, **clear))
    w.shrubs(w.scatter_points(40, ROUTE_FIELD[:2], ROUTE_FIELD[2], **clear))
    return w.write()
