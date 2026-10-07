"""Rover proving ground: a test course for the rocker suspension and traction,
not a URC mission (no tasks, no referee).

Flat ground near MDRS. Layout metres from the course entrance, x east, y
north; the rover starts at the entrance facing north up the central road.
Every station has a field sign by the road and a sheet point saying what it
tests. West of the road, strips run west:

- friction lanes, mu 0.20 / 0.35 / 0.50 / 0.70 / 0.95 from south to north,
  side by side up one hill: 8 m flat, a 10 deg ramp, a landing, a 20 deg
  ramp, a landing, a 30 deg ramp to a plateau, which drops back to the plain
  on the far side at 12 deg. A rover holds or climbs a grade up to atan(mu)
  (11, 19, 27, 35, 44 deg);
- sand, clay and slickrock: each a flat stretch, then a slope up and down
  (the strong dig-in, terrains.DIG: the rover stalls on the 15 deg sand dune
  and 2.7 m up the 12 deg clay slope as its slipping wheels dig in, measured;
  it crosses the slickrock's 25 deg);
- side slopes of 10 and 20 deg across the direction of travel;
- three strips of natural ground, from the same recipes as the mission
  worlds' (terrains.py): crusted sand sheet with its shrubs, a 12 deg badland
  slope with haystack knobs and rills, and a block field of tabular slabs,
  each with its MDRS micro-relief (the rest of the course stays as built).
East of the road, stations run east:
- rock gardens of 0.1 / 0.2 / 0.3 / 0.4 m rocks (the wheel radius is 0.15 m);
- an articulation track: humps of 10 / 20 / 30 cm under one side, then the
  other, and a diagonal ditch that drops a front wheel and the opposite rear
  wheel together, twisting the rockers against each other to their limit
  (measured: 0.496 of 0.5 rad, and the rover drives on across);
- a washboard;
- up-steps and drop-offs of 0.1 / 0.2 / 0.3 m;
- a boulder field.
"""
import math

import gen_model  # the rover's geometry; sim/ is on sys.path wherever urc is used

from .. import features, landscape, props, terrain, terrains
from ..features import Lane, Natural, Step, Surface
from ..world import WorldBuilder, add_relief, garden_spacing, site

KEY = "proving_ground"
TITLE = "Rover proving ground"
SITE = (38.4080, -110.7850)  # lat, lon of the entrance, east of the Equipment Servicing site (altitude: world.site)
SIZE, SAMPLES, CENTER = 256.0, 2049, (-30.0, 62.0)  # 0.125 m samples: humps and corrugations need them
SOLVER = "pgs"  # each wheel grips mu times its own load; physics fast enough (gates G2, G5: 4.8x real time)

ROVER = (0.0, 0.0, math.pi / 2)
GATE = (0.0, 4.0)
ROVER_PARAMS = gen_model.Params()
ROVER_TRACK = 2 * ROVER_PARAMS.pivot_y  # [m] between the wheel planes
ROVER_WHEELBASE = 2 * ROVER_PARAMS.wheel_dx  # [m]
WEST, EAST = math.pi, 0.0

FRICTION_MU = (0.2, 0.35, 0.5, 0.7, 0.95)
FRICTION_LANES = (16.0, 23.0, 30.0, 37.0, 44.0)  # y of each lane's axis, 5 m lanes with 2 m between
FRICTION_PROFILE = ((8.0, 0.0), (8.0, 10.0), (4.0, 0.0), (7.0, 20.0), (4.0, 0.0), (5.0, 30.0))
FRICTION_RISE = sum(length * math.tan(math.radians(grade)) for length, grade in FRICTION_PROFILE)
FRICTION_HILL = Lane("friction_hill", (-8.0, 30.0), WEST, 35.0, FRICTION_PROFILE + (
    (10.0, 0.0), (FRICTION_RISE / math.tan(math.radians(12.0)), -12.0)), falloff=4.0, surfaces=tuple(
    Surface(f"lane_mu{round(mu * 100):03d}", terrains.calibration_surface(mu), 5.0, 30.0 - y,
            sum(length for length, _ in FRICTION_PROFILE)) for mu, y in zip(FRICTION_MU, FRICTION_LANES)))
SAND_PIT = Lane("sand_pit", (-8.0, 62.0), WEST, 5.0, ((10.0, 0.0), (6.0, 15.0), (4.0, 0.0), (6.0, -15.0)),
                (Surface("sand_pit", terrains.SAND, 5.0),))
CLAY_PATCH = Lane("clay_patch", (-8.0, 72.0), WEST, 5.0, ((10.0, 0.0), (6.0, 12.0), (4.0, 0.0), (6.0, -12.0)),
                  (Surface("clay_patch", terrains.CLAY, 5.0),))
SLICKROCK_SLAB = Lane("slickrock_slab", (-8.0, 82.0), WEST, 5.0, ((6.0, 0.0), (6.0, 25.0), (4.0, 0.0), (6.0, -25.0)),
                      (Surface("slickrock_slab", terrains.SLICKROCK, 5.0),))
SIDE_SLOPES = [Lane(f"side_slope_{grade}", (-8.0, y), WEST, 6.0, ((20.0, 0.0),), cross=grade)
               for grade, y in ((10, 96.0), (20, 108.0))]
BUMPS = features.AlternatingBumps("bumps", (8.0, 62.0), EAST, (0.1, 0.2, 0.3), 3.0, ROVER_TRACK / 2)
# Deep enough for a wheel in it to turn its rocker to the limit.
TWIST_DITCH = features.TwistDitch("twist_ditch", 32.0, 62.0, EAST, math.atan2(ROVER_TRACK, ROVER_WHEELBASE),
                                  round(ROVER_WHEELBASE * math.tan(ROVER_PARAMS.rocker_limit), 2))
WASHBOARD = features.Washboard("washboard", (8.0, 74.0), EAST, 20.0, 4.0)
STEPS = [Step(f"step_{round(top * 100)}cm", 16.0, y, EAST, 4.0, 3.0, top) for top, y in ((0.1, 84.0), (0.2, 90.0),
                                                                                          (0.3, 96.0))]
NATURAL = [Natural("natural_sand_sheet", terrains.SAND_SHEET, (-8.0, 132.0), WEST, 40.0, 12.0),
           Natural("natural_badland", terrains.BADLAND_SLOPE, (-8.0, 150.0), WEST, 40.0, 16.0, grade=12.0),
           Natural("natural_block_field", terrains.BLOCK_FIELD, (-8.0, 170.0), WEST, 40.0, 14.0)]
NATURAL_EASE = 2.0  # [m] (A: the strips are 12-16 m wide; the worlds' 5 m would leave them little full relief)
FEATURES = [FRICTION_HILL, SAND_PIT, CLAY_PATCH, SLICKROCK_SLAB, *SIDE_SLOPES, BUMPS, TWIST_DITCH, WASHBOARD, *STEPS,
            *NATURAL]

GARDENS = [(f"garden_{round(size * 100)}cm", 18.0, y, size) for size, y in ((0.1, 16.0), (0.2, 26.0), (0.3, 36.0),
                                                                              (0.4, 46.0))]
GARDEN_SIZE = (12.0, 6.0)  # [m] length (east), width
BOULDER_FIELD = (28.0, 118.0, 14.0)  # x, y, radius
BOULDERS = (70, (0.3, 1.0))  # count, size range [m]



def make_terrain():
    hf = terrain.Heightfield(SIZE, SAMPLES, center=CENTER)
    hf.noise(0.25, 60.0, seed=81, octaves=3).noise(0.015, 2.0, seed=82, octaves=2)
    hf.z -= hf.height(0, 0)
    hf.flatten(*ROVER[:2], 6.0, 6.0, z=0.0)
    features.shape(hf, FEATURES)
    # Micro-relief only on the natural strips: everything else is engineered.
    return add_relief(hf, [landscape.Base(terrains.DEFAULT_GROUND)], NATURAL, seed=23, within=natural(hf))


def natural(hf):
    """Where the course has natural micro-relief (landscape.only): the natural strips, easing in over
    NATURAL_EASE."""
    return landscape.only(hf, [strip.polygon() for strip in NATURAL], NATURAL_EASE)


def lane_info(lane, v=0.0):
    """What a station sheet point says about a strip: heading and profile."""
    return dict(heading_deg=round(math.degrees(lane.yaw) % 360, 1), offset_m=v,
                profile=[{"length_m": round(length, 2), "grade_deg": grade} for length, grade in lane.segments])


def build(models_dir, worlds_dir, media):
    hf = make_terrain()
    w = WorldBuilder(KEY, TITLE, None, site(*SITE), hf, models_dir, worlds_dir, media, seed=23, name=KEY,
                     solver=SOLVER)
    w.terrain()
    features.dress(w, FEATURES)
    w.place(props.start_gate(models_dir, media), "start_gate", *GATE, math.pi / 2)
    w.rover(*ROVER)

    west_sign, east_sign = -5.5, 6.0  # sign x by the road; west signs read from the east and vice versa
    for mu, y, surface in zip(FRICTION_MU, FRICTION_LANES, FRICTION_HILL.surfaces):
        kind = surface.kind
        w.station(surface.key, *FRICTION_HILL.at(0.0, surface.offset), f"Friction lane mu {mu:.2f}",
                  f"{kind.title}: {FRICTION_HILL.describe(surface.length)}; then a plateau and the way down "
                  f"({terrains.TYPES[terrains.DEFAULT_GROUND].title.lower()}). Holds or climbs up to "
                  f"{kind.traction.climb_deg:.0f} deg.",
                  ["FRICTION LANE", f"MU {mu:.2f}", "10 20 30 DEG"], (west_sign, y + 3.5), EAST,
                  mu=mu, length_m=surface.length, **lane_info(FRICTION_HILL, surface.offset))
    for lane in (SAND_PIT, CLAY_PATCH, SLICKROCK_SLAB):
        kind = lane.surfaces[0].kind
        title = lane.key.replace("_", " ").capitalize()
        steepest = max(abs(grade) for _, grade in lane.segments)
        traction = terrains.traction(kind)
        climb, hold = round(traction.climb_deg), round(traction.hold_deg)
        digs = ", but slipping wheels dig in and stall well below that" if traction.dig_rate else ""
        w.station(lane.key, *lane.start, title, f"{kind.title} (holds a parked rover up to {hold} deg, climbs up "
                  f"to {climb} deg{digs}): {lane.describe()}.",
                  [title.upper(), f"CLIMBS {climb} DEG", f"{steepest:.0f} DEG SLOPES"],
                  (west_sign, lane.start[1] + 3.5), EAST, climb_deg=climb, hold_deg=hold, **lane_info(lane))
    for lane in SIDE_SLOPES:
        w.station(lane.key, *lane.start, f"Side slope {lane.cross:.0f} deg",
                  f"{lane.length:.0f} m along a {lane.cross:.0f} deg side slope, the south side up: tests roll "
                  "stability and sideways slip.", ["SIDE SLOPE", f"{lane.cross:.0f} DEG"],
                  (west_sign, lane.start[1] + 4.5), EAST, cross_slope_deg=lane.cross, **lane_info(lane))

    length, width = GARDEN_SIZE
    for key, x, y, size in GARDENS:
        w.rock_garden(key, x, y, length, width, size)
        w.station(key, x - length / 2, y, f"Rock garden {size * 100:.0f} cm",
                  f"A {length:.0f} x {width:.0f} m plot of {size * 100:.0f} cm rocks (+-20 %), one per "
                  f"{garden_spacing(size):.1f} m cell; the wheel radius is {ROVER_PARAMS.wheel_radius * 100:.0f} cm.",
                  ["ROCK GARDEN", f"{size * 100:.0f} CM"],
                  (east_sign, y + 4.5), WEST, rock_size_m=size, length_m=length, width_m=width, heading_deg=0.0)
    heights = " ".join(f"{h * 100:.0f}" for h in BUMPS.heights)
    w.station("articulation", *BUMPS.start, "Articulation track",
              f"Humps of {heights} cm under the left wheels, then the right, every {BUMPS.spacing:.0f} m; then a "
              f"{TWIST_DITCH.depth * 100:.0f} cm diagonal ditch that takes a front wheel and the opposite rear wheel "
              f"together, turning the rockers to their {ROVER_PARAMS.rocker_limit} rad limit.",
              ["ARTICULATION", f"BUMPS {heights}", "TWIST DITCH"], (east_sign, BUMPS.start[1] + 3.5), WEST,
              humps=[dict(w.geo(*BUMPS.at(u, v)), height_m=h, side="left" if v > 0 else "right")
                     for u, v, h in BUMPS.humps()],
              twist_ditch=dict(w.geo(TWIST_DITCH.x, TWIST_DITCH.y), depth_m=TWIST_DITCH.depth,
                               angle_deg=round(math.degrees(TWIST_DITCH.angle), 1)), heading_deg=0.0)
    w.station("washboard", *WASHBOARD.start, "Washboard",
              f"{WASHBOARD.length:.0f} m of corrugations: crests every {WASHBOARD.wavelength} m, "
              f"+-{WASHBOARD.amplitude * 100:.0f} cm.", ["WASHBOARD"], (east_sign, WASHBOARD.start[1] + 3.5), WEST,
              heading_deg=0.0, length_m=WASHBOARD.length)
    for step in STEPS:
        w.station(step.key, step.x - step.length / 2, step.y, f"Step {step.top * 100:.0f} cm",
                  f"Drive east: a {step.top * 100:.0f} cm up-step, {step.length:.0f} m on top, a "
                  f"{step.top * 100:.0f} cm drop-off.", ["STEP", f"{step.top * 100:.0f} CM UP / DOWN"],
                  (east_sign + 2.0, step.y + 3.0), WEST, height_m=step.top, heading_deg=0.0)
    for strip in NATURAL:
        kind = strip.kind
        traction = terrains.traction(kind)
        title = kind.title
        slope = f"a {strip.grade:.0f} deg slope" if strip.grade else "flat"
        w.station(strip.key, *strip.start, f"Natural {title.lower()}",
                  f"{strip.length:.0f} x {strip.width:.0f} m of {title.lower()}, {slope}, as the mission worlds have it: "
                  f"its MDRS micro-relief and clutter (climbs up to {traction.climb_deg:.0f} deg).",
                  ["NATURAL GROUND", title.upper()], (west_sign, strip.start[1] + strip.width / 2 + 1.5), EAST,
                  ground=kind.key, grade_deg=strip.grade, length_m=strip.length, width_m=strip.width,
                  heading_deg=round(math.degrees(strip.yaw) % 360, 1))
    w.clutter(within=natural(hf) > 0, avoid=w.keep_clear(), clearance=2.0)
    (x, y, radius), (count, sizes) = BOULDER_FIELD, BOULDERS
    w.rock_field("boulder_field", w.scatter(count, (x, y), radius, sizes, avoid=w.keep_clear(), clearance=2.0))
    w.station("boulder_field", x - radius, y, "Boulder field",
              f"{count} boulders of {sizes[0]:.1f}-{sizes[1]:.1f} m within {radius:.0f} m.", ["BOULDER FIELD"],
              (east_sign, y), WEST, radius_m=radius)
    return w.write()
