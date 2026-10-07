"""Astrobiology Mission (rule 1.b).

A science site near MDRS. Teams pick their own sample sites within 0.5 km
of the C2 station (1.b.ii); individual sites are not marked (Q&A
Astrobiology 5), only the mission area (orange stakes at 480 m). The world
offers the geologic units a team would choose between, each a ground-truth
entry in the sheet:

- banded mudstone hills (Morrison Formation look: grey-blue bentonite, maroon,
  purple, cream bands by height),
- a dry sandy wash,
- a gypsum outcrop,
- biological soil crust patches (dark, knobbly: cyanobacteria and lichens),
- a sandstone ledge,
- lichen-covered boulders,
- shrubs along the wash.
Sub-surface sampling (10 cm, 1.b.vi) is not simulated: the sheet gives each
unit's notes instead.

The ground the rover crosses (urc/terrains.py, urc/features.py): soft sand
along the wash floor, bentonite clay aprons at the feet of the banded hills,
a loose scree chute up the east face of the small hill; talus on the hill
slopes, two rock fields and the ledge's blocks.
"""
import math

from .. import features, props, rules, terrain, terrains
from ..world import Layer, WorldBuilder, site

KEY = "astrobiology"
TITLE = "Astrobiology"
SITE = (38.4010, -110.7960)  # lat, lon of C2 (altitude: world.site)
SIZE, SAMPLES, CENTER = 1024.0, 1025, (0.0, 0.0)
SOLVER = "pgs"  # each wheel grips mu times its own load; physics fast enough (gates G2, G5: 3.7x real time)

C2 = (0.0, 0.0, 0.0)
ROVER = (8.0, 6.0, math.radians(30))
BOUNDARY_RADIUS = 480.0

HILLS = [features.Mesa("hill_0", 170.0, 150.0, 30.0, 24.0, 22.0, seed=51, irregularity=0.25),
         features.Mesa("hill_1", 265.0, 55.0, 18.0, 14.0, 15.0, seed=52, irregularity=0.25),
         features.Mesa("hill_2", -285.0, 250.0, 25.0, 18.0, 18.0, seed=53, irregularity=0.25)]
GYPSUM_MOUND = (-140.0, 70.0)
SANDSTONE_LEDGE = (-220.0, -200.0)
LANDFORMS = [*HILLS, features.Mesa("gypsum_mound", *GYPSUM_MOUND, 9.0, 1.5, 6.0, seed=64),
             features.Mesa("sandstone_ledge", *SANDSTONE_LEDGE, 30.0, 4.0, 4.0, seed=65, irregularity=0.2)]
WASH = [(-500.0, -150.0), (-250.0, -100.0), (-80.0, -130.0), (100.0, -90.0), (300.0, -160.0), (500.0, -120.0)]
CRUSTS = [(-60.0, -40.0, 9.0), (90.0, -30.0, 7.0), (40.0, 110.0, 8.0), (-180.0, -60.0, 10.0)]
LICHEN_BOULDERS = (-120.0, 200.0, 12.0)
ROCK_FIELDS = [(60.0, 60.0, 30.0, 18.0, 0.15), (-200.0, 130.0, 30.0, 20.0, 0.25)]  # x, y, length, width, rock size

FEATURES = [
    features.Wash("wash", WASH, depth=2.5),
    # Weathered bentonite below the banded hills.
    features.Patch("clay_apron_0", terrains.CLAY, 122.0, 107.0, 12.0),
    features.Patch("clay_apron_1", terrains.CLAY, -245.0, 205.0, 10.0),
    # A planar ~27 deg chute up hill 1's east face.
    features.Slope("scree_chute", terrains.SCREE, (310.0, 55.0), (283.0, 55.0), 10.0, run_on=8.0, falloff=4.0),
    *[features.Patch(f"biocrust_{k}", terrains.BIOCRUST, x, y, r) for k, (x, y, r) in enumerate(CRUSTS)],
    features.Patch("gypsum", terrains.GYPSUM, *GYPSUM_MOUND, 13.0),
]

# Four layers is Gazebo's limit; bands by height give the Morrison look.
LAYERS = [Layer("regolith"), Layer("bentonite", start=4.0, fade=0.8), Layer("mudstone", start=9.0, fade=0.7),
          Layer("caprock", start=16.0, fade=1.0)]

BIOCRUST_NOTES = "Cyanobacteria, lichens and mosses: the highest surface biomass on site."
UNITS = [
    ("banded_hills", "Banded mudstone hills (Morrison-like)", (HILLS[0].x, HILLS[0].y), 55.0,
     "Bentonitic clay layers; clays preserve organics; sample the base of a layer."),
    ("dry_wash", "Dry sandy wash", (-80.0, -130.0), 20.0,
     "Loose sand and gravel; transient water concentrates microbes below the surface."),
    ("gypsum", "Gypsum outcrop", GYPSUM_MOUND, 14.0,
     "Evaporite; endoliths live inside translucent gypsum crusts."),
    ("biocrust", "Biological soil crust", CRUSTS[0][:2], CRUSTS[0][2], BIOCRUST_NOTES),
    ("sandstone_ledge", "Sandstone ledge", SANDSTONE_LEDGE, 30.0,
     "Cemented sandstone; hard to dig; desert varnish on exposed faces."),
    ("lichen_boulders", "Lichen-covered boulders", LICHEN_BOULDERS[:2], LICHEN_BOULDERS[2],
     "Epilithic lichens on north faces; the soil beneath holds fungal hyphae."),
] + [(f"biocrust_{k}", "Biological soil crust", (x, y), r, BIOCRUST_NOTES) for k, (x, y, r) in enumerate(CRUSTS[1:], 1)]


def make_terrain():
    hf = terrain.Heightfield(SIZE, SAMPLES, center=CENTER)
    hf.noise(3.0, 250.0, seed=61, octaves=3).noise(0.8, 40.0, seed=62, octaves=3).noise(0.05, 4.0, seed=63, octaves=2)
    hf.z -= hf.height(0, 0)
    features.shape(hf, LANDFORMS + FEATURES)
    hf.flatten(*C2[:2], 15.0, 20.0, z=0.0)
    return hf


def build(models_dir, worlds_dir, media):
    hf = make_terrain()
    w = WorldBuilder(KEY, TITLE, "1.b", site(*SITE), hf, models_dir, worlds_dir, media, seed=19, solver=SOLVER)
    w.sheet["time_limit_s"] = list(rules.ROVING_TIME)
    w.terrain(LAYERS)
    features.dress(w, FEATURES)
    w.c2(*C2)
    w.rover(*ROVER)
    w.sheet["site_radius_m"] = rules.SITE_RADIUS

    # Mission area stakes (the area is marked, the sample sites are not).
    stake = props.field_sign(models_dir, media, "boundary", ["URC", "SITE LIMIT"], height=0.9)
    w.ring(stake, "boundary", C2[:2], BOUNDARY_RADIUS, 12)

    for key, title, (x, y), radius, notes in UNITS:
        w.point(key, x, y, title=title, radius_m=radius, notes=notes,
                distance_from_c2_m=round(math.hypot(x - C2[0], y - C2[1]), 1))
    w.sheet["ground_truth_units"] = [key for key, *_ in UNITS]

    w.task(id="sites", rule="1.b.ii-iii", title="Investigate at least two sites (panorama, close-up, GNSS with "
           "elevation and accuracy) within 0.5 km of C2", min_sites=rules.MIN_SITES, radius_m=rules.SITE_RADIUS)
    w.task(id="sample", rule="1.b.vi-vii", title="Collect a >= 5 g sub-surface sample from >= 10 cm, seal it in "
           "the onboard cache and return it to C2", depth_m=rules.SAMPLE_DEPTH, judged="return to C2 only")
    w.task(id="life_detection", rule="1.b.v", title="Two life-detection methods on board, one a bioassay",
           judged="not simulated")

    keep = w.keep_clear()
    w.rock_field("scattered", w.scatter(800, CENTER, 470.0, (0.04, 0.35), avoid=keep, clearance=8.0))
    w.rock_field("ledge_blocks", w.scatter(110, SANDSTONE_LEDGE, 40.0, (0.2, 0.8), avoid=keep, clearance=3.0))
    w.rock_field("talus", w.scatter_each([(h.x, h.y, h.radius + h.cliff + 15.0) for h in HILLS], 150, (0.1, 0.6),
                                         avoid=keep, clearance=8.0, min_slope=10.0))  # fallen from the hill faces
    for k, (x, y, length, width, size) in enumerate(ROCK_FIELDS):
        w.rock_garden(f"rock_field_{k}", x, y, length, width, size, avoid=keep, clearance=3.0)
    w.rock_field("lichen_boulders", w.scatter(14, LICHEN_BOULDERS[:2], LICHEN_BOULDERS[2], (0.4, 1.0), clearance=0),
                 palette="lichen")
    w.shrubs(w.points_along(WASH, 25.0, jitter=8.0) + w.scatter_points(30, CENTER, 450.0, avoid=keep, clearance=10.0))
    return w.write()
