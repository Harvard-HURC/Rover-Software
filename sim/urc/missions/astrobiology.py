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
slopes, two rock fields and the ledge's blocks. Around them the ground is
painted (PAINT): packed regolith with patches of crusted sand sheet, wash
sand on the wash floor, the hills' badland slopes banded maroon, grey and
white (STRATA) under sandstone caps that shed block fields, bare slickrock
on the sandstone ledge; badland belts (BADLANDS) lie out towards the site's
edge. Every type carries its MDRS micro-relief and clutter (world.add_relief,
WorldBuilder.clutter), as in Delivery.
"""
import math

from .. import appearance, features, landscape, props, rules, terrain, terrains
from ..world import WorldBuilder, add_relief, site

KEY = "astrobiology"
TITLE = "Astrobiology"
SITE = (38.4010, -110.7960)  # lat, lon of C2 (altitude: world.site)
SIZE, SAMPLES, CENTER = 1024.0, 2049, (0.0, 0.0)  # 0.5 m samples (gate G7), the relief swatches' grid
SOLVER = "pgs"  # each wheel grips mu times its own load; physics fast enough (gates G2, G5: 3.7x real time)

C2 = (0.0, 0.0, 0.0)
ROVER = (8.0, 6.0, math.radians(30))
BOUNDARY_RADIUS = 480.0
STAKES = 12

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

# Badland belts (x, y, radius) out towards the site's edge, clear of every unit and the wash: with the hills
# about an eighth of the ground, which the real square's rough tail needs (design 5.4; Delivery's note).
BADLANDS = [features.Patch(f"badlands_{k}", terrains.BADLAND_SLOPE, x, y, r, irregularity=0.4)
            for k, (x, y, r) in enumerate(((-330.0, 380.0, 100.0), (380.0, 330.0, 110.0), (300.0, -360.0, 100.0),
                                           (-380.0, -330.0, 90.0), (60.0, 400.0, 70.0), (-60.0, -400.0, 70.0),
                                           (430.0, 60.0, 60.0), (-430.0, 40.0, 60.0)))]
PAINT = [landscape.Base("regolith"), landscape.Noise("sand_sheet", feature_m=120.0, cover=0.3),
         landscape.Along(tuple(WASH), "wash_sand", half_width=6.0), landscape.Steeper(20.0, "badland_slope"),
         landscape.Hills(tuple(HILLS), slope="badland_slope", cap="caprock"),
         landscape.Hills((LANDFORMS[-1],), slope="slickrock", cap="slickrock"),  # the sandstone ledge
         landscape.Below("caprock", "block_field", reach_m=30.0)]
# The Morrison look of the banded hills: grey-blue bentonite among the maroon and white bands (A: today's
# bentonite and mudstone layers, NAIP grey shale).
STRATA = {"badland_slope": appearance.Strata(bands=(terrains.NAIP["maroon"], terrains.NAIP["grey_shale"],
                                                    (160, 138, 135), terrains.NAIP["white"]))}
CAPROCK_Z = 16.0  # [m] above C2: the hills' sandstone caps, where slab joints show
STAKE_PADS = [(BOUNDARY_RADIUS * math.cos(a), BOUNDARY_RADIUS * math.sin(a), 3.0)
              for a in (2 * math.pi * k / STAKES for k in range(STAKES))]
RELIEF = dict(features=FEATURES + BADLANDS, pads=[(*C2[:2], 15.0)] + STAKE_PADS)  # kept flat (design 5.4)

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
    hf.noise(3.0, 250.0, seed=61, octaves=3).noise(0.8, 40.0, seed=62, octaves=3)
    hf.z -= hf.height(0, 0)
    features.shape(hf, LANDFORMS + FEATURES + BADLANDS)
    hf.flatten(*C2[:2], 15.0, 20.0, z=0.0)
    return add_relief(hf, PAINT, seed=19, **RELIEF)  # below 4 m the ground is MDRS's own


def build(models_dir, worlds_dir, media):
    hf = make_terrain()
    w = WorldBuilder(KEY, TITLE, "1.b", site(*SITE), hf, models_dir, worlds_dir, media, seed=19, solver=SOLVER)
    w.sheet["time_limit_s"] = list(rules.ROVING_TIME)
    w.paint(PAINT)
    w.terrain(strata=STRATA, cap=CAPROCK_Z)
    features.dress(w, FEATURES + BADLANDS)
    w.c2(*C2)
    w.rover(*ROVER)
    w.sheet["site_radius_m"] = rules.SITE_RADIUS

    # Mission area stakes (the area is marked, the sample sites are not).
    stake = props.field_sign(models_dir, media, "boundary", ["URC", "SITE LIMIT"], height=0.9)
    w.ring(stake, "boundary", C2[:2], BOUNDARY_RADIUS, STAKES)

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
    sites = [(x, y, radius + 10.0) for _, _, (x, y), radius, _ in UNITS]
    w.clutter(avoid=keep, clearance=4.0, rock_sizes=(0.15, 0.3), rocks_within=w.near([(*C2[:2], 60.0)] + sites))
    w.pebbles([(*ROVER[:2], 35.0)] + [(x, y, 6.0) for _, _, (x, y), _, _ in UNITS])
    return w.write()
