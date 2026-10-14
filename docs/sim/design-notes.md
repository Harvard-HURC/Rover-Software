# Design notes

Moved from the simulation's manual ([manual.md](manual.md)); code comments cite them as
"sim/README.md, design notes". Paths here are relative to `rover_sim/` unless they start
with a top-level folder of the repository (`rover_sim/`, `rover_control/`, `docs/`).

## Why the differential is a plugin, not a mimic joint

The obvious way to model a differential in SDF is a mimic joint:
`rocker_right_joint` mimics `rocker_left_joint` with multiplier −1. We tried
it first (2026-10-05) and measured, in this env (gz-sim 8.10, gz-physics 7,
DART 6.19):

- **Only the Bullet-Featherstone engine implements mimic** in Gazebo
  Harmonic; DART ignores the tag.
- **Under Bullet the mimic differential is excellent.** With a 10 cm block
  under the front-left wheel the rockers settled at −0.0558 / +0.0558 rad
  (geometry: ±0.0553), and the coupling is two-sided — pushing either rocker
  moves both equally, like a real differential.
- **But under Bullet the rover cannot turn in place.** Commanding 0.5 rad/s
  for 4 s gave zero yaw while the wheels spun at full speed. Bullet's
  friction is a box model: a wheel sliding forward keeps its full sideways
  grip, and because the half-wheelbase (0.45 m) is longer than the half-track
  (0.40 m), sideways scrub always beats the drive. Bullet ignores the usual
  fix (`fdir1` + `mu`/`mu2`), joint damping, and applies SDF's default
  torsional friction as spinning friction that also blocks yaw.
- **DART lets a contact's friction be set per contact** (below), which is
  what turning in place needs.

So the world runs DART, and `plugins/rocker_differential.cpp` provides the
differential. Every step it applies the same torque to both rocker joints:

    τ = −k (q_L + q_R) − c (q̇_L + q̇_R)        k = 5000 N·m/rad, c = 100 N·m·s/rad

That is a stiff spring-damper on the motion a differential forbids (both
rockers turning the same way relative to the body), leaving the allowed motion
(opposite rotation) free. Equal torque on both joints is exactly what a real
differential does — it splits the reaction torque between the sides. It is
slightly soft: a moment M through the differential leaves an error of
M / 2k (~1e-4 rad here). If a later Gazebo implements mimic in DART, the
plugin can be replaced by the `<mimic>` tag.

The plugins are C++ rather than Python gz-sim systems because Python systems
crash Gazebo's in-process Python test fixture.

## The drivetrain

`plugins/rover_drivetrain.cpp` (system `rover_sim::RoverDrivetrain`, design
spec section 6) drives each wheel the way its motor would and decides how
every wheel contact grips. Every 1 ms step:

- **Setpoints**: (vx ∓ wz·track·track_multiplier/2)/r per side, through a ramp
  (`gz::math::SpeedLimiter`, 8 rad/s², 1.2 m/s² (A) until the driver team
  reports its controller's ramp) and clamped to `Params.wheel_speed`.
  `track_multiplier` is 1.0: the raw skid-steer response, which the team's
  driver compensates (the user's choice; Clearpath ships 1.875 for Husky).
- **Motor** per wheel (`rover_drivetrain.hh`, `DriveParams`): a DC motor
  (48 V, 0.46 Ω, kt = ke 0.0445; the Husky A200 motor at twice its 24 V, for 3 m/s), gearbox 50 at efficiency 0.8,
  current limit 20 A (35.6 N·m at the wheel), driveline 1500 N·m/rad with 1.5°
  backlash, a PI speed controller (kp 4 V/(rad/s), ki 160 V/rad, back-EMF
  feed-forward) integrated in 4 substeps, applied as the wheel joint's
  torque. Typical placeholders until the drivetrain is chosen.
- **Every wheel contact** gets its friction from the ground under it (the
  contact callback, `events::CollectContactSurfaceProperties`): a friction
  circle along the slip while sliding; while sticking a box aligned with the
  load it is expected to hold (the last tangential force, else downhill,
  else the heading), so a parked rover holds to atan(μs) at any heading;
  Stribeck from μs to μk over 3 cm/s on firm ground; smooth spatial μ noise
  (σ 0.2 over 0.3 m); force-dependent slip, slip × |wheel speed × r| / load
  per contact (gz WheelSlip's convention: a stopped wheel has none, so a
  parked rover does not creep). The load is the contact's own: a cylinder
  touches the ground at both tread edges, and the solver splits the wheel's
  load between them as it likes (Dantzig evenly, PGS nearly all on the first
  edge it visits, 90 / 2 N), so each edge's compliance comes from its load
  in the previous step, which the wheel joint's transmitted torque gives
  (within 0.05 N of the contact forces, at no cost; with even shares PGS
  slipped 1.85× the design in sand). The rule reads the wheel's spin through
  the controller's 5 ms speed filter: the 1 ms coupling of motor, contact and
  DART otherwise makes wheels chatter at 250 Hz and parked rovers creep.
- **Where the ground comes from**: on the heightmap, the world's `ground.png`
  under the contact (nearest sample); on the terrain model's other shapes,
  `ground.json`'s collision map (exact name, then prefix: `rocks_`, `slabs_`,
  `risers_` are rock); on a plane, the default surface (regolith); on any
  other model, the μ its SDF `<surface>` sets (as μs = μk, Crr 0.015, slip
  0.05), else the object surface (`manmade`). A world without a ground map
  uses the rows gen_model writes into the plugin from the catalogue.
- **Loads** come from each wheel joint's transmitted wrench (gate G8: within
  2.6 % of contact loads, and cheaper): the contact force is minus the joint
  force, the wheel's weight and the hub force.
- **Hub forces** (in the contact plane, `Link::AddWorldForce`): rolling
  resistance −Crr·D·N along the heading and bulldozing −k_b·D²·N along the
  axle (loose ground pushing back on a wheel moving sideways), each faded in
  over 7.5 mm/s of hub speed. As joint torques the speed controller would
  cancel them inside the joint.
- **Dig-in** per wheel on loose ground: the dig factor D grows with slip at
  dig_rate / sinkage per metre and heals with travel (1/e per 0.3 m), capped
  at dig_max; it scales that wheel's rolling resistance by D and bulldozing
  by D². The default is the strong preset (the user's choice): on loose sand,
  wash sand and dusty clay a sustained spin digs in until the rover can no
  longer turn and must drive out (dig_rate 0.05, D_max 2.0); the crusted sand
  sheet digs in mildly. `terrains.DIG = "mild"` switches every world and the
  rover to the mild preset (a spin in sand slows to ~0.7× and keeps turning).
  Dig-in changes resistance, not geometry: physics never sinks a wheel
  further than the static carve; the cues of Visible dig-in draw it.
- **Visible dig-in**: the tyre sink (`DriveParams.dig_sink`) is drawn from
  each wheel's D at the end of its step, and the ruts and pits
  (`DriveParams.ruts`) are fed from it; both read the drivetrain's state and
  change none of it (Visible dig-in, below).
- **Dust**: none by default (the user's decision of 2026-10-07): gz-rendering
  8.2.2's depth camera sees any visible particle (Gazebo lessons), and clean
  depth images and point clouds come first. `DriveParams(dust=True)` (or
  `dust: bool = True` in `gen_model.py`, then `pixi run sim-model`) puts an
  emitter behind each rear wheel on the rocker links and gives the drivetrain
  their topics; the drivetrain sets their rate from speed, slip, dig factor
  and the ground's dust factor (at most 40 /s, nothing below 0.05 m/s). The
  emitters ask for a scatter ratio near none (`dust_scatter_ratio`), which
  8.2.2 does not apply, so the depth image and point cloud then see the
  dust. Off, the drivetrain sends nothing on the emitter topics. Worlds load
  `gz-sim-particle-emitter-system`, idle without emitters, so the switch
  alone brings the dust back.
- **Odometry and tf** as DiffDrive publishes them (`gz::math::DiffDriveOdometry`,
  from the mean of each side's wheel angles, 50 Hz). No slip: a spin is
  over-reported by 1/ratio, as on a real rover; `/model/rover/ground_truth`
  is the truth. Realistic slip makes a gyro yaw-rate loop worth having in
  the driver.
- **Command timeout**: a command older than 0.5 s (wall clock) counts as
  zero, so a commander that dies cannot leave the rover driving
  (`DriveParams.cmd_timeout`; 0 holds the last command; tests use the sim
  clock).
- **Reset**: `ISystemReset` clears the command, motors, ramps, dig-in and
  odometry, puts the tyres back at their SDF poses and clears the ruts and
  pits.

Measured with the physical rover on flat ground of one type (`pixi run
sim-realism`, μ noise on), the turn-in-place ratio (yaw rate / command, wz
1 rad/s) on both of DART's solvers against the quasi-static closed form of
the design (wheels at (±0.45, ±0.40) m). PGS, the solver of four of the five
worlds, gives each wheel μ times its own load, so the diagonal a turn unloads
grips less and firm ground turns 0.02–0.04 below the closed form (gravel
0.374 against 0.410, the most); every catalogue ground within ±0.04 on both:

| Ground | Dantzig | PGS | Closed form |
|---|---|---|---|
| rock, slickrock, caprock | 0.432 | 0.411 | 0.436 |
| manmade (pads, objects) | 0.431 | 0.404 | 0.434 |
| clay crust | 0.397 | 0.369 | 0.401 |
| regolith (the default) | 0.373 | 0.345 | 0.377 |
| sand sheet | 0.322 | 0.308 | 0.328 |
| sand | 0.248 | 0.253 | 0.261 |
| wash sand | 0.169 | 0.181 | 0.182 |

A 10 s spin in loose sand (yaw ratio in successive seconds), strong preset:
0.145, 0.028, then 0.018 (0.07× fresh: it cannot turn, and drives out
straight); mild: 0.20, 0.22, then 0.19 (0.74×). Other measurements (`test_drivetrain.py`,
2026-10-07): loaded/light diagonal loads while spinning on μ 1 175/43/51/182 N
(PGS; the rocker pivots' moments unload one diagonal), loaded-wheel current
9.75 A on μ 0.8; a 6.8 A current limit stalls the turn (7.5° in 5 s against
115°); turn rise 0.31 s with the ramp, 53 ms without it, stop 27 ms; slip in
sand at 0.5 m/s 20 % on either solver (PGS 19.8 %), torque 3.4 N·m; drift
across a 20° slope over 3.5 m: rock 0.06 m, regolith 0.42 m, sand 1.65 m; a
rover parked 60 s on 15° regolith or 20° sand moves under 1.2 mm.

The **solver** is set per world (`WorldBuilder(solver=...)`, recorded in the
sheet's `physics`): PGS gives each wheel μ times its own load, which the
diagonal unloading needs; DART's default Dantzig sizes the friction limits
from loads before friction. PGS in Delivery, Astrobiology, Autonomy and the
proving ground; Dantzig in Equipment Servicing, where PGS with the lander's
joints runs below real time, so per-wheel friction is approximate there.
`rover_test.sdf` and the test worlds of `simulate.py` use Dantzig; the
calibration rows of `test_drivetrain.py` (spin ratio, slip, dig-in, drift,
creep) run on both, and the proving ground's flat sand gives the test
worlds' numbers on its PGS (0.37 m/s at 0.5 commanded, dig factor 1.26).

## Visible dig-in

Dig-in (above) changes resistance, not geometry, and physics cannot sink a
wheel: DART's per-contact data has no normal offset, its ERP and CFM are
global, and it ignores the normal component of a contact's
`contactSurfaceMotionVelocity` (research of 2026-10-07: a wheel's height
unchanged at ±5 cm/s); tyre compliance lowers the body, not the wheel, and a
deeper static carve changes the contacts. When the user found that "dig is
not really visible", they chose three cues that only draw it (2026-10-07),
each behind its own switch in `gen_model.py`, all on:

| Cue | Switch | Drawn | Seen by |
|---|---|---|---|
| Tyre sink | `DriveParams.dig_sink` (`dig_sink_gain` 1.0, `dig_sink_max` 0.06 m, `dig_sink_tau` 0.1 s): the drivetrain's `<dig_sink>` | each tyre (D − 1) × s below its SDF pose, at true scale | every camera: the station's, the rover's RGB-D and the Gazebo GUI (through `/world/<w>/state`) |
| Ruts and pits | `DriveParams.ruts` (numbers in `TrackParams`): the drivetrain's `<tracks>` | on every ground with s > 0, a faint floor behind each wheel (low berms too where s ≥ 1.5 cm or the wheel digs in); darker, with higher berms and pits, where the wheels dig in | camera sensors only: the station's views and the rover's RGB-D, not `gz sim -g` |
| Tread tyres | `Params.tread_tyre` (`models/rover/meshes/wheel.glb`) | tread bars, a hub and spokes, one of them ochre, so a wheel spinning in place shows | every view of the wheels |

What drives them is each wheel's dig factor D (sinkage over static sinkage,
grown by slip on loose ground, healed by travel; at most 2.0 on sand, dusty
clay and wash sand under the strong preset, 1.25 on the sand sheet, 1
elsewhere) and its ground's static sinkage s (`sinkage_m` in
`terrains.TYPES`: sand and dusty clay 2 cm, wash sand 3 cm, the sand sheet
1.5 cm, mudstone, bentonite and badland slopes 1 cm, regolith, gravel and
biocrust 0.5 cm, rock, slickrock, caprock, pavement and the crusts 0). Not
built, by the user's choice: a "wheels digging in" badge on the station, and
ruts in the Gazebo GUI (it draws its own scene; they would need a mirror to
its marker service).

- **Tyre sink**: each wheel's visuals (the tread mesh or the plain cylinder),
  never its collision, are drawn (D − 1) × s below their SDF pose, straight
  down in the world, through a 0.1 s lag (`ShowSink`, `drive::VisualSink`):
  the spec's extra sinkage z_d at true scale, 2 cm on sand and dusty clay at
  D 2.0, 3 cm on wash sand, under 4 mm on the sand sheet, 5 mm crossing sand
  at its equilibrium D 1.26, nothing where D stays 1 or s is 0 (`dig_sink_max`
  is never reached at gain 1). On soft ground it follows D (healed 1/e per
  0.3 m of travel); on rock (s = 0), and off the ground, its target is 0 and
  the tyre rises through the 0.1 s lag, at its SDF pose within about 0.5 s (a
  wheel off the ground for a few steps does not flicker). The offset is
  re-expressed in the spinning wheel link's frame every step, predicted to the
  end of the step, and written only when it changes. Measured
  (`test_render.DigCues`): dug in to D 2.0 in sand, a camera 1.5 m beside the
  tyres sees their top edges 13 px lower (2 cm); every depth pixel that
  changes lies on a tyre (farther in the band its top left, nearer at its
  lower outline), and colour changes off the tyres are their shadows. At the
  station's 5 m chase camera 2 cm is about 3 px: the sink reads in close and
  fly views.
- **Ruts and pits** (`plugins/rover_tracks.hh`, `rover_tracks_render.hh`):
  each wheel on soft ground (s > 0, on the terrain heightmap or a plane) lays
  a cross-section every 5 cm of its travel (`spacing`), from its ground point,
  heading, D and slip at the end of the step; a wheel digging in place (D up
  0.05 within 5 cm, `pit_dig`) digs a pit, each deeper one replacing the last.
  A transparent dark floor lies 4 mm above the drawn surface, its opacity
  (0.14 + 0.30 (D − 1)) × clamp(s / 2 cm, 0.6, 1.5) drawn in levels of 15, 28,
  39 and 48 %, with tread marks every other 5 cm unless the wheel slips; it
  writes no depth. Berms stand s × (max(D − 1, 0) + 0.3) high (`berm_gain`
  1.0, `berm_base` 0.3), in ±35 % lumps, drawn from 4 mm: sand 6 mm at D 1 and
  2.6 cm at D 2, wash sand up to 3.9 cm, the sand sheet 4.5–8 mm (D up to
  1.25); regolith, gravel, biocrust, mudstone, bentonite and badland slopes (D
  stays 1) get only the faint floor, rock, pavement and the crusts nothing. A
  rim goes round each pit; a wheel scrubbing sideways in a spin leaves a wide
  smear, and a turn sharper than 45° between cross-sections starts a corner
  instead of a twisted strip. Pits stay open (review of 2026-10-07): a pit dug
  inside a standing one (within its rim's crest: the wheel scrubbed or crept
  while digging) replaces it, swept from where the old one began, so one hole
  has one rim; and a track laid while a pit stands draws no berm on a side
  where it would lie over the pit (a wheel backing out, or another wheel
  crossing or passing it). On the tests' dig-and-back-out drive the berms over
  the pits' floors went from 23–66 % of each floor to 0–8 %, seen from above;
  what is left is tracks laid before the pit was dug. Their colour comes from
  the terrain's `albedo.png` (the colour map as Terra draws it, 1024², written
  by `gen_worlds` beside `ground.png`; without one the terrain texture's mean
  colour, on a plane a default colour). The newest 8,000 cross-sections (about
  100 m of travel, `max_segments`) are kept, in chunks of 128, the oldest
  dropped. The rendering thread draws, in each frame, exactly what was laid by
  the frame's sim time (each record carries it, and a dropped chunk the time
  it was dropped: the last four stay readable for a frame drawn while the next
  step runs), at most 256 records a frame (`frame_budget`), so a station that
  connects late catches up over a few frames; a process that renders nothing
  reads no map and draws nothing.
- **Tread tyres**: the tyres' visual is `meshes/wheel.glb`
  (`gen_model.wheel_parts`, the cylinder's outline, 384 triangles, no SDF
  `<material>`): the plain tyre's dark with 16 light tread bars, a light hub
  and four spokes, one of them the chassis' ochre. The bars alias above about
  3.9 rad/s in a 20 Hz view (they seem to creep), the ochre spoke only above
  63 rad/s, past the 20 rad/s top speed: dug in and spinning at 2.67 rad/s,
  its turn between two frames 0.1 s apart is the wheel's own to 1 %, while
  the plain tyre's frames are identical. The collision stays the cylinder; in
  sunlight the mesh (roughness 0.9) looks darker than the plain cylinder.
- **Together**: a wheel dug in to D 2 in sand is drawn 4 cm into the drawn
  surface (the 2 cm carve and the 2 cm sink: the physical s·D), and the rim
  round it stands up to 2.6 cm above that surface (measured 2.9 cm with the
  lumps), the soil pushed aside. The ruts' berm rule was set before the sink
  existed, to show the dig-in on its own; `TrackParams.berm_gain` scales it
  and stays 1.0 (open: design spec 17.5).
- **Visual only**, each switch on its own, and tested so: the base_link pose
  every step and the drivetrain states are bit for bit the same with the cues
  on and off, on both solvers (`test_drivetrain`: `DigSink` with all three
  against none, `Ruts` with the ruts against none); on a quiet machine (Gazebo
  lessons) the pictures are the same every run
  (`test_render.Ruts.test_two_runs_are_identical`) and on rock, or 0.9–1.5 s
  after a world reset, they are those without the cues, bit for bit
  (`DigCues`, `Ruts`). Depth: the ruts only bring it closer, every changed
  pixel on the ground within 0.45 m of a wheel's path, no higher above the
  terrain than a berm's lumpy crest (measured on sand 0–3.3 cm over nine runs,
  on the sand sheet ≤ 1.1 cm; the rule allows wash sand 5.3 cm). Along a ray
  the change is that height over the sine of the ray's angle to the ground, so
  it grows as the view grazes: 11 cm for `test_render.Ruts`'s RGB-D (rays
  about 15° down), 25–43 cm for cameras 0.25–0.6 m up, 66 cm for one 15 cm up
  (2.5°). A tyre drawn lower makes the band its top uncovered farther, all on
  the tyre, nothing floating. `test_gen_model` checks that each switch takes
  away only its own cue; `ctest` the sink's rule and lag and the ruts' layer
  (spacing, soft ground only, the pit rule, pits kept open, the ring, resets,
  the per-frame sim-time cut, with a ring that drops chunks too, geometry);
  `test_urc_terrain` the albedo map.
- **What the station sees**: the chase view shows the track behind the rover
  (1.3 % of its pixels after 1.5 m on sand, 1.7 % dug in) and the tread tyres;
  the rover eye looks ahead and never sees a wheel digging in place, only the
  pits once the rover has backed out or turned (3.2 % of its pixels); in the
  320 px picture-in-picture the tracks are faint.

## Skid-steer friction

A skid-steer rover turns in place only by scrubbing its wheels sideways. With
isotropic Coulomb friction the quasi-static balance of the four wheels'
friction moments gives a yaw rate of c²/(a²+c²) of the command (0.441 here,
a = 0.45, c = 0.40), less with rolling resistance and bulldozing; published
numbers agree (Clearpath's Husky multiplier 1.875 is a ratio of 0.533 against
0.540 predicted; a Pioneer P3-AT measured 0.63–0.71 against 0.687). So the
rover turns at 0.18–0.44 of the command, depending on the ground, and the
odometry over-reports it.

DART's own contact rule cannot do this: it combines the two shapes' μ as
min(μ_a, μ_b) in each box direction. The DiffDrive variant's tyres have μ 1.0
along the tread and 0.5 across it to turn at all (0.87 of the command on
μ 1 ground), and on ground of μ ≤ 0.5 that anisotropy vanishes and the
rover cannot turn in place (measured 2026-10-06: 0.000 on μ 0.4 and 0.5
boxes). Box friction with the wheelbase longer than the track cannot turn in
place at all. Hence the per-contact friction circle of the physical
drivetrain on every wheel contact, terrain or object.

