# Rover simulation (Gazebo Harmonic)

A generic model of our rover: rectangular chassis, rocker suspension with a
differential (4 wheels, a rocker-bogie without the bogies), tank / skid-steer
drive. Geometry is the placeholder from `driver/include/rover_driver/config.hpp`
until the mechanical numbers arrive.

Design spec: `docs/superpowers/specs/2026-10-05-gazebo-rover-model-design.md`.

## Quick start

```bash
pixi run sim          # build the plugin, regenerate the model, open Gazebo
pixi run sim-bridge   # in a second terminal: ROS 2 <-> Gazebo topics
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.2}}'
pixi run sim-test     # headless tests
```

`pixi run sim other_world.sdf` opens another world (or a name from
`sim/worlds`, e.g. `pixi run sim urc_autonomy`; see [URC 2027 mission
worlds](#urc-2027-mission-worlds)). In the GUI you can also drive with the
Teleop plugin (⋮ menu → Teleop, topic `/model/rover/cmd_vel`).

On macOS Gazebo cannot run its server and GUI in one process, so `sim/run.sh`
starts the server in the background and the GUI in the foreground; closing
the GUI stops both.

## Files

| File | What it is |
|---|---|
| `gen_model.py` | All model parameters (`Params`) and the SDF generator |
| `models/rover/model.sdf` | Generated — edit `Params`, not this file |
| `plugins/rocker_differential.cpp` | The differential (see design notes) |
| `worlds/rover_test.sdf` | Test ground: 10 cm step under the left wheels (x 2–3 m), 15° ramp to a 0.3 m platform (x 5–8.7 m), rocks |
| `bridge.yaml` | ROS 2 bridge topics |
| `gen_worlds.py`, `urc/` | URC mission worlds generator (see below) |
| `worlds/urc_*.sdf`, `worlds/urc_*.json` | Generated URC worlds and their mission sheets |
| `models/urc_*` | Generated URC models (terrains, AR posts, astronaut, lander, objects) |
| `referee.py` | Runs a URC mission against a live simulation |
| `plugins/joint_monitor.cpp` | Throttled joint/link state + key presses for the referee |
| `tests/` | `test_gen_model.py` (no physics), `test_rover_sim.py` (headless Gazebo), `test_urc_*.py` (URC worlds, judges), `simulate.py` (helper) |

## Topics

| ROS 2 (via `sim-bridge`) | Gazebo | Type |
|---|---|---|
| `/cmd_vel` → | `/model/rover/cmd_vel` | Twist (vx, wz) |
| `/odom` | `/model/rover/odometry` | wheel odometry, `odom` → `base_link` |
| `/ground_truth/odom` | `/model/rover/ground_truth` | true pose, `world` → `base_link` |
| `/joint_states` | `/model/rover/joint_states` | 2 rockers + 4 wheels |
| `/imu` | `/model/rover/imu` | 100 Hz, frame `base_link` |
| `/gps/fix` | `/model/rover/navsat` | GNSS, 10 Hz, σ 0.5 m horizontal, 1 m vertical |
| `/camera/{image,depth,points,camera_info}` | `/model/rover/camera/{image,depth_image,points,camera_info}` | front RGB-D, 640×480, 15 Hz, frame `camera` |
| `/led` → | `/model/rover/led` | status LED: `red`, `blue`, `green` (flashing), `off` (URC rule 1.e.ii) |
| `/tf` | `/model/rover/tf` | `odom` → `base_link` |
| `/clock` | `/clock` | sim time |

Joint names: `rocker_left_joint`, `rocker_right_joint`, `wheel_{fl,rl,fr,rr}_joint`.
Rocker angle is positive when the rocker's front goes down (rotation about +y);
wheel angle is positive rolling forward. `base_link` is the driver's body
frame: x forward, y left, z up, on the ground midway between the wheels.

## Changing the rover

Edit `Params` in `gen_model.py`, then `pixi run sim-model` (`sim` and
`sim-test` do it for you). Masses, inertias, wheel positions, the
differential, motor limits, tire friction and IMU noise all come from there.

## Design notes

### Why the differential is a plugin, not a mimic joint

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
  (0.40 m), sideways scrub always beats the drive. The usual fix, lower
  sideways friction on the tires (`fdir1` + `mu`/`mu2`), is ignored by
  Bullet. So is joint damping; and Bullet applies SDF's default torsional
  friction (1.0) as spinning friction that also blocks yaw.
- **DART honors the anisotropic tire friction**: lateral μ 0.5, longitudinal
  μ 1.0 turned the rover 1.74 of a commanded 2.0 rad in 4 s.

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

The plugin is C++ rather than a Python gz-sim system because Python systems
crash Gazebo's in-process Python test fixture.

### Skid-steer friction

Tires use `mu2 = 1.0` along the direction of travel and `mu = 0.5` along the
axle (`fdir1`). The ratio is a tuning knob, not a measured tire property:
lower lateral μ turns faster. With the defaults the rover turns at ~87 % of
the commanded rate. `DiffDrive` odometry assumes no slip, so `/odom`
overestimates turns; compare with `/ground_truth/odom`. The real rover will
need an effective track width for the same reason.

## Known limitations

- The differential is a penalty spring (slightly soft, see above).
- The IMU sits at the chassis center but reports in `base_link`: orientation
  and angular rate are exact, linear acceleration differs by lever-arm terms.
- No TF for rockers and wheels in ROS (no URDF / `robot_state_publisher`).
- Sensors are GNSS, one RGB-D camera and the IMU; no lidar. Cameras render
  only in worlds with the Sensors system (the URC worlds), not in
  `rover_test.sdf`.
- Gazebo prints `XML Element[gz_frame_id], child of element[sensor], not
  defined in SDF` on load. Harmless: `gz_frame_id` is a gz-sensors extension
  that sets the IMU's frame id; SDF has no standard element for it.

## URC 2027 mission worlds

One world per field mission of the [URC 2027 rules](https://urc.marssociety.org/home/requirements-guidelines)
(and the Q&A as of 2026-10-03), placed at the real coordinates near MDRS.
Design: `docs/superpowers/specs/2026-10-06-urc-scenarios-design.md`.

![The four URC worlds](../docs/urc_worlds.png)

```bash
pixi run sim urc_autonomy          # or urc_equipment_servicing, urc_delivery, urc_astrobiology
pixi run referee autonomy          # second terminal: scripts the astronaut, judges, shows the LED
pixi run sim-bridge                # third terminal, for ROS 2
pixi run sim-worlds                # regenerate (sim, sim-test do it when sim/urc changes)
```

| World | Rules | What is in it |
|---|---|---|
| `urc_autonomy` | 1.e | Utah state square mile (38.411–38.425 N). Astronaut Assistance east of C2: astronaut at a GNSS point, Follow!/Stay!/Fetch!/Come!/Give!, rock pick hammer. Route-Finding north: start post (ArUco 0), a mesa with cliffs on most sides, a ~9° ramp and a ~23° gully, Post 1 (ArUco 1) on top, Post 2 (ArUco 2) behind it out of radio line of sight. Landing pad, rubble field, boulders. |
| `urc_equipment_servicing` | 1.d | Mock lander 90 m from the gate: K552 keyboard (87 spring keys, 2 cm tags 1–4), e-paper display, drawer with a cache well, latched hinged panel, locks A/B with a tubular key (tags 5/6, 1 cm tags at the keyholes), fuel tank with hose and 1.5" cam-lock coupler, ¼-turn valve, buttons, switches, knobs. Sample stand with the sample tube and the cache container. |
| `urc_delivery` | 1.c | ~900 m course, rougher with distance: toolbox + wrench → astronaut A; astronaut B's sign ("BRING: WATER JUG"); supply crate up a hill; a field sign with a search area for an instrument case; first-aid kit past a boulder field, over a ridge pass and down 0.3/0.6/1.0 m ledges to astronaut C behind the ridge (no radio); spectrometer lost in a wash. Steep loose mesa off-route. |
| `urc_astrobiology` | 1.b | Site within 0.5 km of C2 (boundary stakes at 480 m): banded mudstone hills, dry sandy wash, gypsum outcrop, biological soil crust, sandstone ledge, lichen boulders, shrubs. Sample sites are not marked. |

URC publishes the Delivery course script shortly before the competition
(rule 1.c.i); this course uses every task type the rules list. Terrain is
synthetic: URC has not released its DEM yet (Q&A Autonomy 20).

### Mission sheets

`worlds/urc_<mission>.json` is the judges' hand-out plus ground truth:
every place and object with world `x, y, z` and WGS84 `lat, lon, alt` (what
the rover's GNSS reports), the tasks with rule references, tolerances and
points, the C2 antenna, the rover start, and the terrain (`heightmap.png`
and a GeoTIFF `dem.tif`, so DEM tooling can be practised). `judges_only`
holds what the judges would not hand out (the easy route, the ridge pass).

World frame: x east, y north, z up, origin at the **terrain centre and its
lowest point** (Gazebo needs the heightmap there, see the design notes), so
the C2 station is not at the origin. `pixi run sim` points the GUI camera at
the rover.

### Referee

`pixi run referee <mission> [--method device|sign|speech|gesture] [--key word]`:

- **Autonomy**: moves the astronaut (`set_pose`) and gives each command by the
  chosen method: `device` → `/astronaut/command`, `speech` → `/astronaut/speech`
  (stands in for audio), `sign` raises the astronaut's sign (ArUco 10; the
  command order is fixed, Q&A Autonomy 13), `gesture` uses one gesture per
  command (beckon for Follow!/Come!, palm out for Stay!, point down for
  Fetch!, both arms out for Give!). Scores per rule 1.e (Follow! by method).
  The astronaut waits at the GNSS point until the rover arrives, so the
  sub-missions can come in either order; each later command times out after
  4 min. Stay! needs the rover with the astronaut and the astronaut > 20 m
  away; for Come! the astronaut first walks to 20 m; Fetch!/Give! need the
  hammer picked up after Fetch! and dropped by the astronaut. Route targets
  need the rover stopped within 1 m and the LED green.
- **Equipment Servicing**: draws a 3–6 letter launch key (printed and on
  `/urc/launch_key`), decodes key presses into the display text
  (`/urc/display`, backspace works), and judges the cache, drawer, panel,
  lock, hose, valve and controls from joint angles and part poses.
- **Delivery**: an object counts as delivered lying on the ground, at rest
  for 1 s, within 2 m of the astronaut; a quarter of the task for opening the
  toolbox or finding a stage-2 object (part of the task's share, not extra).
- **Astrobiology**: a site is a stop of ≥ 30 s, ≥ 20 m from other sites, C2
  and the start, within 0.5 km of C2; then back within 15 m of C2. The science
  is judged by people.

It also publishes `/urc/score` (JSON, 1 Hz) and `/urc/radio` (`los` /
`nlos` between the 3 m C2 antenna and the rover), re-sends the launch key and
display text every second, and draws the LED: the
rover's software publishes its state on `/led`, the referee colours the LED
on the rover's back. Where the rules give no points (Equipment Servicing,
Delivery), tasks share 100 points equally; the field judges decide that at
URC. Distances are from the rover's footprint; "stopped" is < 5 cm/s for 2 s.

### Gazebo lessons (gz-sim 8, this env)

Measured while building these worlds; each cost a wrong first attempt.

- **Heightmaps must sit at the world origin with their lowest point at
  z = 0.** DART collision ignores the heightmap's `<pos>` (and a negative
  `<pos>` z makes objects freeze or fall through), while the ogre2 visual
  ignores the model's pose. Anywhere else, what you see and what you hit
  differ. `urc/world.py` therefore shifts each world so the terrain is there.
- **Gazebo scales image heightmaps by the image's own maximum pixel**, not by
  the format's range: `terrain.py` stretches every heightmap to 0–65535.
- **ogre2 heightmaps blend at most four textures**; extra layers are silently
  dropped.
- **Hundreds of static rocks as separate bodies cost 3×**: each one's
  bounding box overlaps the heightmap's and is tested every step. Rocks are
  shapes of the terrain's own link instead (a body never collides with itself),
  and the catch-floor under the terrain is a box, not an infinite plane.
- **NavSat noise is in degrees** for latitude/longitude; `gen_model.py`
  converts metres.
- **The conda gz-rendering has a space-padded OGRE plugin path**: set
  `OGRE2_RESOURCE_PATH` (run.sh and the tests do) or cameras cannot render.
- **On macOS a rendering server must run on the main thread**, and ogre2
  starts once per process: tests strip the Sensors system from world copies
  and render the one camera test in a subprocess.
- `JointStatePublisher` publishes every 1 ms step here; the lander's 101
  joints use `JointMonitor` (50 Hz, plus an event per key press).

## Troubleshooting

- **Nothing on the topics / GUI shows an empty world:** gz-transport discovery
  can fail without a network (`No route to host`). Run everything, including
  the bridge, with `GZ_IP=127.0.0.1`.
- **`Failed to load system plugin [RockerDifferential]` (or `JointMonitor`):**
  run through `pixi run sim` (it builds the plugins and sets
  `GZ_SIM_SYSTEM_PLUGIN_PATH` to `sim/build`).
- **`Unable to load Ogre Plugin`, no camera images:** set
  `OGRE2_RESOURCE_PATH=$CONDA_PREFIX/lib/OGRE-Next` (run.sh does). The error
  is printed once even when the fallback then works.
- **The referee prints "waiting for the simulation":** start or unpause the
  world; the referee needs `/model/rover/ground_truth`.
