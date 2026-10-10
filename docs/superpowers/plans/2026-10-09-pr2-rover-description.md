# PR 2: The rover description, and the simulated rover built from it Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `rover_description` holds every physical number of the rover in `config/rover.yaml` and builds the standard description from it (`urdf/rover.urdf.xacro`, a `ros2_control` block, the tread tyre mesh, an RViz view); `rover_sim` builds the Gazebo rover that the worlds include from that description plus a wrapper xacro and `config/sim.yaml`, and `gen_model.py` is gone, with the simulated rover the same rover, number for number.

**Architecture:** Design spec `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`, sections 5, 6.1, 9.2 (row 2) and 10. `rover_sim/tools/gen_rover.py` expands `rover_sim/urdf/rover_sim.urdf.xacro` (which includes `rover_description/urdf/rover.urdf.xacro` and adds Gazebo's sensors and plugins in `<gazebo>` blocks) with xacro, converts the URDF with `gz sdf -p` and post-processes the converter's quirks away, writing `rover_sim/models/rover/model.sdf`, which the worlds keep including as `model://rover`. An equivalence test compares that model, and every variant the tests build, number by number with frozen copies of what `gen_model.py` wrote, and drives the frozen and the generated rover side by side in Gazebo. The simulation's Python reads both YAML files through one module, `rover_sim/rover_config.py`.

**Tech Stack:** xacro 2.1.1 (RoboStack Jazzy), sdformat 14.8 (`gz sdf -p`), Gazebo Harmonic (gz-sim 8.10), ROS 2 Jazzy (robot_state_publisher, joint_state_publisher_gui, rviz2, launch), ament_cmake with the team's linters, Python 3.12 unittest, pixi 0.81.

---

## Places and rules

| Name | Path |
|---|---|
| SRC | `/Users/alarion239/Desktop/hurc_ws/src` (the team's clone; branch `pr2/rover-description`) |
| WS | `/Users/alarion239/Desktop/hurc_ws` (the colcon workspace; `build/`, `install/`, `log/` are here) |
| T | `/private/tmp/claude-502/rover-pr2` (scripts, patches, logs; deleted in Task 17). If the implementing session lists a scratchpad directory, use `<scratchpad>/rover-pr2` as T in every command instead: the sandbox may refuse writes elsewhere |
| P | `/Users/alarion239/.pixi/bin/pixi` (0.81) |

- **Never push, never change anything on GitHub, never open a pull request.** The user approves every push. `/Users/alarion239/Desktop/Rover` is read-only (the rasters in `rover_sim/data` are symlinks into it; read them, never write there).
- Every commit message ends with the line `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`; the author is the configured `Alexander Belotserkovtsev <amb30239@gmail.com>`. Messages start `PR 2: `, as PR 1's started `PR 1: `.
- Every Gazebo run outside the test suite gets `GZ_IP=127.0.0.1` and its own `GZ_PARTITION`; the commands below set both. The suite sets its own partitions. Stop only processes you started: ports 8765 and 8766 and a heavy `EuclidPolish` Python job belong to the user's other work.
- Shell state does not persist between tool calls: every command block sets the variables it uses and starts with `cd`.
- The Bash tool stops foreground commands after 10 minutes: run `pixi run test` and the baseline suite in the background (`run_in_background`) with the output in a log under `$T`, and wait for them.
- Disk: about 23 GB free. Keep `$T` small (logs and patches); no Docker; one pixi environment, the one in SRC.
- Style: the repository's (PR 1's plan, `rover_sim/README.md`): short declarative sentences, units in brackets, numbers with their source label (A) assumed, (M) measured or tuned, (R) from a reference, and dated user decisions; Python in `rover_sim` at 120 columns; Python in `rover_description` passes the team's linters (`ament_flake8`: 99 columns, single quotes, Google import order; `ament_pep257`).
- **Patches.** Tasks 11, 12, 13 and 16 change many existing files in small ways. Each gives the exact change as a unified diff, made from a dry run of this plan on a copy of the branch. The dry run passed the equivalence and drive tests, `check_urdf`, `rover_description`'s colcon tests and linters, the full simulation suite and the byte-for-byte worlds; it could not run `pixi install` (it resolved the lock only), `rover_control`'s and the plugins' colcon tests, or the view launch with joint_state_publisher (not yet in the environment), so those expectations come from the dry lock and PR 1's counts. The critic reran it (2026-10-10) on an export of the branch, with this plan's files written, its patches applied in order (all applied cleanly) and xacro 2.1.1 from the rattler cache on `PYTHONPATH`: the ten models with zero differences, `Drives` bit for bit, `check_urdf` on both expansions run from another directory, `rover_description`'s 10 tests and its four linters, and the simulation suite, 546 tests with 9 skips and 2 expected failures (one `test_fly_camera.MapTool` failure came from the export's symlinked worlds, not from this plan). Save the diff exactly as shown to the file the step names (the Write tool keeps it byte for byte), then apply it with `git apply`; it applies only to the tree the earlier tasks left, so a hunk that does not apply means an earlier task differs from this plan: find the difference instead of forcing the patch.

## Decisions this plan makes

The spec and the task leave these open; each is final for PR 2 and stated in the commit or document that implements it.

1. **The worlds keep including `model://rover`; that model is now generated from the description.** Spec 9.2 row 2 says the simulation spawns from the description, and 6.1 that worlds stop including the rover once `sim.launch.py` spawns it from `robot_description`. Spawning through ROS needs the launch file and `gz_ros2_control`, which are PR 3. In PR 2 the description is the source of `rover_sim/models/rover/model.sdf` (`xacro` -> URDF -> `gz sdf -p` -> post-processing), so the worlds, the station, the referee and every test keep loading the rover as they do, and the worlds and mission sheets regenerate byte for byte (Task 14). What PR 3 spawns must be this post-processed model (or `gen_rover.build_sdf`'s output), not Gazebo's own conversion of `/robot_description`: that conversion is the same sdformat parser without `postprocess`, and on the default rover it writes the colours as rgb / 0.8 clipped at 1 (the chassis 1, 0.6875, 0.1875: the camera images change), names the LED's visual `base_link_fixed_joint_lump__led_visual_4` (the referee sets `rover::base_link::led_visual`), and keeps the `<frame>`s and the infinite wheel stops (the critic's raw `gz sdf -p` run, 2026-10-10). PR 3 either spawns the generated file or moves those fixes into the description and the wrapper, and points the equivalence test at what it spawns. `rover_sim` installs only `launch/` today; `urdf/` and `config/` are read from the source tree in PR 2 (`gen_rover.py`, `rover_config.py`), and PR 3's `sim.launch.py` installs them.
2. **What goes where** (spec 5): a physical fact of the rover goes into `rover_description/config/rover.yaml`; how the simulation models the rover and the world goes into `rover_sim/config/sim.yaml`. The table below sorts every field of `gen_model.Params`, `DriveParams` and `TrackParams` and every number written inline in `gen_model.py`.
3. **The PI speed loop's gains (`kp`, `ki`) and its speed filter go into `sim.yaml`, not `rover.yaml`.** They tune the motor controller the simulation stands in for against the simulation's own motor model (design 6.9: a turn reaches 90 % within 60 ms). The real motor controller is not chosen; its gains will be tuned on the hardware and live with its driver (PR 3 and later), so these numbers are not facts of the rover. The motor itself (voltage, kt, ke, resistance, gear, efficiency, current limit, rotor inertia, free current, output friction) and the driveline (stiffness, damping, backlash) are physical and go into `rover.yaml`, which the simulation's motor model reads.
4. **Sensor frames are links on fixed joints, which the conversion merges into their parents.** The description has the frames `imu`, `gnss`, `camera` and `led` for ROS (robot_state_publisher publishes them); `gz sdf -p` merges them into `base_link` and `camera_tilt_link`, moving the sensors that `rover_sim.urdf.xacro` attaches to them (`<gazebo reference="imu">`) with their poses, which is the frozen model's layout: no link needs `preserveFixedJoint`. `tools/gen_rover.py` post-processes the converter's output, each step named in its docstring and checked by the equivalence test: it drops the `<frame>` elements the merge adds, gives the shapes back their names (`<link>_fixed_joint_lump__<name>_visual` -> `<name>_visual`), restores the URDF colours (the converter writes rgb / 0.8), gives links their model-frame poses and joints none, restores the URDF's order of links and joints, writes the wheels' stops as +-1e16 instead of infinity, writes plugin numbers with 9 significant digits, drops the default ODE limits the converter puts into joints a `<gazebo>` block names (anything else there is an error), and points the tread mesh at `model://rover/meshes`. URDF needs no closed loop: the differential stays the RockerDifferential spring plugin.
5. **The equivalence test** (`rover_sim/tests/test_equivalence.py`) freezes ten models that `gen_model.py` writes at 9208047 (`rover_sim/tests/fixtures/rover_model`): the default rover, byte for byte today's tracked `model.sdf`, and nine variants that cover every switch and number the tests change (DiffDrive, dust, the cues off, tyre compliance, a variant with every test knob, one and no surface rows). `sdf_equivalence.differences` compares two models number by number (relative 1e-9; poses by rotation matrix; an element only one side has counts when it is not SDF's default), with its own tests that it catches a changed number, a missing parameter and a changed plugin order. A second test drives the frozen and the generated rover for 3 s and requires the same base_link trace, step for step, bit for bit (the dry run: identical for the physical rover, the one with tyre compliance and DiffDrive's).
6. **Variants for the tests.** `rover_config.Variant` is a rover with some numbers of `rover.yaml` or `sim.yaml` changed (`Variant().replace(contact={"dig": False}, motor={"current_limit": 10.0})`; a misspelt key is an error); `gen_rover.build_sdf(variant)` builds it (cached per process, about 0.1 s each). `simulate.physical(**sections)` and `simulate.diffdrive(**sections)` keep their roles with sections instead of `DriveParams` keywords. Outside `test_rover_model` (Decision 15) every assertion of the suite stays as it is; only how a test names a number changes (`P.wheel_radius` -> `ROVER.wheel.radius`).
7. **The ground surfaces** the drivetrain uses where a world has no ground map come from `urc/terrains.py` as before: `gen_rover.py` writes every catalogue type's traction row into `rover_sim/config/surfaces.yaml` (tracked, generated, "do not edit"; a test checks it is fresh) and the xacro takes the rows of the default and object surfaces. The plugin is not changed.
8. **The tread tyre** (`wheel.glb`) moves to `rover_description/meshes` with a generator of its own (`gen_wheel_mesh.py`, numpy and PyYAML only: the description depends on no other package). It writes the same geometry and materials (the dry run: the binary chunk and every JSON field equal) and names itself as the glTF generator, so the file's bytes change in that string only. `gen_rover.py` copies it into `rover_sim/models/rover/meshes`, which stays tracked like the rest of the generated rover model.
9. **rover_description's tests run as a plain unittest under CTest** (`add_test(... python -m unittest ...)`, as `rover_control` runs `test_viz.py`), not `ament_add_pytest_test`: in this pixi environment pytest 9 refuses ROS's `launch_testing` pytest plugin (`PluginValidationError ... pytest_pycollect_makemodule`). The team's linter block (`ament_lint_auto`) stays, so the package's Python and CMake pass `ament_flake8`, `ament_pep257`, `ament_lint_cmake` and `ament_xmllint`; `pixi run colcon-test` now includes `rover_description`.
10. **Comments in `urc/sdf.py`, `urc/terrains.py`, `urc/textures.py` and `urc/world.py` still name `gen_model`.** These four modules never read the rover's numbers (they mention `gen_model` only in comments), so they have nothing to read through `rover_config`. `urc/media.py` names every generated texture and mesh after the CRC of its generator's module and of the `urc` modules it imports; changing a byte of those files renames the media every world references, and the worlds would no longer regenerate byte for byte. `urc/judge.py` and `urc/missions/proving_ground.py`, which must change, are outside every media CRC (no media generator imports them; the dry run regenerated all worlds byte for byte). `rover_sim/README.md` says what replaced `gen_model.py`. Corrected in review (2026-10-10): of the four, only `urc/textures.py` (a generator's module) and `urc/sdf.py` (which `urc/farfield.py` imports) are inside a media CRC; `urc/terrains.py` and `urc/world.py` are outside every one, and the review fixes updated their comments with the worlds regenerating byte for byte.
11. **The joint monitor.** The rover model has no `JointMonitor` (the lander and the props use it); the rover's joint states come from Gazebo's `JointStatePublisher` on `/model/rover/joint_states`, which the wrapper keeps, with the ground-truth `OdometryPublisher` and the camera head's two `JointPositionController`s.
12. **The description's wheel joints carry the motor's limits** (effort 35.6 N m = current limit x kt x gear x efficiency, velocity 20 rad/s = the top speed), what RViz and the driver should see. The wrapper sets the simulation's own: under the physical drivetrain 1000 N m and 32 rad/s (DART never clamps the torque, the velocity limit is never a hidden brake), under DiffDrive 30 N m and the top speed, exactly as today.
13. **The `ros2_control` block** is in the description by default (`ros2_control:=true`, plugin `rover_control/RoverHardwareStub`, which PR 3 adds; nothing loads it in PR 2). The wrapper turns it off until PR 3 brings `gz_ros2_control`.
14. **robot_state_publisher's warning** "The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia" stays: it is harmless, and a massless root link above `base_link` would make the converter rename the simulated rover's links.
15. **test_gen_model.py becomes test_rover_model.py** (a rename git detects, so the history follows): its 44 tests keep their classes and names and test `gen_rover.py`. Their assertions stay but where what they test moved: the three inertia tests check the macros of `inertials.xacro` (`test_box` also keeps `urc.sdf.box_inertia`'s), `test_tread_mesh` checks `gen_wheel_mesh.py`, `test_surface_rows_come_from_the_catalogue` checks every catalogue type (`gen_rover` writes them all) and that the model has the sand and manmade rows, and `test_dust_emitters` compares `particle_scatter_ratio` as the float32 it is in SDF. Two tests are added (the surfaces and the mesh copy are fresh). `test_foundations.Viewers.test_gen_model_finds_the_viewer_names` becomes `test_the_viewer_names_live_in_viewers`. New: `test_equivalence` (11), `test_rover_config` (8), and `rover_description`'s CTest program (10 tests) and linters.

## Where every field of gen_model.py goes (spec 5)

`rover.yaml` is `rover_description/config/rover.yaml`, `sim.yaml` is `rover_sim/config/sim.yaml`; a key `a.b` is `b` under `a`. Every value stays as it is (the equivalence test checks the result); every comment and source label moves with its number.

### `Params`

| Field | Value | Goes to | Why |
|---|---|---|---|
| `wheel_radius`, `wheel_width` | 0.15, 0.10 m | `rover.yaml` `wheel.radius`, `wheel.width` | geometry |
| `pivot_y`, `pivot_z` | 0.40, 0.35 m | `rover.yaml` `rocker.pivot_y`, `rocker.pivot_z` | geometry |
| `wheel_dx`, `wheel_dz` | 0.45, -0.20 m | `rover.yaml` `wheel.dx`, `wheel.dz` | geometry |
| `chassis_size`, `chassis_z` | (0.8, 0.5, 0.2), 0.40 m | `rover.yaml` `chassis.size`, `chassis.z` | geometry |
| `chassis_mass`, `rocker_mass`, `wheel_mass` | 30, 3, 2.5 kg | `rover.yaml` `chassis.mass`, `rocker.mass`, `wheel.mass` | masses (the inertias are computed from the shapes in `urdf/inertials.xacro`) |
| `rocker_limit`, `rocker_damping` | 0.5 rad, 0.5 N m s/rad | `rover.yaml` `rocker.limit`, `rocker.damping` | joint limit and damping |
| `diff_stiffness`, `diff_damping` | 5000, 100 | `rover.yaml` `differential.stiffness`, `differential.damping` | the differential's spring and damper |
| `wheel_effort` | 30 N m | `sim.yaml` `diffdrive.wheel_effort` | used only by the DiffDrive A/B rover as its servos' torque limit; the motor's peak torque is computed from `rover.yaml` `motor` (35.6 N m) |
| `wheel_speed` | 20 rad/s | `rover.yaml` `wheel.top_speed` | the top speed (user decision 2026-10-07); the drivetrain's `max_speed`, the description's wheel velocity limit, DiffDrive's |
| `mu_longitudinal`, `mu_lateral` | 1.0, 0.5 | `sim.yaml` `diffdrive.mu_longitudinal`, `diffdrive.mu_lateral` | DiffDrive's tyre model |
| `imu_rate`, `gyro_noise`, `accel_noise` | 100 Hz, 0.002, 0.02 | `rover.yaml` `imu.rate`, `imu.gyro_noise`, `imu.accel_noise` | the sensor's rate and its noise spec |
| (IMU pose: the chassis centre) | (0, 0, 0.40) | `rover.yaml` `imu.xyz` | a mount of its own (the frame `imu`) |
| `arm_inset`, `arm_thickness`, `arm_height` | 0.08, 0.04, 0.05 m | `rover.yaml` `rocker.arm_inset`, `.arm_thickness`, `.arm_height` | the arms' shape (collision and visual) |
| `tread_tyre` | True | `sim.yaml` `tread_tyre` | a visual switch; the mesh itself is the description's |
| `gnss_xyz`, `gnss_rate`, `gnss_noise_horizontal`, `gnss_noise_vertical` | | `rover.yaml` `gnss.xyz`, `.rate`, `.noise_horizontal`, `.noise_vertical` | sensor mount, rate, noise spec |
| `camera_xyz`, `camera_rate`, `camera_size`, `camera_hfov`, `camera_clip` | | `rover.yaml` `camera.xyz`, `.rate`, `.size`, `.hfov`, `.clip` | the camera (`clip` is the depth range) |
| `camera_far` | 80 km | `sim.yaml` `camera_far` | the RGB image's far clip, a rendering choice (it sees the far field, D14) |
| `camera_pitch` | 0.12 rad | `rover.yaml` `head.home_tilt` | where the head starts and the station centres it |
| `camera_pan_limit`, `camera_tilt_limits`, `camera_head_speed`, `camera_head_effort` | | `rover.yaml` `head.pan_limit`, `.tilt_limits`, `.speed`, `.effort` | the head's joints |
| `camera_pan_mass`, `camera_tilt_mass`, `camera_pan_height` | 0.02, 0.02 kg, 0.05 m | `rover.yaml` `head.pan_mass`, `.tilt_mass`, `.pan_height` | the head |
| `led_xyz` | (-0.41, 0, 0.47) | `rover.yaml` `led.xyz` | the LED's mount (frame `led`) |
| `drive` | | the `DriveParams` table | |
| `tire_compliance` | False | `sim.yaml` `tyre_compliance.enabled` | a modelling switch (phase 2) |
| `tire_radial`, `tire_axial`, `tire_travel` | (60000, 170), (40000, 40), 0.03 | `rover.yaml` `tyre.radial`, `tyre.axial` (`stiffness`, `damping`), `tyre.travel` | the tyre's compliance |
| `hub_mass` | 0.1 kg | `sim.yaml` `tyre_compliance.hub_mass` | the tyre model's hub, a modelling device |

### `DriveParams`

| Field | Goes to | Why |
|---|---|---|
| `mode` | `sim.yaml` `drivetrain.mode` | which model of the drive |
| `cmd_timeout`, `cmd_timeout_clock`, `track_multiplier`, `accel` | `sim.yaml` `drivetrain.*` | the drivetrain's twist input and ramp; PR 3 moves them into the drive controller's configuration |
| `voltage`, `resistance`, `kt`, `ke`, `gear`, `efficiency`, `rotor_inertia`, `free_current`, `output_friction`, `current_limit` | `rover.yaml` `motor.*` | the motors (physical; spec 5 lists voltage, kt, ke, resistance, gear, efficiency and the current limit) |
| `driveline_stiffness`, `driveline_damping`, `backlash` | `rover.yaml` `driveline.stiffness`, `.damping`, `.backlash` | the shaft and gearbox between motor and wheel |
| `kp`, `ki`, `speed_filter` | `sim.yaml` `drivetrain.kp`, `.ki`, `.speed_filter` | the stand-in controller's tuning (Decision 3); the speed filter is also the contact rule's |
| `substeps` | `sim.yaml` `drivetrain.substeps` | the motor model's integration |
| `v_stribeck`, `v_align`, `perp_ratio`, `stick_perp_ratio`, `mu_noise`, `mu_noise_length`, `rr_w0`, `dig`, `dig_heal_length`, `default_surface`, `object_surface` | `sim.yaml` `contact.*` | the contact model, slip compliance, friction noise, rolling resistance, dig-in |
| `dig_sink`, `dig_sink_gain`, `dig_sink_max`, `dig_sink_tau` | `sim.yaml` `dig_sink.enabled`, `.gain`, `.max`, `.tau` | a visual cue |
| `default_surface`/`object_surface` rows (`surface_rows()`) | `sim.yaml` names the keys; the rows come from `urc/terrains.py` into `rover_sim/config/surfaces.yaml` | Decision 7 |
| `wheel_effort`, `wheel_velocity` | `sim.yaml` `drivetrain.wheel_effort`, `.wheel_velocity` | the wheel joints' limits in Gazebo under the physical drivetrain (Decision 12) |
| `odom_rate`, `state_rate` | `sim.yaml` `drivetrain.odom_rate`, `.state_rate` | the plugin's publishing rates |
| `dust` | `sim.yaml` `dust.enabled` | a switch |
| `dust_rate` | `sim.yaml` `dust.rate` | |
| `dust_speed_gain`, `dust_slip_gain`, `dust_max`, `dust_min_speed` | `sim.yaml` `dust.rule.speed_gain`, `.slip_gain`, `.max_rate`, `.min_speed` | the plugin's `<dust_rule>` |
| `dust_box`, `dust_particle`, `dust_lifetime`, `dust_speed`, `dust_alpha`, `dust_pitch`, `dust_scatter_ratio` | `sim.yaml` `dust.box`, `.particle`, `.lifetime`, `.speed`, `.alpha`, `.pitch`, `.scatter_ratio` | the emitters |
| `ruts`, `tracks` | `sim.yaml` `ruts.enabled`, `ruts.tracks` | a visual cue |

### `TrackParams`

Every field (`spacing`, `chunk`, `max_segments`, `berm_base`, `berm_gain`, `berm_max`, `berm_min`, `opacity_base`, `opacity_dig`, `floor_offset`, `pit_dig`, `smear_slip`, `frame_budget`, `stress`, `report`) goes to `sim.yaml` `ruts.tracks` under its own name: the ruts are visual. The `<tracks>` group also gets `rover.yaml` `wheel.width`, as before.

### Names and numbers written inline in `gen_model.py`

| In gen_model.py | Goes to |
|---|---|
| `CMD_VEL_TOPIC`, `ODOM_TOPIC`, `TF_TOPIC`, `JOINT_STATE_TOPIC`, `GROUND_TRUTH_TOPIC`, `IMU_TOPIC`, `NAVSAT_TOPIC`, `CAMERA_TOPIC`, `LED_TOPIC`, `DRIVETRAIN_TOPIC`, `DUST_TOPIC`, `HEAD_TOPIC` | `sim.yaml` `topics.*` (the rover's Gazebo interface); Python reads them as `rover_config.<NAME>` |
| `PAN_JOINT`, `TILT_JOINT`, `LED_VISUAL`, `WHEEL_MESH`, `DUST_SPRITE`, `SIDES`, `ENDS` | `rover_config.py` (names of the description's joints and the model's files) |
| `METERS_PER_DEGREE` (111 320) | `sim.yaml` `meters_per_degree` (Gazebo's NavSat noise is in degrees) |
| `CHASSIS_COLOR`, `ROCKER_COLOR`, `TIRE_COLOR`, `TREAD_LIGHT`; `TREAD_DARK` (= tyre), `TREAD_MARK` (= chassis); the antenna, camera, lens and dark LED colours | `rover.yaml` `looks.colors.chassis`, `.frame`, `.tyre`, `.tread`, `.antenna`, `.camera`, `.lens`, `.led_off` |
| mast radius 0.015 m, camera mast top 0.01 m below the pan bearing, antenna 0.06 x 0.02 m, turntable 0.03 x 0.02 m, yoke arms 0.03 x 0.008 m at y 0.07, pan_height + 0.02 tall, yoke base 0.03 x 0.148 x 0.008 m at z 0.01, camera box 0.04 x 0.12 x 0.04 m, lens 0.014 x 0.012 m at (0.024, 0.03, 0) | `rover.yaml` `looks.*` |
| LED box 0.02 x 0.16 x 0.06 m | `rover.yaml` `led.size` |
| the pan link's inertia box (0.06, 0.15, 0.06); the tilt link's is the camera box | `rover.yaml` `head.pan_box`; `looks.camera_box` |
| the head joints' damping 0.01 | `rover.yaml` `head.damping` |
| the head controllers' `p_gain` 10 | `sim.yaml` `head_p_gain` |
| the ground-truth OdometryPublisher's 50 Hz | `sim.yaml` `ground_truth_rate` |
| DiffDrive's `odom_publish_frequency` 50 | `sim.yaml` `drivetrain.odom_rate` (50 Hz: one number for both drives now) |
| the tyres' isotropic mu 1.0 under the physical drivetrain | `sim.yaml` `drivetrain.tyre_mu` |
| DiffDrive's `fdir1` (0, 0, 1) | `rover_sim.urdf.xacro` (the axle in the collision frame, not a number to tune) |
| the dust emitter's fixed settings (`emitting` false, `rate` 0, `scale_rate` 1, the white `diffuse`, the sprite's URI) and its placement (box / 4 behind the tyre's rear edge, `wheel.dx + wheel.radius + dust.box / 4`; its centre box / 2 above the ground, `wheel.dz - wheel.radius + dust.box / 2`; pitched `dust.pitch` up, yaw pi: facing back) | `rover_sim.urdf.xacro` (what the emitter is: idle until the drivetrain sets its rate, at the ground behind the rear wheel; not numbers to tune) |
| the LED's emissive 0 0 0 1 (off: no glow) | `rover_sim.urdf.xacro` (`<gazebo reference="led">`) |
| the pan joint's start 0; the lens' rotation (0, pi / 2, 0); the rockers' effort and velocity limits -1 (none) | `rover_sim.urdf.xacro` (`head_controller`); `rover.urdf.xacro` (a cylinder turned onto x; a passive pivot) |
| the hubs' inertia 1e-4 | `sim.yaml` `tyre_compliance.hub_inertia` |
| the wheel joints' stops +-1e16 | `sim.yaml` `continuous_stops` (the description's wheel joints are continuous) |
| `urc.sdf.inertial`'s floor of 1e-6 on every moment | dropped: no moment of the rover is near it (the smallest, the tilt link's iyy, is 5.3e-6) |
| frame names (`odom`, `base_link`, `world`, `gnss`, `camera`), the OdometryPublisher's 3 dimensions | `rover_sim.urdf.xacro` |
| `build_sdf(Params)`, `main()` | `rover_sim/tools/gen_rover.py` `build_sdf(Variant)`, `main()` (`pixi run sim-model`) |
| `surface_rows()`, `write_dust_sprite()` | `gen_rover.surface_rows()` (every catalogue type, into `config/surfaces.yaml`), `gen_rover.write_dust_sprite()` (alpha `sim.yaml` `dust.alpha`) |
| `wheel_parts()`, `write_wheel_mesh()` | `rover_description/meshes/gen_wheel_mesh.py` (Decision 8) |
| `wheel_inertia()`, `rocker_inertia()`, the chassis' and head's box inertias | `rover_description/urdf/inertials.xacro` (`wheel_inertial`, `rocker_inertial`, `box_inertial`) |
| `VIEWER_NAMES` and the module `__getattr__` that forwarded them to `viewers.py` | gone: the station and the tests import `viewers` |

## File structure

Created:
- `rover_description/config/rover.yaml`: every physical number of the rover.
- `rover_description/urdf/rover.urdf.xacro`: the description; `urdf/inertials.xacro`: the shapes' inertias; `urdf/ros2_control.xacro`: the `ros2_control` block.
- `rover_description/meshes/wheel.glb`, `meshes/gen_wheel_mesh.py`: the tread tyre and its generator.
- `rover_description/launch/view.launch.py`, `rviz/view.rviz`: the RViz view.
- `rover_description/test/test_description.py`: the description's tests (CTest).
- `rover_sim/config/sim.yaml`: how the simulation models the rover; `config/surfaces.yaml`: the drivetrain's ground surfaces (generated, tracked).
- `rover_sim/urdf/rover_sim.urdf.xacro`: the description plus Gazebo's parts.
- `rover_sim/rover_config.py`: the simulation's one reader of `rover.yaml` and `sim.yaml`, the rover's Gazebo names, `Variant`.
- `rover_sim/tools/gen_rover.py`: `pixi run sim-model`.
- `rover_sim/tests/sdf_equivalence.py`, `tests/test_equivalence.py`, `tests/test_rover_config.py`, `tests/fixtures/rover_model/` (ten frozen models and a README).

Moved: `rover_sim/tests/test_gen_model.py` -> `rover_sim/tests/test_rover_model.py`.

Removed: `rover_sim/gen_model.py`.

Modified: `pixi.toml`, `pixi.lock`, `README.md`; `rover_description/CMakeLists.txt`, `package.xml`, `README.md`; `rover_sim/package.xml`, `README.md`, `viewers.py`, `referee.py`, `bridge.yaml`, `station/drive.py`, `station/link.py`, `station/server.py`, `station/static/station.js` (one message), `urc/judge.py`, `urc/missions/proving_ground.py`, `worlds/rover_test.sdf` (a comment), `models/rover/model.config`, `plugins/rover_drivetrain.cpp`, `plugins/rover_drivetrain.hh`, `plugins/rover_tracks.hh`, `plugins/tests/test_rover_drivetrain.cpp` (comments only), `tools/realism_report.py`, `tests/simulate.py`, `tests/test_drivetrain.py`, `tests/test_fly_camera.py`, `tests/test_foundations.py`, `tests/test_render.py`, `tests/test_rover_sim.py`, `tests/test_station.py`, `tests/test_urc_missions.py`, `tests/test_urc_sim.py`, `tests/test_urc_terrain.py`; `docs/sim/manual.md`, `docs/sim/design-notes.md`, `docs/sim/gazebo-lessons.md`.

Regenerated by `pixi run sim-model` and committed: `rover_sim/models/rover/model.sdf` (the same rover in the converter's layout), `rover_sim/models/rover/meshes/wheel.glb` (Decision 8), `rover_sim/models/{chase_camera,eye_camera,fly_camera}/model.config` (the generator's name).

Unchanged on purpose: the plugins' code, the world generator and every `urc/` module but `judge.py` and `missions/proving_ground.py` (Decision 10), `rover_control` (its `config.hpp` keeps the placeholder geometry until PR 3 reads `rover.yaml`), `rover_sim/launch/gazebo.launch.py`, `.github/workflows/ci.yml` (it runs `pixi run test`, which now includes `rover_description`'s tests).

---

## Part A: rover_description

### Task 1: The starting point and the baseline

**Files:** none in SRC; `$T/list_tests.py`, `$T/worlds_sha.sh`, `$T/tests_before.txt`, `$T/worlds_before.sha`, `$T/baseline.log`.

- [ ] **Step 1: The branch**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain && git rev-parse --abbrev-ref HEAD \
  && git merge-base --is-ancestor 9208047 HEAD && echo "has import/simulation" && git log --oneline 9208047..HEAD
```

Expected: no status lines, `pr2/rover-description`, `has import/simulation`, and two commits: this plan's (`Plan PR 2: the rover description, ...`) and its critic's fixes (`Plan PR 2: critic fixes`). The branch was fast-forwarded to `import/simulation` (9208047, PR 1 with the no-data-release fix) when this plan was written. If `import/simulation` has moved since (`git rev-parse import/simulation` is not 9208047), stop and ask the user: the frozen models of Task 6 must come from the `gen_model.py` the branch has.

- [ ] **Step 2: Build, generate the worlds and render the maps (GPU, about 3 minutes)**

```bash
T=/private/tmp/claude-502/rover-pr2; mkdir -p "$T"
cd /Users/alarion239/Desktop/hurc_ws/src && GZ_IP=127.0.0.1 GZ_PARTITION=pr2_maps_$$ /Users/alarion239/.pixi/bin/pixi run sim-maps > "$T/maps.log" 2>&1; echo "exit $?"; tail -3 "$T/maps.log"; ls rover_sim/worlds
```

Expected: `exit 0`; `rover_sim/worlds` lists `rover_test.sdf` and, for each of `proving_ground`, `urc_astrobiology`, `urc_autonomy`, `urc_delivery`, `urc_equipment_servicing`, its `.sdf`, `.json`, `_map.jpg` and `_map.json` (the Autonomy world needs `data/dem/route_area_lidar_0p5m.tif`, which is linked here; without it the world is skipped and its tests skip, as PR 1 describes).

- [ ] **Step 3: List the tests and hash the generated worlds**

```bash
T=/private/tmp/claude-502/rover-pr2
cat > "$T/list_tests.py" <<'EOF'
import unittest


def walk(suite):
    for test in suite:
        yield from walk(test) if isinstance(test, unittest.TestSuite) else [test]


for test in walk(unittest.defaultTestLoader.discover(".", pattern="test_*.py")):
    print(test.id())
EOF
cat > "$T/worlds_sha.sh" <<'EOF'
#!/bin/sh
# The generated worlds, their mission sheets and their models, hashed (not the rendered maps). $1: the repository.
cd "$1/rover_sim" && {
  find worlds -maxdepth 1 -type f \( -name 'urc_*' -o -name 'proving_ground.*' \) ! -name '*_map.*'
  find models -type f -path 'models/urc_*'
} | LC_ALL=C sort | xargs shasum -a 256
EOF
chmod +x "$T/worlds_sha.sh"
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && GZ_IP=127.0.0.1 GZ_PARTITION=pr2_list_$$ \
  /Users/alarion239/.pixi/bin/pixi run python "$T/list_tests.py" 2>/dev/null | sort > "$T/tests_before.txt"
wc -l < "$T/tests_before.txt"; grep -c '^test_gen_model\.' "$T/tests_before.txt"
"$T/worlds_sha.sh" /Users/alarion239/Desktop/hurc_ws/src > "$T/worlds_before.sha"; wc -l < "$T/worlds_before.sha"
```

Expected: `525` (the critic counted 525 on this branch, 2026-10-10; a lower count elsewhere, such as 510, predates PR 1's review fixes: PR 1's plan, amended 2026-10-09, has 525), `44`, and about 958 hashed files (five worlds, their sheets and the models and media they reference).

- [ ] **Step 4: The baseline suite (about 15 minutes, in the background)**

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src && GZ_IP=127.0.0.1 GZ_PARTITION=pr2_base_$$ /Users/alarion239/.pixi/bin/pixi run test > "$T/baseline.log" 2>&1; echo "exit $?" >> "$T/baseline.log"
```

Run it with `run_in_background` and wait. Then:

```bash
T=/private/tmp/claude-502/rover-pr2
grep -E "tests passed|^Summary" "$T/baseline.log"; grep -E "^Ran |^OK|^FAILED|^exit" "$T/baseline.log"
```

Expected (the build's summary first, then the tests', the packages in any order):
```
Summary: 3 packages finished [...]
100% tests passed, 0 tests failed out of 57
100% tests passed, 0 tests failed out of 3
Summary: 2 packages finished [...]
Ran 525 tests in ...s
OK (skipped=9, expected failures=2)
exit 0
```

The 9 skips are the opt-in performance and slow tests, the 2 expected failures the slow-turn judder and dust in depth, as in PR 1. Write the numbers down: Task 15 compares with them. If a test fails here, the machine differs from PR 1's run (the user's other workload can slow wall-clock-sensitive tests): run that test alone (`cd rover_sim/tests && GZ_IP=127.0.0.1 GZ_PARTITION=pr2_one_$$ pixi run python -m unittest -v <test id>`); a test that fails alone too is reported to the user before going on.

### Task 2: pixi: xacro and joint_state_publisher_gui

**Files:**
- Modify: `pixi.toml` (`[dependencies]`)
- Modify: `pixi.lock` (written by pixi)

The description is built with xacro, and the view launch file starts joint_state_publisher_gui; neither is in the environment (`ros-jazzy-desktop` brings robot_state_publisher and rviz2, not these). Resolving with them adds exactly three packages per platform, nothing else (dry run on this manifest): `ros2-xacro 2.1.1`, `ros2-joint-state-publisher 2.4.3` and `ros2-joint-state-publisher-gui 2.4.3` for osx-arm64, linux-64 and win-64.

- [ ] **Step 1: Add the two dependencies**

In `pixi.toml`, replace

```toml
ros-dev-tools = ">=1.0.3,<2"
```

with

```toml
ros-dev-tools = ">=1.0.3,<2"
# rover_description: xacro builds the description from rover.yaml; the view launch file's joint sliders
ros2-xacro = ">=2.1.1,<3"
ros2-joint-state-publisher-gui = ">=2.4.3,<3"
```

- [ ] **Step 2: Resolve and install (a few minutes)**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi install 2>&1 | tail -2
git diff --stat pixi.lock
git diff pixi.lock | grep -E '^\+\s+- conda:' | sed -E 's#.*/(osx-arm64|linux-64|win-64)/#\1 #'
```

Expected: the environment installs; `pixi.lock` gains lines only, and the new environment entries are, in some order:
```
linux-64 ros2-joint-state-publisher-2.4.3-np2py312h2ed9cc7_21.conda
linux-64 ros2-joint-state-publisher-gui-2.4.3-np2py312h2ed9cc7_21.conda
linux-64 ros2-xacro-2.1.1-np2py312h2ed9cc7_21.conda
osx-arm64 ros2-joint-state-publisher-2.4.3-np2py312haf4906a_21.conda
osx-arm64 ros2-joint-state-publisher-gui-2.4.3-np2py312haf4906a_21.conda
osx-arm64 ros2-xacro-2.1.1-np2py312haf4906a_21.conda
win-64 ros2-joint-state-publisher-2.4.3-np2py312hf80f32c_21.conda
win-64 ros2-joint-state-publisher-gui-2.4.3-np2py312hf80f32c_21.conda
win-64 ros2-xacro-2.1.1-np2py312hf80f32c_21.conda
```
If other packages change (a newer build on the channel), check that none of the simulation's pinned packages (gz-sim8, gz-rendering8, gz-physics, dartsim, ogre-next, python, numpy) moves; if one does, stop and tell the user. Such builds exist: the channel has `ros2-xacro-2.1.1-np2py312h3d3361c_25` (it is in this machine's rattler cache, 2026-10-09), which needs `ros2-distro-mutex 0.19.0 jazzy_25`, while the lock holds `ros2-distro-mutex 0.16.0 jazzy_21`; the dry lock kept the locked mutex and took the `_21` builds listed above. If it moves `ros2-distro-mutex` instead (every ROS package would change), stop and tell the user rather than accepting the new lock.

- [ ] **Step 3: The tools are there**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run python -c "import xacro; print(xacro.__file__)" \
  && /Users/alarion239/.pixi/bin/pixi run which xacro check_urdf && /Users/alarion239/.pixi/bin/pixi run ros2 pkg executables joint_state_publisher_gui
```

Expected: a path under `src/.pixi/envs/default/lib/python3.12/site-packages/xacro/`, the paths of `xacro` and `check_urdf` in the environment's `bin`, and `joint_state_publisher_gui joint_state_publisher_gui`.

- [ ] **Step 4: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add pixi.toml pixi.lock && git commit -q -F - <<'EOF'
PR 2: pixi: xacro and joint_state_publisher_gui for the rover's description

rover_description builds its URDF with xacro from config/rover.yaml, and its view launch file gives every joint a
slider. Resolving adds ros2-xacro, ros2-joint-state-publisher and ros2-joint-state-publisher-gui on osx-arm64,
linux-64 and win-64, nothing else.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 3: rover.yaml and the description

**Files:**
- Create: `rover_description/config/rover.yaml`, `rover_description/urdf/inertials.xacro`, `rover_description/urdf/rover.urdf.xacro`, `rover_description/urdf/ros2_control.xacro`
- Create: `rover_description/test/test_description.py`
- Modify: `rover_description/CMakeLists.txt`, `rover_description/package.xml`, `pixi.toml` (`colcon-test`)

- [ ] **Step 1: Write the failing test**

Create `rover_description/test/test_description.py`:

```python
"""The rover's description: what xacro makes of urdf/rover.urdf.xacro and config/rover.yaml."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / 'urdf' / 'rover.urdf.xacro'
ROVER_YAML = PACKAGE / 'config' / 'rover.yaml'
ROVER = yaml.safe_load(ROVER_YAML.read_text())
WHEELS = ('wheel_fl', 'wheel_rl', 'wheel_fr', 'wheel_rr')


def expand(**args):
    """Return the description expanded with these xacro arguments, as text."""
    doc = xacro.process_file(str(URDF), mappings={k: str(v) for k, v in args.items()})
    return doc.toprettyxml(indent='  ')


def robot(**args):
    """Return the expanded description's <robot> element."""
    return ET.fromstring(expand(**args))


def numbers(text):
    """Return the numbers of a URDF attribute such as xyz or size."""
    return [float(v) for v in text.split()]


class Description(unittest.TestCase):

    def test_check_urdf_accepts_it(self):
        if shutil.which('check_urdf') is None:
            self.skipTest('check_urdf is not installed (liburdfdom-tools)')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'rover.urdf'
            path.write_text(expand())
            result = subprocess.run(['check_urdf', str(path)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Successfully Parsed XML', result.stdout)
        self.assertIn('root Link: base_link', result.stdout)

    def test_joint_tree(self):
        joints = {j.get('name'): (j.get('type'), j.find('parent').get('link'),
                                  j.find('child').get('link'))
                  for j in robot().findall('joint')}
        self.assertEqual(joints, {
            'imu_joint': ('fixed', 'base_link', 'imu'),
            'gnss_joint': ('fixed', 'base_link', 'gnss'),
            'led_joint': ('fixed', 'base_link', 'led'),
            'camera_pan_joint': ('revolute', 'base_link', 'camera_pan_link'),
            'camera_tilt_joint': ('revolute', 'camera_pan_link', 'camera_tilt_link'),
            'camera_joint': ('fixed', 'camera_tilt_link', 'camera'),
            'rocker_left_joint': ('revolute', 'base_link', 'rocker_left'),
            'wheel_fl_joint': ('continuous', 'rocker_left', 'wheel_fl'),
            'wheel_rl_joint': ('continuous', 'rocker_left', 'wheel_rl'),
            'rocker_right_joint': ('revolute', 'base_link', 'rocker_right'),
            'wheel_fr_joint': ('continuous', 'rocker_right', 'wheel_fr'),
            'wheel_rr_joint': ('continuous', 'rocker_right', 'wheel_rr'),
        })

    def test_wheels_sit_where_rover_yaml_says(self):
        rocker, wheel = ROVER['rocker'], ROVER['wheel']
        joints = {j.get('name'): j for j in robot().findall('joint')}
        for side, sign in (('left', 1), ('right', -1)):
            pivot = numbers(joints[f'rocker_{side}_joint'].find('origin').get('xyz'))
            self.assertEqual(pivot, [0.0, sign * rocker['pivot_y'], rocker['pivot_z']])
            for end, ahead in (('f', 1), ('r', -1)):
                offset = numbers(joints[f'wheel_{end}{side[0]}_joint'].find('origin').get('xyz'))
                self.assertEqual(offset, [ahead * wheel['dx'], 0.0, wheel['dz']])
        self.assertAlmostEqual(rocker['pivot_z'] + wheel['dz'], wheel['radius'])  # on the ground

    def test_a_changed_rover_yaml_changes_the_rover(self):
        changed = yaml.safe_load(ROVER_YAML.read_text())
        changed['wheel']['radius'] = 0.2
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'rover.yaml'
            path.write_text(yaml.safe_dump(changed))
            link = robot(rover_yaml=path).find("link[@name='wheel_fl']")
        self.assertEqual(float(link.find('collision/geometry/cylinder').get('radius')), 0.2)
        self.assertEqual(numbers(link.find('visual/geometry/mesh').get('scale')),
                         [0.2, ROVER['wheel']['width'], 0.2])

    def test_inertias_come_from_the_shapes(self):
        description = robot()
        links = {link.get('name'): link for link in description.findall('link')}
        chassis, wheel, head = ROVER['chassis'], ROVER['wheel'], ROVER['head']
        x, y, z = chassis['size']
        inertia = links['base_link'].find('inertial/inertia')
        self.assertAlmostEqual(float(inertia.get('ixx')), chassis['mass'] * (y * y + z * z) / 12)
        self.assertAlmostEqual(float(inertia.get('izz')), chassis['mass'] * (x * x + y * y) / 12)
        iyy = float(links['wheel_fl'].find('inertial/inertia').get('iyy'))
        self.assertAlmostEqual(iyy, wheel['mass'] * wheel['radius'] ** 2 / 2)
        total = sum(float(m.get('value')) for m in description.iter('mass'))
        self.assertAlmostEqual(total, chassis['mass'] + 2 * ROVER['rocker']['mass'] +
                               4 * wheel['mass'] + head['pan_mass'] + head['tilt_mass'])

    def test_it_names_no_simulator(self):
        description = robot()
        self.assertIsNone(description.find('.//gazebo'))
        hardware = set(description.findall('ros2_control/hardware/plugin'))
        self.assertEqual([p for p in description.iter('plugin') if p not in hardware], [])

    def test_ros2_control_block(self):
        block = robot().find('ros2_control')
        self.assertEqual(block.get('type'), 'system')
        self.assertEqual(block.findtext('hardware/plugin'), 'rover_control/RoverHardwareStub')
        interfaces = {j.get('name'): ([c.get('name') for c in j.findall('command_interface')],
                                      [s.get('name') for s in j.findall('state_interface')])
                      for j in block.findall('joint')}
        driven = (['velocity'], ['position', 'velocity', 'effort'])
        sensed = ([], ['position', 'velocity'])
        self.assertEqual(interfaces, {
            **{f'{w}_joint': driven for w in WHEELS},
            **{j: sensed for j in ('rocker_left_joint', 'rocker_right_joint', 'camera_pan_joint',
                                   'camera_tilt_joint')}})
        other = robot(ros2_control_plugin='rover_sim/RoverSimSystem')
        self.assertEqual(other.findtext('ros2_control/hardware/plugin'),
                         'rover_sim/RoverSimSystem')
        self.assertIsNone(robot(ros2_control='false').find('ros2_control'))


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Run it to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_description/test && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_description 2>&1 | tail -3
```

Expected: `Ran 1 test`, `FAILED (errors=1)`: the module cannot be imported, `FileNotFoundError: ... rover_description/config/rover.yaml`.

- [ ] **Step 3: Write rover.yaml**

Create `rover_description/config/rover.yaml`:

```yaml
# The rover's physical numbers, in one place: geometry, masses, joints, the rocker differential, the motors and
# their drivelines, the tyres, the sensors and the camera head, and how the rover looks. urdf/rover.urdf.xacro
# builds the description from them; the simulation (rover_sim) reads them too. How the simulation models the
# world (contact, dig-in, dust, its motor controller's tuning) is in rover_sim/config/sim.yaml instead.
#
# Units SI: m, kg, s, rad, N, V, A. Frames (REP 103): x forward, y left, z up; base_link's origin is on the ground
# midway between the four wheels, with the rockers at zero. Write every number that is not a count with a decimal
# point: the tools keep the type (20.0 is a float, 20 an int).
# Sources: (A) assumed, (M) measured or tuned in the simulation, (R) from a reference in docs/research, with the
# design that chose it. The geometry is the swerve design's placeholder (rover_control's config.hpp, kRover) until
# the mechanical team has real numbers. To change the rover, edit this file and run `pixi run sim-model`
# (rover_description/README.md).

chassis:
  size: [0.80, 0.50, 0.20]          # [m] the body, a box (A: rover model design 2026-10-05)
  z: 0.40                           # [m] the box centre above the ground (A)
  mass: 30.0                        # [kg] everything the chassis carries, the camera head's real mass included (A)

rocker:                             # left and right, each carrying a front and a rear wheel
  pivot_y: 0.40                     # [m] the pivots at (0, +-pivot_y, pivot_z) (A: config.hpp)
  pivot_z: 0.35                     # [m] (A: config.hpp)
  mass: 3.0                         # [kg] each; its arms below are visual and collision only (A)
  limit: 0.5                        # [rad] each way (A)
  damping: 0.5                      # [N m s/rad] the pivot's (A)
  arm_inset: 0.08                   # [m] the arms' plane, from the wheel's centre plane toward the chassis (A)
  arm_thickness: 0.04               # [m] (A)
  arm_height: 0.05                  # [m] (A)

differential:                       # a torsion spring and damper on q_left + q_right, keeping the rockers opposite
  stiffness: 5000.0                 # [N m/rad] static error about 1e-4 rad (A: rover model design)
  damping: 100.0                    # [N m s/rad] (A)

wheel:
  radius: 0.15                      # [m] (A: config.hpp)
  width: 0.10                       # [m] (A)
  dx: 0.45                          # [m] wheel centres at pivot + (+-dx, 0, dz): front +, rear - (A: config.hpp)
  dz: -0.20                         # [m] (A: config.hpp)
  mass: 2.5                         # [kg] (A)
  top_speed: 20.0                   # [rad/s] 3 m/s, the rover's top speed (user decision 2026-10-07)

# The drive motor of each wheel, the same four. Placeholders until the drivetrain is chosen (realism design D20, Q7:
# the prototype's validated set). The names are those of the simulation's motor model (RoverDrivetrain <motor>).
motor:
  # [V] twice the Husky A200's 24 V (R [18]): a 21.6 rad/s free speed at the wheel, 3.2 m/s, for the 3 m/s top speed
  # (user decision 2026-10-07); the torque per amp, and with it every climb and stall, is unchanged.
  voltage: 48.0
  resistance: 0.46                  # [ohm] (R [18])
  kt: 0.0445                        # [N m/A] (R [18])
  ke: 0.0445                        # [V s/rad] (R [18])
  gear: 50.0                        # (M: prototype; datasheet alternative 51, R [22])
  efficiency: 0.8                   # (M: prototype; datasheet alternative 0.7)
  rotor_inertia: 1.2e-5             # [kg m^2] at the motor, 0.03 at the wheel (A)
  free_current: 1.0                 # [A] no-load current (A)
  output_friction: 0.05             # [N m s/rad] (A)
  current_limit: 20.0               # [A] 35.6 N m at the wheel (M: prototype)

driveline:                          # between each gearbox and its wheel (RoverDrivetrain <driveline>)
  stiffness: 1500.0                 # [N m/rad] (A: 12 mm x 0.1 m steel shaft)
  damping: 2.0                      # [N m s/rad] (A)
  backlash: 0.026                   # [rad] 1.5 deg (A: IMS 0.8-2.5 deg [22])

# The tyres' compliance, which the simulation's tyre model uses when it is on (rover_sim/config/sim.yaml
# tyre_compliance; realism design 6.7, D18, phase 2). Placeholders until the wheel type is known (Q8).
tyre:
  radial: {stiffness: 60000.0, damping: 170.0}   # [N/m], [N s/m] (M: prototype)
  axial: {stiffness: 40000.0, damping: 40.0}     # [N/m], [N s/m] (M: prototype)
  travel: 0.03                                   # [m] each way (M: prototype)

imu:                                # frame imu, aligned with base_link
  xyz: [0.0, 0.0, 0.40]             # [m] at the chassis box centre (A)
  rate: 100.0                       # [Hz] (A: rover model design)
  gyro_noise: 0.002                 # [rad/s] stddev (A)
  accel_noise: 0.02                 # [m/s^2] stddev (A)

gnss:                               # the antenna on a short mast (frame gnss)
  xyz: [-0.25, 0.0, 0.85]           # [m] (A)
  rate: 10.0                        # [Hz] (A)
  noise_horizontal: 0.5             # [m] stddev (A)
  noise_vertical: 1.0               # [m] stddev (A)

# The front RGB-D camera on a pan-tilt head on a mast (frame camera). xyz is the camera and its tilt axis; the pan
# axis runs vertically through it. 1280x720: at 640x480 a 20 cm ArUco face read only to ~2.5 m (docs/sim/manual.md,
# Known limitations); the depth image has the same size (one RGB-D sensor).
camera:
  xyz: [0.35, 0.0, 0.85]            # [m] (A)
  rate: 15.0                        # [Hz] (A)
  size: [1280, 720]                 # [px] (M: realism design 7)
  hfov: 1.5                         # [rad] (A)
  clip: [0.1, 40.0]                 # [m] the depth range (A: realism design 7, D14)

# The pan-tilt head. Angles from straight ahead and level; pan positive to the left (about +z), tilt positive down
# (about +y).
head:
  home_tilt: 0.12                   # [rad] down: where the head starts and the station centres it (A)
  pan_limit: 2.8                    # [rad] each way (A)
  tilt_limits: [-0.6, 1.2]          # [rad] (up, down) (A)
  speed: 2.0                        # [rad/s] (A)
  effort: 5.0                       # [N m] (A)
  damping: 0.01                     # [N m s/rad] each joint's (A)
  # Token masses: the head's real mass is part of the chassis'. Skid-steer turning is sensitive to the centre of
  # mass: 0.4 kg on the head moves it 3 mm forward and made a 4 s turn in place wander 0.24 m instead of 0 (M).
  pan_mass: 0.02                    # [kg]
  tilt_mass: 0.02                   # [kg]
  pan_height: 0.05                  # [m] the pan bearing below the tilt axis (A)
  pan_box: [0.06, 0.15, 0.06]       # [m] the box the pan link's inertia is computed for (A)

led:                                # the status LED on the back (URC 2027 rule 1.e.ii; frame led)
  xyz: [-0.41, 0.0, 0.47]           # [m] (A)
  size: [0.02, 0.16, 0.06]          # [m] (A)

# Parts drawn without mass or collision of their own (their mass is the chassis'), and the colours (RGBA, 0-1).
looks:
  mast_radius: 0.015                # [m] the GNSS and camera masts
  camera_mast_gap: 0.01             # [m] the camera mast's top below the pan bearing
  antenna: {radius: 0.06, length: 0.02}
  turntable: {radius: 0.03, length: 0.02}
  yoke: {size: [0.03, 0.008], y: 0.07, extra_height: 0.02}   # the tilt yoke's arms: pan_height + extra_height tall
  yoke_base: {size: [0.03, 0.148, 0.008], z: 0.01}
  camera_box: [0.04, 0.12, 0.04]    # [m] the camera's housing, also the box its link's inertia is computed for
  lens: {radius: 0.014, length: 0.012, xyz: [0.024, 0.03, 0.0]}
  colors:
    chassis: [0.85, 0.55, 0.15, 1.0]     # ochre; also the tread tyre's marked spoke
    frame: [0.35, 0.35, 0.38, 1.0]       # rockers, masts and the head
    tyre: [0.1, 0.1, 0.1, 1.0]           # the plain tyre, and the tread tyre's dark
    tread: [0.30, 0.29, 0.26, 1.0]       # the tread tyre's bars, hub and spokes
    antenna: [0.9, 0.9, 0.9, 1.0]
    camera: [0.1, 0.1, 0.1, 1.0]
    lens: [0.05, 0.08, 0.12, 1.0]
    led_off: [0.15, 0.15, 0.15, 1.0]     # the LED's colour while it is off
```

- [ ] **Step 4: Write the description**

Create `rover_description/urdf/inertials.xacro`:

```xml
<?xml version="1.0"?>
<!-- Inertias of the rover's simple shapes, as <inertial> elements: principal moments about the centre of mass at
     xyz, the products of inertia zero. rover.urdf.xacro uses them; rover_sim's tests check the formulas. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="inertial" params="mass ixx iyy izz xyz:='0 0 0'">
    <inertial>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia ixx="${ixx}" ixy="0" ixz="0" iyy="${iyy}" iyz="0" izz="${izz}"/>
    </inertial>
  </xacro:macro>

  <!-- A solid box of size (x, y, z). -->
  <xacro:macro name="box_inertial" params="mass size xyz:='0 0 0'">
    <xacro:inertial mass="${mass}" xyz="${xyz}"
                    ixx="${mass * (size[1] * size[1] + size[2] * size[2]) / 12}"
                    iyy="${mass * (size[0] * size[0] + size[2] * size[2]) / 12}"
                    izz="${mass * (size[0] * size[0] + size[1] * size[1]) / 12}"/>
  </xacro:macro>

  <!-- A solid cylinder whose axis is y (a wheel on its axle). -->
  <xacro:macro name="wheel_inertial" params="mass radius width">
    <xacro:property name="across" value="${mass * (3 * radius * radius + width * width) / 12}"/>
    <xacro:inertial mass="${mass}" ixx="${across}" iyy="${mass * radius * radius / 2}" izz="${across}"/>
  </xacro:macro>

  <!-- A rocker: two thin rods of mass / 2 from the pivot to (+-dx, 0, dz), about their joint centre of mass
       (0, 0, dz / 2); xyz moves that centre sideways to the arms' plane. -->
  <xacro:macro name="rocker_inertial" params="mass dx dz xyz">
    <xacro:inertial mass="${mass}" xyz="${xyz}"
                    ixx="${mass * dz * dz / 12}"
                    iyy="${mass * (dx * dx + dz * dz) / 12 + mass * dx * dx / 4}"
                    izz="${mass * dx * dx / 3}"/>
  </xacro:macro>
</robot>
```

Create `rover_description/urdf/rover.urdf.xacro`:

```xml
<?xml version="1.0"?>
<!-- The rover: links, joints, visuals, collisions and sensor frames, every number from config/rover.yaml.
     Frames follow REP 103: x forward, y left, z up; base_link's origin is on the ground midway between the wheels
     with the rockers at zero. Expand it with `xacro rover.urdf.xacro`; view it with
     `ros2 launch rover_description view.launch.py`. It names no simulator: rover_sim/urdf/rover_sim.urdf.xacro adds
     Gazebo's parts.

     Arguments:
       rover_yaml           the numbers (default config/rover.yaml beside this folder)
       ros2_control         true: add the ros2_control block (ros2_control.xacro)
       ros2_control_plugin  its hardware interface (default rover_control's real-hardware stub, which PR 3 adds)
     Properties a file that includes this one may set first (xacro:property default): wheel_effort_limit and
     wheel_velocity_limit (the wheel joints' limits; default the motor's peak torque at the wheel and the top speed),
     tread_tyre (the wheels' visual: the tread mesh, or False: a plain cylinder) and wheel_mount (the macro that
     hangs a wheel from its rocker; rover_sim's tyre model hangs it through sprung hubs). -->
<robot name="rover" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="rover_yaml" default="../config/rover.yaml"/>
  <xacro:arg name="ros2_control" default="true"/>
  <xacro:arg name="ros2_control_plugin" default="rover_control/RoverHardwareStub"/>
  <xacro:include filename="inertials.xacro"/>

  <xacro:property name="rover" value="${xacro.load_yaml(xacro.arg('rover_yaml'))}"/>
  <xacro:property name="chassis" value="${rover.chassis}"/>
  <xacro:property name="rocker" value="${rover.rocker}"/>
  <xacro:property name="wheel" value="${rover.wheel}"/>
  <xacro:property name="motor" value="${rover.motor}"/>
  <xacro:property name="head" value="${rover.head}"/>
  <xacro:property name="looks" value="${rover.looks}"/>

  <xacro:property name="wheel_effort_limit" default="${motor.current_limit * motor.kt * motor.gear * motor.efficiency}"/>
  <xacro:property name="wheel_velocity_limit" default="${wheel.top_speed}"/>
  <xacro:property name="tread_tyre" default="${True}"/>
  <xacro:property name="wheel_mount" default="rocker_wheel_mount"/>

  <xacro:property name="chassis_top" value="${chassis.z + chassis.size[2] / 2}"/>
  <!-- A list of numbers as URDF text. -->
  <xacro:macro name="color" params="name rgba">
    <material name="${name}"><color rgba="${' '.join(str(v) for v in rgba)}"/></material>
  </xacro:macro>
  <!-- A thin pole from the chassis top up to top_z. -->
  <xacro:macro name="mast" params="name x y top_z">
    <xacro:property name="height" value="${top_z - chassis_top}"/>
    <visual name="${name}">
      <origin xyz="${x} ${y} ${top_z - height / 2}" rpy="0 0 0"/>
      <geometry><cylinder radius="${looks.mast_radius}" length="${height}"/></geometry>
      <material name="frame"/>
    </visual>
  </xacro:macro>
  <!-- A sensor or part frame fixed to its link (the conversion to SDF merges it into that link). -->
  <xacro:macro name="frame" params="name parent xyz">
    <link name="${name}"/>
    <joint name="${name}_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="${name}"/>
      <origin xyz="${' '.join(str(v) for v in xyz)}" rpy="0 0 0"/>
    </joint>
  </xacro:macro>

  <xacro:color name="chassis" rgba="${looks.colors.chassis}"/>
  <xacro:color name="frame" rgba="${looks.colors.frame}"/>
  <xacro:color name="tyre" rgba="${looks.colors.tyre}"/>
  <xacro:color name="antenna" rgba="${looks.colors.antenna}"/>
  <xacro:color name="camera" rgba="${looks.colors.camera}"/>
  <xacro:color name="lens" rgba="${looks.colors.lens}"/>
  <xacro:color name="led_off" rgba="${looks.colors.led_off}"/>

  <!-- The chassis, with the masts of the GNSS antenna and the camera head. -->
  <link name="base_link">
    <xacro:box_inertial mass="${chassis.mass}" size="${chassis.size}" xyz="0 0 ${chassis.z}"/>
    <collision name="chassis">
      <origin xyz="0 0 ${chassis.z}" rpy="0 0 0"/>
      <geometry><box size="${' '.join(str(v) for v in chassis.size)}"/></geometry>
    </collision>
    <visual name="chassis">
      <origin xyz="0 0 ${chassis.z}" rpy="0 0 0"/>
      <geometry><box size="${' '.join(str(v) for v in chassis.size)}"/></geometry>
      <material name="chassis"/>
    </visual>
    <xacro:mast name="gnss_mast" x="${rover.gnss.xyz[0]}" y="${rover.gnss.xyz[1]}" top_z="${rover.gnss.xyz[2]}"/>
    <visual name="gnss_antenna">
      <origin xyz="${' '.join(str(v) for v in rover.gnss.xyz)}" rpy="0 0 0"/>
      <geometry><cylinder radius="${looks.antenna.radius}" length="${looks.antenna.length}"/></geometry>
      <material name="antenna"/>
    </visual>
    <xacro:mast name="camera_mast" x="${rover.camera.xyz[0]}" y="${rover.camera.xyz[1]}"
                top_z="${rover.camera.xyz[2] - head.pan_height - looks.camera_mast_gap}"/>
  </link>
  <xacro:frame name="imu" parent="base_link" xyz="${rover.imu.xyz}"/>
  <xacro:frame name="gnss" parent="base_link" xyz="${rover.gnss.xyz}"/>
  <!-- The LED, a frame of its own; rover_sim gives its visual an emissive material. -->
  <link name="led">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="${' '.join(str(v) for v in rover.led.size)}"/></geometry>
      <material name="led_off"/>
    </visual>
  </link>
  <joint name="led_joint" type="fixed">
    <parent link="base_link"/>
    <child link="led"/>
    <origin xyz="${' '.join(str(v) for v in rover.led.xyz)}" rpy="0 0 0"/>
  </joint>

  <!-- The camera's pan-tilt head: pan about z below the camera, tilt about y through it. -->
  <link name="camera_pan_link">
    <xacro:box_inertial mass="${head.pan_mass}" size="${head.pan_box}"/>
    <visual name="turntable">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="${looks.turntable.radius}" length="${looks.turntable.length}"/></geometry>
      <material name="frame"/>
    </visual>
    <xacro:macro name="yoke_arm" params="side sign">
      <visual name="yoke_${side}">
        <origin xyz="0 ${sign * looks.yoke.y} ${head.pan_height / 2}" rpy="0 0 0"/>
        <geometry>
          <box size="${looks.yoke.size[0]} ${looks.yoke.size[1]} ${head.pan_height + looks.yoke.extra_height}"/>
        </geometry>
        <material name="frame"/>
      </visual>
    </xacro:macro>
    <xacro:yoke_arm side="left" sign="1"/>
    <xacro:yoke_arm side="right" sign="-1"/>
    <visual name="yoke_base">
      <origin xyz="0 0 ${looks.yoke_base.z}" rpy="0 0 0"/>
      <geometry><box size="${' '.join(str(v) for v in looks.yoke_base.size)}"/></geometry>
      <material name="frame"/>
    </visual>
  </link>
  <joint name="camera_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="camera_pan_link"/>
    <origin xyz="${rover.camera.xyz[0]} ${rover.camera.xyz[1]} ${rover.camera.xyz[2] - head.pan_height}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-head.pan_limit}" upper="${head.pan_limit}" effort="${head.effort}" velocity="${head.speed}"/>
    <dynamics damping="${head.damping}"/>
  </joint>
  <link name="camera_tilt_link">
    <xacro:box_inertial mass="${head.tilt_mass}" size="${looks.camera_box}"/>
    <visual name="camera">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="${' '.join(str(v) for v in looks.camera_box)}"/></geometry>
      <material name="camera"/>
    </visual>
    <visual name="lens">
      <origin xyz="${' '.join(str(v) for v in looks.lens.xyz)}" rpy="0 ${pi / 2} 0"/>
      <geometry><cylinder radius="${looks.lens.radius}" length="${looks.lens.length}"/></geometry>
      <material name="lens"/>
    </visual>
  </link>
  <joint name="camera_tilt_joint" type="revolute">
    <parent link="camera_pan_link"/>
    <child link="camera_tilt_link"/>
    <origin xyz="0 0 ${head.pan_height}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${head.tilt_limits[0]}" upper="${head.tilt_limits[1]}" effort="${head.effort}"
           velocity="${head.speed}"/>
    <dynamics damping="${head.damping}"/>
  </joint>
  <xacro:frame name="camera" parent="camera_tilt_link" xyz="${[0.0, 0.0, 0.0]}"/>

  <!-- A wheel's joint: continuous about y. -->
  <xacro:macro name="wheel_joint" params="name parent xyz">
    <joint name="${name}_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${name}"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="${wheel_effort_limit}" velocity="${wheel_velocity_limit}"/>
    </joint>
  </xacro:macro>
  <!-- The default wheel_mount: the wheel's joint straight from its rocker. -->
  <xacro:macro name="rocker_wheel_mount" params="name rocker xyz">
    <xacro:wheel_joint name="${name}" parent="${rocker}" xyz="${xyz}"/>
  </xacro:macro>

  <!-- A wheel at its rocker's front (ahead 1) or rear (-1) end. The tyre's collision is named after its wheel:
       sdformat applies a <gazebo> block's collision settings only to collisions whose names start with their
       link's (rover_sim sets the tyres' friction that way). -->
  <xacro:macro name="rover_wheel" params="name rocker ahead">
    <link name="${name}">
      <xacro:wheel_inertial mass="${wheel.mass}" radius="${wheel.radius}" width="${wheel.width}"/>
      <collision name="${name}_tire">
        <origin xyz="0 0 0" rpy="${pi / 2} 0 0"/>
        <geometry><cylinder radius="${wheel.radius}" length="${wheel.width}"/></geometry>
      </collision>
      <xacro:if value="${tread_tyre}">
        <!-- The tread tyre: unit size, scaled to the wheel, with its own materials. -->
        <visual name="tire">
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <geometry>
            <mesh filename="package://rover_description/meshes/wheel.glb"
                  scale="${wheel.radius} ${wheel.width} ${wheel.radius}"/>
          </geometry>
        </visual>
      </xacro:if>
      <xacro:unless value="${tread_tyre}">
        <visual name="tire">
          <origin xyz="0 0 0" rpy="${pi / 2} 0 0"/>
          <geometry><cylinder radius="${wheel.radius}" length="${wheel.width}"/></geometry>
          <material name="tyre"/>
        </visual>
      </xacro:unless>
    </link>
    <xacro:call macro="${wheel_mount}" name="${name}" rocker="${rocker}" xyz="${ahead * wheel.dx} 0 ${wheel.dz}"/>
  </xacro:macro>

  <!-- A rocker on its pivot, its two arms down to the wheels, and the wheels. -->
  <xacro:macro name="rover_rocker" params="side sign">
    <xacro:property name="inset" value="${-sign * rocker.arm_inset}"/>
    <xacro:property name="pitch" value="${math.atan2(-wheel.dz, wheel.dx)}"/>
    <link name="rocker_${side}">
      <xacro:rocker_inertial mass="${rocker.mass}" dx="${wheel.dx}" dz="${wheel.dz}" xyz="0 ${inset} ${wheel.dz / 2}"/>
      <xacro:macro name="arm" params="end ahead">
        <xacro:property name="arm_origin" value="${ahead * wheel.dx / 2} ${inset} ${wheel.dz / 2}"/>
        <xacro:property name="arm_size"
                        value="${math.hypot(wheel.dx, wheel.dz)} ${rocker.arm_thickness} ${rocker.arm_height}"/>
        <collision name="${end}_arm">
          <origin xyz="${arm_origin}" rpy="0 ${ahead * pitch} 0"/>
          <geometry><box size="${arm_size}"/></geometry>
        </collision>
        <visual name="${end}_arm">
          <origin xyz="${arm_origin}" rpy="0 ${ahead * pitch} 0"/>
          <geometry><box size="${arm_size}"/></geometry>
          <material name="frame"/>
        </visual>
      </xacro:macro>
      <xacro:arm end="front" ahead="1"/>
      <xacro:arm end="rear" ahead="-1"/>
    </link>
    <joint name="rocker_${side}_joint" type="revolute">
      <parent link="base_link"/>
      <child link="rocker_${side}"/>
      <origin xyz="0 ${sign * rocker.pivot_y} ${rocker.pivot_z}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <!-- A passive pivot: no effort or velocity limit (URDF needs the values; -1 is none to SDF and Gazebo). -->
      <limit lower="${-rocker.limit}" upper="${rocker.limit}" effort="-1" velocity="-1"/>
      <dynamics damping="${rocker.damping}"/>
    </joint>
    <xacro:rover_wheel name="wheel_f${side[0]}" rocker="rocker_${side}" ahead="1"/>
    <xacro:rover_wheel name="wheel_r${side[0]}" rocker="rocker_${side}" ahead="-1"/>
  </xacro:macro>
  <xacro:rover_rocker side="left" sign="1"/>
  <xacro:rover_rocker side="right" sign="-1"/>

  <xacro:if value="$(arg ros2_control)">
    <xacro:include filename="ros2_control.xacro"/>
    <xacro:rover_ros2_control plugin="$(arg ros2_control_plugin)"/>
  </xacro:if>
</robot>
```

Create `rover_description/urdf/ros2_control.xacro`:

```xml
<?xml version="1.0"?>
<!-- The rover's ros2_control block (design spec 2026-10-09, section 5): each wheel takes a velocity command and
     reports its position, velocity and effort (the motor current); the rockers and the camera head report position
     and velocity only. `plugin` is the hardware interface: rover_control's real-hardware stub by default
     (rover.urdf.xacro's ros2_control_plugin argument); the simulation sets its own (PR 3). Nothing loads this block
     yet: the controllers come with PR 3. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="rover_ros2_control" params="plugin">
    <xacro:macro name="driven_joint" params="name">
      <joint name="${name}">
        <command_interface name="velocity"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    </xacro:macro>
    <xacro:macro name="sensed_joint" params="name">
      <joint name="${name}">
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </xacro:macro>
    <ros2_control name="rover" type="system">
      <hardware>
        <plugin>${plugin}</plugin>
      </hardware>
      <xacro:driven_joint name="wheel_fl_joint"/>
      <xacro:driven_joint name="wheel_rl_joint"/>
      <xacro:driven_joint name="wheel_fr_joint"/>
      <xacro:driven_joint name="wheel_rr_joint"/>
      <xacro:sensed_joint name="rocker_left_joint"/>
      <xacro:sensed_joint name="rocker_right_joint"/>
      <xacro:sensed_joint name="camera_pan_joint"/>
      <xacro:sensed_joint name="camera_tilt_joint"/>
    </ros2_control>
  </xacro:macro>
</robot>
```

Three things in it are on purpose. The tyre's collision is named `<wheel>_tire` while its visual is `tire`: sdformat applies a `<gazebo reference="wheel_fl">` block's collision settings only to collisions whose names start with the link's name, and the wrapper sets the tyres' friction that way. The rockers' limits carry `effort="-1" velocity="-1"`: URDF needs the attributes, and -1 is "none" in SDF, as the frozen model has it. `wheel_mount` is called through `xacro:call`, so that the simulation's tyre model can hang a wheel through sprung hubs without the description knowing.

- [ ] **Step 5: Run the test to see it pass**

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src/rover_description/test && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_description 2>&1 | tail -3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run xacro rover_description/urdf/rover.urdf.xacro > "$T/rover.urdf" \
  && /Users/alarion239/.pixi/bin/pixi run check_urdf "$T/rover.urdf" | head -3; rm -f "$T/rover.urdf"
```

Expected: `Ran 7 tests`, `OK`; then `robot name is: rover`, `---------- Successfully Parsed XML ---------------`, `root Link: base_link has 6 child(ren)`.

- [ ] **Step 6: Build and lint the package with colcon**

Replace `rover_description/CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# The rover's numbers, its description, the tread tyre and the RViz view
# (README.md).
install(DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # xacro and check_urdf on the description, its ros2_control block, the
  # wheel mesh and the view launch file (unittest, as rover_control's and
  # rover_sim's Python tests).
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_description
    COMMAND "${Python3_EXECUTABLE}" -m unittest -v test_description
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test"
  )
endif()

ament_package()
```

Replace `rover_description/package.xml` with:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_description</name>
  <version>0.0.0</version>
  <description>The rover's description: every physical number in config/rover.yaml, the URDF built from it with xacro (links, joints, masses, sensor frames, the ros2_control block), the tread tyre mesh, and an RViz view.</description>
  <maintainer email="44044350+BeckettOBrien@users.noreply.github.com">beckett</maintainer>
  <maintainer email="amb30239@gmail.com">Alexander Belotserkovtsev</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>liburdfdom-tools</test_depend>
  <test_depend>python3-numpy</test_depend>
  <test_depend>python3-yaml</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

In `pixi.toml`, replace

```toml
# The C++ tests: rover_control's (GoogleTest, rover_state, viz.py) and rover_sim's three plugin test programs.
colcon-test = { cmd = "colcon test --packages-select rover_control rover_sim --return-code-on-test-failure --event-handlers console_cohesion+", cwd = "..", depends-on = ["build"] }
```

with

```toml
# The colcon tests: rover_description's (xacro and check_urdf, the team's linters), rover_control's (GoogleTest,
# rover_state, viz.py) and rover_sim's three plugin test programs.
colcon-test = { cmd = "colcon test --packages-select rover_description rover_control rover_sim --return-code-on-test-failure --event-handlers console_cohesion+", cwd = "..", depends-on = ["build"] }
```

Then:

```bash
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon build --packages-select rover_description 2>&1 | tail -1 \
  && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon test --packages-select rover_description --event-handlers console_cohesion+ 2>&1 | grep -E "Test #|tests passed"
ls install/rover_description/share/rover_description
```

Expected: `Summary: 1 package finished [...]`; five tests passed (`test_description`, `flake8`, `lint_cmake`, `pep257`, `xmllint`), `100% tests passed, 0 tests failed out of 5`; the share folder lists `cmake config environment hook local_setup.* package.dsv package.ps1 package.sh package.xml package.zsh urdf` (and the like). A linter failure names the file and line: fix the file, not the linter's configuration.

- [ ] **Step 7: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_description pixi.toml && git commit -q -F - <<'EOF'
PR 2: rover_description: every physical number in rover.yaml, the URDF built from it

config/rover.yaml holds the rover's geometry, masses, joint limits and damping, the differential's spring and damper,
the motors and drivelines, the tyres, the sensors and the camera head, each with its unit and source; the values are
gen_model.py's. urdf/rover.urdf.xacro builds the links, joints, inertias (urdf/inertials.xacro), collisions, visuals
and the frames imu, gnss, camera and led from it, and names no simulator. urdf/ros2_control.xacro is the
ros2_control block: the four wheels take a velocity command and report position, velocity and effort, the rockers
and the camera head report position and velocity; the hardware plugin is an argument, by default rover_control's
stub, which PR 3 adds. test/test_description.py runs xacro and check_urdf under colcon test, beside the team's
linters; pixi run colcon-test includes the package.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 4: The tread tyre moves to rover_description

**Files:**
- Create: `rover_description/meshes/gen_wheel_mesh.py`, `rover_description/meshes/wheel.glb` (generated)
- Modify: `rover_description/test/test_description.py`, `rover_description/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

In `rover_description/test/test_description.py`, replace

```python
"""The rover's description: what xacro makes of urdf/rover.urdf.xacro and config/rover.yaml."""
from pathlib import Path
```

with

```python
"""The rover's description: what xacro makes of urdf/rover.urdf.xacro and config/rover.yaml."""
import importlib.util
from pathlib import Path
```

replace

```python
def numbers(text):
    """Return the numbers of a URDF attribute such as xyz or size."""
    return [float(v) for v in text.split()]
```

with

```python
def numbers(text):
    """Return the numbers of a URDF attribute such as xyz or size."""
    return [float(v) for v in text.split()]


def load_module(path, name):
    """Import a Python file that is not on the path (a script, a launch file)."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
```

and replace

```python
        self.assertIsNone(robot(ros2_control='false').find('ros2_control'))
```

with

```python
        self.assertIsNone(robot(ros2_control='false').find('ros2_control'))

    def test_wheel_mesh_is_what_its_generator_writes(self):
        generator = load_module(PACKAGE / 'meshes' / 'gen_wheel_mesh.py', 'gen_wheel_mesh')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'wheel.glb'
            generator.write_glb(path, generator.wheel_parts(*generator.colors()))
            self.assertEqual(path.read_bytes(), (PACKAGE / 'meshes' / 'wheel.glb').read_bytes())
```

- [ ] **Step 2: Run it to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_description/test && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_description 2>&1 | tail -3
```

Expected: `FAILED (errors=1)`: `FileNotFoundError` for `meshes/gen_wheel_mesh.py`.

- [ ] **Step 3: Write the generator and the mesh**

Create `rover_description/meshes/gen_wheel_mesh.py`:

```python
#!/usr/bin/env python3
"""
Write the tread tyre's mesh, meshes/wheel.glb.

The tyre is a cylinder of radius 1 and width 1 round the y axis (the wheel link's axle; the
description scales it to the wheel), flat-shaded: a dark tread with light bars across it, and on
each face a light hub disc and spokes out to the rim, light but for the one at angle 0 (along the
link's x), which has the chassis' colour on both faces, so that a wheel spinning while the rover
stands still shows. Same outline as a plain cylinder: what changes is paint, not shape. The
colours are config/rover.yaml's looks.colors (tyre, tread, chassis).

    python3 rover_description/meshes/gen_wheel_mesh.py

rewrites meshes/wheel.glb; run it after changing those colours, then `pixi run sim-model`, which
copies the mesh into the simulation's rover model. Needs numpy and PyYAML.
"""
import json
import math
from pathlib import Path
import struct

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROVER_YAML = HERE.parent / 'config' / 'rover.yaml'
WHEEL_GLB = HERE / 'wheel.glb'
ROUGHNESS = 0.9
GENERATOR = 'rover_description/meshes/gen_wheel_mesh.py'


def colors(rover_yaml=ROVER_YAML):
    """Return the tread's (dark, light, mark) RGB colours from rover.yaml."""
    looks = yaml.safe_load(Path(rover_yaml).read_text())['looks']['colors']
    return tuple(tuple(looks[name][:3]) for name in ('tyre', 'tread', 'chassis'))


def wheel_parts(dark, light, mark, segments=48, bars=16, spokes=4, spoke_deg=15.0, hub=0.35):
    """Return the tyre as [(vertices, triangles, normals, colour)], one part per colour."""
    buckets = {colour: ([], []) for colour in (dark, light, mark)}  # triangles, their normals

    def tri(colour, a, b, c, n):
        a, b, c, n = (np.asarray(v, float) for v in (a, b, c, n))
        if np.dot(np.cross(b - a, c - a), n) < 0:  # counter-clockwise seen from outside
            b, c = c, b
        buckets[colour][0].append((a, b, c))
        buckets[colour][1].append((n, n, n))

    step = 360.0 / segments
    for k in range(segments):
        a0, a1 = math.radians(k * step), math.radians((k + 1) * step)
        mid = (k + 0.5) * step
        tread = light if k % (segments // bars) == 0 else dark
        n = (math.cos(math.radians(mid)), 0.0, math.sin(math.radians(mid)))
        rim = [(math.cos(a), y, math.sin(a))
               for a, y in ((a0, -0.5), (a1, -0.5), (a1, 0.5), (a0, 0.5))]
        tri(tread, rim[0], rim[1], rim[2], n)
        tri(tread, rim[0], rim[2], rim[3], n)
        off = min(mid % (360.0 / spokes), 360.0 / spokes - mid % (360.0 / spokes))  # from a spoke
        spoke = dark
        if off < spoke_deg / 2:
            spoke = mark if min(mid, 360.0 - mid) < spoke_deg / 2 else light
        for y in (-0.5, 0.5):
            face = (0.0, math.copysign(1.0, y), 0.0)

            def ring(r, a, y=y):
                return (r * math.cos(a), y, r * math.sin(a))

            tri(light, (0.0, y, 0.0), ring(hub, a0), ring(hub, a1), face)
            tri(spoke, ring(hub, a0), ring(1.0, a0), ring(1.0, a1), face)
            tri(spoke, ring(hub, a0), ring(1.0, a1), ring(hub, a1), face)
    parts = []
    for colour, (triangles, normals) in buckets.items():
        vertices = np.array(triangles).reshape(-1, 3)
        parts.append((vertices, np.arange(len(vertices)).reshape(-1, 3),
                      np.array(normals).reshape(-1, 3), colour))
    return parts


def write_glb(path, parts):
    """Write the parts as one binary glTF mesh, a primitive and a material per part."""
    blob = bytearray()
    views, accessors, primitives, materials = [], [], [], []

    def view(data, target):
        blob.extend(b'\0' * (-len(blob) % 4))
        views.append({'buffer': 0, 'byteOffset': len(blob), 'byteLength': len(data),
                      'target': target})
        blob.extend(data)
        return len(views) - 1

    def accessor(array, kind, component, target, bounds=False):
        entry = {'bufferView': view(array.tobytes(), target), 'componentType': component,
                 'count': int(len(array)), 'type': kind}
        if bounds:
            entry['min'] = [float(v) for v in array.min(axis=0)]
            entry['max'] = [float(v) for v in array.max(axis=0)]
        accessors.append(entry)
        return len(accessors) - 1

    for vertices, triangles, normals, colour in parts:
        attributes = {
            'POSITION': accessor(vertices.astype(np.float32), 'VEC3', 5126, 34962, bounds=True),
            'NORMAL': accessor(normals.astype(np.float32), 'VEC3', 5126, 34962)}
        indices = accessor(triangles.astype(np.uint32).reshape(-1), 'SCALAR', 5125, 34963)
        primitives.append({'attributes': attributes, 'indices': indices,
                           'material': len(materials)})
        materials.append({'pbrMetallicRoughness': {
            'baseColorFactor': [float(c) for c in colour] + [1.0], 'metallicFactor': 0.0,
            'roughnessFactor': ROUGHNESS}})
    gltf = {'asset': {'version': '2.0', 'generator': GENERATOR}, 'scene': 0,
            'scenes': [{'nodes': [0]}], 'nodes': [{'mesh': 0}],
            'meshes': [{'primitives': primitives}], 'materials': materials,
            'buffers': [{'byteLength': len(blob)}], 'bufferViews': views, 'accessors': accessors}
    text = json.dumps(gltf, separators=(',', ':')).encode()
    text += b' ' * (-len(text) % 4)
    blob.extend(b'\0' * (-len(blob) % 4))
    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(blob)))  # glTF 2
        f.write(struct.pack('<II', len(text), 0x4E4F534A) + text)  # chunk JSON
        f.write(struct.pack('<II', len(blob), 0x004E4942) + bytes(blob))  # chunk BIN


def main():
    write_glb(WHEEL_GLB, wheel_parts(*colors()))
    print(f'wrote {WHEEL_GLB}')


if __name__ == '__main__':
    main()
```

Then write the mesh and check that it is the simulation's tread tyre, but for the generator's name:

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run python rover_description/meshes/gen_wheel_mesh.py && /Users/alarion239/.pixi/bin/pixi run python - <<'EOF'
import json
import struct


def chunks(path):
    blob = open(path, "rb").read()
    length = struct.unpack("<I", blob[12:16])[0]
    return json.loads(blob[20:20 + length]), blob[20 + length + 8:]


old, old_bin = chunks("rover_sim/models/rover/meshes/wheel.glb")
new, new_bin = chunks("rover_description/meshes/wheel.glb")
print("binary chunk equal:", old_bin == new_bin)
print("generator:", old["asset"].pop("generator"), "->", new["asset"].pop("generator"))
print("JSON equal but the generator:", old == new)
EOF
```

Expected:
```
wrote /Users/alarion239/Desktop/hurc_ws/src/rover_description/meshes/wheel.glb
binary chunk equal: True
generator: sim/urc/meshes.py -> rover_description/meshes/gen_wheel_mesh.py
JSON equal but the generator: True
```

- [ ] **Step 4: Run the tests to see them pass**

Replace in `rover_description/CMakeLists.txt` the line `install(DIRECTORY config urdf` with `install(DIRECTORY config meshes urdf`. Then:

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_description/test && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_description 2>&1 | tail -3
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon build --packages-select rover_description 2>&1 | tail -1 \
  && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon test --packages-select rover_description --event-handlers console_cohesion+ 2>&1 | grep -E "tests passed"
```

Expected: `Ran 8 tests`, `OK`; `Summary: 1 package finished`; `100% tests passed, 0 tests failed out of 5`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_description && git commit -q -F - <<'EOF'
PR 2: rover_description: the tread tyre and its generator

meshes/wheel.glb is the simulation's tread tyre (a unit cylinder the description scales to the wheel), written by
meshes/gen_wheel_mesh.py from rover.yaml's colours with its own glTF writer, so the package depends on no other.
The geometry and materials are gen_model.py's, byte for byte; only the glTF generator's name differs.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 5: The view launch file and RViz

**Files:**
- Create: `rover_description/launch/view.launch.py`, `rover_description/rviz/view.rviz`
- Modify: `rover_description/test/test_description.py`, `rover_description/CMakeLists.txt`

- [ ] **Step 1: Write the failing tests**

In `rover_description/test/test_description.py`, replace

```python
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
```

with

```python
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET
```

and replace

```python
            self.assertEqual(path.read_bytes(), (PACKAGE / 'meshes' / 'wheel.glb').read_bytes())
```

with

```python
            self.assertEqual(path.read_bytes(), (PACKAGE / 'meshes' / 'wheel.glb').read_bytes())


class ViewLaunch(unittest.TestCase):

    def setUp(self):
        self.view = load_module(PACKAGE / 'launch' / 'view.launch.py', 'view_launch')

    def test_no_display_is_linux_without_x11_or_wayland(self):
        self.assertTrue(self.view.headless('linux', {}))
        self.assertFalse(self.view.headless('linux', {'DISPLAY': ':0'}))
        self.assertFalse(self.view.headless('linux', {'WAYLAND_DISPLAY': 'wayland-0'}))
        self.assertFalse(self.view.headless('darwin', {}))
        self.assertEqual(self.view.headless(), self.view.headless(sys.platform, os.environ))

    def test_it_opens_windows_only_where_there_is_a_display(self):
        from launch.actions import DeclareLaunchArgument
        from launch_ros.actions import Node
        share = mock.patch.object(self.view, 'get_package_share_directory',
                                  lambda name: str(PACKAGE))
        for no_display, windows in ((False, 'true'), (True, 'false')):
            with share, mock.patch.object(self.view, 'headless', lambda: no_display):
                entities = self.view.generate_launch_description().entities
            defaults = {e.name: e.default_value[0].text for e in entities
                        if isinstance(e, DeclareLaunchArgument)}
            self.assertEqual((defaults['gui'], defaults['rviz']), (windows, windows))
            self.assertEqual(defaults['rover_yaml'], str(ROVER_YAML))
            self.assertEqual([e.node_executable for e in entities if isinstance(e, Node)],
                             ['robot_state_publisher', 'joint_state_publisher_gui',
                              'joint_state_publisher', 'rviz2'])
```

- [ ] **Step 2: Run them to see them fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_description/test && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_description 2>&1 | tail -3
```

Expected: `FAILED (errors=2)`: `FileNotFoundError` for `launch/view.launch.py`.

- [ ] **Step 3: Write the launch file and the RViz configuration**

Create `rover_description/launch/view.launch.py`:

```python
"""
Show the rover in RViz.

robot_state_publisher publishes the expanded description (/robot_description, /tf_static, and
/tf for the joints that move), joint_state_publisher_gui gives every joint a slider, and rviz2
opens with rviz/view.rviz:

    ros2 launch rover_description view.launch.py

Arguments: gui (the sliders; false: joint_state_publisher without a window), rviz, and
rover_yaml (the numbers, default config/rover.yaml). Where there is no display (Linux without
DISPLAY or WAYLAND_DISPLAY: a CI runner, a shell over SSH) gui and rviz default to false, and the
launch says so.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def headless(platform=None, environ=None):
    """Return whether there is no display to open a window on (Linux without X11 or Wayland)."""
    platform = sys.platform if platform is None else platform
    environ = os.environ if environ is None else environ
    return platform.startswith('linux') and not (environ.get('DISPLAY') or
                                                 environ.get('WAYLAND_DISPLAY'))


def generate_launch_description():
    share = get_package_share_directory('rover_description')
    windows = 'false' if headless() else 'true'
    gui, rviz = LaunchConfiguration('gui'), LaunchConfiguration('rviz')
    description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'rover.urdf.xacro'),
                 ' rover_yaml:=', LaunchConfiguration('rover_yaml')]),
        value_type=str)
    actions = [
        DeclareLaunchArgument('gui', default_value=windows,
                              description='joint_state_publisher_gui (true) or without a window'),
        DeclareLaunchArgument('rviz', default_value=windows, description='open rviz2'),
        DeclareLaunchArgument('rover_yaml',
                              default_value=os.path.join(share, 'config', 'rover.yaml'),
                              description="the rover's numbers"),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'view.rviz')],
             condition=IfCondition(rviz)),
    ]
    if headless():
        actions.insert(0, LogInfo(msg='no display (DISPLAY and WAYLAND_DISPLAY are unset): '
                                      'gui and rviz default to false'))
    return LaunchDescription(actions)
```

Create `rover_description/rviz/view.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Value: true
      Reference Frame: <Fixed Frame>
      Plane: XY
      Plane Cell Count: 10
      Cell Size: 0.5
      Color: 160; 160; 164
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Value: true
      Description Source: Topic
      Description Topic:
        Value: /robot_description
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
      Visual Enabled: true
      Collision Enabled: false
      Alpha: 1
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Value: true
      Show Names: true
      Show Axes: true
      Show Arrows: false
      Marker Scale: 0.3
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Target Frame: <Fixed Frame>
      Distance: 2.5
      Focal Point:
        X: 0
        Y: 0
        Z: 0.35
      Pitch: 0.45
      Yaw: 0.8
      Value: Orbit (rviz)
    Saved: ~
Window Geometry:
  Height: 800
  Width: 1200
```

Replace `rover_description/CMakeLists.txt` with its final form:

```cmake
cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# The rover's numbers, its description, the tread tyre and the RViz view
# (README.md).
install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # xacro and check_urdf on the description, its ros2_control block, the
  # wheel mesh and the view launch file (unittest, as rover_control's and
  # rover_sim's Python tests).
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_description
    COMMAND "${Python3_EXECUTABLE}" -m unittest -v test_description
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test"
  )
endif()

ament_package()
```

- [ ] **Step 4: Run the tests to see them pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_description/test && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_description 2>&1 | tail -3
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon build --packages-select rover_description 2>&1 | tail -1 \
  && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon test --packages-select rover_description --event-handlers console_cohesion+ 2>&1 | grep -E "tests passed"
ls install/rover_description/share/rover_description
```

Expected: `Ran 10 tests`, `OK`; `100% tests passed, 0 tests failed out of 5`; the share folder now also has `launch`, `meshes` and `rviz`.

- [ ] **Step 5: The launch file starts and robot_state_publisher publishes /robot_description**

The windows (rviz2, the sliders) are not opened here: the agent's shell cannot look at them, and on this Mac they would open on the user's screen. The launch description's test covers which nodes start with and without a display; this step starts the launch for real with `gui:=false rviz:=false` and checks what it publishes.

```bash
T=/private/tmp/claude-502/rover-pr2
cat > "$T/probe_description.py" <<'EOF'
"""Wait up to 20 s for /robot_description (transient local) and say what arrived."""
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("probe_description")
got = []
node.create_subscription(String, "/robot_description", lambda m: got.append(m.data),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
end = time.time() + 20
while not got and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
if got:
    print(f"robot_description: {len(got[0])} characters, rover: {'<robot name=\"rover\"' in got[0]}, "
          f"ros2_control: {'<ros2_control' in got[0]}")
else:
    print("robot_description: nothing in 20 s")
rclpy.shutdown()
EOF
cat > "$T/view_check.sh" <<'EOF'
#!/bin/bash
# ros2 launch rover_description view.launch.py without windows; $1: the scripts' folder.
set -m  # job control: the launch gets its own process group and takes SIGINT
cd /Users/alarion239/Desktop/hurc_ws && source install/local_setup.bash
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch rover_description view.launch.py gui:=false rviz:=false > "$1/view.log" 2>&1 &
PID=$!
python "$1/probe_description.py"
ros2 topic list 2>/dev/null | sort | tr '\n' ' '; echo
kill -INT -- -$PID; wait $PID 2>/dev/null
sleep 1; pgrep -g $PID > /dev/null && echo "still running" || echo "stopped"
grep -E "process started|Robot initialized|process has finished|ERROR" "$1/view.log"
EOF
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run bash "$T/view_check.sh" "$T"
```

Expected (pids differ):
```
robot_description: 15402 characters, rover: True, ros2_control: True
/joint_states /parameter_events /robot_description /rosout /tf /tf_static
stopped
[INFO] [robot_state_publisher-1]: process started with pid [...]
[INFO] [joint_state_publisher-2]: process started with pid [...]
[robot_state_publisher-1] [INFO] [...] [robot_state_publisher]: Robot initialized
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid ...]
[INFO] [joint_state_publisher-2]: process has finished cleanly [pid ...]
```

(The character count includes the expanded file's absolute path in xacro's banner, so it is about 15 400. robot_state_publisher also logs the KDL root-inertia warning, Decision 14.) If `still running` is printed, stop the launch's process group yourself (`kill -TERM -- -<pid>`): only processes this step started.

- [ ] **Step 6: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_description && git commit -q -F - <<'EOF'
PR 2: rover_description: the view launch file and its RViz configuration

launch/view.launch.py starts robot_state_publisher on the expanded description, joint_state_publisher_gui (or
joint_state_publisher with gui:=false) and rviz2 with rviz/view.rviz (the model, its frames, a grid). Where there is
no display (Linux without DISPLAY or WAYLAND_DISPLAY) gui and rviz default to false and the launch says so. Its test
checks the nodes and defaults; started headless it publishes /robot_description.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

---

## Part B: the simulated rover from the description, and the equivalence test

### Task 6: Freeze gen_model.py's rovers as the test's fixtures

**Files:**
- Create: `rover_sim/tests/fixtures/rover_model/{default,diffdrive,diffdrive_square,diffdrive_dust,dust,cues_off,tyre_compliance,test_physical,same_surfaces,unknown_surface}.sdf`, `rover_sim/tests/fixtures/rover_model/README.md`
- Script (not committed): `$T/freeze_rover_fixtures.py`

`gen_model.py` still exists here, as it is at 9208047; this is the only task that runs it on purpose.

- [ ] **Step 1: Write the freezing script**

Create `$T/freeze_rover_fixtures.py`:

```python
"""Freeze gen_model.py's rover and the variants the tests build as the equivalence test's fixtures (PR 2 plan,
Task 6): rover_sim/tests/fixtures/rover_model/<name>.sdf. Run once, with the gen_model.py of import/simulation."""
import sys
from pathlib import Path

SIM = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(SIM))
import gen_model as g  # noqa: E402

D, P, T = g.DriveParams, g.Params, g.TrackParams
VARIANTS = {
    "default": P(),
    "diffdrive": P(drive=D(mode="diffdrive")),
    "diffdrive_square": P(drive=D(mode="diffdrive"), mu_lateral=1.0),
    "diffdrive_dust": P(drive=D(mode="diffdrive", dust=True)),
    "dust": P(drive=D(dust=True)),
    "cues_off": P(tread_tyre=False, drive=D(dig_sink=False, ruts=False)),
    "tyre_compliance": P(tire_compliance=True),
    "test_physical": P(drive=D(cmd_timeout_clock="sim", dig=False, mu_noise=0.0, backlash=0.0, current_limit=10.0,
                               accel=0.0, state_rate=1000.0, cmd_timeout=0.0, default_surface="sand",
                               tracks=T(report=1.0))),
    "same_surfaces": P(drive=D(default_surface="manmade")),
    "unknown_surface": P(drive=D(default_surface="lunar_dust")),
}
out = SIM / "tests" / "fixtures" / "rover_model"
out.mkdir(parents=True, exist_ok=True)
for name, params in VARIANTS.items():
    (out / f"{name}.sdf").write_text(g.build_sdf(params))
tracked = (SIM / "models" / "rover" / "model.sdf").read_text()
print("default is the tracked model:", (out / "default.sdf").read_text() == tracked)
print(" ".join(VARIANTS))
```

- [ ] **Step 2: Freeze**

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src && git diff --quiet 9208047 -- rover_sim/gen_model.py rover_sim/urc/terrains.py && echo "gen_model.py as at 9208047" \
  && /Users/alarion239/.pixi/bin/pixi run python "$T/freeze_rover_fixtures.py" rover_sim && ls rover_sim/tests/fixtures/rover_model && du -sh rover_sim/tests/fixtures/rover_model
```

Expected:
```
gen_model.py as at 9208047
default is the tracked model: True
default diffdrive diffdrive_square diffdrive_dust dust cues_off tyre_compliance test_physical same_surfaces unknown_surface
cues_off.sdf  default.sdf  diffdrive.sdf  diffdrive_dust.sdf  diffdrive_square.sdf  dust.sdf  same_surfaces.sdf  test_physical.sdf  tyre_compliance.sdf  unknown_surface.sdf
252K	rover_sim/tests/fixtures/rover_model
```

- [ ] **Step 3: Say what they are**

Create `rover_sim/tests/fixtures/rover_model/README.md`:

````markdown
# The rover as gen_model.py wrote it

These SDF files are the rover model and the variants of it that the tests build, as `rover_sim/gen_model.py` wrote them at commit 9208047 (`import/simulation`, 2026-10-09), before `rover_description` described the rover. `tests/test_equivalence.py` builds the same rovers from `rover_description/config/rover.yaml`, `rover_sim/config/sim.yaml` and `urdf/rover_sim.urdf.xacro` (`tools/gen_rover.py`) and checks that each is its file here, number for number (`tests/sdf_equivalence.py`), and that the default and two variants drive bit for bit alike in Gazebo.

| File | gen_model.py's parameters | The same rover now (`rover_config.Variant().replace(...)`) |
|---|---|---|
| `default.sdf` | `Params()`; byte for byte the tracked `models/rover/model.sdf` of 9208047 | `Variant()` |
| `diffdrive.sdf` | `Params(drive=DriveParams(mode="diffdrive"))` | `drivetrain={"mode": "diffdrive"}` |
| `diffdrive_square.sdf` | `Params(drive=DriveParams(mode="diffdrive"), mu_lateral=1.0)` | `drivetrain={"mode": "diffdrive"}, diffdrive={"mu_lateral": 1.0}` |
| `diffdrive_dust.sdf` | `Params(drive=DriveParams(mode="diffdrive", dust=True))` | `drivetrain={"mode": "diffdrive"}, dust={"enabled": True}` |
| `dust.sdf` | `Params(drive=DriveParams(dust=True))` | `dust={"enabled": True}` |
| `cues_off.sdf` | `Params(tread_tyre=False, drive=DriveParams(dig_sink=False, ruts=False))` | `tread_tyre=False, dig_sink={"enabled": False}, ruts={"enabled": False}` |
| `tyre_compliance.sdf` | `Params(tire_compliance=True)` | `tyre_compliance={"enabled": True}` |
| `test_physical.sdf` | `Params(drive=DriveParams(cmd_timeout_clock="sim", dig=False, mu_noise=0.0, backlash=0.0, current_limit=10.0, accel=0.0, state_rate=1000.0, cmd_timeout=0.0, default_surface="sand", tracks=TrackParams(report=1.0)))`: every number the tests change | `drivetrain={"cmd_timeout_clock": "sim", "accel": 0.0, "state_rate": 1000.0, "cmd_timeout": 0.0}, contact={"dig": False, "mu_noise": 0.0, "default_surface": "sand"}, driveline={"backlash": 0.0}, motor={"current_limit": 10.0}, ruts={"tracks": {"report": 1.0}}` |
| `same_surfaces.sdf` | `Params(drive=DriveParams(default_surface="manmade"))`: one surface row | `contact={"default_surface": "manmade"}` |
| `unknown_surface.sdf` | `Params(drive=DriveParams(default_surface="lunar_dust"))`: no row for a key the catalogue lacks | `contact={"default_surface": "lunar_dust"}` |

When the rover changes on purpose (real numbers from the mechanical team, a new sensor), the test fails as it should. Then write the fixtures again from the new generator in the same commit, and say in the commit why the rover changed:

```bash
cd rover_sim/tests && pixi run python -c "
import worldfiles, gen_rover, test_equivalence as t
for name, variant in t.VARIANTS.items():
    (t.FIXTURES / f'{name}.sdf').write_text(gen_rover.build_sdf(variant))"
```
````

- [ ] **Step 4: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/tests/fixtures && git commit -q -F - <<'EOF'
PR 2: rover_sim tests: the rover gen_model.py writes, frozen

tests/fixtures/rover_model holds gen_model.py's model of the rover (byte for byte the tracked model.sdf) and of the
nine variants the tests build (DiffDrive, dust, the dig-in cues off, tyre compliance, every number a test changes, one
and no surface rows), written at 9208047. The equivalence test compares the rover built from rover_description with
them; the README says what each is and how to refreeze them when the rover changes on purpose.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 7: The comparison of two models, and its tests

**Files:**
- Create: `rover_sim/tests/sdf_equivalence.py`
- Create: `rover_sim/tests/test_equivalence.py` (the comparison's tests; Tasks 9 and 10 add to it)

- [ ] **Step 1: Write the failing test**

Create `rover_sim/tests/test_equivalence.py`:

```python
#!/usr/bin/env python3
"""The rover the simulation loads is the rover gen_model.py wrote, number for number (design spec 2026-10-09,
section 5): tools/gen_rover.py's model of the description, and of each variant the tests build, against the frozen
models in fixtures/rover_model (written by gen_model.py at import/simulation, 9208047; fixtures/rover_model/README.md
lists them). SdfEquivalence checks the comparison itself; Drives runs the frozen and the generated rover side by
side in Gazebo. When the rover changes on purpose, refreeze the fixtures from gen_rover.py in the same commit and say
why (fixtures/rover_model/README.md)."""
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from sdf_equivalence import differences
from worldfiles import SIM_DIR

from urc import sdf  # noqa: E402  (worldfiles puts sim/ on the path)

FIXTURES = SIM_DIR / "tests" / "fixtures" / "rover_model"


def fixture(name):
    return (FIXTURES / f"{name}.sdf").read_text()


class SdfEquivalence(unittest.TestCase):
    """differences() finds what changes a model and nothing else."""

    @classmethod
    def setUpClass(cls):
        cls.reference = fixture("default")

    def changed(self, edit):
        root = ET.fromstring(self.reference)
        edit(root.find("model"))
        return differences(self.reference, sdf.document(root))

    def test_a_model_equals_itself(self):
        self.assertEqual(differences(self.reference, self.reference), [])

    def test_a_number_beyond_the_tolerance_differs(self):
        def heavier(model):
            mass = model.find("link[@name='wheel_fl']/inertial/mass")
            mass.text = sdf.fmt(float(mass.text) * (1 + 1e-6))
        self.assertEqual(len(self.changed(heavier)), 1)

    def test_rounding_within_the_tolerance_does_not(self):
        def rounded(model):
            ixx = model.find("link[@name='wheel_fl']/inertial/inertia/ixx")
            ixx.text = repr(float(ixx.text) * (1 + 1e-12))
        self.assertEqual(self.changed(rounded), [])

    def test_a_missing_plugin_parameter_differs(self):
        def without_ki(model):
            controller = model.find("plugin[@filename='RoverDrivetrain']/controller")
            controller.remove(controller.find("ki"))
        self.assertEqual(len(self.changed(without_ki)), 1)

    def test_the_plugins_order_counts(self):
        def swapped(model):
            plugins = model.findall("plugin")
            for plugin in plugins[2:4]:
                model.remove(plugin)
            model.extend([plugins[3], plugins[2]])
        self.assertTrue(any("plugin order" in line for line in self.changed(swapped)))

    def test_a_default_written_out_is_the_same(self):
        def explicit(model):
            axis = model.find("joint[@name='wheel_fl_joint']/axis")
            sdf.sub(sdf.sub(axis, "dynamics"), "spring_stiffness", 0.0)
        self.assertEqual(self.changed(explicit), [])

    def test_a_pose_compares_its_rotation_not_its_angles(self):
        path = "link[@name='rocker_left']/collision[@name='front_arm_collision']/pose"
        x, y, z, _, pitch, _ = (float(v) for v in ET.fromstring(self.reference).find(f"model/{path}").text.split())

        def turned(rpy):
            return lambda model: setattr(model.find(path), "text", sdf.fmt((x, y, z, *rpy)))

        self.assertEqual(self.changed(turned((np.pi, np.pi - pitch, np.pi))), [])  # the same rotation
        self.assertEqual(len(self.changed(turned((0.0, pitch + 1e-6, 0.0)))), 1)

    def test_an_element_only_one_side_has_differs(self):
        def extra(model):
            sdf.sub(model.find("link[@name='base_link']"), "self_collide", True)
        self.assertEqual(len(self.changed(extra)), 1)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_equivalence 2>&1 | tail -3
```

Expected: `FAILED (errors=1)`: `ModuleNotFoundError: No module named 'sdf_equivalence'`.

- [ ] **Step 3: Write the comparison**

Create `rover_sim/tests/sdf_equivalence.py`:

```python
"""Compare two SDF models number by number (test_rover_model.Equivalence: the generated rover against the frozen one
gen_model.py wrote, tests/fixtures/rover_model).

differences(reference, generated) lists every difference: an element or attribute that one has and the other lacks,
a number that differs beyond the tolerance (a pose by its position and its rotation matrix), different text, a
different order of links, joints or plugins. Children with the same tag pair by name where they all have one (not
plugins: those pair in order), otherwise in order; children with different tags may come in any order, as SDF reads
them. An element that one side lacks counts as SDF's default: DEFAULTS lists the values that the converter writes
where gen_model.py wrote nothing, which sdformat assumes when the element is missing."""
import math
import xml.etree.ElementTree as ET

import numpy as np

REL_TOL = 1e-9  # 9 significant digits, as gen_model.py and `gz sdf -p --precision 9` write numbers
# Numbers this close are the same: a position 1e-12 m apart, or an SDF float (particle_scatter_ratio) that
# the converter writes as its float32 value.
ABS_TOL = 1e-12
ROTATION_TOL = 1e-8  # entries of two rotation matrices: an angle to pi written with 9 significant digits
# The tail of an element's tag path -> SDF 1.11's default for it.
DEFAULTS = {
    ("axis", "dynamics", "damping"): "0",
    ("axis", "dynamics", "friction"): "0",
    ("axis", "dynamics", "spring_reference"): "0",
    ("axis", "dynamics", "spring_stiffness"): "0",
    ("axis", "limit", "effort"): "-1",
    ("axis", "limit", "velocity"): "-1",
    ("inertial", "pose"): "0 0 0 0 0 0",
    ("collision", "pose"): "0 0 0 0 0 0",
    ("visual", "pose"): "0 0 0 0 0 0",
    ("sensor", "pose"): "0 0 0 0 0 0",
    ("depth_camera", "output"): "depths",
}
ORDERED = ("link", "joint", "plugin")  # model children whose order counts (entity creation, the systems' order)


def differences(reference, generated):
    """Every difference between two SDF documents' <model>s, as lines of text (none: the same model)."""
    a, b = ET.fromstring(reference).find("model"), ET.fromstring(generated).find("model")
    out = []
    for tag in ORDERED:
        names_a = [e.get("name") for e in a.findall(tag)]
        names_b = [e.get("name") for e in b.findall(tag)]
        if names_a != names_b:
            out.append(f"model/{tag} order: {names_a} != {names_b}")
    _compare(a, b, ("model",), out)
    return out


def _numbers(text):
    try:
        return [float(v) for v in text.split()]
    except ValueError:
        return None


def _rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _close(x, y):
    return math.isclose(x, y, rel_tol=REL_TOL, abs_tol=ABS_TOL)


def _text_differs(path, ta, tb):
    ta, tb = (ta or "").strip(), (tb or "").strip()
    if ta == tb:
        return False
    na, nb = _numbers(ta), _numbers(tb)
    if na is None or nb is None or len(na) != len(nb):
        return True
    if path[-1] == "pose" and len(na) == 6:
        if not all(_close(x, y) for x, y in zip(na[:3], nb[:3])):
            return True
        return not np.allclose(_rotation(na[3:]), _rotation(nb[3:]), rtol=0.0, atol=ROTATION_TOL)
    return not all(x == y or _close(x, y) for x, y in zip(na, nb))


def _is_default(path, element):
    """Whether `element`, which one side lacks, holds only SDF defaults."""
    children = list(element)
    if children:
        return all(_is_default(path + (c.tag,), c) for c in children)
    default = next((v for k, v in DEFAULTS.items() if path[-len(k):] == k), None)
    return default is not None and not _text_differs(path, element.text, default)


def _compare(a, b, path, out):
    where = "/".join(path)
    if dict(a.attrib) != dict(b.attrib):
        out.append(f"{where}: attributes {dict(a.attrib)} != {dict(b.attrib)}")
    if _text_differs(path, a.text, b.text):
        out.append(f"{where}: {(a.text or '').strip()!r} != {(b.text or '').strip()!r}")
    for tag in dict.fromkeys([c.tag for c in a] + [c.tag for c in b]):
        ca, cb = a.findall(tag), b.findall(tag)
        if tag != "plugin" and all(c.get("name") is not None for c in ca + cb):
            pairs = {c.get("name"): [c, None] for c in ca}
            for c in cb:
                pairs.setdefault(c.get("name"), [None, None])[1] = c
            items = [(f"{tag}[{name}]", x, y) for name, (x, y) in pairs.items()]
        else:
            ca, cb = ca + [None] * (len(cb) - len(ca)), cb + [None] * (len(ca) - len(cb))
            items = [(f"{tag}[{i}]", x, y) for i, (x, y) in enumerate(zip(ca, cb))]
        for label, x, y in items:
            if x is None or y is None:
                present = x if y is None else y
                if not _is_default(path + (tag,), present):
                    side = "reference" if y is None else "generated"
                    text = ET.tostring(present, encoding="unicode").strip()[:120]
                    out.append(f"{where}/{label}: only in the {side} model ({text})")
                continue
            _compare(x, y, path + (tag,), out)
```

- [ ] **Step 4: Run the test to see it pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_equivalence 2>&1 | tail -3
```

Expected: `Ran 8 tests`, `OK`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/tests/sdf_equivalence.py rover_sim/tests/test_equivalence.py && git commit -q -F - <<'EOF'
PR 2: rover_sim tests: compare two rover models number by number

sdf_equivalence.differences lists every difference between two SDF models: a number beyond 1e-9 relative, a pose by
its position and rotation matrix, different text, an element only one has unless it holds SDF's default, the order of
links, joints and plugins. Its tests check that it finds a heavier wheel, a missing PI gain, swapped plugins and an
extra element, and not rounding, a default written out or one rotation written two ways.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 8: sim.yaml and the simulation's one reader of the rover's numbers

**Files:**
- Create: `rover_sim/config/sim.yaml`, `rover_sim/rover_config.py`
- Create: `rover_sim/tests/test_rover_config.py`

- [ ] **Step 1: Write the failing test**

Create `rover_sim/tests/test_rover_config.py`:

```python
#!/usr/bin/env python3
"""rover_config.py, the simulation's one reader of rover.yaml and sim.yaml (no Gazebo)."""
import unittest

import yaml

import worldfiles  # noqa: F401  (puts sim/ on the path)

import rover_config  # noqa: E402
from rover_config import ROVER, SIM, Tree, Variant  # noqa: E402


class Files(unittest.TestCase):
    def test_the_files_share_no_section(self):
        """Variant.replace() finds a section by its name: a name in both files would be ambiguous."""
        self.assertEqual(set(ROVER) & set(SIM), set())

    def test_numbers_keep_their_type(self):
        """Every number that is not a count is written with a decimal point, so the tools read a float."""
        self.assertIsInstance(ROVER.wheel.top_speed, float)
        self.assertIsInstance(ROVER.rocker.limit, float)
        self.assertIsInstance(SIM.ruts.tracks.chunk, int)
        self.assertEqual(ROVER.camera.size, (1280, 720))

    def test_trees_read_like_the_files(self):
        self.assertEqual(ROVER.plain(), yaml.safe_load(rover_config.ROVER_YAML.read_text()))
        self.assertEqual(SIM.plain(), yaml.safe_load(rover_config.SIM_YAML.read_text()))
        self.assertEqual(ROVER["wheel"]["radius"], ROVER.wheel.radius)
        with self.assertRaises(AttributeError):
            ROVER.wheel.no_such_number  # noqa: B018

    def test_topics_are_the_gazebo_interface(self):
        self.assertEqual(rover_config.CMD_VEL_TOPIC, "/model/rover/cmd_vel")
        self.assertEqual(rover_config.HEAD_TOPIC.format(joint=rover_config.PAN_JOINT),
                         "/model/rover/joint/camera_pan_joint/0/cmd_pos")
        self.assertEqual(rover_config.DUST_TOPIC.format(link="rocker_left", emitter="dust_rl"),
                         "/model/rover/link/rocker_left/particle_emitter/dust_rl/cmd")


class Variants(unittest.TestCase):
    def test_replace_merges_into_the_sections(self):
        v = Variant().replace(contact={"dig": False}, motor={"current_limit": 10.0}, tread_tyre=False)
        self.assertFalse(v.sim.contact.dig)
        self.assertEqual(v.sim.contact.mu_noise, SIM.contact.mu_noise)
        self.assertEqual(v.rover.motor.current_limit, 10.0)
        self.assertEqual(v.rover.motor.kt, ROVER.motor.kt)
        self.assertFalse(v.sim.tread_tyre)
        self.assertEqual(Variant().sim, SIM)  # the default is the files' own, untouched

    def test_nested_sections_merge_too(self):
        v = Variant().replace(ruts={"tracks": {"report": 1.0}})
        self.assertEqual(v.sim.ruts.tracks.report, 1.0)
        self.assertEqual(v.sim.ruts.tracks.spacing, SIM.ruts.tracks.spacing)
        self.assertTrue(v.sim.ruts.enabled)

    def test_a_misspelt_name_is_refused(self):
        with self.assertRaises(KeyError):
            Variant().replace(contact={"mu_nosie": 0.0})
        with self.assertRaises(KeyError):
            Variant().replace(no_such_section={"x": 1.0})

    def test_variants_are_values(self):
        """Equal variants are equal and hash alike: gen_rover.build_sdf caches by variant."""
        a = Variant().replace(contact={"dig": False})
        b = Variant().replace(contact={"dig": False})
        self.assertEqual(a, b)
        self.assertEqual(hash(a), hash(b))
        self.assertNotEqual(a, Variant())
        self.assertIsInstance(a.sim.contact, Tree)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_rover_config 2>&1 | tail -3
```

Expected: `FAILED (errors=1)`: `ModuleNotFoundError: No module named 'rover_config'`.

- [ ] **Step 3: Write sim.yaml**

Create `rover_sim/config/sim.yaml`:

```yaml
# How the simulation models the rover and its ground: the drivetrain plugin's settings (its twist input, the stand-in
# motor controller, the motor model's integration), the contact model, the dig-in and its visual cues, the tyre
# model, dust, and the names of the rover's Gazebo interface. urdf/rover_sim.urdf.xacro writes them into the rover's
# model (`pixi run sim-model`); the rover's physical numbers are rover_description/config/rover.yaml. Units SI;
# numbers that are not counts keep a decimal point.
# Sources: (A) assumed, (M) measured, (R) from a reference; the realism design spec
# docs/superpowers/specs/2026-10-06-urc-realism-design.md (sections, D and Q numbers).

drivetrain:
  # physical: plugins/rover_drivetrain.cpp, a DC motor per wheel driving it by torque, every wheel contact gripping
  # like the ground under it (the default); diffdrive: Gazebo's DiffDrive, every wheel a velocity servo on
  # anisotropic tyres (the diffdrive section), kept for A/B tests and cost comparisons (D22).
  mode: physical
  # The twist input (PR 3 moves it into the drive controller). A command older than cmd_timeout counts as zero, so a
  # commander that died cannot leave the rover driving (user decision 2026-10-06); 0 holds the last command, as
  # Gazebo's GUI Teleop needs. The clock: wall (a dead process) or sim (deterministic tests).
  cmd_timeout: 0.5                  # [s]
  cmd_timeout_clock: wall
  track_multiplier: 1.0             # effective-track compensation; 1 = the raw skid-steer response (D24, Q10)
  accel: 8.0                        # [rad/s^2] wheel setpoint ramp, 1.2 m/s^2 (A: until the driver team reports theirs, Q7)
  # The speed loop of the motor controller the simulation stands in for, tuned against this motor model to the
  # design's response (6.9: < 60 ms). The real controller is not chosen; its gains will be tuned on the hardware and
  # live with its driver, so they are the simulation's, not facts of the rover (decision of the PR 2 plan).
  kp: 4.0                           # [V/(rad/s)] (M: prototype tuning)
  # [V/rad] integral time kp / ki 25 ms (A: retuned from the prototype's 40, whose 0.1 s left a turn without the ramp
  # at 90 % only after 118 ms; at 160 it takes 53 ms, design spec 6.9: < 60 ms; measured 2026-10-07).
  ki: 160.0
  speed_filter: 0.005               # [s] measured-speed time constant, also the contact rule's (A)
  substeps: 4                       # motor integration steps per 1 ms physics step
  # The wheel joints' limits under the physical drivetrain: DART never clamps the torque, and the velocity limit
  # (1.5 x the 21.6 rad/s free speed) never acts as a hidden brake (6.2); the motor model is the only limit.
  wheel_effort: 1000.0              # [N m]
  wheel_velocity: 32.0              # [rad/s]
  tyre_mu: 1.0                      # isotropic tyre friction: the drivetrain sets the friction of every wheel contact
  odom_rate: 50.0                   # [Hz] the no-slip odometry (DiffDrive's too)
  state_rate: 50.0                  # [Hz] the drivetrain state topic

# Gazebo's DiffDrive (drivetrain mode diffdrive). Lateral friction below longitudinal emulates tyre scrub; with equal
# values the physics engine's box friction stops the rover turning in place.
diffdrive:
  wheel_effort: 30.0                # [N m] each wheel servo's torque limit (A: rover model design 2026-10-05)
  mu_longitudinal: 1.0
  mu_lateral: 0.5

# Wheel contacts (6.4): a friction circle while sliding, an aligned box while sticking; the names are the
# drivetrain's <contact> elements.
contact:
  v_stribeck: 0.03                  # [m/s] (A)
  v_align: 0.005                    # [m/s] below it a contact sticks (A)
  perp_ratio: 0.0                   # mu across the slip while sliding: 0 is a friction circle
  stick_perp_ratio: 0.3             # mu across the expected load while sticking: holds within 4.4 % of mu_s
  mu_noise: 0.2                     # spatial mu variation (A)
  mu_noise_length: 0.3              # [m] (A)
  # [rad/s] rolling resistance and bulldozing fade in over rr_w0 x radius of hub speed, 7.5 mm/s (A; design spec 6.2
  # has 0.2, but over 3 cm/s a rover dug in by the strong preset creeps round at 0.25 x the fresh rate instead of
  # sticking; at 0.05 it turns 0.07 x, measured).
  rr_w0: 0.05
  # Dig-in on loose ground (6.5, D21); how strong is the ground's (urc/terrains.py DIG, the one switch: strong by
  # default, the user's choice, Q12).
  dig: true
  dig_heal_length: 0.3              # [m] one wheel diameter of travel heals a dug wheel by 1/e (A)
  default_surface: regolith         # urc/terrains.py TYPES key: the ground where the world has no ground map
  object_surface: manmade           # urc/terrains.py TYPES key: objects whose SDF sets no friction

# Tyre compliance (6.7, D18; phase 2, off): a hub between rocker and wheel on two sprung prismatic joints, axial then
# radial, with the tyre's stiffness, damping and travel (rover.yaml tyre). It turns 400-480 Hz contact chatter into a
# 12-16 Hz wheel hop (M); revisit when the wheel type is known (Q8).
tyre_compliance:
  enabled: false
  hub_mass: 0.1                     # [kg] each hub
  hub_inertia: 1.0e-4               # [kg m^2] each principal moment

# The dig-in made visible, the one switch (user decision 2026-10-07): each tyre's visual, never its collision, is
# drawn gain x (D - 1) x the ground's static sinkage lower, straight down, at most max, through a tau lag
# (rover_drivetrain.cpp ShowSink; physics never reads a visual's pose). At gain 1, the true scale the user chose over
# the research's x3, it is the spec's extra sinkage z_d: 2 cm on sand and clay at D_max 2, 3 cm on wash sand, under
# 4 mm on the sand sheet, nothing on rock, pavement and crusts. Off: no <dig_sink>, and the drivetrain writes no visual.
dig_sink:
  enabled: true
  gain: 1.0                         # true scale (user decision 2026-10-07)
  max: 0.06                         # [m] 0.4 x the wheel radius (A); not reached at gain 1 (wash sand: 3 cm)
  tau: 0.1                          # [s] a wheel off the ground for a few steps does not flicker (A)

# Ruts and pits behind the wheels (plugins/rover_tracks.hh), the one switch: on by default (user decision 2026-10-07:
# dig made visible on all soft ground). Off, the drivetrain gets no <tracks> and draws, reads and costs nothing.
ruts:
  enabled: true
  # The drivetrain's <tracks> group, with the wheel's width (rover.yaml). Visual only: a wheel lays a cross-section
  # every `spacing` of its travel from its own state (ground point, heading, dig factor D, slip); camera sensors draw
  # them (the station's views and the rover's RGB-D, not the Gazebo GUI). s: the ground's static sinkage.
  tracks:
    spacing: 0.05                   # [m] of a wheel's travel per cross-section (A: the research prototype's)
    chunk: 128                      # records per chunk: one marker per darkness level and ground (M: 128/512/2048 alike)
    # The ring: 8000 records are about 100 m of the rover's travel (4 wheels x 20 per metre), the oldest go first.
    # The station's frame rate and real-time factor with 8000 were those without ruts within noise (M 2026-10-07,
    # Delivery and the proving ground, eye + chase + RGB-D; the research saw -2-3 % at 4000, -3.5-6 % at 16000).
    max_segments: 8000
    # Berm crest above the visual surface: berm_gain x s x (max(D - 1, 0) + berm_base), at most berm_max, drawn from
    # berm_min: the wheel already sits s into the surface (the carve), so the rut looks about s (D + 0.3) deep, true
    # scale (sand 6 mm at D 1, 2.6 cm at D 2; regolith 1.5 mm: no berm, only the faint floor) (A).
    berm_base: 0.3
    berm_gain: 1.0
    berm_max: 0.06                  # [m]
    berm_min: 0.004                 # [m]
    # Floor overlay opacity (opacity_base + opacity_dig (D - 1)) x clamp(s / 2 cm, 0.6, 1.5), in levels of 15, 28, 39
    # and 48 % (A: the prototype's look; it writes no depth).
    opacity_base: 0.14
    opacity_dig: 0.30
    floor_offset: 0.004             # [m] the overlay above the visual surface, with a depth bias (M: no dropouts)
    pit_dig: 0.05                   # D grown while moving under `spacing` that digs a pit (A)
    smear_slip: 0.3                 # slip over the tyre's surface speed above which tread marks smear (A)
    # Records one frame builds at most: a station that connects late (no frame is drawn while no camera is watched)
    # catches up over frames instead of stalling the simulation, which waits for the frame's scene update (M: 512
    # records took 7-12 ms on Delivery under load; 256, half that, catches up a full ring in about 1.6 s).
    frame_budget: 256
    stress: 0                       # synthetic records laid round the rover at the first step (cost measurements)
    report: 0.0                     # [s] of sim time between cost lines on stderr (measurements); 0: none

# Dust behind the rear wheels (6.5, D15), the one switch. Off, the default: the rover raises no visible dust and the
# model has no particle emitter (user decision 2026-10-07: clean depth images and point clouds over visible dust).
# gz-rendering 8.2.2's depth shader takes any visible particle for a return, so with dust on the depth image and point
# cloud see it (Q11 "depth does not see dust" not met, tests/test_render.py); with none they cannot. On: an emitter
# behind each rear wheel in both drive modes, its rate set by the physical drivetrain (<dust>, <dust_rule>), for when
# gz-rendering honours the emitters' <particle_scatter_ratio> (scatter_ratio).
dust:
  enabled: false
  rate: 10.0                        # [Hz] commands to the emitters
  # Particles per second = dust factor x (speed_gain x hub speed + slip_gain x slip speed) x dig factor, at most
  # max_rate (A gains); the drivetrain's <dust_rule>.
  rule:
    speed_gain: 8.0                 # [1/m]
    slip_gain: 25.0                 # [1/m]
    max_rate: 40.0                  # [1/s]
    min_speed: 0.05                 # [m/s]
  # The emitter (M: tuned in the render research, 6.5): a box at the ground behind each rear wheel blowing back and
  # up, each particle the catalogue's dust colour (terrains.DUST_RGB) at alpha times the sprite's own alpha.
  # gz-rendering 8 applies no colour range (SetColorRange is disabled, a <color_range_image> changed nothing,
  # measured), so the tuned fade 0.28 -> 0 cannot be had: the sprite bakes in the plume's opacity at birth, and a
  # particle keeps it for its lifetime.
  box: 0.25                         # [m]
  particle: 0.2                     # [m]
  lifetime: 1.6                     # [s]
  speed: [0.15, 0.5]                # [m/s]
  alpha: 0.28
  pitch: 0.6                        # [rad] above the horizontal, backwards (A)
  # The share of particle pixels depth and lidar take for a return (D15, Q11): as near none as gz-rendering takes.
  # With no ratio it would apply 0.65, SDF's default too, and it ignores a ratio not above 0 (R: gz-rendering 8's
  # BaseParticleEmitter). In 8.2.2 no ratio reaches the depth shader (M, tests/test_render.py).
  scatter_ratio: 1.0e-6

# The tyres' look (user decision 2026-10-07): a dark tyre with light tread bars and spokes, one spoke ochre, so a
# wheel spinning while the rover stands still shows (rover_description/meshes/wheel.glb; the 16 bars alias above
# about 3.9 rad/s at a 20 Hz view, the one ochre spoke only above 63 rad/s, past the 20 rad/s top speed). Visual only:
# the collision stays the cylinder. Off: the plain dark cylinder.
tread_tyre: true

# The RGB image sees the far field to 80 km while the depth stays clipped at rover.yaml's camera clip (design spec 7,
# D14). No SDF <noise> (design spec 4's stddev 0.06): on an rgbd_camera it aborts gz on Metal (measured: Ogre
# RenderingAPIException, float4 output to an RGBA32Uint attachment), so noise belongs in the station.
camera_far: 80000.0                 # [m]

# Each head joint has a JointPositionController in velocity mode: it slews at rover.yaml's head speed to the angle
# last sent on its topic and holds it (the astronaut's arms in urc/props.py work the same way).
head_p_gain: 10.0

# Gazebo applies NavSat's horizontal noise to latitude and longitude in degrees: the model converts at this many
# metres per degree, so east-west noise is rover.yaml's gnss noise_horizontal times cos(latitude).
meters_per_degree: 111320.0

ground_truth_rate: 50.0             # [Hz] OdometryPublisher, the world-frame pose (3 dimensions)

# Joints without position limits (the wheels, continuous in the description) get these stops, as the model has had
# since 2026-10-05 (the conversion would leave them infinite; neither is ever reached).
continuous_stops: 1.0e+16           # [rad]

# The rover's Gazebo topics; {joint}, {link} and {emitter} are filled in.
topics:
  cmd_vel: /model/rover/cmd_vel
  odometry: /model/rover/odometry
  tf: /model/rover/tf
  joint_states: /model/rover/joint_states
  ground_truth: /model/rover/ground_truth
  imu: /model/rover/imu
  navsat: /model/rover/navsat
  camera: /model/rover/camera                 # rgbd: /image, /depth_image, /points, /camera_info
  led: /model/rover/led                       # set by the rover's software, shown by referee.py
  drivetrain: /model/rover/drivetrain         # physical drivetrain: gz.msgs.StringMsg, JSON (design spec 9.3)
  dust: /model/rover/link/{link}/particle_emitter/{emitter}/cmd   # gz.msgs.ParticleEmitter
  head: /model/rover/joint/{joint}/0/cmd_pos  # gz.msgs.Double target angle [rad]
```

- [ ] **Step 4: Write rover_config.py**

Create `rover_sim/rover_config.py`:

```python
"""The rover's numbers for the simulation's Python, read once: rover_description/config/rover.yaml, the physical
rover (ROVER), and rover_sim/config/sim.yaml, how the simulation models it (SIM), as read-only trees with attribute
access (ROVER.wheel.radius, SIM.drivetrain.accel; YAML lists become tuples). Variant is a rover with some of those
numbers changed, which tools/gen_rover.py builds (the tests drive such variants). The names of the rover's Gazebo
interface (topics, joints, files in its model) are here too.

The station, referee, judge, viewers, world generator and tests read the rover here; nothing else reads the two
files but the xacro (urdf/rover_sim.urdf.xacro).
"""
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

import yaml

SIM_DIR = Path(__file__).resolve().parent
DESCRIPTION_DIR = SIM_DIR.parent / "rover_description"  # the package beside this one in the repository
ROVER_YAML = DESCRIPTION_DIR / "config" / "rover.yaml"
SIM_YAML = SIM_DIR / "config" / "sim.yaml"


class Tree(Mapping):
    """A read-only YAML mapping: tree.key or tree["key"]; nested mappings are Trees, lists tuples."""

    def __init__(self, data):
        self._data = {key: _freeze(value) for key, value in data.items()}

    def __getitem__(self, key):
        return self._data[key]

    def __getattr__(self, name):
        try:
            return self._data[name]
        except KeyError:
            raise AttributeError(name) from None

    def __iter__(self):
        return iter(self._data)

    def __len__(self):
        return len(self._data)

    def __hash__(self):
        return hash(tuple(self._data.items()))

    def __eq__(self, other):
        return isinstance(other, Tree) and self._data == other._data

    def __repr__(self):
        return f"Tree({self.plain()!r})"

    def merged(self, changes):
        """A copy with `changes` (a nested dict) merged in: a mapping merges into the mapping it replaces, anything
        else replaces the value. Every key must exist already (KeyError: a misspelt name never adds a number the
        model ignores)."""
        data = dict(self._data)
        for key, value in changes.items():
            if key not in data:
                raise KeyError(f"no {key!r} among {sorted(data)}")
            old = data[key]
            data[key] = old.merged(value) if isinstance(old, Tree) and isinstance(value, Mapping) else value
        return Tree(data)

    def plain(self):
        """Plain dicts and lists, for yaml.safe_dump."""
        return {key: value.plain() if isinstance(value, Tree) else _plain(value) for key, value in self._data.items()}


def _freeze(value):
    if isinstance(value, Mapping):
        return value if isinstance(value, Tree) else Tree(value)
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(v) for v in value)
    return value


def _plain(value):
    if isinstance(value, Tree):
        return value.plain()
    if isinstance(value, tuple):
        return [_plain(v) for v in value]
    return value


def load(path):
    """A YAML file as a Tree."""
    return Tree(yaml.safe_load(Path(path).read_text()))


ROVER = load(ROVER_YAML)
SIM = load(SIM_YAML)


@dataclass(frozen=True)
class Variant:
    """A rover to build (gen_rover.build_sdf): rover.yaml's numbers and sim.yaml's, the files' own by default."""
    rover: Tree = field(default=ROVER)
    sim: Tree = field(default=SIM)

    def replace(self, **sections):
        """A copy with sections changed, each a top-level key of rover.yaml or of sim.yaml (they share none) and a
        nested dict of new values, or a new value: replace(contact={"dig": False}, motor={"current_limit": 10.0},
        tread_tyre=False)."""
        rover = {k: v for k, v in sections.items() if k in self.rover}
        sim = {k: v for k, v in sections.items() if k not in self.rover}
        return Variant(self.rover.merged(rover), self.sim.merged(sim))


# The rover's Gazebo interface (sim.yaml topics; the joint and visual names are the description's).
CMD_VEL_TOPIC = SIM.topics.cmd_vel
ODOM_TOPIC = SIM.topics.odometry
TF_TOPIC = SIM.topics.tf
JOINT_STATE_TOPIC = SIM.topics.joint_states
GROUND_TRUTH_TOPIC = SIM.topics.ground_truth
IMU_TOPIC = SIM.topics.imu
NAVSAT_TOPIC = SIM.topics.navsat
CAMERA_TOPIC = SIM.topics.camera  # rgbd: /image, /depth_image, /points, /camera_info
LED_TOPIC = SIM.topics.led  # set by the rover's software, shown by referee.py
DRIVETRAIN_TOPIC = SIM.topics.drivetrain  # physical drivetrain: gz.msgs.StringMsg, JSON (design spec 9.3)
DUST_TOPIC = SIM.topics.dust  # .format(link=..., emitter=...): gz.msgs.ParticleEmitter
HEAD_TOPIC = SIM.topics.head  # .format(joint=...): gz.msgs.Double target angle [rad]
PAN_JOINT = "camera_pan_joint"
TILT_JOINT = "camera_tilt_joint"
LED_VISUAL = "led_visual"  # on base_link
WHEEL_MESH = "meshes/wheel.glb"  # in the rover model: a copy of rover_description/meshes/wheel.glb
DUST_SPRITE = "materials/textures/dust_puff.png"  # in the rover model: a soft dust puff (textures.dust_puff)
SIDES = (("left", "l", 1.0), ("right", "r", -1.0))  # name, suffix, sign of y
ENDS = (("front", "f", 1.0), ("rear", "r", -1.0))  # name, prefix, sign of x
```

- [ ] **Step 5: Run the test to see it pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_rover_config 2>&1 | tail -3
```

Expected: `Ran 8 tests`, `OK`.

- [ ] **Step 6: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/config/sim.yaml rover_sim/rover_config.py rover_sim/tests/test_rover_config.py && git commit -q -F - <<'EOF'
PR 2: rover_sim: sim.yaml, how the simulation models the rover; rover_config.py reads both files

config/sim.yaml holds gen_model.py's simulation numbers with their sources: the drivetrain's twist input, the stand-in
motor controller's PI tuning and speed filter (the real controller's gains will be tuned on the hardware, so they are
not facts of the rover), the motor model's substeps, the wheel joints' limits in Gazebo, the contact model, dig-in,
the tyre sink, ruts, dust, the tyre model, DiffDrive's tyres, and the rover's Gazebo topics. rover_config.py reads it
and rover_description's rover.yaml as read-only trees (ROVER, SIM), names the rover's Gazebo interface, and makes
variants (Variant.replace) for the tests; a misspelt key is an error.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 9: rover_sim.urdf.xacro and gen_rover.py: the generated rovers are the frozen ones

**Files:**
- Create: `rover_sim/urdf/rover_sim.urdf.xacro`, `rover_sim/tools/gen_rover.py`, `rover_sim/config/surfaces.yaml` (generated)
- Modify: `rover_sim/tests/test_equivalence.py`, `rover_sim/package.xml`

- [ ] **Step 1: Write the failing test**

Replace `rover_sim/tests/test_equivalence.py` with:

```python
#!/usr/bin/env python3
"""The rover the simulation loads is the rover gen_model.py wrote, number for number (design spec 2026-10-09,
section 5): tools/gen_rover.py's model of the description, and of each variant the tests build, against the frozen
models in fixtures/rover_model (written by gen_model.py at import/simulation, 9208047; fixtures/rover_model/README.md
lists them). SdfEquivalence checks the comparison itself; Drives runs the frozen and the generated rover side by
side in Gazebo. When the rover changes on purpose, refreeze the fixtures from gen_rover.py in the same commit and say
why (fixtures/rover_model/README.md)."""
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from sdf_equivalence import differences
from worldfiles import SIM_DIR

import gen_rover  # noqa: E402  (worldfiles puts sim/ and sim/tools on the path)
from rover_config import Variant  # noqa: E402
from urc import sdf  # noqa: E402

FIXTURES = SIM_DIR / "tests" / "fixtures" / "rover_model"
V = Variant()
# The frozen models and the variants of rover.yaml and sim.yaml that are the same rovers (README.md beside them).
VARIANTS = {
    "default": V,
    "diffdrive": V.replace(drivetrain={"mode": "diffdrive"}),
    "diffdrive_square": V.replace(drivetrain={"mode": "diffdrive"}, diffdrive={"mu_lateral": 1.0}),
    "diffdrive_dust": V.replace(drivetrain={"mode": "diffdrive"}, dust={"enabled": True}),
    "dust": V.replace(dust={"enabled": True}),
    "cues_off": V.replace(tread_tyre=False, dig_sink={"enabled": False}, ruts={"enabled": False}),
    "tyre_compliance": V.replace(tyre_compliance={"enabled": True}),
    "test_physical": V.replace(
        drivetrain={"cmd_timeout_clock": "sim", "accel": 0.0, "state_rate": 1000.0, "cmd_timeout": 0.0},
        contact={"dig": False, "mu_noise": 0.0, "default_surface": "sand"}, driveline={"backlash": 0.0},
        motor={"current_limit": 10.0}, ruts={"tracks": {"report": 1.0}}),
    "same_surfaces": V.replace(contact={"default_surface": "manmade"}),
    "unknown_surface": V.replace(contact={"default_surface": "lunar_dust"}),
}


def fixture(name):
    return (FIXTURES / f"{name}.sdf").read_text()


class SdfEquivalence(unittest.TestCase):
    """differences() finds what changes a model and nothing else."""

    @classmethod
    def setUpClass(cls):
        cls.reference = fixture("default")

    def changed(self, edit):
        root = ET.fromstring(self.reference)
        edit(root.find("model"))
        return differences(self.reference, sdf.document(root))

    def test_a_model_equals_itself(self):
        self.assertEqual(differences(self.reference, self.reference), [])

    def test_a_number_beyond_the_tolerance_differs(self):
        def heavier(model):
            mass = model.find("link[@name='wheel_fl']/inertial/mass")
            mass.text = sdf.fmt(float(mass.text) * (1 + 1e-6))
        self.assertEqual(len(self.changed(heavier)), 1)

    def test_rounding_within_the_tolerance_does_not(self):
        def rounded(model):
            ixx = model.find("link[@name='wheel_fl']/inertial/inertia/ixx")
            ixx.text = repr(float(ixx.text) * (1 + 1e-12))
        self.assertEqual(self.changed(rounded), [])

    def test_a_missing_plugin_parameter_differs(self):
        def without_ki(model):
            controller = model.find("plugin[@filename='RoverDrivetrain']/controller")
            controller.remove(controller.find("ki"))
        self.assertEqual(len(self.changed(without_ki)), 1)

    def test_the_plugins_order_counts(self):
        def swapped(model):
            plugins = model.findall("plugin")
            for plugin in plugins[2:4]:
                model.remove(plugin)
            model.extend([plugins[3], plugins[2]])
        self.assertTrue(any("plugin order" in line for line in self.changed(swapped)))

    def test_a_default_written_out_is_the_same(self):
        def explicit(model):
            axis = model.find("joint[@name='wheel_fl_joint']/axis")
            sdf.sub(sdf.sub(axis, "dynamics"), "spring_stiffness", 0.0)
        self.assertEqual(self.changed(explicit), [])

    def test_a_pose_compares_its_rotation_not_its_angles(self):
        path = "link[@name='rocker_left']/collision[@name='front_arm_collision']/pose"
        x, y, z, _, pitch, _ = (float(v) for v in ET.fromstring(self.reference).find(f"model/{path}").text.split())

        def turned(rpy):
            return lambda model: setattr(model.find(path), "text", sdf.fmt((x, y, z, *rpy)))

        self.assertEqual(self.changed(turned((np.pi, np.pi - pitch, np.pi))), [])  # the same rotation
        self.assertEqual(len(self.changed(turned((0.0, pitch + 1e-6, 0.0)))), 1)

    def test_an_element_only_one_side_has_differs(self):
        def extra(model):
            sdf.sub(model.find("link[@name='base_link']"), "self_collide", True)
        self.assertEqual(len(self.changed(extra)), 1)


class Equivalence(unittest.TestCase):
    """Each generated variant is its frozen model: links' mass, inertia and pose; joints' type, axis, limits,
    damping and springs; collisions, visuals and materials; sensors; every plugin and parameter, in order."""

    def test_the_generated_rovers_are_the_frozen_ones(self):
        self.assertEqual(sorted(p.stem for p in FIXTURES.glob("*.sdf")), sorted(VARIANTS))
        for name, variant in VARIANTS.items():
            with self.subTest(name):
                self.assertEqual(differences(fixture(name), gen_rover.build_sdf(variant)), [])

    def test_the_rover_is_the_tracked_model(self):
        """The tracked model://rover is the default rover (the comparison, not the bytes: gen_rover.py lays it out
        as the converter does)."""
        tracked = (SIM_DIR / "models" / "rover" / "model.sdf").read_text()
        self.assertEqual(differences(fixture("default"), tracked), [])


if __name__ == "__main__":
    unittest.main()
```

(It adds `VARIANTS`, the variants of `rover.yaml` and `sim.yaml` that are the frozen rovers, and `Equivalence`.)

- [ ] **Step 2: Run it to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_equivalence 2>&1 | tail -3
```

Expected: `FAILED (errors=1)`: `ModuleNotFoundError: No module named 'gen_rover'`.

- [ ] **Step 3: Write the wrapper xacro**

Create `rover_sim/urdf/rover_sim.urdf.xacro`:

```xml
<?xml version="1.0"?>
<!-- The simulated rover: rover_description's rover with what only Gazebo needs, set from config/sim.yaml (how the
     simulation models the rover) and the description's rover.yaml: the sensors (IMU, NavSat, RGB-D camera), the LED's
     emissive material, the tyres' friction, the rocker differential, the drivetrain and motor model with its ground
     surfaces, the joint state and ground-truth publishers, the camera head's controllers, the visual cues (tyre sink,
     ruts, tread tyre), the opt-in dust and tyre compliance. tools/gen_rover.py expands it, converts it with
     `gz sdf -p` and writes models/rover/model.sdf (`pixi run sim-model`). gz_ros2_control comes with PR 3.

     Arguments:
       description_dir  rover_description's folder (default: the package's, found through the ROS package index)
       rover_yaml       the rover's numbers (default: the description's config/rover.yaml)
       sim_yaml         the simulation's (default config/sim.yaml beside this folder)
       surfaces_yaml    the drivetrain's ground surfaces (default config/surfaces.yaml, written from urc/terrains.py)
       ros2_control     the description's ros2_control block: false until PR 3 brings gz_ros2_control -->
<robot name="rover" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="description_dir" default="$(find rover_description)"/>
  <xacro:arg name="sim_yaml" default="../config/sim.yaml"/>
  <xacro:arg name="surfaces_yaml" default="../config/surfaces.yaml"/>
  <!-- PR 3 adds gz_ros2_control; until then the simulated rover has no ros2_control block. -->
  <xacro:arg name="ros2_control" default="false"/>

  <xacro:property name="sim" value="${xacro.load_yaml(xacro.arg('sim_yaml'))}"/>
  <xacro:property name="surfaces" value="${xacro.load_yaml(xacro.arg('surfaces_yaml'))}"/>
  <xacro:property name="topics" value="${sim.topics}"/>
  <!-- The slots of the topic templates (${} cannot hold a brace): topics.head.replace(joint_slot, name). -->
  <xacro:property name="joint_slot" value="{joint}"/>
  <xacro:property name="link_slot" value="{link}"/>
  <xacro:property name="emitter_slot" value="{emitter}"/>
  <xacro:property name="drive" value="${sim.drivetrain}"/>
  <xacro:if value="${drive.mode not in ('physical', 'diffdrive')}">
    ${xacro.fatal("sim.yaml drivetrain mode " + repr(drive.mode) + ": 'physical' or 'diffdrive'")}
  </xacro:if>
  <xacro:property name="physical" value="${drive.mode == 'physical'}"/>

  <!-- Set before the description reads them (its xacro:property defaults). -->
  <xacro:if value="${physical}">
    <xacro:property name="wheel_effort_limit" value="${drive.wheel_effort}"/>
    <xacro:property name="wheel_velocity_limit" value="${drive.wheel_velocity}"/>
  </xacro:if>
  <xacro:unless value="${physical}">
    <xacro:property name="wheel_effort_limit" value="${sim.diffdrive.wheel_effort}"/>
  </xacro:unless>
  <xacro:property name="tread_tyre" value="${sim.tread_tyre}"/>
  <xacro:if value="${sim.tyre_compliance.enabled}">
    <xacro:property name="wheel_mount" value="compliant_wheel_mount"/>
  </xacro:if>

  <!-- Tyre compliance: the wheel hangs from its rocker through a hub on an axial, then a radial sprung prismatic
       joint, each within the tyre's travel (rover.yaml tyre). -->
  <xacro:macro name="tyre_hub" params="name parent tag axis xyz spring">
    <link name="${name}_hub_${tag}">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="${sim.tyre_compliance.hub_mass}"/>
        <inertia ixx="${sim.tyre_compliance.hub_inertia}" ixy="0" ixz="0" iyy="${sim.tyre_compliance.hub_inertia}"
                 iyz="0" izz="${sim.tyre_compliance.hub_inertia}"/>
      </inertial>
    </link>
    <joint name="${name}_tire_${tag}" type="prismatic">
      <parent link="${parent}"/>
      <child link="${name}_hub_${tag}"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="${axis}"/>
      <limit lower="${-rover.tyre.travel}" upper="${rover.tyre.travel}" effort="-1" velocity="-1"/>
      <dynamics damping="${spring.damping}"/>
    </joint>
    <gazebo reference="${name}_tire_${tag}">
      <springStiffness>${spring.stiffness}</springStiffness>
      <springReference>0</springReference>
    </gazebo>
  </xacro:macro>
  <xacro:macro name="compliant_wheel_mount" params="name rocker xyz">
    <xacro:tyre_hub name="${name}" parent="${rocker}" tag="axial" axis="0 1 0" xyz="${xyz}" spring="${rover.tyre.axial}"/>
    <xacro:tyre_hub name="${name}" parent="${name}_hub_axial" tag="radial" axis="0 0 1" xyz="0 0 0"
                    spring="${rover.tyre.radial}"/>
    <xacro:wheel_joint name="${name}" parent="${name}_hub_radial" xyz="0 0 0"/>
  </xacro:macro>

  <xacro:include filename="$(arg description_dir)/urdf/rover.urdf.xacro"/>

  <!-- A YAML mapping's items as elements: <key>value</key>, booleans as true/false. -->
  <xacro:macro name="elements" params="items">
    <xacro:if value="${items}">
      <xacro:element xacro:name="${items[0][0]}">${str(items[0][1]).lower() if python.isinstance(items[0][1], bool) else items[0][1]}</xacro:element>
      <xacro:elements items="${items[1:]}"/>
    </xacro:if>
  </xacro:macro>

  <!-- Sensors, on the description's frames. -->
  <xacro:macro name="gaussian" params="stddev">
    <noise type="gaussian">
      <mean>0</mean>
      <stddev>${stddev}</stddev>
    </noise>
  </xacro:macro>
  <gazebo reference="imu">
    <!-- Reported in base_link (the same orientation): only the linear acceleration differs, by lever-arm terms,
         from a sensor at base_link's origin. -->
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>${rover.imu.rate}</update_rate>
      <topic>${topics.imu}</topic>
      <gz_frame_id>base_link</gz_frame_id>
      <imu>
        <angular_velocity>
          <x><xacro:gaussian stddev="${rover.imu.gyro_noise}"/></x>
          <y><xacro:gaussian stddev="${rover.imu.gyro_noise}"/></y>
          <z><xacro:gaussian stddev="${rover.imu.gyro_noise}"/></z>
        </angular_velocity>
        <linear_acceleration>
          <x><xacro:gaussian stddev="${rover.imu.accel_noise}"/></x>
          <y><xacro:gaussian stddev="${rover.imu.accel_noise}"/></y>
          <z><xacro:gaussian stddev="${rover.imu.accel_noise}"/></z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>
  <gazebo reference="gnss">
    <sensor name="gnss" type="navsat">
      <always_on>true</always_on>
      <update_rate>${rover.gnss.rate}</update_rate>
      <topic>${topics.navsat}</topic>
      <gz_frame_id>gnss</gz_frame_id>
      <navsat>
        <position_sensing>
          <horizontal><xacro:gaussian stddev="${rover.gnss.noise_horizontal / sim.meters_per_degree}"/></horizontal>
          <vertical><xacro:gaussian stddev="${rover.gnss.noise_vertical}"/></vertical>
        </position_sensing>
      </navsat>
    </sensor>
  </gazebo>
  <gazebo reference="camera">
    <sensor name="camera" type="rgbd_camera">
      <always_on>true</always_on>
      <update_rate>${rover.camera.rate}</update_rate>
      <topic>${topics.camera}</topic>
      <gz_frame_id>camera</gz_frame_id>
      <camera>
        <horizontal_fov>${rover.camera.hfov}</horizontal_fov>
        <image>
          <width>${rover.camera.size[0]}</width>
          <height>${rover.camera.size[1]}</height>
        </image>
        <clip>
          <near>${rover.camera.clip[0]}</near>
          <far>${sim.camera_far}</far>
        </clip>
        <depth_camera>
          <clip>
            <near>${rover.camera.clip[0]}</near>
            <far>${rover.camera.clip[1]}</far>
          </clip>
        </depth_camera>
      </camera>
    </sensor>
  </gazebo>
  <!-- The LED is off (dark grey, no glow) until the referee shows a colour on its topic. -->
  <gazebo reference="led">
    <visual>
      <material>
        <emissive>0 0 0 1</emissive>
      </material>
    </visual>
  </gazebo>

  <!-- The tyres' friction: isotropic under the physical drivetrain, which sets every wheel contact's friction itself;
       under DiffDrive mu along the axle (fdir1, in the collision frame: lateral) and mu2 across it (longitudinal). -->
  <xacro:macro name="tyre_friction" params="wheel">
    <gazebo reference="${wheel}">
      <collision>
        <surface>
          <friction>
            <ode>
              <xacro:if value="${physical}">
                <mu>${drive.tyre_mu}</mu>
                <mu2>${drive.tyre_mu}</mu2>
              </xacro:if>
              <xacro:unless value="${physical}">
                <mu>${sim.diffdrive.mu_lateral}</mu>
                <mu2>${sim.diffdrive.mu_longitudinal}</mu2>
                <fdir1>0 0 1</fdir1>
              </xacro:unless>
            </ode>
          </friction>
        </surface>
      </collision>
    </gazebo>
  </xacro:macro>
  <xacro:tyre_friction wheel="wheel_fl"/>
  <xacro:tyre_friction wheel="wheel_rl"/>
  <xacro:tyre_friction wheel="wheel_fr"/>
  <xacro:tyre_friction wheel="wheel_rr"/>

  <!-- Dust (off by default): an emitter at the ground behind each rear wheel, on its rocker (a wheel link spins),
       idle until the physical drivetrain sets its rate. It starts not emitting (SDF's default is to emit). Each
       particle is the soft puff sprite, which carries the dust's colour and opacity; the material needs its white
       diffuse (particles without one render black, measured). -->
  <xacro:property name="dust" value="${sim.dust}"/>
  <xacro:macro name="dust_emitter" params="side">
    <gazebo reference="rocker_${side}">
      <particle_emitter name="dust_r${side[0]}" type="box">
        <pose>${-(rover.wheel.dx + rover.wheel.radius + dust.box / 4)} 0 ${rover.wheel.dz - rover.wheel.radius + dust.box / 2} 0 ${-dust.pitch} ${pi}</pose>
        <emitting>false</emitting>
        <size>${dust.box} ${dust.box} ${dust.box}</size>
        <particle_size>${dust.particle} ${dust.particle} ${dust.particle}</particle_size>
        <lifetime>${dust.lifetime}</lifetime>
        <rate>0</rate>
        <min_velocity>${dust.speed[0]}</min_velocity>
        <max_velocity>${dust.speed[1]}</max_velocity>
        <scale_rate>1</scale_rate>
        <topic>${topics.dust.replace(link_slot, 'rocker_' + side).replace(emitter_slot, 'dust_r' + side[0])}</topic>
        <particle_scatter_ratio>${dust.scatter_ratio}</particle_scatter_ratio>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://rover/materials/textures/dust_puff.png</albedo_map></metal></pbr>
        </material>
      </particle_emitter>
    </gazebo>
  </xacro:macro>
  <xacro:if value="${dust.enabled}">
    <xacro:dust_emitter side="left"/>
    <xacro:dust_emitter side="right"/>
  </xacro:if>

  <!-- The camera head: a JointPositionController per joint in velocity mode slews at the head's speed to the angle
       last sent on its topic and holds it. -->
  <xacro:macro name="head_controller" params="joint start">
    <gazebo>
      <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
        <joint_name>${joint}</joint_name>
        <topic>${topics.head.replace(joint_slot, joint)}</topic>
        <use_velocity_commands>true</use_velocity_commands>
        <p_gain>${sim.head_p_gain}</p_gain>
        <cmd_max>${rover.head.speed}</cmd_max>
        <cmd_min>${-rover.head.speed}</cmd_min>
        <initial_position>${start}</initial_position>
      </plugin>
    </gazebo>
  </xacro:macro>
  <xacro:head_controller joint="camera_pan_joint" start="0.0"/>
  <xacro:head_controller joint="camera_tilt_joint" start="${rover.head.home_tilt}"/>

  <!-- The rocker differential: a torsion spring and damper on the sum of the rocker angles. -->
  <gazebo>
    <plugin filename="RockerDifferential" name="rover_sim::RockerDifferential">
      <left_joint>rocker_left_joint</left_joint>
      <right_joint>rocker_right_joint</right_joint>
      <stiffness>${rover.differential.stiffness}</stiffness>
      <damping>${rover.differential.damping}</damping>
    </plugin>
  </gazebo>

  <!-- The drive: the physical drivetrain (plugins/rover_drivetrain.cpp), never beside DiffDrive, which would override
       its torques (D22). Its dust elements only with dust on; <dig_sink> and <tracks> only with their switches. -->
  <xacro:macro name="drivetrain_wheel" params="name side">
    <wheel>
      <name>${name}</name>
      <joint>wheel_${name}_joint</joint>
      <link>wheel_${name}</link>
      <side>${side}</side>
    </wheel>
  </xacro:macro>
  <xacro:macro name="surface_row" params="key">
    <xacro:if value="${key in surfaces}">
      <surface>
        <key>${key}</key>
        <xacro:elements items="${list(surfaces[key].items())}"/>
      </surface>
    </xacro:if>
  </xacro:macro>
  <xacro:if value="${physical}">
    <gazebo>
      <plugin filename="RoverDrivetrain" name="rover_sim::RoverDrivetrain">
        <topic>${topics.cmd_vel}</topic>
        <cmd_timeout>${drive.cmd_timeout}</cmd_timeout>
        <cmd_timeout_clock>${drive.cmd_timeout_clock}</cmd_timeout_clock>
        <odom_topic>${topics.odometry}</odom_topic>
        <tf_topic>${topics.tf}</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>${drive.odom_rate}</odom_publish_frequency>
        <state_topic>${topics.drivetrain}</state_topic>
        <state_rate>${drive.state_rate}</state_rate>
        <xacro:if value="${dust.enabled}">
          <dust_rate>${dust.rate}</dust_rate>
        </xacro:if>
        <track>${2 * rover.rocker.pivot_y}</track>
        <radius>${rover.wheel.radius}</radius>
        <track_multiplier>${drive.track_multiplier}</track_multiplier>
        <xacro:drivetrain_wheel name="fl" side="left"/>
        <xacro:drivetrain_wheel name="rl" side="left"/>
        <xacro:drivetrain_wheel name="fr" side="right"/>
        <xacro:drivetrain_wheel name="rr" side="right"/>
        <xacro:if value="${dust.enabled}">
          <dust>
            <wheel>wheel_rl</wheel>
            <topic>${topics.dust.replace(link_slot, 'rocker_left').replace(emitter_slot, 'dust_rl')}</topic>
          </dust>
          <dust>
            <wheel>wheel_rr</wheel>
            <topic>${topics.dust.replace(link_slot, 'rocker_right').replace(emitter_slot, 'dust_rr')}</topic>
          </dust>
        </xacro:if>
        <motor>
          <xacro:elements items="${list(rover.motor.items())}"/>
        </motor>
        <driveline>
          <xacro:elements items="${list(rover.driveline.items())}"/>
        </driveline>
        <controller>
          <kp>${drive.kp}</kp>
          <ki>${drive.ki}</ki>
          <speed_filter>${drive.speed_filter}</speed_filter>
          <accel>${drive.accel}</accel>
          <max_speed>${rover.wheel.top_speed}</max_speed>
          <substeps>${drive.substeps}</substeps>
        </controller>
        <contact>
          <xacro:elements items="${list(sim.contact.items())}"/>
          <!-- The default and object surfaces' traction, for worlds without a ground map. -->
          <xacro:surface_row key="${sim.contact.default_surface}"/>
          <xacro:if value="${sim.contact.object_surface != sim.contact.default_surface}">
            <xacro:surface_row key="${sim.contact.object_surface}"/>
          </xacro:if>
        </contact>
        <xacro:if value="${dust.enabled}">
          <dust_rule>
            <xacro:elements items="${list(dust.rule.items())}"/>
          </dust_rule>
        </xacro:if>
        <xacro:if value="${sim.dig_sink.enabled}">
          <dig_sink>
            <gain>${sim.dig_sink.gain}</gain>
            <max>${sim.dig_sink.max}</max>
            <tau>${sim.dig_sink.tau}</tau>
          </dig_sink>
        </xacro:if>
        <xacro:if value="${sim.ruts.enabled}">
          <tracks>
            <xacro:elements items="${list(sim.ruts.tracks.items())}"/>
            <width>${rover.wheel.width}</width>
          </tracks>
        </xacro:if>
      </plugin>
    </gazebo>
  </xacro:if>
  <xacro:unless value="${physical}">
    <gazebo>
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>wheel_fl_joint</left_joint>
        <left_joint>wheel_rl_joint</left_joint>
        <right_joint>wheel_fr_joint</right_joint>
        <right_joint>wheel_rr_joint</right_joint>
        <wheel_separation>${2 * rover.rocker.pivot_y}</wheel_separation>
        <wheel_radius>${rover.wheel.radius}</wheel_radius>
        <topic>${topics.cmd_vel}</topic>
        <odom_topic>${topics.odometry}</odom_topic>
        <tf_topic>${topics.tf}</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>${drive.odom_rate}</odom_publish_frequency>
      </plugin>
    </gazebo>
  </xacro:unless>

  <gazebo>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>${topics.joint_states}</topic>
    </plugin>
  </gazebo>
  <gazebo>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_topic>${topics.ground_truth}</odom_topic>
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <dimensions>3</dimensions>
      <odom_publish_frequency>${sim.ground_truth_rate}</odom_publish_frequency>
    </plugin>
  </gazebo>
</robot>
```

- [ ] **Step 4: Write the generator**

Create `rover_sim/tools/gen_rover.py`:

```python
#!/usr/bin/env python3
"""Generate the rover's Gazebo model, models/rover/model.sdf, from its description: urdf/rover_sim.urdf.xacro
(rover_description's rover.urdf.xacro and Gazebo's parts) expanded by xacro, converted to SDF by `gz sdf -p` and
post-processed (postprocess), so that the converter's quirks leave no trace; with it the drivetrain's ground surfaces
(config/surfaces.yaml, from urc/terrains.py), the dust sprite, a copy of the tread tyre's mesh and the driver
station's cameras (viewers.py). `pixi run sim-model` runs it.

build_sdf(variant) builds a variant of the rover (rover_config.Variant: some of rover.yaml's or sim.yaml's numbers
changed) without writing anything; the tests drive such variants. tests/test_rover_model.py checks that the default
rover and the variants the tests use are the frozen models gen_model.py wrote (tests/fixtures/rover_model).
"""
import dataclasses
import functools
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
import rover_config  # noqa: E402
from rover_config import DUST_SPRITE, SIM, WHEEL_MESH, Variant  # noqa: E402
from urc import sdf  # noqa: E402  (the SDF writer every generated model shares)

XACRO = SIM_DIR / "urdf" / "rover_sim.urdf.xacro"
SURFACES_YAML = SIM_DIR / "config" / "surfaces.yaml"
MODELS_DIR = SIM_DIR / "models"
MODEL_DIR = MODELS_DIR / "rover"
DESCRIPTION_MESH = rover_config.DESCRIPTION_DIR / WHEEL_MESH  # the tread tyre (rover_description/meshes)
CONVERTED_MESHES = "model://rover_description/meshes/"  # how the converter writes the description's package:// URIs
LUMP = "fixed_joint_lump__"  # sdformat's marker in the names of shapes it moved across fixed joints
# What the converter writes into every joint a <gazebo> block names: SDF's defaults for ODE's joint limits.
JOINT_PHYSICS_DEFAULTS = {"ode/limit/cfm": "0", "ode/limit/erp": "0.2"}
SURFACES_HEADER = ("# The drivetrain's ground surfaces: every urc/terrains.py type's traction under its dig-in preset\n"
                   "# (terrains.DIG) and its dust factor, keyed by type. Written by tools/gen_rover.py\n"
                   "# (pixi run sim-model) from the catalogue; do not edit.\n")


def surface_rows():
    """Every catalogue ground type's traction (terrains.TYPES, design spec 5.6, under the dig-in preset terrains.DIG,
    as worlds write it) and dust factor: the drivetrain's <surface> rows, keyed by type, for config/surfaces.yaml.
    The model takes the rows of sim.yaml's default and object surfaces; a key the catalogue lacks has none, and the
    drivetrain reports it when it starts and grips there as plain Coulomb mu 1."""
    from urc import terrains  # here: the catalogue imports the texture generators
    return {key: {**dataclasses.asdict(terrains.traction(kind)), "dust": kind.appearance.dust}
            for key, kind in terrains.TYPES.items()}


def surfaces_text():
    """config/surfaces.yaml as surface_rows() makes it."""
    return SURFACES_HEADER + yaml.safe_dump(surface_rows(), sort_keys=False)


def build_urdf(variant=Variant()):
    """The variant's URDF: rover_sim.urdf.xacro expanded with its numbers."""
    import xacro  # here: only building a model needs it
    with tempfile.TemporaryDirectory(prefix="gen_rover_") as tmp:
        rover_yaml, sim_yaml = Path(tmp) / "rover.yaml", Path(tmp) / "sim.yaml"
        rover_yaml.write_text(yaml.safe_dump(variant.rover.plain(), sort_keys=False))
        sim_yaml.write_text(yaml.safe_dump(variant.sim.plain(), sort_keys=False))
        doc = xacro.process_file(str(XACRO), mappings={
            "description_dir": str(rover_config.DESCRIPTION_DIR), "rover_yaml": str(rover_yaml),
            "sim_yaml": str(sim_yaml), "surfaces_yaml": str(SURFACES_YAML)})
    return doc.toprettyxml(indent="  ")


@functools.lru_cache(maxsize=64)
def build_sdf(variant=Variant()):
    """The variant's model as an SDF 1.11 document: build_urdf, `gz sdf -p`, postprocess."""
    mode = variant.sim.drivetrain.mode
    if mode not in ("physical", "diffdrive"):
        raise ValueError(f"sim.yaml drivetrain mode {mode!r}: 'physical' or 'diffdrive'")
    urdf = build_urdf(variant)
    with tempfile.TemporaryDirectory(prefix="gen_rover_") as tmp:
        path = Path(tmp) / "rover.urdf"
        path.write_text(urdf)
        result = subprocess.run(["gz", "sdf", "-p", "--precision", "9", str(path)], capture_output=True, text=True)
    if result.returncode != 0 or "<sdf" not in result.stdout:
        raise RuntimeError(f"gz sdf -p failed:\n{result.stdout}\n{result.stderr}")
    return postprocess(result.stdout, urdf, variant.sim.continuous_stops)


# --- The converter's quirks ----------------------------------------------------------------------------------------

def postprocess(text, urdf, stops):
    """The SDF that `gz sdf -p` writes for the rover's URDF, written as the model gen_model.py wrote (the
    equivalence test compares them number by number). sdformat's URDF conversion (parser_urdf.cc):
    - merges every link that hangs from a fixed joint (the sensor and LED frames) into its parent and adds a <frame>
      for it and for its joint: the frames are dropped (they are the description's, for ROS; the sensors and the
      LED's visual already sit in their link with their poses);
    - names a shape "<link>_[fixed_joint_lump__]<name>_<collision|visual>[_<n>]": back to "<name>_<...>";
    - writes a URDF colour rgba as diffuse and ambient rgb / 0.8, clipped at 1: both back to the URDF's rgba;
    - writes each link's pose relative to its joint and each joint's relative to its parent link: links get their
      model-frame poses and joints none (a URDF joint's frame is its child link's, SDF's default);
    - orders links and joints by walking the tree, children by joint name: back to the URDF's order;
    - leaves a continuous joint's stops infinite: +-stops (sim.yaml continuous_stops), as before;
    - passes plugin parameters through as xacro wrote them (Python's str): numbers with 9 significant digits, as
      --precision 9 writes SDF's own and gen_model.py wrote them;
    - adds a <physics> with ODE's default limits (JOINT_PHYSICS_DEFAULTS) to every joint a <gazebo> block names (the
      tyre model's springs): dropped; anything else there is an error;
    - leaves empty <surface> scaffolding in collisions without friction: dropped;
    - turns the description's package:// mesh URIs into model://rover_description/...: model://rover/..., where
      main() copies the meshes."""
    root = ET.fromstring(text)
    model = root.find("model")
    urdf_root = ET.fromstring(urdf)
    for frame in model.findall("frame"):
        model.remove(frame)
    colors = _urdf_colors(urdf_root)
    for link in model.findall("link"):
        name = link.get("name")
        for shape in [*link.findall("collision"), *link.findall("visual")]:
            match = re.fullmatch(rf"{re.escape(name)}_(?:{LUMP})?(.+_(?:collision|visual))(?:_\d+)?", shape.get("name"))
            if match is None:
                raise ValueError(f"unexpected shape name {shape.get('name')!r} in link {name!r}")
            shape.set("name", match.group(1))
        for visual in link.findall("visual"):
            material = visual.find("material")
            if material is not None:
                rgba = colors[name, visual.get("name")]
                for tag in ("ambient", "diffuse"):
                    material.find(tag).text = sdf.fmt(rgba)
        for uri in link.iter("uri"):
            if uri.text.startswith(CONVERTED_MESHES):
                uri.text = sdf.model_uri(MODEL_DIR.name, "meshes", uri.text[len(CONVERTED_MESHES):])
        for collision in link.findall("collision"):
            for surface in collision.findall("surface"):
                _drop_empty(collision, surface)
    _resolve_poses(model)
    for joint in model.findall("joint"):
        for physics in joint.findall("physics"):
            found = {"/".join(_path(physics, e)): (e.text or "").strip() for e in physics.iter() if not len(e)}
            if found != JOINT_PHYSICS_DEFAULTS:
                raise ValueError(f"joint {joint.get('name')!r}: unexpected <physics> {found}")
            joint.remove(physics)
        limit = joint.find("axis/limit")
        if limit is not None and limit.findtext("lower") == "-inf" and limit.findtext("upper") == "inf":
            limit.find("lower").text, limit.find("upper").text = sdf.fmt(-stops), sdf.fmt(stops)
    for plugin in model.findall("plugin"):
        for element in plugin.iter():
            element.text = _number_text(element.text)
    order = [e.get("name") for e in urdf_root if e.tag in ("link", "joint")]
    links = sorted(model.findall("link"), key=lambda e: order.index(e.get("name")))
    joints = sorted(model.findall("joint"), key=lambda e: order.index(e.get("name")))
    rest = [e for e in model if e.tag not in ("link", "joint")]
    for element in list(model):
        model.remove(element)
    model.extend(links + joints + rest)
    root.attrib = {"version": sdf.SDF_VERSION}
    return sdf.document(root)


def _path(top, element):
    """The tags from below `top` down to `element`."""
    parents = {child: parent for parent in top.iter() for child in parent}
    tags = []
    while element is not top:
        tags.append(element.tag)
        element = parents[element]
    return reversed(tags)


def _drop_empty(parent, element):
    """Remove `element` and its descendants that hold no text and no attributes (the converter's scaffolding)."""
    for child in list(element):
        _drop_empty(element, child)
    if not len(element) and not element.attrib and not (element.text or "").strip():
        parent.remove(element)


def _number_text(text):
    """A parameter's text with every number in 9 significant digits (sdf.fmt); other text unchanged."""
    if text is None or not text.strip():
        return text
    try:
        values = [float(v) for v in text.split()]
    except ValueError:
        return text
    return sdf.fmt(tuple(values) if len(values) > 1 else values[0])


def _urdf_colors(urdf_root):
    """(SDF link, SDF visual name) -> the URDF colour, for every URDF visual with a material: a link merged across
    fixed joints counts as the link it merged into, and a visual's SDF name is its URDF name (or its link's) +
    "_visual"."""
    named = {m.get("name"): m.find("color").get("rgba") for m in urdf_root.findall("material")
             if m.find("color") is not None}
    parent = {j.find("child").get("link"): (j.find("parent").get("link"), j.get("type"))
              for j in urdf_root.findall("joint")}

    def owner(link):
        while link in parent and parent[link][1] == "fixed":
            link = parent[link][0]
        return link

    colors = {}
    for link in urdf_root.findall("link"):
        for visual in link.findall("visual"):
            material = visual.find("material")
            if material is None:
                continue
            color = material.find("color")
            rgba = color.get("rgba") if color is not None else named[material.get("name")]
            key = (owner(link.get("name")), (visual.get("name") or link.get("name")) + "_visual")
            colors[key] = tuple(float(v) for v in rgba.split())
    return colors


def _pose_matrix(text):
    x, y, z, roll, pitch, yaw = (float(v) for v in (text or "0 0 0 0 0 0").split())
    m = np.eye(4)
    m[:3, :3] = sdf.rpy_to_matrix(roll, pitch, yaw)
    m[:3, 3] = (x, y, z)
    return m


def _resolve_poses(model):
    """Every link's pose in the model frame, from the converter's chain (a link relative to its joint, a joint
    relative to its parent link); the joints' poses dropped."""
    joints = {j.get("name"): j for j in model.findall("joint")}
    links = {link.get("name"): link for link in model.findall("link")}
    frames = {}

    def frame(name):  # a link's or joint's model-frame transform
        if name not in frames:
            element = links[name] if name in links else joints[name]
            pose = element.find("pose")
            if pose is None:
                frames[name] = np.eye(4)
            else:
                base = pose.get("relative_to")
                frames[name] = (frame(base) if base and base != "__model__" else np.eye(4)) @ _pose_matrix(pose.text)
        return frames[name]

    for name, link in links.items():
        m = frame(name)
        pose = link.find("pose")
        if pose is None:
            continue
        roll, pitch, yaw = sdf.matrix_to_rpy(m[:3, :3])
        pose.attrib.pop("relative_to", None)
        pose.text = sdf.fmt(tuple(float(v) + 0.0 for v in (*m[:3, 3], roll, pitch, yaw)))
    for joint in joints.values():
        pose = joint.find("pose")
        if pose is not None:
            joint.remove(pose)


# --- The files of the rover model ----------------------------------------------------------------------------------

def write_dust_sprite(path, alpha=SIM.dust.alpha):
    """The rover's dust sprite (DUST_SPRITE): textures.dust_puff in the catalogue's dust colour at sim.yaml's dust
    alpha. main() writes it whether or not dust is on, so the emitters find it when it is, and so do
    tests/test_render.py's."""
    from urc import terrains, textures  # here: the catalogue imports the texture generators
    textures.dust_puff(path, terrains.DUST_RGB, alpha)


def write_if_changed(path, data):
    """Write `data` (text or bytes) to `path` unless it holds those bytes already, through a hidden file and a rename:
    main() reads config/surfaces.yaml right after writing it, and two runs at once (pixi run test beside pixi run
    drive, both run sim-model) must never read a half-written file."""
    data = data.encode() if isinstance(data, str) else data
    if path.is_file() and path.read_bytes() == data:
        return
    partial = path.with_name(f".{path.name}.{os.getpid()}")
    partial.write_bytes(data)
    partial.replace(path)


def main():
    write_if_changed(SURFACES_YAML, surfaces_text())
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    out = MODEL_DIR / "model.sdf"
    write_if_changed(out, build_sdf(Variant()))
    sprite = MODEL_DIR / DUST_SPRITE
    sprite.parent.mkdir(parents=True, exist_ok=True)
    write_dust_sprite(sprite)
    wheel = MODEL_DIR / WHEEL_MESH
    wheel.parent.mkdir(parents=True, exist_ok=True)
    write_if_changed(wheel, DESCRIPTION_MESH.read_bytes())
    print(f"wrote {SURFACES_YAML.relative_to(SIM_DIR)}, {out.relative_to(SIM_DIR)}, "
          f"{sprite.relative_to(MODEL_DIR)} and {wheel.relative_to(MODEL_DIR)}")
    import viewers  # here: the station's cameras are the simulation's own models
    viewers.write_all(MODELS_DIR)


if __name__ == "__main__":
    main()
```

In `rover_sim/package.xml`, replace

```xml
  <depend>gz_common_vendor</depend>
```

with

```xml
  <depend>gz_common_vendor</depend>

  <!-- tools/gen_rover.py builds the rover's model from the description (rover_config.py reads its YAML) -->
  <exec_depend>rover_description</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
```

- [ ] **Step 5: Write the surfaces from the catalogue**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run python -c "
import sys; sys.path.insert(0, 'rover_sim/tools')
import gen_rover; gen_rover.SURFACES_YAML.write_text(gen_rover.surfaces_text())" \
  && head -14 rover_sim/config/surfaces.yaml && grep -c '^[a-z_]*:$' rover_sim/config/surfaces.yaml
```

Expected: the three header lines, then the first row (`gravel:` with `mu_s: 0.62`, `mu_k: 0.52`, `crr: 0.05`, `bulldoze: 0.0`, `slip: 0.3`, `sinkage_m: 0.005`, `dig_rate: 0.0`, `dig_max: 1.0`, `dust: 0.3`), and `21` (every type of `urc/terrains.py`).

- [ ] **Step 6: Run the test to see it pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_equivalence 2>&1 | tail -3
```

Expected: `Ran 10 tests`, `OK`. A difference prints as lines like `model/link[wheel_fl]/inertial/mass: '2.5' != '2.6'`: fix the YAML or the xacro, never the fixture.

- [ ] **Step 7: The wrapper expands without the generator too**

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run xacro rover_sim/urdf/rover_sim.urdf.xacro description_dir:=$PWD/rover_description > "$T/rover_sim.urdf" \
  && /Users/alarion239/.pixi/bin/pixi run check_urdf "$T/rover_sim.urdf" | head -3 && grep -c "<gazebo" "$T/rover_sim.urdf"; rm -f "$T/rover_sim.urdf"
```

Expected: `robot name is: rover`, `Successfully Parsed XML`, `root Link: base_link has 6 child(ren)`, then `14` (`<gazebo>` blocks: three sensors, the LED, four tyres, two head controllers, the differential, the drivetrain, the joint state and ground-truth publishers).

- [ ] **Step 8: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/urdf rover_sim/tools/gen_rover.py rover_sim/config/surfaces.yaml rover_sim/tests/test_equivalence.py rover_sim/package.xml && git commit -q -F - <<'EOF'
PR 2: rover_sim: the simulated rover built from rover_description, the same rover

urdf/rover_sim.urdf.xacro includes rover_description's rover.urdf.xacro and adds, in <gazebo> blocks, what only
Gazebo needs, from sim.yaml and rover.yaml: the IMU, NavSat and RGB-D sensors on the description's frames, the LED's
emissive material, the tyres' friction, the rocker differential, the drivetrain with its motor model and ground
surfaces, the joint state and ground-truth publishers, the camera head's controllers, the dig-in cues, the opt-in
dust and tyre compliance. tools/gen_rover.py expands it with xacro, converts it with gz sdf -p and post-processes the
converter's quirks (merged frames, shape names, colours, poses, order, infinite stops, plugin numbers, default ODE
limits, mesh URIs), each named in its docstring. config/surfaces.yaml is every catalogue type's traction row, from
urc/terrains.py. test_equivalence: the default rover and the nine variants are gen_model.py's, number for number.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 10: The generated rover drives as the frozen one

**Files:**
- Modify: `rover_sim/tests/test_equivalence.py`

The models are equal number for number; this test shows that Gazebo makes the same rover of them: the frozen and the generated model drive the same schedule to the same trace, bit for bit. It passes at once (a characterisation test of Task 9's result); if it does not, find the difference with superpowers:systematic-debugging before going on (an element order Gazebo depends on, a number the comparison's tolerance hides).

- [ ] **Step 1: Write the test**

Replace `rover_sim/tests/test_equivalence.py` with its final form:

```python
#!/usr/bin/env python3
"""The rover the simulation loads is the rover gen_model.py wrote, number for number (design spec 2026-10-09,
section 5): tools/gen_rover.py's model of the description, and of each variant the tests build, against the frozen
models in fixtures/rover_model (written by gen_model.py at import/simulation, 9208047; fixtures/rover_model/README.md
lists them). SdfEquivalence checks the comparison itself; Drives runs the frozen and the generated rover side by
side in Gazebo. When the rover changes on purpose, refreeze the fixtures from gen_rover.py in the same commit and say
why (fixtures/rover_model/README.md)."""
import contextlib
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from sdf_equivalence import differences
from simulate import simulate
from worldfiles import SIM_DIR

import gen_rover  # noqa: E402  (worldfiles puts sim/ and sim/tools on the path)
from rover_config import Variant  # noqa: E402
from urc import sdf  # noqa: E402

FIXTURES = SIM_DIR / "tests" / "fixtures" / "rover_model"
V = Variant()
# The frozen models and the variants of rover.yaml and sim.yaml that are the same rovers (README.md beside them).
VARIANTS = {
    "default": V,
    "diffdrive": V.replace(drivetrain={"mode": "diffdrive"}),
    "diffdrive_square": V.replace(drivetrain={"mode": "diffdrive"}, diffdrive={"mu_lateral": 1.0}),
    "diffdrive_dust": V.replace(drivetrain={"mode": "diffdrive"}, dust={"enabled": True}),
    "dust": V.replace(dust={"enabled": True}),
    "cues_off": V.replace(tread_tyre=False, dig_sink={"enabled": False}, ruts={"enabled": False}),
    "tyre_compliance": V.replace(tyre_compliance={"enabled": True}),
    "test_physical": V.replace(
        drivetrain={"cmd_timeout_clock": "sim", "accel": 0.0, "state_rate": 1000.0, "cmd_timeout": 0.0},
        contact={"dig": False, "mu_noise": 0.0, "default_surface": "sand"}, driveline={"backlash": 0.0},
        motor={"current_limit": 10.0}, ruts={"tracks": {"report": 1.0}}),
    "same_surfaces": V.replace(contact={"default_surface": "manmade"}),
    "unknown_surface": V.replace(contact={"default_surface": "lunar_dust"}),
}
WALL_CLOCK = "<cmd_timeout_clock>wall</cmd_timeout_clock>"
SIM_CLOCK = "<cmd_timeout_clock>sim</cmd_timeout_clock>"


def fixture(name):
    return (FIXTURES / f"{name}.sdf").read_text()


class SdfEquivalence(unittest.TestCase):
    """differences() finds what changes a model and nothing else."""

    @classmethod
    def setUpClass(cls):
        cls.reference = fixture("default")

    def changed(self, edit):
        root = ET.fromstring(self.reference)
        edit(root.find("model"))
        return differences(self.reference, sdf.document(root))

    def test_a_model_equals_itself(self):
        self.assertEqual(differences(self.reference, self.reference), [])

    def test_a_number_beyond_the_tolerance_differs(self):
        def heavier(model):
            mass = model.find("link[@name='wheel_fl']/inertial/mass")
            mass.text = sdf.fmt(float(mass.text) * (1 + 1e-6))
        self.assertEqual(len(self.changed(heavier)), 1)

    def test_rounding_within_the_tolerance_does_not(self):
        def rounded(model):
            ixx = model.find("link[@name='wheel_fl']/inertial/inertia/ixx")
            ixx.text = repr(float(ixx.text) * (1 + 1e-12))
        self.assertEqual(self.changed(rounded), [])

    def test_a_missing_plugin_parameter_differs(self):
        def without_ki(model):
            controller = model.find("plugin[@filename='RoverDrivetrain']/controller")
            controller.remove(controller.find("ki"))
        self.assertEqual(len(self.changed(without_ki)), 1)

    def test_the_plugins_order_counts(self):
        def swapped(model):
            plugins = model.findall("plugin")
            for plugin in plugins[2:4]:
                model.remove(plugin)
            model.extend([plugins[3], plugins[2]])
        self.assertTrue(any("plugin order" in line for line in self.changed(swapped)))

    def test_a_default_written_out_is_the_same(self):
        def explicit(model):
            axis = model.find("joint[@name='wheel_fl_joint']/axis")
            sdf.sub(sdf.sub(axis, "dynamics"), "spring_stiffness", 0.0)
        self.assertEqual(self.changed(explicit), [])

    def test_a_pose_compares_its_rotation_not_its_angles(self):
        path = "link[@name='rocker_left']/collision[@name='front_arm_collision']/pose"
        x, y, z, _, pitch, _ = (float(v) for v in ET.fromstring(self.reference).find(f"model/{path}").text.split())

        def turned(rpy):
            return lambda model: setattr(model.find(path), "text", sdf.fmt((x, y, z, *rpy)))

        self.assertEqual(self.changed(turned((np.pi, np.pi - pitch, np.pi))), [])  # the same rotation
        self.assertEqual(len(self.changed(turned((0.0, pitch + 1e-6, 0.0)))), 1)

    def test_an_element_only_one_side_has_differs(self):
        def extra(model):
            sdf.sub(model.find("link[@name='base_link']"), "self_collide", True)
        self.assertEqual(len(self.changed(extra)), 1)


class Equivalence(unittest.TestCase):
    """Each generated variant is its frozen model: links' mass, inertia and pose; joints' type, axis, limits,
    damping and springs; collisions, visuals and materials; sensors; every plugin and parameter, in order."""

    def test_the_generated_rovers_are_the_frozen_ones(self):
        self.assertEqual(sorted(p.stem for p in FIXTURES.glob("*.sdf")), sorted(VARIANTS))
        for name, variant in VARIANTS.items():
            with self.subTest(name):
                self.assertEqual(differences(fixture(name), gen_rover.build_sdf(variant)), [])

    def test_the_rover_is_the_tracked_model(self):
        """The tracked model://rover is the default rover (the comparison, not the bytes: gen_rover.py lays it out
        as the converter does)."""
        tracked = (SIM_DIR / "models" / "rover" / "model.sdf").read_text()
        self.assertEqual(differences(fixture("default"), tracked), [])


@contextlib.contextmanager
def model_uri(text):
    """A temporary model directory holding the SDF document `text`, its command timeout on sim time (so runs repeat
    exactly)."""
    with tempfile.TemporaryDirectory(prefix="rover_equivalence_") as directory:
        model = Path(directory) / "rover"
        model.mkdir()
        assert text.count(WALL_CLOCK) <= 1
        (model / "model.sdf").write_text(text.replace(WALL_CLOCK, SIM_CLOCK))
        (model / "model.config").write_text(sdf.model_config("rover", "A rover to compare.",
                                                             "sim/tests/test_equivalence.py"))
        yield model.as_uri()


class Drives(unittest.TestCase):
    """The frozen and the generated rover drive the same 3 s schedule (forward, an arc, a turn in place) on flat
    ground to the same base_link trace, step for step, and the same rocker angles, bit for bit: the physical rover,
    the one with tyre compliance and DiffDrive's."""
    SCHEDULE = [(0.0, 0.0, 0.0), (0.5, 0.5, 0.3), (2.0, 0.0, 1.0)]

    def run_rover(self, text):
        with model_uri(text) as uri:
            return simulate(3.0, cmd=self.SCHEDULE, rover_uri=uri, trace_every=1)

    def test_bit_for_bit(self):
        for name in ("default", "tyre_compliance", "diffdrive"):
            with self.subTest(name):
                old = self.run_rover(fixture(name))
                new = self.run_rover(gen_rover.build_sdf(VARIANTS[name]))
                self.assertGreater(abs(old.poses["base_link"][0]), 0.3)  # it drove
                np.testing.assert_array_equal(old.trace, new.trace)
                self.assertEqual(old.rockers, new.rockers)
                self.assertEqual(old.poses, new.poses)


if __name__ == "__main__":
    unittest.main()
```

(It adds `model_uri`, which writes a model into a temporary folder with its command timeout on sim time, so runs repeat exactly, and `Drives`.)

- [ ] **Step 2: Run it**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run build 2>&1 | tail -1
cd rover_sim/tests && GZ_IP=127.0.0.1 /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_equivalence 2>&1 | grep -E "\.\.\. |^Ran|^OK|^FAIL"
```

Expected: `Summary: 3 packages finished`; eleven lines ending `ok` (the `Drives` line also carries Gazebo's `gz_frame_id` warnings), `Ran 11 tests`, `OK`. The six Gazebo runs take a few seconds.

- [ ] **Step 3: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/tests/test_equivalence.py && git commit -q -F - <<'EOF'
PR 2: rover_sim tests: the generated rover drives as gen_model.py's, bit for bit

test_equivalence.Drives runs the frozen and the generated model for 3 s on flat ground (forward, an arc, a turn in
place) and requires the same base_link trace, step for step, and the same rocker angles: the physical rover, the one
with tyre compliance and DiffDrive's.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

---

## Part C: gen_model.py removed, every consumer on the description

### Task 11: sim-model builds the rover from the description; the model tests move to it

**Files:**
- Move and modify: `rover_sim/tests/test_gen_model.py` -> `rover_sim/tests/test_rover_model.py`
- Modify: `rover_sim/viewers.py`, `rover_sim/models/rover/model.config`, `rover_sim/tests/test_foundations.py`, `pixi.toml` (`sim-model`, `sim-worlds` inputs)
- Regenerate: `rover_sim/models/rover/model.sdf`, `rover_sim/models/rover/meshes/wheel.glb`, `rover_sim/models/{chase_camera,eye_camera,fly_camera}/model.config`

The model tests keep every class and name, and every assertion but where what it tests moved (Decision 15), and build the rover with `gen_rover.build_sdf(Variant)`; their numbers come from `rover_config.ROVER` and `SIM` (`P.wheel_radius` -> `ROVER.wheel.radius`, `P.drive.dust` -> `SIM.dust.enabled`), the inertia formulas are checked on `inertials.xacro`'s macros, the tread mesh on `gen_wheel_mesh.py`, and `particle_scatter_ratio` is compared as the float32 it is in SDF (the converter writes 9.99999997e-07 for 1e-06; Gazebo reads both as the same float). Two tests are new: `config/surfaces.yaml` and the mesh copy are fresh. `viewers.py` takes the eye camera's mount, start and limits from `ROVER`. `sim-model` runs `tools/gen_rover.py`, so every task that depends on it (`sim`, `drive`, `launcher`, `sim-test`, `sim-maps`, `sim-perf`, `sim-slow`, `sim-worlds`) builds the rover from the description; `sim-worlds` depends on `rover_config.py` and `rover.yaml` instead of `gen_model.py` (the proving ground reads the rover's track, wheelbase, wheel radius and rocker limit).

- [ ] **Step 1: Apply the change**

Save this patch as `$T/t11.patch`:

```diff
diff --git a/pixi.toml b/pixi.toml
index 2919e72..8b1e2f4 100644
--- a/pixi.toml
+++ b/pixi.toml
@@ -29,5 +29,7 @@ sim-media = "python rover_sim/tools/gz_media.py || echo 'sim-media: no patched m
 sim-build = { cmd = "colcon build --packages-select rover_sim", cwd = "..", depends-on = ["_workspace", "sim-media"] }
-sim-model = "python rover_sim/gen_model.py"
+# The rover's model (rover_sim.urdf.xacro through xacro and gz sdf), its surfaces, sprite and mesh, and the station's
+# cameras.
+sim-model = "python rover_sim/tools/gen_rover.py"
 sim-relief = { cmd = "python rover_sim/tools/make_relief_swatches.py", inputs = ["rover_sim/tools/make_relief_swatches.py", "rover_sim/urc/dem.py", "rover_sim/urc/landscape.py", "rover_sim/urc/terrain.py", "rover_sim/data/dem/*_0p5m*", "rover_sim/data/research/mdrs_terrain_measurements.json"], outputs = ["rover_sim/data/relief/*.npz"] }
-sim-worlds = { cmd = "python rover_sim/gen_worlds.py", depends-on = ["sim-model"], inputs = ["rover_sim/gen_worlds.py", "rover_sim/gen_model.py", "rover_sim/urc/**/*.py", "rover_sim/data/dem/*", "rover_sim/data/imagery/*_naip2024.*", "rover_sim/data/imagery/naip2021_far_60km.*", "rover_sim/data/relief/*", "rover_sim/data/soils/*", "rover_sim/data/research/terrain_targets.json"], outputs = ["rover_sim/worlds/urc_*.sdf", "rover_sim/worlds/urc_*.json", "rover_sim/worlds/proving_ground.sdf", "rover_sim/worlds/proving_ground.json", "rover_sim/models/urc_*/**"] }
+sim-worlds = { cmd = "python rover_sim/gen_worlds.py", depends-on = ["sim-model"], inputs = ["rover_sim/gen_worlds.py", "rover_sim/rover_config.py", "rover_description/config/rover.yaml", "rover_sim/urc/**/*.py", "rover_sim/data/dem/*", "rover_sim/data/imagery/*_naip2024.*", "rover_sim/data/imagery/naip2021_far_60km.*", "rover_sim/data/relief/*", "rover_sim/data/soils/*", "rover_sim/data/research/terrain_targets.json"], outputs = ["rover_sim/worlds/urc_*.sdf", "rover_sim/worlds/urc_*.json", "rover_sim/worlds/proving_ground.sdf", "rover_sim/worlds/proving_ground.json", "rover_sim/models/urc_*/**"] }
 # Orthophoto maps of the worlds for the station's Map view (needs the GPU, so not part of sim-test).
diff --git a/rover_sim/models/rover/model.config b/rover_sim/models/rover/model.config
index e31f1f1..512caee 100644
--- a/rover_sim/models/rover/model.config
+++ b/rover_sim/models/rover/model.config
@@ -7,3 +7,4 @@
     Generic 4-wheel rocker-differential skid-steer rover.
-    model.sdf is generated by sim/gen_model.py; edit Params there.
+    model.sdf is generated by rover_sim/tools/gen_rover.py (pixi run sim-model) from
+    rover_description/config/rover.yaml and rover_sim/config/sim.yaml: edit those.
   </description>
diff --git a/rover_sim/tests/test_foundations.py b/rover_sim/tests/test_foundations.py
index e283dfc..b77ca04 100644
--- a/rover_sim/tests/test_foundations.py
+++ b/rover_sim/tests/test_foundations.py
@@ -25,2 +25,3 @@ import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
 import gzenv  # noqa: E402
+import rover_config  # noqa: E402
 import viewers  # noqa: E402
@@ -213,7 +214,9 @@ class Catalogue(unittest.TestCase):
 class Viewers(unittest.TestCase):
-    def test_gen_model_finds_the_viewer_names(self):
-        for name in gen_model.VIEWER_NAMES:
-            self.assertIs(getattr(gen_model, name), getattr(viewers, name), name)
-        with self.assertRaises(AttributeError):
-            gen_model.no_such_name  # noqa: B018
+    def test_the_viewer_names_live_in_viewers(self):
+        """The station's cameras have one home, viewers.py; the rover's names are rover_config's."""
+        for name in ("CHASE_MODEL", "CHASE_IMAGE_TOPIC", "CHASE_CMD_TOPIC", "CHASE_MODE_TOPIC", "CHASE_STATE_TOPIC",
+                     "EYE_MODEL", "EYE_IMAGE_TOPIC", "EYE_LOOK_TOPIC", "EYE_STATE_TOPIC", "ChaseParams", "EyeParams",
+                     "build_chase_sdf", "build_eye_sdf"):
+            self.assertTrue(hasattr(viewers, name), name)
+            self.assertFalse(hasattr(rover_config, name), name)
 
diff --git a/rover_sim/tests/test_gen_model.py b/rover_sim/tests/test_rover_model.py
similarity index 65%
rename from rover_sim/tests/test_gen_model.py
rename to rover_sim/tests/test_rover_model.py
index 5f8c40b..3cbda2a 100644
--- a/rover_sim/tests/test_gen_model.py
+++ b/rover_sim/tests/test_rover_model.py
@@ -1,5 +1,7 @@
 #!/usr/bin/env python3
-"""Unit tests for sim/gen_model.py (no physics; pixi run sim-test)."""
+"""Unit tests for tools/gen_rover.py, the rover's Gazebo model built from rover_description and config/sim.yaml
+(no physics; pixi run sim-test). test_equivalence checks that it is the rover gen_model.py wrote."""
 import copy
 import dataclasses
+import importlib.util
 import json
@@ -14,2 +16,3 @@ from unittest import mock
 import numpy as np
+import xacro
 from PIL import Image
@@ -18,15 +21,20 @@ from worldfiles import MODELS, gz_check, temp_sdf, vec
 
-import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
+import gen_rover  # noqa: E402  (worldfiles puts sim/ and sim/tools on the path)
+import rover_config  # noqa: E402
+import viewers  # noqa: E402
+from rover_config import ROVER, SIM, Variant  # noqa: E402
 from urc import sdf, terrains  # noqa: E402
 
-P = gen_model.Params()
-DIFFDRIVE = dataclasses.replace(P, drive=gen_model.DriveParams(mode="diffdrive"))
-DUST = dataclasses.replace(P, drive=dataclasses.replace(P.drive, dust=True))  # the opt-in dust (DriveParams.dust)
+V = Variant()
+DIFFDRIVE = V.replace(drivetrain={"mode": "diffdrive"})
+DUST = V.replace(dust={"enabled": True})  # the opt-in dust (sim.yaml dust)
 DUST_PLUGIN_TAGS = ("dust_rate", "dust", "dust_rule")  # the drivetrain's dust elements
-# The tyre's dig-in cues off (Params.tread_tyre, DriveParams.dig_sink): the rover before them, but for the ruts
-# (their own switch, DriveParams.ruts).
-CUES_OFF = dataclasses.replace(P, tread_tyre=False, drive=dataclasses.replace(P.drive, dig_sink=False))
-NO_RUTS = dataclasses.replace(P, drive=dataclasses.replace(P.drive, ruts=False))  # DriveParams.ruts off
-WHEELS = [f"wheel_{e}{s}" for _, s, _ in gen_model.SIDES for _, e, _ in gen_model.ENDS]
-SIM = Path(gen_model.__file__).resolve().parent
+# The tyre's dig-in cues off (sim.yaml tread_tyre, dig_sink): the rover before them, but for the ruts (their own
+# switch, sim.yaml ruts).
+CUES_OFF = V.replace(tread_tyre=False, dig_sink={"enabled": False})
+NO_RUTS = V.replace(ruts={"enabled": False})  # sim.yaml ruts off
+WHEELS = [f"wheel_{e}{s}" for _, s, _ in rover_config.SIDES for _, e, _ in rover_config.ENDS]
+SIM_DIR = rover_config.SIM_DIR
+INERTIALS = rover_config.DESCRIPTION_DIR / "urdf" / "inertials.xacro"
+WHEEL_MESH_GENERATOR = rover_config.DESCRIPTION_DIR / "meshes" / "gen_wheel_mesh.py"
 
@@ -38,2 +46,23 @@ def canonical(element):
 
+def inertia(macro, **params):
+    """The principal moments (ixx, iyy, izz) that a macro of rover_description's urdf/inertials.xacro gives for
+    `params` (xacro expressions as text)."""
+    attributes = " ".join(f'{key}="{value}"' for key, value in params.items())
+    with tempfile.TemporaryDirectory() as tmp:
+        document = Path(tmp) / "inertia.xacro"
+        document.write_text(f'<robot name="t" xmlns:xacro="http://www.ros.org/wiki/xacro">'
+                            f'<xacro:include filename="{INERTIALS}"/>'
+                            f'<link name="l"><xacro:{macro} {attributes}/></link></robot>')
+        element = ET.fromstring(xacro.process_file(str(document)).toxml()).find("link/inertial/inertia")
+    return tuple(float(element.get(key)) for key in ("ixx", "iyy", "izz"))
+
+
+def wheel_mesh_generator():
+    """rover_description/meshes/gen_wheel_mesh.py, imported from its file."""
+    spec = importlib.util.spec_from_file_location("gen_wheel_mesh", WHEEL_MESH_GENERATOR)
+    module = importlib.util.module_from_spec(spec)
+    spec.loader.exec_module(module)
+    return module
+
+
 class Inertia(unittest.TestCase):
@@ -41,5 +70,6 @@ class Inertia(unittest.TestCase):
         self.assertEqual(sdf.box_inertia(12.0, (1.0, 2.0, 3.0)), (13.0, 10.0, 5.0))
+        self.assertEqual(inertia("box_inertial", mass="12.0", size="${[1.0, 2.0, 3.0]}"), (13.0, 10.0, 5.0))
 
     def test_wheel_axis_is_y(self):
-        ixx, iyy, izz = gen_model.wheel_inertia(2.0, 0.5, 1.0)
+        ixx, iyy, izz = inertia("wheel_inertial", mass="2.0", radius="0.5", width="1.0")
         self.assertAlmostEqual(iyy, 2.0 * 0.5**2 / 2)
@@ -50,3 +80,4 @@ class Inertia(unittest.TestCase):
         # Rods to (+-1, 0, 0) form one rod of length 2 about its middle.
-        self.assertEqual(gen_model.rocker_inertia(2.0, 1.0, 0.0), (0.0, 2.0 * 4 / 12, 2.0 * 4 / 12))
+        self.assertEqual(inertia("rocker_inertial", mass="2.0", dx="1.0", dz="0.0", xyz="0 0 0"),
+                         (0.0, 2.0 * 4 / 12, 2.0 * 4 / 12))
 
@@ -54,3 +85,3 @@ class Inertia(unittest.TestCase):
         # Both rods straight down to (0, 0, -1) overlap: one rod of length 1.
-        ixx, iyy, izz = gen_model.rocker_inertia(2.0, 0.0, -1.0)
+        ixx, iyy, izz = inertia("rocker_inertial", mass="2.0", dx="0.0", dz="-1.0", xyz="0 0 -0.5")
         self.assertAlmostEqual(ixx, 2.0 / 12)
@@ -63,3 +94,3 @@ class Structure(unittest.TestCase):
     def setUpClass(cls):
-        cls.sdf = gen_model.build_sdf(P)
+        cls.sdf = gen_rover.build_sdf(V)
         cls.model = ET.fromstring(cls.sdf).find("model")
@@ -82,3 +113,3 @@ class Structure(unittest.TestCase):
         for joint in self.model.findall("joint"):
-            axis = [0, 0, 1] if joint.get("name") == gen_model.PAN_JOINT else [0, 1, 0]
+            axis = [0, 0, 1] if joint.get("name") == rover_config.PAN_JOINT else [0, 1, 0]
             self.assertEqual(vec(joint.findtext("axis/xyz")), axis, joint.get("name"))
@@ -86,5 +117,7 @@ class Structure(unittest.TestCase):
     def test_wheel_centers_follow_params(self):
-        z = P.pivot_z + P.wheel_dz
-        expected = {"wheel_fl": (P.wheel_dx, P.pivot_y), "wheel_rl": (-P.wheel_dx, P.pivot_y),
-                    "wheel_fr": (P.wheel_dx, -P.pivot_y), "wheel_rr": (-P.wheel_dx, -P.pivot_y)}
+        """The wheel centres of rover.yaml: rocker pivots plus the wheel offsets."""
+        rocker, wheel = ROVER.rocker, ROVER.wheel
+        z = rocker.pivot_z + wheel.dz
+        expected = {"wheel_fl": (wheel.dx, rocker.pivot_y), "wheel_rl": (-wheel.dx, rocker.pivot_y),
+                    "wheel_fr": (wheel.dx, -rocker.pivot_y), "wheel_rr": (-wheel.dx, -rocker.pivot_y)}
         for name, (x, y) in expected.items():
@@ -95,3 +128,3 @@ class Structure(unittest.TestCase):
     def test_wheels_rest_on_the_ground(self):
-        self.assertAlmostEqual(P.pivot_z + P.wheel_dz, P.wheel_radius)
+        self.assertAlmostEqual(ROVER.rocker.pivot_z + ROVER.wheel.dz, ROVER.wheel.radius)
 
@@ -99,4 +132,4 @@ class Structure(unittest.TestCase):
         total = sum(float(m.text) for m in self.model.iter("mass"))
-        self.assertAlmostEqual(total, P.chassis_mass + 2 * P.rocker_mass + 4 * P.wheel_mass + P.camera_pan_mass
-                               + P.camera_tilt_mass)
+        self.assertAlmostEqual(total, ROVER.chassis.mass + 2 * ROVER.rocker.mass + 4 * ROVER.wheel.mass
+                               + ROVER.head.pan_mass + ROVER.head.tilt_mass)
 
@@ -104,3 +137,3 @@ class Structure(unittest.TestCase):
         sensor = self.model.find("link[@name='camera_tilt_link']/sensor[@name='camera']")
-        self.assertEqual(sensor.findtext("topic"), gen_model.CAMERA_TOPIC)
+        self.assertEqual(sensor.findtext("topic"), rover_config.CAMERA_TOPIC)
         # The tilt axis runs through the camera, the pan axis vertically below it.
@@ -108,12 +141,12 @@ class Structure(unittest.TestCase):
         pan = vec(self.model.find("link[@name='camera_pan_link']/pose").text)
-        self.assertEqual(tilt, [*P.camera_xyz, 0, 0, 0])
+        self.assertEqual(tilt, [*ROVER.camera.xyz, 0, 0, 0])
         self.assertEqual(pan[:2], tilt[:2])
-        for joint, start in ((gen_model.PAN_JOINT, 0.0), (gen_model.TILT_JOINT, P.camera_pitch)):
+        for joint, start in ((rover_config.PAN_JOINT, 0.0), (rover_config.TILT_JOINT, ROVER.head.home_tilt)):
             controller = self.model.find(f"plugin[joint_name='{joint}']")
             self.assertEqual(controller.get("name"), "gz::sim::systems::JointPositionController")
-            self.assertEqual(controller.findtext("topic"), gen_model.HEAD_TOPIC.format(joint=joint))
+            self.assertEqual(controller.findtext("topic"), rover_config.HEAD_TOPIC.format(joint=joint))
             self.assertEqual(controller.findtext("use_velocity_commands"), "true")
             self.assertAlmostEqual(float(controller.findtext("initial_position")), start)
-        lower, upper = P.camera_tilt_limits
-        self.assertTrue(lower < P.camera_pitch < upper)
+        lower, upper = ROVER.head.tilt_limits
+        self.assertTrue(lower < ROVER.head.home_tilt < upper)
 
@@ -124,3 +157,3 @@ class Structure(unittest.TestCase):
         self.assertEqual(diff.findtext("right_joint"), "rocker_right_joint")
-        self.assertAlmostEqual(float(diff.findtext("stiffness")), P.diff_stiffness)
+        self.assertAlmostEqual(float(diff.findtext("stiffness")), ROVER.differential.stiffness)
         self.assertIsNone(self.model.find(".//mimic"))
@@ -134,3 +167,3 @@ class Structure(unittest.TestCase):
         depth = camera.find("depth_camera/clip")
-        self.assertEqual((float(depth.findtext("near")), float(depth.findtext("far"))), P.camera_clip)
+        self.assertEqual((float(depth.findtext("near")), float(depth.findtext("far"))), ROVER.camera.clip)
         self.assertIsNone(camera.find("noise"))
@@ -146,8 +179,7 @@ class TrackedModel(unittest.TestCase):
     def test_the_default_rover_is_physical_and_tracked(self):
-        """The default rover (DriveParams.mode "physical", design spec D22;
-        no dust, the user's decision of 2026-10-07) is the tracked
-        models/rover/model.sdf, byte for byte."""
-        self.assertEqual(P.drive.mode, "physical")
-        self.assertFalse(P.drive.dust)
-        self.assertEqual(gen_model.build_sdf(P), (MODELS / "rover" / "model.sdf").read_text())
+        """The default rover (sim.yaml drivetrain mode physical, design spec D22; no dust, the user's decision of
+        2026-10-07) is the tracked models/rover/model.sdf, byte for byte."""
+        self.assertEqual(SIM.drivetrain.mode, "physical")
+        self.assertFalse(SIM.dust.enabled)
+        self.assertEqual(gen_rover.build_sdf(V), (MODELS / "rover" / "model.sdf").read_text())
 
@@ -155,3 +187,12 @@ class TrackedModel(unittest.TestCase):
         with self.assertRaises(ValueError):
-            gen_model.build_sdf(dataclasses.replace(P, drive=gen_model.DriveParams(mode="servo")))
+            gen_rover.build_sdf(V.replace(drivetrain={"mode": "servo"}))
+
+    def test_the_surfaces_are_the_catalogues(self):
+        """The tracked config/surfaces.yaml is what gen_rover.py writes from urc/terrains.py."""
+        self.assertEqual((SIM_DIR / "config" / "surfaces.yaml").read_text(), gen_rover.surfaces_text())
+
+    def test_the_tread_mesh_is_the_descriptions(self):
+        """The rover model's tread tyre is a copy of rover_description's (main() copies it)."""
+        self.assertEqual((MODELS / "rover" / rover_config.WHEEL_MESH).read_bytes(),
+                         gen_rover.DESCRIPTION_MESH.read_bytes())
 
@@ -159,5 +200,4 @@ class TrackedModel(unittest.TestCase):
 class DiffDriveVariant(unittest.TestCase):
-    """DriveParams(mode="diffdrive"): Gazebo's DiffDrive on anisotropic tyres,
-    kept for A/B tests and cost comparisons (design spec D22); everything but
-    the drive is the default rover's."""
+    """sim.yaml drivetrain mode diffdrive: Gazebo's DiffDrive on anisotropic tyres, kept for A/B tests and cost
+    comparisons (design spec D22); everything but the drive is the default rover's."""
 
@@ -165,3 +205,3 @@ class DiffDriveVariant(unittest.TestCase):
     def setUpClass(cls):
-        cls.model = ET.fromstring(gen_model.build_sdf(DIFFDRIVE)).find("model")
+        cls.model = ET.fromstring(gen_rover.build_sdf(DIFFDRIVE)).find("model")
 
@@ -171,5 +211,5 @@ class DiffDriveVariant(unittest.TestCase):
         self.assertEqual([e.text for e in drive.findall("right_joint")], ["wheel_fr_joint", "wheel_rr_joint"])
-        self.assertAlmostEqual(float(drive.findtext("wheel_separation")), 2 * P.pivot_y)
-        self.assertAlmostEqual(float(drive.findtext("wheel_radius")), P.wheel_radius)
-        self.assertEqual(drive.findtext("topic"), gen_model.CMD_VEL_TOPIC)
+        self.assertAlmostEqual(float(drive.findtext("wheel_separation")), 2 * ROVER.rocker.pivot_y)
+        self.assertAlmostEqual(float(drive.findtext("wheel_radius")), ROVER.wheel.radius)
+        self.assertEqual(drive.findtext("topic"), rover_config.CMD_VEL_TOPIC)
         self.assertIsNone(self.model.find("plugin[@filename='RoverDrivetrain']"))  # never both (D22)
@@ -179,4 +219,4 @@ class DiffDriveVariant(unittest.TestCase):
             self.assertEqual(vec(ode.findtext("fdir1")), [0, 0, 1])  # the axle
-            self.assertAlmostEqual(float(ode.findtext("mu")), P.mu_lateral)
-            self.assertAlmostEqual(float(ode.findtext("mu2")), P.mu_longitudinal)
+            self.assertAlmostEqual(float(ode.findtext("mu")), SIM.diffdrive.mu_lateral)
+            self.assertAlmostEqual(float(ode.findtext("mu2")), SIM.diffdrive.mu_longitudinal)
         self.assertEqual(len(list(self.model.iter("ode"))), 4)
@@ -184,3 +224,3 @@ class DiffDriveVariant(unittest.TestCase):
             if joint.get("name").startswith("wheel_"):
-                self.assertEqual(float(joint.findtext("axis/limit/effort")), P.wheel_effort)
+                self.assertEqual(float(joint.findtext("axis/limit/effort")), SIM.diffdrive.wheel_effort)
 
@@ -188,4 +228,4 @@ class DiffDriveVariant(unittest.TestCase):
         """Camera, sensors and links are the default rover's, and so are the
-        dust emitters: none by default, the same two with DriveParams.dust."""
-        default = ET.fromstring(gen_model.build_sdf(P)).find("model")
+        dust emitters: none by default, the same two with sim.yaml's dust on."""
+        default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
         for path in ("link[@name='camera_tilt_link']/sensor", "link[@name='base_link']/sensor[@name='gnss']"):
@@ -195,7 +235,7 @@ class DiffDriveVariant(unittest.TestCase):
 
-        def emitters(params):
-            return [ET.tostring(e) for e in ET.fromstring(gen_model.build_sdf(params)).iter("particle_emitter")]
+        def emitters(variant):
+            return [ET.tostring(e) for e in ET.fromstring(gen_rover.build_sdf(variant)).iter("particle_emitter")]
 
         self.assertEqual(emitters(DIFFDRIVE), [])
-        dusty = emitters(dataclasses.replace(DIFFDRIVE, drive=dataclasses.replace(DIFFDRIVE.drive, dust=True)))
+        dusty = emitters(DIFFDRIVE.replace(dust={"enabled": True}))
         self.assertEqual(len(dusty), 2)
@@ -210,4 +250,3 @@ class PhysicalVariant(unittest.TestCase):
     def setUpClass(cls):
-        cls.params = P
-        cls.sdf = gen_model.build_sdf(cls.params)
+        cls.sdf = gen_rover.build_sdf(V)
         cls.model = ET.fromstring(cls.sdf).find("model")
@@ -218,13 +257,13 @@ class PhysicalVariant(unittest.TestCase):
         self.assertEqual(self.plugin.get("filename"), "RoverDrivetrain")
-        d = self.params.drive
-        expect = {"topic": gen_model.CMD_VEL_TOPIC, "odom_topic": gen_model.ODOM_TOPIC, "tf_topic": gen_model.TF_TOPIC,
-                  "state_topic": gen_model.DRIVETRAIN_TOPIC, "frame_id": "odom", "child_frame_id": "base_link",
-                  "cmd_timeout_clock": "wall"}
+        expect = {"topic": rover_config.CMD_VEL_TOPIC, "odom_topic": rover_config.ODOM_TOPIC,
+                  "tf_topic": rover_config.TF_TOPIC, "state_topic": rover_config.DRIVETRAIN_TOPIC,
+                  "frame_id": "odom", "child_frame_id": "base_link", "cmd_timeout_clock": "wall"}
         for tag, value in expect.items():
             self.assertEqual(self.plugin.findtext(tag), value, tag)
-        numbers = {"cmd_timeout": 0.5, "track": 2 * P.pivot_y, "radius": P.wheel_radius, "track_multiplier": 1.0,
-                   "motor/gear": 50.0, "motor/efficiency": 0.8, "motor/current_limit": 20.0, "motor/voltage": 48.0,
-                   "driveline/backlash": d.backlash, "controller/kp": 4.0, "controller/ki": 160.0,
-                   "controller/accel": d.accel, "controller/max_speed": P.wheel_speed,
-                   "contact/v_stribeck": d.v_stribeck, "contact/stick_perp_ratio": 0.3, "contact/perp_ratio": 0.0}
+        numbers = {"cmd_timeout": 0.5, "track": 2 * ROVER.rocker.pivot_y, "radius": ROVER.wheel.radius,
+                   "track_multiplier": 1.0, "motor/gear": 50.0, "motor/efficiency": 0.8,
+                   "motor/current_limit": 20.0, "motor/voltage": 48.0, "driveline/backlash": ROVER.driveline.backlash,
+                   "controller/kp": 4.0, "controller/ki": 160.0, "controller/accel": SIM.drivetrain.accel,
+                   "controller/max_speed": ROVER.wheel.top_speed, "contact/v_stribeck": SIM.contact.v_stribeck,
+                   "contact/stick_perp_ratio": 0.3, "contact/perp_ratio": 0.0}
         for tag, value in numbers.items():
@@ -244,4 +283,3 @@ class PhysicalVariant(unittest.TestCase):
         def contact(dig):
-            drive = gen_model.DriveParams(dig=dig)
-            model = ET.fromstring(gen_model.build_sdf(dataclasses.replace(P, drive=drive))).find("model")
+            model = ET.fromstring(gen_rover.build_sdf(V.replace(contact={"dig": dig}))).find("model")
             return model.find("plugin[@filename='RoverDrivetrain']/contact")
@@ -254,11 +292,11 @@ class PhysicalVariant(unittest.TestCase):
     def test_surface_rows_come_from_the_catalogue(self):
-        """The default and object surfaces' traction under the catalogue's
-        dig-in preset (terrains.DIG), as the worlds' ground.json, for worlds
-        without a ground map."""
-        model = ET.fromstring(gen_model.build_sdf(dataclasses.replace(
-            P, drive=gen_model.DriveParams(default_surface="sand")))).find("model")
+        """Every catalogue type's traction under the catalogue's dig-in
+        preset (terrains.DIG), as the worlds' ground.json, and the model's
+        rows for its default and object surfaces, for worlds without a ground
+        map."""
+        model = ET.fromstring(gen_rover.build_sdf(V.replace(contact={"default_surface": "sand"}))).find("model")
         for dig in ("strong", "mild"):
             with mock.patch.object(terrains, "DIG", dig):
-                rows = {row["key"]: row for row in gen_model.surface_rows("sand", "manmade")}
-            self.assertEqual(set(rows), {"sand", "manmade"})
+                rows = gen_rover.surface_rows()
+            self.assertEqual(set(rows), set(terrains.TYPES))
             for key, row in rows.items():
@@ -270,2 +308,3 @@ class PhysicalVariant(unittest.TestCase):
                    for row in model.findall("plugin[@filename='RoverDrivetrain']/contact/surface")}
+        self.assertEqual(set(written), {"sand", "manmade"})
         self.assertEqual(float(written["sand"].findtext("dig_max")), terrains.STRONG_DIG[1])  # the default, strong
@@ -292,4 +331,4 @@ class PhysicalVariant(unittest.TestCase):
 class Dust(unittest.TestCase):
-    """DriveParams.dust, the one switch for the rover's wheel dust (design
-    spec 6.5, D15). Off by default (the user's decision of 2026-10-07): in
+    """sim.yaml dust, the one switch for the rover's wheel dust (design spec
+    6.5, D15). Off by default (the user's decision of 2026-10-07): in
     gz-rendering 8.2.2 the depth image and point cloud see any visible
@@ -302,5 +341,5 @@ class Dust(unittest.TestCase):
     def setUpClass(cls):
-        cls.default = ET.fromstring(gen_model.build_sdf(P)).find("model")
+        cls.default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
         cls.tracked = ET.parse(MODELS / "rover" / "model.sdf").getroot().find("model")
-        cls.sdf = gen_model.build_sdf(DUST)
+        cls.sdf = gen_rover.build_sdf(DUST)
         cls.model = ET.fromstring(cls.sdf).find("model")
@@ -313,4 +352,4 @@ class Dust(unittest.TestCase):
         elements), while the same rover with the switch on has both."""
-        self.assertFalse(P.drive.dust)
-        diffdrive = ET.fromstring(gen_model.build_sdf(DIFFDRIVE)).find("model")
+        self.assertFalse(SIM.dust.enabled)
+        diffdrive = ET.fromstring(gen_rover.build_sdf(DIFFDRIVE)).find("model")
         for name, model in (("generated", self.default), ("tracked", self.tracked), ("diffdrive", diffdrive)):
@@ -343,3 +382,3 @@ class Dust(unittest.TestCase):
         does not apply: the sprite carries the dust's colour and opacity
-        (DriveParams.dust_alpha). A scatter ratio near none but above 0: without
+        (sim.yaml dust alpha). A scatter ratio near none but above 0: without
         one, or at 0, a gz-rendering that honours it applies 0.65 and depth sees
@@ -347,2 +386,3 @@ class Dust(unittest.TestCase):
         written with the model whether or not the switch is on."""
+        wheel = ROVER.wheel
         for side, s in (("left", "l"), ("right", "r")):
@@ -354,18 +394,19 @@ class Dust(unittest.TestCase):
             self.assertIsNotNone(ratio, "particle_scatter_ratio")  # without one: SDF's 0.65
-            self.assertEqual(float(ratio), P.drive.dust_scatter_ratio)
+            # An SDF float: the converter writes its float32 value, which Gazebo reads as it read 1e-06.
+            self.assertEqual(np.float32(ratio), np.float32(SIM.dust.scatter_ratio))
             self.assertTrue(0 < float(ratio) <= 1e-3, ratio)
             self.assertEqual(vec(emitter.findtext("material/diffuse")), [1.0, 1.0, 1.0, 1.0])
-            topic = gen_model.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{s}")
+            topic = rover_config.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{s}")
             self.assertEqual(emitter.findtext("topic"), topic)
             x, _, z = vec(emitter.findtext("pose"))[:3]
-            self.assertLess(x, -(P.wheel_dx + P.wheel_radius))  # behind the rear tyre
-            self.assertAlmostEqual(z - (P.wheel_dz - P.wheel_radius), P.drive.dust_box / 2)  # on the ground
+            self.assertLess(x, -(wheel.dx + wheel.radius))  # behind the rear tyre
+            self.assertAlmostEqual(z - (wheel.dz - wheel.radius), SIM.dust.box / 2)  # on the ground
             sprite = emitter.findtext("material/pbr/metal/albedo_map")  # the soft puff, tracked with the model
-            self.assertEqual(sprite, f"model://rover/{gen_model.DUST_SPRITE}")
-            self.assertTrue((MODELS / "rover" / gen_model.DUST_SPRITE).is_file())
+            self.assertEqual(sprite, f"model://rover/{rover_config.DUST_SPRITE}")
+            self.assertTrue((MODELS / "rover" / rover_config.DUST_SPRITE).is_file())
         with tempfile.TemporaryDirectory() as tmp:
-            gen_model.write_dust_sprite(Path(tmp) / "puff.png")
+            gen_rover.write_dust_sprite(Path(tmp) / "puff.png")
             written = np.asarray(Image.open(Path(tmp) / "puff.png"))
-        np.testing.assert_array_equal(np.asarray(Image.open(MODELS / "rover" / gen_model.DUST_SPRITE)), written)
-        self.assertAlmostEqual(written[..., 3].max() / 255, 0.72 * P.drive.dust_alpha, delta=0.02)
+        np.testing.assert_array_equal(np.asarray(Image.open(MODELS / "rover" / rover_config.DUST_SPRITE)), written)
+        self.assertAlmostEqual(written[..., 3].max() / 255, 0.72 * SIM.dust.alpha, delta=0.02)
         np.testing.assert_array_equal(written[64, 64, :3], np.round(np.array(terrains.DUST_RGB) * 255))
@@ -374,11 +415,11 @@ class Dust(unittest.TestCase):
         """The drivetrain commands the rear emitters on their topics, at
-        dust_rate, by the dust rule (DriveParams' gains)."""
-        d = DUST.drive
+        the dust rate, by the dust rule (sim.yaml's gains)."""
+        d = DUST.sim.dust
         for side, s in (("left", "l"), ("right", "r")):
-            topic = gen_model.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{s}")
+            topic = rover_config.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{s}")
             self.assertIn((f"wheel_r{s}", topic), [(e.findtext("wheel"), e.findtext("topic"))
                                                    for e in self.plugin.findall("dust")])
-        numbers = {"dust_rate": d.dust_rate, "dust_rule/speed_gain": d.dust_speed_gain,
-                   "dust_rule/slip_gain": d.dust_slip_gain, "dust_rule/max_rate": d.dust_max,
-                   "dust_rule/min_speed": d.dust_min_speed}
+        numbers = {"dust_rate": d.rate, "dust_rule/speed_gain": d.rule.speed_gain,
+                   "dust_rule/slip_gain": d.rule.slip_gain, "dust_rule/max_rate": d.rule.max_rate,
+                   "dust_rule/min_speed": d.rule.min_speed}
         for tag, value in numbers.items():
@@ -396,5 +437,5 @@ class DigCues(unittest.TestCase):
     cue visual only and behind its own switch, both on by default:
-    DriveParams.dig_sink, the drivetrain drawing a dug wheel's tyre sunk at
-    true scale (<dig_sink>), and Params.tread_tyre, the tyres drawn with the
-    tread mesh (WHEEL_MESH) instead of the plain cylinder."""
+    sim.yaml dig_sink, the drivetrain drawing a dug wheel's tyre sunk at
+    true scale (<dig_sink>), and sim.yaml tread_tyre, the tyres drawn with
+    the tread mesh (rover_config.WHEEL_MESH) instead of the plain cylinder."""
 
@@ -402,3 +443,3 @@ class DigCues(unittest.TestCase):
     def setUpClass(cls):
-        cls.default = ET.fromstring(gen_model.build_sdf(P)).find("model")
+        cls.default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
         cls.tracked = ET.parse(MODELS / "rover" / "model.sdf").getroot().find("model")
@@ -423,5 +464,6 @@ class DigCues(unittest.TestCase):
         still the cylinder; the drivetrain gets <dig_sink> at true scale."""
-        self.assertTrue(P.tread_tyre)
-        self.assertTrue(P.drive.dig_sink)
-        self.assertEqual(P.drive.dig_sink_gain, 1.0)  # true scale, not the research's x3 (user decision)
+        wheel, sink_params = ROVER.wheel, SIM.dig_sink
+        self.assertTrue(SIM.tread_tyre)
+        self.assertTrue(sink_params.enabled)
+        self.assertEqual(sink_params.gain, 1.0)  # true scale, not the research's x3 (user decision)
         for label, model in (("generated", self.default), ("tracked", self.tracked)):
@@ -432,4 +474,4 @@ class DigCues(unittest.TestCase):
                 mesh = visuals[0].find("geometry/mesh")
-                self.assertEqual(mesh.findtext("uri"), f"model://rover/{gen_model.WHEEL_MESH}")
-                self.assertEqual(vec(mesh.findtext("scale")), [P.wheel_radius, P.wheel_width, P.wheel_radius])
+                self.assertEqual(mesh.findtext("uri"), f"model://rover/{rover_config.WHEEL_MESH}")
+                self.assertEqual(vec(mesh.findtext("scale")), [wheel.radius, wheel.width, wheel.radius])
                 self.assertEqual(vec(visuals[0].findtext("pose")), [0.0] * 6)
@@ -438,3 +480,3 @@ class DigCues(unittest.TestCase):
                 self.assertEqual((float(cylinder.findtext("radius")), float(cylinder.findtext("length"))),
-                                 (P.wheel_radius, P.wheel_width))
+                                 (wheel.radius, wheel.width))
             sink = model.find("plugin[@name='rover_sim::RoverDrivetrain']/dig_sink")
@@ -442,4 +484,4 @@ class DigCues(unittest.TestCase):
             self.assertEqual({e.tag: float(e.text) for e in sink},
-                             {"gain": P.drive.dig_sink_gain, "max": P.drive.dig_sink_max, "tau": P.drive.dig_sink_tau})
-        self.assertTrue((MODELS / "rover" / gen_model.WHEEL_MESH).is_file())
+                             {"gain": sink_params.gain, "max": sink_params.max, "tau": sink_params.tau})
+        self.assertTrue((MODELS / "rover" / rover_config.WHEEL_MESH).is_file())
 
@@ -449,9 +491,8 @@ class DigCues(unittest.TestCase):
         else changes, in either drive mode. With both off, and the ruts
-        (DriveParams.ruts, Ruts) too, the rover is the one before the cues
+        (sim.yaml ruts, Ruts) too, the rover is the one before the cues
         (byte for byte the model.sdf of b124f7d, checked when they were built
         and again when they were merged, 2026-10-07)."""
-        plain = ET.fromstring(gen_model.build_sdf(dataclasses.replace(P, tread_tyre=False))).find("model")
-        no_sink = ET.fromstring(gen_model.build_sdf(dataclasses.replace(
-            P, drive=dataclasses.replace(P.drive, dig_sink=False)))).find("model")
-        before = ET.fromstring(gen_model.build_sdf(CUES_OFF)).find("model")
+        plain = ET.fromstring(gen_rover.build_sdf(V.replace(tread_tyre=False))).find("model")
+        no_sink = ET.fromstring(gen_rover.build_sdf(V.replace(dig_sink={"enabled": False}))).find("model")
+        before = ET.fromstring(gen_rover.build_sdf(CUES_OFF)).find("model")
         for name in WHEELS:
@@ -461,3 +502,3 @@ class DigCues(unittest.TestCase):
             self.assertEqual(visual.findtext("pose"), collision.findtext("pose"))
-            self.assertEqual(vec(visual.findtext("material/diffuse")), list(gen_model.TIRE_COLOR))
+            self.assertEqual(vec(visual.findtext("material/diffuse")), list(ROVER.looks.colors.tyre))
         self.assertIsNone(no_sink.find("plugin[@name='rover_sim::RoverDrivetrain']/dig_sink"))
@@ -470,10 +511,10 @@ class DigCues(unittest.TestCase):
             self.assertEqual(canonical(before.find(f"link[@name='{name}']")), canonical(plain.find(f"link[@name='{name}']")))
-        for params in (DIFFDRIVE, CUES_OFF):
-            diffdrive = dataclasses.replace(params, drive=dataclasses.replace(params.drive, mode="diffdrive"))
-            model = ET.fromstring(gen_model.build_sdf(diffdrive)).find("model")
+        for variant in (DIFFDRIVE, CUES_OFF):
+            model = ET.fromstring(gen_rover.build_sdf(variant.replace(drivetrain={"mode": "diffdrive"}))).find("model")
             mesh = model.find("link[@name='wheel_fl']/visual/geometry/mesh")
-            self.assertEqual(mesh is not None, params.tread_tyre)  # the look is the rover's, not the drive's
+            self.assertEqual(mesh is not None, variant.sim.tread_tyre)  # the look is the rover's, not the drive's
 
     def test_tread_mesh(self):
-        """WHEEL_MESH, tracked with the model, is what write_wheel_mesh
+        """The rover model's tread tyre (rover_config.WHEEL_MESH, a copy of
+        rover_description's meshes/wheel.glb) is what gen_wheel_mesh.py
         writes, byte for byte: a cylinder of radius 1 and width 1 round the
@@ -483,5 +524,7 @@ class DigCues(unittest.TestCase):
         nowhere else. The GLB's materials are those colours."""
-        tracked = MODELS / "rover" / gen_model.WHEEL_MESH
+        tracked = MODELS / "rover" / rover_config.WHEEL_MESH
+        generator = wheel_mesh_generator()
+        dark, light, mark = generator.colors()
         with tempfile.TemporaryDirectory() as tmp:
-            gen_model.write_wheel_mesh(Path(tmp) / "wheel.glb")
+            generator.write_glb(Path(tmp) / "wheel.glb", generator.wheel_parts(dark, light, mark))
             self.assertEqual((Path(tmp) / "wheel.glb").read_bytes(), tracked.read_bytes())
@@ -492,7 +535,6 @@ class DigCues(unittest.TestCase):
         colours = [tuple(m["pbrMetallicRoughness"]["baseColorFactor"]) for m in gltf["materials"]]
-        expected = [tuple(gen_model.TREAD_DARK) + (1.0,), tuple(gen_model.TREAD_LIGHT) + (1.0,),
-                    tuple(gen_model.TREAD_MARK) + (1.0,)]
-        np.testing.assert_allclose(colours, expected)
-        self.assertEqual(gen_model.TREAD_DARK, gen_model.TIRE_COLOR[:3])
-        parts = gen_model.wheel_parts()
+        np.testing.assert_allclose(colours, [dark + (1.0,), light + (1.0,), mark + (1.0,)])
+        self.assertEqual(dark, ROVER.looks.colors.tyre[:3])
+        self.assertEqual(mark, ROVER.looks.colors.chassis[:3])
+        parts = generator.wheel_parts(dark, light, mark)
         V = np.concatenate([part[0] for part in parts])
@@ -500,8 +542,8 @@ class DigCues(unittest.TestCase):
         self.assertEqual((V[:, 1].min(), V[:, 1].max()), (-0.5, 0.5))
-        (_, dark_n), (light, light_n), (mark, mark_n) = ((part[0], part[2]) for part in parts)
+        (_, dark_n), (light_v, light_n), (mark_v, mark_n) = ((part[0], part[2]) for part in parts)
         self.assertEqual(int(np.sum(np.abs(light_n[::3, 1]) < 1e-9)), 2 * 16)  # the bars: 16, two triangles each
-        self.assertEqual(len(mark), 3 * 2 * 2 * 2)  # one spoke: two segments, two triangles each, on both faces
-        self.assertEqual(set(np.sign(mark[:, 1])), {-1.0, 1.0})
+        self.assertEqual(len(mark_v), 3 * 2 * 2 * 2)  # one spoke: two segments, two triangles each, on both faces
+        self.assertEqual(set(np.sign(mark_v[:, 1])), {-1.0, 1.0})
         np.testing.assert_allclose(np.abs(mark_n[:, 1]), 1.0)  # on the faces, not the tread
-        rim = mark[np.hypot(mark[:, 0], mark[:, 2]) > 0.99]
+        rim = mark_v[np.hypot(mark_v[:, 0], mark_v[:, 2]) > 0.99]
         self.assertTrue(np.all(np.abs(np.degrees(np.arctan2(rim[:, 2], rim[:, 0]))) <= 7.5 + 1e-9))
@@ -511,4 +553,4 @@ class DigCues(unittest.TestCase):
 class Ruts(unittest.TestCase):
-    """DriveParams.ruts, the one switch for the ruts and pits behind the wheels
-    (gen_model.TrackParams, plugins/rover_tracks.hh): on by default (the
+    """sim.yaml ruts, the one switch for the ruts and pits behind the wheels
+    (its tracks numbers, plugins/rover_tracks.hh): on by default (the
     user's decision of 2026-10-07), written as the drivetrain's <tracks>
@@ -518,5 +560,5 @@ class Ruts(unittest.TestCase):
     def setUpClass(cls):
-        cls.default = ET.fromstring(gen_model.build_sdf(P)).find("model")
+        cls.default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
         cls.tracked = ET.parse(MODELS / "rover" / "model.sdf").getroot().find("model")
-        cls.off = ET.fromstring(gen_model.build_sdf(NO_RUTS)).find("model")
+        cls.off = ET.fromstring(gen_rover.build_sdf(NO_RUTS)).find("model")
 
@@ -524,6 +566,6 @@ class Ruts(unittest.TestCase):
         """The default rover, generated and tracked (model://rover), has one
-        <tracks> in its drivetrain with every TrackParams field and the
-        wheel's width; DiffDrive has no drivetrain and no ruts."""
-        self.assertTrue(P.drive.ruts)
-        expected = {**dataclasses.asdict(P.drive.tracks), "width": P.wheel_width}
+        <tracks> in its drivetrain with every number of sim.yaml's ruts tracks
+        and the wheel's width; DiffDrive has no drivetrain and no ruts."""
+        self.assertTrue(SIM.ruts.enabled)
+        expected = {**SIM.ruts.tracks, "width": ROVER.wheel.width}
         for name, model in (("generated", self.default), ("tracked", self.tracked)):
@@ -533,3 +575,3 @@ class Ruts(unittest.TestCase):
             self.assertEqual(written, {k: float(v) for k, v in expected.items()}, name)
-        diffdrive = ET.fromstring(gen_model.build_sdf(DIFFDRIVE)).find("model")
+        diffdrive = ET.fromstring(gen_rover.build_sdf(DIFFDRIVE)).find("model")
         self.assertEqual(list(diffdrive.iter("tracks")), [])
@@ -540,3 +582,3 @@ class Ruts(unittest.TestCase):
         self.assertEqual(list(self.off.iter("tracks")), [])
-        model = ET.fromstring(gen_model.build_sdf(P)).find("model")  # a copy to strip
+        model = ET.fromstring(gen_rover.build_sdf(V)).find("model")  # a copy to strip
         plugin = model.find("plugin[@name='rover_sim::RoverDrivetrain']")
@@ -549,5 +591,5 @@ class Ruts(unittest.TestCase):
         misspelt parameter would silently keep the C++ default."""
-        source = (SIM / "plugins" / "rover_tracks_render.hh").read_text()
-        header = (SIM / "plugins" / "rover_tracks.hh").read_text()
-        for name, value in {**dataclasses.asdict(P.drive.tracks), "width": P.wheel_width}.items():
+        source = (SIM_DIR / "plugins" / "rover_tracks_render.hh").read_text()
+        header = (SIM_DIR / "plugins" / "rover_tracks.hh").read_text()
+        for name, value in {**SIM.ruts.tracks, "width": ROVER.wheel.width}.items():
             self.assertIn(f'get("{name}", p.{name})', source, name)
@@ -558,3 +600,3 @@ class Ruts(unittest.TestCase):
     def test_gz_accepts_it(self):
-        with temp_sdf(gen_model.build_sdf(P)) as path:
+        with temp_sdf(gen_rover.build_sdf(V)) as path:
             result = gz_check(path)
@@ -565,4 +607,4 @@ class Ruts(unittest.TestCase):
 class TyreCompliance(unittest.TestCase):
-    """Params.tire_compliance (design spec 6.7, phase 2): each wheel hangs from its
-    rocker through an axial and a radial sprung hub."""
+    """sim.yaml tyre_compliance (design spec 6.7, phase 2): each wheel hangs from its
+    rocker through an axial and a radial sprung hub, with rover.yaml's tyre."""
 
@@ -570,4 +612,3 @@ class TyreCompliance(unittest.TestCase):
     def setUpClass(cls):
-        cls.params = dataclasses.replace(P, tire_compliance=True)
-        cls.sdf = gen_model.build_sdf(cls.params)
+        cls.sdf = gen_rover.build_sdf(V.replace(tyre_compliance={"enabled": True}))
         cls.model = ET.fromstring(cls.sdf).find("model")
@@ -575,2 +616,3 @@ class TyreCompliance(unittest.TestCase):
     def test_hub_chain(self):
+        tyre = ROVER.tyre
         joints = {j.get("name"): j for j in self.model.findall("joint")}
@@ -585,9 +627,8 @@ class TyreCompliance(unittest.TestCase):
                 self.assertEqual(joints[f"{wheel}_joint"].findtext("parent"), f"{wheel}_hub_radial")
-                for joint, axis, (stiffness, damping) in ((axial, [0, 1, 0], P.tire_axial),
-                                                         (radial, [0, 0, 1], P.tire_radial)):
+                for joint, axis, spring in ((axial, [0, 1, 0], tyre.axial), (radial, [0, 0, 1], tyre.radial)):
                     self.assertEqual(joint.get("type"), "prismatic")
                     self.assertEqual(vec(joint.findtext("axis/xyz")), axis)
-                    self.assertEqual(float(joint.findtext("axis/dynamics/spring_stiffness")), stiffness)
-                    self.assertEqual(float(joint.findtext("axis/dynamics/damping")), damping)
-                    self.assertEqual(float(joint.findtext("axis/limit/upper")), P.tire_travel)
+                    self.assertEqual(float(joint.findtext("axis/dynamics/spring_stiffness")), spring.stiffness)
+                    self.assertEqual(float(joint.findtext("axis/dynamics/damping")), spring.damping)
+                    self.assertEqual(float(joint.findtext("axis/limit/upper")), tyre.travel)
 
@@ -602,3 +643,3 @@ class ChaseCameraModel(unittest.TestCase):
     def setUpClass(cls):
-        cls.sdf = gen_model.build_chase_sdf(gen_model.ChaseParams())
+        cls.sdf = viewers.build_chase_sdf(viewers.ChaseParams())
         cls.model = ET.fromstring(cls.sdf).find("model")
@@ -606,3 +647,3 @@ class ChaseCameraModel(unittest.TestCase):
     def test_floats_without_collisions(self):
-        self.assertEqual(self.model.get("name"), gen_model.CHASE_MODEL)
+        self.assertEqual(self.model.get("name"), viewers.CHASE_MODEL)
         self.assertEqual(self.model.findtext("link/gravity"), "false")
@@ -611,5 +652,5 @@ class ChaseCameraModel(unittest.TestCase):
     def test_camera_and_plugin(self):
-        c = gen_model.ChaseParams()
+        c = viewers.ChaseParams()
         sensor = self.model.find("link/sensor[@type='camera']")
-        self.assertEqual(sensor.findtext("topic"), gen_model.CHASE_IMAGE_TOPIC)
+        self.assertEqual(sensor.findtext("topic"), viewers.CHASE_IMAGE_TOPIC)
         self.assertEqual((int(sensor.findtext("camera/image/width")), int(sensor.findtext("camera/image/height"))),
@@ -619,3 +660,3 @@ class ChaseCameraModel(unittest.TestCase):
         self.assertEqual(plugin.findtext("target"), "rover")
-        self.assertEqual(plugin.findtext("cmd_topic"), gen_model.CHASE_CMD_TOPIC)
+        self.assertEqual(plugin.findtext("cmd_topic"), viewers.CHASE_CMD_TOPIC)
         self.assertGreater(float(plugin.findtext("min_pitch")), 0)  # stays above the target
@@ -631,3 +672,3 @@ class EyeCameraModel(unittest.TestCase):
     def setUpClass(cls):
-        cls.sdf = gen_model.build_eye_sdf(gen_model.EyeParams(), P)
+        cls.sdf = viewers.build_eye_sdf(viewers.EyeParams(), ROVER)
         cls.model = ET.fromstring(cls.sdf).find("model")
@@ -636,3 +677,3 @@ class EyeCameraModel(unittest.TestCase):
     def test_floats_without_collisions(self):
-        self.assertEqual(self.model.get("name"), gen_model.EYE_MODEL)
+        self.assertEqual(self.model.get("name"), viewers.EYE_MODEL)
         self.assertEqual(self.model.findtext("link/gravity"), "false")
@@ -642,3 +683,3 @@ class EyeCameraModel(unittest.TestCase):
         # Where the tilt axis runs through the RGB-D sensor and the pan axis meets it.
-        rover = ET.fromstring(gen_model.build_sdf(P)).find("model")
+        rover = ET.fromstring(gen_rover.build_sdf(V)).find("model")
         tilt = vec(rover.find("link[@name='camera_tilt_link']/pose").text)
@@ -652,6 +693,7 @@ class EyeCameraModel(unittest.TestCase):
         value = lambda tag: float(self.plugin.findtext(tag))  # noqa: E731
-        self.assertEqual((value("yaw"), value("pitch")), (0.0, P.camera_pitch))
-        self.assertEqual((value("min_yaw"), value("max_yaw")), (-P.camera_pan_limit, P.camera_pan_limit))
-        self.assertEqual((value("min_pitch"), value("max_pitch")), P.camera_tilt_limits)
-        self.assertEqual(self.plugin.findtext("look_topic"), gen_model.EYE_LOOK_TOPIC)
+        head = ROVER.head
+        self.assertEqual((value("yaw"), value("pitch")), (0.0, head.home_tilt))
+        self.assertEqual((value("min_yaw"), value("max_yaw")), (-head.pan_limit, head.pan_limit))
+        self.assertEqual((value("min_pitch"), value("max_pitch")), head.tilt_limits)
+        self.assertEqual(self.plugin.findtext("look_topic"), viewers.EYE_LOOK_TOPIC)
         self.assertIsNone(self.plugin.find("cmd_topic"))  # an eye only turns, it does not orbit or zoom
@@ -659,5 +701,5 @@ class EyeCameraModel(unittest.TestCase):
     def test_camera(self):
-        e = gen_model.EyeParams()
+        e = viewers.EyeParams()
         sensor = self.model.find("link/sensor[@type='camera']")
-        self.assertEqual(sensor.findtext("topic"), gen_model.EYE_IMAGE_TOPIC)
+        self.assertEqual(sensor.findtext("topic"), viewers.EYE_IMAGE_TOPIC)
         self.assertEqual((int(sensor.findtext("camera/image/width")), int(sensor.findtext("camera/image/height"))),
@@ -666,3 +708,3 @@ class EyeCameraModel(unittest.TestCase):
         self.assertEqual(float(sensor.findtext("camera/horizontal_fov")), e.hfov)
-        self.assertEqual(e.hfov, P.camera_hfov)
+        self.assertEqual(e.hfov, ROVER.camera.hfov)
 
diff --git a/rover_sim/viewers.py b/rover_sim/viewers.py
index 829e752..58dd1e7 100644
--- a/rover_sim/viewers.py
+++ b/rover_sim/viewers.py
@@ -1,3 +1,3 @@
 """The driver station's cameras (sim/station), generated with the rover by
-`pixi run sim-model` (gen_model.main calls write_all): the chase camera that
+`pixi run sim-model` (tools/gen_rover.py calls write_all): the chase camera that
 follows the rover, sim/models/chase_camera; the rover eye at its camera
@@ -12,3 +12,3 @@ it. Every dimension lives in ChaseParams, EyeParams and FlyParams; the eye
 takes its mount, start and limits from the rover's pan-tilt head
-(gen_model.Params).
+(rover_description/config/rover.yaml, rover_config.ROVER).
 """
@@ -17,3 +17,3 @@ from pathlib import Path
 
-from gen_model import Params
+from rover_config import ROVER
 from urc import sdf  # the SDF writer every generated model shares
@@ -82,3 +82,3 @@ class EyeParams:
     """The driver station's first-person camera (plugins/chase_camera.cpp in
-    eye mode): it rides at the rover's camera pivot, Params.camera_xyz, where
+    eye mode): it rides at the rover's camera pivot, rover.yaml's camera xyz, where
     the pan and tilt axes meet, and turns there like the pan-tilt head and
@@ -88,3 +88,3 @@ class EyeParams:
     size: tuple[int, int] = (960, 540)
-    hfov: float = Params.camera_hfov  # [rad] as wide as the sensor
+    hfov: float = ROVER.camera.hfov  # [rad] as wide as the sensor
     # The near clip also hides the camera's own housing and yoke, all within
@@ -92,3 +92,3 @@ class EyeParams:
     # sensor's depth stops at 40 m).
-    clip: tuple[float, float] = (Params.camera_clip[0], 80_000.0)  # [m]
+    clip: tuple[float, float] = (ROVER.camera.clip[0], 80_000.0)  # [m]
 
@@ -155,10 +155,12 @@ def build_chase_sdf(c: ChaseParams) -> str:
 
-def build_eye_sdf(e: EyeParams, p: Params) -> str:
+def build_eye_sdf(e: EyeParams, rover=ROVER) -> str:
     """The rover eye as an SDF 1.11 document: the plugin's eye mode at the
-    camera pivot, starting and limited like the pan-tilt head."""
+    camera pivot, starting and limited like the pan-tilt head (rover: the
+    numbers of rover.yaml, rover_config.ROVER)."""
+    head = rover.head
     return _viewer_sdf(EYE_MODEL, EYE_IMAGE_TOPIC, e.rate, e.size, e.hfov, e.clip, (
-        ("target", e.target), ("mount", p.camera_xyz),
-        ("yaw", 0.0), ("pitch", p.camera_pitch),
-        ("min_yaw", -p.camera_pan_limit), ("max_yaw", p.camera_pan_limit),
-        ("min_pitch", p.camera_tilt_limits[0]), ("max_pitch", p.camera_tilt_limits[1]),
+        ("target", e.target), ("mount", rover.camera.xyz),
+        ("yaw", 0.0), ("pitch", head.home_tilt),
+        ("min_yaw", -head.pan_limit), ("max_yaw", head.pan_limit),
+        ("min_pitch", head.tilt_limits[0]), ("max_pitch", head.tilt_limits[1]),
         ("look_topic", EYE_LOOK_TOPIC), ("state_topic", EYE_STATE_TOPIC)))
@@ -185,3 +187,3 @@ def write_all(models_dir):
                 "(edit ChaseParams)."),
-               (EYE_MODEL, build_eye_sdf(EyeParams(), Params()),
+               (EYE_MODEL, build_eye_sdf(EyeParams(), ROVER),
                 "First-person camera for the driver station (sim/station): the ChaseCamera plugin keeps it at "
@@ -195,3 +197,3 @@ def write_all(models_dir):
         (directory / "model.sdf").write_text(text)
-        (directory / "model.config").write_text(sdf.model_config(name, description, "sim/gen_model.py"))
+        (directory / "model.config").write_text(sdf.model_config(name, description, "rover_sim/tools/gen_rover.py"))
         print(f"wrote {directory / 'model.sdf'}")
```

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src && git apply "$T/t11.patch" && git add -A && git status --short
```

Expected:
```
M  pixi.toml
M  rover_sim/models/rover/model.config
M  rover_sim/tests/test_foundations.py
R  rover_sim/tests/test_gen_model.py -> rover_sim/tests/test_rover_model.py
M  rover_sim/viewers.py
```

- [ ] **Step 2: Run the tests to see the tracked model fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_rover_model test_foundations.Viewers 2>&1 | grep -E "^(FAIL|ERROR):|^Ran|^FAILED|^OK"
```

Expected (the tracked files are still gen_model.py's):
```
FAIL: test_tread_mesh (test_rover_model.DigCues.test_tread_mesh)
FAIL: test_the_default_rover_is_physical_and_tracked (test_rover_model.TrackedModel.test_the_default_rover_is_physical_and_tracked)
FAIL: test_the_tread_mesh_is_the_descriptions (test_rover_model.TrackedModel.test_the_tread_mesh_is_the_descriptions)
FAIL: test_write_all_writes_the_checked_in_models (test_foundations.Viewers.test_write_all_writes_the_checked_in_models)
Ran 49 tests in ...s
FAILED (failures=4)
```

- [ ] **Step 3: Generate the rover model from the description**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run sim-model && git status --short rover_sim/models rover_sim/config
```

Expected:
```
wrote config/surfaces.yaml, models/rover/model.sdf, materials/textures/dust_puff.png and meshes/wheel.glb
wrote .../rover_sim/models/chase_camera/model.sdf
wrote .../rover_sim/models/eye_camera/model.sdf
wrote .../rover_sim/models/fly_camera/model.sdf
 M rover_sim/models/chase_camera/model.config
 M rover_sim/models/eye_camera/model.config
 M rover_sim/models/fly_camera/model.config
 M rover_sim/models/rover/meshes/wheel.glb
 M rover_sim/models/rover/model.sdf
```

(`surfaces.yaml` keeps its bytes, so `write_if_changed` leaves it alone; the dust sprite and the three cameras' `model.sdf` are written again with the same bytes; `model.config` names the new generator.)

- [ ] **Step 4: Run the tests to see them pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && GZ_IP=127.0.0.1 /Users/alarion239/.pixi/bin/pixi run python -m unittest test_rover_model test_equivalence test_rover_config test_foundations.Viewers 2>&1 | grep -E "^(FAIL|ERROR):|^Ran|^FAILED|^OK"
```

Expected: `Ran 68 tests`, `OK` (46 + 11 + 8 + 3). `test_equivalence.Equivalence.test_the_rover_is_the_tracked_model` now compares the generated `model.sdf` with the frozen one.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add -A && git commit -q -F - <<'EOF'
PR 2: rover_sim: sim-model builds the rover from rover_description; the model tests follow

pixi run sim-model runs tools/gen_rover.py: models/rover/model.sdf is now rover_sim.urdf.xacro through xacro and
gz sdf, the same rover as gen_model.py's (test_equivalence) in the converter's layout; the tread mesh is
rover_description's (its generator's name changed, nothing else); the station cameras' model.config name the new
generator. viewers.py takes the eye's mount, start and limits from rover.yaml. test_gen_model.py becomes
test_rover_model.py with every test and assertion kept, two added (surfaces.yaml and the mesh copy are fresh).
sim-worlds depends on rover_config.py and rover.yaml instead of gen_model.py.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 12: The tests' rover variants on rover_config

**Files:**
- Modify: `rover_sim/tests/simulate.py`, `rover_sim/tests/test_drivetrain.py`, `rover_sim/tests/test_fly_camera.py`, `rover_sim/tests/test_foundations.py`, `rover_sim/tests/test_render.py`, `rover_sim/tests/test_rover_sim.py`, `rover_sim/tests/test_station.py`, `rover_sim/tests/test_urc_missions.py`, `rover_sim/tests/test_urc_sim.py`, `rover_sim/tests/test_urc_terrain.py`, `rover_sim/tools/realism_report.py`

`simulate.physical(**sections)` and `simulate.diffdrive(**sections)` return a `rover_config.Variant` (sections of `rover.yaml` or `sim.yaml`: `physical(contact={"dig": False}, motor={"current_limit": 10.0})`), `rover_model` and `world_file` build it with `gen_rover.build_sdf`, and every test names its numbers through `rover_config` (`P.wheel_dx` -> `ROVER.wheel.dx`, `P.drive.accel` -> `SIM.drivetrain.accel`, the viewers' names through `viewers`). No assertion changes. `test_render`'s dust emitter, which `gen_model._add_dust_emitter` built on its own, is now the left emitter of the generated rover with dust on, renamed `e` with its topic: the same elements in the same order with the same values (its `particle_scatter_ratio` written as the float32 SDF reads, 9.99999997e-07).

- [ ] **Step 1: Apply the change**

Save this patch as `$T/t12.patch`:

```diff
diff --git a/rover_sim/tests/simulate.py b/rover_sim/tests/simulate.py
index db0ac7a..03e2728 100644
--- a/rover_sim/tests/simulate.py
+++ b/rover_sim/tests/simulate.py
@@ -6,3 +6,3 @@ the state is read from the entity-component manager after the last step.
 The rover is model://rover (the physical drivetrain, its command timeout on
-the wall clock) or one built from gen_model Params (physical(): the same
+the wall clock) or one built from a rover_config.Variant (physical(): the same
 with the timeout on sim time, so runs repeat exactly; diffdrive(): Gazebo's
@@ -38,4 +38,6 @@ from worldfiles import SIM_DIR, temp_sdf
 
-import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
+import gen_rover  # noqa: E402  (worldfiles puts sim/ and sim/tools on the path)
 import gzenv  # noqa: E402
+import rover_config  # noqa: E402
+from rover_config import ROVER, Variant  # noqa: E402
 from urc import sdf, terrain, terrains  # noqa: E402
@@ -134,3 +136,3 @@ def variant_sdf(text, rover_uri=None, solver=None):
 def rover_model(params):
-    """URI of a temporary rover model built from gen_model Params `params`."""
+    """URI of a temporary rover model built from the rover_config.Variant `params` (gen_rover.build_sdf)."""
     with tempfile.TemporaryDirectory(prefix="rover_model_") as directory:
@@ -138,3 +140,3 @@ def rover_model(params):
         model.mkdir()
-        (model / "model.sdf").write_text(gen_model.build_sdf(params))
+        (model / "model.sdf").write_text(gen_rover.build_sdf(params))
         (model / "model.config").write_text(sdf.model_config("rover", "A test variant of the rover.",
@@ -144,13 +146,17 @@ def rover_model(params):
 
-def physical(**drive):
-    """Params of the rover with the physical drivetrain (the default), its
-    DriveParams changed by `drive`; the command timeout runs on sim time, so
-    runs repeat exactly."""
-    return gen_model.Params(drive=gen_model.DriveParams(mode="physical", cmd_timeout_clock="sim", **drive))
+def physical(**sections):
+    """The rover with the physical drivetrain (the default) as a
+    rover_config.Variant, sections of rover.yaml and sim.yaml changed by
+    `sections` (Variant.replace: physical(contact={"dig": False})); the
+    command timeout runs on sim time, so runs repeat exactly."""
+    drivetrain = {"mode": "physical", "cmd_timeout_clock": "sim", **sections.pop("drivetrain", {})}
+    return Variant().replace(drivetrain=drivetrain, **sections)
 
 
-def diffdrive(**params):
-    """Params of the rover with Gazebo's DiffDrive (design spec D22: the A/B
-    and cost reference), other Params changed by `params`."""
-    return gen_model.Params(drive=gen_model.DriveParams(mode="diffdrive"), **params)
+def diffdrive(**sections):
+    """The rover with Gazebo's DiffDrive (design spec D22: the A/B and cost
+    reference) as a rover_config.Variant, other sections changed by
+    `sections` (diffdrive(diffdrive={"mu_lateral": 1.0}))."""
+    drivetrain = {"mode": "diffdrive", **sections.pop("drivetrain", {})}
+    return Variant().replace(drivetrain=drivetrain, **sections)
 
@@ -161,5 +167,5 @@ def world_file(world=None, extra="", spawn_z=0.02, rover_uri=None, params=None,
     None, else the SDF file `world` (a copy when its rover or solver change).
-    The rover: rover_uri, or one built from Params `params`, or model://rover.
-    A physical rover built from params takes default_surface as its
-    drivetrain's default surface, which is what it feels on a plane."""
+    The rover: rover_uri, or one built from the rover_config.Variant `params`,
+    or model://rover. A physical rover built from params takes default_surface
+    as its drivetrain's default surface, which is what it feels on a plane."""
     with contextlib.ExitStack() as stack:
@@ -167,5 +173,4 @@ def world_file(world=None, extra="", spawn_z=0.02, rover_uri=None, params=None,
             assert rover_uri is None, "rover_uri or params, not both"
-            if default_surface is not None and params.drive.mode == "physical":
-                drive = dataclasses.replace(params.drive, default_surface=default_surface)
-                params, default_surface = dataclasses.replace(params, drive=drive), None
+            if default_surface is not None and params.sim.drivetrain.mode == "physical":
+                params, default_surface = params.replace(contact={"default_surface": default_surface}), None
             rover_uri = stack.enter_context(rover_model(params))
@@ -213,3 +218,3 @@ def _run(seconds, cmd, subscribe, world_path, trace_every=0, publish_until=None)
         node.subscribe(msg_type, topic, messages[topic].append)
-    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
+    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
     scripted = bool(cmd) and isinstance(cmd[0], (tuple, list))
@@ -300,3 +305,3 @@ def follow(world, path, seconds, speed=0.8, tolerance=1.0, **driver):
     node = Node()
-    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
+    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
     goal = tuple(path[-1])
@@ -334,3 +339,3 @@ def follow(world, path, seconds, speed=0.8, tolerance=1.0, **driver):
 
-def spin_ratio(traction, dig=1.0, params=gen_model.Params()):
+def spin_ratio(traction, dig=1.0, rover=ROVER):
     """The yaw rate of the rover turning in place on ground `traction`
@@ -339,4 +344,4 @@ def spin_ratio(traction, dig=1.0, params=gen_model.Params()):
     = crr D c + bulldoze D^2 a, wheels at (+-a, +-c), sx = c (1 - r), sy = a r;
-    0 if it cannot turn."""
-    a, c = params.wheel_dx, params.pivot_y
+    0 if it cannot turn. rover: rover.yaml's numbers (rover_config.ROVER)."""
+    a, c = rover.wheel.dx, rover.rocker.pivot_y
     resist = traction.crr * dig * c + traction.bulldoze * dig * dig * a
@@ -401,3 +406,3 @@ def ground_world(hf, ground, rows, rover=(0.0, 0.0, 0.0), *, lift=0.02, ground_o
     rows and ground_options (ground_json's keywords): ground.json. The rover:
-    model rover_uri, or one built from Params `params`, at rover = (x, y,
+    model rover_uri, or one built from the rover_config.Variant `params`, at rover = (x, y,
     yaw), `lift` above the terrain and tilted to its slope. terrain_extra:
@@ -471,3 +476,3 @@ def twist_publisher(worlds, schedule=DRIVE_SCHEDULE, rate=20.0):
         node.subscribe(WorldStatistics, f"/world/{name}/stats", on_stats)
-    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
+    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
     stop = threading.Event()
diff --git a/rover_sim/tests/test_drivetrain.py b/rover_sim/tests/test_drivetrain.py
index c919c33..5988194 100644
--- a/rover_sim/tests/test_drivetrain.py
+++ b/rover_sim/tests/test_drivetrain.py
@@ -5,3 +5,3 @@ calibration targets of the realism design
 
-The rover is built with DriveParams(mode="physical") into temporary
+The rover is built with the physical drivetrain (sim.yaml) into temporary
 directories (simulate.physical) and mostly stands on synthetic ground maps
@@ -30,4 +30,4 @@ from pathlib import Path
 import numpy as np
-from simulate import (cpu_time_per_step, diffdrive, gen_model, ground_row, ground_world, physical, simulate,
-                      spin_ratio, world_file)
+from simulate import (cpu_time_per_step, diffdrive, ground_row, ground_world, physical, simulate, spin_ratio,
+                      world_file)
 from worldfiles import WORLDS, temp_sdf, world_copy
@@ -46,4 +46,6 @@ from urc import meshes, terrain, terrains  # noqa: E402
 
-P = gen_model.Params()
-A, C = P.wheel_dx, P.pivot_y  # half wheelbase, half track [m]
+import rover_config  # noqa: E402
+from rover_config import ROVER, SIM  # noqa: E402
+
+A, C = ROVER.wheel.dx, ROVER.rocker.pivot_y  # half wheelbase, half track [m]
 T = terrains.Traction
@@ -67,3 +69,4 @@ OPTIONS = dict(terrain_default="rock", object_default="manmade")
 FLAT = terrain.Heightfield(32.0, 65)  # 0.5 m samples
-WEIGHT = 9.81 * (P.chassis_mass + 2 * P.rocker_mass + 4 * P.wheel_mass + P.camera_pan_mass + P.camera_tilt_mass)
+WEIGHT = 9.81 * (ROVER.chassis.mass + 2 * ROVER.rocker.mass + 4 * ROVER.wheel.mass + ROVER.head.pan_mass
+                 + ROVER.head.tilt_mass)
 LOAD = WEIGHT / 4  # [N] a wheel's share on flat ground, 113 N
@@ -110,6 +113,7 @@ def drive(seconds, cmd, ground="regolith", params=None, solver=None, hf=FLAT, ra
     with ground_world(hf, raster, rows, rover, lift=lift, ground_options=OPTIONS, terrain_extra=terrain_extra,
-                      extra=extra, params=params or physical(dig=False), solver=solver, gravity=gravity) as world:
+                      extra=extra, params=params or physical(contact={"dig": False}), solver=solver,
+                      gravity=gravity) as world:
         s = simulate(seconds, world=world, cmd=cmd, trace_every=trace_every, publish_until=publish_until,
-                     subscribe=[(gen_model.DRIVETRAIN_TOPIC, StringMsg), *subscribe])
-    return Run(s, [json.loads(m.data) for m in s.messages[gen_model.DRIVETRAIN_TOPIC]])
+                     subscribe=[(rover_config.DRIVETRAIN_TOPIC, StringMsg), *subscribe])
+    return Run(s, [json.loads(m.data) for m in s.messages[rover_config.DRIVETRAIN_TOPIC]])
 
@@ -140,3 +144,3 @@ class SpinInPlace(unittest.TestCase):
                 torque = np.abs(drive(6.0, SPIN, key).wheel("tau", 3.0, 6.0)).mean()
-                self.assertAlmostEqual(torque, mu * LOAD * P.wheel_radius * 0.748, delta=0.25 * 12.7 * mu)
+                self.assertAlmostEqual(torque, mu * LOAD * ROVER.wheel.radius * 0.748, delta=0.25 * 12.7 * mu)
 
@@ -175,3 +179,3 @@ class SpinOnObjects(unittest.TestCase):
                 with ground_world(FLAT, everywhere("regolith"), ROWS, lift=0.12, ground_options=options,
-                                  terrain_extra=shape, params=physical(dig=False)) as world:
+                                  terrain_extra=shape, params=physical(contact={"dig": False})) as world:
                     s = simulate(6.0, world=world, cmd=SPIN, trace_every=10)
@@ -191,3 +195,3 @@ class DiagonalLoads(unittest.TestCase):
     def setUpClass(cls):
-        cls.runs = {key: drive(6.0, SPIN, key, solver="pgs", params=physical(dig=False, mu_noise=0.0))
+        cls.runs = {key: drive(6.0, SPIN, key, solver="pgs", params=physical(contact={"dig": False, "mu_noise": 0.0}))
                     for key in ("test_mu100", "test_mu080")}
@@ -226,3 +230,3 @@ class Stall(unittest.TestCase):
             run = drive(6.0, [(0.0, 0.0, 0.0), (1.0, 0.0, 1.0)], "test_mu080", solver="pgs",
-                        params=physical(dig=False, current_limit=limit))
+                        params=physical(contact={"dig": False}, motor={"current_limit": limit}))
             return math.degrees(run.turned(1.0, 6.0))
@@ -261,5 +265,5 @@ class Response(unittest.TestCase):
         is the controller's, an assumption, design spec 6.3)."""
-        rise, _ = self.times(physical(dig=False))
-        setpoint = 1.0 * C / P.wheel_radius  # [rad/s] each wheel
-        self.assertAlmostEqual(rise, setpoint / P.drive.accel, delta=0.2 * setpoint / P.drive.accel)
+        rise, _ = self.times(physical(contact={"dig": False}))
+        setpoint = 1.0 * C / ROVER.wheel.radius  # [rad/s] each wheel
+        self.assertAlmostEqual(rise, setpoint / SIM.drivetrain.accel, delta=0.2 * setpoint / SIM.drivetrain.accel)
 
@@ -267,3 +271,3 @@ class Response(unittest.TestCase):
         """The motor, contact and tyre model alone stop the turn in < 40 ms (physics)."""
-        self.assertLess(self.times(physical(dig=False, accel=0.0))[1], 0.040)
+        self.assertLess(self.times(physical(contact={"dig": False}, drivetrain={"accel": 0.0}))[1], 0.040)
 
@@ -274,3 +278,3 @@ class Response(unittest.TestCase):
         118 ms, with ki 160 (25 ms) 53 ms (measured 2026-10-07)."""
-        self.assertLess(self.times(physical(dig=False, accel=0.0))[0], 0.060)
+        self.assertLess(self.times(physical(contact={"dig": False}, drivetrain={"accel": 0.0}))[0], 0.060)
 
@@ -283,3 +287,4 @@ class SlowTurn(unittest.TestCase):
     def judder(ground, tire_compliance=False):
-        params = dataclasses.replace(physical(dig=False, accel=4.0), tire_compliance=tire_compliance)
+        params = physical(contact={"dig": False}, drivetrain={"accel": 4.0},
+                          tyre_compliance={"enabled": tire_compliance})
         run = drive(9.0, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.15)], ground, trace_every=1, params=params)
@@ -300,3 +305,3 @@ class SlowTurn(unittest.TestCase):
         smooth. Measured 2026-10-07: 0.011 (Dantzig); 0.60 on PGS, all solver
-        noise at 333 Hz. With Params.tire_compliance: 0.41 (Dantzig) and 0.78
+        noise at 333 Hz. With sim.yaml tyre_compliance: 0.41 (Dantzig) and 0.78
         (PGS), at the 22-24 Hz wheel hop, not 0.5-5 Hz. (Before the contact
@@ -335,3 +340,3 @@ class LooseSand(unittest.TestCase):
                     speed = (tr[-1, 1] - tr[0, 1]) / (tr[-1, 0] - tr[0, 0])
-                    surface = np.abs(run.wheel("w", 3.0, 8.0)).mean() * P.wheel_radius
+                    surface = np.abs(run.wheel("w", 3.0, 8.0)).mean() * ROVER.wheel.radius
                     ground = kind(key)
@@ -340,3 +345,3 @@ class LooseSand(unittest.TestCase):
                         torque = np.abs(run.wheel("tau", 3.0, 8.0)).mean()
-                        expected = ground.crr * LOAD * P.wheel_radius
+                        expected = ground.crr * LOAD * ROVER.wheel.radius
                         self.assertAlmostEqual(torque, expected, delta=0.15 * expected)
@@ -377,3 +382,3 @@ class LooseSand(unittest.TestCase):
         x, y = tr[:, 1], tr[:, 2]
-        out = np.flatnonzero(np.hypot(x, y) > 3.0 + math.hypot(A, C) + P.wheel_radius)  # every wheel on the rock
+        out = np.flatnonzero(np.hypot(x, y) > 3.0 + math.hypot(A, C) + ROVER.wheel.radius)  # every wheel on the rock
         self.assertTrue(len(out), "the rover did not drive out of the sand")
@@ -411,4 +416,4 @@ class LooseSand(unittest.TestCase):
             s = simulate(30.0, world=world, cmd=[(0.0, 0.0, 0.0), (1.0, 0.5, 0.0)], params=physical(),
-                         trace_every=100, subscribe=[(gen_model.DRIVETRAIN_TOPIC, StringMsg)])
-        run = Run(s, [json.loads(m.data) for m in s.messages[gen_model.DRIVETRAIN_TOPIC]])
+                         trace_every=100, subscribe=[(rover_config.DRIVETRAIN_TOPIC, StringMsg)])
+        run = Run(s, [json.loads(m.data) for m in s.messages[rover_config.DRIVETRAIN_TOPIC]])
         tr = run.window(0.0, 30.0)
@@ -456,3 +461,3 @@ def visual_poses(messages, name="tire_visual"):
 class DigSink(unittest.TestCase):
-    """The dig-in made visible (DriveParams.dig_sink, on by default; the
+    """The dig-in made visible (sim.yaml dig_sink, on by default; the
     user's decision of 2026-10-07): each tyre's visual, never its collision,
@@ -472,3 +477,3 @@ class DigSink(unittest.TestCase):
         raster = np.where(np.hypot(X, Y) < 1.2, GROUND["sand"][0], GROUND["rock"][0]).astype(np.uint8)
-        before = dataclasses.replace(physical(dig_sink=False, ruts=False), tread_tyre=False)
+        before = physical(dig_sink={"enabled": False}, ruts={"enabled": False}, tread_tyre=False)
         cls.runs = {}
@@ -480,4 +485,5 @@ class DigSink(unittest.TestCase):
                     s = simulate(11.0, world=world, cmd=cls.CMD, trace_every=1,
-                                 subscribe=[(gen_model.DRIVETRAIN_TOPIC, StringMsg), (STATE_TOPIC, SerializedStepMap)])
-                cls.runs[solver, cues] = Run(s, [json.loads(m.data) for m in s.messages[gen_model.DRIVETRAIN_TOPIC]])
+                                 subscribe=[(rover_config.DRIVETRAIN_TOPIC, StringMsg),
+                                            (STATE_TOPIC, SerializedStepMap)])
+                cls.runs[solver, cues] = Run(s, [json.loads(m.data) for m in s.messages[rover_config.DRIVETRAIN_TOPIC]])
 
@@ -539,6 +545,6 @@ class Washboard(unittest.TestCase):
                               ground_options=dict(OPTIONS, collisions={"washboard": "regolith"}),
-                              params=physical(dig=False, state_rate=1000.0)) as world:
+                              params=physical(contact={"dig": False}, drivetrain={"state_rate": 1000.0})) as world:
                 s = simulate(20.0, world=world, cmd=[(0.0, 0.0, 0.0), (0.5, 0.5, 0.0)], trace_every=10,
-                             subscribe=[(gen_model.DRIVETRAIN_TOPIC, StringMsg)])
-        cls.trip = Run(s, [json.loads(m.data) for m in s.messages[gen_model.DRIVETRAIN_TOPIC]])
+                             subscribe=[(rover_config.DRIVETRAIN_TOPIC, StringMsg)])
+        cls.trip = Run(s, [json.loads(m.data) for m in s.messages[rover_config.DRIVETRAIN_TOPIC]])
         tr = cls.trip.window(0.0, 20.0)
@@ -572,3 +578,3 @@ class Washboard(unittest.TestCase):
         steepest = 0.04 * 2 * math.pi / 0.8  # d/dx of 0.04 (1 - cos(2 pi x / 0.8))
-        expected = LOAD * P.wheel_radius * steepest / math.sqrt(2)
+        expected = LOAD * ROVER.wheel.radius * steepest / math.sqrt(2)
         for std in self.torque.std(axis=0):
@@ -647,3 +653,4 @@ class Slopes(unittest.TestCase):
                     run = drive(4.0, (0.0, 0.0), "regolith", rover=(0.0, 0.0, math.radians(heading)), lift=0.0,
-                                gravity=gravity, params=physical(dig=False, mu_noise=0.0, backlash=0.0))
+                                gravity=gravity, params=physical(contact={"dig": False, "mu_noise": 0.0},
+                                                                 driveline={"backlash": 0.0}))
                     moved = math.dist(run.window(1.0, 1.1)[0][1:4], run.window(3.9, 4.0)[-1][1:4])
@@ -674,5 +681,5 @@ class Interfaces(unittest.TestCase):
         angles, odom -> base_link (design spec 6.8)."""
-        topics = [(gen_model.ODOM_TOPIC, Odometry), (gen_model.TF_TOPIC, Pose_V)]
+        topics = [(rover_config.ODOM_TOPIC, Odometry), (rover_config.TF_TOPIC, Pose_V)]
         run = drive(4.0, [(0.0, 0.0, 0.0), (0.5, 0.5, 0.0)], "rock", subscribe=topics, rover=(-3.0, 0.0, 0.0))
-        odom, tf = run.state.messages[gen_model.ODOM_TOPIC][-1], run.state.messages[gen_model.TF_TOPIC][-1]
+        odom, tf = run.state.messages[rover_config.ODOM_TOPIC][-1], run.state.messages[rover_config.TF_TOPIC][-1]
         self.assertEqual(frame_ids(odom.header), {"frame_id": "odom", "child_frame_id": "base_link"})
@@ -683,3 +690,3 @@ class Interfaces(unittest.TestCase):
         self.assertAlmostEqual(odom.twist.linear.x, 0.5, delta=0.02)
-        self.assertGreater(len(run.state.messages[gen_model.ODOM_TOPIC]), 0.8 * 50 * 4.0)  # 50 Hz
+        self.assertGreater(len(run.state.messages[rover_config.ODOM_TOPIC]), 0.8 * 50 * 4.0)  # 50 Hz
 
@@ -688,4 +695,4 @@ class Interfaces(unittest.TestCase):
         (plumbing; with track_multiplier 1, as a real rover's odometry, design spec 6.8)."""
-        run = drive(6.0, SPIN, "regolith", subscribe=[(gen_model.ODOM_TOPIC, Odometry)])
-        odom = [m.twist.angular.z for m in run.state.messages[gen_model.ODOM_TOPIC]
+        run = drive(6.0, SPIN, "regolith", subscribe=[(rover_config.ODOM_TOPIC, Odometry)])
+        odom = [m.twist.angular.z for m in run.state.messages[rover_config.ODOM_TOPIC]
                 if 3.0 <= m.header.stamp.sec + m.header.stamp.nsec * 1e-9 < 6.0]
@@ -706,3 +713,3 @@ class Interfaces(unittest.TestCase):
             self.assertIsInstance(wheel["sat"], bool)
-        self.assertAlmostEqual(last["wheels"]["fl"]["sp"], (0.3 - 0.2 * C) / P.wheel_radius, delta=1e-4)
+        self.assertAlmostEqual(last["wheels"]["fl"]["sp"], (0.3 - 0.2 * C) / ROVER.wheel.radius, delta=1e-4)
         self.assertTrue(90 <= len(run.states) <= 101, len(run.states))
@@ -715,3 +722,4 @@ class Interfaces(unittest.TestCase):
         stops = drive(4.0, cmd, "rock", publish_until=1.0)
-        holds = drive(4.0, cmd, "rock", publish_until=1.0, params=physical(dig=False, cmd_timeout=0.0))
+        holds = drive(4.0, cmd, "rock", publish_until=1.0,
+                      params=physical(contact={"dig": False}, drivetrain={"cmd_timeout": 0.0}))
         self.assertEqual(stops.states[-1]["cmd"], [0.0, 0.0])
@@ -750,3 +758,4 @@ class Interfaces(unittest.TestCase):
             with self.subTest(surface=key):
-                s = simulate(6.0, cmd=SPIN, params=physical(dig=False), default_surface=key, trace_every=10)
+                s = simulate(6.0, cmd=SPIN, params=physical(contact={"dig": False}), default_surface=key,
+                             trace_every=10)
                 self.assertAlmostEqual(Run(s, []).yaw_rate(3.0, 6.0), spin_ratio(terrains.TYPES[key].traction),
@@ -755,3 +764,3 @@ class Interfaces(unittest.TestCase):
     def test_dust(self):
-        """With DriveParams.dust on (off by default, the user's decision of
+        """With sim.yaml dust on (off by default, the user's decision of
         2026-10-07) the rear emitters get a rate from speed, slip and the
@@ -761,3 +770,3 @@ class Interfaces(unittest.TestCase):
         either emitter's topic."""
-        topics = [gen_model.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{side[0]}")
+        topics = [rover_config.DUST_TOPIC.format(link=f"rocker_{side}", emitter=f"dust_r{side[0]}")
                   for side in ("left", "right")]
@@ -766,4 +775,4 @@ class Interfaces(unittest.TestCase):
         for key in ("sand", "rock"):
-            run = drive(6.0, schedule, key, params=physical(dig=False, dust=True), rover=(-4.0, 0.0, 0.0),
-                        subscribe=[(topic, ParticleEmitter) for topic in topics])
+            run = drive(6.0, schedule, key, params=physical(contact={"dig": False}, dust={"enabled": True}),
+                        rover=(-4.0, 0.0, 0.0), subscribe=[(topic, ParticleEmitter) for topic in topics])
             self.assertTrue(run.state.messages[topics[1]])  # the right emitter too
@@ -774,8 +783,8 @@ class Interfaces(unittest.TestCase):
             rates[key] = max(m.rate.data for m in messages)
-            self.assertLessEqual(rates[key], P.drive.dust_max)
-        expected = GROUND["sand"][2] * (P.drive.dust_speed_gain * 0.4 + P.drive.dust_slip_gain * 0.1)
+            self.assertLessEqual(rates[key], SIM.dust.rule.max_rate)
+        expected = GROUND["sand"][2] * (SIM.dust.rule.speed_gain * 0.4 + SIM.dust.rule.slip_gain * 0.1)
         self.assertAlmostEqual(rates["sand"], expected, delta=0.25 * expected)  # 0.5 m/s at 20 % slip
         self.assertGreater(rates["sand"], 3 * rates["rock"])
-        self.assertFalse(physical().drive.dust)
-        off = drive(6.0, schedule, "sand", params=physical(dig=False), rover=(-4.0, 0.0, 0.0),
+        self.assertFalse(physical().sim.dust.enabled)
+        off = drive(6.0, schedule, "sand", params=physical(contact={"dig": False}), rover=(-4.0, 0.0, 0.0),
                     subscribe=[(topic, ParticleEmitter) for topic in topics])
@@ -789,6 +798,6 @@ class Interfaces(unittest.TestCase):
         with ground_world(FLAT, everywhere("rock"), ROWS, ground_options=OPTIONS,
-                          params=physical(dig=False, cmd_timeout=0.0)) as world:
+                          params=physical(contact={"dig": False}, drivetrain={"cmd_timeout": 0.0})) as world:
             fixture = TestFixture(world)
             node = Node()
-            publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
+            publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
             twist = Twist()
@@ -800,6 +809,6 @@ class Interfaces(unittest.TestCase):
             before, after = [], []
-            node.subscribe(Odometry, gen_model.ODOM_TOPIC, before.append)
+            node.subscribe(Odometry, rover_config.ODOM_TOPIC, before.append)
             fixture.server().run(True, 3000, False)
             commanding[0] = False
-            node.unsubscribe(gen_model.ODOM_TOPIC)  # a request with a subscription active can lose its reply
+            node.unsubscribe(rover_config.ODOM_TOPIC)  # a request with a subscription active can lose its reply
             request = WorldControl()
@@ -808,3 +817,3 @@ class Interfaces(unittest.TestCase):
             self.assertTrue(ok)
-            node.subscribe(Odometry, gen_model.ODOM_TOPIC, after.append)
+            node.subscribe(Odometry, rover_config.ODOM_TOPIC, after.append)
             fixture.server().run(True, 1000, False)
@@ -838,3 +847,3 @@ def captured_stderr(lines):
 class Ruts(unittest.TestCase):
-    """The ruts and pits behind the wheels (DriveParams.ruts, on by default
+    """The ruts and pits behind the wheels (sim.yaml ruts, on by default
     since the user's decisions of 2026-10-07; plugins/rover_tracks.hh) are
@@ -847,3 +856,3 @@ class Ruts(unittest.TestCase):
         """LooseSand's sand disc at the strong preset: a spin that digs every wheel in to D = 2, then out onto the
-        rock; base_link every step. With ruts, their cost lines on stderr (TrackParams.report)."""
+        rock; base_link every step. With ruts, their cost lines on stderr (sim.yaml ruts tracks report)."""
         X, Y = FLAT.grid()
@@ -852,6 +861,6 @@ class Ruts(unittest.TestCase):
         sand = ground_row(index, "sand", terrains.traction(terrains.SAND, "strong"), dust)
-        tracks = dataclasses.replace(gen_model.TrackParams(), report=1.0)
         lines = []
         with captured_stderr(lines):
-            run = drive(16.0, self.SCHEDULE, raster=raster, params=physical(ruts=ruts, tracks=tracks), solver=solver,
+            run = drive(16.0, self.SCHEDULE, raster=raster,
+                        params=physical(ruts={"enabled": ruts, "tracks": {"report": 1.0}}), solver=solver,
                         rows=[sand if r["key"] == "sand" else r for r in ROWS], trace_every=1)
@@ -894,3 +903,3 @@ class Cost(unittest.TestCase):
                 with temp_sdf(text, WORLDS) as plain, world_file(plain, params=diffdrive()) as servo, \
-                        world_file(plain, params=physical(cmd_timeout=0.0)) as torque:
+                        world_file(plain, params=physical(drivetrain={"cmd_timeout": 0.0})) as torque:
                     costs = cpu_time_per_step({"diffdrive": servo, "physical": torque}, iterations=20_000, runs=5)
@@ -901,3 +910,3 @@ class Cost(unittest.TestCase):
     def test_ruts_cost_almost_nothing(self):
-        """CPU time per step with the ruts on against off (DriveParams.ruts),
+        """CPU time per step with the ruts on against off (sim.yaml ruts),
         the same world and drive, Sensors stripped (the drawing runs on the
@@ -912,4 +921,6 @@ class Cost(unittest.TestCase):
                               Path(copy).read_text())
-                with temp_sdf(text, WORLDS) as plain, world_file(plain, params=physical(cmd_timeout=0.0)) as on, \
-                        world_file(plain, params=physical(cmd_timeout=0.0, ruts=False)) as off:
+                with temp_sdf(text, WORLDS) as plain, \
+                        world_file(plain, params=physical(drivetrain={"cmd_timeout": 0.0})) as on, \
+                        world_file(plain, params=physical(drivetrain={"cmd_timeout": 0.0},
+                                                          ruts={"enabled": False})) as off:
                     costs = cpu_time_per_step({"on": on, "off": off}, iterations=20_000, runs=5)
diff --git a/rover_sim/tests/test_fly_camera.py b/rover_sim/tests/test_fly_camera.py
index d3aa8a7..a3a5ff2 100644
--- a/rover_sim/tests/test_fly_camera.py
+++ b/rover_sim/tests/test_fly_camera.py
@@ -28,6 +28,7 @@ from PIL import Image as PILImage
 import gpu
-from simulate import gen_model, world_sdf
+from simulate import world_sdf
 from worldfiles import SIM_DIR, gz_check, temp_sdf
 
-import viewers  # noqa: E402  (worldfiles puts sim/ on the path)
+import rover_config  # noqa: E402  (worldfiles puts sim/ on the path)
+import viewers  # noqa: E402
 from urc import meshes  # noqa: E402
@@ -368,3 +369,3 @@ class Flight(unittest.TestCase):
                                     (6.0, viewers.FLY_MODE_TOPIC, text("free"))],
-                holds=[(0.0, 9.0, gen_model.CMD_VEL_TOPIC, twist(0.6))], track=["rover"])
+                holds=[(0.0, 9.0, rover_config.CMD_VEL_TOPIC, twist(0.6))], track=["rover"])
 
diff --git a/rover_sim/tests/test_foundations.py b/rover_sim/tests/test_foundations.py
index b77ca04..e32ef9d 100644
--- a/rover_sim/tests/test_foundations.py
+++ b/rover_sim/tests/test_foundations.py
@@ -23,4 +23,3 @@ from worldfiles import MODELS, SIM_DIR, temp_sdf
 
-import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
-import gzenv  # noqa: E402
+import gzenv  # noqa: E402  (worldfiles puts sim/ on the path)
 import rover_config  # noqa: E402
@@ -276,3 +275,3 @@ class SimulateRuns(unittest.TestCase):
         callback sets friction, so this needs the DiffDrive variant."""
-        square = dataclasses.replace(diffdrive(), mu_lateral=1.0)
+        square = diffdrive(diffdrive={"mu_lateral": 1.0})
         yaw = simulate(2.0, cmd=(0.0, 1.0), params=square).poses["base_link"][5]
@@ -297,3 +296,3 @@ class CostHelpers(unittest.TestCase):
         xs = []
-        node.subscribe(Odometry, gen_model.GROUND_TRUTH_TOPIC, lambda m: xs.append(m.pose.position.x))
+        node.subscribe(Odometry, rover_config.GROUND_TRUTH_TOPIC, lambda m: xs.append(m.pose.position.x))
         with temp_sdf(world_sdf()) as world:
diff --git a/rover_sim/tests/test_render.py b/rover_sim/tests/test_render.py
index 91a885c..f6046b1 100644
--- a/rover_sim/tests/test_render.py
+++ b/rover_sim/tests/test_render.py
@@ -3,3 +3,3 @@
 field, merged GLB clutter, what the haze leaves alone (depth), the
-record of the rover's opt-in dust (gen_model.DriveParams.dust, off by
+record of the rover's opt-in dust (sim.yaml dust, off by
 default since the user's decision of 2026-10-07): drawn, and seen by depth,
@@ -17,3 +17,2 @@ directory) or Gazebo's stock media.
 import contextlib
-import dataclasses
 import json
@@ -38,4 +37,7 @@ from worldfiles import SIM_DIR
 
-import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
+import gen_rover  # noqa: E402  (worldfiles puts sim/ and sim/tools on the path)
 import gzenv  # noqa: E402
+import rover_config  # noqa: E402
+import viewers  # noqa: E402
+from rover_config import ROVER, Variant  # noqa: E402
 from urc import appearance as A  # noqa: E402
@@ -223,5 +225,5 @@ def aim(eye, target):
 def drive_world(world_name, rover, params, cameras, extra=""):
-    """A copy of sim/worlds/<world_name>.sdf to drive in: as fast as it runs, the rover built from gen_model
-    Params `params` (its model written beside, see Renderer.drive_variant) at rover = (x, y, yaw) 0.25 m above the
-    terrain, the station's chase camera and rover eye (the models it spawns), and static cameras
+    """A copy of sim/worlds/<world_name>.sdf to drive in: as fast as it runs, the rover built from the
+    rover_config.Variant `params` (its model written beside, see Renderer.drive_variant) at rover = (x, y, yaw)
+    0.25 m above the terrain, the station's chase camera and rover eye (the models it spawns), and static cameras
     [(name, eye, target, kind, size)] (kind "camera" or "rgbd_camera"), eye and target `z` above the terrain."""
@@ -243,3 +245,3 @@ def drive_world(world_name, rover, params, cameras, extra=""):
         body += camera(name, eye, *aim(eye, target), size=size, hfov=1.0, kind=kind)
-    return text.replace("</world>", body + extra + "\n  </world>"), gen_model.build_sdf(params)
+    return text.replace("</world>", body + extra + "\n  </world>"), gen_rover.build_sdf(params)
 
@@ -276,4 +278,4 @@ def _drive_main(cfg_path):
     states = []
-    node.subscribe(StringMsg, gen_model.DRIVETRAIN_TOPIC, lambda m: states.append((phase[0], m.data)))
-    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
+    node.subscribe(StringMsg, rover_config.DRIVETRAIN_TOPIC, lambda m: states.append((phase[0], m.data)))
+    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
     twist, handles, trace = Twist(), {}, []
@@ -348,3 +350,3 @@ def _render_main(cfg_path):
         node.subscribe(Image, topic, lambda msg, t=topic: last.__setitem__(t, msg))
-    node.subscribe(StringMsg, gen_model.DRIVETRAIN_TOPIC, lambda msg: state.__setitem__("drivetrain", msg.data))
+    node.subscribe(StringMsg, rover_config.DRIVETRAIN_TOPIC, lambda msg: state.__setitem__("drivetrain", msg.data))
     fixture = TestFixture(cfg["world"])
@@ -353,3 +355,3 @@ def _render_main(cfg_path):
     if commands:
-        publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
+        publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
         twist = Twist()
@@ -542,8 +544,11 @@ class Render(unittest.TestCase):
     def dust():
-        """The rover's own dust emitter (gen_model._add_dust_emitter, which the rover has only with
-        DriveParams.dust; its sprite from sim/models/rover) emitting at 400 /s, ten times the drivetrain's most,
-        2 m in front of the camera."""
+        """The rover's own dust emitter (the left one of the rover with sim.yaml dust on, which the rover has only
+        then; its sprite from sim/models/rover) emitting at 400 /s, ten times the drivetrain's most, 2 m in front of
+        the camera."""
+        rover = ET.fromstring(gen_rover.build_sdf(Variant().replace(dust={"enabled": True})))
+        emitter = rover.find("model/link[@name='rocker_left']/particle_emitter")
+        emitter.set("name", "e")
+        emitter.find("topic").text = rover_config.DUST_TOPIC.format(link="link", emitter="e")
         link = ET.Element("link", name="link")
-        gen_model._add_dust_emitter(link, gen_model.Params(), "e")
-        emitter = link.find("particle_emitter")
+        link.append(emitter)
         emitter.find("emitting").text, emitter.find("rate").text = "true", "400"
@@ -565,3 +570,3 @@ class Render(unittest.TestCase):
     def test_dust_is_drawn(self):
-        """The rover's opt-in dust (DriveParams.dust, off by default) shows in
+        """The rover's opt-in dust (sim.yaml dust, off by default) shows in
         the colour image as dust: pale tan, its sprite's colour
@@ -588,6 +593,6 @@ class Render(unittest.TestCase):
         depth image, and it renders black. Hence the user's decision of
-        2026-10-07: the rover has no dust by default (DriveParams.dust off,
-        test_gen_model.Dust), and Q11 is met by its absence. Kept as an
+        2026-10-07: the rover has no dust by default (sim.yaml dust off,
+        test_rover_model.Dust), and Q11 is met by its absence. Kept as an
         expected failure, the record for the opt-in switch: the emitter asks
-        for a scatter ratio near none (DriveParams.dust_scatter_ratio; with
+        for a scatter ratio near none (sim.yaml dust scatter_ratio; with
         none a gz-rendering that honours it would apply 0.65), so the test
@@ -655,3 +660,2 @@ class Render(unittest.TestCase):
 
-ROVER = gen_model.Params()
 SIDE = "/render/side"  # side_view_world's camera on the rover
@@ -661,3 +665,3 @@ SIDE_POSE = (0.0, 1.95, 0.25, 0.0, 0.0, -math.pi / 2)  # in base_link: level, lo
 SIDE_FOCAL = SIDE_SIZE[0] / 2 / math.tan(SIDE_HFOV / 2)  # [px]
-FACE = SIDE_POSE[1] - (ROVER.pivot_y + ROVER.wheel_width / 2)  # [m] the left tyres' outer faces from the camera
+FACE = SIDE_POSE[1] - (ROVER.rocker.pivot_y + ROVER.wheel.width / 2)  # [m] the left tyres' outer faces from the camera
 SPIN = [(0.0, 0.0, 0.0), (0.5, 0.0, 1.0), (5.0, 0.0, 0.0)]  # turn in place at 1 rad/s from 0.5 s to 5 s
@@ -669,3 +673,3 @@ def side_view_world(params, key):
     type `key` (terrains.TYPES, under the strong dig-in preset) with the
-    rover built from gen_model Params `params` at the origin, and a camera
+    rover built from the rover_config.Variant `params` at the origin, and a camera
     on its base_link 1.5 m beyond its left tyres (SIDE, SIDE_POSE), level. A
@@ -684,3 +688,3 @@ def side_view_world(params, key):
                f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>')
-    root = ET.fromstring(gen_model.build_sdf(params))
+    root = ET.fromstring(gen_rover.build_sdf(params))
     sensor = sdf.sub(root.find("model/link[@name='base_link']"), "sensor", name="side", type="rgbd_camera")
@@ -728,4 +732,4 @@ class DigCues(unittest.TestCase):
     only (the user's decisions of 2026-10-07): the tyre sink
-    (DriveParams.dig_sink: a dug wheel's tyre drawn (D - 1) x the ground's
-    static sinkage lower, true scale) and the tread tyre (Params.tread_tyre:
+    (sim.yaml dig_sink: a dug wheel's tyre drawn (D - 1) x the ground's
+    static sinkage lower, true scale) and the tread tyre (sim.yaml tread_tyre:
     spokes, one of them ochre). The rover turns in place at 1 rad/s on
@@ -745,4 +749,4 @@ class DigCues(unittest.TestCase):
         from simulate import physical  # here: see _render_main
-        cues, no_sink = physical(), physical(dig_sink=False)  # the defaults: both cues on
-        plain = dataclasses.replace(no_sink, tread_tyre=False)  # the rover before the cues
+        cues, no_sink = physical(), physical(dig_sink={"enabled": False})  # the defaults: both cues on
+        plain = no_sink.replace(tread_tyre=False)  # the rover before the cues
         rock_spin = [(0.0, 0.0, 0.0), (0.5, 0.0, 1.0), (3.0, 0.0, 0.0)]
@@ -764,3 +768,3 @@ class DigCues(unittest.TestCase):
         lower in the side camera's depth image than without the sink, +-1.5
-        px (13.3 px at 1.5 m): the true scale (DriveParams.dig_sink_gain 1).
+        px (13.3 px at 1.5 m): the true scale (sim.yaml dig_sink gain 1).
         The physics is the same (the drivetrain states are equal)."""
@@ -769,7 +773,7 @@ class DigCues(unittest.TestCase):
         s = terrains.SAND.traction.sinkage_m
-        for wheel, x in (("fl", ROVER.wheel_dx), ("rl", -ROVER.wheel_dx)):
+        for wheel, x in (("fl", ROVER.wheel.dx), ("rl", -ROVER.wheel.dx)):
             dig = sunk.drivetrain["wheels"][wheel]["dig"]
             self.assertGreater(dig, 1.95, wheel)
-            u, v = side_px(x, SIDE_POSE[1] - FACE, ROVER.wheel_radius)
-            top = round(v - SIDE_FOCAL * ROVER.wheel_radius / FACE)
+            u, v = side_px(x, SIDE_POSE[1] - FACE, ROVER.wheel.radius)
+            top = round(v - SIDE_FOCAL * ROVER.wheel.radius / FACE)
             rows = range(top - 20, round(v))
@@ -795,5 +799,5 @@ class DigCues(unittest.TestCase):
         boxes = np.zeros(changed.shape, bool)
-        r, w, m = ROVER.wheel_radius, ROVER.wheel_width, 0.03
-        for x in (ROVER.wheel_dx, -ROVER.wheel_dx):
-            for y in (ROVER.pivot_y, -ROVER.pivot_y):
+        r, w, m = ROVER.wheel.radius, ROVER.wheel.width, 0.03
+        for x in (ROVER.wheel.dx, -ROVER.wheel.dx):
+            for y in (ROVER.rocker.pivot_y, -ROVER.rocker.pivot_y):
                 corners = [side_px(x + dx, y + dy, r + dz) for dx in (-r - m, r + m) for dy in (-w / 2 - m, w / 2 + m)
@@ -806,3 +810,3 @@ class DigCues(unittest.TestCase):
         def tyre(depth):  # pixels at a tyre's depth: the near tyres' outer faces and tread, the far ones'
-            near, far = (SIDE_POSE[1] + side * ROVER.pivot_y for side in (-1, 1))
+            near, far = (SIDE_POSE[1] + side * ROVER.rocker.pivot_y for side in (-1, 1))
             with np.errstate(invalid="ignore"):
@@ -845,3 +849,3 @@ class DigCues(unittest.TestCase):
     def test_tread_tyre_shows_the_spin(self):
-        """The tread tyre (Params.tread_tyre) on a wheel spinning in place, the
+        """The tread tyre (sim.yaml tread_tyre) on a wheel spinning in place, the
         rover dug in and still: its face is not uniform (light hub and spokes
@@ -851,4 +855,4 @@ class DigCues(unittest.TestCase):
         tread, plain = self.runs["no_sink"], self.runs["plain"]  # no sink: the tyres where they are drawn plain
-        u, v = side_px(ROVER.wheel_dx, SIDE_POSE[1] - FACE, ROVER.wheel_radius)
-        radius = SIDE_FOCAL * ROVER.wheel_radius / FACE
+        u, v = side_px(ROVER.wheel.dx, SIDE_POSE[1] - FACE, ROVER.wheel.radius)
+        radius = SIDE_FOCAL * ROVER.wheel.radius / FACE
         vv, uu = np.mgrid[:SIDE_SIZE[1], :SIDE_SIZE[0]]
@@ -898,3 +902,3 @@ def back_project(depth, eye, target, hfov=1.0):
 class Ruts(unittest.TestCase):
-    """The ruts and pits behind the wheels (gen_model.DriveParams.ruts, on by
+    """The ruts and pits behind the wheels (sim.yaml ruts, on by
     default since the user's decisions of 2026-10-07; plugins/rover_tracks.hh,
@@ -917,3 +921,3 @@ class Ruts(unittest.TestCase):
                ("d", *DEPTH, "rgbd_camera", (640, 360))]
-    TOPICS = [gen_model.CHASE_IMAGE_TOPIC, gen_model.EYE_IMAGE_TOPIC, "/render/top", "/render/side",
+    TOPICS = [viewers.CHASE_IMAGE_TOPIC, viewers.EYE_IMAGE_TOPIC, "/render/top", "/render/side",
               "/render/d/image", "/render/d/depth_image"]
@@ -932,3 +936,3 @@ class Ruts(unittest.TestCase):
             cameras = [(n, (e[0], e[1] + dy, e[2]), (t[0], t[1] + dy, t[2]), k, s) for n, e, t, k, s in cls.CAMERAS]
-            text, rover = drive_world(cls.WORLD, pose, physical(ruts=ruts), cameras)
+            text, rover = drive_world(cls.WORLD, pose, physical(ruts={"enabled": ruts}), cameras)
             rock = label.startswith("rock")
@@ -961,6 +965,5 @@ class Ruts(unittest.TestCase):
         trace = trace[trace[:, 0] == phase]
-        p = gen_model.Params()
+        a, c = ROVER.wheel.dx, ROVER.rocker.pivot_y
         out = []
-        for dx, dy in ((p.wheel_dx, p.pivot_y), (p.wheel_dx, -p.pivot_y), (-p.wheel_dx, p.pivot_y),
-                       (-p.wheel_dx, -p.pivot_y)):
+        for dx, dy in ((a, c), (a, -c), (-a, c), (-a, -c)):
             c, s = np.cos(trace[:, 5]), np.sin(trace[:, 5])
@@ -986,3 +989,3 @@ class Ruts(unittest.TestCase):
         and where they are."""
-        for topic, share in (("/render/top", 0.02), ("/render/side", 0.02), (gen_model.CHASE_IMAGE_TOPIC, 0.005)):
+        for topic, share in (("/render/top", 0.02), ("/render/side", 0.02), (viewers.CHASE_IMAGE_TOPIC, 0.005)):
             self.assertGreater(self.changed(topic, 0).mean(), share, topic)
@@ -1017,6 +1020,6 @@ class Ruts(unittest.TestCase):
         row = trace[(trace[:, 0] == 0) & (trace[:, 1] <= 11.0)][-1]  # where the wheels dug in
-        p = gen_model.Params()
+        a, b = ROVER.wheel.dx, ROVER.rocker.pivot_y
         c, s = math.cos(row[5]), math.sin(row[5])
         wheels = np.array([(row[2] + c * dx - s * dy, row[3] + s * dx + c * dy)
-                           for dx in (p.wheel_dx, -p.wheel_dx) for dy in (p.pivot_y, -p.pivot_y)])
+                           for dx in (a, -a) for dy in (b, -b)])
         eye, target = ((q[0], q[1], q[2] + self.hf.height(q[0], q[1])) for q in self.DEPTH)
@@ -1068,4 +1071,4 @@ class Ruts(unittest.TestCase):
         and ruts lie ahead of it and the eye shows them."""
-        self.assertFalse(self.changed(gen_model.EYE_IMAGE_TOPIC, 1, 0).any())
-        self.assertGreater(self.changed(gen_model.EYE_IMAGE_TOPIC, 2).mean(), 0.01)
+        self.assertFalse(self.changed(viewers.EYE_IMAGE_TOPIC, 1, 0).any())
+        self.assertGreater(self.changed(viewers.EYE_IMAGE_TOPIC, 2).mean(), 0.01)
 
@@ -1108,3 +1111,3 @@ class Ruts(unittest.TestCase):
         for ruts in (False, True):
-            rover = gen_model.build_sdf(physical(ruts=ruts, default_surface="sand"))
+            rover = gen_rover.build_sdf(physical(ruts={"enabled": ruts}, contact={"default_surface": "sand"}))
             shots[ruts], trace, _, _, _ = self.r.drive_variant(world(body), rover, dict(
diff --git a/rover_sim/tests/test_rover_sim.py b/rover_sim/tests/test_rover_sim.py
index 1da4ce1..2c659dd 100644
--- a/rover_sim/tests/test_rover_sim.py
+++ b/rover_sim/tests/test_rover_sim.py
@@ -2,3 +2,2 @@
 """Headless physics tests for the rover model (pixi run sim-test)."""
-import dataclasses
 import json
@@ -7,8 +6,9 @@ import unittest
 
-from simulate import diffdrive, gen_model, physical, simulate, spin_ratio
+from simulate import diffdrive, physical, simulate, spin_ratio
 from worldfiles import world_copy
 
-from urc import terrains  # noqa: E402  (worldfiles puts sim/ on the path)
+import rover_config  # noqa: E402  (worldfiles puts sim/ on the path)
+from rover_config import ROVER  # noqa: E402
+from urc import terrains  # noqa: E402
 
-P = gen_model.Params()
 STEP = 0.10
@@ -17,3 +17,3 @@ BLOCK_UNDER_FL = f"""
       <static>true</static>
-      <pose>{P.wheel_dx} {P.pivot_y} {STEP / 2} 0 0 0</pose>
+      <pose>{ROVER.wheel.dx} {ROVER.rocker.pivot_y} {STEP / 2} 0 0 0</pose>
       <link name="link">
@@ -40,3 +40,3 @@ class Suspension(unittest.TestCase):
         self.assertLess(abs(q_l + q_r), 0.005)
-        self.assertAlmostEqual(q_l, -math.atan(STEP / (2 * P.wheel_dx)) / 2, delta=0.01)
+        self.assertAlmostEqual(q_l, -math.atan(STEP / (2 * ROVER.wheel.dx)) / 2, delta=0.01)
         # A rigid chassis would leave one wheel in the air.
@@ -44,3 +44,3 @@ class Suspension(unittest.TestCase):
         for wheel, height in ground.items():
-            self.assertAlmostEqual(s.poses[wheel][2], height + P.wheel_radius, delta=0.02, msg=wheel)
+            self.assertAlmostEqual(s.poses[wheel][2], height + ROVER.wheel.radius, delta=0.02, msg=wheel)
 
@@ -76,4 +76,4 @@ class Topics(unittest.TestCase):
 
-        topics = [(gen_model.IMU_TOPIC, IMU), (gen_model.JOINT_STATE_TOPIC, Model),
-                  (gen_model.ODOM_TOPIC, Odometry), (gen_model.GROUND_TRUTH_TOPIC, Odometry)]
+        topics = [(rover_config.IMU_TOPIC, IMU), (rover_config.JOINT_STATE_TOPIC, Model),
+                  (rover_config.ODOM_TOPIC, Odometry), (rover_config.GROUND_TRUTH_TOPIC, Odometry)]
         s = simulate(1.0, subscribe=topics)
@@ -81,3 +81,3 @@ class Topics(unittest.TestCase):
             self.assertTrue(s.messages[topic], f"nothing on {topic}")
-        imu = s.messages[gen_model.IMU_TOPIC][-1]
+        imu = s.messages[rover_config.IMU_TOPIC][-1]
         self.assertEqual(frame_id(imu.header), "base_link")
@@ -85,6 +85,6 @@ class Topics(unittest.TestCase):
         self.assertAlmostEqual(math.sqrt(a.x**2 + a.y**2 + a.z**2), 9.8, delta=0.3)
-        joints = {j.name for j in s.messages[gen_model.JOINT_STATE_TOPIC][-1].joint}
+        joints = {j.name for j in s.messages[rover_config.JOINT_STATE_TOPIC][-1].joint}
         self.assertEqual(joints, {"rocker_left_joint", "rocker_right_joint", "wheel_fl_joint",
                                   "wheel_rl_joint", "wheel_fr_joint", "wheel_rr_joint",
-                                  gen_model.PAN_JOINT, gen_model.TILT_JOINT})
+                                  rover_config.PAN_JOINT, rover_config.TILT_JOINT})
 
@@ -92,4 +92,4 @@ class Topics(unittest.TestCase):
 class PhysicalRover(unittest.TestCase):
-    """The rover with the physical drivetrain (the default, DriveParams mode
-    "physical", plugins/rover_drivetrain.cpp) on the flat ground of these
+    """The rover with the physical drivetrain (the default, sim.yaml's
+    drivetrain mode physical, plugins/rover_drivetrain.cpp) on the flat ground of these
     tests: a plane, ground of the drivetrain's default surface
@@ -99,3 +99,3 @@ class PhysicalRover(unittest.TestCase):
     PARAMS = physical()
-    GROUND = terrains.TYPES[PARAMS.drive.default_surface].traction
+    GROUND = terrains.TYPES[PARAMS.sim.contact.default_surface].traction
 
@@ -105,6 +105,6 @@ class PhysicalRover(unittest.TestCase):
         self.assertLess(abs(q_l + q_r), 0.005)
-        self.assertAlmostEqual(q_l, -math.atan(STEP / (2 * P.wheel_dx)) / 2, delta=0.01)
+        self.assertAlmostEqual(q_l, -math.atan(STEP / (2 * ROVER.wheel.dx)) / 2, delta=0.01)
         ground = {"wheel_fl": STEP, "wheel_fr": 0.0, "wheel_rl": 0.0, "wheel_rr": 0.0}
         for wheel, height in ground.items():
-            self.assertAlmostEqual(s.poses[wheel][2], height + P.wheel_radius, delta=0.02, msg=wheel)
+            self.assertAlmostEqual(s.poses[wheel][2], height + ROVER.wheel.radius, delta=0.02, msg=wheel)
 
@@ -120,3 +120,4 @@ class PhysicalRover(unittest.TestCase):
         surface (design spec 5.6), not DiffDrive's 0.89."""
-        s = simulate(6.0, cmd=[(0.0, 0.0, 0.0), (0.5, 0.0, 1.0)], params=physical(dig=False), trace_every=10)
+        s = simulate(6.0, cmd=[(0.0, 0.0, 0.0), (0.5, 0.0, 1.0)], params=physical(contact={"dig": False}),
+                     trace_every=10)
         rate = s.trace[s.trace[:, 0] >= 3.0, 7].mean()
@@ -130,10 +131,10 @@ class PhysicalRover(unittest.TestCase):
 
-        topics = [(gen_model.ODOM_TOPIC, Odometry), (gen_model.DRIVETRAIN_TOPIC, StringMsg)]
+        topics = [(rover_config.ODOM_TOPIC, Odometry), (rover_config.DRIVETRAIN_TOPIC, StringMsg)]
         s = simulate(1.0, cmd=(0.3, 0.0), subscribe=topics, params=self.PARAMS)
-        odom = s.messages[gen_model.ODOM_TOPIC][-1]
+        odom = s.messages[rover_config.ODOM_TOPIC][-1]
         self.assertEqual(frame_id(odom.header), "odom")
         self.assertGreater(odom.pose.position.x, 0.05)
-        state = json.loads(s.messages[gen_model.DRIVETRAIN_TOPIC][-1].data)
+        state = json.loads(s.messages[rover_config.DRIVETRAIN_TOPIC][-1].data)
         self.assertEqual(state["cmd"], [0.3, 0.0])
-        self.assertEqual({w["surface"] for w in state["wheels"].values()}, {self.PARAMS.drive.default_surface})
+        self.assertEqual({w["surface"] for w in state["wheels"].values()}, {self.PARAMS.sim.contact.default_surface})
 
@@ -141,3 +142,3 @@ class PhysicalRover(unittest.TestCase):
         """The test ground, with its particle-emitter system (idle: the rover has
-        dust emitters only with DriveParams.dust, off by default)."""
+        dust emitters only with sim.yaml dust on, off by default)."""
         with world_copy("rover_test") as world:
@@ -149,3 +150,3 @@ class PhysicalRover(unittest.TestCase):
     def test_tyre_compliance_rests_and_drives(self):
-        params = dataclasses.replace(self.PARAMS, tire_compliance=True)
+        params = self.PARAMS.replace(tyre_compliance={"enabled": True})
         s = simulate(4.0, cmd=(0.5, 0.0), params=params)
diff --git a/rover_sim/tests/test_station.py b/rover_sim/tests/test_station.py
index 2ec5892..9076394 100644
--- a/rover_sim/tests/test_station.py
+++ b/rover_sim/tests/test_station.py
@@ -31,6 +31,8 @@ from aiohttp.test_utils import AioHTTPTestCase
 import simulate
-from simulate import gen_model, world_sdf
+from simulate import world_sdf
 from worldfiles import SENSORS, SIM_DIR, WORLDS, temp_sdf
 
-import viewers  # noqa: E402  (worldfiles puts sim/ on the path)
+import rover_config  # noqa: E402  (worldfiles puts sim/ on the path)
+import viewers  # noqa: E402
+from rover_config import ROVER, SIM  # noqa: E402
 from station import drive, minimap, server, video  # noqa: E402
@@ -54,3 +56,2 @@ import render_map  # noqa: E402  (server.py puts sim/tools on the path)
 
-P = gen_model.Params()
 FLY = viewers.FlyParams()
@@ -77,3 +78,3 @@ class DriveMapping(unittest.TestCase):
                                     min(2 * drive.PRESETS[3][1], drive.MAX_ANGULAR)))
-        self.assertLessEqual(vx, P.wheel_speed * P.wheel_radius)
+        self.assertLessEqual(vx, ROVER.wheel.top_speed * ROVER.wheel.radius)
         self.assertEqual(drive.target(command("KeyW", "GamepadFast"), 1)[0], 2 * linear)
@@ -165,3 +166,3 @@ class DriveLoop(unittest.TestCase):
         self.assertEqual(link.sent, [])
-        self.assertEqual((station.look.pan, station.look.tilt), (0.0, P.camera_pitch))
+        self.assertEqual((station.look.pan, station.look.tilt), (0.0, ROVER.head.home_tilt))
 
@@ -173,3 +174,3 @@ class LookMapping(unittest.TestCase):
         look = drive.Look()
-        self.assertEqual((look.pan, look.tilt), (0.0, P.camera_pitch))
+        self.assertEqual((look.pan, look.tilt), (0.0, ROVER.head.home_tilt))
         look.nudge(0.3, 0.1)
@@ -177,12 +178,12 @@ class LookMapping(unittest.TestCase):
         self.assertAlmostEqual(look.pan, 0.2)
-        self.assertAlmostEqual(look.tilt, P.camera_pitch + 0.15)
+        self.assertAlmostEqual(look.tilt, ROVER.head.home_tilt + 0.15)
         look.nudge(10.0, -10.0)
-        self.assertEqual((look.pan, look.tilt), (P.camera_pan_limit, P.camera_tilt_limits[0]))
+        self.assertEqual((look.pan, look.tilt), (ROVER.head.pan_limit, ROVER.head.tilt_limits[0]))
         look.nudge(-20.0, 20.0)
-        self.assertEqual((look.pan, look.tilt), (-P.camera_pan_limit, P.camera_tilt_limits[1]))
+        self.assertEqual((look.pan, look.tilt), (-ROVER.head.pan_limit, ROVER.head.tilt_limits[1]))
         look.center()
-        self.assertEqual((look.pan, look.tilt), (0.0, P.camera_pitch))
+        self.assertEqual((look.pan, look.tilt), (0.0, ROVER.head.home_tilt))
 
     def test_the_eye_model_starts_and_stops_like_the_head(self):
-        plugin = ET.fromstring(gen_model.build_eye_sdf(gen_model.EyeParams(), P)).find("model/plugin")
+        plugin = ET.fromstring(viewers.build_eye_sdf(viewers.EyeParams(), ROVER)).find("model/plugin")
         value = lambda tag: float(plugin.findtext(tag))  # noqa: E731
@@ -199,3 +200,4 @@ class LookMapping(unittest.TestCase):
         station.handle({"t": "look", "pan": 0.5, "tilt": -0.2}, 0.0)
-        self.assertEqual(link.sent, [("eye", 0.5, P.camera_pitch - 0.2), ("head", 0.5, P.camera_pitch - 0.2)])
+        self.assertEqual(link.sent, [("eye", 0.5, ROVER.head.home_tilt - 0.2),
+                                     ("head", 0.5, ROVER.head.home_tilt - 0.2)])
         station.set_control(False)
@@ -204,3 +206,3 @@ class LookMapping(unittest.TestCase):
         station.handle({"t": "look_center"}, 0.2)
-        self.assertEqual(link.sent, [("eye", 0.6, P.camera_pitch - 0.2), ("eye", 0.0, P.camera_pitch)])
+        self.assertEqual(link.sent, [("eye", 0.6, ROVER.head.home_tilt - 0.2), ("eye", 0.0, ROVER.head.home_tilt)])
 
@@ -211,3 +213,3 @@ class LookMapping(unittest.TestCase):
             station.tick(k * server.CONTROL_PERIOD)
-        self.assertEqual(link.of("eye"), [("eye", 0.0, P.camera_pitch)] * 4)  # at 0, 1, 2 and 3 s
+        self.assertEqual(link.of("eye"), [("eye", 0.0, ROVER.head.home_tilt)] * 4)  # at 0, 1, 2 and 3 s
         self.assertEqual(link.of("head") + link.of("twist"), [])  # released: the head is autonomy's
@@ -385,3 +387,3 @@ class Video(unittest.TestCase):
     def test_depth_is_coloured_and_no_return_is_black(self):
-        bgr = video.to_bgr(image(video.R_FLOAT32), P.camera_clip)
+        bgr = video.to_bgr(image(video.R_FLOAT32), ROVER.camera.clip)
         self.assertEqual(bgr.shape, (24, 32, 3))
@@ -510,4 +512,4 @@ class Telemetry(unittest.TestCase):
         link.state["joints"] = {"rocker_left_joint": (0.03, 0.0), "rocker_right_joint": (-0.028, 0.0),
-                                "wheel_fl_joint": (1.0, 2.0), gen_model.PAN_JOINT: (0.1, 0.0),
-                                gen_model.TILT_JOINT: (0.2, 0.0)}
+                                "wheel_fl_joint": (1.0, 2.0), rover_config.PAN_JOINT: (0.1, 0.0),
+                                rover_config.TILT_JOINT: (0.2, 0.0)}
         t = server.Station(link, "test").telemetry(0.0)
@@ -515,5 +517,5 @@ class Telemetry(unittest.TestCase):
         self.assertAlmostEqual(t["rockers"]["error"], 0.002)  # what the differential keeps at 0
-        self.assertEqual(t["wheels"], {"fl": 2.0 * P.wheel_radius})  # ground speed of the wheel
+        self.assertEqual(t["wheels"], {"fl": 2.0 * ROVER.wheel.radius})  # ground speed of the wheel
         self.assertEqual(t["head"], {"pan": 0.1, "tilt": 0.2})
-        self.assertEqual(t["look"], {"pan": 0.0, "tilt": P.camera_pitch})  # what the station commands
+        self.assertEqual(t["look"], {"pan": 0.0, "tilt": ROVER.head.home_tilt})  # what the station commands
         json.dumps(t)
@@ -856,8 +858,8 @@ class App(AioHTTPTestCase):
         self.assertEqual(info["map"]["photo"], {"status": "missing", "changed": [], "url": None})
-        self.assertEqual(info["rover"]["wheel_dx"], P.wheel_dx)
+        self.assertEqual(info["rover"]["wheel_dx"], ROVER.wheel.dx)
         self.assertEqual(sorted(info["presets"]), ["1", "2", "3", "4", "5"])
-        self.assertEqual(info["hfov"], {"eye": gen_model.EyeParams().hfov, "chase": gen_model.ChaseParams().hfov,
-                                        "rgb": P.camera_hfov, "depth": P.camera_hfov, "fly": FLY.hfov})
+        self.assertEqual(info["hfov"], {"eye": viewers.EyeParams().hfov, "chase": viewers.ChaseParams().hfov,
+                                        "rgb": ROVER.camera.hfov, "depth": ROVER.camera.hfov, "fly": FLY.hfov})
         self.assertEqual(info["fly"]["aspect"], ASPECT)
-        self.assertEqual(info["drivetrain"]["current_limit"], P.drive.current_limit)
+        self.assertEqual(info["drivetrain"]["current_limit"], ROVER.motor.current_limit)
 
@@ -942,4 +944,4 @@ class App(AioHTTPTestCase):
             sent = self.link.of(kind)
-            self.assertIn((kind, 0.3, P.camera_pitch + 0.1), sent)
-            self.assertEqual(sent[-1], (kind, 0.0, P.camera_pitch))
+            self.assertIn((kind, 0.3, ROVER.head.home_tilt + 0.1), sent)
+            self.assertEqual(sent[-1], (kind, 0.0, ROVER.head.home_tilt))
         self.assertEqual(self.station.drive.preset, 3)
@@ -973,3 +975,3 @@ class App(AioHTTPTestCase):
             self.assertEqual(len(rover()), count)  # cmd_vel, LED and head left alone
-            self.assertEqual(self.link.of("eye")[-1], ("eye", 0.2, P.camera_pitch))  # the eye still looks
+            self.assertEqual(self.link.of("eye")[-1], ("eye", 0.2, ROVER.head.home_tilt))  # the eye still looks
             await ws.send_json({"t": "control", "on": True})
@@ -977,3 +979,3 @@ class App(AioHTTPTestCase):
             self.assertIn(("led", "blue"), rover()[count:])
-            self.assertIn(("head", 0.2, P.camera_pitch), rover()[count:])  # the head catches up
+            self.assertIn(("head", 0.2, ROVER.head.home_tilt), rover()[count:])  # the head catches up
 
@@ -1107,9 +1109,9 @@ class ChaseCameraFlight(unittest.TestCase):
     def test_follows_orbits_and_stays_above(self):
-        c = gen_model.ChaseParams()
-        extra = (f"<include><uri>model://{gen_model.CHASE_MODEL}</uri><name>{gen_model.CHASE_MODEL}</name>"
+        c = viewers.ChaseParams()
+        extra = (f"<include><uri>model://{viewers.CHASE_MODEL}</uri><name>{viewers.CHASE_MODEL}</name>"
                  "<pose>-5 0 2 0 0 0</pose></include>")
         node = Node()
-        twist_pub = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
-        cmd_pub = node.advertise(gen_model.CHASE_CMD_TOPIC, Vector3d)
-        mode_pub = node.advertise(gen_model.CHASE_MODE_TOPIC, StringMsg)
+        twist_pub = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
+        cmd_pub = node.advertise(viewers.CHASE_CMD_TOPIC, Vector3d)
+        mode_pub = node.advertise(viewers.CHASE_MODE_TOPIC, StringMsg)
         twist = Twist()
@@ -1133,3 +1135,3 @@ class ChaseCameraFlight(unittest.TestCase):
                 models["rover"] = Model(world.model_by_name(ecm, "rover"))
-                models["camera"] = Model(world.model_by_name(ecm, gen_model.CHASE_MODEL))
+                models["camera"] = Model(world.model_by_name(ecm, viewers.CHASE_MODEL))
             if info.iterations % 20 == 0:
@@ -1280,3 +1282,3 @@ class EyeCameraRide(unittest.TestCase):
                  "</link></model>"
-                 f"<include><uri>model://{gen_model.EYE_MODEL}</uri><name>{gen_model.EYE_MODEL}</name>"
+                 f"<include><uri>model://{viewers.EYE_MODEL}</uri><name>{viewers.EYE_MODEL}</name>"
                  "<pose>0 0 3 0 0 0</pose></include>")
@@ -1285,4 +1287,4 @@ class EyeCameraRide(unittest.TestCase):
         self.addCleanup(lambda node=station.link.node: [node.unsubscribe(t) for t in node.subscribed_topics()])
-        raw_look = Node().advertise(gen_model.EYE_LOOK_TOPIC, Vector3d)  # one advertiser per node and topic
-        mount = gz.math7.Pose3d(gz.math7.Vector3d(*P.camera_xyz), gz.math7.Quaterniond())
+        raw_look = Node().advertise(viewers.EYE_LOOK_TOPIC, Vector3d)  # one advertiser per node and topic
+        mount = gz.math7.Pose3d(gz.math7.Vector3d(*ROVER.camera.xyz), gz.math7.Quaterniond())
         # (sim time [s], message to the station); keys W from 0.5 s, W + A (turn left) from 4.5 s.
@@ -1290,4 +1292,4 @@ class EyeCameraRide(unittest.TestCase):
                  7.0: {"t": "look_center"}}
-        expected = {3.0: (0.8, P.camera_pitch + 0.3), 6.0: (-0.7, P.camera_pitch - 0.3),
-                    8.5: (0.0, P.camera_pitch)}
+        expected = {3.0: (0.8, ROVER.head.home_tilt + 0.3), 6.0: (-0.7, ROVER.head.home_tilt - 0.3),
+                    8.5: (0.0, ROVER.head.home_tilt)}
         handles = {}
@@ -1301,5 +1303,5 @@ class EyeCameraRide(unittest.TestCase):
                 handles["base"] = Link(rover.link_by_name(ecm, "base_link"))
-                handles["eye"] = Link(Model(world.model_by_name(ecm, gen_model.EYE_MODEL)).canonical_link(ecm))
-                handles["joints"] = [Joint(rover.joint_by_name(ecm, j)) for j in (gen_model.PAN_JOINT,
-                                                                                 gen_model.TILT_JOINT)]
+                handles["eye"] = Link(Model(world.model_by_name(ecm, viewers.EYE_MODEL)).canonical_link(ecm))
+                handles["joints"] = [Joint(rover.joint_by_name(ecm, j)) for j in (rover_config.PAN_JOINT,
+                                                                                 rover_config.TILT_JOINT)]
                 for joint in handles["joints"]:
@@ -1352,5 +1354,5 @@ class EyeCameraRide(unittest.TestCase):
         _, pitch, yaw = samples[8.9]["look"]
-        self.assertAlmostEqual(yaw, P.camera_pan_limit, delta=0.01)
-        self.assertAlmostEqual(pitch, P.camera_tilt_limits[0], delta=0.01)
-        for got, want in zip(samples[8.9]["joints"], (0.0, P.camera_pitch)):
+        self.assertAlmostEqual(yaw, ROVER.head.pan_limit, delta=0.01)
+        self.assertAlmostEqual(pitch, ROVER.head.tilt_limits[0], delta=0.01)
+        for got, want in zip(samples[8.9]["joints"], (0.0, ROVER.head.home_tilt)):
             self.assertAlmostEqual(got, want, delta=0.02)
@@ -1402,3 +1404,3 @@ class DrivetrainReadout(unittest.TestCase):
             self.assertGreater(abs(w["tau"]), 1.0, name)
-            self.assertEqual(w["surface"], P.drive.default_surface, name)
+            self.assertEqual(w["surface"], SIM.contact.default_surface, name)
             self.assertFalse(w["sat"], name)
diff --git a/rover_sim/tests/test_urc_missions.py b/rover_sim/tests/test_urc_missions.py
index 4a256ff..80986a9 100644
--- a/rover_sim/tests/test_urc_missions.py
+++ b/rover_sim/tests/test_urc_missions.py
@@ -14,3 +14,3 @@ from worldfiles import terrain as world_terrain
 
-import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
+from rover_config import ROVER  # noqa: E402  (worldfiles puts sim/ on the path)
 from urc import appearance, dem, lander, landscape, props, realism, routes, rules, sdf, terrain, terrains  # noqa: E402
@@ -80,3 +80,3 @@ class Autonomy(unittest.TestCase):
     ROCK_STOP = 0.25  # [m] the rover crosses 0.2 m rocks, 0.3 m ones stop it (proving ground)
-    ROVER_HALF_WIDTH = gen_model.Params().pivot_y + gen_model.Params().wheel_width / 2  # [m] over the wheels
+    ROVER_HALF_WIDTH = ROVER.rocker.pivot_y + ROVER.wheel.width / 2  # [m] over the wheels
 
diff --git a/rover_sim/tests/test_urc_sim.py b/rover_sim/tests/test_urc_sim.py
index 731af17..4865030 100644
--- a/rover_sim/tests/test_urc_sim.py
+++ b/rover_sim/tests/test_urc_sim.py
@@ -28,3 +28,4 @@ from gz.transport13 import Node
 
-import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
+import rover_config  # noqa: E402  (worldfiles puts sim/ on the path)
+from rover_config import ROVER  # noqa: E402
 from urc import geo, landscape, routes, terrains  # noqa: E402
@@ -54,3 +55,3 @@ DUG = 0.5  # how far a route planner takes digging ground's wheels to be dug in
 ROCK_STOP = 0.25  # [m] a rock standing higher than this stops the rover (proving ground: 0.2 m crossed, 0.3 m not)
-ROVER_HALF_WIDTH = gen_model.Params().pivot_y + gen_model.Params().wheel_width / 2  # [m] over the wheels
+ROVER_HALF_WIDTH = ROVER.rocker.pivot_y + ROVER.wheel.width / 2  # [m] over the wheels
 SLOW = unittest.skipUnless(os.environ.get("ROVER_SLOW"), "slow (many minutes of sim time): set ROVER_SLOW=1")
@@ -154,3 +155,3 @@ class Worlds(unittest.TestCase):
                     wx, wy, wz = state.poses[wheel][:3]
-                    self.assertAlmostEqual(wz - gen_model.Params().wheel_radius, ground.height(wx, wy), delta=0.06,
+                    self.assertAlmostEqual(wz - ROVER.wheel.radius, ground.height(wx, wy), delta=0.06,
                                            msg=f"{mission} {wheel}")
@@ -162,4 +163,4 @@ class Worlds(unittest.TestCase):
         with world_copy("urc_autonomy") as world:
-            state = simulate(4.0, world=world, subscribe=[(gen_model.NAVSAT_TOPIC, NavSat)])
-        fixes = state.messages[gen_model.NAVSAT_TOPIC][5:]
+            state = simulate(4.0, world=world, subscribe=[(rover_config.NAVSAT_TOPIC, NavSat)])
+        fixes = state.messages[rover_config.NAVSAT_TOPIC][5:]
         self.assertGreater(len(fixes), 20)
@@ -210,3 +211,3 @@ class Worlds(unittest.TestCase):
         with world_copy("urc_autonomy", rover=(x, y, post["yaw"] + math.pi), sensors=True) as world:
-            code = CAMERA_CHECK.format(tests=str(SIM_DIR / "tests"), topic=gen_model.CAMERA_TOPIC + "/image",
+            code = CAMERA_CHECK.format(tests=str(SIM_DIR / "tests"), topic=rover_config.CAMERA_TOPIC + "/image",
                                        world=world)
@@ -265,3 +266,3 @@ class RefereeRun(unittest.TestCase):
             ref = referee.Referee("autonomy", path, judge)
-            led = Node().advertise(gen_model.LED_TOPIC, StringMsg)
+            led = Node().advertise(rover_config.LED_TOPIC, StringMsg)
             msg = StringMsg()
diff --git a/rover_sim/tests/test_urc_terrain.py b/rover_sim/tests/test_urc_terrain.py
index 5187791..79b7cd2 100644
--- a/rover_sim/tests/test_urc_terrain.py
+++ b/rover_sim/tests/test_urc_terrain.py
@@ -28,3 +28,4 @@ from worldfiles import terrain as world_terrain
 
-import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
+import rover_config  # noqa: E402  (worldfiles puts sim/ on the path)
+from rover_config import ROVER  # noqa: E402
 from urc import appearance, dem, features, geo, landscape, lighting, meshes, terrain, terrains, textures  # noqa: E402
@@ -312,3 +313,3 @@ class SmallWorld(unittest.TestCase):
         names = [p.get("name") for p in world_sdf.iter("plugin")]
-        self.assertIn("gz::sim::systems::ParticleEmitter", names)  # wheel dust, with DriveParams.dust on
+        self.assertIn("gz::sim::systems::ParticleEmitter", names)  # wheel dust, with sim.yaml dust on
         for key, name in (("ground_map", "ground.png"), ("ground_legend", "ground.json")):
@@ -1032,3 +1033,3 @@ def wheel_poses(world, seconds, speed):
     node = Node()
-    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
+    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
     twist = Twist()
@@ -1063,5 +1064,5 @@ def wheel_penetration(rows, height):
     """How far [m] the surface height(x, y) reaches into each wheel's cylinder
-    (radius and width from gen_model.Params), per row of wheel_poses and wheel:
+    (radius and width from rover.yaml), per row of wheel_poses and wheel:
     the surface sampled every 5 mm under the hub."""
-    p = gen_model.Params()
+    wheel = ROVER.wheel
     grid = np.arange(-0.2, 0.2001, 0.005)
@@ -1077,4 +1078,4 @@ def wheel_penetration(rows, height):
             local = R.T @ P  # the wheel's frame: its axle along y
-            inside = (np.abs(local[1]) <= p.wheel_width / 2) & (P[2] < 0)
-            out.append(float(np.max(p.wheel_radius - np.hypot(local[0][inside], local[2][inside])))
+            inside = (np.abs(local[1]) <= wheel.width / 2) & (P[2] < 0)
+            out.append(float(np.max(wheel.radius - np.hypot(local[0][inside], local[2][inside])))
                        if inside.any() else 0.0)
@@ -1158,3 +1159,3 @@ class ProvingGroundPhysics(unittest.TestCase):
         pose = state.poses["base_link"]
-        self.assertGreater(state.rocker_peak, 0.9 * gen_model.Params().rocker_limit)
+        self.assertGreater(state.rocker_peak, 0.9 * ROVER.rocker.limit)
         self.assertGreater(pose[0], ditch["x"] + 2.0, pose)  # and drives on across
diff --git a/rover_sim/tools/realism_report.py b/rover_sim/tools/realism_report.py
index 18fe70d..ce4af7b 100644
--- a/rover_sim/tools/realism_report.py
+++ b/rover_sim/tools/realism_report.py
@@ -151,3 +151,3 @@ def spins():
         for key in keys:
-            trace = run(6.0, key, simulate.physical(dig=False), solver=solver)
+            trace = run(6.0, key, simulate.physical(contact={"dig": False}), solver=solver)
             measured = float(trace[(trace[:, 0] >= 3.0) & (trace[:, 0] < 6.0), 7].mean())
```

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src && git apply "$T/t12.patch" && git add -A && git status --short | wc -l \
  && git grep -n "gen_model\|DriveParams\|TrackParams" -- rover_sim/tests rover_sim/tools ':!rover_sim/tests/fixtures'
```

Expected: `11` changed files; the grep prints only the lines that name `gen_model.py` as the history the equivalence test checks against (`tests/sdf_equivalence.py` lines 2, 9 and 15, `tests/test_equivalence.py` lines 2 and 4, `tests/test_rover_model.py` line 3, `tools/gen_rover.py` lines 10, 92 and 104).

- [ ] **Step 2: Run the tests that build variants (a few minutes)**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && GZ_IP=127.0.0.1 /Users/alarion239/.pixi/bin/pixi run python -m unittest test_rover_sim test_foundations test_rover_model test_equivalence 2>&1 | grep -E "^(FAIL|ERROR):|^Ran|^FAILED|^OK"
```

Expected: `Ran 98 tests`, `OK` (13 + 28 + 46 + 11). The other files the patch touches run in Task 15's full suite.

- [ ] **Step 3: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add -A && git commit -q -F - <<'EOF'
PR 2: rover_sim tests: the rover's variants and numbers from rover_config

simulate.physical() and diffdrive() return a rover_config.Variant, sections of rover.yaml or sim.yaml changed
(physical(contact={"dig": False})), which rover_model and world_file build with gen_rover. The tests and the realism
report name the rover's numbers through rover_config (ROVER.wheel.radius, SIM.drivetrain.accel) and the station's
cameras through viewers; no assertion changes. test_render's dust emitter is the generated rover's own.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 13: The station, referee, judge and proving ground on rover_config; gen_model.py removed

**Files:**
- Modify: `rover_sim/station/drive.py`, `rover_sim/station/link.py`, `rover_sim/station/server.py`, `rover_sim/station/static/station.js`, `rover_sim/referee.py`, `rover_sim/urc/judge.py`, `rover_sim/urc/missions/proving_ground.py`
- Modify (comments only): `rover_sim/bridge.yaml`, `rover_sim/worlds/rover_test.sdf`, `rover_sim/plugins/rover_drivetrain.cpp`, `rover_sim/plugins/rover_drivetrain.hh`, `rover_sim/plugins/rover_tracks.hh`, `rover_sim/plugins/tests/test_rover_drivetrain.cpp`
- Remove: `rover_sim/gen_model.py`

The station's page gets the same `info` (the same keys and values: `rover.pivot_z` is `ROVER.rocker.pivot_z` and so on). The judge's footprint and the proving ground's track, wheelbase, garden text and rocker limit come from `rover_config.ROVER`; `proving_ground.py` keeps the name `ROVER_PARAMS`, because `ROVER` there is the rover's start pose.

- [ ] **Step 1: Apply the change and remove gen_model.py**

Save this patch as `$T/t13.patch`:

```diff
diff --git a/rover_sim/bridge.yaml b/rover_sim/bridge.yaml
index 839a3b0..39c8c16 100644
--- a/rover_sim/bridge.yaml
+++ b/rover_sim/bridge.yaml
@@ -1,3 +1,3 @@
 # ros_gz_bridge config for the rover (pixi run sim-bridge).
-# gz topics are set in sim/gen_model.py.
+# gz topics are set in rover_sim/config/sim.yaml (topics).
 - ros_topic_name: /cmd_vel
@@ -37,3 +37,3 @@
   direction: GZ_TO_ROS
-# Physical drivetrain only (DriveParams.mode "physical"): per-wheel setpoint, speed, current, torque, voltage,
+# Physical drivetrain only (sim.yaml drivetrain mode physical): per-wheel setpoint, speed, current, torque, voltage,
 # saturation, slip, load, ground and dig factor as JSON, 50 Hz.
@@ -44,3 +44,3 @@
   direction: GZ_TO_ROS
-# Sensors for the URC missions (sim/gen_model.py).
+# Sensors for the URC missions (rover_sim/urdf/rover_sim.urdf.xacro).
 - ros_topic_name: /gps/fix
@@ -112,3 +112,3 @@
   direction: GZ_TO_ROS
-# Camera pan-tilt head (sim/gen_model.py): target angles [rad], pan + left, tilt + down.
+# Camera pan-tilt head (rover_description's rover.urdf.xacro): target angles [rad], pan + left, tilt + down.
 - ros_topic_name: /camera/pan
diff --git a/rover_sim/plugins/rover_drivetrain.cpp b/rover_sim/plugins/rover_drivetrain.cpp
index ca197fd..725eaee 100644
--- a/rover_sim/plugins/rover_drivetrain.cpp
+++ b/rover_sim/plugins/rover_drivetrain.cpp
@@ -17,3 +17,3 @@
 // - the dust emitters behind the rear wheels, if the model has them (<dust>), get a rate from speed, slip and
-//   the ground (D15). gen_model.py writes them only with DriveParams.dust, off by default (user decision
+//   the ground (D15). The model has them only with sim.yaml's dust on, off by default (user decision
 //   2026-10-07: depth and point clouds see any visible particle); without <dust> nothing is advertised or sent.
@@ -21,5 +21,5 @@
 //   state as JSON (spec 9.3).
-// - the dig-in made visible (<dig_sink>, DriveParams.dig_sink): each wheel's visuals, never its collision, drawn
+// - the dig-in made visible (<dig_sink>, sim.yaml dig_sink): each wheel's visuals, never its collision, drawn
 //   (D - 1) x the ground's static sinkage lower (ShowSink). Physics never reads a visual's pose.
-// - the ruts and pits behind the wheels, if the plugin has <tracks> (gen_model.DriveParams.ruts, on by default):
+// - the ruts and pits behind the wheels, if the plugin has <tracks> (sim.yaml ruts, on by default):
 //   rover_tracks.hh and rover_tracks_render.hh, fed at the end of the step from each wheel's state; visual only,
@@ -31,3 +31,3 @@
 // object surface. Without a ground map the default and object surfaces come from this plugin's <surface>
-// rows (gen_model.py writes them from the catalogue, sim/urc/terrains.py).
+// rows (tools/gen_rover.py writes them from the catalogue, sim/urc/terrains.py, into config/surfaces.yaml).
 //
@@ -48,3 +48,3 @@
 //
-// SDF: see gen_model.py (_add_drivetrain), which writes every element; units SI.
+// SDF: see urdf/rover_sim.urdf.xacro, which writes every element from rover.yaml and config/sim.yaml; units SI.
 
@@ -289,3 +289,3 @@ class RoverDrivetrain : public gz::sim::System,
     }
-    // The emitters to drive, if any (none by default: DriveParams.dust); a wheel without one raises no dust.
+    // The emitters to drive, if any (none by default: sim.yaml dust); a wheel without one raises no dust.
     for (auto e = sdf->FindElement("dust"); e; e = e->GetNextElement("dust")) {
diff --git a/rover_sim/plugins/rover_drivetrain.hh b/rover_sim/plugins/rover_drivetrain.hh
index 2e86ba6..030537d 100644
--- a/rover_sim/plugins/rover_drivetrain.hh
+++ b/rover_sim/plugins/rover_drivetrain.hh
@@ -35,3 +35,3 @@ struct MotorParams {
   double kp = 4.0;                ///< [V/(rad/s)] (M: prototype tuning)
-  double ki = 160.0;              ///< [V/rad] integral time 25 ms (A: retuned, gen_model.DriveParams.ki)
+  double ki = 160.0;              ///< [V/rad] integral time 25 ms (A: retuned, sim.yaml drivetrain ki)
   double speed_filter = 0.005;    ///< [s] time constant of the measured speed (A)
@@ -230,3 +230,3 @@ inline double DustRate(const DustParams& p, const Traction& ground, double hub_s
 
-// --- The dig-in made visible: the tyre sink (gen_model.DriveParams.dig_sink) -------------------------------------
+// --- The dig-in made visible: the tyre sink (sim.yaml dig_sink) --------------------------------------------------
 
diff --git a/rover_sim/plugins/rover_tracks.hh b/rover_sim/plugins/rover_tracks.hh
index 8e6bcb2..48e449d 100644
--- a/rover_sim/plugins/rover_tracks.hh
+++ b/rover_sim/plugins/rover_tracks.hh
@@ -67,3 +67,3 @@ constexpr Stamp kNever = std::numeric_limits<Stamp>::max();
 
-/// The <tracks> group of the drivetrain plugin (gen_model.TrackParams writes it; units SI).
+/// The <tracks> group of the drivetrain plugin (urdf/rover_sim.urdf.xacro writes it from sim.yaml ruts; units SI).
 struct Params {
diff --git a/rover_sim/plugins/tests/test_rover_drivetrain.cpp b/rover_sim/plugins/tests/test_rover_drivetrain.cpp
index 2ce7c49..6d5c4eb 100644
--- a/rover_sim/plugins/tests/test_rover_drivetrain.cpp
+++ b/rover_sim/plugins/tests/test_rover_drivetrain.cpp
@@ -14,3 +14,3 @@ namespace {
 constexpr double kDt = 0.001;              // [s] the worlds' physics step
-constexpr double kWheelInertia = 0.028125;  // [kg m2] gen_model.wheel_inertia(2.5, 0.15, 0.1), about the axle
+constexpr double kWheelInertia = 0.028125;  // [kg m2] rover.yaml's wheel (2.5 kg, r 0.15 m) about the axle
 
diff --git a/rover_sim/referee.py b/rover_sim/referee.py
index 0590006..ecd3ee2 100644
--- a/rover_sim/referee.py
+++ b/rover_sim/referee.py
@@ -26,3 +26,3 @@ SIM_DIR = Path(__file__).resolve().parent
 sys.path.insert(0, str(SIM_DIR))
-import gen_model  # noqa: E402
+import rover_config  # noqa: E402
 from urc import judge as J  # noqa: E402
@@ -57,4 +57,4 @@ class Referee:
         self.poses, self.joints, self.presses = {}, {}, []
-        self.node.subscribe(Odometry, gen_model.GROUND_TRUTH_TOPIC, self._on_odometry)
-        self.node.subscribe(StringMsg, gen_model.LED_TOPIC, self._on_led)
+        self.node.subscribe(Odometry, rover_config.GROUND_TRUTH_TOPIC, self._on_odometry)
+        self.node.subscribe(StringMsg, rover_config.LED_TOPIC, self._on_led)
         self.node.subscribe(StringMsg, "/model/lander/presses", self._on_press)
@@ -142,3 +142,3 @@ class Referee:
         msg = self.msgs["MaterialColor"]()
-        msg.entity.name = f"rover::base_link::{gen_model.LED_VISUAL}"
+        msg.entity.name = f"rover::base_link::{rover_config.LED_VISUAL}"
         r, g, b = LED_COLORS[shown]
diff --git a/rover_sim/station/drive.py b/rover_sim/station/drive.py
index 4778512..9ede196 100644
--- a/rover_sim/station/drive.py
+++ b/rover_sim/station/drive.py
@@ -9,6 +9,5 @@ import numpy as np
 
-import gen_model
 import viewers
+from rover_config import ROVER
 
-ROVER = gen_model.Params()
 FLY = viewers.FlyParams()
@@ -17,3 +16,3 @@ PRESETS = {1: (0.25, 0.4), 2: (0.6, 0.8), 3: (1.2, 1.2), 4: (2.0, 1.2), 5: (3.0,
 FAST = 2.0  # Shift multiplies the preset, up to what the rover can do:
-MAX_LINEAR = ROVER.wheel_speed * ROVER.wheel_radius  # [m/s] wheel joint velocity limit
+MAX_LINEAR = ROVER.wheel.top_speed * ROVER.wheel.radius  # [m/s] the top speed
 MAX_ANGULAR = 1.5  # [rad/s] spinning faster than this is not drivable from a camera
@@ -116,7 +115,7 @@ class Look:
     def center(self):
-        self.pan, self.tilt = 0.0, self.rover.camera_pitch
+        self.pan, self.tilt = 0.0, self.rover.head.home_tilt
 
     def nudge(self, pan, tilt):
-        limit = self.rover.camera_pan_limit
-        lower, upper = self.rover.camera_tilt_limits
+        limit = self.rover.head.pan_limit
+        lower, upper = self.rover.head.tilt_limits
         self.pan = max(-limit, min(limit, self.pan + pan))
diff --git a/rover_sim/station/link.py b/rover_sim/station/link.py
index d25be0f..ca155eb 100644
--- a/rover_sim/station/link.py
+++ b/rover_sim/station/link.py
@@ -16,3 +16,3 @@ import time
 
-import gen_model
+import rover_config
 import viewers
@@ -34,8 +34,8 @@ from .video import Feed
 
-ROVER = gen_model.Params()
+ROVER = rover_config.ROVER
 CAMERAS = {  # page name -> image topic
-    "eye": gen_model.EYE_IMAGE_TOPIC,
-    "chase": gen_model.CHASE_IMAGE_TOPIC,
-    "rgb": gen_model.CAMERA_TOPIC + "/image",
-    "depth": gen_model.CAMERA_TOPIC + "/depth_image",
+    "eye": viewers.EYE_IMAGE_TOPIC,
+    "chase": viewers.CHASE_IMAGE_TOPIC,
+    "rgb": rover_config.CAMERA_TOPIC + "/image",
+    "depth": rover_config.CAMERA_TOPIC + "/depth_image",
     "fly": viewers.FLY_IMAGE_TOPIC,
@@ -44,4 +44,4 @@ CAMERAS = {  # page name -> image topic
 # topic their ChaseCamera plugin publishes at 5 Hz.
-VIEWERS = ((gen_model.EYE_MODEL, "eye", gen_model.EYE_STATE_TOPIC),
-           (gen_model.CHASE_MODEL, "chase", gen_model.CHASE_STATE_TOPIC))
+VIEWERS = ((viewers.EYE_MODEL, "eye", viewers.EYE_STATE_TOPIC),
+           (viewers.CHASE_MODEL, "chase", viewers.CHASE_STATE_TOPIC))
 # The fly camera (FlyCamera plugin, state at 10 Hz of sim time) is spawned
@@ -114,9 +114,9 @@ class GzLink:
         sub = self.node.subscribe
-        sub(Odometry, gen_model.GROUND_TRUTH_TOPIC, self._on_odometry)
+        sub(Odometry, rover_config.GROUND_TRUTH_TOPIC, self._on_odometry)
         throttled = SubscribeOptions()
         throttled.msgs_per_sec = JOINT_RATE
-        sub(Model, gen_model.JOINT_STATE_TOPIC, self._on_joints, throttled)
-        sub(NavSat, gen_model.NAVSAT_TOPIC, self._on_navsat)
+        sub(Model, rover_config.JOINT_STATE_TOPIC, self._on_joints, throttled)
+        sub(NavSat, rover_config.NAVSAT_TOPIC, self._on_navsat)
         sub(WorldStatistics, f"/world/{world}/stats", self._on_stats)
-        sub(StringMsg, gen_model.LED_TOPIC, lambda m: self._put("led", m.data.strip().lower()))
+        sub(StringMsg, rover_config.LED_TOPIC, lambda m: self._put("led", m.data.strip().lower()))
         for _, key, topic in (*VIEWERS, FLY_VIEWER):
@@ -125,3 +125,3 @@ class GzLink:
         drivetrain.msgs_per_sec = DRIVETRAIN_RATE
-        sub(StringMsg, gen_model.DRIVETRAIN_TOPIC, lambda m: self._put("drivetrain", json.loads(m.data)), drivetrain)
+        sub(StringMsg, rover_config.DRIVETRAIN_TOPIC, lambda m: self._put("drivetrain", json.loads(m.data)), drivetrain)
         sub(StringMsg, STATION_TOPIC, self._on_station)
@@ -131,9 +131,9 @@ class GzLink:
         # subscribers have connected.
-        self._twist = self.node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
-        self._led = self.node.advertise(gen_model.LED_TOPIC, StringMsg)
-        self._chase_cmd = self.node.advertise(gen_model.CHASE_CMD_TOPIC, Vector3d)
-        self._chase_mode = self.node.advertise(gen_model.CHASE_MODE_TOPIC, StringMsg)
-        self._eye = self.node.advertise(gen_model.EYE_LOOK_TOPIC, Vector3d)
-        self._head = {joint: self.node.advertise(gen_model.HEAD_TOPIC.format(joint=joint), Double)
-                      for joint in (gen_model.PAN_JOINT, gen_model.TILT_JOINT)}
+        self._twist = self.node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
+        self._led = self.node.advertise(rover_config.LED_TOPIC, StringMsg)
+        self._chase_cmd = self.node.advertise(viewers.CHASE_CMD_TOPIC, Vector3d)
+        self._chase_mode = self.node.advertise(viewers.CHASE_MODE_TOPIC, StringMsg)
+        self._eye = self.node.advertise(viewers.EYE_LOOK_TOPIC, Vector3d)
+        self._head = {joint: self.node.advertise(rover_config.HEAD_TOPIC.format(joint=joint), Double)
+                      for joint in (rover_config.PAN_JOINT, rover_config.TILT_JOINT)}
         self._fly_cmd = self.node.advertise(viewers.FLY_CMD_TOPIC, Twist)
@@ -143,3 +143,3 @@ class GzLink:
         self._fly_mode = self.node.advertise(viewers.FLY_MODE_TOPIC, StringMsg)
-        self.feeds = {name: Feed(name, topic, self._watch, self._unwatch, ROVER.camera_clip)
+        self.feeds = {name: Feed(name, topic, self._watch, self._unwatch, ROVER.camera.clip)
                       for name, topic in CAMERAS.items()}
@@ -278,3 +278,3 @@ class GzLink:
     def head(self, pan, tilt):
-        for joint, angle in ((gen_model.PAN_JOINT, pan), (gen_model.TILT_JOINT, tilt)):
+        for joint, angle in ((rover_config.PAN_JOINT, pan), (rover_config.TILT_JOINT, tilt)):
             msg = Double()
diff --git a/rover_sim/station/server.py b/rover_sim/station/server.py
index b055b60..66486d1 100644
--- a/rover_sim/station/server.py
+++ b/rover_sim/station/server.py
@@ -20,3 +20,3 @@ from aiohttp import WSMsgType, web
 
-import gen_model
+import rover_config
 import viewers
@@ -264,7 +264,7 @@ class Station:
             rockers = {"left": left, "right": right, "error": left + right}
-        wheels = {w: joints[f"wheel_{w}_joint"][1] * drive.ROVER.wheel_radius
+        wheels = {w: joints[f"wheel_{w}_joint"][1] * drive.ROVER.wheel.radius
                   for w in ("fl", "fr", "rl", "rr") if f"wheel_{w}_joint" in joints} or None
         head = None
-        if gen_model.PAN_JOINT in joints and gen_model.TILT_JOINT in joints:
-            head = {"pan": joints[gen_model.PAN_JOINT][0], "tilt": joints[gen_model.TILT_JOINT][0]}
+        if rover_config.PAN_JOINT in joints and rover_config.TILT_JOINT in joints:
+            head = {"pan": joints[rover_config.PAN_JOINT][0], "tilt": joints[rover_config.TILT_JOINT][0]}
         return {
@@ -294,9 +294,9 @@ class Station:
             "map": minimap.map_info(self.sheet),
-            "rover": {"pivot_z": r.pivot_z, "wheel_dx": r.wheel_dx, "wheel_dz": r.wheel_dz,
-                      "wheel_radius": r.wheel_radius, "chassis_size": r.chassis_size, "chassis_z": r.chassis_z,
-                      "rocker_limit": r.rocker_limit},
-            "hfov": {"eye": gen_model.EyeParams().hfov, "chase": gen_model.ChaseParams().hfov,
-                     "rgb": r.camera_hfov, "depth": r.camera_hfov, "fly": FLY.hfov},
+            "rover": {"pivot_z": r.rocker.pivot_z, "wheel_dx": r.wheel.dx, "wheel_dz": r.wheel.dz,
+                      "wheel_radius": r.wheel.radius, "chassis_size": r.chassis.size, "chassis_z": r.chassis.z,
+                      "rocker_limit": r.rocker.limit},
+            "hfov": {"eye": viewers.EyeParams().hfov, "chase": viewers.ChaseParams().hfov,
+                     "rgb": r.camera.hfov, "depth": r.camera.hfov, "fly": FLY.hfov},
             "fly": {"aspect": FLY.size[0] / FLY.size[1], "fast": FLY.fast, "speed_scales": FLY.speed_scales},
-            "drivetrain": {"current_limit": r.drive.current_limit},
+            "drivetrain": {"current_limit": r.motor.current_limit},
             "presets": drive.PRESETS, "max": [drive.MAX_LINEAR, drive.MAX_ANGULAR], "deadman": drive.DEADMAN,
diff --git a/rover_sim/station/static/station.js b/rover_sim/station/static/station.js
index bf2809c..40b56d4 100644
--- a/rover_sim/station/static/station.js
+++ b/rover_sim/station/static/station.js
@@ -645,3 +645,3 @@ function showDrivetrain(m) {
   $("drivetrain-note").textContent = d ? ""
-    : "No motor data: this rover drives with Gazebo's DiffDrive (DriveParams.mode = \"physical\" has the motor model).";
+    : "No motor data: this rover drives with Gazebo's DiffDrive (sim.yaml's drivetrain mode physical has the motor model).";
   if (!d) return;
diff --git a/rover_sim/urc/judge.py b/rover_sim/urc/judge.py
index 1715522..8f3a2f0 100644
--- a/rover_sim/urc/judge.py
+++ b/rover_sim/urc/judge.py
@@ -18,3 +18,3 @@ from dataclasses import dataclass, field
 
-import gen_model  # the rover's geometry; sim/ is on sys.path wherever urc is used
+from rover_config import ROVER as _ROVER  # the rover's geometry; sim/ is on sys.path wherever urc is used
 
@@ -22,5 +22,4 @@ from . import lander, props, rules, sheet as sheets
 
-_ROVER = gen_model.Params()
-ROVER_HALF_LENGTH = _ROVER.wheel_dx + _ROVER.wheel_radius  # [m] wheel centres plus the wheel radius
-ROVER_HALF_WIDTH = _ROVER.pivot_y + _ROVER.wheel_width / 2
+ROVER_HALF_LENGTH = _ROVER.wheel.dx + _ROVER.wheel.radius  # [m] wheel centres plus the wheel radius
+ROVER_HALF_WIDTH = _ROVER.rocker.pivot_y + _ROVER.wheel.width / 2
 STOPPED_SPEED = 0.05  # [m/s]
diff --git a/rover_sim/urc/missions/proving_ground.py b/rover_sim/urc/missions/proving_ground.py
index f998dfc..11f2ef2 100644
--- a/rover_sim/urc/missions/proving_ground.py
+++ b/rover_sim/urc/missions/proving_ground.py
@@ -35,3 +35,3 @@ import math
 
-import gen_model  # the rover's geometry; sim/ is on sys.path wherever urc is used
+import rover_config  # the rover's geometry; sim/ is on sys.path wherever urc is used
 
@@ -49,5 +49,5 @@ ROVER = (0.0, 0.0, math.pi / 2)
 GATE = (0.0, 4.0)
-ROVER_PARAMS = gen_model.Params()
-ROVER_TRACK = 2 * ROVER_PARAMS.pivot_y  # [m] between the wheel planes
-ROVER_WHEELBASE = 2 * ROVER_PARAMS.wheel_dx  # [m]
+ROVER_PARAMS = rover_config.ROVER  # rover.yaml (ROVER above is the rover's start)
+ROVER_TRACK = 2 * ROVER_PARAMS.rocker.pivot_y  # [m] between the wheel planes
+ROVER_WHEELBASE = 2 * ROVER_PARAMS.wheel.dx  # [m]
 WEST, EAST = math.pi, 0.0
@@ -73,3 +73,3 @@ BUMPS = features.AlternatingBumps("bumps", (8.0, 62.0), EAST, (0.1, 0.2, 0.3), 3
 TWIST_DITCH = features.TwistDitch("twist_ditch", 32.0, 62.0, EAST, math.atan2(ROVER_TRACK, ROVER_WHEELBASE),
-                                  round(ROVER_WHEELBASE * math.tan(ROVER_PARAMS.rocker_limit), 2))
+                                  round(ROVER_WHEELBASE * math.tan(ROVER_PARAMS.rocker.limit), 2))
 WASHBOARD = features.Washboard("washboard", (8.0, 74.0), EAST, 20.0, 4.0)
@@ -154,3 +154,3 @@ def build(models_dir, worlds_dir, media):
                   f"A {length:.0f} x {width:.0f} m plot of {size * 100:.0f} cm rocks (+-20 %), one per "
-                  f"{garden_spacing(size):.1f} m cell; the wheel radius is {ROVER_PARAMS.wheel_radius * 100:.0f} cm.",
+                  f"{garden_spacing(size):.1f} m cell; the wheel radius is {ROVER_PARAMS.wheel.radius * 100:.0f} cm.",
                   ["ROCK GARDEN", f"{size * 100:.0f} CM"],
@@ -161,3 +161,3 @@ def build(models_dir, worlds_dir, media):
               f"{TWIST_DITCH.depth * 100:.0f} cm diagonal ditch that takes a front wheel and the opposite rear wheel "
-              f"together, turning the rockers to their {ROVER_PARAMS.rocker_limit} rad limit.",
+              f"together, turning the rockers to their {ROVER_PARAMS.rocker.limit} rad limit.",
               ["ARTICULATION", f"BUMPS {heights}", "TWIST DITCH"], (east_sign, BUMPS.start[1] + 3.5), WEST,
diff --git a/rover_sim/worlds/rover_test.sdf b/rover_sim/worlds/rover_test.sdf
index 23ef27a..3388369 100644
--- a/rover_sim/worlds/rover_test.sdf
+++ b/rover_sim/worlds/rover_test.sdf
@@ -23,4 +23,4 @@
     <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
-    <!-- Takes the physical drivetrain's dust rates (gen_model.DUST_TOPIC) when the rover has dust emitters
-         (gen_model.DriveParams.dust, off by default since 2026-10-07); idle otherwise. -->
+    <!-- Takes the physical drivetrain's dust rates (rover_config.DUST_TOPIC) when the rover has dust emitters
+         (rover_sim/config/sim.yaml dust, off by default since 2026-10-07); idle otherwise. -->
     <plugin filename="gz-sim-particle-emitter-system" name="gz::sim::systems::ParticleEmitter"/>
```

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src && git apply "$T/t13.patch" && git rm -q rover_sim/gen_model.py && git add -A \
  && git grep -n "gen_model" -- . ':!docs' ':!rover_sim/tests/fixtures' | cut -c1-110
```

Expected, only these: `rover_sim/README.md:24` (Task 16 rewrites it), the nine history lines of Task 12 Step 1, and the comments Decision 10 leaves: `rover_sim/urc/sdf.py:2`, `rover_sim/urc/terrains.py:17`, `rover_sim/urc/terrains.py:123`, `rover_sim/urc/textures.py:412`, `rover_sim/urc/world.py:70`.

- [ ] **Step 2: Every consumer imports**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim && /Users/alarion239/.pixi/bin/pixi run python -c "
import sys; sys.path[:0] = ['.', 'tools']
import referee, viewers, gen_rover, rover_config
from station import drive, link, server
from urc import judge
from urc.missions import proving_ground
print(drive.MAX_LINEAR, judge.ROVER_HALF_LENGTH, judge.ROVER_HALF_WIDTH, proving_ground.ROVER_TRACK, proving_ground.ROVER_WHEELBASE)"
```

Expected: `3.0 0.6 0.45 0.8 0.9` (gen_model.py's numbers: 20 rad/s x 0.15 m; 0.45 + 0.15; 0.4 + 0.05; 2 x 0.4; 2 x 0.45).

- [ ] **Step 3: Rebuild (the plugins' comments changed) and run the colcon tests**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run colcon-test 2>&1 | grep -E "tests passed|^Summary"
```

Expected, the packages in any order: `100% tests passed, 0 tests failed out of 5` (rover_description), `out of 57` (rover_control), `out of 3` (rover_sim), and `Summary: 3 packages finished` for the build and for the tests.

- [ ] **Step 4: Run the station, judge and mission tests (several minutes)**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && GZ_IP=127.0.0.1 /Users/alarion239/.pixi/bin/pixi run python -m unittest test_station test_urc_judge test_urc_missions 2>&1 | grep -E "^(FAIL|ERROR):|^Ran|^FAILED|^OK"
```

Expected: `Ran 126 tests`, `OK` (75 + 24 + 27).

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add -A && git commit -q -F - <<'EOF'
PR 2: rover_sim: the station, referee, judge and proving ground read rover_config; gen_model.py removed

The driver station (its speed limit, the head's limits, the page's rover geometry and current limit), the referee
(topics, the LED's visual), the judge (the rover's footprint) and the proving ground (track, wheelbase, wheel radius,
rocker limit) read rover.yaml and sim.yaml through rover_config, the same numbers. Comments that named gen_model.py in
the plugins, bridge.yaml, rover_test.sdf and one station message now name the YAML files and the xacro. The comments
in urc/sdf.py, terrains.py, textures.py and world.py stay: changing those modules renames the media every world
references (urc/media.py), and the worlds regenerate byte for byte.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 14: The worlds and mission sheets regenerate byte for byte

**Files:** none (verification).

- [ ] **Step 1: Regenerate and compare (about a minute)**

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src && GZ_IP=127.0.0.1 GZ_PARTITION=pr2_worlds_$$ /Users/alarion239/.pixi/bin/pixi run sim-worlds > "$T/worlds.log" 2>&1; echo "exit $?"; grep -c "^wrote rover_sim/worlds/" "$T/worlds.log"
"$T/worlds_sha.sh" /Users/alarion239/Desktop/hurc_ws/src > "$T/worlds_after.sha"; diff "$T/worlds_before.sha" "$T/worlds_after.sha" && echo "worlds, sheets and their models byte for byte"
```

Expected: `exit 0`, `5` (pixi reruns the generator: its inputs changed), `worlds, sheets and their models byte for byte`. A difference is a regression: the file names of a world's media come from the CRC of the `urc` modules that made them (Decision 10), so look first for a changed `urc/` module (`git diff 9208047 --stat -- rover_sim/urc`), then for a rover number the proving ground reads.

### Task 15: The full suite and the colcon tests

**Files:** none (verification). A failure is fixed in the task that caused it, as a new commit.

- [ ] **Step 1: List the tests**

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && GZ_IP=127.0.0.1 GZ_PARTITION=pr2_list_$$ \
  /Users/alarion239/.pixi/bin/pixi run python "$T/list_tests.py" 2>/dev/null | sort > "$T/tests_after.txt"
wc -l < "$T/tests_after.txt"
diff <(sed 's/^test_gen_model\./test_rover_model./' "$T/tests_before.txt" | sort) "$T/tests_after.txt"
```

Expected: `546`, and this diff (the dry run's):
```
75a76,86
> test_equivalence.Drives.test_bit_for_bit
> test_equivalence.Equivalence.test_the_generated_rovers_are_the_frozen_ones
> test_equivalence.Equivalence.test_the_rover_is_the_tracked_model
> test_equivalence.SdfEquivalence.test_a_default_written_out_is_the_same
> test_equivalence.SdfEquivalence.test_a_missing_plugin_parameter_differs
> test_equivalence.SdfEquivalence.test_a_model_equals_itself
> test_equivalence.SdfEquivalence.test_a_number_beyond_the_tolerance_differs
> test_equivalence.SdfEquivalence.test_a_pose_compares_its_rotation_not_its_angles
> test_equivalence.SdfEquivalence.test_an_element_only_one_side_has_differs
> test_equivalence.SdfEquivalence.test_rounding_within_the_tolerance_does_not
> test_equivalence.SdfEquivalence.test_the_plugins_order_counts
139c150
< test_foundations.Viewers.test_gen_model_finds_the_viewer_names
---
> test_foundations.Viewers.test_the_viewer_names_live_in_viewers
206a218,225
> test_rover_config.Files.test_numbers_keep_their_type
> test_rover_config.Files.test_the_files_share_no_section
> test_rover_config.Files.test_topics_are_the_gazebo_interface
> test_rover_config.Files.test_trees_read_like_the_files
> test_rover_config.Variants.test_a_misspelt_name_is_refused
> test_rover_config.Variants.test_nested_sections_merge_too
> test_rover_config.Variants.test_replace_merges_into_the_sections
> test_rover_config.Variants.test_variants_are_values
247a267,268
> test_rover_model.TrackedModel.test_the_surfaces_are_the_catalogues
> test_rover_model.TrackedModel.test_the_tread_mesh_is_the_descriptions
```

(The 44 model tests keep their ids but for the module, which the `sed` maps; the line numbers are those of a branch with the Autonomy world, as here.)

- [ ] **Step 2: Run `pixi run test` in the background (about 15 minutes)**

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src && GZ_IP=127.0.0.1 GZ_PARTITION=pr2_test_$$ /Users/alarion239/.pixi/bin/pixi run test > "$T/test.log" 2>&1; echo "exit $?" >> "$T/test.log"
```

Run it with `run_in_background` and wait. Then:

```bash
T=/private/tmp/claude-502/rover-pr2
grep -E "tests passed|^Summary" "$T/test.log"; grep -E "^Ran |^OK|^FAILED|^exit" "$T/test.log"
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain && echo "tree clean"
```

Expected (the build's summary first, then the tests', the packages in any order):
```
Summary: 3 packages finished [...]
100% tests passed, 0 tests failed out of 5
100% tests passed, 0 tests failed out of 57
100% tests passed, 0 tests failed out of 3
Summary: 3 packages finished [...]
Ran 546 tests in ...s
OK (skipped=9, expected failures=2)
exit 0
tree clean
```

PR 1's 525 tests with its 9 skips and 2 expected failures, plus the 21 new ones (the dry run on a copy of this branch: 546 tests, OK, with one more skip there because it had no maps). A test that passed in Task 1 (its log is `$T/baseline.log`) and fails here is a regression of this PR: run it alone, find the cause with superpowers:systematic-debugging (a variant built from the wrong section, a number named through the wrong key), fix it in a new commit in the task's spirit and rerun the suite. `tree clean` shows that the suite's own `sim-model` wrote the tracked models with the same bytes.

### Task 16: Documents

**Files:**
- Modify: `rover_description/README.md`, `README.md`, `rover_sim/README.md`, `docs/sim/manual.md`, `docs/sim/design-notes.md`, `docs/sim/gazebo-lessons.md`

- [ ] **Step 1: rover_description's README**

Replace `rover_description/README.md` with (its first two lines are the team's):

````markdown
# Rover Description
This package primarily contains the URDF files that define the rover's hardware in a way that other ROS packages can use.

Every physical number of the rover is in one file, `config/rover.yaml`; the description, the simulation (`rover_sim`) and RViz read it. The numbers are placeholders until the mechanical team has real ones: the geometry is the swerve design's (`rover_control`'s `config.hpp`, which keeps its own copy until the drive code reads this file), the motors a validated simulation set.

### What is where
| Path | What it is |
|---|---|
| `config/rover.yaml` | Every physical number, with its unit and source ((A) assumed, (M) measured or tuned, (R) from a reference): chassis, rockers, the differential's spring and damper, wheels and the top speed, the motors and drivelines, the tyres, the IMU, GNSS, the camera and its pan-tilt head, the LED, and the look (masts, colours) |
| `urdf/rover.urdf.xacro` | The description built from `rover.yaml`: links with their masses and inertias, joints with limits and damping, collisions and visuals, and the frames `imu`, `gnss`, `camera` and `led` |
| `urdf/inertials.xacro` | The inertias of a box, a wheel and a rocker |
| `urdf/ros2_control.xacro` | The `ros2_control` block: the four wheels take a velocity command and report position, velocity and effort; the rockers and the camera head report position and velocity. Its hardware plugin is the argument `ros2_control_plugin`, by default `rover_control/RoverHardwareStub`, which a later pull request adds; nothing loads the block yet |
| `meshes/wheel.glb`, `meshes/gen_wheel_mesh.py` | The tread tyre (a unit cylinder the description scales to the wheel) and the script that writes it from `rover.yaml`'s colours |
| `launch/view.launch.py`, `rviz/view.rviz` | The rover in RViz |
| `test/test_description.py` | Its tests: `check_urdf`, the joint tree, the numbers, the `ros2_control` block, the mesh, the launch file (`colcon test`, with the team's linters) |

The description names no simulator. `rover_sim/urdf/rover_sim.urdf.xacro` adds Gazebo's sensors and plugins to it, and `rover_sim/config/sim.yaml` holds how the simulation models the rover (contact, dig-in, its motor controller's tuning, the visual cues).

### Change the rover
1. Edit `config/rover.yaml`; keep the unit and say where the number comes from.
2. `pixi run sim-model` builds the simulated rover again (`rover_sim/models/rover/model.sdf`); `pixi run sim`, `drive` and `test` do it for you.
3. `pixi run test`. A changed number fails `rover_sim`'s `test_equivalence`, which holds the rover as it was before this package described it: refreeze its fixtures in the same commit and say why (`rover_sim/tests/fixtures/rover_model/README.md`).

A new frame, joint or shape goes into `urdf/rover.urdf.xacro`, its numbers into `rover.yaml`. Check the result in `pixi shell`: `xacro rover_description/urdf/rover.urdf.xacro > /tmp/rover.urdf && check_urdf /tmp/rover.urdf`. After changing the tread tyre's colours run `python rover_description/meshes/gen_wheel_mesh.py`, then `pixi run sim-model`.

### View it in RViz
From the workspace root, in `pixi shell`:

```bash
colcon build --packages-select rover_description
source install/local_setup.bash        # local_setup.zsh in zsh
ros2 launch rover_description view.launch.py
```

The sliders move the joints. Arguments: `gui:=false` (no sliders), `rviz:=false`, `rover_yaml:=<file>` (another set of numbers). Without a display (Linux without `DISPLAY` or `WAYLAND_DISPLAY`) `gui` and `rviz` default to false and the launch says so. `robot_state_publisher` warns that KDL does not support a root link with an inertia; the warning is harmless, and a massless root link would rename the simulated rover's links.

### Use it from another package
`xacro $(ros2 pkg prefix rover_description)/share/rover_description/urdf/rover.urdf.xacro` expands it; the arguments are `rover_yaml` (default the package's `config/rover.yaml`), `ros2_control` (`true` or `false`) and `ros2_control_plugin`.
````

- [ ] **Step 2: The other documents**

Save this patch as `$T/t16.patch`:

```diff
diff --git a/README.md b/README.md
index 5a1e32e..278f624 100644
--- a/README.md
+++ b/README.md
@@ -31,3 +31,3 @@ The manual is [docs/sim/manual.md](docs/sim/manual.md), with its [design notes](
 ### Tests
-- `pixi run test` runs the C++ tests of `rover_control` and `rover_sim` (`colcon test`) and the simulation's suite: about 15 minutes on an Apple M4. Tests that render skip on a machine without a GPU and say why
+- `pixi run test` runs the colcon tests of `rover_description` (xacro and `check_urdf` on the description, the team's linters), `rover_control` and `rover_sim` (C++) and the simulation's suite: about 15 minutes on an Apple M4. Tests that render skip on a machine without a GPU and say why
 - CI (`.github/workflows/ci.yml`) runs the same tasks on every pull request: on Ubuntu 24.04 and macOS (Apple silicon) `pixi run build`, `pixi run fetch-data` and `pixi run test` (the rendering tests on macOS only), on Windows `pixi run build`. CI has only the rasters fetch-data can fetch, so the Autonomy world's tests skip there until its lidar DEM can be rebuilt
@@ -41,4 +41,4 @@ The manual is [docs/sim/manual.md](docs/sim/manual.md), with its [design notes](
 Most of our work is structured into folders called "packages". Whenever you create a new package, please add a description of it here:
-- `rover_description`: Contains the URDFs that define the rover's structure, joints, actuators, sensors, etc
-- `rover_sim`: Contains SDFs, URDFs, launch configurations, and any other tools that help with simulating the rover in Gazebo. Today it holds the URC simulation: Gazebo plugins for the drivetrain and motors, the rocker differential and the driver station's cameras; the URC 2027 mission worlds and their generator; a browser driver station, a launcher and a referee (macOS and Linux)
+- `rover_description`: Contains the URDFs that define the rover's structure, joints, actuators, sensors, etc. Every physical number of the rover is in `config/rover.yaml` (geometry, masses, joints, the rocker differential, the motors, the sensors and the camera head); `urdf/rover.urdf.xacro` builds the description from it, with the `ros2_control` block in `urdf/ros2_control.xacro`; `meshes/` has the tread tyre and `launch/view.launch.py` shows the rover in RViz. Its README says how to change the rover
+- `rover_sim`: Contains SDFs, URDFs, launch configurations, and any other tools that help with simulating the rover in Gazebo. Today it holds the URC simulation: Gazebo plugins for the drivetrain and motors, the rocker differential and the driver station's cameras; the rover's Gazebo model, built from `rover_description` and `config/sim.yaml` (how the simulation models the rover) with `urdf/rover_sim.urdf.xacro`; the URC 2027 mission worlds and their generator; a browser driver station, a launcher and a referee (macOS and Linux)
 - `rover_control`: The rover's drive code. For now the swerve design's kinematics library `rover_driver` with its tests and tools, superseded by tank drive; the ros2_control work replaces it
diff --git a/docs/sim/design-notes.md b/docs/sim/design-notes.md
index ef9521b..3101b09 100644
--- a/docs/sim/design-notes.md
+++ b/docs/sim/design-notes.md
@@ -53,6 +53,6 @@ every wheel contact grips. Every 1 ms step:
   (`gz::math::SpeedLimiter`, 8 rad/s², 1.2 m/s² (A) until the driver team
-  reports its controller's ramp) and clamped to `Params.wheel_speed`.
+  reports its controller's ramp) and clamped to the top speed (`rover.yaml` `wheel: top_speed`).
   `track_multiplier` is 1.0: the raw skid-steer response, which the team's
   driver compensates (the user's choice; Clearpath ships 1.875 for Husky).
-- **Motor** per wheel (`rover_drivetrain.hh`, `DriveParams`): a DC motor
+- **Motor** per wheel (`rover_drivetrain.hh`; `rover.yaml` `motor`, `driveline`): a DC motor
   (48 V, 0.46 Ω, kt = ke 0.0445; the Husky A200 motor at twice its 24 V, for 3 m/s), gearbox 50 at efficiency 0.8,
@@ -85,3 +85,3 @@ every wheel contact grips. Every 1 ms step:
   0.05), else the object surface (`manmade`). A world without a ground map
-  uses the rows gen_model writes into the plugin from the catalogue.
+  uses the rows `tools/gen_rover.py` writes from the catalogue (`config/surfaces.yaml`).
 - **Loads** come from each wheel joint's transmitted wrench (gate G8: within
@@ -104,5 +104,5 @@ every wheel contact grips. Every 1 ms step:
   further than the static carve; the cues of Visible dig-in draw it.
-- **Visible dig-in**: the tyre sink (`DriveParams.dig_sink`) is drawn from
+- **Visible dig-in**: the tyre sink (`sim.yaml` `dig_sink`) is drawn from
   each wheel's D at the end of its step, and the ruts and pits
-  (`DriveParams.ruts`) are fed from it; both read the drivetrain's state and
+  (`sim.yaml` `ruts`) are fed from it; both read the drivetrain's state and
   change none of it (Visible dig-in, below).
@@ -110,4 +110,4 @@ every wheel contact grips. Every 1 ms step:
   8.2.2's depth camera sees any visible particle (Gazebo lessons), and clean
-  depth images and point clouds come first. `DriveParams(dust=True)` (or
-  `dust: bool = True` in `gen_model.py`, then `pixi run sim-model`) puts an
+  depth images and point clouds come first. `dust: enabled: true` in
+  `rover_sim/config/sim.yaml` (then `pixi run sim-model`) puts an
   emitter behind each rear wheel on the rocker links and gives the drivetrain
@@ -127,3 +127,3 @@ every wheel contact grips. Every 1 ms step:
   zero, so a commander that dies cannot leave the rover driving
-  (`DriveParams.cmd_timeout`; 0 holds the last command; tests use the sim
+  (`sim.yaml` drivetrain `cmd_timeout`; 0 holds the last command; tests use the sim
   clock).
@@ -182,3 +182,3 @@ deeper static carve changes the contacts. When the user found that "dig is
 not really visible", they chose three cues that only draw it (2026-10-07),
-each behind its own switch in `gen_model.py`, all on:
+each behind its own switch in `rover_sim/config/sim.yaml`, all on:
 
@@ -186,5 +186,5 @@ each behind its own switch in `gen_model.py`, all on:
 |---|---|---|---|
-| Tyre sink | `DriveParams.dig_sink` (`dig_sink_gain` 1.0, `dig_sink_max` 0.06 m, `dig_sink_tau` 0.1 s): the drivetrain's `<dig_sink>` | each tyre (D − 1) × s below its SDF pose, at true scale | every camera: the station's, the rover's RGB-D and the Gazebo GUI (through `/world/<w>/state`) |
-| Ruts and pits | `DriveParams.ruts` (numbers in `TrackParams`): the drivetrain's `<tracks>` | on every ground with s > 0, a faint floor behind each wheel (low berms too where s ≥ 1.5 cm or the wheel digs in); darker, with higher berms and pits, where the wheels dig in | camera sensors only: the station's views and the rover's RGB-D, not `gz sim -g` |
-| Tread tyres | `Params.tread_tyre` (`models/rover/meshes/wheel.glb`) | tread bars, a hub and spokes, one of them ochre, so a wheel spinning in place shows | every view of the wheels |
+| Tyre sink | `dig_sink` (`gain` 1.0, `max` 0.06 m, `tau` 0.1 s): the drivetrain's `<dig_sink>` | each tyre (D − 1) × s below its SDF pose, at true scale | every camera: the station's, the rover's RGB-D and the Gazebo GUI (through `/world/<w>/state`) |
+| Ruts and pits | `ruts` (numbers under `tracks`): the drivetrain's `<tracks>` | on every ground with s > 0, a faint floor behind each wheel (low berms too where s ≥ 1.5 cm or the wheel digs in); darker, with higher berms and pits, where the wheels dig in | camera sensors only: the station's views and the rover's RGB-D, not `gz sim -g` |
+| Tread tyres | `tread_tyre` (`rover_description/meshes/wheel.glb`) | tread bars, a hub and spokes, one of them ochre, so a wheel spinning in place shows | every view of the wheels |
 
@@ -252,4 +252,4 @@ its marker service).
   reads no map and draws nothing.
-- **Tread tyres**: the tyres' visual is `meshes/wheel.glb`
-  (`gen_model.wheel_parts`, the cylinder's outline, 384 triangles, no SDF
+- **Tread tyres**: the tyres' visual is `rover_description/meshes/wheel.glb`
+  (`gen_wheel_mesh.py` beside it, the cylinder's outline, 384 triangles, no SDF
   `<material>`): the plain tyre's dark with 16 light tread bars, a light hub
@@ -265,3 +265,3 @@ its marker service).
   lumps), the soil pushed aside. The ruts' berm rule was set before the sink
-  existed, to show the dig-in on its own; `TrackParams.berm_gain` scales it
+  existed, to show the dig-in on its own; `ruts: tracks: berm_gain` scales it
   and stays 1.0 (open: design spec 17.5).
@@ -282,3 +282,3 @@ its marker service).
   (2.5°). A tyre drawn lower makes the band its top uncovered farther, all on
-  the tyre, nothing floating. `test_gen_model` checks that each switch takes
+  the tyre, nothing floating. `test_rover_model` checks that each switch takes
   away only its own cue; `ctest` the sink's rule and lag and the ruts' layer
diff --git a/docs/sim/gazebo-lessons.md b/docs/sim/gazebo-lessons.md
index 2c7305e..a15de1c 100644
--- a/docs/sim/gazebo-lessons.md
+++ b/docs/sim/gazebo-lessons.md
@@ -83,3 +83,3 @@ cost a wrong first attempt.
   messages while spinning); its pose is right.
-- **NavSat noise is in degrees** for latitude/longitude; `gen_model.py`
+- **NavSat noise is in degrees** for latitude/longitude; `rover_sim.urdf.xacro`
   converts metres. NavSat reports the ellipsoidal height of the world
@@ -179,3 +179,3 @@ cost a wrong first attempt.
   does a camera `<visibility_mask>`. Hence the rover has no dust by default
-  (`DriveParams.dust`).
+  (`sim.yaml` `dust`).
 - **Orthographic near plane**: an orthographic camera clips at its near plane
diff --git a/docs/sim/manual.md b/docs/sim/manual.md
index 18f48dd..2c0ad6f 100644
--- a/docs/sim/manual.md
+++ b/docs/sim/manual.md
@@ -51,3 +51,3 @@ do. Gazebo GUI's Teleop plugin (⋮ menu → Teleop, topic
 the rover moves for 0.5 s; use its keyboard mode, or build the rover with
-`DriveParams(cmd_timeout=0)`, which holds the last command.
+`cmd_timeout: 0.0` (`rover_sim/config/sim.yaml`, drivetrain), which holds the last command.
 
@@ -61,7 +61,11 @@ the GUI stops both.
 |---|---|
-| `gen_model.py` | Rover parameters (`Params`, `DriveParams`) and the SDF generator; it also writes the station cameras (`viewers.py`) |
+| `config/sim.yaml` | How the simulation models the rover: the drivetrain's settings, the contact model, dig-in, the visual cues, dust, the tyre model and the Gazebo topics, each number with its source; the rover's physical numbers are `rover_description/config/rover.yaml` (Changing the rover) |
+| `config/surfaces.yaml` | The drivetrain's ground surfaces, written from `urc/terrains.py` by `tools/gen_rover.py`: do not edit |
+| `urdf/rover_sim.urdf.xacro` | The simulated rover: `rover_description`'s description with Gazebo's parts (sensors, plugins, visual cues) |
+| `rover_config.py` | The one reader of `rover.yaml` and `sim.yaml` for the simulation's Python (station, referee, judge, viewers, tests), with the rover's Gazebo topics; `Variant`, a rover with numbers changed |
+| `tools/gen_rover.py` | `pixi run sim-model`: the rover's model from the xacro (xacro, `gz sdf -p`, post-processing), its surfaces, dust sprite and tread mesh; it also writes the station cameras (`viewers.py`) |
 | `viewers.py` | The station's cameras: `ChaseParams`, `EyeParams`, `FlyParams` and their models |
-| `models/rover`, `models/chase_camera`, `models/eye_camera`, `models/fly_camera` | Generated by `gen_model.py`: edit the parameters, not these |
-| `plugins/rover_drivetrain.cpp`, `.hh` | System `rover_sim::RoverDrivetrain`: motors, wheel contacts, dig-in and the tyre sink that shows it (it also feeds the ruts), odometry, and the dust when `DriveParams.dust` is on (see design notes) |
-| `plugins/rover_tracks.hh`, `plugins/rover_tracks_render.hh` | The ruts and pits behind the wheels (`DriveParams.ruts`): the layer the drivetrain feeds (cross-sections, the pit rule, the ring, their geometry; no Gazebo includes) and its drawing on the rendering thread (`events::SceneUpdate`, camera sensors only) |
+| `models/rover`, `models/chase_camera`, `models/eye_camera`, `models/fly_camera` | Generated by `tools/gen_rover.py`: edit the YAML files and parameters, not these |
+| `plugins/rover_drivetrain.cpp`, `.hh` | System `rover_sim::RoverDrivetrain`: motors, wheel contacts, dig-in and the tyre sink that shows it (it also feeds the ruts), odometry, and the dust when `sim.yaml`'s `dust` is on (see design notes) |
+| `plugins/rover_tracks.hh`, `plugins/rover_tracks_render.hh` | The ruts and pits behind the wheels (`sim.yaml` `ruts`): the layer the drivetrain feeds (cross-sections, the pit rule, the ring, their geometry; no Gazebo includes) and its drawing on the rendering thread (`events::SceneUpdate`, camera sensors only) |
 | `plugins/terrain_ground.hh`, `plugins/terrain_heightmap.hh` | The ground map reader (`ground.png`, `ground.json`) and the heightmap lookup the C++ systems share |
@@ -100,3 +104,3 @@ the GUI stops both.
 |---|---|---|
-| `/cmd_vel` → | `/model/rover/cmd_vel` | Twist (vx, wz); a command older than 0.5 s counts as zero (`DriveParams.cmd_timeout`, wall clock) |
+| `/cmd_vel` → | `/model/rover/cmd_vel` | Twist (vx, wz); a command older than 0.5 s counts as zero (`sim.yaml` drivetrain `cmd_timeout`, wall clock) |
 | `/odom` | `/model/rover/odometry` | wheel odometry, `odom` → `base_link`, 50 Hz; no slip, so it over-reports a turn in place (2.65× on packed ground) |
@@ -116,3 +120,3 @@ the GUI stops both.
 | `/lander/joint_states` | `/model/lander/state` | the lander's joints (JointMonitor, 50 Hz) |
-| – | `/model/rover/link/rocker_{left,right}/particle_emitter/dust_r{l,r}/cmd` | ParticleEmitter: the drivetrain sets the dust behind the rear wheels (10 Hz); only with `DriveParams.dust`, off by default (no dust, nothing on these topics) |
+| – | `/model/rover/link/rocker_{left,right}/particle_emitter/dust_r{l,r}/cmd` | ParticleEmitter: the drivetrain sets the dust behind the rear wheels (10 Hz); only with `sim.yaml`'s `dust` on, off by default (no dust, nothing on these topics) |
 
@@ -131,12 +135,23 @@ The station's cameras have their own topics (Driver station, below).
 
-Edit `Params` in `gen_model.py`, then `pixi run sim-model` (`sim`, `drive` and
-`sim-test` do it for you). Masses, inertias, wheel positions, the
-differential, tire friction, the sensors and the camera head all come from
-there; `DriveParams` (`Params.drive`) holds every motor, controller, contact
-and dust number of the drivetrain, each with its source or an (A) label for
-an assumption; `ChaseParams`, `EyeParams` and `FlyParams` (`viewers.py`) set
-the station's cameras.
-
-`DriveParams.mode` picks the drivetrain: `"physical"` (the default,
-`plugins/rover_drivetrain.cpp`) or `"diffdrive"` (Gazebo's DiffDrive, every
+The rover's physical numbers are in `rover_description/config/rover.yaml`
+(`rover_description/README.md`): geometry, masses (the inertias come from the
+shapes), joint limits and damping, the differential, the motors and
+drivelines, the tyres, the sensors and the camera head, each with its source
+or an (A) label for an assumption. How the simulation models the rover is in
+`rover_sim/config/sim.yaml`: the drivetrain's settings (its twist input, the
+stand-in motor controller's PI tuning, the motor model's substeps), the
+contact model, dig-in, the visual cues, dust and the tyre model. Edit either,
+then `pixi run sim-model` (`sim`, `drive` and `sim-test` do it for you):
+`tools/gen_rover.py` expands `urdf/rover_sim.urdf.xacro` (the description and
+Gazebo's parts), converts it with `gz sdf -p` and writes `models/rover/model.sdf`.
+`test_equivalence` compares that model with the rover frozen before the
+description existed (`tests/fixtures/rover_model`); a change on purpose
+refreezes the fixtures in the same commit (their README says how).
+`ChaseParams`, `EyeParams` and `FlyParams` (`viewers.py`) set the station's
+cameras. In tests, `simulate.physical(**sections)` and
+`simulate.diffdrive(**sections)` build a variant with sections of `rover.yaml`
+or `sim.yaml` changed, as in `physical(contact={"dig": False}, motor={"current_limit": 10.0})`.
+
+`sim.yaml`'s drivetrain `mode` picks the drivetrain: `physical` (the default,
+`plugins/rover_drivetrain.cpp`) or `diffdrive` (Gazebo's DiffDrive, every
 wheel a velocity servo on anisotropic tyres), kept as the A/B and cost
@@ -144,14 +159,14 @@ reference; in tests `simulate.diffdrive()`. The two must never be in one
 model: DiffDrive's velocity servo overrides the torque. Both rovers carry the
-same camera. `DriveParams.dust` (off: the user's decision of 2026-10-07, see
-design notes) gives both an emitter behind each rear wheel, which the
-physical drivetrain feeds. `Params.tire_compliance` (off) adds sprung
-radial and axial hubs (design spec 6.7; revisit when the wheel type is
+same camera. `dust` (off: the user's decision of 2026-10-07, see design
+notes) gives both an emitter behind each rear wheel, which the physical
+drivetrain feeds. `tyre_compliance` (off) adds sprung radial and axial hubs
+with `rover.yaml`'s tyre (design spec 6.7; revisit when the wheel type is
 known). Three switches, all on, show the dig-in (the user's decisions of
-2026-10-07, design notes, Visible dig-in): `DriveParams.dig_sink` draws a
-dug wheel's tyre sunk at true scale, `DriveParams.ruts` (its numbers in
-`TrackParams`) lays ruts and pits behind the wheels on soft ground, and
-`Params.tread_tyre` draws the tyres with tread bars and spokes, one ochre,
-from `models/rover/meshes/wheel.glb`. All three are visual only; off, each
+2026-10-07, design notes, Visible dig-in): `dig_sink` draws a dug wheel's
+tyre sunk at true scale, `ruts` (its numbers under `tracks`) lays ruts and
+pits behind the wheels on soft ground, and `tread_tyre` draws the tyres with
+tread bars and spokes, one ochre, from `rover_description/meshes/wheel.glb`
+(copied into `models/rover/meshes`). All three are visual only; off, each
 takes away only its own cue, and with all three off the model is the one
-before them, byte for byte.
+before them (`test_rover_model.DigCues`; `test_equivalence`'s `cues_off`).
 
@@ -328,3 +343,3 @@ hides the camera's own housing, all within 0.1 m of the pivot), far clip
 The `ChaseCamera` plugin in eye mode (`<mount>` in the SDF) places it every
-step at rover pose × `Params.camera_xyz` × Rz(yaw)·Ry(pitch): body-fixed, it
+step at rover pose × the camera's `xyz` (`rover.yaml`) × Rz(yaw)·Ry(pitch): body-fixed, it
 pitches and rolls with the chassis like the real camera; unsmoothed, one 1 ms
@@ -636,3 +651,3 @@ from it by name, and each type carries four groups:
   colour, lightness and spread from NAIP), a detail texture and a dust factor
-  (it scales the rover's dust when `DriveParams.dust` turns it on).
+  (it scales the rover's dust when `sim.yaml`'s `dust` turns it on).
 - **Relief**: which lidar swatch, at what amplitude, plus haystack knobs and
@@ -875,3 +890,3 @@ models they use, in about 65 s for all five (Autonomy 23 s, Astrobiology
 deterministic; edit the layouts in `urc/missions/`, not the output. Its pixi
-inputs are `gen_worlds.py`, `gen_model.py`, `urc/**/*.py`, the DEMs, the NAIP
+inputs are `gen_worlds.py`, `rover_config.py`, `rover_description/config/rover.yaml`, `urc/**/*.py`, the DEMs, the NAIP
 rasters, the relief swatches, the soil map and the roughness targets.
@@ -1100,3 +1115,3 @@ generation, before them): 9 skipped (the opt-in
 performance and slow tests), 2 expected failures (the slow-turn judder; dust
-in depth, on the emitter `DriveParams.dust` adds, kept as the record for that
+in depth, on the emitter `sim.yaml`'s `dust` adds, kept as the record for that
 switch); `pixi run colcon-test` 3 C++ programs; `sim-slow` (the mission routes)
@@ -1150,3 +1165,3 @@ spin.
   publish `cmd_vel` at a rate (the station sends 20 Hz), or use
-  `DriveParams(cmd_timeout=0)` for Gazebo's GUI Teleop.
+  `cmd_timeout: 0.0` (`sim.yaml`, drivetrain) for Gazebo's GUI Teleop.
 - **The rover will not turn in sand, or stops climbing:** that is the dig-in
diff --git a/rover_sim/README.md b/rover_sim/README.md
index 4437f7f..8fe8e09 100644
--- a/rover_sim/README.md
+++ b/rover_sim/README.md
@@ -23,3 +23,4 @@ Every world needs the rasters fetch-data fetches, the test ground too (the tasks
 | `plugins/` | Gazebo system plugins (RoverDrivetrain, RockerDifferential, JointMonitor, ChaseCamera, FlyCamera) and their C++ tests; colcon builds them into the workspace's `build/rover_sim` and installs them to `lib/rover_sim` |
-| `gen_model.py`, `viewers.py`, `models/` | The rover's parameters and its generated Gazebo model; the driver station's cameras |
+| `config/`, `urdf/`, `rover_config.py`, `tools/gen_rover.py` | How the simulation models the rover (`config/sim.yaml`), the Gazebo parts it adds to `rover_description`'s description (`urdf/rover_sim.urdf.xacro`), the one reader of both YAML files for the simulation's Python, and the generator of the rover's Gazebo model (`pixi run sim-model`) |
+| `viewers.py`, `models/` | The driver station's cameras; the generated models (the rover's, the cameras') |
 | `gen_worlds.py`, `urc/`, `worlds/` | The URC world generator and the worlds (`rover_test.sdf` is tracked; the rest is generated and git-ignored) |
@@ -34,2 +35,4 @@ Comments and docstrings written before the simulation joined this repository cal
 
+The rover itself is `rover_description`'s: its physical numbers are `rover_description/config/rover.yaml` and its README says how to change it. `config/sim.yaml` holds how the simulation models it (the drivetrain's settings, contact, dig-in, the visual cues, dust, the Gazebo topics); `pixi run sim-model` builds `models/rover/model.sdf` from both, and `tests/test_equivalence.py` checks it against the rover frozen before the description existed. `gen_model.py`, which held all of it as Python before, is gone; comments in `urc/sdf.py`, `urc/terrains.py`, `urc/textures.py` and `urc/world.py` still name it, because a change to a `urc/` module renames the media files every world references (`urc/media.py` names them after their sources), so they wait for a change that regenerates the worlds anyway.
+
 ### Launch Files
```

```bash
T=/private/tmp/claude-502/rover-pr2
cd /Users/alarion239/Desktop/hurc_ws/src && git apply "$T/t16.patch" && git add -A \
  && git grep -n "gen_model\|DriveParams\|TrackParams\|\bParams\." -- README.md rover_sim/README.md rover_description docs/sim | cut -c1-120
```

Expected: one line, `rover_sim/README.md`'s paragraph that says `gen_model.py` is gone and why four `urc/` comments still name it.

- [ ] **Step 3: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add -A && git commit -q -F - <<'EOF'
PR 2: docs: the rover description, where each number lives, how to change the rover

rover_description/README.md says what is where, how to change the rover (rover.yaml, sim-model, refreezing the
equivalence fixtures on purpose) and how to view it in RViz. The README's package list and test line, rover_sim's
README and the simulation's manual, design notes and Gazebo lessons name rover.yaml, sim.yaml, the wrapper xacro,
rover_config.py and gen_rover.py where they named gen_model.py and its Params.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 17: Hand-off

**Files:** none.

- [ ] **Step 1: The branch, complete and not pushed**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain && git log --oneline 9208047..HEAD
git branch -vv | grep pr2/rover-description; git ls-remote --heads origin pr2/rover-description | wc -l
cd /Users/alarion239/Desktop/Rover && git status --porcelain | wc -l
```

Expected: a clean tree; the plan's two commits and the thirteen commits of Tasks 2 to 13 and 16 (pixi; rover_description x3; fixtures; comparison; sim.yaml; the wrapper and generator; the drive test; sim-model; the tests' variants; the consumers; the documents); `pr2/rover-description` without an upstream; `0` (not on GitHub); the Rover repository untouched (`0`).

- [ ] **Step 2: Remove the temporary files**

```bash
rm -rf /private/tmp/claude-502/rover-pr2 && ls /private/tmp/claude-502 | grep -c rover-pr2
```

Expected: `0`. The workspace keeps its environment, `build/`, `install/`, `log/`, the generated worlds and maps.

- [ ] **Step 3: Report to the user**

Report: the commits; the baseline and final counts (Task 1 Step 4, Task 15 Step 2) and the test-id diff (Task 15 Step 1); the equivalence result (ten models, zero differences; the drive test bit for bit); the worlds byte for byte (Task 14); the view launch check and why the windows were not opened (Task 5 Step 5); Decision 1 (the worlds still include `model://rover`, now generated from the description; spawning from `robot_description` comes with PR 3's `sim.launch.py`, which must spawn the post-processed model or move its fixes into the description) and Decision 10 (four `urc/` comments still name `gen_model`); the lock: PR 2 and the data pull request (`data/lidar-from-usgs`: GDAL's tools, PROJ, laspy) both change `pixi.toml` and `pixi.lock`, and the lock is `merge=binary`, so whichever of the two merges second merges `pixi.toml` by hand and regenerates the lock with `pixi lock` (never by merging the lock's text; then `pixi install`), and checks that against the base only the two pull requests' packages were added (the data plan's Task 15 says the same); and the decision that needs the user: the push of `pr2/rover-description` and its pull request, which builds on PR 1 (`import/simulation`) and should be opened after PR 1 merges, or against `import/simulation` until then.

---

## Acceptance (spec 9.2, row 2; the task's list)

| Criterion | Where it is shown |
|---|---|
| `rover_description`: `config/rover.yaml` with every physical number; `urdf/rover.urdf.xacro` built from it with `xacro.load_yaml`; `urdf/ros2_control.xacro` (wheels: velocity command, position/velocity/effort state; rockers and head: state only; plugin argument, default `rover_control/RoverHardwareStub`, loaded by nothing); `meshes/` with the tread tyre and its generator; `launch/view.launch.py` with a saved RViz configuration; `package.xml` dependencies; no Gazebo plugin named | Tasks 3, 4, 5; `test_description` (10 tests) |
| Every field of `Params`/`DriveParams`/`TrackParams` sorted between `rover.yaml` and `rover_sim/config/*.yaml`, the PI tuning decided | the field table, Decision 3, Tasks 3 and 8 |
| `rover_sim/urdf/rover_sim.urdf.xacro`: the description plus every Gazebo-only part (sensors, LED, differential, drivetrain with its twist input and surface rows from the catalogue, joint states, dust off, the cues, the tread switch); no `gz_ros2_control` | Task 9; Decision 11 for the joint monitor |
| `model.sdf` generated from the wrapper (xacro -> URDF -> `gz sdf -p` -> post-processing) by `pixi run sim-model`; `gen_model.py` removed; its callers on `rover_config.py`; the station's cameras stay simulation-side | Tasks 9, 11, 12, 13 |
| The equivalence test passes: today's model frozen (and nine variants), the new one equal number for number; converter quirks handled explicitly | Tasks 6, 7, 9 (zero differences), 10 (bit for bit in Gazebo) |
| `check_urdf` passes | Task 3 Step 5, Task 9 Step 7, `test_description` |
| `view.launch.py` starts, `/robot_description` published, the windows not opened for a stated reason | Task 5 Step 5 |
| The full suite passes with PR 1's counts plus the new tests | Task 15 (546 tests: 525 + 21; 9 skipped, 2 expected failures) |
| `rover_control`'s tests pass | Task 13 Step 3, Task 15 (57) |
| Worlds and mission sheets regenerate byte for byte | Task 14 |
| Documents: `rover_description/README.md`, the README's package list, `rover_sim`'s documents | Task 16 |

## Risks

| Risk | Handling |
|---|---|
| A later xacro or sdformat changes the converter's output (names, defaults, where merged sensors go) | The equivalence test fails and names the difference; `gen_rover.postprocess` raises on a shape name or joint `<physics>` it does not know instead of guessing |
| The generated `model.sdf` orders the elements inside a link differently from the frozen one (collisions before visuals, the GNSS sensor before the IMU) | Gazebo creates the same links, joints and systems in the same order (the comparison checks the order of links, joints and plugins); `Drives` shows the same trajectory bit for bit; the IMU and NavSat are updated by their own systems, so their order in a link does not change which noise a sensor draws |
| The comparison's absolute tolerance (1e-12) hides a change of a tiny number | Only numbers below about 1e-3 relative to it could hide, and the rover has none that matters (the dust's `particle_scatter_ratio` 1e-6 is a float in SDF, read the same either way); the plugins' numbers are written with 9 significant digits on both sides |
| The rover is changed on purpose and the fixtures are refrozen without thought | `fixtures/rover_model/README.md` asks for the reason in the commit; the reviewer sees the fixture diff beside the YAML change |
| Comments in four `urc/` modules name the removed `gen_model.py` | Decision 10: changing them renames every world's media; `rover_sim/README.md` says so; a pull request that regenerates the worlds anyway updates them |
| The tests' variants are now built by xacro and `gz sdf` (about 0.1 s each instead of microseconds) | Cached per process by variant (`functools.lru_cache`); the dry run's full suite took 788 s, PR 1's documented 782 s |
| `test_render`'s dust emitter is now cut from a generated model instead of built alone | The same elements, order and values (Task 12); in the dry run `test_render.Render.test_dust_is_drawn` passed and `test_depth_unchanged_by_dust` failed as expected, as before |
| pytest 9 in the pixi environment refuses ROS's `launch_testing` plugin | Decision 9: `rover_description`'s tests run as unittest under CTest; on apt-based Ubuntu with pytest 7 nothing changes |
| The view launch file's windows are not exercised in CI or by the implementing agent | Its launch description is tested with and without a display; the headless launch is run in Task 5; the user sees the windows with `ros2 launch rover_description view.launch.py` |
| `rover_control`'s `config.hpp` keeps a second copy of the placeholder geometry | Unchanged until PR 3's tank kinematics read `rover.yaml` (spec 5: "`kRover` and `config.hpp` go away"); `rover_description/README.md` says so |
| PR 2 builds on PR 1, which is pushed but not merged | Task 17's report: open PR 2 after PR 1 merges, or against `import/simulation` until then |
| PR 3 spawns the rover from `/robot_description`, which Gazebo converts without `gen_rover.postprocess` (other colours, the LED's visual renamed, frames and infinite stops kept: Decision 1) | Decision 1 says what PR 3 must spawn; Task 17's report names it, so that PR 3's plan starts from it |
| The data pull request changes `pixi.toml` and `pixi.lock` too (GDAL's tools, PROJ, laspy), and the lock is `merge=binary -diff` | Whichever of the two merges second regenerates the lock with `pixi lock` after merging `pixi.toml` by hand (Task 17's report; the data plan's Task 15) |
| The user's other workload slows the machine during the suites | Task 1 records the baseline on the same machine; a failing timing test is rerun alone before it counts as a regression |

## Restacked on PR 1's fixes (2026-10-10)

`import/simulation` gained four commits after this plan was carried out (CI on a fresh clone): `fbc4aee` (fetch-data reads USDA's NAIP 2024 tiles marked alpha), `6c59cd8` (a raster fetch-data rebuilds carries the original file's georeferencing and metadata), `86406c8` (CI ends with `git diff --exit-code`, which fails when the tests left a tracked file changed) and `195577e` (PR 1's plan as amended). The branch was rebased onto `195577e` (`git rebase import/simulation pr2/rover-description`). One commit conflicted, `PR 2: docs: ...`, in the README's Tests section: it keeps PR 2's `pixi run test` line and PR 1's CI line with `git diff --exit-code`. Every other commit applied as it was, with the same changes.

- Where the tasks say `9208047..HEAD`, the branch's own commits are now `import/simulation..HEAD`. The fixtures stay what `gen_model.py` wrote at `9208047`: the four commits do not touch `gen_model.py` or anything it reads.
- Counts: PR 1 now has 529 tests (525 + three tests of the tags and one of the alpha mark in `test_fetch_data`, 21), so the suite is 551: 529 + 22 (Task 15 planned 21; the review fixes added `test_equivalence`'s twelfth).
- CI's new `git diff --exit-code` after `pixi run test` checks what Task 15's `tree clean` checks: the suite's `sim-model` writes the tracked rover model, mesh and sprite with the committed bytes.
- Checked after the rebase (M, 2026-10-10, on the Apple M4): `pixi run build` (3 packages); `test_equivalence` (12) and `test_rover_model` (46) OK; `pixi run test`: colcon 57, 5 and 3 tests passed, then `Ran 551 tests in 787 s`, `OK (skipped=9, expected failures=2)`, exit 0, and the tree clean after it.
- Lock: the data pull request changes `pixi.toml` and `pixi.lock` too; whichever of the two merges second regenerates the lock with `pixi lock` (Task 17, Risks). Tried on 2026-10-10 in a scratch folder: `git merge-file` merges the two `pixi.toml` files without a conflict, and `pixi lock --dry-run` on this branch's lock adds the data pull request's laspy, lazrs-python, m2r2 and mistune on linux-64 and osx-arm64 and changes nothing else.
