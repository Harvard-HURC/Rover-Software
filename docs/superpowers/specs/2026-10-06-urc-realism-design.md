# URC realism: real ground, a physical drivetrain, believable pictures and a free-fly map — design

Date: 2026-10-06
Status: **implemented** (2026-10-07), in waves 0–3 on git branches merged into `main` (WS-0; WS-D, WS-T1,
WS-A, WS-F1; WS-T2 in phases T2a and T2b, WS-F2; WS-V). This document is revision 2 of the design as it was
reviewed, kept as written, except §5.8 (updated by WS-0) and this status. What changed during implementation,
and where each goal and calibration target stands, is in §17 (Changes during implementation). The code is the
reference; `sim/README.md` describes it as built.

Inputs (research of 2026-10-06). Every number below comes from one of these or carries a label:

| Research | Artifacts |
|---|---|
| Terrain and soils at MDRS | `sim/data/research/mdrs_terrain_measurements.json`, `mdrs_surface_types_crops.jpg` |
| Detailed maps | `sim/data/dem/*.tif` (+ `.json` provenance), `sim/data/imagery/*.tif` (+ `.json`), `sim/data/research/{overview_*,closeup*,microrelief_*,landforms_*}`, `lidar_microrelief.json`, `colour_stats.json`, `strata_colour_ramps.json`, `ssurgo_soils.json`, `ssurgo_polys.geojson` |
| Drivetrain prototype | `/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/drive.TDh3/` (temporary; it still existed on 2026-10-06 20:25. See R5: copy it now. The algorithms are written out in §6) |
| Rendering prototypes | `sim/data/research/render/` (`manifest.json`, `steps/`, `before_after/`, `prototype/`) |
| Fly camera and map prototypes | `sim/data/research/flycam/` (`measurements/`, `prototype/flycam_proto.cpp`, `exp5_orthomap.py`) |
| Critiques of revision 1 | Physics and data: `scratchpad/critique/` (`rms.py`, `rms_worlds.py`, `rms_autonomy.json`, extracted source texts). Engineering: `scratchpad/critique_g1.CxWO/` (CPU-time runs: `run*.sh`, `log*_*.txt`) |
| This revision | `scratchpad/specrev/`: `spin.py`, `table.py` and `dig.py` (spin-in-place moment balance, §5.6 derived columns); `types_rms.py` and `types_rms.json` (the real Autonomy square painted by the §5.3 rules, roughness per type); `sda.py` (SSURGO surface fragments). Scratchpad paths are under the session scratchpad above |

Labels used throughout:
- **(M)** measured in a research workflow on this machine (gz-sim 8.10, DART, ogre2 on Metal, Apple M4, 16 GB).
  **(M, rev2)** means measured or computed for this revision.
- **(R)** read in a cited source `[n]` (§15).
- **(A)** assumed, to be calibrated.
- **(T)** transferred: measured elsewhere, applied here.

## 0. What changes, in one paragraph

Today (M):
- The rover turns in place at 89 % of the commanded rate, with zero jitter, and stops dead in 4–23 ms.
- Its ground is about 3–4× too smooth at 4 m scale and 1.5–3× too smooth at 8 m. It is about right at 16 m.
- The ground is coloured in height bands, under a terrain shader that renders it mirror-smooth.

After this work:
1. **Terrain.** The Autonomy world stands on the 0.5 m USGS lidar DEM, draped with de-shaded NAIP 2024 imagery.
   The synthetic worlds keep their mission layouts. They gain the roughness of each ground type at MDRS, as
   measured on that lidar. They also gain tabular sandstone slabs, ledges, sand, dust and gravel, all from one
   ground catalogue and one ground raster per world.
2. **Drivetrain.** A C++ drivetrain drives each wheel by torque, through a DC-motor model. Every wheel contact,
   on terrain or on objects, gets:
   - a surface-dependent friction circle;
   - rolling resistance;
   - sideways bulldozing resistance in loose soil;
   - slip;
   - a dig-in state on sand.

   Turning in place then runs at a rate that depends on the surface: 0.44 of the command on rock, 0.38 on
   packed ground and 0.26 on fresh sand, slowing as the wheels dig in. It judders when slow on rock. The loaded
   diagonal stalls when the current limit is below what the ground demands.
3. **Pictures.** A patched terrain shader, a procedural sky with haze, the sun of the mission date, a visible
   far-field horizon and wheel dust make the pictures read as a desert.
4. **Fly and Map views.** The driver station gets a Fly view and a Map view, with a Gazebo-rendered orthophoto of
   each world.

## 1. Goals, in the user's words

| The user said | Goal | Done when |
|---|---|---|
| "the terrains are still kind of simplistic", "less even" | Roughness of every ground type matches its MDRS counterpart at 4–32 m | Each synthetic ground type: the median plane-detrended RMS height at 4/8/16 m lies within the real p25–p75 of the same type, measured on the real Autonomy square painted by the same rules (§5.4). Each synthetic world's natural ground: RMS p50 and p90 at 4/8/16 m within ±30 % of the real whole-square distribution. Today's worlds are 3–4× too smooth at 4 m and 1.5–3× at 8 m, and about right at 16 m (M, verified rev2) |
| "more kind of slabs" | Tabular caprock blocks and sub-metre ledges where MDRS has them | Block fields reproduce the measured cumulative counts N(≥1), N(≥2), N(≥4 m) within ±30 %, and 8–9 % cover by 1–7 m blocks (M). Features of 7 m or more are ledges or macro shape. Sizes below 1 m are extrapolated (A). Rims shed slabs; benches carry 0.1–1 m risers (§5.5) |
| "sometimes dust or sandish" | Sand that sinks, resists, slips and digs in; dusty clay; gravel lags; dust behind the wheels | Spin in place on fresh loose sand: 0.26 ± 0.04 of the command. A sustained spin slows to 0.6–0.8× of that as the wheels dig in (§6.5, (A) strength, Q12). Wheels sit 2–3 cm into sand on screen. Dust plumes scale with speed, slip and surface |
| "Take a look at how terrains actually look… load more detailed maps" | Real 0.5 m lidar and NAIP 2024 where a world stands on real ground | Autonomy is built from `route_area_lidar_0p5m.tif` and `route_area_naip2024.tif`. Gazebo-rendered map vs its colour map: smoothed Lab ΔE ≤ 5. Colour map vs boosted NAIP: ΔE ≤ 5 outside de-shaded and inpainted pixels (§11). Today the rendered map differs from NAIP by ΔE 29.3 (M) |
| "analyze the soil that usually is in these kind of deserts" | Surfaces from the NRCS soil survey and published traction data | Every ground type carries μs, μk, Crr, bulldozing, slip, sinkage and dig-in, each with a source or an (A) label (§5.6). Autonomy's ground raster comes from SSURGO map units |
| "I do not feel like this rover is real, it feels like a simulation" | Physical drivetrain and believable pictures | Drivetrain targets of §6.9 met. Terrain no longer mirror-like. Sky, haze, mission sun and a visible far field in every URC world and every station view |
| "add a free map… birdeye position of view, flying around and looking, inspecting" | Fly view and Map view in the driver station | Fly view at ≥ 15 fps (gate G6) that never enters the ground, with follow, top-down and orthographic modes. Map view shows a rendered orthophoto of the world with live rover and fly-camera markers |
| "turn inplace feels very unrealistic, I do not think it would be as smooth" | Skid-steer turning physics | Spin-in-place yaw rate per surface from the quasi-static moment balance, ±0.04 (rock 0.44, regolith 0.38, fresh sand 0.26; table in §5.6). Today it is 0.889, and 0.000 on friction tiles with μ ≤ 0.5, a bug (M). Also: stick-slip judder at slow rates on rock; stall/no-stall at 0.7× and 1.3× the analytic current; dig-in on loose sand (§6.9) |
| "I really love your current implementation of the environments" | Keep the missions as they are | Every point, object, task and judge of the four missions keeps its meaning; the mission tests keep passing |
| "make sure that we reuse terrains, rocks etc." (standing instruction) | One catalogue, one raster, shared assets | Each ground type, texture, mesh variant and recipe is defined once (`terrains.TYPES`, `urc_media`); missions only say where and how much |

## 2. Non-goals

- The robotic arm.
- Deformable ground geometry: ruts, wheel tracks, mud sticking to wheels (§4 says why). The dig-in state of
  §6.5 changes resistance, not geometry: a digging wheel does not visibly sink further than the static carve.
- A wet-weather ("after rain") variant. It is designed as a catalogue switch (§5.6) but built only if asked (Q4).
- Photoscanned CC0 textures (ambientCG, Poly Haven): not among the approved download sources (Q6).
- Linux/GLSL validation of the shader patch (only Metal was tested).
- Speeding up the Equipment Servicing lander beyond measuring it (Q2).

## 3. Decisions

| # | Decision | Evidence |
|---|---|---|
| D1 | **Terrain friction comes from a per-contact callback, not from box tiles.** The rover's drivetrain system sets the friction of every wheel contact. On the heightmap, μ comes from the ground under the contact point. On other terrain-link shapes it comes from the collision map in `ground.json`. On any other model it comes from that model's SDF `<surface>`, or else the `manmade` surface. No wheel contact is left to DART's default rule (§6.4). Friction tiles, carving for tiles and levelling for tiles are deleted once the drivetrain passes its tests. The carving code is reused for sand sinkage. | (M) gz-sim 8.10's `events::CollectContactSurfaceProperties`, with `components::EnableContactSurfaceCustomization` on the wheel collisions, overrides friction on heightmap contacts: a zone μ 0.3 on the slickrock heightmap cut wheel torques to 0.30× with no tiles. DART honours per contact: μ, the friction direction, slip compliance, restitution, CFM and ERP. (R) gz-sim 8 `Physics.cc` registers one world-level callback and calls it for every contact that involves a customised collision. (M) Bug today: DART takes min(μ_tyre, μ_ground) per direction, which erases the tyre's 0.5/1.0 anisotropy. On every tile with μ ≤ 0.5 (lanes 0.20/0.35/0.50, sand 0.40, clay 0.25), turning in place gives 0.000 while the wheels spin at 3.4–8.5 N·m. Box friction with wheelbase > track cannot turn in place at all (M: ratio 9e-8), so an untouched contact would bring the bug back on objects. |
| D2 | **A C++ system `RoverDrivetrain`** drives the wheels: per-wheel DC motor + gearbox (+ driveline compliance and backlash) + PI speed control, applied as joint torque. DiffDrive stays available as a model variant (D22). | (M) DiffDrive makes each wheel a DART velocity servo, capped at the SDF effort (30 N·m, reached only in the first ~15 ms). The result: steady turn yaw-rate std 0.0000 rad/s; wheel torque 10.43 ± 0.01 N·m whatever the rate; no current, no stall. The prototype drew 4–10 A per wheel; a 10 A limit stalled a diagonal; washboard wheel-torque std was 0.2–0.8 N·m instead of 12.8 N·m. Rise times are not evidence for the motor model: they come from the assumed setpoint ramp (§6.3). |
| D3 | **Friction circle** (friction direction along the contact's slip). **Stribeck peak μs > μk only on firm surfaces**; μs = μk on loose sand and powder. Smooth spatial μ noise. | (R) Isotropic Coulomb contact predicts a spin-in-place rate of c²/(a²+c²) of the command, for wheels at (±a, ±c). Clearpath compensates Husky with `wheel_separation_multiplier` 1.875, an effective yaw ratio of 0.533, vs 0.540 predicted for wheelbase 0.512 m and track 0.555 m [18]. Pioneer P3-AT: α·χ = 0.63–0.71 measured on asphalt and concrete, vs 0.687 predicted [16]. Ours (a = 0.45, c = 0.40): 0.441; prototype 0.42–0.44 on μ-only lanes (M). (R) Loose granular soils follow an exponential shear curve with no peak; dense or crusted soils show a peak [34]. (A) μs/μk 1.1–1.25 on firm types; Stribeck speed 0.03 m/s; noise σ 0.2 over 0.3 m. |
| D4 | **Solver per world**: PGS where gates G2 and G5 pass, Dantzig elsewhere, with the per-wheel friction caveat documented. | (M) DART's default Dantzig solver sizes friction limits from loads computed before friction: one contact carried 60 N of friction on a 25 N load at μ = 1. PGS gives each wheel μ times its own load, which the diagonal-unloading behaviour (§6.9) needs. Cost (M, critique E, CPU time, interleaved pairs): PGS/Dantzig = 1.22–1.25 in Delivery; 1.05–1.50 in Equipment Servicing, where PGS ran below real time at idle on a loaded machine. Revision 1's "+6 %" came from a Python-heavy harness and is retracted. PGS solver noise is numerical; it is not counted as realism. |
| D5 | **Rolling resistance and bulldozing are hub forces** from the ground table, never joint torques: rolling resistance along the wheel's heading, bulldozing along its axle. | (M) As a joint torque, the speed controller cancels rolling resistance inside the joint: current is drawn, but no force reaches the ground. As a hub force on sand (Crr 0.2): 3.2–3.5 N·m per wheel at 0.5 m/s. (R) Ishigami et al. split the side force of a wheel in loose soil into shear under the contact and sidewall bulldozing [20]. A hub force along the axle models the second part. A friction-ellipse μ scaling, as in revision 1, leaves the spin rate unchanged (M, rev2: 0.4414 at λ = 1.2) and is dropped. |
| D6 | **Autonomy moves to the 0.5 m lidar DEM and NAIP 2024**: 4097 samples over 2048 m if gate G1 passes, else 2049. Collision and visual always share one grid. | (M) The 0.5 m lidar adds what the 1 m DEM smooths away: rill networks on banded mounds (5.5 cm RMS, max slope 48° → 57°), 0.5–1 m ledge steps (21.7 cm RMS), wash cutbanks (7.9 cm). At 4 m the real central 600 m has RMS 2.28 cm on the lidar vs 1.29 cm in today's 3DEP world (M, rev2). 0.25 m is not supported by the point density (53–58 % empty cells). (M) In the render prototype, visual and collision on different grids differed by p99 18 cm: not acceptable. (M, critique E) A 4097² collision heightmap costs +0.6 s start-up, +200 MB and no measurable per-step time. |
| D7 | **The synthetic worlds keep their mission layouts and get real MDRS micro-relief.** High-pass residuals of real 0.5 m lidar windows ("relief swatches") are added to their macro shapes with variance-preserving feathering, plus procedural haystack knobs and rills. Moving Astrobiology onto the real DEM at its real coordinates is offered as an option (Q1). | The user loves the current environments. (M) Real lidar windows exist for every landform. The research close-ups show real banded hills, a dry wash and a sandstone ledge at Astrobiology's coordinates (`closeup_astro_*.png`), so the option is feasible. |
| D8 | **One ground catalogue, one ground raster.** `terrains.TYPES` gains traction, palette, relief and clutter recipes. Each world writes `ground.png` (one type index per heightmap sample) and `ground.json` (legend, traction table, collision map). These are read by the drivetrain (traction), the colour-map baker, relief transfer, clutter placement, the station's map and the sheet. | The user's reuse instruction. Today friction (tiles), colour (height layers, decals) and rocks are three separate mechanisms. |
| D9 | **Slabs are a second rock shape and risers a third.** Both are merged into the existing 128 m rock chunks of the terrain link. On real-DEM worlds, recipe slabs are limited to D < 1 m, because larger blocks are already in the DEM. | (M, README "Gazebo lessons") Separate static bodies cost 3×. Every collision shape costs ~0.6–1 µs per 1 ms step, touched or not. Merged chunk meshes avoid both. (M) Blocks and steps of 1 m or more show in the 0.5 m bare-earth DEM. |
| D10 | **Shrubs are visual-only merged meshes** (GLB, 128 m chunks), 3D only within a mission radius, baked as dark dots into the colour map elsewhere. On the sand sheet they are at most 0.3 m tall; on wash margins 0.5–2.5 m. Autonomy uses NAIP dark spots that pass a local NDVI-anomaly test. | (M) Absolute NDVI is unusable (median −0.028 to −0.029). Relative NDVI works: dark spots carry a local NDVI anomaly of +0.011 to +0.017, vs a background p90 of +0.004, and 53–68 % of dark-spot pixels exceed +0.01, vs 2 % of all pixels (critique P). (M) Lidar vegetation above 25 cm covers 0.006 % of the sand-shrub plain (veg p50 0.29 m), but 7 % of the wash cutbank (p50 0.85 m, p95 2.5 m). (M) 9,208 shrubs within 400 m as meshes cost 1.48 M triangles, +390 MB and +0.9 ms per frame. |
| D11 | **Terra layer 0 is a full-size colour map** (with a flat normal map), plus up to three shared detail layers with per-texel compensation. Height bands (`Layer`) stay only as a fallback. | (M) Calibration: when compensation is done in linear light, the rendered mean equals the colour map to 0.1 DN (13 DN off in sRGB). (M/R [26]) SDF requires `<normal>` for every texture; Terra weights come only from height. |
| D12 | **A patched copy of gz-rendering's media**, made at build time from the pixi env plus a diff, and set via `GZ_RENDERING_RESOURCE_PATH`. It carries a Terra roughness fix, a procedural clear sky and distance haze. If the diff does not apply, it warns and the stock media are used; the build never fails. | (M) Stock Terra gives roughness 0 (a mirror) when a layer has no roughness map; the dark wavy "puddles" in today's worlds are sky reflections. (M/R [26]) gz-sim 8 never applies SDF `<fog>` or `<sky><cubemap_uri>`. Frame cost within noise. |
| D13 | **Sun of the mission date**: 2027-05-28 10:30 MDT at 38.418 N, −110.777 → elevation 49.8°, azimuth 102°, direction (−0.630, 0.138, −0.764); intensity 1.4, colour (1.0, 0.95, 0.87); ambient (0.32, 0.34, 0.40). **NAIP is de-shaded before draping**, so slopes are not shaded twice. | (M) Computed with the NOAA algorithm in the render prototype; intensity honoured (mean ground 147 → 169 DN). Critique check: 49.9°, azimuth 102.5°. (M) NAIP 2024 has west-facing steep slopes 15 % darker: a baked morning sun. Draping it unchanged under a sim sun shades slopes twice and keeps NAIP's cast shadows under a different sun. De-shading method in §5.7. |
| D14 | **A far-field ring per world** (visual GLB, 65 × 80 km on a 120 m grid, Earth curvature, a hole under the terrain) replaces the 8 km horizon plane. **Viewer cameras and the rover's RGB render to 80 km**; the depth image stays clipped at 40 m. A 2–8 km middle ring is phase 2. | (M) The Henry Mountains and Factory Butte appear in the right places; +260–300 MB; frame cost within noise, measured with far clip 80,000 m in every view. (R) Today's clip far is 2000 m (eye, chase), 6000 m (fly, rev1) and 40 m (rover RGB-D). gz-sensors 8 `RgbdCameraSensor` reads `<depth_camera><clip>` separately and clips the depth buffer, so RGB can reach 80 km while depth stays at 40 m. |
| D15 | **Dust**: one particle emitter behind each rear wheel, on the rocker link. The drivetrain drives its rate from wheel speed, slip and the surface's dust factor. Emitters start with `<emitting>false</emitting>`, with an explicit `<topic>` and `<particle_scatter_ratio>` 0, so depth and lidar do not see dust (Q11). | (M) Particle emitters render in camera sensors; tuned plume parameters in `render/manifest.json`; rate can be changed at runtime on the emitter's `cmd` topic. (R) `<emitting>` defaults to true, and gz-sim creates emitters from link SDF even without the system. `particle_scatter_ratio` defaults to 0.65, which makes depth and lidar see particles. |
| D16 | **FlyCamera is its own C++ system.** It integrates velocity commands every physics step, keeps clearance from the world's own heightmap, and switches to orthographic projection from an `events::SceneUpdate` hook, keeping no rendering pointer. Its deadman runs on wall time by default and on sim time in tests. | (M) Picture motion CV 0.46 % with plugin integration, vs 26–35 % with Python `set_pose`. Blocking Python service calls stall ~1 s under busy subscribers. PreRender/Render/PostRender hooks cost 35–45 % sim speed; SceneUpdate costs nothing. SDF orthographic settings are ignored. A stored CameraPtr crashes gz at shutdown. |
| D17 | **The Map view uses an offline orthophoto rendered by Gazebo** for each world (`sim/tools/render_map.py`), with live markers. The hillshade stays as a fallback, with a fixed metres-per-colour scale. The map is stale when any input changes: world SDF, heightmap, colour map, GLBs, media patch. | (M) 4096 px orthophotos in 4.7–17.7 s per world; 2–3.3 MB JPEG; tile seams geometrically continuous. |
| D18 | **Tyre compliance (hub links with radial and axial springs) is phase 2**, behind a `Params` flag. | (M) It turns 400–480 Hz contact chatter into a physical 12–16 Hz wheel hop, but adds 8 joints and changes the joint tree. Its RTF cost was not measured separately. |
| D19 | **Imagery: NAIP 2024 is the production source.** The NAIP 2021 route-area file is redundant; deleting it needs the user's OK (Q3). The far-field DEM and overview stay. | (M) NAIP 2024 is 4-band, newer, and on the DEM's pixel grid (each 1 m DEM pixel = 2 × 2 image pixels). |
| D20 | **Default drivetrain parameters = the prototype's validated set** (gear 50, efficiency 0.8, 20 A), not the datasheet alternative (51, 0.7), until the real drivetrain is chosen (Q7). | (M) `results_summary.json`; (R) IMS 3-stage datasheet [22]. |
| D21 | **Dig-in state per wheel on loose soil.** Extra sinkage grows with slip at the contact and heals with travel. It scales that wheel's rolling resistance (×D) and bulldozing (×D²), within a cap D_max (§6.5). The default is mild (D_max 1.15–1.25, (A)); a strong preset reproduces getting stuck (Q12). | (R) Wheel sinkage at slip 0.6 is 3–7× the static sinkage for lunar-rover wheels [36]; slip sinkage is a known soil–vehicle effect [35]. (R) URC 2017, Team Anveshak (45.6 kg): "When performing a zero radius turn in loose soil, one of the drive motors stalled"; the wheels "got stuck in the soil as they were trying to dig the soil" [14]. Without dig-in, the model cannot dig: spinning on sand needs only 5–8 N·m per wheel, against 35.6 N·m available. |
| D22 | **`DriveParams.mode` = `diffdrive` \| `physical`.** In wave 1 the default stays `diffdrive`, and its generated output stays byte-identical. WS-T2 flips the default at the start of wave 2. DiffDrive stays as a permanent variant for A/B tests, the cost regression and as a fallback. | (Critique E) A rover-model change in wave 1 would change tests owned by other workstreams and deadlock the wave gate. The "≤ +25 % vs DiffDrive" check needs a DiffDrive rover. (M) DiffDrive and torque control must never be in the same model: with DiffDrive present, `Joint::SetForce` overrides its velocity command and the rover does not move. |
| D23 | **Slip compliance uses the real wheel speed (no floor) and is zero while a contact sticks.** One isotropic compliance per contact. | (R) DART slip compliance is constraint force mixing on the friction rows: slip speed = compliance × force, even at rest (`dart/dynamics/ShapeFrame.hpp`; gz-physics `ContactProperties.hh` "force-dependent slip"). Revision 1's 0.1 m/s floor would make a parked rover creep, about 0.5 m/min on 15° regolith (computed, critiques P and E). gz `WheelSlip` uses the real wheel speed. |
| D24 | **Effective-track multiplier** `track_multiplier` in the drivetrain, default 1.0 (raw skid-steer response). Like Clearpath's `wheel_separation_multiplier`, it scales both the commanded wheel-speed difference and the odometry track. | (R) Clearpath ships 1.875 for Husky [18]. Without compensation, the operator gets 0.18–0.44 of the commanded yaw rate (§5.6). The default is a user decision (Q10). |
| D25 | **Workstreams work in isolated copies of the repo.** Preferred: `git init`, a baseline commit and one git worktree per workstream (needs the user's OK, Q9). Otherwise, a private copy per workstream with its own build directory and temp outputs. No `pixi run sim-test` inside a wave. | (R) The repo is not under git. `pixi run sim-test` rebuilds `sim/build` and regenerates rover, camera and world models from whatever is on disk. `WorldBuilder.terrain()` `rmtree`s the terrain model. Two agents running make in one CMake dir corrupt it. |

## 4. Not feasible in this build (gz-sim 8.10, gz-rendering 8.2.2, Ogre-Next 2.3.3 Metal, DART)

| Wanted | Why not (evidence) | Instead |
|---|---|---|
| Wheels that sink dynamically, ruts, bulldozed berms | DART contacts are rigid. Per-contact CFM is not a stiffness: CFM 1 gave 20 mm at rest and 42 mm while driving; CFM ≥ 10 fell through the ground (M) | Static collision carve under sand (the wheel sits 2–3 cm into the visual surface), rolling resistance, bulldozing, slip compliance and a dig-in state that raises resistance, not sinkage (§6.5) |
| Friction on heightmaps or meshes from SDF `<friction>` | Ignored on both (M, today's `terrains.py`) | Per-contact callback (D1) |
| Mud sticking to wheels, adhesion | No adhesion in DART | Low-μ wet variant only (optional, Q4) |
| Tyre tracks as projected decals | Any `<projector>` in a world with a Terra heightmap aborts gz on Metal: the Terra pixel shader fails to compile (`decalsDiffuseTex` undeclared), also with stock media (M) | Deferred: a plugin-grown ribbon mesh (not prototyped) |
| Terra blending by slope or position; more than 4 textures | Terra weights come only from height (`smoothstep(min_height, …)`); the weight map is never used. The 5th-texture path needs an empty `<normal>`, which SDF rejects (M, R [26]) | All spatial variation baked into the layer-0 colour map; 3 global detail layers; optional local detail decals |
| SDF `<scene><fog>`, `<sky><cubemap_uri>` | Never applied by gz-sim 8; no fog API in gz-rendering 8 ogre2 (M, R [26]) | Shader patch: haze piece and procedural sky (D12) |
| Lens distortion; SDF camera noise at face value | `<distortion>` ignored (byte-identical output). Noise strongly attenuated: stddev 0.05 → 1.5 DN, 0.1 → 6 DN (M) | stddev 0.06 on the rover camera (~2 DN), or noise added in the station |
| Motion blur, exposure, bloom, SSAO | No API in gz-rendering 8 ogre2 (M, symbol scan) | — |
| Orthographic camera from SDF | SDF 1.11 has no `<projection_type>`; `<lens><type>orthographic` is ignored by a plain camera (M) | The FlyCamera plugin sets the projection (D16) |
| Shadows from high up or in orthographic views | No cast shadows beyond ~500 m camera distance (gone by 590 m), and none in orthographic projection (M) | Offline map tiles shot from 300 m; the station says so in orthographic mode |
| Camera frames while the world is paused | 0 frames in 3 s paused (M) | Inspect paused worlds in the Gazebo GUI |
| Horizontal FOV below 0.1 rad | SDF rejects the world (M) | Telephoto tiles at ≥ 0.1 rad |
| A 0.25 m DEM from the lidar | 53–58 % of 0.25 m cells have no ground return (M) | 0.5 m DEM + procedural detail below 0.5 m |
| Mesh terrain with tiled detail texture | gz PBS has one UV albedo and one normal map; its light map is the emissive slot, added not multiplied (R [26], M) | Terra near the rover; meshes only for the far field |
| Python gz-sim systems; Python `Joint.transmitted_wrench` | Python systems crash the in-process test fixture (README); `transmitted_wrench` raises TypeError (M) | C++ systems. The drivetrain publishes currents and torques on a topic that tests read. The C++ `JointTransmittedWrench` component is a candidate wheel-load source (G8) |
| `WheelSlip` system per terrain | One compliance per wheel, not per surface (M) | Per-contact slip compliance in the callback |
| A shrub mask from absolute NDVI | Absolute NDVI is negative almost everywhere in NAIP 2024 (M) | Darkness test filtered by the local NDVI anomaly (M: works, D10) |
| Mesh LOD for scattered clutter | gz-sim exposes no mesh LOD control (A: none found) | Static density fall-off away from mission areas; far shrubs baked into the colour map |
| Surface clast cover from SSURGO | The component surface-fragment table (`cosurffrags`) has no rows for any component of the seven local map units (M, rev2, Soil Data Access) | OSD rock-fragment ranges, NAIP texture and (A) labels (§5.3) |

## 5. Terrain model

### 5.1 One catalogue, one raster

`terrains.TYPES` stays the one catalogue of ground. Each `TerrainType` gains four groups of fields. Recipes are
small frozen dataclasses that types share by reference. WS-0 commits these dataclasses with today's behaviour as
defaults, so that WS-T1 and WS-A code against the same structure (§12):

```
traction:   mu_s, mu_k, crr, bulldoze, slip, sinkage_m, dig_rate, dig_max     (§5.6)
appearance: palette (base sRGB, p10/p90 spread), detail texture, dust factor and dust colour   (§5.7)
relief:     swatch key | None, swatch_sigma_m, amplitude, haystacks | None, rills (depth, spacing) | None   (§5.4)
clutter:    slabs (tabulated N(≥D), d_min, d_max), rocks, shrubs (per_ha, height range), risers   (§5.5)
```

Each world writes **`ground.png`**: 8-bit, one `TerrainType` index per heightmap sample, on exactly the grid of
`heightmap.png` (n × n, centred on the world origin, row 0 north, column 0 west). It also writes
**`ground.json`**: legend, traction table and collision map (§9.1). Together they are the single source for:
- drivetrain traction;
- colour-map baking;
- relief-transfer masks;
- clutter placement;
- the station's map legend;
- the mission sheet.

Zones (`features.Patch`, `Slope`, `Lane` surfaces, `WorldBuilder.zone`/`zone_rect`) paint into the raster instead
of creating tiles.

### 5.2 Worlds and their data

| World | Macro terrain | Below 0.5 m | Colour | Ground raster |
|---|---|---|---|---|
| `urc_autonomy` | `route_area_lidar_0p5m.tif` → 4097² over 2048 m (gate G1; fallback 2049²) | Slabs D < 1 m, risers, gravel by recipe (larger blocks are in the DEM); shrubs at their NAIP positions | NAIP 2024 orthophoto, de-shaded (§5.7), shrubs inpainted where 3D shrubs stand; saturation and contrast boost re-tuned on NAIP 2024 (the prototype used NAIP 2021) | SSURGO map units refined by slope (§5.3), then the mission's zones |
| `urc_astrobiology` | Default: today's layout at 2049² (0.5 m) + relief transfer (gate G7). Option (Q1): `mdrs_area_lidar2018_0p5m.tif` + `mdrs_area_naip2024.tif` at its real coordinates, with units re-placed on the real banded hills, wash and ledge | Recipes | Baked from the ground raster (default) or NAIP (option) | Paint rules + zones |
| `urc_delivery` | Today's layout at 2049² (gate G7) + relief transfer, still rougher with distance (rule 1.c.ii [13]) | Recipes | Baked | Paint rules + zones |
| `urc_equipment_servicing` | Today's flat site (rule: relatively flat), 1025² over 256 m (0.25 m), + a low-amplitude clay-crust/sand-sheet swatch (4 m RMS ≤ 2 cm; real clay crust p50 1.7 cm at 4 m (M, rev2)) | Gravel and sparse slabs off the approach | Baked | Paint rules + the gravel apron |
| `proving_ground` | Engineered stations unchanged. Three new "natural ground" strips (sand sheet, badland slope, block field) built from the same recipes, for calibration | As today + the strips | Baked | Lanes paint calibration types |
| `rover_test.sdf` | Unchanged hand-written world; solver per G2 | — | — | None (default surface) |

All 0.5 m files load with `urc.dem.read_geotiff` and cover every world's terrain (M, maps research). The lidar
files already carry the measured NAD83(2011)→WGS84 shift (−0.8925 m E, +0.6175 m N; residual ≤ 0.16 m (M)).
Any future conversion of USGS lidar must apply it. NAIP is 4-band; `urc.dem` gains a multi-band reader in WS-0,
because PIL silently drops the NIR band (M, critique E).

### 5.3 Painting the ground raster

**Synthetic worlds** declare paint rules as data next to their `FEATURES`, in order, later rules winning.
Features then override (sketch; names final in WS-T1):

```python
PAINT = [landscape.Base("regolith"),
         landscape.Noise("sand_sheet", feature_m=120, cover=0.4),       # tan sand-sheet patches on the plains
         landscape.Along(WASH, "wash_sand", half_width=6.0),          # wash floors
         landscape.Hills(HILLS, slope="badland_slope", floor="silt_flat", cap="caprock"),
         landscape.Steeper(30.0, "rock"),                             # faces too steep for soil
         landscape.Below("caprock", "block_field", reach_m=30.0)]     # talus aprons under rims
```

**Real-DEM worlds** map SSURGO map units [2] (polygons in `ssurgo_polys.geojson`, copied to `sim/data/soils/`) to
types, refined by slope. Slope is computed on the DEM smoothed with σ = 2 m. Thresholds are (A), tuned visually
against NAIP and `landforms_*.png`:

| SSURGO map unit (share of the Autonomy square (M, rev2)) | < 8° | 8–30° | > 30° |
|---|---|---|---|
| Sheppard-Leebench, sand sheet and fans (50 %) | `sand_sheet`; `sand` on dune crests (A) | `sand_sheet` | `rock` |
| Chipeta-Badland (26 %) | `clay_crust` (shale pediment; its A horizon is gravelly silty clay, 30 % > 2 mm [2], so a gravel detail layer) | `badland_slope` | `rock` |
| Badland-Rock outcrop (24 %) | `silt_flat` (floors) | `badland_slope` (popcorn crust) | `rock`, `caprock` on caps |
| Farb-Rock outcrop (none in the square; 1.8 % of the route area) | `slickrock` with `sand_sheet` pockets | `slickrock` + risers | `rock` |
| Green River-Myton (none in the square; 0.8 % of the route area) | `wash_sand` (channels), `gravel` (terraces: 23 % cobbles, 30 % stones [2]) | `gravel` | `rock` |

Painting the square with these rules gives `sand_sheet` 49.1 %, `clay_crust` 21.2 %, `silt_flat` 15.0 %,
`badland_slope` 11.9 % and `rock` 2.8 % (M, rev2).

**Leebench is not mapped to `fan_pavement`.** Its surface horizon in this map unit is fine sandy loam with 92.5 %
passing the No. 10 sieve; gravel starts at 28 cm [2]. The official series description gives a gravelly E horizon
(20 % gravel, 5–35 % range) with vesicular pores [2], which suggests some pavement but not ≥ 65 % clast cover.
SSURGO has no surface-fragment records here (M, rev2). Leebench areas therefore stay `sand_sheet`/`regolith`, with
5–20 % surface gravel as clutter and detail (A). `fan_pavement` remains in the catalogue for designed patches only
(A), until crew photos or a NAIP texture check settle it (Q14).

### 5.4 Relief

**(a) Heightmap scale (≥ 0.5 m).** Real-DEM worlds use the DEM as is: it already has the roughness, so no
swatches are added. Synthetic worlds compose

```
z = macro (today's features: noise, mesa, ridge, channel, ramps, strips, pads)
  + Σ_types  mask_t · amplitude_t · swatch_t        (real lidar residuals, variance-preserving blend)
  + haystacks (badland belts) + rills (badland and pediment slopes)
```

- *Swatches*: high-pass residuals (DEM minus a Gaussian of σ) of real 0.5 m lidar windows. They are laid in 64 m
  blocks with random offset, 90° rotation or mirror, and cosine-feathered over 16 m.
- *Variance-preserving blend*: where independent residual fields overlap with weights wᵢ, the sum is divided by
  sqrt(Σwᵢ²). Plain feathering scales the RMS by sqrt(w² + (1 − w)²), down to 0.71 at the midpoint, which biases
  about 40 % of the area low (computed, critique P).
- *Mask*: the feathered type mask × a keep-flat mask. The keep-flat mask is 0, eased over 5 m, on:
  - pads (C2, objects, posts);
  - graded ramps and strips;
  - proving-ground lanes and steps;
  - ±3 m around judges' easy routes.

  Designed grades must survive (tests in §11).
- *Haystacks* (new `Heightfield` op): rounded knobs 10–40 m across, with flanks up to 35–40° [9] (T: measured in
  Mancos Shale near Hanksville, applied to MDRS's Brushy Basin badlands). About half of a badland belt stays flat
  floor. About 6 % of a belt's area lies over 30° (M: Badland-Rock outcrop unit, 5.9 % over 30° and 47 % under 5°,
  at 2 m slope).
- *Rills* (new op): gradient-descent traces from seeds every 10–30 m on badland and pediment slopes, carving
  5–20 cm deep and 0.5–1 m wide, deepening downslope.
  - Depth evidence: the lidar shows −5 to −20 cm rill lines (M); rills average 7.9 cm near Caineville [8] (T, Mancos Shale).
  - Implementation: vectorised over seeds, with no per-cell Python loop (the env has no scipy or numba).
- Badland swatches use σ = 2 m, so they carry only isotropic bumps. Their rills come from the rill op, which runs
  downslope on the actual synthetic slope (a transferred real rill would point the wrong way).

Swatch sources (window centres computed from `windows_def`; all inside the two 0.5 m DEMs):

| Swatch | Lidar window (size) | Centre lat, lon | DEM file | σ |
|---|---|---|---|---|
| `sand_sheet` | sand_sheet_D (125 × 200 m), sand_sheet_C (225 × 140 m) | 38.40881, −110.76886; 38.41261, −110.77836 | `route_area_lidar_0p5m.tif` | 16 m |
| `shale_pediment` | shale_pediment_A (300 m) | 38.40796, −110.79770 | `mdrs_area_lidar2018_0p5m.tif` | 16 m |
| `badland` | badland_banded_B (300 m) | 38.41336, −110.79310 | `mdrs_area_lidar2018_0p5m.tif` | 2 m |
| `block_field` | boulder_field_I (128 × 225 m), boulder_ridge_J (150 × 188 m) | 38.40604, −110.77066; 38.40212, −110.77911 | I: `route_area_lidar_0p5m.tif` (it pokes ~6 m past the MDRS file's east edge); J: `mdrs_area_lidar2018_0p5m.tif` (south of the route file) | 8 m |
| `slickrock` | slickrock_ledges_H (300 m) | 38.41015, −110.76620 | `route_area_lidar_0p5m.tif` | 8 m |

They are stored as `sim/data/relief/<swatch>.npz` (float16 metres at 0.5 m, with provenance and their own RMS
table; 0.1–0.7 MB each). `sim/tools/make_relief_swatches.py` makes them reproducibly from the DEMs.

**Targets.** All targets use plane-detrended RMS height in non-overlapping square windows, in cm. The 2 m scale is
dropped. Lidar point noise is 0.8–1.5 cm (M), and 2 m values are noise-dominated, not lower bounds.

*World level*: the real 0.5 m lidar over the Autonomy 2048 m square, p25/p50/p75/p90 (M, critique P; world values
re-run rev2):

| Window | 4 m | 8 m | 16 m | 32 m |
|---|---|---|---|---|
| Real, whole square | 1.6 / 2.7 / 4.9 / 9.1 | 3.5 / 5.8 / 11.6 / 23.0 | 6.4 / 11.5 / 26.5 / 52.8 | 12.2 / 23.8 / 55.1 / 114.4 |
| Real, central 600 m | 1.6 / 2.3 / 3.5 / 5.4 | 3.0 / 4.2 / 6.3 / 11.2 | 4.8 / 6.3 / 11.1 / 21.0 | 7.3 / 10.7 / 20.3 / 38.0 |
| *Today*, central 600 m, p50 / p90: Delivery | 0.84 / 2.9 | 2.7 / 15.0 | 7.5 / 62.4 | — |
| *Today*: Astrobiology | 0.64 / 1.2 | 1.7 / 3.2 | 4.1 / 38.4 | — |
| *Today*: Autonomy (3DEP 1 m) | 1.29 / 3.4 | 3.1 / 9.4 | 5.4 / 19.9 | — |

Revision 1's headline "2–10× too smooth" compared world medians with hand-picked rough landform windows.
Comparing like with like, the deficit is at 8 m and below. The rough tail is already present (Delivery's p90 at
16 m is 62 cm, against 53 cm real).

*Per type*: the real Autonomy square painted by the §5.3 rules. Windows count only if at least 80 % of their
samples carry one type. p25 / p50 / p75 (M, rev2, `types_rms.json`):

| Type | 4 m | 8 m | 16 m | 32 m |
|---|---|---|---|---|
| `sand_sheet` | 1.6 / 2.6 / 4.3 | 3.2 / 5.1 / 8.5 | 5.6 / 8.8 / 16.7 | 9.9 / 17.3 / 34.0 |
| `clay_crust` (shale pediment) | 1.0 / 1.7 / 2.8 | 2.4 / 3.9 / 6.2 | 5.1 / 8.0 / 12.4 | 10.1 / 15.2 / 23.6 |
| `silt_flat` (badland floors) | 1.5 / 2.4 / 3.8 | 3.6 / 5.4 / 8.3 | 7.1 / 11.0 / 17.0 | 13.0 / 20.4 / 30.7 |
| `badland_slope` | 4.3 / 6.6 / 10.3 | 12.5 / 18.3 / 26.3 | 33 / 47 / 63 | 84 / 121 / 146 |
| `rock` (> 30°) | 7.2 / 12.8 / 33.9 | 17.5 / 31 / 65 | 40 / 72 / 131 | — |
| `block_field`, `slickrock` (provisional, single landform windows with mixed slopes (M)) | 8.5–10; 6 | 20–23; 15 | 43–50; 37 | — |

WS-T1 re-derives these with its final rules in `sim/tools/terrain_targets.py`. That tool paints both 0.5 m DEMs
(the MDRS-area file contains Farb slickrock and block fields) and writes
`sim/data/research/terrain_targets.json`, which the tests read.

**(b) Discrete features, 0.1–8 m**: §5.5.

**(c) Below 0.1 m**: visual only. This covers detail textures and normal maps for:
- popcorn crust, ~3.8 cm relief (T: Mancos Shale, Caineville [8]) and 1–5 cm thick (A);
- wind ripples (λ ≈ 8 cm; λ/H 15–18 [12]);
- pavement, cracked silt and slab joints;
- pebble meshes near routes.

The *feel* of this scale comes from the drivetrain: μ noise (A), stick-slip on firm ground, and dig-in on loose
ground (§6.4–6.5).

### 5.5 Discrete clutter recipes

All placements go through the existing rock machinery: merged 128 m chunks in the terrain link, and `keep_clear`
of routes and placed models.

| Clutter | Ground types | Density and sizes | Shape | Collides | Basis |
|---|---|---|---|---|---|
| **Slabs** (tabular blocks) | `block_field`; talus under `caprock` rims (slopes 22–38°); badland edges (~3.6 % cover at MDRS) | Tabulated cumulative counts per 100 m², interpolated log-log, at 1–7 m: N(≥1) 1.44–2.04, N(≥1.5) 0.99–1.30, N(≥2) 0.69–0.94, N(≥3) 0.40–0.47, N(≥4) 0.23–0.24, N(≥5) 0.13–0.16, N(≥7) 0.07 (windows J–I). Cover by 1–7 m blocks 8.0–9.2 %. Below 1 m: extrapolated with the local exponent at 1–2 m, b ≈ 1.1, down to 0.3 m (A, unmeasured: the top-hat resolves ≥ 1 m only). Real-DEM worlds: D 0.15–1 m only | `meshes.slab`: irregular 6–10-gon outline extruded, height 0.4 D × U(0.7, 1.3), chamfered, tilted 0–30° on the slope (A), buried 15 % | D ≥ 0.15 m | M: DSM top-hat counts [3], H/D ≈ 0.4. The curve is not one power law: local b ≈ 1.1 at 1–2 m, 1.5–2.0 at 2–4 m, 2.1–2.2 at 4–7 m (M). Objects ≥ 7 m (6.5–7.6 % of the area) are knob crests and ledge segments: macro shape or risers, not slabs. The research JSON's `golombek_k_fit` = 0.6 in 8 of 9 windows is a clamped fit; it is not used |
| **Risers** (ledges) | `slickrock`, `caprock` edges, wash cutbanks (0.3–2.4 m (M)) | 0.1–1 m risers along contours, in 20–40 m segments every 10–30 m (A: below lidar resolution). Steps ≥ 1 m are in the DEM (real worlds; risers are not placed where the DEM already has a step) or the macro shape (median 1.7–2.2 m, p90 3.5–8 m (M)) | Mesh strip (vertical, slightly irregular face, 1–3 m deep, buried) | Yes | M (ledges 22–46/ha ≥ 1 m) + A |
| **Rocks, cobbles** | `gravel`, `fan_pavement`, `wash_sand`, talus; sparse on Leebench (§5.3) and Chipeta | 0.04–0.3 m; Golombek k 0.05–0.2 (A: no local data) | Existing rock variants | ≥ 0.1 m (as today) | A; Myton 23 % cobbles [2]; Chipeta A horizon 30 % gravel [2] |
| **Shrubs** | `sand_sheet`; wash margins | Density from NAIP dark spots that pass the NDVI-anomaly test, measured per painted type by WS-A. Route area: 38–92/ha before the test, depending on the darkness threshold; 53–68 % of dark-spot pixels pass it (M, critique P). Sand sheet: height ≤ 0.3 m (M: lidar vegetation > 25 cm covers 0.006 %); diameter 0.4–1.0 m (A: NAIP's 0.6 m pixels cannot resolve it). Wash margins: 0.5–2.5 m tall, ~7 % cover (M, lidar). Autonomy: detected positions | Merged GLB per chunk, low-poly; alpha-cut branch cards if gate G4 passes | No (the rover drives through brush, as today) | M |
| **Pebbles** | Within 35 m of starts and routes | ~20,000 (M: +130 MB, frame cost within noise) | GLB | No | M |

Sampling: D is drawn by inverting the tabulated cumulative distribution (log-log interpolation; the power-law
extrapolation N = N(≥1)·D^−1.1 below 1 m). The expected count over S m² is N(≥D_min)·S/100. Colour: 70 %
varnished (dark tops (105–136, 100–116, 101–116) (M)) and 30 % fresh tan faces (10YR 6/3, (169, 145, 115)
[2][28]); the split is (A).

### 5.6 Traction

The drivetrain uses μ as the Coulomb limit of the *gross* tyre force. It applies rolling resistance and bulldozing
as separate hub forces (§6.5).
- The *net* traction on flat ground is μk − Crr, so a rover climbs up to atan(μk − Crr).
- A parked rover with stopped wheels holds up to atan(μs), independent of heading (§6.4).
- The terrain research's "effective μ" values (sand 0.35–0.45, moist clay 0.26) are net values.

Where Bekker numbers exist, gross μk = 0.85 × (DP/W + Crr), using the Bekker calculation for our wheel [5]. The
0.85 is a small-wheel derating (A). Its direction is supported by Meirion-Griffith and Spenko: for wheel diameters
below ~0.5 m, Bekker's flat-plate assumption fails and sinkage is underpredicted [33]. Bekker DP/W is already net
of compaction resistance.

Bekker for our wheel (W 113 N, D 0.30 m, b 0.10 m), with [5] parameters (M calculation, reproduced by critique P):

| Soil | Sinkage | Crr | DP/W |
|---|---|---|---|
| Dry sand | 21.3 mm | 0.20 | 0.41–0.45 (the research JSON gives 0.45; the critique's recomputation with contact length 8.1 cm gives 0.41) |
| Sandy loam | 5.0 mm | 0.10 | 0.51 |
| Moist clayey soil | 3.0 mm | 0.08 | 0.26 |

Car-tyre rolling-resistance coefficients [6] bracket the firm surfaces:

| Surface (car tyres) | Crr |
|---|---|
| Concrete | 0.01–0.015 |
| Rolled new gravel | 0.02 |
| Solid sand, loose worn gravel, medium-hard soil | 0.04–0.08 |
| Loose sand | 0.2–0.4 |

These are larger wheels than ours, so they are lower bounds.

Columns of the table below:
- **Bulldoze** k_b: the lateral hub-force coefficient (§6.5).
- **Spin**: the fresh spin-in-place ratio at wz = 1 rad/s. It is the root r of the moment balance
  μk·(c·sx − a·sy)/|s| = Crr·c + k_b·a, with sx = c(1 − r) and sy = a·r. It does not depend on the load split
  (M, rev2, `table.py`).
- **Dug**: the same ratio at D = D_max.
- **Dig**: dig_rate (m of extra sinkage per m of slip) and D_max (total sinkage / static sinkage).

| Key | Ground (soil, landform) | μs | μk | Crr | Bulldoze | Net, climb | Hold | Slip | Sinkage | Dig | Spin (dug) | Dust | Basis |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `sand` | Loose sand: dunes, sand patches (Sheppard) | 0.52 | 0.52 | 0.20 | 0.06 | 0.32, 17.7° | 27.5° | 1.0 | 0.02 m | 0.01, 1.25 | 0.261 (0.188) | 0.8 | Bekker dry sand [5] with derating: 0.85 × (0.41 + 0.20); loose sand has no peak [34]; Crr [5][6]; bulldozing (A), between Rankine passive pressure (0.006) and Bekker's bulldozing formula (0.11) at 2 cm sinkage (computed, Sheppard bulk density 1.55 g/cm³ [2]); slip (A), Janosi bracket below; dig (A, mild; [36] gives 3–7× at slip 0.6) |
| `wash_sand` | Wash floors (Riverwash: 98 % sand in the top 15 cm [2]) | 0.52 | 0.52 | 0.25 | 0.10 | 0.27, 15.1° | 27.5° | 1.2 | 0.03 m | 0.012, 1.15 | 0.182 (0.108) | 0.6 | Looser than `sand` (A); URC teams stuck in sand [14][32] |
| `sand_sheet` | Crusted sand sheet between shrubs (Sheppard, crust interspaces) | 0.60 | 0.55 | 0.15 | 0.03 | 0.40, 21.8° | 31.0° | 0.6 | 0.015 m | 0.005, 1.25 | 0.328 (0.291) | 0.6 | Between sand and loam (A); a crust gives a small peak (A, [34]) |
| `regolith` | Packed sandy loam: the default | 0.62 | 0.52 | 0.10 | 0 | 0.42, 22.8° | 31.8° | 0.3 | 0.005 m | — | 0.377 | 0.4 | Bekker sandy loam [5]: 0.85 × (0.51 + 0.10); μs/μk (A) |
| `fan_pavement` | Gravel lag, designed patches only (§5.3) | 0.75 | 0.62 | 0.03 | 0 | 0.59, 30.5° | 36.9° | 0.1 | 0 | — | 0.425 | 0.3 | Crr between rolled gravel 0.02 and worn gravel 0.04 [6]; μ (A) |
| `gravel` | Loose gravel, terraces | 0.62 | 0.52 | 0.05 | 0 | 0.47, 25.2° | 31.8° | 0.3 | 0.005 m | — | 0.410 | 0.3 | Crr loose worn gravel 0.04–0.08 [6]; μ (A) |
| `scree` | Loose debris on steep slopes (URC 1.c.ii [13]) | 0.46 | 0.40 | 0.06 | 0 | 0.34, 18.8° | 24.7° | 0.4 | 0 | — | 0.392 | 0.4 | (A): a parked rover slides above ~25° |
| `clay_crust` | Dry shale-pediment crust (Chipeta gravelly silty clay) | 0.78 | 0.65 | 0.08 | 0 | 0.57, 29.7° | 38.0° | 0.1 | 0 | — | 0.401 | 0.5 | Crr medium-hard soil 0.04–0.08 [6]; dry crust μ (A) |
| `badland_slope` | Bentonite popcorn crust on badland slopes | 0.60 | 0.50 | 0.10 | 0 | 0.40, 21.8° | 31.0° | 0.3 | 0.01 m | — | 0.375 | 0.7 | Loose granules over a firm crust (A, [10]) |
| `clay` | Dry pulverised bentonite powder ("dusty clay", today's CLAY) | 0.45 | 0.45 | 0.15 | 0.06 | 0.30, 16.7° | 24.2° | 0.5 | 0.02 m | 0.008, 1.25 | 0.273 (0.203) | 1.0 | A loose, fine granular layer: no peak [34]. Higher μ than revision 1's 0.32, which came from *moist* clay [5]. 4.8 cm pulverised mantle on disturbed slopes [8] (T, Mancos). μ, Crr, sinkage (A) |
| `silt_flat` | Silt/clay flats (Billings, Hanksville) | 0.74 | 0.62 | 0.06 | 0 | 0.56, 29.2° | 36.5° | 0.1 | 0 | — | 0.409 | 0.9 | Crr 0.04–0.08 [6]; μ (A) |
| `slickrock`, `caprock`, `rock` | Bare sandstone; rocks, slabs, risers, steps (by collision map) | 1.00 | 0.85 | 0.015 | 0 | 0.835, 39.9° | 45.0° | 0.05 | 0 | — | 0.436 | 0.1 | Rubber on rough sandstone 0.8–1.0 (A); Crr concrete 0.01–0.015 [6] |
| `biocrust` | Biological soil crust | 0.62 | 0.52 | 0.12 | 0 | 0.40, 21.8° | 31.8° | 0.3 | 0.005 m | — | 0.364 | 0.2 | As loam; pinnacles (≤ 10 cm [11]) crush (A) |
| `gypsum` | Gypsum crust | 0.72 | 0.60 | 0.06 | 0 | 0.54, 28.4° | 35.8° | 0.1 | 0 | — | 0.408 | 0.6 | (A) |
| `manmade` | Objects whose SDF sets no friction (landing pad, C2 pad, lander) | 0.80 | 0.70 | 0.015 | 0 | 0.685, 34.4° | 38.7° | 0.05 | 0 | — | 0.434 | 0 | Rubber on dry concrete μs 1.0 / μk 0.7 [23], lowered for painted wood/plastic (A) |
| `test_muXXX` | Calibration lanes (`calibration_surface(mu)`) | μ | μ | 0 | 0 | μ | atan μ | 0 | 0 | — | 0.441 | 0 | Exact calibration |

Objects whose SDF sets `<surface><friction>` use that μ as μs = μk, with Crr 0.015 and slip 0.05.

*Slip* is the force-dependent slip parameter: in steady driving, slip ratio = slip × F/N (§6.4). For loose sand, a
Janosi–Hanamoto shear curve [5] gives a bracket. With contact length 8.1 cm and shear modulus K 1–2.5 cm (A), it
gives 11–31 % slip at the thrust needed on flat sand (H/W = 0.2) (M, rev2), so slip = 1.0 (20 %) lies inside it.
For firm soils, the same curve with Bekker contact lengths (3.9 cm on loam) predicts 12–29 % slip at H/W = 0.1.
That is implausible for a tyre, whose deflected contact is longer, so firm-soil slip values stay (A).

Visual-only types today (`mudstone`, `bentonite`, `pavement`) map onto `badland_slope`, `badland_slope` and
`fan_pavement` traction. The catalogue test "types never repeat" changes from unique μ to unique keys and colours.
Expect mission behaviour to change: the default ground now climbs 23°, not 45°, and spin rates fall to 0.18–0.44
of the command (§13, R1).

*Wet variant* (A, optional, Q4): `clay_crust`, `badland_slope`, `clay` and `silt_flat` → μk 0.15–0.30 and
Crr +0.05. URC allows light rain (rule 3.d.iii [13]), and crews report impassable mud [31][10].

### 5.7 Appearance

- **Colour map** per world: 4096² over the terrain (0.5 m per texel at 2 km, 0.25 m at 1 km, 6.25 cm at 256 m).
  It is Terra layer 0, with a flat 16² normal map and `<size>` = terrain size (D11).
- **Autonomy**: NAIP 2024 resampled to the world frame, then:
  1. **De-shaded**. Fit NAIP luminance per SSURGO unit as ρ̄·(k + cos i), where i is the illumination angle of a
     fitted NAIP sun (azimuth and elevation by least squares) on the lidar DEM resampled to 0.6 m. Then divide by
     (k + cos i)/(k + cos i_flat), a C-correction [41].
  2. **Cast shadows** under the fitted sun are found by ray-marching the lidar, and inpainted (cv2 Telea).
  3. **Shrubs** are inpainted where 3D shrubs stand.
  4. **Saturation and contrast** are boosted in linear light (×1.25 and ×1.1 in the prototype on NAIP 2021;
     re-tuned on NAIP 2024).

  NAIP is hazy and pale (median #dfc8b0 (M)). The test: after de-shading, |corr(luminance, cos i)| < 0.1 on
  slopes > 10°.
- **Synthetic worlds**, baked from the ground raster:
  - a per-type palette, with hue and saturation from the soil survey's Munsell colours (via the RIT renotation
    [28]) and lightness and p10–p90 spread from NAIP medians, saturated ×1.25 like the orthophoto;
  - badland strata coloured by a gently dipping elevation ramp (≤ 3.4°) plus noise (`strata_colour_ramps.json`:
    elevation explains 33–70 % of redness on banded mounds (M));
  - slope darkening (NAIP mean 221/208/193 at 0–2° → 212/190/176 at 30–45° (M), kept as a mild material effect
    of 2–4 %; most of NAIP's slope darkening is illumination, which Terra already renders);
  - cavity shading from the heightfield;
  - brighter washes;
  - far shrubs as dark dots (1.7–2.3 % cover (M)).

| Ground | Hue/saturation (Munsell → sRGB) | NAIP median (lightness) |
|---|---|---|
| Sand, sand sheet | 5YR 6/6 → (194, 137, 95) | tan (197, 186, 168); Sheppard #e2ccb3 |
| Shale pediment, clay crust | 2.5Y 6/2 → (160, 148, 124) | grey (199, 196, 189); Chipeta #d9d2c4 |
| Badland bands | maroon 10R–5YR (A) | maroon (181, 157, 149), p10–p90 (160, 138, 135)–(195, 170, 161); white (215, 212, 206) |
| Fan pavement | 10YR 7/3 → (196, 172, 140) | — |
| Slickrock, fresh sandstone | 10YR 6/3–6/4 → (169, 145, 115)–(174, 144, 105) | Farb #d9c0a4 |
| Varnished rock | — | (127, 111, 112) |
| Silt flat | 5Y 4/2 → (105, 97, 73) (Hanksville) | Billings #ded6c9 |
| Gypsum | — | (215, 212, 206) |
| Biocrust | (72, 60, 50) (today's value, kept (A)) | — |

- **Detail layers** (shared in `urc_media`, tileable 2048²):
  - gravel lag at 3.3 m tile and weight ≈ 0.35;
  - cracked silt at 7.7 m and ≈ 0.15;
  - slab joints at 5.1 m above cap heights.

  Constant partial weights come from a huge fade (min −755, fade 2000 gives 0.32–0.38 (M)). Per-texel
  compensation is O' = (O − Σ aᵢmᵢ)/a₀ in linear light (M: 0.1 DN). Normal maps are 3× stronger, because the
  weights scale them; ~1.2 % of texels clip (M). Port `render/prototype/{detail_tex.py, terra_mix.py}`, and add
  rippled-sand and popcorn-crust textures.
- **Decals** no longer carry a zone's colour (the colour map does, with soft edges). They remain only as optional
  detail decals (sand ripples, popcorn) on large zones, and only if gate G4 shows feathered alpha edges work.

### 5.8 Building blocks: what is reused, extended, new, removed

The "Today" column names the code as the previous workflow left it, re-read by WS-0 on 2026-10-06. Where that
code differs from what revision 2 assumed, the cell says so (*code:*).

| Today | After |
|---|---|
| `terrains.TYPES`, `TerrainType`, textures shared via `urc_media` (`TerrainType.texture(media)`, `.normal(media)`) | Extended (traction, palette, recipes). *Done by WS-0:* frozen `Traction`, `Palette`, `Appearance`, `Relief`, `Clutter` and the shared recipes `Haystacks`, `Rills`, `Slabs`, `Rocks`, `Shrubs`, `Risers`; `TerrainType` fields `traction` (default `Traction.coulomb(mu)`), `appearance` (default the texture colour, no detail, no dust), `relief`, `clutter` (default none). `Traction`'s field names are `ground.json`'s keys (§9.1). `calibration_surface(mu)` stays for lanes |
| `terrains.Zone`, `blob`, `rect`, `inside` | Kept: their outlines paint `ground.png` |
| `terrains.fit_tiles`, `Tile`, `top_height`, `tops` (vectorised), tile constants | Behind `FRICTION = "tiles"` in wave 1, where tile collisions map to their zone's type in `ground.json`; deleted in wave 2 after the drivetrain gate |
| `terrains.carve_depth`. *Code: there is no carve* (no function, no carved collision heightmap) | New in WS-T1 (nothing to reuse): cut = the type's `sinkage_m` under its samples, eased over 0.5–1 m. World z = 0 drops to the deepest sinkage below the lowest point |
| `world._write_terrain`. *Code: one `heightmap.png` serves collision and visual*, normalised to its own maximum (the sheet's `terrain.z_max`), plus `dem.tif`; no second PNG and no `ValueError` | Once sinkage carves the collision surface, each PNG is normalised to its own maximum with its own `<size>` z, so carving anywhere, including the global maximum, is exact. A test of decoded heights (carved ≤ original; equal outside carved types within one 16-bit step) extends today's `test_urc_terrain.SmallWorld.test_heightmap_is_the_terrain`. The sheet records both z ranges |
| `WorldBuilder.place` | Sets objects on the collision surface (carved height), not the visual one |
| `features.Patch(level=True)` | `level` defaults to False (nothing needs planar tiles); lanes keep exact planes via `Lane.shape` |
| `features.Patch`, `along`, `Wash`, `Mesa`, `Slope`, `Surface`, `Lane`, `Washboard`, `AlternatingBumps`, `TwistDitch`, `Step`, `Ledge`; `features.shape(hf, features)`, `features.dress(w, features)` | Same API; `Surface` paints the raster. `Step`/`Ledge` collisions (`<key>_collision`, from `WorldBuilder.block`) go into the `ground.json` collision map as `rock` |
| `WorldBuilder.zone/zone_rect` | Paint + optional detail decal + sinkage; no collisions |
| `WorldBuilder.ground()` | Heightmap minus sinkage (no tiles) |
| `WorldBuilder.rock_field`, `terrains.ROCKS` palettes; *code: `_write_rocks` is now `_write_clutter`* (rocks and shrubs, one mesh per `ROCK_CHUNK` = 128 m square: visuals `rocks_<i>_<j>_c<k>` and `shrubs_<i>_<j>_c<k>`, collisions `rocks_<i>_<j>`, written by `_write_merged`) | Generalised to clutter kinds: `rock`, `slab`, `riser` (collision + visual); `shrub`, `pebble` (visual GLB). Palettes + `varnish`, `fresh_sandstone` |
| `WorldBuilder.scatter/scatter_each/scatter_points/points_along/rock_garden/keep_clear` | Unchanged; `landscape.place(recipes)` produces placements for them |
| `WorldBuilder.shrubs`. *Code: already merged visual-only OBJ meshes* per 128 m chunk (`meshes.shrub(variant)`, colours `terrains.SHRUBS`); `props.shrub` (one model per shrub) is unused | GLB chunk meshes with recipe densities (§5.5); `props.shrub` deleted (WS-T2) |
| `WorldBuilder.terrain(layers)`, `Layer` | `terrain(colour_map, details)`; `Layer` kept as fallback |
| `terrain.Heightfield` ops | + `detail(swatch, mask, amplitude)` (variance-preserving), `haystacks(...)`, `rills(...)` |
| `dem.read_geotiff`, `dem.to_heightfield` | Used as is for the 0.5 m DEMs. *Done by WS-0:* `dem.read_raster(path) -> (bands (B, H, W) float32 in file band order, GDAL geotransform)` for 1-, 3- and 4-band GeoTIFFs (pixels through OpenCV, which keeps NAIP's NIR band; a band marked alpha is refused, as OpenCV would premultiply by it); `read_geotiff` uses it |
| `sheet.py` (the one sheet reader: `load`, `path`, `find`, `terrain`, `radio_los`) | + `ground(x, y)` → type key, paths of `ground.png` and the colour map |
| `media.Media` (in `sim/urc/media.py`, kinds `.png` and `.obj`) | + a `glb` kind (so `prune()` handles it); slab variants, shrub meshes, detail textures, dust puff texture; far-field texture shared across worlds |
| `meshes.py` (`write_obj`, `rock`, `rock_variant`, `rock_base`, `tilt`, `shrub`, `hull_faces`, `combine`, `quad`, `blob_outline`, `drape`, `drape_rect`) | + `slab()`, `riser_strip()`, `write_glb()` (Z-up: gz does not rotate glTF Y-up (M)) |
| `gen_model.py` viewer cameras (`ChaseParams`, `EyeParams`, `_viewer_sdf`, `build_chase_sdf`, `build_eye_sdf`, viewer topics) | *Done by WS-0:* in `sim/viewers.py`, outputs byte-identical. `gen_model.main()` calls `viewers.write_all(models_dir)`; `gen_model` re-exports the names in `gen_model.VIEWER_NAMES` lazily (module `__getattr__`, because `viewers` imports `gen_model.Params`). The camera helper is `sdf.camera(sensor, hfov, size, clip, image_format=None)` |
| The Gazebo environment, set in `run.sh`, `station/__main__.py` and `tests/simulate.py` separately | *Done by WS-0:* `sim/gzenv.py`, `environment(build_dir=None, *, partition=None, ip=None, base=None)`; `run.sh` evals `python sim/gzenv.py`. It sets `GZ_RENDERING_RESOURCE_PATH` to `<build>/gz-rendering-media` only when that directory holds the marker `gzenv.MEDIA_COMPLETE` (`.complete`), which the media tool writes last |
| `tests/simulate.py` (`simulate`, `world_sdf`) | *Done by WS-0:* `simulate(..., rover_uri=, params=, default_surface=, solver=)`, `cmd` a twist or a schedule `[(t, vx, wz)]`; `world_sdf(..., rover_uri, default_surface, solver)`; `variant_sdf`, `rover_model(params)`, `world_file`; `cpu_time_per_step(worlds, iterations, runs, schedule)` → `Cost` (CPU and wall time per step: `real_time_factor` by §10.3's CPU-time method, `wall_real_time_factor` by wall clock, 13–29 % higher on an idle machine because CPU time also counts gz's helper threads; gate G5), `gz_run`, `twist_publisher(worlds, schedule)` (sim time from the 10 Hz `/world/<name>/stats`: `/clock` is sent every step and would add to the cost measured). `default_surface` puts the type's Coulomb μ on the flat ground's SDF; how the physical drivetrain resolves that ground is WS-D's |
| No ECM heightmap lookup in the plugins | *Done by WS-0:* `sim/plugins/terrain_heightmap.hh`: `rover_sim::FindTerrainHeightmap(ecm, HeightmapGeometry::kVisual or kCollision)` → `TerrainHeightmap` (`path`, `size`, `origin`, `samples`, `heights`; `Height(x, y)` bilinear, −∞ outside; `Nearest(x, y, row, col)`), `ResolveUri(uri)` |

New modules:
- `sim/urc/landscape.py`: paint rules, SSURGO import, relief transfer, haystacks and rills, clutter recipes → placements.
- `sim/urc/appearance.py`: colour maps; orthophoto path with de-shading; shrub detection with the NDVI-anomaly filter; inpainting; Terra layers and compensation.
- `sim/urc/farfield.py`.
- `sim/urc/lighting.py`: sun from date and time; light colours.
- `sim/viewers.py`.
- `sim/gzenv.py`: one environment helper for `run.sh`, the station, `simulate.py`, `render_map.py` and the camera subprocess tests.

Sheet additions: `terrain.ground_map`, `terrain.ground_legend`, `terrain.colour_map`, `terrain.sources` (DEM and
imagery provenance).

## 6. Drivetrain and contact model

### 6.1 Why turning in place is smooth today (M)

- DiffDrive turns each wheel into a velocity servo. Wheel speed is exact (2.667 rad/s at wz = 1); torque hits the
  30 N·m cap only for ~15 ms.
- One Coulomb μ, no static/kinetic difference, rigid contacts, box friction, no rolling resistance (straight
  driving needs 2e-5 N·m). Steady yaw-rate std 0.0000 rad/s; speed std 3e-13.
- Turn rate is 0.889 of the command (0.997 with PGS), because the tyre has μ 0.5 across and 1.0 along. Isotropic
  contact gives 0.441. On tiles with μ ≤ 0.5 the anisotropy vanishes, and the rover cannot turn at all (D1).

### 6.2 The system and its SDF

`RoverDrivetrain` (`sim/plugins/rover_drivetrain.cpp`, one target in `sim/CMakeLists.txt`) replaces DiffDrive in the
rover model when `DriveParams.mode = "physical"` (D22). `gen_model.py` writes, from a new `DriveParams` block:

```xml
<plugin filename="RoverDrivetrain" name="rover_sim::RoverDrivetrain">
  <topic>/model/rover/cmd_vel</topic>          <cmd_timeout>0</cmd_timeout>   <!-- 0 = hold the last command, like DiffDrive -->
  <odom_topic>/model/rover/odometry</odom_topic> <tf_topic>/model/rover/tf</tf_topic>
  <frame_id>odom</frame_id> <child_frame_id>base_link</child_frame_id> <odom_publish_frequency>50</odom_publish_frequency>
  <state_topic>/model/rover/drivetrain</state_topic> <state_rate>50</state_rate>
  <track>0.8</track> <radius>0.15</radius> <track_multiplier>1.0</track_multiplier>
  <wheel><joint>wheel_fl_joint</joint><link>wheel_fl</link><side>left</side></wheel>   <!-- fl, rl, fr, rr -->
  <dust><wheel>wheel_rl</wheel><topic>/model/rover/link/rocker_left/particle_emitter/dust_rl/cmd</topic></dust>
  <motor><voltage>24</voltage><resistance>0.46</resistance><kt>0.0445</kt><ke>0.0445</ke><gear>50</gear>
         <efficiency>0.8</efficiency><rotor_inertia>1.2e-5</rotor_inertia><free_current>1.0</free_current>
         <output_friction>0.05</output_friction><current_limit>20</current_limit></motor>
  <driveline><stiffness>1500</stiffness><damping>2</damping><backlash>0.026</backlash></driveline>
  <controller><kp>4</kp><ki>40</ki><speed_filter>0.005</speed_filter><accel>8</accel><max_speed>10</max_speed>
              <substeps>4</substeps></controller>
  <contact><v_stribeck>0.03</v_stribeck><v_align>0.005</v_align><perp_ratio>0</perp_ratio>
           <stick_perp_ratio>0.3</stick_perp_ratio>
           <mu_noise>0.2</mu_noise><mu_noise_length>0.3</mu_noise_length><rr_w0>0.2</rr_w0>
           <dig>true</dig><dig_heal_length>0.3</dig_heal_length>
           <load_source>contact</load_source>   <!-- contact | joint, decided by gate G8 -->
           <default_surface>regolith</default_surface><object_surface>manmade</object_surface></contact>
</plugin>
```

Other model changes:
- The wheel joints' `<effort>` rises to 1000, so DART never clamps the applied torque.
- The wheel joints' `<velocity>` limit rises to 16 rad/s, 1.5 × the motor's free speed of 10.8 rad/s, so it can
  never act as a hidden brake. (R: whether DART enforces it on a torque-driven joint is unverified.) The
  controller clamps at `max_speed` = `Params.wheel_speed` = 10 rad/s, and the station keeps deriving `MAX_LINEAR`
  from `Params.wheel_speed` × `Params.wheel_radius`.
- The tyre collision's friction becomes isotropic μ 1.0 (no `fdir1`). The plugin overrides every wheel contact.
- The dust emitters are declared with `<emitting>false</emitting>`, `<topic>` and `<particle_scatter_ratio>0`.
- DiffDrive must be removed, not kept alongside (D22).

In wave 1, all of these changes exist only in the `physical` variant. At the start of wave 2, WS-T2 makes the
non-drivetrain parts unconditional: camera noise, RGB and depth clip (§7), dust emitters.

### 6.3 Motor, gearbox and controller (port of the prototype, per wheel, every 1 ms step)

```
setpoint      ω* = (vx ∓ wz·track·track_multiplier/2) / radius   (left −, right +), clamped to ±max_speed
ramp          sp = gz::math::SpeedLimiter (max acceleration accel) applied to ω*     accel 8 rad/s² (A)
speed filter  ω_m ← ω_m + dt/(τ+dt)·(ω_o − ω_m)         τ 5 ms, ω_o = gearbox output speed
PI + ff       u = clamp(ke·N·sp + kp·e + ki·∫e, ±V),     e = sp − ω_m; no integration while saturated in e's direction
4 substeps of h = dt/4:
  current     i = clamp((u − ke·N·ω_o)/R, ±I_lim)
  motor       τ_m = N·η·kt·i
  friction    τ_f = N·kt·i_free·tanh(ω_o/0.05) + b_out·ω_o
  driveline   d = output angle − wheel angle; δ = backlash
              τ_s = max(0, k_s(d − δ/2) + c_s(ω_o − ω_w)) if d > δ/2;  min(0, k_s(d + δ/2) + c_s(ω_o − ω_w)) if d < −δ/2; else 0
  dynamics    J_m·N²·dω_o/dt = τ_m − τ_s − τ_f;   dd/dt = ω_o − ω_w
applied wheel torque = mean τ_s over the substeps → Joint::SetForce
odometry      gz::math::DiffDriveOdometry with wheel separation track·track_multiplier (as DiffDrive and Clearpath)
reset         ISystemReset clears integrators, ramp, backlash state, dig-in state and odometry
```

The setpoint ramp is a controller choice, not motor physics. Without it, the motor, contact and tyre model rises
to 90 % in 24 ms and stops in 16 ms (M, prototype). With it, the rise is about sp/accel = 0.33 s at wz = 1. The
8 rad/s² default stands until the driver team reports their motor controller's ramp (Q7).

| Parameter | Value | Basis |
|---|---|---|
| Motor | 24 V, R 0.46 Ω, kt = ke 0.0445 N·m/A | Husky A200 motor (R [18]) |
| Gear, efficiency | 50, 0.8 (datasheet alternative 51, 0.7) | Prototype (M); IMS 3-stage planetary (R [22]) |
| Rotor inertia | 1.2e-5 kg·m² (0.03 kg·m² at the wheel) | (A), consistent with Husky 0.08 kg·m² at 78.71:1 [18] |
| No-load current, output friction | 1.0 A, 0.05 N·m·s | (A) (REV NEO 1.8 A free [22]) |
| Current limit | 20 A → 35.6 N·m at the wheel | (M) prototype; user's motor range 10–40 N·m; traction limit μ·N·r ≈ 17–27 N·m |
| Free speed | 24/(0.0445·50) = 10.8 rad/s → 1.62 m/s; clamp 10 rad/s | — |
| Driveline | 1500 N·m/rad, 2 N·m·s/rad, backlash 1.5° (0.026 rad) | (A): 12 mm × 0.1 m steel shaft ≈ 1600 N·m/rad; IMS backlash 0.8–2.5° [22] |
| PI | kp 4 V/(rad/s), ki 40 V/rad, feed-forward ke·N | (M) prototype tuning |
| Ramp | 8 rad/s² (1.2 m/s²) | (A), to be replaced by the real controller's value (Q7) |

### 6.4 Contact callback (every wheel contact, ~8 per step)

```
k       = the wheel whose tyre collision is collision1 or collision2 (else: return, contact untouched)
v       = v_hub,k + ω_k × (p − p_hub,k);  v_t = v − n(n·v);  s = |v_t|                (the other body is static or slow)
surface = other is the terrain heightmap      → ground.png at p (nearest sample); <default_surface> if the world has none
          other is in the terrain model       → ground.json "collisions" (exact name), else "prefixes", else "rock"
          other is any other model            → that collision's explicit SDF <surface><friction> μ (μs = μk = μ,
                                                Crr 0.015, slip 0.05), else <object_surface> ("manmade")
                                                (looked up once per collision entity through components::Collision, cached)
μ       = μk + (μs − μk)·exp(−(s/v_stribeck)²)                    (μs = μk on loose types: no peak)
μ      *= max(0.05, 1 + σ·(0.7·n(p/L) + 0.3·n(3p/L)))             n = smooth value noise in [−1, 1], σ 0.2, L 0.3 m (A)
if s > v_align (5 mm/s):                                         sliding: friction circle
    fdir1 = v_t/s;  μ1 = μ;  μ2 = μ·perp_ratio (0)
    slipCompliance = secondarySlipCompliance = slip · |ω_k·r| / max(N_k / ncon, 5)     [m/s/N], no speed floor
else:                                                            sticking: box aligned with the expected load
    û     = direction of the previous step's tangential force at wheel k, else the tangential part of gravity, else the heading
    fdir1 = û;  μ1 = μ;  μ2 = μ·stick_perp_ratio (0.3)           (resultant limit μ along û, ≤ 1.044 μ in any direction)
    slipCompliance = secondarySlipCompliance = 0
```

The callback gets point, normal and contact count from the event (`gz/sim/physics/Events.hh` in the env). Hub pose
and velocities are read in `PreUpdate`. `N_k` and the previous tangential force come from the load source:
- `contact`: `ContactSensorData` on the tyre collisions, from the previous step, as in the prototype.
- `joint`: the C++ `JointTransmittedWrench` of the wheel joint, projected on the contact normal, plus the wheel's
  weight.

Gate G8 chooses between them by cost and by agreement within 10 % on a slope test. The slip conversion is gz's
`WheelSlip` convention (unitless slip × wheel speed / normal force). Its consequences:
- A stopped wheel has zero compliance: no creep at rest.
- A locked, sliding wheel (s > v_align, ω = 0) is pure Coulomb sliding.

Why this replaces revision 1's contact rule:
- **Every wheel contact is overridden.** Revision 1 left non-terrain contacts to DART's box rule, which cannot
  turn in place (D1). With the friction circle everywhere, the landing pad, the C2 pad, proving-ground steps,
  Delivery ledges and the wave-1 friction tiles all turn at their surface's ratio.
- **No μ scaling by a lateral ratio.** It changed only the torque, not the spin rate (D5). Lateral soil resistance
  is the bulldozing hub force of §6.5.
- **No 0.1 m/s floor on the compliance speed, and zero compliance while sticking** (D23).
- **The sticking branch is aligned with the expected load.** DART bounds each friction direction separately, so
  revision 1's μ1 = μ2 = μ box let a parked rover hold up to √2·μs, depending on its heading: 31.8°–41.2° on
  regolith (critique E). The aligned box holds within 4.4 % of μs at any heading. It also removes the
  static-friction overshoot that made the 10 A stall non-monotonic (§6.9).

### 6.5 Rolling resistance, bulldozing, dig-in, sinkage, dust

All forces in this section are applied in `PreUpdate`, per wheel, with `Link::AddWorldForce`. They use the surface
under the wheel's deepest contact.

- **Rolling resistance**: F = −Crr·D·N·tanh(v_f/(rr_w0·r))·f̂.
  - f̂ is the wheel's heading (axle × up, normalised); v_f is the hub's ground speed along f̂.
- **Bulldozing**: F = −k_b·D²·N·tanh(v_l/(rr_w0·r))·â.
  - â is the axle; v_l is the hub's ground speed along â.
  - This is the sidewall part of the side force in loose soil [20]. It acts when the wheel moves sideways: in a
    spin, on a side slope, when skidding.
  - In the spin moment balance it adds k_b·a per wheel (§5.6).
  - Magnitude (A): between Rankine passive pressure (0.006) and Bekker's bulldozing formula (0.11) at 2 cm sinkage
    in dry sand (computed from [39] and [40] with Sheppard's bulk density [2]).
- **Dig-in** (D21), per wheel, on types with dig_rate > 0:
  - State: extra sinkage z_d, with D = 1 + z_d/sinkage_m.
  - dz_d/dt = dig_rate·s − z_d·|v_hub|/L_heal, where s is the contact's slip speed (§6.4) and L_heal = 0.3 m
    (one wheel diameter, (A)).
  - z_d is clamped to [0, (D_max − 1)·sinkage_m] and decays (heals) on any surface.
  - Defaults (§5.6) on sand: a sustained spin reaches D_max in ~1.5 s. Forced slip 0.6 at constant hub speed
    settles at D = 1.22 (computed). That is far milder than the 3–7× of [36], chosen so that the default spin
    slows but does not stop.
  - Spinning stops entirely once Crr·D·c + k_b·D²·a ≥ μk·c: on sand near D ≈ 1.7. A "strong" preset (D_max 2.0,
    dig_rate 0.05) reproduces Anveshak's stuck zero-radius turn [14]; choosing it is Q12.
  - The state is not geometric: the wheel does not visibly sink further (§4).
- **Slip**: from slip compliance (§6.4). On flat sand at 0.5 m/s it gives slip × Crr = 20 % (M), which restates
  the parameters (§6.9).
- **Sinkage**: static, from the collision carve under the type (§5.8). It is what makes wheels look sunk in sand.
- **Dust**: rate = dust(type)·(8 s/m·|v_hub| + 25 s/m·slip speed)·D, capped at 40 /s, and emitting off below
  0.05 m/s. It is published at 10 Hz as `gz.msgs.ParticleEmitter` (rate, emitting) to each rear emitter (A gains).
  - Tuned emitter (M): box 0.25 m; particle 0.2 m; lifetime 1.6 s; speed 0.15–0.5 m/s; scale rate 1; colour
    (0.80, 0.70, 0.56) fading alpha 0.28 → 0; soft puff texture.
  - Emitters sit on the rocker links behind the rear wheels (a wheel link spins).
  - Worlds load `gz-sim-particle-emitter-system`.

### 6.6 Solver

The solver is chosen per world (D4). PGS goes where gate G2 passes (lander key presses and stacked objects behave)
and gate G5 shows physics ≥ 1.1× real time with the drivetrain. That covers the proving ground, Delivery,
Astrobiology, Autonomy, `rover_test.sdf` and the test world of `simulate.py`, if the gates confirm. Equipment
Servicing is expected to stay on Dantzig unless the lander is optimised (Q2). There, the per-wheel friction limits
are approximate and the diagonal-stall targets are not tested. PGS's small yaw-rate noise (std 0.003–0.008 rad/s
(M)) is numerical and is not counted as realism.

### 6.7 Tyre compliance (phase 2, `Params.tire_compliance`, default off)

A hub link (0.1 kg) between rocker and wheel, with two springs, both set via SDF `spring_stiffness`/damping and
limited to ±3 cm:
- radial prismatic: 60 kN/m, 170 N·s/m;
- axial prismatic: 40 kN/m, 40 N·s/m.

(M) DART honours it: a 12–16 Hz wheel hop and 0.03–0.05 m/s² rms body vibration, instead of 400–480 Hz chatter.
It also strengthens slow-turn judder (§6.9). Revisit when the wheel type is known (Q8).

### 6.8 Topics, odometry and who notices

- `/model/rover/cmd_vel`, `/model/rover/odometry` and `/model/rover/tf` keep their names and types, so the station,
  the bridge and the referee need no change.
- Odometry stays no-slip, like DiffDrive. With `track_multiplier` 1, during a spin it over-reports yaw by
  1/ratio: 2.3× on rock, 2.65× on regolith, 3.8× on fresh sand, more as the wheels dig. A real rover's odometry
  does the same. `/model/rover/ground_truth` remains the truth.
- New `/model/rover/drivetrain` (`gz.msgs.StringMsg`, JSON, 50 Hz), §9.3: per-wheel setpoint, speed, current,
  torque, voltage, saturation, slip speed, load, surface, dig factor. Tests read torques and currents here.
- The **driver team** (memory: the driver executes twists, the controller owns position) should know:
  - realistic slip makes the deferred gyro yaw-rate loop necessary;
  - an effective-track calibration on the real rover is worthwhile (Mandow eq. 11 [16]; Clearpath's multiplier
    [18]), and the sim exposes the same knob (D24);
  - their motor controller's ramp rate sets the turn rise time (§6.3).
- The **mechanical team** should know: in a spin, the longitudinal friction moments about the rocker pivots are
  opposite on the two sides (the differential's free mode). They unload one diagonal:
  ΔN/ΣN = h·μ·0.748/a, with pivot height h = 0.35 m. That is 0.58 at μ 1 (179/47 N per wheel) and 0.465 at
  μ 0.8 (166/60 N). A lower pivot reduces diagonal unloading and stalls.

### 6.9 Calibration targets (regression tests)

Kinds: **physics** means it follows from the model's mechanics and could fail. **plumbing** means it restates
input parameters and checks they are wired. **(A)** means it tests an assumed sub-model.

| Behaviour | Target | Kind | Prototype (M) | Today (M) |
|---|---|---|---|---|
| Spin in place, flat, wz 1 rad/s, each catalogue type, fresh | yaw ratio = closed-form root of §5.6 ± 0.04 (rock 0.436, regolith 0.377, sand 0.261, wash 0.182, test μ 0.441) | physics | 0.42–0.44 on μ-only lanes (closed form 0.441) | 0.889; 0.000 on μ ≤ 0.5 tiles |
| Spin in place on objects and tiles: landing pad, a proving-ground step top, a μ 0.2 tile (wave 1) | their surface's closed-form ratio ± 0.04 (manmade 0.434, rock 0.436, test 0.441) | physics (regression of today's bug) | box friction: 9e-8 | 0.000 on tiles |
| Mean wheel torque while spinning on μ-only lanes | 12.7·μ N·m ± 25 % (= μ·N·r·0.748) | physics | 2.4–2.7 at μ 0.2; 9.2–14.9 at μ 0.95 | 10.4 regardless |
| Diagonal load split while spinning (PGS worlds) | loads within ±15 % of 179/47 N at μ 1 and 166/60 N at μ 0.8; loaded/light torque ≥ 2 | physics | 179/47/47/179 N and 20/5 N·m at μ 1; 15–16/5 N·m at μ 0.8 | equal |
| Current while spinning on a μ 0.8 slab, 20 A limit | loaded wheels 9.7 ± 1.5 A (analytic: (14.9 + 2.3 N·m)/1.78 N·m/A); light wheels 4–6 A | physics, with (A) free current | 9.7–9.8 / 4.3–5.0 A | — |
| Stall at a current limit | 0.7 × 9.7 = 6.8 A: yaw over 5 s < 15 % of the unlimited run; 1.3 × 9.7 = 12.6 A: ≥ 85 %; yaw non-decreasing over limits {6, 8, 10, 12, 14} A (±2°) | physics | non-monotonic with the box sticking branch: 7 A 6°, 10 A 0°, 20 A 118° | — |
| Rise to 90 % of steady yaw rate; stop | with the ramp: rise = sp/accel ± 20 % (0.33 s at wz 1); without the ramp: rise < 60 ms, stop < 40 ms | plumbing (ramp); physics (no ramp) | 0.273 / 0.139 s with ramp; 0.024 / 0.016 s without | 11–72 ms; 4–23 ms |
| Slow spin 0.15 rad/s on a μ 0.8 slab, no tyre compliance | yaw-rate peak-to-peak/mean ≥ 0.8, dominant 0.5–5 Hz. On sand (μs = μk): peak-to-peak/mean < 0.5 (no stick-slip in loose soil) | physics with (A) Stribeck | 1.26 at 0.8 Hz (phase 2 with tyres: 4.6 at 1.8 Hz) | 0 |
| Loose sand straight 0.5 m/s | slip = slip × Crr = 20 % ± 3 %; torque = Crr·N·r = 3.4 N·m ± 15 % | plumbing | 20 %, 3.4 N·m | 0 %, ~0 N·m |
| Dig-in on sand, defaults | a 10 s spin: ratio falls monotonically to 0.6–0.8× of the fresh ratio (0.72× computed) and never to 0. The rover then drives straight out at 0.3 m/s. After 1 m of rolling, D < 1.05. No dig on rock or regolith | (A) | — | — |
| Washboard 0.5 m/s | wheel-torque std < 1.5 N·m | physics | 0.2–0.8 N·m | 12.8 N·m (to ±30) |
| Spin on a 20° side slope, 2.5 s, regolith | slides 0.4–1.0 m downhill (re-baselined with the slip semantics of §6.4) | physics | 0.66–0.74 m (slip 0) | 0.48 m |
| Straight across a 20° side slope, 3.5 m at 0.5 m/s | downhill drift = slip·tan 20°·3.5 m ± 30 % (rock 0.06 m, regolith 0.38 m, sand 1.27 m); < 0.1 m on rock | plumbing + direction | < 3 cm (slip 0) | — |
| Parked, cmd 0, on 15° regolith and 20° sand, 60 s | moves < 1 cm | physics (creep regression, D23) | not tested (default slip 0) | via tiles |
| Parked hold angle vs heading | holds at atan(μs) − 2°, slides at atan(μs) + 2°, the same within ±1.5° at headings 0°, 30°, 45°, 90° | physics | box: up to √2·μs depending on heading | — |
| Odometry yaw / true yaw while spinning (`track_multiplier` 1) | 1/ratio of the surface ± 15 % | plumbing | ~2.3 (1/0.438) | 1.12 (1/0.889) |
| Parked on μ 0.2 / μ 0.95 calibration lanes | slides / holds | physics | existing tests | via tiles |
| Cost vs the DiffDrive variant, same world, rover driving | CPU time per step ≤ +25 % (method of §10.3) | budget | +15–26 % (critique E) | — |

## 7. Rendering

1. **Media patch** (D12): `sim/tools/gz_media.py` copies `share/gz/gz-rendering8` of the pixi env to
   `sim/build/gz-rendering-media/`, applies `sim/patches/gz-rendering8-ogre2-media.diff` (from
   `render/prototype/gz_media_patch/`), and writes the sun direction of `urc/lighting.py` into the sky shader.
   - `sim/gzenv.py` sets `GZ_RENDERING_RESOURCE_PATH` when the directory is complete. Its users are `run.sh`
     (server and GUI), the station, `tests/simulate.py`, `render_map.py` and the camera subprocess tests.
   - If the diff does not apply (env upgrade), the tool warns, leaves no directory, and exits 0, so the stock media
     are used and `sim-build` never fails. A test covers that fallback.
   - Patch contents:
     - Terra `roughness = 1` without a roughness map;
     - a procedural clear-desert sky (zenith-to-horizon gradient, Mie glow, sun disk) replacing `skybox_fs`;
     - a haze piece (β 4e-5 /m, ~98 km visibility) for Pbs and Terra, guarded so it applies only where `inPs.pos`
       exists.
   - Report the Terra roughness bug upstream.
2. **Lighting** (D13) in `world.py` from `urc/lighting.py`. `<scene><sky>` stays on; the patch replaces its shader.
3. **Terrain**: colour map (de-shaded for Autonomy) + detail layers (§5.7).
4. **Far field** (D14): `farfield.build(models_dir, origin, terrain_size, terrain_center)` writes a per-world GLB.
   - Inputs: `render/data/far_dem_3dep_60km.tif` and `imagery/naip2021_far_60km.tif`, moved to `sim/data/dem/`
     and `sim/data/imagery/`.
   - The geometry is sunk 4 m at the seam, with vertices Z-up. The texture is shared through `Media` across
     worlds, so only the geometry is per world.
   - The horizon plane goes; the catch-floor box stays.
5. **Vegetation and pebbles** (D10, §5.5) as merged GLB chunks.
6. **Dust** (D15).
7. **Cameras**:
   - Clip far 80,000 m for the eye, chase and fly cameras and the rover's RGB.
   - The rover's depth stays 0.1–40 m via `<depth_camera><clip>`, which gz-sensors 8 `RgbdCameraSensor` honours.
   - Rover RGB-D noise stddev 0.06 (~2 DN (M)).
   - Optional lens flare on the chase camera (scale 0.6, colour (1.0, 0.95, 0.9) (M)); G6 checks that LensFlare
     disconnects from PostRender.
   - Not used: `<distortion>`, `<fog>`, `<projector>` (§4).

Expected cost (M, 1280 × 720, visual-only world, all views at far 80 km, median of 3, spread ±1.5 ms):
- frame time 10.4 ms → 11.1 ms for the terrain changes;
- 15.1 ms with the far field, 1.5 M shrub triangles and 0.8 M pebble triangles;
- peak memory per gz process 0.77 → 1.42 → 2.15 GB.

The GUI process loads the same assets again (not measured). The biggest memory levers: a compressed (BC1/BC7) DDS
colour map, fewer shrub triangles, no pebbles outside mission sites.

## 8. Free-fly camera and bird's-eye map

### 8.1 FlyCamera system (`sim/plugins/fly_camera.cpp`)

FlyCamera is a fourth plugin, not a third kind in `chase_camera.cpp`: it shares little, needs gz-rendering and
gz-common geospatial, and keeps the rover-eye code untouched. It moves the `fly_camera` model: one link, no
gravity, no collision, no visual, so no other camera sees it. Contract (from the measured prototype
`sim/data/research/flycam/prototype/flycam_proto.cpp`):

- **SDF parameters**:
  - `target` (rover), `clearance` 1.0 m;
  - `time_constant` 0.2 s, `look_time_constant` 0.08 s;
  - `deadman` 0.3 s, with `deadman_clock` `wall` (default) or `sim` (tests at RTF 0 use sim);
  - `speed_per_agl` 1.0 /s, `min_speed` 2 m/s, `max_speed` 200 m/s, fast ×4;
  - `max_altitude` 2000 m above the highest terrain, `margin` 100 m beyond the terrain edge.
- **Topics** (all published, no services):
  - `/fly_camera/cmd` (Twist: linear = normalised forward/left/up in the yaw frame; angular.z/y = yaw/pitch rates);
  - `/fly_camera/speed` (Double, ×0.25–4);
  - `/fly_camera/look` (Vector3d: yaw/pitch deltas, smoothed);
  - `/fly_camera/goto` (Pose, smoothstep over clamp(dist/50, 0.4, 1.5) s; header key `jump` = instant);
  - `/fly_camera/mode` (StringMsg: `free | follow | top | level | stop | ortho <width> | perspective`);
  - `/fly_camera/state` (StringMsg JSON at 10 Hz: mode, x, y, z, yaw, pitch, agl, ground, speed, ortho).
- **Each step** (sim time; skipped while paused; re-initialised when sim time goes backwards):
  1. Apply inputs under a mutex.
  2. Target velocity = cmd (if newer than the deadman) × clamp(agl·k, v_min, v_max) × multiplier, rotated by yaw.
  3. First-order lag towards the target velocity.
  4. Look target += deltas + rates·dt; the view follows with 0.08 s.
  5. Position += v·dt (+ the rover's displacement in follow mode).
  6. Floor = max(ground(p), ground(p + 0.5 s·v)) + clearance, approached with 0.1 s; hard floor ground + 0.3 m.
  7. Clamp to bounds.
  8. `SetWorldPoseCmd` (measured free).
- **Ground**: the world's visual HEIGHTMAP geometry from the ECM, loaded with `gz::common::ImageHeightmap`. Heights
  are divided by their maximum × `size.z`, row 0 north. (M) Within 6e-5 m of the sheet; up to 56 m off before the
  normalisation fix, and 19 m off with the wrong row order. This lookup is the shared header `terrain_heightmap.hh`
  (§12). Each heightmap has its own `size.z` after §5.8, and FlyCamera reads the visual one.
- **Orthographic**:
  - Connect `events::SceneUpdate` once, in `Configure`. Never use PreRender/Render/PostRender (−35–45 % sim speed
    (M)), and never connect lazily (`EventT::Connect` is not thread-safe (R)).
  - When the width changes, look the camera up by scoped name in `rendering::sceneFromFirstRenderEngine()`, set
    `CPT_ORTHOGRAPHIC` and a custom ortho matrix of width w × w·H/W. Keep no pointer (M: crash at shutdown otherwise).
  - While orthographic, pitch is locked straight down and w = 2·agl·tan(hfov/2), so toggling does not jump.
- **Reset**: implements `ISystemReset`.

`FlyParams` live in `sim/viewers.py` (WS-0 split, §5.8): 20 Hz, 1280 × 720, hfov 1.2, clip 0.1–80,000 m, the
speeds above. The camera is built through `_viewer_sdf`, with constants `FLY_MODEL` and `FLY_*_TOPIC`. An
unwatched camera costs nothing measurable; a watched 1280 × 720 at 20 Hz costs ~4 % sim speed (M).

### 8.2 Offline orthophoto map (`sim/tools/render_map.py`, pixi task `sim-maps`)

`render_map.py` is a port of `flycam/prototype/exp5_orthomap.py`:
- **Capture**: a world copy (via `tests/worldfiles.py`) without the rover; FlyCamera with time constant 0; one
  `goto` per tile; wait for a frame stamped after state t + 0.15 s. The camera flies 300 m above the highest terrain
  (inside shadow range), with hfov ≥ 0.1 rad and 12 % tile overlap.
- **Assembly**: each map pixel is projected through the tile camera at its terrain height (cv2.remap) and stitched
  by nearest tile centre, with a feather in the overlap. (M) Seams continuous; without feathering, the brightness
  step was 1.0–1.9× the adjacent-pixel difference.
- **Output**: `sim/worlds/<world>_map.jpg` (4096 px: 6.25 cm/px at 256 m, 0.25 m at 1 km, 0.5 m at 2 km; 2–3.3 MB)
  and `<world>_map.json` (size, gsd, frame, and a SHA1 of every input: world SDF, heightmaps, colour map, GLBs,
  media patch diff).
- **Staleness**: it needs the GPU, so it is not part of `sim-test`; the station flags a stale map by hash.
- **Phase 2**: a "Live" layer drawing the orthographic fly-camera stream at its exact rectangle.

### 8.3 Driver station

- **Views**: driving views [eye (default), chase, onboard, depth] cycled by V, as today. Inspection views are
  **Fly** (B toggles, and returns to the previous driving view) and **Map** (G toggles; a click on the rail map
  opens it).
- **Fly keys** (same handler, reinterpreted):
  - W/S, A/D move and strafe; E/Q up/down; Shift fast; wheel or +/− speed;
  - arrows/IJKL and drag look;
  - R jump to the rover, F follow (keep offset), T top-down, O orthographic, C level, Space stop;
  - double-click flies to the clicked spot.

  Gamepad: left stick move, right stick look, LB/RB down/up, RT fast, B stop, Y top-down, X next view.
- **While in Fly or Map**, the page keeps sending empty driving input: the rover decelerates and holds, the deadman
  stays satisfied, the LED stays teleop blue, and the status reads "Inspecting: the rover holds still". When the
  rover is released to autonomy, the page sends no driving input anyway, so the user can watch autonomy from the
  air. Fly look never moves the pan-tilt head. The small view shows the rover eye; clicking it returns to the eye
  view.
- **Protocol**: new WebSocket messages, with a pure `drive.Fly` helper and unit tests:
  - `{t:'fly', move:[f,l,u], turn:[yaw_rate,pitch_rate], fast}`;
  - `{t:'fly_look', yaw, pitch}`;
  - `{t:'fly_speed', scale}`;
  - `{t:'fly_mode', mode}`;
  - `{t:'fly_goto', kind:'rover'|'point'|'top'|'pixel', x, y | u, v}`.

  Goto poses:
  - rover: 6 m behind, 3 m above, looking at it;
  - point: keep yaw, pitch 0.6 rad, distance clamp(span/4, 15, 400) m, z ≥ ground + clearance from the sheet heightmap;
  - top: straight down at agl = span/1.4;
  - pixel: march the ray through the clicked pixel against the sheet heightmap and stop 15 m short.
- **Map view**: a canvas with drag-pan and cursor-anchored wheel zoom; a live rover arrow and trail, places, and
  the fly-camera icon and footprint. A click flies there; a double-click enters Fly top-down there. A Photo/Relief
  toggle sits on the rail map. The relief (hillshade) uses a fixed metres-per-colour scale (today 1.3 m of relief
  looks like mountains (M)).
- **Telemetry** adds `fly` and a compact drivetrain readout: per-wheel current bars, a saturation marker and a
  dig-in marker on the wheels canvas. `info()` adds `hfov.fly` and whether a current photo map exists.
- **Help overlay** gets "Fly camera" and "Map" sections.
- Hard-won rule: per-tick control goes over published topics, never services (M: blocking Python requests stall
  ~1 s under a busy subscriber).

## 9. Interfaces between workstreams

### 9.1 Ground map (in each terrain model directory, written by `WorldBuilder.write`)

- `ground.png`: 8-bit grayscale, n × n = heightmap samples. Sample (row, col) is at world
  x = −S/2 + col·S/(n−1), y = S/2 − row·S/(n−1). The value is an index into `ground.json`; lookup is
  nearest-sample.
- `ground.json`:

```json
{"format": "rover-ground/2", "size_m": 2048.0, "samples": 4097, "default": 3,
 "types": [{"index": 3, "key": "regolith", "title": "Packed regolith", "mu_s": 0.62, "mu_k": 0.52,
            "crr": 0.10, "bulldoze": 0.0, "slip": 0.3, "sinkage_m": 0.005, "dig_rate": 0.0, "dig_max": 1.0,
            "dust": 0.4, "dust_rgb": [0.80, 0.70, 0.56]}],
 "collisions": {"step_10cm_collision": "rock", "ledge_0_collision": "rock", "zone_sand_3": "sand"},
 "prefixes": {"rocks_": "rock", "slabs_": "rock", "risers_": "rock"},
 "terrain_default": "rock",
 "object_default": "manmade"}
```

- `collisions` holds the exact names of every non-heightmap collision in the terrain model that is not covered by
  a prefix: steps, ledges, blocks and, in wave 1, every friction tile with its zone's type. `WorldBuilder` writes
  it.
- The drivetrain finds the files next to the heightmap whose URI it reads from the ECM (the same lookup as
  FlyCamera). A world without them gets `<default_surface>` everywhere.
- A golden cross-language test compares the C++ lookup with `sheet.ground(x, y)` (§11).

### 9.2 Relief swatches and terrain targets

- `sim/data/relief/<swatch>.npz`: `z` (float16, H × W, metres), `res_m` 0.5, `sigma_m`, `source` (DEM file, window,
  centre lat/lon), `rms_cm` {4, 8, 16, 32}.
- `sim/data/research/terrain_targets.json`: per type and for the whole real square, p25/p50/p75/p90 of window RMS at
  4/8/16/32 m, plus the paint rules and DEM used.

### 9.3 Drivetrain state

`/model/rover/drivetrain`, `gz.msgs.StringMsg`, 50 Hz:
`{"t": 12.345, "cmd": [vx, wz], "wheels": {"fl": {"sp": 2.67, "w": 2.61, "i": 6.2, "tau": 9.8, "u": 14.1,
"sat": false, "slip": 0.12, "load": 113.0, "surface": "sand", "dig": 1.08}, "rl": {…}, "fr": {…}, "rr": {…}}}`
(rad/s, A, N·m, V, m/s, N, D).

### 9.4 Fly camera and map

Topics of §8.1; map files of §8.2.

### 9.5 Python and C++ APIs (names final in the owning workstream, signatures fixed here)

| Owner | API |
|---|---|
| WS-0 | `terrains.Traction/Appearance/Relief/Clutter` dataclasses and `TerrainType` fields (§5.1), with today's behaviour as defaults; `dem.read_raster(path) -> (bands (B, H, W) float32, geotransform)`; `gzenv.environment(build_dir=None) -> dict`; `sdf.camera(...)`; `viewers.write_all(models_dir)`; `simulate.py`: `rover_uri`/`params` override, `default_surface`, `solver`, `cpu_time_per_step()`, scripted twist publisher |
| WS-T1 | `landscape.paint(hf, rules, features) -> np.uint8 raster`; `landscape.from_ssurgo(hf, origin, geojson, slope_rules)`; `landscape.relief(hf, raster, keep_flat) -> dz`; `landscape.place(hf, raster, recipe_kind, rng, avoid) -> [(x, y, size, yaw, tilt)]`; `terrains.TYPES[k].traction`; `sheet.ground(sheet, path)`; `WorldBuilder` writes `ground.json` with the collision map |
| WS-A | `appearance.colour_map(hf, raster, types, rng) -> uint8 (N, N, 3)`; `appearance.deshade(naip, dem_hf, units) -> (naip', shadow_mask, fitted_sun)`; `appearance.ortho_colour_map(naip_path, origin, size, n, inpaint_mask)`; `appearance.detect_shrubs(naip_path, origin, size, slope) -> [(x, y, d)]` (NDVI-anomaly filtered); `appearance.terra_layers(colour_map, hf, details) -> (layer0_png, [detail textures], blend params)`; `meshes.slab(variant)`, `meshes.riser_strip(polyline, height, depth)`, `meshes.write_glb(path, V, F, N, UV, texture)`; `media.Media` `glb` kind; `farfield.build(models_dir, origin, size, center) -> model name`; `lighting.sun(date, time, lat, lon) -> direction, colour, intensity` |
| WS-D | `terrain_ground.hh` (C++ reader of `ground.png`/`ground.json`, built on `terrain_heightmap.hh`'s resource-path resolution) |

### 9.6 Media patch

`sim/build/gz-rendering-media/` (generated, ignored by git) and `GZ_RENDERING_RESOURCE_PATH`, set only by `gzenv`.

## 10. Performance budgets

### 10.1 Baseline today (M)

Headless, Sensors system stripped, `real_time_factor` 0, rover idle, 15 s of sim time, wall time minus the ~2.6 s
start-up. The machine was loaded by a parallel workflow (load average 6–12), so these are pessimistic:

| World | Real-time factor (runs) |
|---|---|
| `proving_ground` | 7.8 |
| `urc_autonomy` | 3.1, 2.7 |
| `urc_astrobiology` | 2.2 |
| `urc_delivery` | 1.9, 1.7 |
| `urc_equipment_servicing` | 1.29, 1.21, 1.14 |
| `urc_equipment_servicing` without the lander | 3.1 (the lander's 101 joints take ~2/3 of the step time) |

CPU time per step (M, critique E: plain `gz sim -s`, 10,001 steps, user+sys minus start-up, interleaved pairs,
load average 6–14):

| World | Dantzig | PGS |
|---|---|---|
| Delivery | 0.55 ms | 0.69 ms |
| Equipment Servicing (best runs) | 0.81 ms (1.23× real time) | 1.06 ms (below real time) |

### 10.2 Known costs (M)

| Item | Cost |
|---|---|
| PGS vs Dantzig | CPU ×1.22–1.25 in Delivery; ×1.05–1.50 in Equipment Servicing (noisy) (critique E) |
| Drivetrain prototype vs DiffDrive (Dantzig) | +18 % in Delivery; +15 % minimum / +26 % median in Equipment Servicing (critique E). Revision 1's "+5 %/+11 %" came from a harness with heavy Python logging every step and is retracted. Likely contributors: world-wide contact extraction for `ContactSensorData`, and the callback's per-contact filter (R, `Physics.cc`), both of which grow with the number of contacts in the world |
| Autonomy at 4097² vs 2049² (collision + visual heightmaps) | start-up 2.58 → 3.22 s, peak memory 548 → 749 MB, per-step time unchanged within noise (critique E); +350 MB for the visual (render research) |
| Cameras | unwatched extra camera 0; watched 1280 × 720 at 20 Hz ~4 % |
| Render changes | +0.7 ms (terrain only) to +4.7 ms (everything) per 1280 × 720 frame; +0.65–1.4 GB peak per gz process |
| FlyCamera | pose updates free; render-event hooks other than SceneUpdate −35–45 % |
| Delivery estimate | 0.55 × 1.25 × 1.18 ≈ 0.81 ms/step, about 1.2× real time: above the floor, below the target (§10.3) |

### 10.3 Budgets

Method for every physics number: plain `gz sim -s -r --iterations N`; CPU time (user+sys) from `/usr/bin/time`
minus a 1-iteration start-up run; interleaved A/B pairs; median of at least 5. The rover is driven by a separate
`gz topic` publisher, with no Python per-step callbacks. Load average is recorded. Absolute numbers need load
average < 4; ratios are accepted on a loaded machine.

| Budget | Value | How measured |
|---|---|---|
| Physics, every world | target ≥ 1.3× real time, floor ≥ 1.1× | Method above, rover driving a scripted twist sequence for 20 s sim |
| Equipment Servicing | ≥ 1.1× with Dantzig; ≥ 1.3× only if the lander is optimised (Q2) | Same |
| Drivetrain cost | ≤ +25 % CPU per step vs the DiffDrive variant | Same, interleaved pairs |
| Driver station | ≥ 15 fps on the main view (target 20) with the small view on, in every URC world, full render config. Expected frame rate ≈ min(render throughput, 20 Hz × RTF), because the Sensors system waits for the previous render | `Feed.fps()` of eye, chase and fly over 10 s after 5 s warm-up, world with Sensors at RTF 1 (gate G6) |
| Start-up | ≤ 30 s to the first step (Autonomy at 4097² included) | Wall time to first `/clock` |
| Memory | ≤ 3.5 GB peak per gz server | `/usr/bin/time -l` |
| Generation | `pixi run sim-worlds` ≤ 5 min for all worlds | Wall time |

### 10.4 Gates (run first in WS-0 unless noted; results in `sim/data/research/gates.json`)

| Gate | Question | Pass | If it fails |
|---|---|---|---|
| G1 | Autonomy at 4097² (collision + visual), now with a 60 s driving run | RTF loss ≤ 15 % vs 2049², start-up ≤ 30 s, memory ≤ 3.5 GB (idle already measured: passes) | 2049² for both (1 m); real 0.5 m relief lost below 1 m |
| G2 | PGS on Equipment Servicing, Delivery and the proving ground | Lander key-press test passes; toolbox + wrench stay put 60 s | Dantzig in that world (§6.6) |
| G3 | GLB meshes as collisions in DART; GLB vs OBJ visual memory | Loads, contacts correct | OBJ for collisions, GLB for visuals |
| G4 | Alpha-tested textures from SDF (shrub cards, feathered decal edges) | Visible cut-out / feathered edge | Opaque low-poly shrubs; no detail decals |
| G5 | Drivetrain prototype + PGS per world, method of §10.3 | ≥ 1.1× real time | Dantzig in that world; feeds Q2 |
| G6 | Station frame rate: full-config render assets in Delivery and Equipment Servicing; eye and fly watched, RGB-D subscribed, RTF 1 | ≥ 15 fps; LensFlare logs "Render pass added" and disconnects | Lower shrub/pebble budgets; fly camera 960 × 540; lens flare off |
| G7 | Delivery and Astrobiology at 2049² (4× today's samples) | Generation ≤ 2 min each; RTF loss ≤ 10 %; mission grades and easy-route invariants unchanged | 1025² + relief transfer at 1 m |
| G8 | Drivetrain cost: `load_source` contact vs joint, and contact customisation on vs off (prototype plugin) | Joint wheel loads within 10 % of contact loads on a 15° slope; pick the cheaper source | Keep `contact`; accept the cost against the floor |

## 11. Test plan

**Unit, no physics** (`test_landscape.py`, `test_appearance.py`, additions to `test_urc_terrain.py`,
`test_urc_unit.py`, `test_gen_model.py`, `test_station.py`):
- Ground raster on the heightmap grid (row 0 north); every type known; zones paint where their outlines are;
  the collision map covers every non-heightmap collision of the terrain model.
- Roughness:
  - each synthetic ground type's median RMS at 4/8/16 m lies within the real p25–p75 of that type (§5.4,
    `terrain_targets.json`);
  - each world's natural ground (keep-flat excluded) has p50 and p90 at 4/8/16 m within ±30 % of the real
    whole-square values;
  - the variance-preserving blend keeps a swatch's RMS within 5 % across feathers.
- Keep-flat masks leave pads, ramps, lanes and easy routes within 2 cm of the macro shape; easy-route and approach
  grades are still as designed.
- Slab placements reproduce the tabulated N(≥1), N(≥2) and N(≥4) within ±30 %, with cover by 1–7 m blocks of
  8–9 % ± 25 %; tilt ≤ 30°; nothing within keep-clear radii; on real-DEM worlds D < 1 m.
- Heightmaps: the decoded collision heights equal the original minus the carve, within one 16-bit step, including
  at the global maximum; objects stand on the collision surface.
- Colour:
  - per-type medians within ΔE 10 of the palette target;
  - compensation reproduces the colour map to 1 DN on a synthetic case (linear light);
  - NAIP de-shading gives |corr(luminance, cos i)| < 0.1 on slopes > 10°;
  - shrub detections pass the NDVI-anomaly filter.
- `ground.json` schema; traction values per §5.6; types unique by key and colour.
- `gen_model`:
  - with `mode = "diffdrive"`, output byte-identical to the wave-0 baseline;
  - with `mode = "physical"`: no DiffDrive; RoverDrivetrain with `DriveParams` values; wheel effort 1000;
    joint velocity limit 16 rad/s; isotropic tyre; dust emitters on the rocker links with `emitting` false,
    explicit topic, scatter ratio 0; depth clip 40 m and RGB clip 80 km.
- `viewers`: FlyParams model; eye, chase and fly clip far 80 km.
- `drive.Fly`: key mapping, speed clamp, goto pose maths, ray–ground intersection.
- Media tool: applies the diff to a copy and leaves stock media untouched; on a diff that does not apply, warns,
  writes nothing and exits 0.
- `gzenv`: the same variables for every user.

**C++ unit** (ctest target `sim/plugins/tests/`, no Gazebo):
- motor and driveline step: free speed, stall torque, current limit, backlash dead band;
- dig-in integrator: growth, cap, healing;
- friction-direction selection in both branches;
- `ground.json` parsing.

**Physics, headless TestFixture** (`test_drivetrain.py`, adapted `test_rover_sim.py`, `test_urc_terrain.py`):
- every row of §6.9;
- existing suspension tests: differential, twist ditch to the rocker limit, 20 cm rock garden crossable;
- `test_turns_in_place` moves to the per-surface targets;
- golden cross-language test: an asymmetric `ground.png` (north half sand, south half rock); park or drive the rover
  at known points; compare `surface` on `/model/rover/drivetrain` with Python `sheet.ground(x, y)`; the same for the
  C++ heightmap height vs `sheet.terrain`;
- RoverDrivetrain reset through `ISystemReset`;
- determinism: the same run twice gives identical poses;
- odometry and tf message types and frame ids;
- `cmd_timeout` 0 holds the last command, as GUI Teleop needs.

**FlyCamera** (`test_fly_camera.py`, pose only, no rendering, `deadman_clock` sim):
- a constant command gives the expected speed and ease-in;
- a flight into a hill keeps z ≥ ground + clearance;
- the deadman stops it;
- follow mode keeps the offset;
- reset.

**Rendering, one render per subprocess** (`test_render.py` + the existing camera test):
- ArUco 0 still decoded at 2.5 m with haze and camera noise;
- depth image unchanged by the haze, and unchanged within its field of view while the dust emitters run at full rate;
- orthographic object size equal at 40 m and 80 m (±2 %);
- stock cumulus sky gone;
- no specular glint on sunlit flat terrain (brightest 0.1 % of ground pixels ≤ p99 + 15 DN);
- in the Autonomy eye view, pixels toward the Henry Mountains and Factory Butte are not sky.

**Missions** (existing `test_urc_missions.py`, `test_urc_sim.py` and `test_urc_judge.py` pass unchanged in meaning).
Their invariants are re-checked after Autonomy moves to the 0.5 m lidar DEM and Delivery and Astrobiology move to
2049²: easy route, NLOS and grades. Slow tests with the physical drivetrain:
- a pure-pursuit driver on ground truth follows the Autonomy easy route from the start post to within 3 m of
  Post 1 in ≤ 6 min of sim time (A threshold);
- the Delivery crate hill is climbable;
- the scree chute on the steep mesa is not;
- a Delivery sand zone can be crossed straight, and a spin in it slows as §6.9 says.

**Performance** (`test_perf.py`, opt-in `ROVER_PERF=1`): the budgets of §10.3, by the CPU-time method.

**Realism report** (`sim/tools/realism_report.py`, wave 3 and on demand): it writes
`sim/data/research/realism_report.json` with:
- RMS-vs-scale per ground type and per world vs `terrain_targets.json`;
- slab size-frequency;
- shrub density;
- palette ΔE;
- render vs colour map ΔE and colour map vs boosted NAIP ΔE (both ≤ 5), plus render vs raw NAIP, reported for
  information;
- spin ratio per surface;
- a contact sheet next to `mdrs_surface_types_crops.jpg`.

## 12. Workstreams, file ownership and order

Rules:
1. Within a wave, no two workstreams edit the same file. A file changes owner only between waves.
2. Each workstream works in its own isolated copy (D25): a git worktree if the user approves `git init` (Q9),
   otherwise a private copy of the repo without `.pixi/`, `sim/build/` and `sim/data/`. The orchestrator merges
   owned files back at the end of the wave; exclusive ownership guarantees no conflicts. Each copy has its own
   build directory and its own `GZ_SIM_SYSTEM_PLUGIN_PATH` and `GZ_PARTITION`.
3. Generated outputs are regenerated only by the owner of their generator, and only in its own copy:
   `sim/models/rover/`, the camera models, `sim/models/urc_*` and `sim/worlds/urc_*`. Tests write generated
   worlds and models to temp dirs.
4. No `pixi run sim-test` inside a wave. Agents run their own tests with `python -m unittest`. The full suite runs
   at the wave gate, on the merged tree.
5. In wave 1, every new behaviour is behind a flag whose default keeps today's outputs: `DriveParams.mode`,
   `FRICTION`, relief, clutter and sinkage switches. Only new files (`ground.png`, `ground.json`) may appear in
   generated worlds. So every test owned by another workstream keeps passing, and the wave gate cannot deadlock.

**Now, before the workflow starts (orchestrator)**: copy `drive.TDh3/{cpp/rover_drive.cpp, cpp/CMakeLists.txt, *.py,
results_summary.json, variants/}` (not `logs/`, 185 MB) from the session scratchpad to
`sim/data/research/drive/prototype/`. It still exists today, and the session scratchpad may be cleaned (R5).

**Wave 0 — WS-0 Foundations and gates** (one agent, first). It refactors and adds infrastructure; generated
outputs stay byte-identical.
- Workspace isolation (D25, Q9).
- Files it owns:
  - `sim/CMakeLists.txt`: one plugin target per `plugins/*.cpp`, linking gz-sim8, gz-plugin2::register,
    gz-rendering8 and gz-common5::geospatial; build dir selectable; a ctest target `sim/plugins/tests/`.
  - `pixi.toml`: tasks `sim-media` (soft), `sim-relief`, `sim-maps`, `sim-perf`. `sim-build` depends on
    `sim-media`, which cannot fail it. `sim-worlds` inputs gain only the DEMs, NAIP 2024, relief, soils and
    targets files it reads, not all of `sim/data/**` (~107 MB of research artifacts).
  - `.gitignore`; `sim/run.sh`; `sim/station/__main__.py` (environment only, via `sim/gzenv.py`, new).
  - `sim/tests/simulate.py`: rover URI/Params override, default surface, solver argument, CPU-time helpers,
    scripted twist publisher.
  - `sim/plugins/terrain_heightmap.hh` (new: the ECM heightmap lookup ported from `flycam_proto.cpp`).
  - The mechanical splits: `sim/viewers.py` (new) out of `sim/gen_model.py`; `sdf.camera()` in `sim/urc/sdf.py`.
  - The frozen dataclasses of §5.1 in `sim/urc/terrains.py`; `dem.read_raster` in `sim/urc/dem.py`.
  - `sim/data/research/gates.json`.
- It runs gates G1–G8 in temporary directories. G6 and G8 use the prototype plugin from
  `sim/data/research/drive/prototype/`.
- It re-reads the code left by the current workflow and records any renamed building block in this spec's §5.8
  table. That is the only edit it makes to this spec.

**Wave 1 — four parallel workstreams:**

| WS | Scope | Owns (exclusive in wave 1) |
|---|---|---|
| **WS-D Drivetrain & contact** | §6, all behind `DriveParams.mode = "physical"`: RoverDrivetrain (SpeedLimiter, DiffDriveOdometry, `track_multiplier`, `ISystemReset`); ground lookup with the collision map and object surfaces; the friction circle and aligned sticking box; slip compliance; rolling resistance, bulldozing, dig-in; dust control; load source per G8; rover camera noise and clips; tyre-compliance flag (off). The physical rover is generated into temp dirs for its tests | `sim/plugins/rover_drivetrain.cpp`, `sim/plugins/terrain_ground.hh` (new), `sim/plugins/tests/` (new), `sim/gen_model.py`, `sim/urc/sdf.py`, `sim/models/rover/`, `sim/worlds/rover_test.sdf`, `sim/bridge.yaml`, `sim/tests/simulate.py`, `sim/tests/test_drivetrain.py` (new), `sim/tests/test_rover_sim.py`, `sim/tests/test_gen_model.py` |
| **WS-T1 Ground building blocks** | §5.1–5.6, §5.8: catalogue extension; ground raster and `ground.json` with collision map; SSURGO import; swatches (variance-preserving); haystacks, rills; clutter recipes → placements (tabulated slabs); per-PNG heightmap normalisation and sinkage carve; `place()` on the collision surface; `terrain_targets.py`; tiles behind `FRICTION="tiles"`. All existing APIs keep working, and missions generate unchanged | `sim/urc/terrains.py`, `sim/urc/terrain.py`, `sim/urc/features.py`, `sim/urc/landscape.py` (new), `sim/urc/sheet.py`, `sim/urc/dem.py`, `sim/urc/world.py`, `sim/tools/make_relief_swatches.py` (new), `sim/tools/terrain_targets.py` (new), `sim/data/relief/`, `sim/data/soils/`, `sim/data/research/terrain_targets.json`, `sim/tests/test_urc_terrain.py`, `sim/tests/test_urc_unit.py`, `sim/tests/test_urc_dem.py`, `sim/tests/test_landscape.py` (new) |
| **WS-A Appearance assets** | §5.5 meshes, §5.7, §7 items 1, 2, 4–7: pure functions and tools, no world wiring. Covers NAIP de-shading, NDVI-anomaly shrub detection, the `Media` GLB kind, and the soft-failing media tool | `sim/urc/appearance.py`, `sim/urc/farfield.py`, `sim/urc/lighting.py` (new), `sim/urc/textures.py`, `sim/urc/meshes.py`, `sim/urc/media.py`, `sim/tools/gz_media.py` (new), `sim/patches/` (new), `sim/data/imagery/` and `sim/data/dem/` moves of the far-field files, `sim/tests/test_appearance.py`, `sim/tests/test_render.py` (new) |
| **WS-F1 Fly camera** | §8.1 plugin, FlyParams and the fly camera model, §8.2 tool | `sim/plugins/fly_camera.cpp` (new), `sim/viewers.py`, `sim/models/fly_camera/` (new, generated), `sim/tools/render_map.py` (new), `sim/tests/test_fly_camera.py` (new) |

Wave-1 contracts:
- WS-D tests against a synthetic `ground.png`/`ground.json` it writes itself (§9.1); WS-T1 writes the same files.
- WS-A and WS-T1 meet only through the WS-0 dataclasses and the APIs of §9.5.
- WS-F1 includes `terrain_heightmap.hh` read-only and builds its own test worlds.
- `sim/urc/sdf.py` belongs to WS-D in wave 1 (the rover model uses its helpers). An SDF helper that WS-T1 needs
  stays local to `world.py` until WS-T2 moves it into `sdf.py`.
- `sim/urc/props.py` is not edited in wave 1. Media additions go to `media.py`.

**Wave 2 — two parallel workstreams** (after every wave-1 test passes on the merged tree):

| WS | Scope | Owns (exclusive in wave 2) |
|---|---|---|
| **WS-T2 World integration** | Phase T2a (flip), with a full test checkpoint at its end: `DriveParams.mode` → physical (non-drivetrain model parts made unconditional, §6.2); `FRICTION` → ground map, tiles deleted; sinkage on; solver per world (G2, G5); mission slow tests; designs whose intent changed listed for the user. Phase T2b (appearance): Autonomy on 0.5 m lidar + de-shaded NAIP 2024 with SSURGO ground; colour maps and detail layers everywhere; relief; slabs, risers, gravel, shrubs and pebbles per recipe; far field; lighting; particle-emitter system; proving-ground natural strips; Astrobiology real-site variant only if Q1 says so | `sim/urc/world.py`, `sim/urc/terrains.py`, `sim/urc/features.py`, `sim/urc/landscape.py`, `sim/urc/sdf.py`, `sim/urc/props.py`, `sim/urc/missions/*.py`, `sim/gen_worlds.py`, `sim/gen_model.py`, `sim/tests/simulate.py`, `sim/tests/test_urc_terrain.py`, `sim/tests/test_urc_missions.py`, `sim/tests/test_urc_sim.py`, `sim/tests/test_urc_judge.py` |
| **WS-F2 Station** | §8.3 Fly and Map views; drivetrain readout; fixed-scale hillshade; eye and chase clip 80 km; optional ChaseCamera terrain floor and lens flare | `sim/station/*.py`, `sim/station/static/*`, `sim/viewers.py`, `sim/models/chase_camera/`, `sim/models/eye_camera/`, `sim/models/fly_camera/`, `sim/plugins/chase_camera.cpp`, `sim/plugins/fly_camera.cpp` (fixes only), `sim/tests/test_station.py`, `sim/tests/test_fly_camera.py` |

WS-T2 stays one agent. Its two phases are serial, because both edit `world.py`, and the checkpoint separates the
behaviour change from the appearance change.

**Wave 3 — WS-V Validation, performance and docs** (one agent, serial; it may fix any file, because nobody else
edits):
- Run `pixi run sim-test`, `sim-perf`, `sim-maps`, the realism report and the mission slow tests.
- Tune the (A) values of §5.6 and §6 within their ranges to meet the targets.
- Update `sim/README.md`:
  - Design notes: skid-steer friction, the drivetrain, dig-in;
  - URC worlds: ground catalogue, real terrain;
  - Gazebo lessons: per-contact friction and its slip-compliance semantics; PGS vs Dantzig; Terra roughness; SDF
    fog/ortho/distortion ignored; projector crash; render hooks; no rendering pointers; blocking Python services;
    hfov ≥ 0.1; shadow range; paused worlds; ImageHeightmap scaling and per-PNG `size.z`; GLB Z-up; lidar datum
    shift; `<emitting>` default; particle scatter in depth; RGB-D clip split; GUI Teleop needs no command timeout.
- Set this spec's status to implemented, with a "changes during implementation" section.
- Then an adversarial review.

Order and dependencies: copy the prototype → WS-0 → {WS-D, WS-T1, WS-A, WS-F1} → {WS-T2 (needs D, T1, A),
WS-F2 (needs F1)} → WS-V. An optional **WS-L** (lander speed-up; owns `sim/urc/lander.py` and its tests) can run in
wave 2 if Q2 approves it.

## 13. Risks

| # | Risk | Mitigation |
|---|---|---|
| R1 | The new traction changes what is drivable: the default ground climbs 23° instead of 45°, sand 15–22°. Spin rates fall to 0.18–0.44 of the command, and lower when dug in. Designed routes may become impassable, or intended obstacles passable | Mission slow tests of §11 in phase T2a; WS-V tunes (A) values within range; designs whose intent changes are listed for the user; `track_multiplier` (Q10) |
| R2 | Physics budgets: Delivery is expected at ~1.2× real time with PGS and the drivetrain, and Equipment Servicing is below real time with PGS (M, loaded machine) | Floor 1.1× / target 1.3×; per-world solver (G2, G5); load-source optimisation (G8); lander work (Q2) |
| R3 | DART collision cost of a 4097² heightmap while driving | Idle cost measured (no per-step change); G1 driving run; fallback 2049² |
| R4 | PGS accuracy with 101 lander joints and stacked objects unknown | G2 with a per-world fallback |
| R5 | The drivetrain prototype lives in a session scratch directory that may be gone | Copy it before WS-0 (§12); §6 writes the full algorithm and parameters; the flycam and render prototypes are already in the repo |
| R6 | Traction numbers are generic Bekker/Wong soils plus assumptions; Bekker is unreliable for small wheels; there is no MDRS field data (cone index, shear vane, drawbar or spin tests) | (A) labels; ranges instead of points; per-surface targets that follow the model; a spin-in-place and drawbar test on the real rover would calibrate (Q13) |
| R7 | Relief transfer can leave seams, repeated patterns or wrongly oriented rills, and can spoil designed grades | Feathered random blocks with variance-preserving blending; σ 2 m for badlands with procedural downslope rills; keep-flat masks; grade tests |
| R8 | Memory: +1.4 GB per gz process, and the GUI loads everything again | Budget test; DDS colour map; triangle caps; shared far-field texture |
| R9 | The current workflow renames building blocks this spec refers to. It has already happened: `Media` now lives in `media.py` | Names here are by role; WS-0 re-reads the code and updates §5.8 |
| R10 | Odometry over-reports turns by 2.3–5.5×, which may confuse users and autonomy code | Documented (§6.8); ground truth unchanged; `track_multiplier` (Q10) |
| R11 | The media patch targets gz-rendering 8.2.2's file layout; an env upgrade can break it | Tool fails soft (stock media); diff stored in the repo; test of the fallback |
| R12 | NAIP is pale, hazy and shaded; colours are apparent, not reflectance; not checked against ground photos | De-shading with a measurable test; saturation boost re-tuned on NAIP 2024; ΔE tests split so they are not self-referential |
| R13 | ~400 MB of data under `sim/data` in a repo that is not under git yet | Q3: delete the redundant NAIP 2021 route file (70 MB); optional fetch script from the provenance JSON |
| R14 | Inference, not map: the grey pediment west of MDRS as Mancos Tununk (from soils and stratigraphy [1][2]); Mancos-Shale measurements transferred to Brushy Basin badlands (T) | Only colour, traction and relief recipes depend on it; they come from the soil survey and the local lidar, with literature values labelled (T) |
| R15 | Dig-in is a positive feedback: once Crr·D·c + k_b·D²·a reaches μk·c, a spin stops entirely, a bifurcation like the old stall knife edge | Default D_max kept well below it (sand stops near D ≈ 1.7; default 1.25); the stuck behaviour only behind the strong preset (Q12); a monotonic-decay test |
| R16 | A per-world solver makes per-wheel friction and diagonal stalls differ between worlds | Documented in the sheet; the stall and load-split targets run only in PGS worlds |
| R17 | Parallel agents in one working tree corrupt builds and generated models | D25 isolation; no `sim-test` inside a wave |
| R18 | Slip-compliance side drift on loose ground is large (1.3 m in 3.5 m across a 20° sand slope) and is set by an (A) parameter | Plumbing test exposes it; WS-V tunes slip within range; Q13 field data |

## 14. Open questions for the user

1. **Real terrain for Astrobiology?** Move it onto the real 0.5 m DEM and NAIP at its real coordinates (units
   re-placed on the real banded hills, wash and ledge there)? Or keep today's layout with real micro-relief (the
   default)? Delivery's designed course (ridge and radio shadow) stays synthetic either way.
2. **Equipment Servicing speed**: it runs below 1.3× real time today because of the lander's 101 joints, and PGS
   pushes it below real time. Approve a lander optimisation (for example, fewer moving keys until the arm exists)?
   Or accept Dantzig there, with approximate per-wheel friction?
3. **Data**: delete the redundant NAIP 2021 route-area file (70 MB)? Keep the ~400 MB of DEMs and imagery in the
   repo, or fetch them by script from their provenance?
4. **Wet-weather variant** ("after rain": slick, sticky clay): wanted?
5. **Fly view**: should a gamepad still drive the rover while the keyboard flies? Should Fly and Map be marked
   sim-only and logged by the referee during scored runs (operators at URC see only what the rover sends)?
6. **CC0 photoscanned ground textures** from ambientCG or Poly Haven (not among the approved sources)?
7. **Real drivetrain numbers**, when the mechanical and driver teams have them: motor, voltage, gear ratio,
   current limit, and the motor controller's speed ramp (it sets the turn rise time).
8. **Wheel type** (pneumatic, foam, rigid with grousers): decides μ on rock, the tyre-compliance numbers of D18,
   and whether stick-slip judder is realistic.
9. **May the workflow `git init` the repo**, commit a baseline and give each workstream a git worktree (D25)?
   Without git, it uses private copies and merges owned files back.
10. **Effective-track multiplier**: keep the raw skid-steer response (1.0: the operator gets 0.18–0.44 of the
    commanded yaw rate, and learns real behaviour)? Or compensate like Clearpath, so the commanded rate is roughly
    achieved on packed ground?
11. **Dust in depth and point cloud**: should the rover's depth camera see dust (realistic, may confuse autonomy
    testing)? Default: no.
12. **Dig-in strength**: mild (default: a spin in sand slows to ~0.7× and keeps turning)? Or Anveshak-like (strong
    preset: a sustained spin in sand digs in until the rover cannot turn, and must drive out straight)?
13. **Field calibration**: could the team run a spin-in-place and a drawbar test with the real rover, or a proxy,
    on MDRS-like ground? Ground photos of the route area would also settle Leebench's surface gravel and shrub
    heights.
14. **Leebench surface**: until photos exist, the sim treats Leebench fans as sand sheet and regolith with sparse
    gravel, not desert pavement. Acceptable?

## 15. Sources

1. Clarke & Stoker 2011, "Concretions in exhumed and inverted channels near Hanksville Utah: implications for
   Mars", Int. J. Astrobiology 10(3). https://ntrs.nasa.gov/api/citations/20110023235/downloads/20110023235.pdf
2. NRCS SSURGO, Henry Mountains Area (UT631), via Soil Data Access https://SDMDataAccess.sc.egov.usda.gov/Tabular/post.rest;
   official series descriptions: https://soilseries.sc.egov.usda.gov/OSD_Docs/S/SHEPPARD.html,
   https://soilseries.sc.egov.usda.gov/OSD_Docs/C/CHIPETA.html, https://soilseries.sc.egov.usda.gov/OSD_Docs/L/LEEBENCH.html
   (E horizon gravelly, 20 % gravel, 5–35 % rock fragments, vesicular pores; checked rev2),
   https://soilseries.sc.egov.usda.gov/OSD_Docs/F/FARB.html, https://soilseries.sc.egov.usda.gov/OSD_Docs/H/HANKSVILLE.html
   (tables in `sim/data/research/ssurgo_soils.json`, `mdrs_terrain_measurements.json`; the `cosurffrags` query of
   rev2 returned no surface-fragment rows for mukeys 55112, 55149, 55151, 55156, 55162, 55165, 55166).
3. USGS 3DEP lidar: 2018 UT Southern QL1 0.5 m DEM and DSM, 2020 UT Statewide South point clouds.
   https://tnmaccess.nationalmap.gov/api/v1/products; https://storage.googleapis.com/state-of-utah-sgid-downloads/lidar/southern-utah-2018/QL1/DEMs/;
   https://gis.utah.gov/products/sgid/elevation/lidar/; quality levels https://www.usgs.gov/3d-elevation-program/topographic-data-quality-levels-qls
   (provenance in `sim/data/dem/*.json`).
4. USDA FSA NAIP 2024 via https://apps.geo.fpac.usda.gov/geo-imagery/rest/services/naip/conus_naip/ImageServer
   (listed at https://gis.utah.gov/data/aerial-photography/naip/); NAIP 2021 via
   https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPPlus/ImageServer (provenance in `sim/data/imagery/*.json`).
5. Shibly, Iagnemma & Dubowsky 2005, J. Terramechanics 42:1–13 (Bekker/Wong soil parameters)
   https://fada.birzeit.edu/handle/20.500.11889/7935; Wong, *Theory of Ground Vehicles*, Table 2.3 and the
   Janosi–Hanamoto shear curve (recalled by the drive research, not re-checked).
6. Engineering ToolBox, "Rolling resistance" (car tyres): concrete 0.01–0.015; tar or asphalt 0.02; gravel, rolled
   new 0.02; solid sand, gravel loose worn, soil medium hard 0.04–0.08; loose sand 0.2–0.4.
   https://www.engineeringtoolbox.com/rolling-friction-resistance-d_1303.html (checked rev2; replaces revision 1's
   Wikipedia citation, which does not list these values).
7. Golombek et al. 2003, JGR 108(E12) 8086, rock size-frequency model. https://ntrs.nasa.gov/citations/20030111177
   (not used for slabs in rev2).
8. Dohrenwend 2005, "Accelerated erosion in Mancos Shale badlands disturbed by OHV activity, Caineville, Utah",
   GSA Rocky Mtn. Section. https://gsa.confex.com/gsa/2005RM/webprogram/Paper86764.html (Mancos Shale: crust relief
   1.5 in, rills 3.1 in, mantle 1.9 in; transferred).
9. Godfrey & Everitt 2005, "Episodic movement of sediment off Mancos Shale slopes near Hanksville", GSA.
   https://gsa.confex.com/gsa/2005RM/webprogram/Paper86308.html (35–40° slopes, Tununk/Blue Gate; transferred).
10. Chan et al. 2011, "Utah's geologic and geomorphic analogs to Mars", GSA SP 483 https://pubs.usgs.gov/publication/70201979;
    UGS photo of the day https://geology.utah.gov/potd-september-28-2016-san-rafael-swell-near-muddy-creek-emery-county/;
    UGS GeoSights, Bentonite Hills https://geology.utah.gov/map-pub/survey-notes/geosights/geosights-bentonite-hills/
11. Belnap et al. 2001, *Biological Soil Crusts: Ecology and Management*, BLM TR 1730-2. https://www.ntc.blm.gov/krc/uploads/231/CrustManual.pdf
12. Sharp 1963, "Wind ripples", J. Geology 71:617–636.
13. URC 2027 Requirements and Guidelines (rules 1.c.ii, 1.e.xii, 3.d.iii, 3.d.iv). https://urc.marssociety.org/home/requirements-guidelines
14. Team Anveshak, URC 2017 report (45.6 kg rover: in a zero-radius turn in loose soil a drive motor stalled and
    the articulation joint bent; the wheels got stuck as they dug into the soil at slow speed).
    https://joyofgiving.alumni.iitm.ac.in/data/utilreports/Team%20Anveshak%20-%20URC%202017%20Report%20(1)%20(2).pdf
15. Shamah 1999, "Experimental comparison of skid steering vs. explicit steering for a wheeled mobile robot", CMU MS
    thesis. https://publications.ri.cmu.edu/experimental-comparison-of-skid-steering-vs-explicit-steering-for-a-wheeled-mobile-robot
16. Mandow et al. 2007, "Experimental kinematics for wheeled skid-steer mobile robots", IROS (`papers/SkidDrive.pdf`):
    P3-AT χ 0.69–0.76 and α 0.90–0.95, so a yaw ratio α·χ of 0.63–0.71 on asphalt and concrete.
17. Baril et al. 2020, arXiv:2004.05131, skid-steer on sub-arctic terrain (`papers/SkidOnSubArctic.pdf`).
18. Husky: motor and linear-graph model, McCormick et al., arXiv:2110.00323 https://arxiv.org/pdf/2110.00323 (its
    0.541 is a coefficient in a motor-voltage formula, not a measured yaw ratio); Clearpath `husky_control`
    `wheel_separation_multiplier: 1.875`
    https://raw.githubusercontent.com/husky/husky/noetic-devel/husky_control/config/control.yaml (checked rev2);
    Clearpath Husky A200 manual (wheelbase 512 mm, track 555 mm)
    https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a200/user_manual_husky
19. Czapla, Fice & Niestroj 2022, Sci. Rep. 12:16015 (stick-slip at large slip angles). https://pmc.ncbi.nlm.nih.gov/articles/PMC9512802
20. Ishigami et al. 2007, J. Field Robotics 24(3):233–250 (side force with sidewall bulldozing).
21. Toupet et al. 2020, J. Field Robotics 37:699 (`papers/Inverse3d.pdf`).
22. IMS gear datasheets https://www.imsgear.com/en/product-detail/ims.45-promax, https://www.imsgear.com/en/product-detail/ims.22-promax;
    REV NEO https://www.revrobotics.com/rev-21-1650/
23. OpenStax College Physics, Table 5.1 (rubber on dry concrete μs 1.0 / μk 0.7) https://pressbooks.online.ucf.edu/phy2053bc/chapter/friction/;
    Burckhardt tyre model via https://arxiv.org/pdf/2211.10336
24. Chiodini et al. 2026, small-wheel sand testbed (adherence recovery 0.03–0.72 s). https://iris.unime.it/handle/11570/3355491
25. ODE solver notes (Dantzig/LCP friction limits). https://mirror.umd.edu/roswiki/physics_ode(2f)ODE.html
26. gz-rendering 8 (`Ogre2RenderEngine.cc`, `Ogre2Heightmap.cc`, `Ogre2Material.cc`, `ParticleEmitter.hh`)
    https://github.com/gazebosim/gz-rendering/tree/gz-rendering8; gz-sim 8 (`RenderUtil.cc`, `SceneManager.cc`,
    `Physics.cc`, `SdfEntityCreator.cc`, `Sensors.cc`, `LensFlare.cc`, `System.hh` `ISystemReset`)
    https://github.com/gazebosim/gz-sim/tree/gz-sim8; gz-sensors 8 `CameraSensor.cc`, `RgbdCameraSensor.cc`
    https://github.com/gazebosim/gz-sensors/tree/gz-sensors8; gz-gui 8 `Teleop.cc` (publishes only on key or button
    events); sdformat 1.9 `particle_emitter.sdf` (`emitting` default true, `particle_scatter_ratio` 0.65) — read in
    the pixi env and sources by the critiques.
27. Desert pavement definition, GSA 2002 https://gsa.confex.com/gsa/2002AM/webprogram/Paper46340.html; Greeley et
    al. 1991, topographic roughness https://ntrs.nasa.gov/api/citations/19920001583/downloads/19920001583.pdf
28. Munsell renotation data (RIT). https://www.rit-mcsl.org/MunsellRenotation/real.dat
29. Westenberg & Nelson 2010, EPSC2010-895, "The Mars Desert Research Station". https://meetingorganizer.copernicus.org/EPSC2010/EPSC2010-895.pdf
30. Stoker et al. 2011, Int. J. Astrobiology 10(3), MDRS soil cores (phyllosilicates and sulfates).
31. Mars Society Canada, "Diaries from analog Mars, part 8" (mud on EVAs). https://www.marssociety.ca/2021/10/12/diaries-from-analog-mars-part-8-would-you-do-that-for-your-breakfast/
32. Iowa State news on URC terrain (thick, sometimes wet sand; ridgeline), as summarised in search results; the page
    itself returned an error. https://news.engineering.iastate.edu/?p=26724
33. Meirion-Griffith & Spenko 2013, "A pressure-sinkage model for small-diameter wheels on compactive, deformable
    terrain", J. Terramechanics 50(1):37–44 (flat-plate assumption fails below ~50 cm wheel diameter); Meirion-Griffith,
    Nie & Spenko 2014, "Development and experimental validation of an improved pressure-sinkage model for
    small-wheeled vehicles on dilative, deformable terrain", J. Terramechanics. https://istvs.org/publication-news/2012/7/3/a-pressure-sinkage-model-for-small-diameter-wheels-on-compactive-deformable-terrain;
    https://www.istvs.org/publication-news/2014/1/15/development-and-experimental-validation-of-an-improved-pressure-sinkage-model-for-small-wheeled-vehicles-on-dilative-deformable-terrain
    (abstracts read rev2). The critique also named a 2011 J. Terramechanics 48:149–155 paper by the same authors,
    which this revision could not verify.
34. Wong & Preston-Thomas 1983, "On the characterization of the shear stress–displacement relationship of terrain",
    J. Terramechanics 19(4):225–234 (exponential curve for loose granular soils, peaked curve for dense/crusted ones;
    bibliographic details checked rev2, content as summarised by the critique and Wong's textbook).
35. Lyasko 2010, "Slip sinkage effect in soil–vehicle mechanics", J. Terramechanics 47(1):21–31 (title and venue
    checked rev2).
36. Ding, Gao, Deng & Tao 2010, "Wheel slip-sinkage and its prediction model of lunar rover", J. Central South
    University of Technology 17(1):129–135: "the sinkage with slip ratio of 0.6 is 3–7 times of the static
    sinkage". https://journal.hep.com.cn/jocsu/EN/10.1007/s11771-010-0021-7 (abstract read rev2).
37. DART `dart/dynamics/ShapeFrame.hpp` ("Slip compliance parameters act as constraint force mixing (cfm) for the
    friction constraints") and gz-physics 7 `ContactProperties.hh` ("Force-dependent slip coefficient"), in the pixi
    env (read rev2).
38. gz-math 7 `DiffDriveOdometry.hh` and `SpeedLimiter.hh`, in the pixi env (read rev2).
39. Rankine passive earth pressure, K_p = (1 + sin φ)/(1 − sin φ) (standard soil mechanics; the lower bulldozing
    estimate uses φ 30°, K_p 3).
40. Wong, *Theory of Ground Vehicles*: bulldozing resistance R_b = b(0.67·c·z·K_pc + 0.5·z²·γ·K_pγ) (recalled, not
    re-checked; the rev2 estimate uses N_γ ≈ 20 at φ 30°, K_pγ ≈ 53, γ from Sheppard's bulk density 1.55 g/cm³ [2]).
41. Teillet, Guindon & Goodenough 1982, "On the slope-aspect correction of multispectral scanner data", Canadian
    J. Remote Sensing 8(2):84–106 (C-correction; recalled, not re-checked).

## 16. Review

Two critiques of revision 1 ran read-only on 2026-10-06: **P** (physics and data fidelity) and **E**
(feasibility and engineering). Each finding is listed below with its verdict. **Accepted** means the critique's
diagnosis and fix were taken. **Accepted, different fix** means the diagnosis was taken but the fix differs.
**Rejected** means neither was taken. Numbers re-checked in this revision are marked "verified rev2".

### 16.1 Physics and data fidelity (P)

| # | Finding | Verdict | What changed, and why |
|---|---|---|---|
| P1 | The single 0.38–0.48 spin band contradicts the spec's own rolling resistance; regolith 0.377, sand 0.318, wash 0.284 | Accepted | Per-surface targets from the closed form μk(c·sx − a·sy)/\|s\| = Crr·c + k_b·a, ±0.04 (§5.6, §6.9). Verified rev2 (`spin.py`: same values with revision 1's table). The lower rates on soft ground serve the user's "not so smooth" |
| P2 | The lat_ratio ellipse leaves the spin rate unchanged | Accepted, different fix | Verified rev2 (0.4414 at λ = 1.2). lat_ratio is dropped. Sideways soil resistance becomes a bulldozing hub force along the axle (D5, §6.5), following Ishigami's split of the side force into shear and sidewall bulldozing [20]. It does enter the moment balance (sand 0.318 → 0.261). The critique's wheel-aligned anisotropic box (μ1 = μ·\|d_x\|, μ2 = λ·μ·\|d_y\|) was not adopted: it was never prototyped, while the slip-aligned circle reproduced the closed form (0.42–0.44 vs 0.441) |
| P3 | The slip-compliance floor of 0.1 m/s makes parked rovers creep, and the side-slope target would fail | Accepted, one part rejected | Real wheel speed with no floor, and zero compliance while sticking (D23, §6.4); parked-on-slope tests (§6.9); side-slope drift target re-derived per surface. The semantics were verified rev2 in the DART and gz-physics headers. **Rejected for phase 1**: separate longitudinal and lateral compliances. With fdir1 along the slip, DART's second direction is perpendicular to the slip, not the axle, so a wheel-frame split needs the untested wheel-aligned formulation. One isotropic compliance is (A) |
| P4 | Sand slip 20 % and torque 3.4 N·m restate inputs | Accepted, one part rejected | Relabelled as plumbing (§6.9). **Rejected**: deriving every slip value from a Janosi curve. For sand it gives a bracket of 11–31 % (rev2 calculation with l 8.1 cm, K 1–2.5 cm; the critique got 5–13 %). For firm soils it gives 12–29 % slip on flat loam, implausible for a tyre's longer deflected contact. Slip stays (A) (§5.6) |
| P5 | The 10 A stall is a static-friction knife edge, with non-monotonic results | Accepted | Stall/no-stall at 0.7× and 1.3× the analytic loaded-wheel current (9.7 A: verified rev2), plus a monotonicity test (§6.9). The sticking branch is now a box aligned with the expected load, so breakaway follows the circle (§6.4) |
| P6 | Rise and stop times come from the assumed ramp | Accepted | The ramp is labelled a controller assumption (§6.3, Q7), implemented with gz-math `SpeedLimiter`. Rise is a plumbing test with the ramp and a physics test without it (< 60 ms; prototype 24 ms) |
| P7 | The slow-turn judder target needs the deferred tyre compliance | Accepted | Re-baselined without tyres: peak-to-peak/mean ≥ 0.8 in 0.5–5 Hz (prototype 1.26 at 0.8 Hz). The with-tyres figure (4.6 at 1.8 Hz) moves to phase 2. Sand must not judder (μs = μk) |
| P8 | Turning on objects regresses to today's bug | Accepted | Every wheel contact gets the friction circle. μ comes from the object's explicit SDF friction or the `manmade` surface (§6.4). Tests on the landing pad, a step top and a μ 0.2 tile (§6.9) |
| P9 | "2–10× too smooth" is a selection artefact | Accepted | Headline corrected (§0, §1); world values verified rev2 (Delivery 0.84/2.7/7.5, Astrobiology 0.64/1.7/4.1, Autonomy 1.29/3.1/5.4 cm). Targets are now world-level distributions plus per-type values from painting the real Autonomy square by the §5.3 rules, computed in this revision (§5.4, `types_rms.json`), with WS-T1 re-deriving them (`terrain_targets.py`) |
| P10 | Block cover of 9–16 % includes features of 7 m or more; the curve is not a power law; golombek_k_fit is degenerate | Accepted | Verified rev2 from the JSON (1–7 m cover 9.2 % and 8.0 %; ≥ 7 m 6.5 % and 7.6 %; k_fit 0.6 in 8 of 9 windows). Slabs use the tabulated measured counts at 1–7 m, an (A) extrapolation below 1 m, and an 8–9 % cover target; ≥ 7 m features are macro shape or risers (§5.5). The Golombek statements are dropped |
| P11 | Shrubs come out too tall; relative NDVI does work | Accepted | Sand-sheet shrubs ≤ 0.3 m; diameter (A); wash-margin shrubs 0.5–2.5 m from the lidar (verified rev2: 7 % cover, p50 0.85 m). Detections are filtered by the local NDVI anomaly. D10 and §4 now say absolute NDVI is unusable and relative NDVI usable |
| P12 | Leebench "fan pavement", dry-clay μ and the Stribeck ratio on loose soil lack support | Accepted | (a) Leebench is no longer fan pavement. Verified rev2: OSD 20 % gravel, 5–35 % range; SSURGO E horizon 92.5 % passing No. 10; and a new Soil Data Access query found no surface-fragment rows at all (§5.3). (b) Dry powder μ raised to 0.45, with Crr 0.15 and sinkage 2 cm; low μ only in the wet variant. (c) μs = μk on sand, wash sand and powder [34] |
| P13 | The Husky "0.541 measured" is a voltage-formula coefficient | Accepted | Verified rev2 (eqs. 24–25 of [18]; Clearpath multiplier 1.875). D3 cites 0.533 vs 0.540, and α·χ 0.63–0.71 for the P3-AT. The effective-track point became D24 and Q10 |
| P14 | Bekker sand DP/W is 0.41, not 0.45; the 0.85 derating is unsourced | Accepted | DP/W shown as 0.41–0.45; sand μk 0.52 (was 0.55). The derating stays (A), with its direction supported by Meirion-Griffith & Spenko [33]. The 2013 and 2014 papers were verified; the critique's 2011 citation was not |
| P15 | Mancos-Shale literature transferred to Brushy Basin; 2 m RMS at the noise floor; "~10 % over 30°" | Accepted | (T) labels on [8] and [9] values; "1–5 cm popcorn thickness" now (A); 2 m targets dropped; haystack target now ~6 % over 30° (5.9 % measured, verified rev2) |
| P16 | Swatch feathering lowers RMS by 10–20 % | Accepted | Variance-preserving blend, divided by sqrt(Σw²) (§5.4), with a test |
| P17 | The diagonal load-split row mixes μ values | Accepted | Row split by μ (§6.9); the pivot-height note for the mechanical team is in §6.8 |
| P18 | Citation gaps: Crr table, μ noise, NAIP drape, PGS noise | Accepted | (a) [6] replaced by Engineering ToolBox, verified rev2: values and their car-tyre caveat. (b) μ noise (A). (c) NAIP is de-shaded before draping, and the ΔE tests are no longer self-referential (§5.7, §11). (d) PGS noise is not counted as realism (D4, §6.6) |
| P19 | Anveshak evidence does not match the failure mode the model produces | Accepted | A per-wheel dig-in state (D21, §6.5), supported by slip-sinkage data [35][36]. Defaults are mild; the stuck behaviour is a preset (Q12). The model reproduces wheels digging in and turns slowing or stopping. It does not reproduce a motor stall in sand at 20 A, which depended on Anveshak's unknown motors |
| P-rec | Expose an effective-track multiplier; tell the mechanical team about pivot height | Accepted | D24, Q10, §6.8 |

### 16.2 Feasibility and engineering (E)

| # | Finding | Verdict | What changed, and why |
|---|---|---|---|
| E1 | PGS and drivetrain costs are understated; the budgets are at risk | Accepted | Revision 1's +6 %/+5 % retracted; the critique's CPU-time numbers adopted (§10.1–10.2). Load-robust measurement method (§10.3). Budget floor 1.1× and target 1.3×. Solver per world (D4, §6.6). Load-source optimisation gate G8 |
| E2 | Turning stalls on contacts the callback does not map (pad, C2, steps, ledges, wave-1 tiles); the `block_` prefix matches nothing | Accepted | As P8. `ground.json` gets an exact-name collision map written by WorldBuilder (steps, ledges and tiles), plus prefixes and defaults (§9.1) |
| E3 | Default-ground sinkage carves the highest point and breaks world generation | Accepted | Each heightmap PNG is normalised to its own maximum with its own `<size>` z; the ValueError goes; the test checks decoded heights; `place()` uses the collision surface (§5.8). The critique's alternative (carve only ≥ 1 cm, never at the maximum) is unnecessary with this fix |
| E4 | Parked rovers creep; the hold angle depends on heading | Accepted | D23; the sticking box is aligned with the previous tangential force, with μ2 = 0.3·μ (§6.4); tests at four headings (§6.9) |
| E5 | The far field and haze are clipped out of every station view | Accepted | Eye, chase, fly and rover RGB clip far 80 km; depth stays at 40 m via `<depth_camera><clip>` (D14, §7); a render test that the Henry Mountains are not sky |
| E6 | A rover-model change in wave 1 crosses ownership; the gate can deadlock; no DiffDrive for the cost regression; simulate.py lacks overrides | Accepted | `DriveParams.mode` with a byte-identical default in wave 1, flipped by WS-T2 in phase T2a (D22, §12); DiffDrive kept as a variant; simulate.py gains rover and Params overrides (WS-0); `Params.wheel_speed` and `wheel_radius` kept for the station |
| E7 | Shared working tree with no git; generated outputs and the build dir are unowned | Accepted | D25: worktrees after `git init` (Q9) or private copies, with per-copy build dirs; generated outputs only in the owner's copy; no `sim-test` inside a wave (§12) |
| E8 | Ownership gaps: media.py, TerrainType fields, NAIP reader, fly model in gen_model, WS-T2 load | Accepted; one fix rejected | media.py → WS-A; WS-0 freezes the dataclasses, adds `dem.read_raster` and splits `viewers.py` (owned by WS-F1, then WS-F2); props.py not edited in wave 1. **Rejected**: splitting WS-T2 into two agents, because both halves edit `world.py`. Instead, T2 runs as two serial phases with a checkpoint |
| E9 | Dust emitters default to emitting; depth sees particles | Accepted | `<emitting>false</emitting>`, explicit `<topic>`, `particle_scatter_ratio` 0 (D15); depth-under-dust test; Q11 |
| E10 | `cmd_timeout` 0.5 s breaks GUI Teleop | Accepted | Default 0 (hold the last command, like DiffDrive); the station keeps its own deadman (§6.2); test |
| E11 | G1 is low risk for collision; Delivery and Astrobiology at 2049² are not gated | Accepted | G1 idle result recorded (§10.2), with a driving run added; new gate G7 for 2049² |
| E12 | The slab recipe cannot reach its own cover target | Accepted | As P10 |
| E13 | Recipe slabs double-count blocks already in the real DEM | Accepted | Real-DEM worlds get recipe slabs of D < 1 m only, and risers only where the DEM has no step (D9, §5.5) |
| E14 | The NAIP ΔE ≤ 8 acceptance is mostly used up by the deliberate boost; the prototype was tuned on NAIP 2021 | Accepted | Two ΔE ≤ 5 tests: render vs colour map, and colour map vs boosted NAIP. Render vs raw NAIP is reported only. The boost is re-tuned on NAIP 2024 (§5.7, §11) |
| E15 | Reuse gz-math DiffDriveOdometry and SpeedLimiter; two-language lookups; duplicated environment setup; world_copy | Accepted | §6.3; golden cross-language test (§11); `sim/gzenv.py`; `render_map.py` uses `tests/worldfiles.py` |
| E16 | The contact callback checks every contact in the world; the cost grows with contacts | Accepted (informational) | Recorded in §10.2 and folded into G8 |
| E17 | Station frame rate is tied to RTF; it is only checked in wave 3 | Accepted | Gate G6 in WS-0, with the budget stated as min(render throughput, 20 Hz × RTF); the LensFlare disconnect is checked |
| E18 | A media patch failure would break the build; sim-worlds inputs pull in 107 MB | Accepted | Soft failure with a fallback test (D12, §7); inputs narrowed (§12) |
| E19 | The drive prototype still exists; copy it now | Accepted | The copy is the first step of §12, done by the orchestrator. This review agent could only edit this spec |
| E20 | Smaller gaps: wall-clock deadman in tests, drivetrain reset, joint velocity limit below free speed, per-world far-field size, map staleness hash | Accepted | `deadman_clock` (§8.1); `ISystemReset` for both plugins; joint velocity limit 16 rad/s (§6.2); shared far-field texture (§7); the map hash covers all inputs (D17, §8.2) |

### 16.3 What this revision measured or checked itself

- **Spin-in-place ratios.** The quasi-static moment balance for every catalogue type, with and without
  bulldozing and dig-in (`spin.py`, `table.py`, `dig.py`). The ratio does not depend on the load split.
- **World roughness.** Today's world heightmaps re-run (same numbers as critique P).
- **Per-type real roughness.** The real Autonomy square painted by the §5.3 rules (`types_rms.py`): the SSURGO
  polygons rasterised on the 0.5 m lidar grid, slope from σ 2 m smoothing, windows ≥ 80 % one type. In the square:
  Sheppard-Leebench 50 %, Chipeta-Badland 26 %, Badland-Rock outcrop 24 %, and no Farb or Green River polygons.
- **Surface fragments.** A Soil Data Access query of `cosurffrags` for the seven local map units: no rows.
- **Leebench.** The OSD's surface description re-read.
- **Citations re-checked.** Engineering ToolBox rolling-resistance table; Clearpath `control.yaml`;
  Meirion-Griffith & Spenko; Wong & Preston-Thomas; Lyasko; Ding et al. (abstract: 3–7× sinkage at slip 0.6).
- **Engine and library sources.** The DART and gz-physics slip-compliance comments and the gz-math
  DiffDriveOdometry and SpeedLimiter headers, read in the pixi env.
- **Bulldozing estimate.** Rankine passive pressure vs Bekker's bulldozing formula at 2 and 3 cm sinkage.
- **Analytic loaded-wheel current on a μ 0.8 slab.** 9.7 A, matching the prototype's 9.7–9.8 A.
- **Not run in Gazebo.** Parked-rover creep under revision 1's floor, the dig-in dynamics and the bulldozing
  force are analytic predictions. WS-D's tests are the first runs.

## 17. Changes during implementation

Implemented 2026-10-06 to 2026-10-07. Each workstream reported its deviations; the wave gates merged them; WS-V
(wave 3) ran the full suite, the performance budgets, the maps, the realism report and the mission slow tests,
tuned what the targets needed and wrote this section. Measurements are on the M4 of §10.1, (M) as before.

### 17.1 The user's decisions (2026-10-06), as built

| Q | Decision | Built as |
|---|---|---|
| Q1 | Astrobiology keeps its layout, with real micro-relief | As decided (relief swatches, badland belts, slabs, risers) |
| Q2 | Equipment Servicing on Dantzig, approximate per-wheel friction, no lander optimisation | Dantzig there (gate G5: PGS 0.86× by CPU time). The lander's collisions got collide bitmask `ABOVE_GROUND` (17.2): a filter, not an optimisation of the lander, but listed for the user to confirm |
| Q3 | Keep all data | Nothing deleted; the far-field rasters moved to `sim/data/dem` and `sim/data/imagery` |
| Q4 | No wet variant | Not built |
| Q5 | In Fly a gamepad still drives; Fly/Map marked sim only; no referee logging | As decided |
| Q6 | No CC0 texture downloads | Procedural detail textures |
| Q7 | Placeholder motor numbers, D20 | Gear 50, efficiency 0.8, 20 A, all in `gen_model.DriveParams` |
| Q8 | Wheel type unknown | Spec's assumption; tyre compliance off |
| Q9 | git worktrees | One per workstream |
| Q10 | Raw turn physics, `track_multiplier` 1.0; the station shows commanded vs achieved yaw rate | As decided (station Drivetrain panel and "got N %") |
| Q11 | Depth and point cloud do not see dust | **Not achievable** in gz-rendering 8.2.2: `particle_scatter_ratio` has no effect on depth (dust shows at 1e-6, 0.65 and 1.0; 0 is ignored). The emitters sit behind the rear wheels, so a forward camera rarely sees it. `test_render` keeps the target as an expected failure |
| Q12 | Strong dig-in by default, mild available | Strong on loose sand, wash sand and dusty clay; mild under a switch (`terrains.DIG`). The crusted sand sheet keeps mild values under both (17.2) |
| Q14 | Leebench as sand sheet + regolith with sparse gravel | As decided (`landscape.SSURGO_UNITS`) |

Folded-in known issues: the rover RGB-D is 1280×720 (a 20 cm tag decodes to 5 m square on; 2.5 m before); the
drivetrain has a 0.5 s command timeout on the wall clock (overrides E10's 0; Gazebo GUI Teleop's button mode
needs `cmd_timeout` 0); every world's origin altitude is ellipsoidal through one helper (`world.site`, synthetic
worlds dropped 20.9 m); `props.shrub` is deleted.

### 17.2 Changes to the design

**Drivetrain and contact (§6).**
- Slip compliance applies in both friction branches (§6.4, D23 said zero while sticking): the contact of a wheel
  rolling under traction counts as sticking, so the literal rule gave 0.0 % slip in sand instead of 20 %. A
  stopped wheel still has none: parked rovers move ≤ 1.2 mm in 60 s on 15° regolith and 20° sand.
- The contact rule reads the wheel's spin through the controller's 5 ms speed filter: with the raw joint speed,
  parked wheels chattered at 250 Hz (±0.1 rad/s) and a rover parked across 29° regolith crept at 2 cm/s.
- `rr_w0` 0.05 rad/s (7.5 mm/s at the hub) instead of 0.2: at 0.2 a rover dug in by the strong preset kept
  creeping round at 0.25× the fresh rate instead of stopping.
- Hub forces act in the contact plane (axle × contact normal); the dig factor D is the state and carries over
  between ground types; odometry uses the mean of each side's wheel angles.
- Wheel loads from `JointTransmittedWrench` (gate G8: within 2.6 % of contact loads, cheaper); no
  `<load_source>` element and no `ContactSensorData`.
- The strong preset is the catalogue's own values (`terrains.DIG` is the one switch, read by `ground.json` and
  by the rover's surface rows); the plugin's gain knobs were removed. It applies to loose ground only (sand,
  wash sand, dusty clay). WS-V gave the crusted sand sheet its mild values under both presets: with the strong
  ones a pure-pursuit driver's corrections dug the rover in on 13° of it, and it is half the ground of every
  synthetic world; the user's decision named loose sand.
- WS-V retuned the PI integral `ki` 40 → 160 V/rad (integral time 25 ms, (A)): the turn without the ramp rose to
  90 % in 118 ms, the IR drop under load waiting on a 0.1 s integral; now 53 ms (target < 60 ms). Every other
  §6.9 row was re-run with it.
- Washboard target re-baselined (17.4): the torque std the wheels need to roll the rover over ±4 cm crests is
  N r × steepest slope / √2 = 3.8 N·m; the < 1.5 N·m target came from the prototype on the proving ground's
  corrugated heightmap, into which wheels sink (below). The chatter the target was after is the ripple above
  5 Hz, 0.8–1.0 N·m.
- Command timeout default 0.5 s, wall clock (the user's issue; E10's 0 for GUI Teleop is a parameter).
- No SDF camera noise: `<noise>` on an `rgbd_camera` aborts gz on Metal (Ogre RenderingAPIException).
- `rover_test.sdf` and the test worlds of `simulate.py` stay on Dantzig, on which `test_drivetrain` was
  calibrated (§6.6 suggested PGS there). The DiffDrive variant is no longer byte-identical to the wave-0 rover
  (§11): since T2a every rover carries the HD camera and the dust emitters.

**Ground (§5).**
- Friction tiles are deleted entirely, and `features.Patch` lost `level` and `falloff`: zones only paint.
- Terrain targets re-derived over both lidar squares (`terrain_targets.json`); the slickrock swatch is cut with
  σ 2 m, not 8 m (window H's 8–32 m relief is its ledges; no amplitude fitted at 8 m).
- Slab targets are the measured tables less the features of 7 m and more (§1); measured N(≥4) is −17 % against
  that, −42 % against the raw table.
- Wash channels have banks of at most 15° (`features.WASH_BANK`; Delivery's wash grew from ~28 to ~55 m wide),
  no micro-relief, and soft `sand` floors: on `wash_sand` under the strong preset a dug-in rover climbs 1°.
- Delivery and Astrobiology got badland belts (7 and 8 zones of 50–110 m, off the course and the units) to reach
  the real square's rough tail; synthetic faces over 20° paint `badland_slope` (rock has no relief recipe);
  Delivery's stage-1 sand flat is kept flat and its relief halved near the start; Equipment Servicing's ground
  is clay crust with sand-sheet patches.
- Autonomy's easy route crosses ground steeper than 15° on a 12 m rib of caprock (`EASY_ROUTE_RIB`, traction
  only): the soil map's ground there climbs 22° and the rim is 23–26°. Delivery's clay flank stops a straight
  climb by dig-in after 10 m, not by sliding.
- Terrain shapes carry collide bitmask `GROUND`, the lander's parts `ABOVE_GROUND`: a heightmap's bounding box
  spans its height range, so relief that raised Equipment Servicing's top from 1.1 to 2.5 m put ~140 lander
  links into the heightfield narrowphase (+9 % per step).
- WS-V: the proving ground's washboard rides a collision mesh (`WorldBuilder.surface_mesh`, the collision
  heightmap dipped 0.2 m under it): on the corrugated heightmap alone DART's wheels sink 5–15 cm and the rover
  stalled 1–4 m in, with either drivetrain. It now crosses at ~0.45 m/s.
- Clutter budgets: pebbles thinned to 20,000 per world; recipe gravel only in corridors of the course.

**Appearance (§5.7, §7).**
- No detail decals and no alpha-cut shrub cards (gate G4: alpha is tested at 0.5, no feathering).
- NAIP's sun elevation comes from the acquisition date: fitted, its vertical component came out negative.
- Detail textures are rescaled to the colour map's mean before compensation (shared, unscaled ones clipped 78 %
  of Autonomy's texels; now 1–6 %).
- The NAIP 2024 boost was not re-tuned (×1.25 saturation, ×1.1 contrast, (A)): there are no ground photos to
  tune against (Q13).
- WS-V raised the sun's intensity 1.4 → 1.9: at 1.4 the rendered maps came out at 0.77 of their colour maps
  (CIE76 8.0–8.5, failing §1's ≤ 5); at 1.9 sunlit flat ground renders at 1.00 of its colour map and the median
  CIE76 is 2.1–2.4. The ambient barely moves it (0.32 → 0.50 gave 0.83).
- Far field 60.6 × 79.7 km (D14 said 65 × 80).

**Fly camera, map and station (§8).**
- The fly camera's floor samples its whole next 0.5 s along both the velocity and the target velocity, with a
  lag of min(0.1 s, time to arrival / 4): the two-point rule reached the 0.3 m hard floor at 16× cruise over a
  30 m hill (now 0.995 m clearance).
- The orthographic window is tied to the height above ground where orthographic began, so panning keeps the
  scale and climbing zooms out; §11's "orthographic object size equal at 40 and 80 m" test was replaced by a
  projection check (box-top width 22 px against 23.4 expected).
- `SetWorldPoseCmd` is sent every step (sent only on change, motion started 5 steps late).
- The WebSocket `fly` message carries held key codes and the right stick (`{keys, axes}`), mapped in Python
  (`drive.fly_command`, unit-tested), not `{move, turn, fast}`.
- The station computes the turn rate from successive ground-truth yaws: OdometryPublisher's twist spiked to
  −628 rad/s (4π/dt) once in 1233 messages.
- The ChaseCamera terrain floor was built; the lens flare was not.

**Performance and tests (§10, §11).**
- CPU time per step counts gz's helper threads and reads 13–29 % below the wall-clock real-time factor an idle
  machine reaches. `test_perf` judges the §10.3 floor by wall clock and prints both (WS-0's open question,
  decided by WS-V: the wall clock is what an operator gets; CPU time stays the measure for ratios).
- `tests/test_realism.py` (no GPU) and `tools/realism_report.py` (GPU, `pixi run sim-realism`) share
  `urc/realism.py`; the report writes `sim/data/research/realism_report.json` and
  `realism_contact_sheet.jpg`.
- The mission slow tests drive with pure pursuit on ground truth that never turns in place (arcs of ≥ 1.5 m
  radius: a spin in sand digs in, which is intended), on routes planned on a 1 m grid that treats digging
  ground as half dug in; Autonomy's easy route with a 4 m look-ahead. `pixi run sim-slow` runs them;
  `pixi run sim-perf` adds the drivetrain cost test.

### 17.3 Status against §1 ("Done when")

| Goal | Verdict | Measured (2026-10-07) |
|---|---|---|
| Roughness per type within the real p25–p75 at 4/8/16 m | **Met** | Every painted type with enough windows in Delivery, Astrobiology and Equipment Servicing (sand sheet, badland slope, block field, slickrock, clay crust): e.g. sand sheet 2.03/4.85/10.26 cm against 1.55–4.08/3.17–7.89/5.47–15.26; badland 6.5/18.9/48.1 against 4.4–10.5/13.3–28.0/36.7–67.4 |
| Each synthetic world's natural ground p50 and p90 within ±30 % of the real square | **Met** for Delivery and Astrobiology; **not met** for Equipment Servicing, by design | Delivery −18/−26 %, −7/−22 %, +4/−22 % (p50/p90 at 4/8/16 m); Astrobiology −4/−16 %, +7/−16 %, +20/−21 %; Equipment Servicing −32/−51 %, −20/−56 %, −12/−66 %: rule 1.d's "relatively flat" site (4 m p50 1.82 cm, §5.2 asks ≤ 2 cm) |
| Block fields: N(≥1), N(≥2), N(≥4) within ±30 %, 8–9 % cover by 1–7 m blocks | **Met** (against the table less features ≥ 7 m) | 1.69/0.81/0.14 per 100 m² against 1.67/0.74/0.17; cover 8.3 %; badland edges 0.54/0.28/0.074 against 0.55/0.27/0.074 |
| Sand: fresh spin 0.26 ± 0.04 | **Met** | 0.248 (closed form 0.261) |
| Sand: a sustained spin slows to 0.6–0.8× (§6.5 mild) | **Replaced by the user's strong default, met**; mild met | Strong: 0.145 in the first second, 0.018 (0.07×) from the third: the rover cannot turn and drives out at ~0.22 m/s of 0.3 commanded. Mild: 0.19–0.22, 0.74× |
| Wheels sit 2–3 cm into sand on screen | **Met** (by construction, checked in physics) | Collision carve 2 cm under sand, 3 cm under wash sand: a parked rover rests 0.030 m lower on wash sand than on the visual surface, 0.025 m lower than on regolith |
| Dust scales with speed, slip and surface | **Met** for the rate; dust in depth not avoided (Q11) | Rate test: sand > 3× rock; one "off" when stopped |
| Autonomy from the 0.5 m lidar and NAIP 2024 | **Met** | 4097² over 2048 m; de-shaded NAIP 2024 |
| Rendered map vs its colour map, smoothed CIE76 ≤ 5 | **Met** (after WS-V's sun change) | Median 2.35 Delivery, 2.34 Astrobiology, 2.11 Equipment Servicing, 2.44 Autonomy; p90 2.9–4.6 |
| Colour map vs boosted NAIP ≤ 5 outside de-shaded and inpainted ground | **Met** | Median 0.52 on slopes < 5°, 0.72 at 5–10°, 1.17 at 10–20°, 2.28 over 20° (p90 6.3 there: the de-shading) |
| Every ground type carries traction with sources or (A); Autonomy's ground from SSURGO | **Met** | `terrains.py`; Autonomy's ground shares (sand sheet, clay crust, silt flat, badland slope, rock, sand) 47.40/21.20/14.98/11.88/2.74/1.67 % against the soil map's 47.47/21.20/14.98/11.88/2.74/1.65 % |
| §6.9 met | **Partly**: every row but the slow-turn judder (17.4) | |
| Terrain no longer mirror-like | **Met** | `test_render`: brightest 0.1 % of sunlit ground ≤ p99 + 15 DN with the patched media (stock: glint) |
| Sky, haze, mission sun and a visible far field in every URC world and station view | **Met** | Every URC world includes its far field and the mission sun; eye, chase, fly and rover RGB clip at 80 km; the patched media draw sky and haze (Metal) |
| Fly view ≥ 15 fps (G6), never enters the ground, follow/top/ortho; Map view with an orthophoto and live markers | **Met** | Fly 1280×720 main with the eye small: 20/20 fps at real-time factor 0.99–1.00 in all four URC worlds; eye + chase 20/20; onboard RGB 14–15 of its 15 Hz with chase 19. Minimum clearance 0.995 m at 16× cruise over a 30 m hill. Maps: `pixi run sim-maps`, 64 s for five worlds |
| Spin ratio per surface ± 0.04 of the closed form | **Met** | All 21 catalogue grounds within 0.004–0.013 below the closed form (realism report: rock 0.432/0.436, regolith 0.373/0.377, sand 0.248/0.261, wash sand 0.169/0.182) |
| Stick-slip judder on rock | **Not met** | 0.012 peak-to-peak/mean (target ≥ 0.8); 17.4 |
| Stall at 0.7× / no stall at 1.3× the analytic current | **Met** | 6.8 A: 7.5° in 5 s (6.5 % of unlimited); 12.6 A: ≥ 85 %; monotonic |
| Dig-in on loose sand | **Met** (strong default) | as above |
| The missions keep their meaning; mission tests pass | **Met**, with design changes for the user to approve (17.5) | `pixi run sim-test`: 438 tests OK (8 opt-in skipped; 2 expected failures: the judder and dust in depth); slow route drives: all 6 pass (Autonomy's easy route 233 m in 332 s of the 360 allowed; Post 2; the astronaut walk; Delivery's 11 legs, 1823 m; Astrobiology's 3 legs; Equipment Servicing's approach) |
| One catalogue, shared assets | **Met** | `terrains.TYPES`, `landscape` recipes, `urc_media`, `WorldBuilder`; missions say where and how much |

### 17.4 Status against §6.9

| Behaviour | Verdict | Measured |
|---|---|---|
| Spin in place per catalogue type | Met | 17.3 |
| Spin on objects and tiles | Met | landing pad (manmade), step top (rock), μ 0.2 shape, SDF μ 0.5 object: each within ±0.04 |
| Mean wheel torque on μ-only lanes | Met | 2.53 N·m at μ 0.2 (2.54), 12.04 N·m at μ 0.95 (12.06) |
| Diagonal load split (PGS) | Met | 175/43/51/182 N at μ 1; 161/56/64/170 N at μ 0.8 |
| Current on a μ 0.8 slab | Met | loaded 9.75 A, light 4.3–4.4 A |
| Stall at a current limit | Met | 17.3 |
| Rise and stop | Met (after `ki` 160) | with the ramp 0.31 s (0.33 ± 20 %); without: rise 53 ms (< 60), stop 27 ms (< 40) |
| Slow spin judder on a μ 0.8 slab | **Not met** | 0.012 (≥ 0.8). A rigid rover turning in place slips at the yaw ratio where its slip is least, c²/(a²+c²), so the Stribeck drop has no first-order effect; the prototype's 1.26 came from its run with tyre compliance and PGS. With `Params.tire_compliance` 0.41 (Dantzig) / 0.78 (PGS) at the 22–24 Hz wheel hop, not 0.5–5 Hz. Sand stays smooth (0.009). Kept as an expected failure; revisit with the real wheels (Q8) |
| Loose sand straight | Met | slip 20.0 %, torque 3.38 N·m |
| Dig-in on sand | Met for both presets | 17.3 |
| Washboard | Re-baselined, met | torque std 3.5–3.9 N·m against the climb's 3.8 ± 30 %; above 5 Hz 0.8–1.0 N·m (< 1.5) |
| Spin on a 20° side slope | Met | slid 0.66 m (0.4–1.0) |
| Drift across a 20° side slope | Met | rock 0.062, regolith 0.422, sand 1.646 m against 0.064/0.394/1.59 (the formula times 1/(1 − forward slip): the slip law scales with the wheel's speed) |
| Parked 60 s | Met | ≤ 1.2 mm |
| Hold angle vs heading | Met on flat ground under tilted gravity, without backlash and μ noise | holds at atan(μs) − 2° at 0/30/45/90°, slides at + 2°. With the 1.5° backlash a rover released at 30° to the fall line rolls through the dead band and slides at 29.8° (a rover that drove there has its backlash taken up) |
| Odometry / true yaw | Met | 1/ratio ± 15 % |
| Calibration lanes | Met | μ 0.2 slides, μ 0.95 holds |
| Cost vs DiffDrive ≤ +25 % | Met | +9.5 % (rover_test), +7.4 % (Delivery) |

### 17.5 Open

- Design changes the user has not yet approved: the Autonomy caprock rib, the 15° wash banks and soft-sand wash
  floors, dig-in stopping Delivery's clay flank and the proving ground's sand dune and clay slope, zones that no
  longer level the ground, badland belts as round zones, recipe and NAIP shrubs instead of hand-placed ones, no
  zone decals, the lander's collide bitmask (Q2), the sand sheet's mild dig-in under the strong preset.
- Strong dig-in also stalls straight climbs in loose ground (sand about 10–15°, dusty clay 12°); the user's
  decision spoke of spins.
- The synthetic worlds' palettes are more saturated than the real ground as Autonomy drapes it: the sand
  sheet's (237, 176, 132) against Autonomy's draped sand sheet (230, 198, 158), CIE76 14.3; sand 14.7; silt flat
  9.1; clay crust 5.5 (§5.7's rule takes chroma from the Munsell colour, the drape from NAIP). From above the
  synthetic worlds read as orange blobs on beige (`realism_contact_sheet.jpg`). Ground photos (Q13) would settle
  which is right.
- The slow-turn judder (17.4); dust in depth (Q11); no camera noise.
- The rover high-centres on drops of 0.6 m and more (Delivery's 0.6 and 1.0 m ledges), with either drivetrain;
  the course goes round them.
- `/model/rover/ground_truth`'s twist spikes (OdometryPublisher); ROS consumers should differentiate the pose.
- Equipment Servicing reaches 1.40× by wall clock (1.00× by CPU time): above the 1.1× floor, the 1.3× target
  only by wall clock.
- The GLSL half of the media patch is untested (Metal only); the NAIP 2024 boost is untuned (Q13).
