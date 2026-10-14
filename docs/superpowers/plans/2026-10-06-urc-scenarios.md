# URC 2027 scenarios Implementation Plan

> **For agentic workers:** executed inline (goal mode). Steps use checkbox (`- [x]`) syntax for tracking.
> Deviation from the writing-plans template: interfaces and tests are fixed here, full code is not
> duplicated into the plan (the user asked for uninterrupted execution of a ~3k-line build).

**Goal:** Generate one Gazebo world per URC 2027 field mission (Autonomy, Equipment Servicing,
Delivery, Astrobiology) plus mission sheets and a referee, per
`docs/superpowers/specs/2026-10-06-urc-scenarios-design.md`.

**Architecture:** `sim/urc/` package (stdlib + numpy + OpenCV + Pillow, all in the pixi env) generates
terrain, textures, meshes, model directories and world SDF; `sim/gen_worlds.py` is the CLI;
`sim/urc/judge.py` is a Gazebo-free mission state machine wrapped by `sim/referee.py`.

**Tech Stack:** Gazebo Harmonic (gz-sim 8.10, DART/ODE detector, ogre2), Python 3.12, numpy, cv2.aruco,
Pillow, gz.sim8 / gz.transport13 bindings, unittest.

**Verification commands:** `pixi run sim-worlds`, `pixi run sim-test`.
No git repository here, so no commit steps.

---

### Task 1: Foundations — rules, geo, sdf helpers
**Files:** create `sim/urc/__init__.py`, `sim/urc/rules.py`, `sim/urc/geo.py`, `sim/urc/sdf.py`;
test `sim/tests/test_urc_unit.py`.
- [x] `geo.Origin(lat, lon, alt)`; `enu_to_wgs84(o, x, y, z=0) -> (lat, lon, alt)`;
  `wgs84_to_enu(o, lat, lon, alt) -> (x, y, z)` via WGS84 ECEF. Test: round trip < 1 mm at 1 km;
  1 km north ≈ +0.009 deg lat.
- [x] `rules.py`: constants with rule refs (AR face 0.20 m, cell 0.025 m, centre height range
  0.5–1.5 m, tag IDs start/post1/post2 = 0/1/2, keyboard tag IDs and size, key tag IDs 5/6 at 1 cm,
  route tolerance 1 m, astronaut tolerance 3 m, stay distance 20 m, lander distance ≤100 m,
  equipment height ≤1.5 m, delivery object limits 5 kg / 0.40 m / 7.5 cm, cache handle ≥0.10 m long
  ≤0.05 m dia <5 kg, astrobiology radius 500 m, sample depth 0.10 m, antenna height 3 m,
  square-mile bounds, LED colours).
- [x] `sdf.py`: `fmt`, `sub`, `pose`, `inertial`, inertia helpers (box, cylinder-z, sphere),
  geometry builders, `material`, `shape`, `document`, `write_model(models_dir, name, model, description)`.

### Task 2: Terrain
**Files:** `sim/urc/terrain.py`; tests in `test_urc_unit.py`.
- [x] `Heightfield(size, n)`: `z[row, col]`, row 0 = north (+y), col 0 = west; `xs`, `ys`, `grid()`,
  `height(x, y)` bilinear, `slope_deg(x, y)`, `line_of_sight(a, b)`, `max_slope_along(path)`.
- [x] Ops: `fbm(...)`, `blend(mask, target)`, masks (`radial_mask`, `path_mask`), `flatten`,
  `mesa`, `ramp`, `channel`, `ridge`.
- [x] Export: `write_png(path) -> (zmin, zmax)` (16-bit, full range so Gazebo's per-image
  normalisation is exact), `write_geotiff(path, origin)` (float32, EPSG:4326 tags).
- [x] Tests: bilinear height on a plane; slope of a known ramp; LoS blocked by a wall; PNG round-trip.
- [x] Headless test: probes dropped on a generated heightmap rest at `height(x, y)` (checks row
  orientation and scaling).

### Task 3: Textures and meshes
**Files:** `sim/urc/textures.py`, `sim/urc/meshes.py`.
- [x] `aruco_image(tag_id, px_per_cell)` (8×8 cells incl. white border); test: cv2 decodes the ID.
- [x] `terrain_texture(path, rgb, seed)`, `normal_map(path, seed)`, `sign_image(path, lines)`,
  `solid(path, rgb)`.
- [x] `write_obj(path, V, F, N=None, UV=None)`, `rock(seed, radii)`, `quad()` (unit, faces +x, UV
  not mirrored), `drape(hf, cx, cy, radius, offset, seed)` (terrain-conforming patch with UVs).

### Task 4: Props
**Files:** `sim/urc/props.py`.
- [x] AR post (3 faces, IDs 0–2), astronaut (articulated, gravity off, JointPositionControllers,
  sign), rock pick hammer, C2 station (+3 m mast), landing pad, start gate, rocks (N variants),
  shrub, toolbox (hinged lid) + wrench, supply crate, water jug, first-aid kit, instrument case,
  spectrometer, field sign, cache container (lid, lock knob, handle), sample tube, sample stand,
  fuel tank + segmented hose with cam-lock coupler.
- [x] Tests: post face size/height/IDs; object masses/sizes/handles within rule limits;
  all model SDFs pass `gz sdf -k`.

### Task 5: Lander
**Files:** `sim/urc/lander.py`.
- [x] World-fixed base, panel with keyboard (87 spring keys, `KEYMAP` joint→char), tags 1–4,
  display, drawer (+ cache slot), hinged panel + latch, locks A/B + key, hose inlet, valve, buttons,
  switches, knobs; `JointStatePublisher` on `/model/lander/joint_states`.
- [x] Tests: all interactive parts ≤1.5 m; joint names in `lander.JOINTS`; `gz sdf -k`.

### Task 6: World assembly and missions
**Files:** `sim/urc/world.py`, `sim/urc/missions/{__init__,autonomy,equipment,delivery,astrobiology}.py`,
`sim/gen_worlds.py`; modify `pixi.toml`.
- [x] `WorldBuilder(name, origin, terrain, out)`: physics, systems, sky/sun, terrain (PNG +
  GeoTIFF + textures), horizon plane, `place(uri, name, x, y, yaw, ...)` snapped to terrain,
  rock fields, decals, `task(...)` sheet entries, `write()`.
- [x] Four mission layouts; sheet JSON per world.
- [x] Tests (`test_urc_missions.py`): rules satisfied per layout (bounds, distances, NLOS Post 2,
  LoS start/Post 1, easy-route slope ≤15°, other hill sides ≥35°, lander ≤100 m, astrobiology units
  ≤500 m, delivery ≤1 km, stay ≥20 m); worlds pass `gz sdf -k`.

### Task 7: Rover sensors, bridge, run script
**Files:** modify `sim/gen_model.py`, `sim/bridge.yaml`, `sim/run.sh`, `sim/tests/simulate.py`,
`sim/tests/test_gen_model.py`.
- [x] Params + navsat, rgbd camera, LED visual; bridge entries; `OGRE2_RESOURCE_PATH` in run.sh and
  simulate.py.
- [x] Tests: sensors present with topics; existing tests still pass.

### Task 8: Headless world tests
**Files:** `sim/tests/test_urc_sim.py`.
- [x] Each world loads, rover settles at spawn on terrain; NavSat ≈ sheet start; camera at the
  route start detects ArUco 0; lander key press detected by the judge decoder.

### Task 9: Judge and referee
**Files:** `sim/urc/judge.py`, `sim/referee.py`, tests `sim/tests/test_urc_judge.py`; pixi task `referee`.
- [x] Observation/Action types; judges for autonomy (assistance + route), equipment, delivery,
  astrobiology; typing decoder with backspace; stopped detection; footprint distance.
- [x] gz adapter: subscriptions, set_pose_vector, joint commands, material_color LED, `/urc/score`,
  `/urc/radio`.
- [x] Tests: synthetic streams for each judge; one headless integration (teleport rover to Post 1 +
  LED green → scored).

### Task 10: Docs
- [x] `sim/README.md` URC section (worlds, sheets, referee, topics); update spec status.
