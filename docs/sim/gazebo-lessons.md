# Gazebo lessons (gz-sim 8.10, gz-rendering 8.2.2, DART, ogre2 on Metal)

Moved from the simulation's manual ([manual.md](manual.md)); code comments cite them as
"sim/README.md, Gazebo lessons".

Measured while building the worlds, the station and the realism work; each
cost a wrong first attempt.

**Physics**
- **Heightmaps must sit at the world origin with their lowest point at
  z = 0** (or above it). DART collision ignores the heightmap's `<pos>` (and a
  negative `<pos>` z makes objects freeze or fall through), while the ogre2
  visual ignores the model's pose. `urc/world.py` therefore shifts each world
  so the terrain is there.
- **Gazebo scales an image heightmap by the image's own maximum pixel**
  (pixel / max pixel × `<size>` z) and does not shift its lowest pixel: a
  heightmap whose lowest pixel is above 0 is drawn and collides at its true
  height (render-checked with a depth camera). So the visual and the carved
  collision heightmap are separate PNGs, each with its own `<size>` z.
- **No dynamic sinkage**: DART contacts are rigid, its per-contact data has
  no normal offset, its ERP and CFM are global (a per-contact CFM is no
  stiffness: CFM 1 gave 20 mm at rest and 42 mm driving, CFM ≥ 10 fell
  through), and it ignores the normal component of
  `contactSurfaceMotionVelocity` (a wheel's height unchanged at ±5 cm/s,
  research of 2026-10-07). A wheel can only be drawn sunk (Visible dig-in).
- **`<friction>` works only on primitive shapes.** Heightmaps and meshes
  always have μ 1.0, and DART takes min(μ_a, μ_b) of two touching shapes in
  each box direction, which erases a tyre's anisotropy on slippery ground.
  **Per-contact friction works on every shape**: a system that sets
  `components::EnableContactSurfaceCustomization` on a collision gets
  `events::CollectContactSurfaceProperties` for its every contact and may set
  μ, the friction direction, slip compliance and more. Slip compliance is
  constraint force mixing on the friction rows (slip speed = compliance ×
  force, even at rest): scale it with the wheel's own speed (gz WheelSlip's
  convention) or a parked rover creeps.
- **PGS vs Dantzig** (`<physics><dart><solver><solver_type>`): Dantzig, DART's
  default, sizes friction limits from the loads before friction (one contact
  carried 60 N of friction on 25 N at μ 1); PGS gives each contact μ times its
  own load and costs more (+12–25 %). Both kept Delivery's toolbox with the
  wrench in it still for 60 s (gate G2).
- **A heightmap's bounding box spans its whole height range**, so every
  dynamic link below the terrain's highest point runs the heightfield
  narrowphase each step, touching or not: one 3 m spike on Equipment
  Servicing's flat terrain cost +9 % per step through the lander's links.
  Collide bitmasks fix it (`sdf.GROUND` on terrain shapes, `ABOVE_GROUND` on
  the lander's parts).
- **DART's cylinder–heightmap collision lets wheels into relief**: on a
  corrugated heightmap they sink 5–15 cm and stall (DiffDrive's too); on
  smooth 5 cm RMS relief the surface reaches 2–3 cm into them (p99) at any
  sample spacing; on a mesh of the same surface they ride it (0.5 mm). The
  Bullet collision detector cut it to 1 cm but left wheels up to 9 mm above
  the ground.
- **A cylinder wheel touches flat ground at both tread edges, and the
  solver splits the load**: Dantzig evenly, PGS nearly all on the first
  contact it visits (90 / 2 N); per-contact friction or slip compliance must
  follow each contact's own load (the wheel joint's transmitted torque gives
  the split), or PGS and Dantzig disagree (1.85× the slip).
- **Every shape costs time every step, touched or not**: 0.56–0.83 µs per
  collision shape and 0.10–0.18 µs per visual per 1 ms step. A separate static
  model costs about 2–4 µs and its bounding box is tested against the
  heightmap every step: hundreds of separate rocks cost 3×. So rocks, slabs,
  risers, shrubs, pebbles and blocks are shapes of the terrain's own link,
  merged per 128 m chunk. The catch-floor under the terrain is a box, not an
  infinite plane, whose bounding box would overlap everything.
- **GLB meshes work as DART collisions** (identical to OBJ) and use 0.41× the
  visual memory of OBJ. gz writes glTF's Y-up as is: our GLBs are Z-up. An
  SDF `<material>` on a glTF visual replaces all its materials.
- **DiffDrive makes every wheel a velocity servo** (exact speed, torque at its
  cap only for ~15 ms, no current, no stall) and never times out its command:
  the last twist stays in force. It must not share a model with a
  torque-driven drivetrain: its servo overrides `Joint::SetForce`.
- **`gz::math::SpeedLimiter` cannot be copied or moved**: keep it in place
  (a `std::deque`).
- **A rover spawned on a sloped heightmap lands on its uphill wheels first**;
  that jolt can start a slide on slopes between atan(μk) and atan(μs). A
  wheel spawned inside the heightmap stays inside it: on a 15° cross slope a
  0.1 m lift left the uphill wheel 9 cm deep, easing out over metres of
  driving (spawn high enough to clear every wheel).
- **Gazebo's GUI Teleop sends one message per button press**: a drivetrain
  with a command timeout stops after the timeout; it needs `cmd_timeout` 0.
- **OdometryPublisher's twist can spike** (one yaw rate of 4π/dt in 1233
  messages while spinning); its pose is right.
- **NavSat noise is in degrees** for latitude/longitude; `gen_model.py`
  converts metres. NavSat reports the ellipsoidal height of the world
  origin's altitude plus z: give the origin an ellipsoidal altitude.
- **Lidar and imagery datums**: PROJ's default NAD83(2011) → WGS84 is a null
  transformation, while the 3DEP and NAIP image services apply a real shift
  (here 1.1 m); align rasters by measurement, not by their tags.
- `JointStatePublisher` publishes every 1 ms step here; the lander's 101
  joints use `JointMonitor` (50 Hz, plus an event per key press).

**Rendering**
- **ogre2 heightmaps blend at most four textures, by height only**
  (`smoothstep(min_height, ...)`; the weight map is never used); extra layers
  are silently dropped. All spatial variation goes into layer 0, a full-size
  colour map; detail layers get near-constant weights from a huge fade and
  per-texel compensation in linear light.
- **Stock Terra renders a layer without a roughness map at roughness 0**, a
  mirror: specular glints and sky reflections that look like puddles (fixed
  by the media patch).
- **gz-sim 8 ignores SDF `<scene><fog>`, `<sky><cubemap_uri>`, lens
  `<distortion>` and orthographic `<lens><type>`**; SDF camera `<noise>` is
  strongly attenuated (stddev 0.05 → 1.5 DN) and on an RGB-D camera aborts gz
  on Metal (Ogre RenderingAPIException). Sky and haze come from the media
  patch, orthographic projection from the FlyCamera plugin.
- **Any `<projector>` in a world with a Terra heightmap aborts gz on Metal**
  (the Terra pixel shader fails to compile), stock media too: no projected
  decals or tyre tracks (the ruts are markers the drivetrain draws).
- **Render hooks**: a system that changes a rendering camera should hook
  `events::SceneUpdate`; a PreRender, Render or PostRender hook makes the
  Sensors system render every step (−35 to −45 % sim speed). Look the camera
  up by name each time and keep no `CameraPtr`: a stored pointer crashes gz at
  shutdown. `SetWorldPoseCmd` sent only when the pose changed took effect 5
  steps late after idle steps; send it every step.
- **`events::SceneUpdate` runs beside the next step**: on the Sensors
  rendering thread, at the end of `RenderUtil::Update`, while the next step's
  PreUpdate already runs (Sensors' PostUpdate wakes the thread and returns;
  the following PostUpdate waits for it). A hook reading what the server
  thread writes saw the next step's data in about one frame in ten: stamp
  such data with sim time and draw what is due at `scene->Time()` (the
  ruts). The hook runs at start-up, when the cameras are made, and
  afterwards only while some camera is watched, so a late viewer's first
  frame meets everything laid since.
- **Markers** (`gz::rendering` dynamic renderables, the ruts' berms):
  append to a marker (`AddPoint`), never recreate it (a marker made anew
  every frame cost 0.5–1 ms a frame and 7–32 % of the throughput); a changed
  marker refills its whole vertex buffer, so keep markers to chunks. They
  have no UVs and no vertex colours: one colour per material, darkness in
  levels. `Ogre2Marker::SetMaterial` turns the material's shadows off;
  receiving can be turned back on after the marker takes the material,
  casting cannot, so berms show relief by shading only. Destroying a visual
  frees its markers' meshes recursively: destroy the visual first, then its
  own materials.
- **Transparent markers over Terra**: setting a material's transparency
  moves only the renderables already linked to it into render queue 200,
  after Terra's 11; call `SetTransparency` again after a marker takes the
  material, or Terra draws over it. A transparent material with depth
  writing off appears in no depth image and no point cloud (the ruts'
  floor).
- **Terra draws the colour map's own colour**: Terra over PBS measured
  0.93–1.03 per channel at 8 spots in two worlds, 0.99 / 1.00 / 1.00 over the
  5 whose red the colour map does not clip (a flat PBS square of
  `albedo.png`'s colour against the terrain round it, from above), so
  geometry coloured from the uncompensated colour map matches the ground
  from above. At eye level the sunlit sand's red clips in the cameras (72–86 %
  of its pixels at 255 beside the ruts' berms): the sand there looks yellower
  than its colour, and darker geometry of the same colour, below the clip,
  shows the true, redder hue (berm and unclipped sand within 1° of hue). No
  single colour matches both views.
- **Terra's level of detail follows the camera's place, not its range**: Terra
  draws its finest level over a square of its cells (64 heightmap samples,
  32 m on Autonomy) round the camera's cell, reaching 32–96 m from the camera
  by direction and by where in its cell the camera stands (measured on
  Autonomy: tracks whole from 40.8 m, broken from 41.8 m across a cell line).
  Beyond it, on relief, the drawn surface leaves the heightmap by ±2.5–3 cm
  (inside, within about 1 cm), enough to bury geometry a few mm above the
  ground (the ruts' 4 mm floor, the sand sheet's berms) and leave other bits
  standing: tracks break into fragments with straight cut-offs. On flat ground
  nothing changes (Autonomy's sand: tracks whole at 60, 150 and 400 m), and
  cameras straight above them saw them whole from 60 and 150 m up.
- **`rgbd_camera` ignores `<visibility_mask>`** in its image and its depth:
  whatever is drawn for people the robot sees too, so a cue must sit on the
  ground or on the tyre, never float.
- **A camera's horizontal field of view below 0.1 rad is rejected** (the world
  fails to load): the map tool's telephoto tiles stay at ≥ 0.1 rad.
- **No cast shadows beyond ~500 m camera distance** (gone by 590 m) and none in
  orthographic projection: map tiles are shot from 300 m.
- **A paused world renders no camera frames** (0 frames in 3 s): inspect
  paused worlds in the Gazebo GUI.
- **Particle emitters**: `<emitting>` defaults to true, and gz-sim creates
  emitters from link SDF even without the particle-emitter system; give them
  `<emitting>false</emitting>` and an explicit `<topic>`. A particle material
  without a `<diffuse>` renders black. Colour ranges are not applied
  (`SetColorRange` is disabled, a `<color_range_image>` changed nothing):
  the sprite carries colour and opacity. The depth camera takes every
  particle pixel with any red for a return at a fixed scatter ratio:
  `particle_scatter_ratio` (SDF or the emitter's topic) never reaches it, nor
  does a camera `<visibility_mask>`. Hence the rover has no dust by default
  (`DriveParams.dust`).
- **Orthographic near plane**: an orthographic camera clips at its near plane
  just under it, and a custom projection with a near plane behind the camera
  changed nothing; draw it from above whatever it must show (the window
  depends only on the projection).
- **`google::protobuf::Struct` fields**: take `fields()` after parsing; a
  reference taken before `JsonStringToMessage` sees an empty map.
- **RGB-D clip split**: an `rgbd_camera` reads `<depth_camera><clip>`
  separately, so the RGB can reach 80 km while the depth stays clipped at 40 m.
- **Albedo-map alpha is tested at 0.5**: cut-outs work, feathered edges do
  not (`<transparency>` blends only above 0.5).
- **The conda gz-rendering has a space-padded OGRE plugin path**: set
  `OGRE2_RESOURCE_PATH` (`gzenv` does) or cameras cannot render; the
  `Unable to load Ogre Plugin` error is printed once even when the fallback
  works.
- **On macOS a rendering server must run on the main thread**, and ogre2
  starts once per process: tests strip the Sensors system from world copies
  and render in subprocesses.
- **A camera renders only while its topic has subscribers**, and the RGB-D
  camera's first colour image after a time without colour subscribers is all
  black (gz-sensors 8; depth is not affected): the station drops it.
- LensFlare logs "Render pass added", then disconnects its PostRender hook.
- **A visual's pose written at runtime** (its `components::Pose`, in its link)
  is drawn at once by every camera sensor (RenderUtil re-reads every visual's
  pose each update) and moves no physics. The GUI gets only what is marked
  changed, and `SetComponentData` marks nothing itself (gz-sim 8: it stores
  the value and returns whether it differs); `Pose3d`'s `==` has a 1 mm
  tolerance, so it returns false for a smaller move, and a caller that marks
  the change from its return value marks nothing: the SceneBroadcaster never
  sends it (measured: a sinking tyre moving 0.16 mm a step reached the
  cameras and no state message). Compare exactly, write the component, and
  `SetChanged(..., PeriodicChange)`: periodic changes are cached until the
  next state message, so the last pose, the SDF pose put back too, always
  reaches the GUI.
- **Picking frames in tests**: a camera at 20 Hz has two frames in every
  0.1 s window, so a harness that keeps the first frame to arrive is not
  repeatable; pick frames by their stamp (`test_render.Renderer.drive_variant`
  keeps the earliest at or after each grab time). Frames of the same sim time
  in two runs are then bit for bit the same as a rule (16 repeated drives),
  but with world generation running beside them one terrain pixel of the eye
  camera once came out 1 DN off (1 of 36 frames compared, no cue drawn), and
  a chase frame differed in one `test_render.Ruts` run: run the
  bit-for-bit picture tests on a quiet machine.

**Transport and processes**
- **Python `gz.transport13` `node.request()` keeps the GIL while it waits.**
  With any subscription active, gz-transport's reception thread blocks on the
  Python callback and the reply can be lost, although the server still acts
  on the request. Send requests before subscribing, or confirm the effect
  another way (the station watches the camera's state topic); per-tick fly
  control goes over published topics.
- **Python gz-sim systems crash the in-process test fixture**: systems are C++.
- **`/world/<w>/stats` comes at 10 Hz wall clock**, also while paused and at
  real-time factor 0.2; `/world/<w>/clock` every step (about 1 kHz).
- **`topic_list()` lists advertised topics only**, not subscribed-only ones.
  After a clean SIGINT stop, a server's topics leave the list at once.
- **CPU time per step counts gz's helper threads** (transport, `/clock` every
  step): on an M4 the server keeps about 1.3 cores busy, and the wall-clock
  step is 13–29 % shorter than the CPU time. Judge real time by the wall
  clock on an idle machine, ratios by CPU time on interleaved runs.
- **`gz sim`'s stdout is block-buffered when redirected** and lost if the
  server is killed: run it on a pseudo-terminal for logs.
- **Importing `tests/simulate.py` sets the tests' Gazebo environment**
  (`gzenv.environment`, the patched media's `GZ_RENDERING_RESOURCE_PATH`
  among it): a picture subprocess that imports it silently renders its
  "stock" runs with the patched media. `test_render` imports it only in the
  parent and hands the subprocess precomputed commands.

