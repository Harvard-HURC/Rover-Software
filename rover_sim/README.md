# Rover Sim
This package contains any useful tools, SDFs, or URDFs that are useful for simulating the rover in Gazebo. For example, this might contain simplified rover descriptions that abstract away the drivetrain or SDFs for each testing scenario we want to run in the simulator (obstacle courses, interactibles, entire missions, etc).

Today it holds the URC simulation: a model of our rover (rocker suspension with a differential, skid-steer drive with a DC motor per wheel, an RGB-D camera on a pan-tilt head), the URC 2027 mission worlds on MDRS ground, a proving ground, a browser driver station, a launcher and a referee. It runs on macOS and Linux (on Windows in WSL2). The manual is [docs/sim/manual.md](../docs/sim/manual.md), with its [design notes](../docs/sim/design-notes.md) and [Gazebo lessons](../docs/sim/gazebo-lessons.md).

### Quick start
From the repository's root (`<workspace>/src`), after `pixi install`:

```bash
pixi run fetch-data             # the terrain rasters, once (git-ignored)
pixi run sim                    # build the plugins, generate rover and worlds, open Gazebo with the ROS 2 stack (rover_test)
pixi run sim urc_autonomy       # a mission world (also urc_equipment_servicing, urc_delivery, urc_astrobiology, proving_ground)
pixi run drive urc_autonomy     # drive from the browser
pixi run launcher               # every world as a tile
pixi run test                   # the colcon tests (rover_description, rover_control, rover_sim) and the simulation's suite
```

Every world needs the rasters fetch-data fetches, the test ground too (the tasks generate all worlds). fetch-data cannot rebuild the Autonomy world's 0.5 m lidar DEM (`route_area_lidar_0p5m.tif`) from official sources yet; a script that rebuilds it from USGS lidar is planned. Until then, link it from a teammate's clone that has it: `rover_sim/tools/link_data.sh <that clone>` (it links every raster that clone has, also when fetch-data cannot reach a source), or copy the file into `rover_sim/data/dem/` (`pixi run fetch-data --verify` checks it). Without it `sim-worlds` skips `urc_autonomy` (`pixi run sim urc_autonomy` and `pixi run drive urc_autonomy` say what is missing) and the tests that need it skip, saying why.

### Layout
| Path | What it is |
|---|---|
| `plugins/` | Gazebo system plugins (RoverDrivetrain, RockerDifferential, JointMonitor, ChaseCamera, FlyCamera) and their C++ tests; colcon builds them into the workspace's `build/rover_sim` and installs them to `lib/rover_sim` |
| `config/`, `urdf/`, `rover_config.py`, `tools/gen_rover.py` | How the simulation models the rover (`config/sim.yaml`), the Gazebo parts it adds to `rover_description`'s description (`urdf/rover_sim.urdf.xacro`), the one reader of both YAML files for the simulation's Python, and the generator of the rover's Gazebo model (`pixi run sim-model`) |
| `viewers.py`, `models/` | The driver station's cameras; the generated models (the rover's, the cameras') |
| `gen_worlds.py`, `urc/`, `worlds/` | The URC world generator and the worlds (`rover_test.sdf` is tracked; the rest is generated and git-ignored) |
| `station/`, `launcher/`, `referee.py` | Driver station, launcher, URC referee |
| `ros2_control/`, `rover_sim_system.xml` | `RoverSimSystem`, the simulated rover's ros2_control hardware: gz_ros2_control loads it inside Gazebo, and it hands each wheel's velocity command to the drivetrain's motor model (`plugins/rover_components.hh`) and reports the joints and the motor currents |
| `launch/`, `gzenv.py`, `run.sh`, `bridge.yaml` | `sim.launch.py` (below), the Gazebo environment (the one place it is set), `pixi run sim`, the sensor bridge's topics |
| `wheel_cmd.py` | The ROS-free way to drive the simulated rover: a twist as the wheel setpoints the drivetrain takes on `/model/rover/wheel_cmd` (the tests and tools) |
| `env-hooks/` | Points `GZ_SIM_SYSTEM_PLUGIN_PATH` at the installed plugins when a shell sources `install/` |
| `tools/`, `patches/` | Fetching the rasters, the patched rendering media (`patches/`: the gz-rendering patch it applies), maps, the realism report, relief swatches |
| `data/` | Terrain data: the rasters (git-ignored, `pixi run fetch-data`, listed in `data/rasters.json`) and the `.json` that says where each came from, relief swatches, the soil map, the research files the code reads |
| `tests/` | The simulation's suite (`pixi run sim-test`) |

Comments and docstrings written before the simulation joined this repository call this folder `sim/` (read `rover_sim/`), cite `sim/README.md` (now the three files in `docs/sim/`) and `sim/data/research/` (now `docs/research/`, except the four files kept in `data/research/`).

The rover itself is `rover_description`'s: its physical numbers are `rover_description/config/rover.yaml` and its README says how to change it. `config/sim.yaml` holds how the simulation models it (the drivetrain's settings, contact, dig-in, the visual cues, dust, the Gazebo topics); `pixi run sim-model` builds `models/rover/model.sdf` from both, and `tests/test_equivalence.py` checks it against the rover frozen before the description existed. `gen_model.py`, which held all of it as Python before, is gone; comments in `urc/sdf.py` and `urc/textures.py` still name it, because a change to either renames the media files every world references (`urc/media.py` names a generated texture or mesh after the source of its generator's module and of the `urc` modules that one imports: `textures.py` holds generators, and `farfield.py`'s imports include `sdf.py`), so they wait for a change that regenerates the worlds anyway.

### Launch Files
- `sim.launch.py`: the rover in a world with its ROS 2 stack: `ros2 launch rover_sim sim.launch.py world:=<name> gui:=true|false station:=true|false controller:=<name> sensors:=true|false`. Gazebo's server (and its GUI as a separate process), robot_state_publisher, the controllers of `rover_control/config/controllers.yaml` in the controller manager gz_ros2_control runs inside Gazebo, the sensor bridge (`bridge.yaml`) and, with `station:=true`, the driver station (it opens its page in the browser). `pixi run sim`, `pixi run drive <world>` and the launcher start it. The worlds include the ROS-free rover (`model://rover`, which the tests drive through `/model/rover/wheel_cmd`); the launch runs a copy of the world whose rover has ros2_control, at the same pose. After a world reset it spawns the controllers again (gz_ros2_control is reloaded); a spawner that fails stops it. The topics are [docs/topics.md](../docs/topics.md)
- `gazebo.launch.py`: Start Gazebo using the correct method for your operating system. Set the `world` argument to change which SDF is loaded

### SDF Worlds
- `worlds/rover_test.sdf`: test ground: a 10 cm step under the left wheels, a 15° ramp to a 0.3 m platform, rocks
- `worlds/urc_autonomy.sdf`, `urc_equipment_servicing.sdf`, `urc_delivery.sdf`, `urc_astrobiology.sdf`: the URC 2027 field missions at their real places near MDRS, and `proving_ground.sdf`, a test course; generated by `pixi run sim-worlds` (git-ignored). The manual describes them

Open these worlds with `pixi run sim <world>` or `sim.launch.py`; `gazebo.launch.py` cannot load them (it sets neither Gazebo's resource path nor its plugin path, which `gzenv.py` sets for `sim.launch.py`).

A new world is a hand-written `.sdf` in `worlds/` (tracked; describe it in the list above) or a generated one (`urc/missions/<name>.py`, `pixi run sim-worlds <name>`; git-ignored). Either includes `model://rover` exactly once, at the start pose, since `sim.launch.py` swaps that include for the rover with ros2_control: [CONTRIBUTING.md, Add a world](../CONTRIBUTING.md#add-a-world), and the manual's [A new world](../docs/sim/manual.md#a-new-world).

### Tests
- `pixi run sim-test`: the suite in `tests/` (597 tests, Python unittest), after building the plugins and generating the rover and the worlds. It drives the rover in Gazebo without ROS (`wheel_cmd.py`), and with ROS 2 in `test_ros2_control` (RoverSimSystem inside Gazebo with the controllers) and `test_sim_launch` (`sim.launch.py`).
- One module, class or test: `cd rover_sim/tests && pixi run python -m unittest -v <module>[.<Class>[.<test>]]`, after `pixi run build` (and `pixi run sim-worlds` for tests that use the generated worlds). The smoke test is `test_sim_launch.SmokeTest`: `sim.launch.py` headless on `rover_test`, `/cmd_vel` moves `/odom` (18 s on an Apple M4, M).
- The C++ tests of the plugins (`plugins/tests/`) and of RoverSimSystem (`ros2_control/test/`) run under `colcon test` (`pixi run colcon-test`).
- The rendering tests skip on a machine without a GPU and say why (`tests/gpu.py`; `ROVER_RENDERING=0` or `1` decides). `pixi run sim-perf` and `pixi run sim-slow` are opt-in (the manual's Quick start).
- Every test process takes a gz partition and a ROS domain of its own (50 to 99), so the suite's worlds do not talk to a simulation you have open.

[CONTRIBUTING.md](../CONTRIBUTING.md) has the setup (the simulation on Windows runs in WSL2), the workflow, how to add a sensor and the code style.
