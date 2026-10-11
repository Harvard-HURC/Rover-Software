# HURC Rover Software

Welcome to the central repository for the HURC Rover's control software!

### Setup
We use [pixi](https://pixi.prefix.dev/latest/#installation) to manage ROS packages with [RoboStack](https://robostack.github.io/GettingStarted.html). [CONTRIBUTING.md](CONTRIBUTING.md#set-up) has these steps for macOS, Linux and Windows (natively, and the simulation in WSL2)

1. Install pixi
2. Create a new folder to be your root ROS workspace (ex: `/home/username/hurc_ws/`)
3. Clone this repo into a subfolder called `src` (ex: `/home/username/hurc_ws/src/`)
4. Navigate to the `src` folder and run `pixi install`
5. Run `pixi shell` from this folder anytime you need to interact with installed packages (like ROS)

### Usage
To build and source our custom packages for testing, follow these steps:
1. Starting from a pixi shell, go to the root folder of the workspace (such that you can see the `src/` folder)
2. Run `colcon build` and wait for all packages to build (you can use `--packages-select package_name other_package_name ...` to build only specific packages, and you can use `--continue-on-error` to continue building other packages after one fails)
3. Source the appropriate setup file from the `install` folder (ex: on bash, run `source install/local_setup.bash`)
You should now be able to use launch files, executables, and assets installed by any of our packages in your current shell.

From the `src` folder, `pixi run build` does step 2 for you (`colcon build` in the workspace root; on Windows without the tests), and the tasks below build what they need first.

### Simulation
The rover's Gazebo simulation (`rover_sim`) runs on macOS and Linux; on Windows it runs in WSL2 (see Platforms). From the `src` folder:
1. `pixi run fetch-data` downloads the terrain rasters the simulation reads from their official sources, once (about 90 MB of USGS elevation data and USDA NAIP imagery; git ignores them). The tasks below generate every world, so they need these, also for the test ground. fetch-data cannot rebuild the Autonomy world's 0.5 m lidar DEM (`route_area_lidar_0p5m.tif`, 41 MB) yet: a script that rebuilds it from USGS's public lidar is planned. Until then, link it from a teammate's clone that has it: `rover_sim/tools/link_data.sh <that clone>` (it links every raster that clone has, also when fetch-data cannot reach a source), or copy the file into `rover_sim/data/dem/` (`pixi run fetch-data --verify` checks it). Without it the tasks generate every world but `urc_autonomy` (`pixi run sim urc_autonomy` and `pixi run drive urc_autonomy` say what is missing), and the tests that need it skip and say why
2. `pixi run sim` builds the simulation's plugins, generates the rover and the worlds and opens Gazebo on the test ground with the rover's ROS 2 stack: `rover_sim/launch/sim.launch.py` runs the controllers, robot_state_publisher and the sensor bridge (`pixi run sim urc_autonomy` opens a URC mission world)
3. `pixi run drive urc_autonomy` drives the rover from the browser; `pixi run launcher` shows every world as a tile that starts it with its driver station. Programs drive the simulated rover as they will drive the real one, through `/cmd_vel` (geometry_msgs/TwistStamped) into the controllers: in `pixi shell`, `ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}}}"`. The topics are listed in [docs/topics.md](docs/topics.md)

The manual is [docs/sim/manual.md](docs/sim/manual.md), with its [design notes](docs/sim/design-notes.md) and [Gazebo lessons](docs/sim/gazebo-lessons.md).

### The real rover
`ros2 launch rover_control rover.launch.py` (in a shell that sourced `install/`, Usage above) starts the real rover's stack: robot_state_publisher on `rover_description`'s description and the same controllers as the simulation (`rover_control/config/controllers.yaml`). Until the motor controllers are chosen its hardware interface is a stub that reports zero motion and logs the wheel commands it would send (`rover_control/README.md`).

### Tests
- `pixi run test` runs the colcon tests of `rover_description` (xacro and `check_urdf` on the description, the team's linters), `rover_control` (GoogleTest, and `rover.launch.py` on the hardware stub) and `rover_sim` (C++, `RoverSimSystem` among them) and the simulation's suite, which also runs `RoverSimSystem` inside Gazebo with the controllers and `sim.launch.py` headless (the smoke test): about 17 minutes on an Apple M4. Tests that render skip on a machine without a GPU and say why
- CI (`.github/workflows/ci.yml`) runs the same tasks on every pull request: on Ubuntu 24.04 and macOS (Apple silicon) `pixi run build`, `pixi run fetch-data`, `pixi run test` (the rendering tests on macOS only) and then `git diff --exit-code`, which fails when the tests left a tracked file changed (they regenerate the tracked rover model, so one committed stale fails); on Windows `pixi run build`. CI has only the rasters fetch-data can fetch, so the Autonomy world's tests skip there until its lidar DEM can be rebuilt

### Platforms
- **macOS (Apple silicon):** everything
- **Linux (Ubuntu 24.04):** everything; the camera views and the rendering tests need a GPU
- **Windows:** `colcon build` (or `pixi run build`) builds `rover_description` and `rover_control` natively (`rover_sim` builds nothing there), and RViz and the ROS 2 tools run in `pixi shell`. The Gazebo simulation runs in WSL2: install Ubuntu 24.04 (`wsl --install -d Ubuntu-24.04`), install pixi inside it, clone this repository into a Linux folder (for example `~/hurc_ws/src`, not under `/mnt/c`, which is slow) and follow the steps above there. WSLg shows Gazebo's window; open the address the driver station prints in a Windows browser. [CONTRIBUTING.md](CONTRIBUTING.md#windows-the-simulation-in-wsl2) has the step-by-step guide, with the GPU driver and what to do when something does not work

## Project Structure
Most of our work is structured into folders called "packages". Whenever you create a new package, please add a description of it here:
- [`rover_description`](rover_description/README.md): Contains the URDFs that define the rover's structure, joints, actuators, sensors, etc. Every physical number of the rover is in `config/rover.yaml` (geometry, masses, joints, the rocker differential, the motors, the sensors and the camera head); `urdf/rover.urdf.xacro` builds the description from it, with the `ros2_control` block in `urdf/ros2_control.xacro`; `meshes/` has the tread tyre and `launch/view.launch.py` shows the rover in RViz. Its README says how to change the rover
- [`rover_sim`](rover_sim/README.md): Contains SDFs, URDFs, launch configurations, and any other tools that help with simulating the rover in Gazebo. Today it holds the URC simulation: Gazebo plugins for the drivetrain and motors, the rocker differential and the driver station's cameras; `RoverSimSystem`, the simulated rover's ros2_control hardware, which hands the controllers' wheel commands to the motor model; `launch/sim.launch.py`, the rover in a world with its ROS 2 stack; the rover's Gazebo model, built from `rover_description` and `config/sim.yaml` (how the simulation models the rover) with `urdf/rover_sim.urdf.xacro`; the URC 2027 mission worlds and their generator; a browser driver station, a launcher and a referee (macOS and Linux)
- [`rover_control`](rover_control/README.md): The rover's drive: the controllers both the real and the simulated rover run (`config/controllers.yaml`: diff_drive_controller and joint_state_broadcaster, started by `launch/controllers.launch.py`), the real rover's `launch/rover.launch.py` with its ros2_control hardware stub until the motor controllers are chosen, and `tank_kinematics`, a skid-steer kinematics library that reads `rover_description`'s `rover.yaml`

Beside the packages, `docs/` holds the topic contract ([docs/topics.md](docs/topics.md)), the design specs and plans (`docs/superpowers`), the simulation's manual (`docs/sim`) and the research behind it (`docs/research`); `.github/` holds CI (`workflows/ci.yml`), the code owners (`CODEOWNERS`) and the pull request and issue templates.

## Contributing
Please do not push directly to the main branch. Any time you want to do work locally, please create a new branch with a descriptive name for that feature. To merge your work into the main branch, please create a pull request that can be reviewed by the team. (You can pull from your branch on the rover or base station any time you need to test with real hardware).

[CONTRIBUTING.md](CONTRIBUTING.md) has the details: the workflow and how pull requests are merged, how to change the rover, add a world or a sensor, swap or write a drive controller and run the tests, the code style and commit messages.
