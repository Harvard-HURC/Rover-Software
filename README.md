# HURC Rover Software

Welcome to the central repository for the HURC Rover's control software!

### Setup
We use [pixi](https://pixi.prefix.dev/latest/#installation) to manage ROS packages with [RoboStack](https://robostack.github.io/GettingStarted.html)

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
1. `pixi run fetch-data` downloads the terrain rasters the simulation reads, once (about 130 MB of USGS elevation and lidar data and USDA NAIP imagery; git ignores them). The tasks below generate every world, so they need all of them, also for the test ground. `route_area_lidar_0p5m.tif` has no official source: it comes from the team's copy, this repository's release `data-2026-10-06`. If fetch-data cannot get a raster (that release may not be published yet), link them from a clone that has them: `rover_sim/tools/link_data.sh <that clone>`
2. `pixi run sim` builds the simulation's plugins, generates the rover and the worlds and opens Gazebo on the test ground (`pixi run sim urc_autonomy` opens a URC mission world)
3. `pixi run drive urc_autonomy` drives the rover from the browser; `pixi run launcher` shows every world as a tile that starts it with its driver station

The manual is [docs/sim/manual.md](docs/sim/manual.md), with its [design notes](docs/sim/design-notes.md) and [Gazebo lessons](docs/sim/gazebo-lessons.md).

### Tests
- `pixi run test` runs the C++ tests of `rover_control` and `rover_sim` (`colcon test`) and the simulation's suite: about 15 minutes on an Apple M4. Tests that render skip on a machine without a GPU and say why
- CI (`.github/workflows/ci.yml`) runs the same tasks on every pull request: on Ubuntu 24.04 and macOS (Apple silicon) `pixi run build`, `pixi run fetch-data` and `pixi run test` (the rendering tests on macOS only), on Windows `pixi run build`

### Platforms
- **macOS (Apple silicon):** everything
- **Linux (Ubuntu 24.04):** everything; the camera views and the rendering tests need a GPU
- **Windows:** `colcon build` (or `pixi run build`) builds `rover_description` and `rover_control` natively (`rover_sim` builds nothing there), and RViz and the ROS 2 tools run in `pixi shell`. The Gazebo simulation runs in WSL2: install Ubuntu 24.04 (`wsl --install -d Ubuntu-24.04`), install pixi inside it, clone this repository into a Linux folder (for example `~/hurc_ws/src`, not under `/mnt/c`, which is slow) and follow the steps above there. WSLg shows Gazebo's window; open the address the driver station prints in a Windows browser

## Project Structure
Most of our work is structured into folders called "packages". Whenever you create a new package, please add a description of it here:
- `rover_description`: Contains the URDFs that define the rover's structure, joints, actuators, sensors, etc
- `rover_sim`: Contains SDFs, URDFs, launch configurations, and any other tools that help with simulating the rover in Gazebo. Today it holds the URC simulation: Gazebo plugins for the drivetrain and motors, the rocker differential and the driver station's cameras; the URC 2027 mission worlds and their generator; a browser driver station, a launcher and a referee (macOS and Linux)
- `rover_control`: The rover's drive code. For now the swerve design's kinematics library `rover_driver` with its tests and tools, superseded by tank drive; the ros2_control work replaces it

Beside the packages, `docs/` holds the design specs and plans (`docs/superpowers`), the simulation's manual (`docs/sim`) and the research behind it (`docs/research`).

## Contributing
Please do not push directly to the main branch. Any time you want to do work locally, please create a new branch with a descriptive name for that feature. To merge your work into the main branch, please create a pull request that can be reviewed by the team. (You can pull from your branch on the rover or base station any time you need to test with real hardware).
