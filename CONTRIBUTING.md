# Contributing

How to set up the rover's software, change it and get the change into `main`. The [README](README.md) is the short version; this file has the steps per platform and the how-tos. Report a bug or propose a feature with an issue: its templates ask for what helps.

[Before you start](#before-you-start) · [Set up](#set-up) · [Build and run](#build-and-run) · [Branches and pull requests](#branches-and-pull-requests) · [How to](#how-to) · [Code style](#code-style) · [Commit messages](#commit-messages)

## Before you start

The repository is the `src/` folder of a colcon workspace: clone it into `<workspace>/src`, and colcon builds in the folder above it (`build/`, `install/` and `log/` land there). It holds three ROS 2 packages:

| Package | What it is |
|---|---|
| [`rover_description`](rover_description/README.md) | The rover: every physical number in `config/rover.yaml`, and the URDF built from it |
| [`rover_control`](rover_control/README.md) | The drive, the same on the real and the simulated rover: the controllers, the real rover's launch file and its hardware stub, the tank kinematics |
| [`rover_sim`](rover_sim/README.md) | The Gazebo simulation: the rover's simulated hardware, the URC worlds, the driver station, the launcher and the referee |

`rover_control` depends on `rover_description`, `rover_sim` on both, and nothing depends on `rover_sim`. Programs talk to the rover, simulated or real, only through the topics of [docs/topics.md](docs/topics.md) and through ros2_control, so a program that drives the simulated rover drives the real one. The rover's computer builds `rover_description` and `rover_control` only. The design behind this layout is [docs/superpowers/specs/2026-10-09-rover-software-integration-design.md](docs/superpowers/specs/2026-10-09-rover-software-integration-design.md).

## Set up

Every platform needs git, [pixi](https://pixi.prefix.dev/latest/installation/) and about 7 GB of free disk: the pixi environment takes 5.0 GB on macOS, the generated worlds and models 0.7 GB and the terrain rasters 0.1 to 0.3 GB (M, 2026-10-10). pixi installs everything else from `pixi.lock`, the same versions on every machine: ROS 2 Jazzy from [RoboStack](https://robostack.github.io/GettingStarted.html), Gazebo Harmonic, the compilers on macOS and Linux, Python and its libraries. CI installs pixi 0.81.0; a newer one works too. If `pixi install` rewrites `pixi.lock` although you did not change `pixi.toml`, leave that out of your commit (`git restore pixi.lock`): CI installs with `--locked`, which fails when the two disagree.

### macOS (Apple silicon)

1. Install the Xcode Command Line Tools: `xcode-select --install`. They bring git, and pixi's compilers build against their macOS SDK.
2. Install pixi: `curl -fsSL https://pixi.sh/install.sh | sh`, then open a new terminal.
3. Clone the repository into a workspace and build it:

   ```bash
   mkdir -p ~/hurc_ws
   git clone https://github.com/Harvard-HURC/Rover-Software.git ~/hurc_ws/src
   cd ~/hurc_ws/src
   pixi install
   pixi run build
   ```

4. For the simulation, `pixi run fetch-data` once, then `pixi run sim` ([README, Simulation](README.md#simulation)).

The clone must be in a folder called `src`: the colcon tasks check it and stop with a message otherwise.

### Linux (Ubuntu 24.04)

The macOS steps, with git and curl from apt in place of step 1: `sudo apt update && sudo apt install -y git curl`.

- Gazebo's window, the camera views and the rendering tests need a GPU and its driver. `glxinfo -B` (`sudo apt install mesa-utils`) names the renderer; `llvmpipe` there means software rendering. Without a GPU, `pixi run test` skips the rendering tests and says why, and `ros2 launch rover_sim sim.launch.py gui:=false sensors:=false` runs the simulation with nothing rendered, as the smoke test does.
- Do not source an apt-installed ROS (`/opt/ros/jazzy/setup.bash`, often from `~/.bashrc`) in a shell you use pixi in: RoboStack warns that its `PYTHONPATH` breaks the pixi environment.
- These steps have not been run on Linux with this repository yet (2026-10-10); CI's Ubuntu job runs the same tasks headless from the first push.

### Windows: the rover's packages and RViz

Natively, Windows builds `rover_description` and `rover_control` (`rover_sim` configures to nothing there) and runs RViz and the ROS 2 tools; the simulation runs in WSL2 (next section). These steps have not been run on Windows with this repository yet (2026-10-10): they follow RoboStack's and pixi's instructions, and CI's Windows job runs `pixi run build` from the first push. Whoever goes through them first, please correct this section in a pull request.

1. Install Visual Studio 2022 with the "Desktop development with C++" workload (RoboStack needs Visual Studio 2022 with C++; the Community edition is free) and Git for Windows (`winget install --id Git.Git -e`).
2. Install pixi in PowerShell: `powershell -ExecutionPolicy Bypass -c "irm -useb https://pixi.sh/install.ps1 | iex"` (or `winget install prefix-dev.pixi`), then open a new terminal.
3. Clone into a short path (colcon's build folders nest deep, and Windows limits a path to 260 characters unless long paths are enabled) and build:

   ```powershell
   git clone https://github.com/Harvard-HURC/Rover-Software.git C:\hurc_ws\src
   cd C:\hurc_ws\src
   pixi install
   pixi run build
   ```

4. Run `pixi shell`, go to `C:\hurc_ws` and source the workspace: `call install\local_setup.bat` in cmd, `install\local_setup.ps1` in PowerShell. Then `ros2 launch rover_description view.launch.py` shows the rover in RViz with sliders for its joints.

On Windows `pixi run build` builds without the tests (`pixi.toml`, `[target.win-64.tasks]`); the tests and the simulation tasks run in WSL2.

### Windows: the simulation in WSL2

The simulation runs in Ubuntu 24.04 under WSL2, by the Linux steps; WSLg shows Gazebo's window on the Windows desktop, rendered on the GPU. This guide has not been run yet (2026-10-10): it follows Microsoft's WSL documentation ([install](https://learn.microsoft.com/en-us/windows/wsl/install), [GUI apps](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps), [networking](https://learn.microsoft.com/en-us/windows/wsl/networking), [settings](https://learn.microsoft.com/en-us/windows/wsl/wsl-config)), WSLg's [GPU selection](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg) notes and the Gazebo community's [WSLg announcement](https://discourse.openrobotics.org/t/wslg-with-gpu-support-available-on-latest-version-of-gazebo-garden-and-harmonic/48128). Please correct it in a pull request where it differs.

1. **Windows and the GPU driver.** WSLg needs Windows 11, or Windows 10 build 19044 or later (`winver` shows the build). Install the current Windows driver for your GPU from Intel, AMD or NVIDIA: WSL hands it to Linux. Do not install a Linux GPU driver inside Ubuntu.
2. **WSL and Ubuntu.** In PowerShell run as administrator:

   ```powershell
   wsl --install -d Ubuntu-24.04
   ```

   Restart Windows when asked, open "Ubuntu 24.04" from the Start menu and choose a Linux user name and password. If WSL was installed before, run `wsl --update` (it brings WSLg) and `wsl --shutdown` first. `wsl -l -v` lists the distribution with VERSION 2.
3. **Check the GPU**, in Ubuntu:

   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install -y git curl mesa-utils
   glxinfo -B | grep "OpenGL renderer"
   ```

   It should name your GPU through Direct3D 12, `D3D12 (<your GPU>)`. `llvmpipe` means software rendering: update the Windows driver, run `wsl --update` and `wsl --shutdown` in PowerShell, and check again. On a laptop with two GPUs, `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` (any part of the GPU's name in Device Manager; put the line in `~/.bashrc`) picks the discrete one.
4. **pixi and the repository**, in Ubuntu's home folder:

   ```bash
   curl -fsSL https://pixi.sh/install.sh | sh
   exec bash                      # or open a new Ubuntu terminal
   mkdir -p ~/hurc_ws
   git clone https://github.com/Harvard-HURC/Rover-Software.git ~/hurc_ws/src
   cd ~/hurc_ws/src
   pixi install
   pixi run build
   pixi run fetch-data
   ```

   Keep this clone in the Linux file system (`~/hurc_ws`), not under `/mnt/c`, which is much slower from Linux, and separate from a native Windows clone. `explorer.exe .` opens the folder in Windows' Explorer; VS Code's WSL extension edits it from Windows (`code .` in Ubuntu).
5. **Run it.** `pixi run sim` opens Gazebo's window. `pixi run drive urc_autonomy` prints the station's address (`driver station on http://127.0.0.1:8765/`, or the next free port); open it in a Windows browser, since WSL forwards `localhost` to Windows. `pixi run launcher` prints its own (port 8790). `pixi run test` runs the tests, the rendering ones too (`rover_sim/tests/gpu.py` counts WSL2's `/dev/dxg` as a GPU).
6. **Memory.** WSL2 gets half of Windows' memory by default. On a computer with 8 GB, give it more in the WSL Settings app, or in `%UserProfile%\.wslconfig`, then `wsl --shutdown`:

   ```ini
   [wsl2]
   memory=6GB
   ```

When something does not work:
- **No window** ("cannot open display"): WSLg's [troubleshooting guide](https://github.com/microsoft/wslg/wiki/Diagnosing-%22cannot-open-display%22-type-issues-with-WSLg).
- **Gazebo's window is black or Gazebo crashes** while `glxinfo` names your GPU: WSLg is not a platform Gazebo tests (the announcement above). `LIBGL_ALWAYS_SOFTWARE=1 pixi run sim` renders in software, slowly, which tells a GPU problem from another one. Open an issue with the output of `glxinfo -B`.
- **The rendering tests fail**: `ROVER_RENDERING=0 pixi run test` skips them, as on a machine without a GPU.
- **ROS tools on Windows do not see the simulation in WSL2**: ROS 2 finds its peers by multicast, which WSL2's default (NAT) network does not carry between Windows and Linux. Run RViz and the ROS tools in Ubuntu, where WSLg shows them too. WSL's mirrored networking carries multicast (Windows 11 22H2 or later: `networkingMode=mirrored` under `[wsl2]` in `.wslconfig`); untried here.

### The rover's computer

It builds `rover_description` and `rover_control` only: [rover_control/README.md](rover_control/README.md#on-the-rovers-computer).

## Build and run

From `<workspace>/src`:

| Command | What it does |
|---|---|
| `pixi run build` | `colcon build` of every package, in the workspace root |
| `pixi shell` | A shell in the environment. To use what you built, `source install/local_setup.bash` in the workspace root (`local_setup.zsh` in zsh); then `ros2 launch ...`, `ros2 topic ...` |
| `colcon build --packages-select rover_control` | One package (in `pixi shell`, in the workspace root) |
| `pixi run sim [world] [--follow]`, `pixi run drive [world]`, `pixi run launcher` | The simulation ([README, Simulation](README.md#simulation)); `pixi run drive --help` lists the station's options |
| `ros2 launch rover_sim sim.launch.py world:=<name> gui:= station:= controller:= sensors:=` | The simulation's launch file itself (in a shell that sourced `install/`): [rover_sim/README.md](rover_sim/README.md#launch-files) |
| `ros2 launch rover_control rover.launch.py` | The real rover's stack, with the hardware stub until the motor controllers are chosen ([rover_control/README.md](rover_control/README.md)) |

ROS 2 finds every node on the network in the same ROS domain, so in domain 0 two simulations on the team's network, or a simulation and the rover, drive each other ([docs/topics.md](docs/topics.md)). Give yours a domain of its own (`export ROS_DOMAIN_ID=<0 to 101>`), or keep it on your computer (`export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`).

## Branches and pull requests

The team's rules ([README, Contributing](README.md#contributing)): never push to `main`; every change is a branch and a pull request the team reviews.

1. Start from an up-to-date `main` (`git switch main && git pull`) and make a branch named for the change, `git switch -c <area>/<what>` (for example `sim/lidar-sensor`, `control/teleop-node`, `docs/wsl-fixes`).
2. Commit in steps that each build and pass the tests they touch ([Commit messages](#commit-messages)). A tracked file that a tool generates goes into the same commit as its source ([Change the rover](#change-the-rover)).
3. Push the branch (`git push -u origin <branch>`) and open a pull request against `main`, on GitHub or with `gh pr create`. Its template asks what changed and how you tested it, in the simulation, on hardware or not at all. Open it as a draft while you are still working: CI runs on it, and reviewers are requested once you mark it ready.
4. CI ([.github/workflows/ci.yml](.github/workflows/ci.yml)) runs `pixi run build`, `pixi run fetch-data` and `pixi run test` on Ubuntu 24.04 and macOS, then fails if the tests left a tracked file changed, and `pixi run build` on Windows ([README, Tests](README.md#tests)). A failed job's log names the step; the same `pixi run` task reproduces it on your computer.
5. [.github/CODEOWNERS](.github/CODEOWNERS) requests a review from the owner of each part the pull request changes. Answer comments with new commits on the branch rather than a force-push, so a reviewer sees what changed since their review. Merge when CI passes and a reviewer has approved.
6. Merge on GitHub:
   - **Create a merge commit** for a pull request whose commits are history worth keeping: the integration series (PR 1 to PR 4), the terrain data pull request, any branch whose commits each say what changed and why. Never "Rebase and merge" such a branch: the documents cite commits by hash (`rover_sim/tests/fixtures/rover_model/README.md` cites `9208047`), and a rebase gives every commit a new one.
   - **Squash and merge** suits a small change whose commits are work in progress; edit the squashed message to say what changed.
7. Delete the branch after the merge. A pull request based on that branch (stacked, as PR 2 is on PR 1) then has its base changed to `main` by GitHub and shows only its own commits.

Bring `main` into a long-lived branch with `git fetch origin && git merge origin/main` rather than a rebase, which would rewrite commits others may have pulled. To try a branch on the rover or the base station, pull it there (`git fetch && git switch <branch>`); it need not be merged first.

Requiring CI and a review on `main` (branch protection, and "Require review from Code Owners") needs a repository admin. Until an admin turns it on, these rules hold by agreement.

## How to

### Change the rover

1. Physical numbers (geometry, masses, joints, the differential, the motors, the tyres, the sensor mounts) are in `rover_description/config/rover.yaml`: keep each number's unit and say where it comes from ([Code style](#code-style)). New frames, joints and shapes go into `rover_description/urdf/rover.urdf.xacro`, the joints' ros2_control interfaces into `urdf/ros2_control.xacro`.
2. How the simulation models the rover (contact, dig-in, the stand-in motor controller's tuning, the visual cues, dust) is in `rover_sim/config/sim.yaml`; the drive controller's numbers are in `rover_control/config/controllers.yaml`.
3. `pixi run sim-model` builds the simulated rover again (`pixi run sim`, `drive` and `test` do it first). Its output is tracked: commit what `git status` shows changed (`rover_sim/models/rover/model.sdf`, and the station's camera models or the tread mesh's copy when they change) with the change. CI fails when a tracked file differs after the tests.
4. `pixi run test`. A changed rover fails `rover_sim`'s `test_equivalence` on purpose: write its fixtures again in the same commit and say why ([rover_sim/tests/fixtures/rover_model/README.md](rover_sim/tests/fixtures/rover_model/README.md)). A changed wheel radius, track or top speed also fails `rover_control`'s `test_controllers_yaml`: write the new numbers into `controllers.yaml`.
5. Look at it: `ros2 launch rover_description view.launch.py`, and `pixi run sim`.

The details are in [rover_description/README.md](rover_description/README.md#change-the-rover).

### Add a world

A world is an SDF file in `rover_sim/worlds/`. `pixi run sim <name>`, `pixi run drive <name>` and `sim.launch.py world:=<name>` open it, and the launcher shows it as a tile. It must include the rover exactly once, as `<include><uri>model://rover</uri>...</include>` at the start pose: `sim.launch.py` swaps that include for the rover with ros2_control and refuses a world with none or two.

- **By hand**, for a test course: copy `rover_sim/worlds/rover_test.sdf` to `rover_sim/worlds/<name>.sdf` and rename its `<world name="rover_test">` to `<name>`. Keep its 1 ms physics step (the rover's motor model and drivetrain are tuned on it), its systems (IMU, NavSat and Sensors feed the rover's sensors, ParticleEmitter its optional dust), its `spherical_coordinates` (where GNSS places the origin) and its rover include, and change the rest. Hand-written worlds are tracked; list it in [rover_sim/README.md](rover_sim/README.md#sdf-worlds) (the team's rule).
- **Generated**, with terrain, ground types and clutter like the URC worlds: a module `rover_sim/urc/missions/<name>.py` with `KEY` and `build(models_dir, worlds_dir, media)`, registered in `MISSIONS` or `COURSES` in `rover_sim/urc/missions/__init__.py`, then `pixi run sim-worlds <name>`. Generated worlds are git-ignored and deterministic: commit the generator, not its output. The manual's [A new world](docs/sim/manual.md#a-new-world) has a complete example; [Mission sheets](docs/sim/manual.md#mission-sheets) and [Referee](docs/sim/manual.md#referee) say how a mission is scored.

### Add a sensor

1. Its mount and numbers go into `rover.yaml` (a section like `imu:` or `gnss:`: position, rate, noise, each with unit and source), its frame into `rover.urdf.xacro`, as `imu` and `gnss` are (`<xacro:frame name="imu" parent="base_link" xyz="${rover.imu.xyz}"/>`).
2. The simulated sensor goes into `rover_sim/urdf/rover_sim.urdf.xacro`, a `<gazebo reference="<frame>"><sensor ...>` beside the IMU's, with its Gazebo topic in `rover_sim/config/sim.yaml` (`topics`). A sensor that a world system runs (the IMU and NavSat systems; the Sensors system renders cameras and lidars) needs that system in every world: `rover_sim/worlds/rover_test.sdf` and the generated worlds' list in `rover_sim/urc/world.py`.
3. Bridge it to ROS in `rover_sim/bridge.yaml` (`ros_topic_name`, `gz_topic_name`, the two types, `GZ_TO_ROS`). A heavy one (images, point clouds) gets `lazy: true`, and joins the lazy topics that `rover_sim/tests/test_sim_launch.py` (`SensorBridge`) lists.
4. Add its topic to the contract in [docs/topics.md](docs/topics.md): the real sensor's driver publishes the same name and type.
5. `pixi run sim-model`, commit the model, and write the equivalence fixtures again (step 4 of [Change the rover](#change-the-rover)): a new sensor changes the rover.

### Swap the drive controller

The controllers are in `rover_control/config/controllers.yaml`, the same file on the real and the simulated rover. Change `drive_controller`'s `type`, or add another controller beside it (its `type` under `controller_manager`, its parameters under its own name) and start it with `controller:=<name>` (`ros2 launch rover_sim sim.launch.py controller:=<name>`, `ros2 launch rover_control rover.launch.py controller:=<name>`). Keep its geometry `rover.yaml`'s: `rover_control/test/test_controllers_yaml.cpp` checks `drive_controller`'s. More in [rover_control/README.md](rover_control/README.md#change-the-drive-controller).

### Write a program that drives the rover

A program drives the rover by publishing `/cmd_vel` (geometry_msgs/TwistStamped) and reads it from `/odom`, `/imu`, `/gps/fix`, `/camera/*` and `/joint_states` ([docs/topics.md](docs/topics.md)). Publish at a steady rate (a command older than 0.5 s counts as zero, so the rover stops when the program does) and leave `header.stamp` zero (the drive controller stamps the command on arrival). This one drives forward at 0.3 m/s for 3 s:

```python
"""Drive the rover forward at 0.3 m/s for 3 s, then stop."""
import time

import rclpy
from geometry_msgs.msg import TwistStamped


def main():
    rclpy.init()
    node = rclpy.create_node('drive_forward')
    cmd_vel = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    for step in range(80):                     # 4 s at 20 Hz: a command older than 0.5 s counts as zero
        msg = TwistStamped()                   # header.stamp stays zero: the controller stamps it on arrival
        msg.twist.linear.x = 0.3 if step < 60 else 0.0   # [m/s]; twist.angular.z [rad/s] turns, left positive
        cmd_vel.publish(msg)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Run it with `python drive_forward.py` in `pixi shell` while the simulation runs (`pixi run sim`, or `pixi run drive <world> --released`, where the station watches without commanding until you take control on its page). It drove the simulated rover 0.92 m on `rover_test`, and against the real rover's stub (`rover.launch.py`) the stub logged `would send wheel_fl_joint 2.00, ...` (0.3 m/s over the 0.15 m wheel radius) (M, 2026-10-10). A program the team keeps becomes a node in a package ([Add a package](#add-a-package)); drive code belongs in `rover_control`.

### Write a drive controller

A drive controller turns `/cmd_vel` into the four wheels' speeds inside the controller manager, at its 100 Hz, on the rover and in the simulation alike. It is a C++ class derived from `controller_interface::ControllerInterface` (`controller_interface/controller_interface.hpp`, in the pixi environment), and `diff_drive_controller` ([ros2_controllers](https://github.com/ros-controls/ros2_controllers), 4.42.1 in `pixi.lock`) is the example to follow.

1. Its `command_interface_configuration()` claims `wheel_fl_joint/velocity`, `wheel_rl_joint/velocity`, `wheel_fr_joint/velocity` and `wheel_rr_joint/velocity` [rad/s]. Its `state_interface_configuration()` may read each wheel's `position`, `velocity` and `effort` (the motor current [A]) and the `position` and `velocity` of `rocker_left_joint`, `rocker_right_joint`, `camera_pan_joint` and `camera_tilt_joint` (`rover_description/urdf/ros2_control.xacro`).
2. Put it in `rover_control` (`include/rover_control/`, `src/`), export it with pluginlib under the base class `controller_interface::ControllerInterface` (a plugin description file and `pluginlib_export_plugin_description_file(controller_interface <file>.xml)`, as `rover_hardware_stub.xml` does for the hardware), add `controller_interface` to `package.xml`, and link only the libraries it uses ([rover_control/README.md](rover_control/README.md#when-the-motor-controllers-are-chosen) says why). The tank kinematics library (`tank_kinematics`, which reads `rover.yaml`) has the skid-steer arithmetic, tested.
3. Name it in `controllers.yaml` and start it with `controller:=<name>` ([Swap the drive controller](#swap-the-drive-controller)); test it in the simulation before the rover.

### Add a package

In `pixi shell`, in `<workspace>/src`: `ros2 pkg create --build-type ament_cmake <name>` (or `ament_python`). Then:
- describe it in the README's package list (the team's rule) and give it a `README.md`; add its folder to [.github/CODEOWNERS](.github/CODEOWNERS);
- keep the dependency rule: nothing depends on `rover_sim`, and rover code builds without Gazebo;
- put its ROS dependencies in `package.xml` and, when the environment lacks one, in `pixi.toml` (`[dependencies]`, or `[target.unix.dependencies]` for the simulation only; `pixi install` then updates `pixi.lock` for all three platforms; a dependency change is best a pull request of its own);
- it builds on Windows too (CI's Windows job builds every package); a package that cannot does nothing there, as `rover_sim/CMakeLists.txt` does (`if(WIN32) ... return()`);
- add it to the `colcon-test` task's `--packages-select` in `pixi.toml`, so that `pixi run test` and CI run its tests.

The repository has no license yet (every `package.xml` says `TODO: License declaration`); the team chooses one.

### Fetch the terrain data

`pixi run fetch-data` downloads the terrain rasters the simulation reads (USGS elevation, USDA NAIP imagery) from their official sources once, and checks them against `rover_sim/data/rasters.json`. `--verify` checks the rasters present and fetches nothing, `--dry-run` says what it would fetch, and `--all` adds the rasters only the research and the relief tool read. The rasters are git-ignored: never commit a `.tif`. Which raster fetch-data cannot rebuild yet, and how to get it from a teammate's clone, is in the [README, Simulation](README.md#simulation).

### Run the tests

| Command | What it runs |
|---|---|
| `pixi run test` | Everything CI runs: the colcon tests of the three packages and the simulation's suite, about 17 minutes on an Apple M4 |
| `pixi run colcon-test` | The colcon tests: `rover_description` (xacro and `check_urdf`, the team's linters), `rover_control` (GoogleTest; `rover.launch.py` on the stub), `rover_sim` (C++, `RoverSimSystem`); 11 s on an Apple M4 after a build (M). `colcon test-result --verbose` in the workspace root (in `pixi shell`) lists what failed |
| `colcon test --packages-select <package>` | One package's colcon tests (in `pixi shell`, in the workspace root), then `colcon test-result --verbose` |
| `pixi run sim-test` | The simulation's suite (`rover_sim/tests`), after building the plugins and generating the rover and the worlds |
| `cd rover_sim/tests && pixi run python -m unittest -v <module>[.<Class>[.<test>]]` | One module, class or test of the suite, after `pixi run build` (and `pixi run sim-worlds` for tests that use the generated worlds) |
| `cd rover_sim/tests && pixi run python -m unittest -v test_sim_launch.SmokeTest` | The smoke test: `sim.launch.py` headless on `rover_test`; `/cmd_vel` moves `/odom`, the topic contract's topics are there, the drive controller's timeout brakes, a world reset spawns the controllers again, Ctrl-C stops it all. 18 s on an Apple M4 (M, 2026-10-10) |
| `pixi run sim-perf`, `pixi run sim-slow`, `pixi run sim-realism` | Opt-in: the performance budgets (about 25 minutes), the mission routes (about 40 minutes), the realism report ([manual, Quick start](docs/sim/manual.md#quick-start)) |

The rendering tests skip on a machine without a GPU and say why; `ROVER_RENDERING=0` skips them anywhere and `ROVER_RENDERING=1` runs them anyway. Before you ask for a review, run `pixi run test` (or at least `pixi run colcon-test` and the suite's modules that cover your change) and check that `git status` shows no tracked file changed by the tests.

## Code style

| Where | Style |
|---|---|
| `rover_description` | The team's ament linters, run by `colcon test`: `ament_flake8` (99 columns, single quotes, Google import order), `ament_pep257`, `ament_lint_cmake`, `ament_xmllint`. A failure names the file and the line: fix the file, not the linter's configuration |
| `rover_control` | No linter. C++ and Python at 120 columns; Python single-quoted, as the team's launch files are; C++ as its sources are (two-space indent, braces on the line that opens them) |
| `rover_sim` | No linter (its `CMakeLists.txt` says why: the code came with the simulation and keeps its style, which the ROS linters reject). Python at 120 columns, double quotes; C++ in the plugins' style, 120 columns |

Everywhere:
- Comments and documents say what something is and why, in plain sentences; the design specs and plans in `docs/superpowers/` show the tone.
- Units in brackets: `[m]`, `[rad/s]`, `[A]`.
- Every number in `rover.yaml`, `sim.yaml` and `controllers.yaml` says where it comes from: (A) assumed, (M) measured or tuned, (R) from a reference (in `docs/research/`). A decision the team made is dated: "user decision 2026-10-06".
- Standard ROS messages; a custom message only when none fits, in a `rover_msgs` package then (design spec, section 4).
- New behaviour comes with a test in its package (`rover_description/test`, `rover_control/test`, `rover_sim/tests`).
- Generated and large files stay out of git (`.gitignore`): `build/`, `install/`, `log/`, the rasters, the generated worlds. The tracked generated files (the rover's model, above) are committed with their source.

## Commit messages

```
rover_description: the GNSS antenna at the mast's measured height

What changed and why, in a few short paragraphs or a list; what a reviewer should know; how it was tested (the
commands and their results, for example `pixi run colcon-test`: 29 tests, all passed).
```

- The subject says what changed, with the part of the repository first (`rover_description:`, `rover_sim:`, `rover_control:`, `docs:`). A series of pull requests may use the series' prefix, as the integration series does (`PR 3: ...`).
- The body says why, and how it was tested; wrap it at about 100 columns.
- One change per commit; a generated file goes in with its source.
- When an AI assistant wrote part of a commit, say so with a `Co-Authored-By:` line at the end, as the existing commits do.
- Commit with your name and an email GitHub knows (`git config --global user.name "..."` and `user.email "..."`), so GitHub links the commits to you.
