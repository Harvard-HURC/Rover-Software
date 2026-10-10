# PR 3: The ros2_control boundary Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The rover's programs drive the simulated rover exactly as they will drive the real one: `/cmd_vel` (geometry_msgs/TwistStamped) into the controllers of `rover_control/config/controllers.yaml` (diff_drive_controller and joint_state_broadcaster), per-wheel velocity commands through ros2_control into `RoverSimSystem`, which hands them to the drivetrain's motor model inside Gazebo; on the real rover the same controllers drive a hardware stub until the motor controllers are chosen; `sim.launch.py` starts it all, the driver station publishes `/cmd_vel`, and every calibration test keeps its numbers.

**Architecture:** Design spec `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`, section 6 in full, 9.2 row 3, 10 and 11. A new ECM component on each wheel joint (`rover_sim::components::WheelSetpoint`) carries the wheel's speed setpoint and the sim time it was written; `RoverDrivetrain` follows it with its motor model (PI loop, current limit, backlash) under a setpoint watchdog and writes the motor current into a second component (`WheelMotorCurrent`). `RoverSimSystem`, a `gz_ros2_control::GazeboSimSystemInterface` plugin in `rover_sim`, writes the first component from the controller manager's velocity commands and reports joint position, velocity and the motor current as effort; a ROS-free gz-transport topic, `/model/rover/wheel_cmd`, feeds the same component for the simulation's own tests and tools, whose helpers (`rover_sim/wheel_cmd.py`) turn their twists into setpoints with the skid-steer formula and ramp the plugin used until now, so their runs stay bit for bit what they were. The worlds keep including `model://rover` (the ROS-free rover); `sim.launch.py` loads a copy of the world whose rover is the post-processed model with ros2_control, at the same pose.

**Tech Stack:** ROS 2 Jazzy (RoboStack, pixi 0.81): ros2_control 4.47.0 (controller_manager, hardware_interface, ros2controlcli), ros2_controllers 4.42.1 (diff_drive_controller, joint_state_broadcaster), gz_ros2_control 1.2.19, pluginlib, robot_state_publisher, ros_gz_bridge, rclpy, launch; Gazebo Harmonic (gz-sim 8.10, gz-transport 13, gz-msgs 10); yaml-cpp 0.8; GoogleTest 1.18; Python 3.12 unittest.

---

## Places and rules

| Name | Path |
|---|---|
| SRC | `/Users/alarion239/Desktop/hurc_ws/src` (the team's clone; branch `pr3/ros2-control`, made from `pr2/rover-description` at `1cf53cd`) |
| WS | `/Users/alarion239/Desktop/hurc_ws` (the colcon workspace; `build/`, `install/`, `log/` are here) |
| T | `/private/tmp/claude-502/rover-pr3` (scripts, logs, the baseline traces; deleted in Task 21). If the implementing session lists a scratchpad directory, use `<scratchpad>/rover-pr3` as T in every command instead: the sandbox may refuse writes elsewhere |
| P | `/Users/alarion239/.pixi/bin/pixi` (0.81) |
| S | `/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/pr3src` (shallow clones of gz_ros2_control 1.2.19, ros2_control 4.47.0, ros2_controllers 4.42.1 and gz-math 7.7.0 that the planning session read; reference only, never built; deleted in Task 21) |

- **Never push, never change anything on GitHub, never open a pull request.** The user approves every push. `/Users/alarion239/Desktop/Rover` is read-only (the rasters in `rover_sim/data` are symlinks into it).
- **Never create a git worktree or a clone inside WS**: colcon would find the packages twice. Scratch copies go under T.
- Every commit message starts `PR 3: ` and ends with the line `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`; the author is the configured `Alexander Belotserkovtsev <amb30239@gmail.com>`.
- **Isolation.** Every Gazebo or ROS run outside the test suite gets `GZ_IP=127.0.0.1`, its own `GZ_PARTITION`, its own `ROS_DOMAIN_ID` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; the commands below set them (domains 63 to 66 for the manual runs). The suite sets its own (`gzenv.private_ros_domain`, 50 to 99, from the process id). Run `ros2` CLI commands with `--no-daemon` where they take it, so no ROS daemon is left behind. Stop only processes you started: ports 8765 and 8766, a heavy `EuclidPolish` Python job and a ROS daemon on domain 87 belong to other work.
- Shell state does not persist between tool calls: every command block sets the variables it uses and starts with `cd`.
- The Bash tool stops foreground commands after 10 minutes: run `pixi run test`, `pixi run sim-test` and the baseline in the background (`run_in_background`) with the output in a log under `$T`, and wait for them.
- Disk: about 26 GB free. Keep `$T` small (logs, two `.npz` files); no Docker; one pixi environment, the one in SRC (the lock adds 31 packages, about 60 MB).
- Style: the repository's (PR 1 and PR 2 plans, `rover_sim/README.md`): short declarative sentences, units in brackets, numbers with their source label (A) assumed, (M) measured or tuned, (R) from a reference, and dated user decisions. Python in `rover_sim` at 120 columns, double quotes; C++ in `rover_sim` at 120 columns in the plugins' style. `rover_control`'s new code (C++ and Python, its launch files and test single-quoted as the team's) at 120 columns too: nothing lints `rover_control` (no `ament_lint_auto`), and the swerve code's 100 columns leave with it. `rover_description` keeps the team's linters (`ament_flake8`: 99 columns); PR 3 changes only comments there.
- **Edits.** A step that changes an existing file gives each change as the exact text to find and the exact text to put instead (the Edit tool's `old_string` and `new_string`); the old text was read from the branch at `1cf53cd`. A step whose old text is not in the file means an earlier task differs from this plan: find the difference instead of forcing the edit. New files are given whole.

## Decisions this plan makes

The spec and the task leave these open; each is final for PR 3 and stated in the commit or document that implements it. Section 12 of the spec (Task 21) records the ones that deviate from the spec's text.

1. **The prototype is built from the pieces the PR keeps, on the branch, and gates the rest (Part A).** It adds the setpoint component and the drivetrain's setpoint input beside the twist input, `RoverSimSystem` with its unit test, the ros2_control variant of the simulated rover and `controllers.yaml`, then runs `RoverSimSystem` inside `gz sim -s` on this Mac with the real controllers (`tests/test_ros2_control.py`) and opens Gazebo's GUI as a separate process beside it. The measured facts go into this plan (Task 6, "Prototype results"). Parts B to E start only if gz_ros2_control loads `RoverSimSystem`, the controllers drive the wheels, the step timing is measured, and the readback matches; otherwise the implementer stops and reports (Risks). Linux runs the same test in CI once the user pushes.
2. **The setpoint component** (`rover_sim/plugins/rover_components.hh`): `WheelSetpoint`, a `Setpoint {velocity [rad/s], time (when it was written, on the writer's sim clock), ros2_control (written by RoverSimSystem)}` on each wheel joint, and `WheelMotorCurrent` [A]. Both libraries that use them (the `RoverDrivetrain` system plugin and the `rover_sim_system` pluginlib library) register them under the same names; gz-sim's component factory supports one component registered from several libraries (`Factory::Register` sets the type's static id in each library and adds the descriptor once; the plan critic's probe, 2026-10-10, saw a component RoverSimSystem created read by another system plugin in the same Gazebo server). `time` is the time `write()` is given for RoverSimSystem's setpoints and the drivetrain's sim time at arrival for the ROS-free topic's. ros2_control 4.47 ignores the time gz_ros2_control passes the controller manager: `ResourceManager::read`/`write` hand the hardware the controller manager's clock (`resource_manager.cpp`, `current_time = ...get_clock()->now()`), and `ControllerManager::update` hands the controllers `this->now()`. Under gz_ros2_control that clock is sim time from the bridged `/clock` (the controller manager runs with `use_sim_time`), which the probe measured 1 to 16 ms (mean 4.4 ms) behind the step. The watchdog (Decision 5) works on that stamp: the lag is far below `cmd_timeout`, and if `/clock` stops, the stamps freeze and the motors stop after `cmd_timeout`. That is the case where diff_drive_controller's own `cmd_vel_timeout` cannot work, because its clock is frozen too.
3. **Same-step timing comes from the plugins' order, not from system priorities.** gz-sim 8 runs PreUpdates by priority, then in load order. The ros2_control variant puts the gz_ros2_control plugin just before `RoverDrivetrain` in the model. gz_ros2_control runs the controller manager's `read()` and `update()` in its PostUpdate of every 10th step: its own count on sim time from 0, 100 Hz on the 1 ms step, so steps 10, 20, 30 and so on. It calls `write()` in every PreUpdate. So a setpoint the controllers compute at the end of step 10k is handed over at the start of step 10k + 1, and the motor model uses it in that same step. `tests/test_ros2_control.py` checks this from the drivetrain's per-step state: every change of a setpoint falls in a step 11, 21, 31 and so on; one step late would be 12, 22, 32. The stamps of the controllers' own messages (`cmd_vel_out`, `/joint_states`) cannot show it. They carry the controller manager's clock (Decision 2), and the probe saw them irregular, 8 to 15 ms apart. The plan critic's probe on this Mac (2026-10-10) ran gz_ros2_control 1.2.19 with this plan's RoverSimSystem and a stand-in for the drivetrain's command path after it in the model, under `gz sim -s`, with the plan's `controllers.yaml`. All 42 setpoint changes fell in steps that are 1 mod 10. `RoverDrivetrain` gets no `ISystemConfigurePriority`: that would also move it relative to the tests' TestFixture helper system and shift every ROS-free run by one step.
4. **The ROS-free setpoint topic** is `/model/rover/wheel_cmd` (`sim.yaml` `topics.wheel_cmd`), a `gz.msgs.Double_V` with one value per wheel in the drivetrain's `<wheel>` order, fl, rl, fr, rr [rad/s]. The drivetrain copies the newest message into the four components at the start of its next PreUpdate, stamped with the sim time it came at; a message of another length is refused with one error. While a wheel's component says `ros2_control`, the topic is ignored and the drivetrain says so once; both messages go out at Gazebo's error level (`gzerr`), the one `gz sim` prints at its default verbosity (1), so the warning is seen.
5. **The watchdog stays the drivetrain's** (`sim.yaml` `drivetrain.cmd_timeout`, 0.5 s, and `cmd_timeout_clock`, wall or sim), now per wheel setpoint: a setpoint older than `cmd_timeout` counts as zero, at once (a motor controller's watchdog stops its motor; its own PI and current limit decelerate it). `cmd_timeout: 0` still holds the last setpoint. The drivetrain also keeps the clamp to `<controller><max_speed>` (rover.yaml's top speed), a motor controller's velocity limit.
6. **Kinematics, `track_multiplier` and the ramp leave the plugin.** For the ROS stack they are the drive controller's (`controllers.yaml`: `wheel_separation`, `wheel_radius`, `wheel_separation_multiplier: 1.0`, `linear.x.max_acceleration: 1.2`, `angular.z.max_acceleration: 3.0`, each a wheel's 8 rad/s² driving straight or turning in place). For the ROS-free helpers they are `rover_sim/wheel_cmd.py` with `sim.yaml` `wheel_cmd.accel: 8.0`: the plugin's formula `(vx - side wz track / 2) / radius` and gz::math::SpeedLimiter's arithmetic (`acc = (target - previous) / dt`, clamped, `previous + acc dt`), step by step, so that a run's setpoints are the doubles the plugin made (Task 10 compares traces bit for bit). `previous + acc dt` is rounded once, a fused multiply-add. That is how libgz-math7 computes it on osx-arm64: the conda-forge build compiles `SpeedLimiter::LimitAcceleration` to `fmadd` (the plan critic disassembled it, 2026-10-10). Two roundings differ from it in the step that reaches a target. For example, the drivetrain's ramp from -2.67 rad/s down to 0 left residues of 1e-17, 1e-33 and so on for 19 steps where two roundings land on 0, and that ramp is the spin-stop calibration's. Python 3.12 has no `math.fma`, so `Ramp` adds the exact rational product and rounds once (`fractions.Fraction`; only in steps whose acceleration is not 0, about 0.01 s of a 6 s run). A probe compared it with libgz-math7's own function over four ramps, 4000 steps each, and found no difference. On linux-64, where libgz-math7 rounds twice, the helpers' last bit may differ from the plugin's before PR 3 in such a step. Nothing compares them there. The helpers use the default rover's geometry (`rover_config.ROVER`); no test drives a variant with another track, radius or top speed (a `Driver` takes `rover=` for one). `tests/test_wheel_cmd.py` checks that `controllers.yaml`'s ramp and `wheel_cmd.accel` are the same wheel rate. `sim.yaml` loses `drivetrain.track_multiplier` and `drivetrain.accel`; tests that set `drivetrain={"accel": ...}` set `wheel_cmd={"accel": ...}` instead, with the same numbers.
7. **The drivetrain's state message keeps its keys.** `"cmd"` was the twist the plugin applied; it is now the twist the setpoints in effect amount to (no-slip: `vx = r (left + right) / 2`, `wz = r (right - left) / track`, each side's mean), so the station's "asked turn" keeps reading `drivetrain.cmd[1]` and now also shows what autonomy's controller asks. The tests that compare `"cmd"` with `assertEqual` (`[0.3, 0.2]`, `[0.5, 0.0]`, `[0.0, 0.0]`) keep passing: the message prints 6 significant digits, and the twist computed back from the setpoints differs from the commanded one only in the 16th.
8. **The equivalence test keeps the frozen models of `gen_model.py`.** The drivetrain's command interface changed on purpose and nothing physical did, so `test_equivalence.ros2_control_boundary()` applies exactly that change to a frozen model (removes `<topic>`, `<track_multiplier>` and `<controller><accel>` from `RoverDrivetrain`, adds `<setpoint_topic>`) before comparing; a test checks that it changes those four elements and nothing else. The bit-for-bit drive test is unchanged.
9. **`controllers.yaml`** (`rover_control/config/`) names the controllers `joint_state_broadcaster` and `drive_controller` (type `diff_drive_controller/DiffDriveController`: left FL and RL, right FR and RR). The team's own drive controller replaces it by changing `drive_controller`'s type, or is added beside it and started with `controller:=<name>`. The geometry is written out (0.8 m, 0.15 m, 3 m/s, 7.5 rad/s) and `rover_control`'s `test_controllers_yaml.cpp` checks every number against `rover.yaml`; a YAML file cannot include another, and a launch-time override would tie both launch files to diff_drive_controller's parameter names. `update_rate: 100` (A), the same on the rover and in the simulation. `rover_control/launch/controllers.launch.py` spawns both controllers for both launches, remapping the drive controller's `~/cmd_vel` and `~/odom` to `/cmd_vel` and `/odom`. The parameter names are diff_drive_controller 4.42.1's (`src/diff_drive_controller_parameter.yaml`):
   - there is no `wheels_per_side`: the count comes from the name lists, and odometry averages each side's wheels;
   - the controller takes TwistStamped only;
   - the `has_*_limits` flags are deprecated and default to true, so every limit that is set is in force;
   - `max_deceleration` must be at most 0, and the `*_reverse` limits default to the forward ones mirrored;
   - the default `tf_frame_prefix_enable` under the root namespace gives the frames `odom` and `base_link` unprefixed.

   The plan critic's probe ran this `controllers.yaml` unchanged with the spawner arguments of `controllers.launch.py`. Both controllers configured and activated, `/odom` moved, and `node_options_args` carried the remaps. Under ros2_control the setpoints change at the controllers' 100 Hz. A ramp therefore reaches the motors as 10 ms stairs: a wheel's 8 rad/s² comes as 0.08 rad/s every 10 ms, as a real motor controller receives its commands. The controllers also ramp with the period their clock measures, 8 to 15 ms in the probe, 10 ms on average. This is new behaviour of the ROS stack only. The ROS-free helpers still ramp in every 1 ms step, so the calibration runs are unchanged.
10. **Specific packages instead of the metapackages.** `rover_control` and pixi depend on `controller_manager`, `hardware_interface`, `diff_drive_controller`, `joint_state_broadcaster` and `ros2controlcli`, not on `ros2_control` and `ros2_controllers` (the latter brings every controller); the rover's computer installs what it runs.
11. **The hardware stub activates.** The spec says it "refuses to activate with a clear message if asked to drive"; activating with zero motion lets `rover.launch.py` start the controllers on the rover's computer, which the acceptance asks for. It reports zero position, velocity and effort, warns at start and on activation that no motors are connected, and logs the wheel commands it would send, at most once a second.
12. **The worlds keep including `model://rover`; the launch swaps it.** Every ROS-free path (the suite's TestFixture runs, `world_copy`, the referee, the maps, the realism report, the station's tests) loads the worlds as they are, and gz_ros2_control in a model blocks Gazebo's start until `/robot_description` arrives, so the ROS rover cannot be the worlds' `model://rover`. `sim.launch.py` builds the ROS rover per launch (`gen_rover.build_urdf(..., ros2_control=True)`, converted and post-processed by `gen_rover.convert`: PR 2's Decision 1), writes it and a copy of the world whose one rover include points at it into a temporary folder, and runs Gazebo on the copy; the include keeps its pose, which `urc/world.py` writes from the same numbers as the mission sheet's `rover_start` (`rover_test.sdf`, which has no sheet, keeps its origin). robot_state_publisher publishes the same URDF. No spawn service is involved, so the rover is in the world from the first step, as the station's cameras expect.
13. **`pixi run drive <world>` and the launcher's tiles start the stack through `sim.launch.py`, with the station as its parent.** The station's `start_simulation` runs `ros2 launch rover_sim sim.launch.py world:=<path> gui:=false station:=false` where it ran `gz sim -s -r`, in its own session, and keeps its tested stop handling (Ctrl-C, SIGTERM and SIGHUP stop everything; `ros2 launch` does not handle SIGHUP). `sim.launch.py station:=true` starts the station attached (`--world-name <name> --wait 120`) for `ros2 launch` users. `pixi run sim` (`run.sh`) runs `sim.launch.py gui:=true` and keeps its `--follow`.
14. **The station publishes `/cmd_vel` through rclpy** (`station/ros_link.py`, `CmdVel`), TwistStamped with a zero stamp: diff_drive_controller stamps a zero-stamped command with its own clock (sim time in the simulation), so its `cmd_vel_timeout` counts from arrival whatever clock the station has; a wall-clock stamp under sim time would never time out. `CmdVel` starts rclpy with `SignalHandlerOptions.NO`, as the controller spawner does. The station handles Ctrl-C, SIGTERM and SIGHUP itself. rclpy's default handlers shut its context down on SIGINT and SIGTERM before the station's own handler runs. A probe (2026-10-10) showed that the publish that follows then fails: `RCLError: publisher's context is invalid`. That publish is the stop's zero twist (`Station.shutdown`). `CmdVel` sends no command once rclpy is down. The publisher needs no executor or spinning: the station publishes from its asyncio thread, and the unheard-warning timer only counts subscriptions. `GzLink` takes its drive output as an argument: the station's `main` gives it `CmdVel`, the default (and the tests) `wheel_cmd.TwistDrive`, which publishes the station's already-slewed twist as unramped wheel setpoints on the ROS-free topic. When nothing subscribes to `/cmd_vel` 10 s after the start, the station says so.
15. **The sensor bridge is `bridge.yaml` without `/cmd_vel`, `/odom`, `/joint_states` and `/tf`**, which the controllers and robot_state_publisher now provide; `sim.launch.py` runs it with `use_sim_time`. `/clock` must be bridged: the controller nodes' clock (`use_sim_time`, forced by gz_ros2_control) stamps the zero-stamped commands. The `sim-bridge` pixi task is removed.
16. **The environment comes from `gzenv.py`, as before.** It appends the pixi environment's `lib` to `GZ_SIM_SYSTEM_PLUGIN_PATH` (gz_ros2_control's system plugin lives there; ros_gz_sim's own launch file adds `LD_LIBRARY_PATH` the same way), puts the workspace's `install/rover_sim` first on `AMENT_PREFIX_PATH` once it exists (pluginlib finds `rover_sim/RoverSimSystem` there without a sourced `install/`), and sets a private ROS domain with `ros_domain=` (`ROS_DOMAIN_ID` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`). The tests (through `simulate.py`) and the launcher's worlds take `gzenv.private_ros_domain(pid)`, 50 to 99.
17. **`sim.launch.py` starts the GUI as a separate process on every platform** (`gz sim -g`; one code path; the team's `gazebo.launch.py` splits only on macOS because there it must), and `sensors:=false` drops the Sensors system for runs without a GPU (the smoke test, CI's Linux runner). The launch runs the simulation from its source tree, as `gzenv.py` and `rover_config.py` do; the installed copy finds the tree through `share/rover_sim/source_dir`, which `CMakeLists.txt` writes, and `rover_sim` now also installs `urdf/` and `config/` (PR 2's Decision 1).
18. **`rover_control`'s new code**: `tank_kinematics`, a static library (skid-steer inverse and forward kinematics, saturation that keeps the turn radius, and `LoadRoverYaml` with yaml-cpp; geometry from a `rover.yaml` or from numbers a node takes from ROS parameters); `rover_hardware_stub`, the pluginlib library; tests in `test/` (GoogleTest, and `test_rover_launch.py` under CTest as `rover_description` runs its unittest). The swerve library, `rover_state`, `viz.py`, their tests and `config.hpp` are removed (they stay in git history), with the `control-viz` task and `eigen` in `pixi.toml`.
19. **Documents**: the topic contract (spec 6.2) in `docs/topics.md`, with the stamp rule and the simulation's own Gazebo topics; the README, `rover_sim/README.md`, `rover_control/README.md`, `rover_description/README.md`, `docs/sim/manual.md` and `docs/sim/design-notes.md` updated; spec section 12 "As built (PR 3)".

## File structure

Created:
- `rover_sim/plugins/rover_components.hh`: the two ECM components and their stream operators; `rover_sim/plugins/tests/test_rover_components.cpp`.
- `rover_sim/ros2_control/rover_sim_system.hpp`, `.cpp`: `RoverSimSystem`; `rover_sim/rover_sim_system.xml`: its pluginlib description; `rover_sim/ros2_control/test/test_rover_sim_system.cpp`: GoogleTest.
- `rover_sim/wheel_cmd.py`: the ROS-free setpoints (formula, `Ramp`, `Driver`, `TwistDrive`).
- `rover_sim/launch/sim.launch.py`.
- `rover_sim/station/ros_link.py`: the station's `/cmd_vel`.
- `rover_sim/tests/rosstack.py` (processes and the probe node of the ROS tests), `tests/test_wheel_cmd.py`, `tests/test_ros2_control.py`, `tests/test_sim_launch.py`. The bit-for-bit check (`traces.py`) stays in T: it compares two versions once.
- `rover_control/config/controllers.yaml`, `launch/controllers.launch.py`, `launch/rover.launch.py`.
- `rover_control/include/rover_control/tank_kinematics.hpp`, `src/tank_kinematics.cpp`; `include/rover_control/rover_hardware_stub.hpp`, `src/rover_hardware_stub.cpp`, `rover_hardware_stub.xml`.
- `rover_control/test/test_tank_kinematics.cpp`, `test_controllers_yaml.cpp`, `test_rover_hardware_stub.cpp`, `test_rover_launch.py`.
- `docs/topics.md`.

Removed: `rover_control/include/rover_driver/` (5 headers), `rover_control/src/kinematics_2d.cpp`, `kinematics_3d.cpp`, `steering.cpp`, `rover_control/tests/` (5 files), `rover_control/tools/` (2 files).

Modified: `pixi.toml`, `pixi.lock`, `.github/workflows/ci.yml` (one environment variable), `README.md`; `rover_description/README.md`, `config/rover.yaml` (one comment), `urdf/rover.urdf.xacro` and `urdf/ros2_control.xacro` (comments); `rover_sim/CMakeLists.txt`, `package.xml`, `README.md`, `config/sim.yaml`, `urdf/rover_sim.urdf.xacro`, `plugins/rover_drivetrain.cpp`, `plugins/tests/CMakeLists.txt`, `rover_config.py`, `gzenv.py`, `run.sh`, `bridge.yaml`, `tools/gen_rover.py`, `station/__main__.py`, `station/link.py`, `launcher/__main__.py`, `models/rover/model.sdf` (regenerated), `tests/simulate.py`, `tests/test_drivetrain.py`, `tests/test_rover_sim.py`, `tests/test_render.py`, `tests/test_urc_terrain.py`, `tests/test_station.py`, `tests/test_fly_camera.py`, `tests/test_rover_model.py`, `tests/test_equivalence.py`, `tests/fixtures/rover_model/README.md`, `tests/test_rover_config.py`, `tests/test_foundations.py`, `tests/test_launcher.py`; `rover_control/CMakeLists.txt`, `package.xml`, `README.md`; `docs/sim/manual.md`, `docs/sim/design-notes.md`; the spec (section 12).

Unchanged on purpose: the motor, contact, dig-in, visual-cue and odometry code of the drivetrain (`rover_drivetrain.hh` and everything in `rover_drivetrain.cpp` but the command path and the state message's `"cmd"`), the world generator and every world and mission sheet (they regenerate byte for byte: the generator reads `rover.yaml`'s numbers, not its comments), the frozen fixtures, `rover_sim/launch/gazebo.launch.py`, the referee, the judge, the station's page.

---

## Part A: the prototype (the risky boundary first, from the pieces the PR keeps)

### Task 1: The starting point and the baseline

**Files:** none in SRC; `$T/list_tests.py`, `$T/traces.py`, `$T/lockdiff.py`, `$T/tests_before.txt`, `$T/baseline.log`, `$T/traces_before.npz`.

- [ ] **Step 1: The branch**

```bash
T=/private/tmp/claude-502/rover-pr3; mkdir -p $T
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain && git rev-parse --abbrev-ref HEAD && git log --oneline -3
```

Expected: no status lines; `pr3/ros2-control`; the plan's two commits (`Plan PR 3: critic fixes ...` on `Plan PR 3: the ros2_control boundary`) on top of `1cf53cd Plan PR 2: restacked on PR 1's fixes; the lock with the data pull request`.

- [ ] **Step 2: The helper scripts**

Write `$T/list_tests.py`:

```python
"""Print the id of every test of the simulation's suite, sorted, without running them: list_tests.py <tests dir>."""
import sys
import unittest

sys.path.insert(0, sys.argv[1])
suite = unittest.defaultTestLoader.discover(sys.argv[1])


def walk(tests):
    for test in tests:
        if isinstance(test, unittest.TestSuite):
            yield from walk(test)
        else:
            yield test.id()


print("\n".join(sorted(walk(suite))))
```

Write `$T/traces.py`:

```python
"""Base_link traces, every step, of the runs the calibration tests make, to compare before and after PR 3's
twist-to-setpoint helpers bit for bit (plan Tasks 1 and 10): python traces.py <out.npz>."""
import sys
from pathlib import Path

TESTS = Path("/Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests")
sys.path.insert(0, str(TESTS))

import numpy as np  # noqa: E402

from simulate import diffdrive, follow, ground_row, ground_world, physical, simulate, world_file  # noqa: E402
from rover_config import SIM  # noqa: E402
from urc import terrain, terrains  # noqa: E402

SPIN_STOP = [(0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (3.0, 0.0, 0.0)]  # test_drivetrain.Response
ARC = [(0.0, 0.0, 0.0), (0.5, 0.5, 0.3), (2.0, 0.0, 1.0)]  # test_equivalence.Drives
TOUR = [(0.0, 0.0, 0.0), (1.0, 0.8, 0.0), (3.0, 0.0, 1.0), (4.5, 0.8, 0.2)]


def no_ramp():
    """The physical rover without the ramp: sim.yaml drivetrain accel 0 before PR 3, wheel_cmd accel 0 after it."""
    if "wheel_cmd" in SIM and "accel" not in SIM.drivetrain:
        return physical(contact={"dig": False}, wheel_cmd={"accel": 0.0})
    return physical(contact={"dig": False}, drivetrain={"accel": 0.0})


def main(out):
    traces = {
        "model_rover": simulate(6.0, cmd=TOUR, trace_every=1).trace,
        "spin_stop": simulate(4.0, cmd=SPIN_STOP, params=physical(contact={"dig": False}), trace_every=1).trace,
        "spin_stop_no_ramp": simulate(4.0, cmd=SPIN_STOP, params=no_ramp(), trace_every=1).trace,
        "arc_sand": simulate(3.0, cmd=ARC, params=physical(), default_surface="sand", trace_every=1).trace,
        "diffdrive": simulate(3.0, cmd=ARC, params=diffdrive(), trace_every=1).trace,
    }
    with world_file(params=physical(contact={"dig": False})) as world:
        traces["follow"] = follow(world, [(0.0, 0.0), (5.0, 2.0), (10.0, 0.0)], 12.0).trace
    flat = terrain.Heightfield(32.0, 65)
    rows = [ground_row(7, "sand", terrains.SAND.traction, 0.8), ground_row(12, "rock", terrains.ROCK.traction, 0.1),
            ground_row(20, "manmade", terrains.MANMADE.traction, 0.0)]
    with ground_world(flat, np.full((65, 65), 7, dtype=np.uint8), rows, params=physical(),
                      ground_options=dict(terrain_default="rock", object_default="manmade")) as world:
        traces["sand_dig_spin"] = simulate(6.0, world=world, cmd=[(0.0, 0.0, 0.0), (0.5, 0.0, 1.0)],
                                           trace_every=1).trace
    np.savez(out, **traces)
    for name, trace in traces.items():
        print(f"{name}: {len(trace)} rows, last {np.round(trace[-1][:4], 6).tolist()}")


if __name__ == "__main__":
    main(sys.argv[1])
```

Write `$T/lockdiff.py`:

```python
"""Packages one pixi.lock has and the other lacks, per platform: python lockdiff.py OLD NEW."""
import sys

import yaml


def packages(path):
    environment = yaml.safe_load(open(path))["environments"]["default"]["packages"]
    return {platform: {next(iter(p.values())).rsplit("/", 1)[-1] for p in found}
            for platform, found in environment.items()}


old, new = packages(sys.argv[1]), packages(sys.argv[2])
for platform in sorted(new):
    added, removed = sorted(new[platform] - old[platform]), sorted(old[platform] - new[platform])
    print(f"{platform}: {len(added)} added, {len(removed)} removed")
    for name in added:
        print(f"  + {name}")
    for name in removed:
        print(f"  - {name}")
```

- [ ] **Step 3: Build, and the suite as it is**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run build 2>&1 | tail -3
```

Expected: `Summary: 3 packages finished [...]`. Then run in the background (`run_in_background`), about 15 minutes:

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run test > $T/baseline.log 2>&1; echo "exit $?" >> $T/baseline.log
```

When it is done:

```bash
T=/private/tmp/claude-502/rover-pr3
grep -E "tests passed|tests failed|^Ran |^OK|^FAILED|^exit" $T/baseline.log
```

Expected (PR 2's counts after the restack): `100% tests passed, 0 tests failed out of 57` (rover_control), `... out of 5` (rover_description), `... out of 3` (rover_sim), `Ran 551 tests in ...s`, `OK (skipped=9, expected failures=2)`, `exit 0`. Record the lines; Task 18 compares.

- [ ] **Step 4: The test ids and the traces**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run python $T/list_tests.py rover_sim/tests > $T/tests_before.txt 2>/dev/null; wc -l < $T/tests_before.txt
/Users/alarion239/.pixi/bin/pixi run python $T/traces.py $T/traces_before.npz 2>&1 | grep rows
```

Expected: `551`; then seven lines `model_rover: 6000 rows, ...`, `spin_stop: 4000 rows`, `spin_stop_no_ramp: 4000 rows`, `arc_sand: 3000 rows`, `diffdrive: 3000 rows`, `follow: <about 120> rows`, `sand_dig_spin: 6000 rows`. Nothing to commit.

### Task 2: pixi: ros2_control, the controllers, gz_ros2_control

**Files:** Modify `pixi.toml`, `pixi.lock`.

- [ ] **Step 1: The dependencies**

In `pixi.toml` replace

```toml
# rover_control: the kinematics library and its tests
eigen = ">=5.0.1,<6"
gtest = ">=1.18.0,<2"
```

with

```toml
# rover_control: the kinematics library and its tests
eigen = ">=5.0.1,<6"
gtest = ">=1.18.0,<2"
yaml-cpp = ">=0.8.0,<0.9"
# The drive boundary (design spec 2026-10-09, section 6): the controller manager and the hardware interface that the
# rover's hardware stub and the simulation's RoverSimSystem implement, the drive controller, the joint state
# broadcaster and `ros2 control`; the packages themselves, not the ros2_control and ros2_controllers metapackages
ros2-controller-manager = ">=4.47.0,<5"
ros2-hardware-interface = ">=4.47.0,<5"
ros2-diff-drive-controller = ">=4.42.1,<5"
ros2-joint-state-broadcaster = ">=4.42.1,<5"
ros2-ros2controlcli = ">=4.47.0,<5"
```

and replace the last line of the file

```toml
aiohttp = ">=3.14.3,<4"
```

with

```toml
aiohttp = ">=3.14.3,<4"
# gz_ros2_control: the Gazebo system that runs the controller manager inside the simulation, with RoverSimSystem
ros2-gz-ros2-control = ">=1.2.19,<2"
```

- [ ] **Step 2: Lock, compare, install**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && git show HEAD:pixi.lock > $T/pixi.lock.before
/Users/alarion239/.pixi/bin/pixi lock 2>&1 | tail -2
/Users/alarion239/.pixi/bin/pixi run python $T/lockdiff.py $T/pixi.lock.before pixi.lock | grep -E "added|removed|gz-ros2|diff-drive|hardware-interface"
```

Expected (the planning session's dry run, 2026-10-10): `linux-64: 31 added, 0 removed`, `osx-arm64: 31 added, 0 removed`, `win-64: 29 added, 0 removed` (win-64 lacks `ros2-gz-ros2-control` and `ros2-ros2-control-cmake`), among them `ros2-gz-ros2-control-1.2.19-np2py312..._21`, `ros2-diff-drive-controller-4.42.1-..._21`, `ros2-hardware-interface-4.47.0-..._21`: every new package is a `_21` build like the rest of the environment, and nothing is updated. If any package is updated or removed, stop: the lock would move the team's environment; report the lines to the user. Then:

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi install 2>&1 | tail -1
/Users/alarion239/.pixi/bin/pixi run bash -c 'ls "$CONDA_PREFIX"/lib/libgz_ros2_control-system.* "$CONDA_PREFIX"/lib/libgz_hardware_plugins.*; for p in gz_ros2_control controller_manager hardware_interface diff_drive_controller joint_state_broadcaster; do ros2 pkg prefix $p; done; echo "GZ_SIM_SYSTEM_PLUGIN_PATH=[$GZ_SIM_SYSTEM_PLUGIN_PATH]"; ls "$CONDA_PREFIX"/include/gz_ros2_control/gz_ros2_control/gz_system_interface.hpp'
```

Expected: `✔ The default environment has been installed.`; the two `.dylib` files; five times the environment's prefix (`.../src/.pixi/envs/default`); `GZ_SIM_SYSTEM_PLUGIN_PATH=[]` (no activation script sets it: `gzenv.py` adds the environment's `lib`, Task 5); the header path. Record the plugin-path line for Task 6's results.

- [ ] **Step 3: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add pixi.toml pixi.lock && git commit -q -F - <<'EOF'
PR 3: pixi: ros2_control, the drive controller, gz_ros2_control

controller_manager, hardware_interface and ros2controlcli 4.47.0 (ros2_control), diff_drive_controller and
joint_state_broadcaster 4.42.1 (ros2_controllers), gz_ros2_control 1.2.19 on macOS and Linux, and yaml-cpp for
rover_control's rover.yaml loader. The packages themselves, not the metapackages: the rover's computer installs what
it runs. The lock adds 31 packages on osx-arm64 and linux-64 and 29 on win-64, all builds of the environment's
RoboStack snapshot (_21), and changes no other package.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 3: The setpoint component, and the drivetrain's setpoint input beside the twist

**Files:** Create `rover_sim/plugins/rover_components.hh`, `rover_sim/plugins/tests/test_rover_components.cpp`. Modify `rover_sim/plugins/tests/CMakeLists.txt`, `rover_sim/plugins/rover_drivetrain.cpp`.

The drivetrain follows a wheel's `WheelSetpoint` component when the wheel has one (only `RoverSimSystem` creates one so far) and its twist input otherwise, so every ROS-free run is unchanged; Task 9 removes the twist input.

- [ ] **Step 1: The failing test**

Write `rover_sim/plugins/tests/test_rover_components.cpp`:

```cpp
// The rover's ECM components (rover_components.hh): a setpoint compares and streams as its three fields, so the ECM's
// change tracking and its serialization see each of them, and both components carry their data.
#include <chrono>
#include <sstream>
#include <string>

#include "check.hh"
#include "rover_components.hh"

namespace components = rover_sim::components;

namespace {

void SetpointEquality() {
  const components::Setpoint a{2.5, std::chrono::milliseconds(1234), true};
  components::Setpoint b = a;
  CHECK(a == b);
  b.velocity = 2.4;
  CHECK(a != b);
  b = a;
  b.time = std::chrono::milliseconds(1235);
  CHECK(a != b);
  b = a;
  b.ros2_control = false;
  CHECK(a != b);
}

void SetpointStreams() {
  const components::Setpoint a{-3.25, std::chrono::nanoseconds(1500000001), true};
  std::stringstream text;
  text << a;
  CHECK(text.str() == "-3.25 1500000001 1");
  components::Setpoint b;
  text >> b;
  CHECK(a == b);
  const components::Setpoint fine{0.1 + 0.2, std::chrono::nanoseconds(0), false};  // 17 digits survive
  std::stringstream again;
  again << fine;
  components::Setpoint back;
  again >> back;
  CHECK(back == fine);
}

void ComponentsCarryTheirData() {
  const components::WheelSetpoint setpoint(components::Setpoint{1.0, std::chrono::seconds(2), false});
  CHECK(setpoint.Data().velocity == 1.0);
  CHECK(setpoint.Data().time == std::chrono::seconds(2));
  const components::WheelMotorCurrent current(12.5);
  CHECK(current.Data() == 12.5);
  // Registered (the header's static registration in this program set the names).
  CHECK(components::WheelSetpoint::typeName != nullptr &&
        std::string(components::WheelSetpoint::typeName) == "rover_sim_components.WheelSetpoint");
  CHECK(components::WheelMotorCurrent::typeName != nullptr &&
        std::string(components::WheelMotorCurrent::typeName) == "rover_sim_components.WheelMotorCurrent");
}

}  // namespace

int main() {
  SetpointEquality();
  SetpointStreams();
  ComponentsCarryTheirData();
  if (Failures() == 0) std::cout << "test_rover_components: all checks passed\n";
  return Failures() == 0 ? 0 : 1;
}
```

In `rover_sim/plugins/tests/CMakeLists.txt` replace

```cmake
# map reader (terrain_ground.hh) and the ruts' layer and geometry (rover_tracks.hh).
# Run: ctest --test-dir <build directory> --output-on-failure
foreach(name test_rover_drivetrain test_terrain_ground test_rover_tracks)
```

with

```cmake
# map reader (terrain_ground.hh), the ruts' layer and geometry (rover_tracks.hh) and the rover's ECM components
# (rover_components.hh).
# Run: ctest --test-dir <build directory> --output-on-failure
foreach(name test_rover_drivetrain test_terrain_ground test_rover_tracks test_rover_components)
```

- [ ] **Step 2: Run it to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run sim-build 2>&1 | grep -E "rover_components.hh|Failed|Summary" | head -5
```

Expected: `fatal error: 'rover_components.hh' file not found` and `Summary: 0 packages finished [...] 1 package failed: rover_sim`.

- [ ] **Step 3: The components**

Write `rover_sim/plugins/rover_components.hh`:

```cpp
// The rover's ECM components that are not Gazebo's, one of each on every wheel joint (design spec 2026-10-09,
// section 6.1):
// - WheelSetpoint: the speed the wheel's motor is to turn at, which ros2_control's RoverSimSystem
//   (ros2_control/rover_sim_system.cpp) writes at the start of every step, or RoverDrivetrain from its ROS-free
//   setpoint topic, and which RoverDrivetrain's motor model follows. Not Gazebo's JointVelocityCmd: the Physics
//   system would enforce that as a velocity constraint and bypass the motor model.
// - WheelMotorCurrent: the motor's current, which RoverDrivetrain writes every step and RoverSimSystem reports as
//   the wheel's effort.
// Both libraries register them under the same names (gz-sim's component factory takes a component registered from
// several libraries), so they are one component type in the ECM.
#pragma once

#include <chrono>
#include <cstdint>
#include <istream>
#include <limits>
#include <ostream>

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace rover_sim::components {

/// A wheel's speed setpoint, and who wrote it when.
struct Setpoint {
  double velocity = 0.0;                       ///< [rad/s] at the wheel
  /// When it was written, on sim time: the time ros2_control gave RoverSimSystem's write() (the controller manager's
  /// clock, the bridged /clock, a few ms behind the step), or the drivetrain's when a topic message came. The
  /// watchdog's age counts from it.
  std::chrono::steady_clock::duration time{};
  bool ros2_control = false;                   ///< written by RoverSimSystem: the setpoint topic is then ignored

  bool operator==(const Setpoint& other) const {
    return velocity == other.velocity && time == other.time && ros2_control == other.ros2_control;
  }
  bool operator!=(const Setpoint& other) const { return !(*this == other); }
};

/// "velocity nanoseconds 0|1", the velocity in 17 significant digits (a double read back is the same double).
inline std::ostream& operator<<(std::ostream& out, const Setpoint& s) {
  const auto precision = out.precision(std::numeric_limits<double>::max_digits10);
  out << s.velocity << ' ' << std::chrono::duration_cast<std::chrono::nanoseconds>(s.time).count() << ' '
      << (s.ros2_control ? 1 : 0);
  out.precision(precision);
  return out;
}

inline std::istream& operator>>(std::istream& in, Setpoint& s) {
  int64_t nanoseconds = 0;
  int ros2_control = 0;
  in >> s.velocity >> nanoseconds >> ros2_control;
  s.time = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::nanoseconds(nanoseconds));
  s.ros2_control = ros2_control != 0;
  return in;
}

using WheelSetpoint = gz::sim::components::Component<Setpoint, class WheelSetpointTag>;
GZ_SIM_REGISTER_COMPONENT("rover_sim_components.WheelSetpoint", WheelSetpoint)

/// A wheel motor's current [A] at the end of the latest physics step.
using WheelMotorCurrent = gz::sim::components::Component<double, class WheelMotorCurrentTag>;
GZ_SIM_REGISTER_COMPONENT("rover_sim_components.WheelMotorCurrent", WheelMotorCurrent)

}  // namespace rover_sim::components
```

- [ ] **Step 4: Run it to see it pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run sim-build 2>&1 | tail -1
/Users/alarion239/.pixi/bin/pixi run bash -c 'ctest --test-dir ../build/rover_sim -R test_rover_components --output-on-failure' | tail -3
```

Expected: `Summary: 1 package finished [...]`, then `100% tests passed, 0 tests failed out of 1`.

- [ ] **Step 5: The drivetrain follows a wheel's setpoint component when it has one**

Edits in `rover_sim/plugins/rover_drivetrain.cpp`. Replace

```cpp
// - cmd_vel (vx, wz) -> wheel-speed setpoints (vx -+ wz track track_multiplier / 2) / radius, left -, right +,
//   through a ramp (gz::math::SpeedLimiter); a command older than <cmd_timeout> counts as zero.
```

with

```cpp
// - each wheel's speed setpoint: its joint's WheelSetpoint component (rover_components.hh) when it has one, which
//   ros2_control's RoverSimSystem (ros2_control/rover_sim_system.cpp) writes, clamped to <controller><max_speed> and
//   zero once older than <cmd_timeout> (the motor controller's watchdog); otherwise cmd_vel (vx, wz) ->
//   (vx -+ wz track track_multiplier / 2) / radius, left -, right +, through a ramp (gz::math::SpeedLimiter), a
//   command older than <cmd_timeout> counting as zero. Each motor's current goes into its joint's WheelMotorCurrent.
```

Replace

```cpp
#include "rover_drivetrain.hh"
#include "rover_tracks_render.hh"
```

with

```cpp
#include "rover_components.hh"
#include "rover_drivetrain.hh"
#include "rover_tracks_render.hh"
```

Replace

```cpp
  double setpoint = 0.0;  // [rad/s] after the ramp
```

with

```cpp
  double setpoint = 0.0;  // [rad/s] after the ramp
  std::chrono::steady_clock::duration cmd_time = std::chrono::steady_clock::duration::min();  // stamp last seen
  Clock::time_point cmd_wall{};  // when that stamp was first seen (the wall-clock watchdog)
```

Replace

```cpp
      w.joint.EnableTransmittedWrenchCheck(ecm, true);
```

with

```cpp
      w.joint.EnableTransmittedWrenchCheck(ecm, true);
      ecm.CreateComponent(w.joint.Entity(), components::WheelMotorCurrent(0.0));
```

Replace

```cpp
      double target = (vx - w.side * wz * track_ / 2) / radius_;
      w.ramp.LimitVelocity(target);
      w.ramp.LimitAcceleration(target, w.setpoint, info.dt);
      w.setpoint = target;
      w.joint.SetForce(ecm, {w.motor.Step(motor_, w.setpoint, w.speed, dt)});
```

with

```cpp
      if (ecm.Component<components::WheelSetpoint>(w.joint.Entity())) {
        w.setpoint = Setpoint(w, ecm, t);
      } else {
        double target = (vx - w.side * wz * track_ / 2) / radius_;
        w.ramp.LimitVelocity(target);
        w.ramp.LimitAcceleration(target, w.setpoint, info.dt);
        w.setpoint = target;
      }
      w.joint.SetForce(ecm, {w.motor.Step(motor_, w.setpoint, w.speed, dt)});
      if (auto* current = ecm.Component<components::WheelMotorCurrent>(w.joint.Entity())) {
        current->Data() = w.motor.current;
      }
```

Replace

```cpp
    for (auto& w : wheels_) {
      w.setpoint = w.spin = 0.0;
      w.motor.Reset();
```

with

```cpp
    for (auto& w : wheels_) {
      w.setpoint = w.spin = 0.0;
      w.cmd_time = std::chrono::steady_clock::duration::min();
      w.motor.Reset();
```

Replace

```cpp
  void OnCmd(const gz::msgs::Twist& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_ = {msg.linear().x(), msg.angular().z()};
    cmd_sim_ = sim_time_;
    cmd_wall_ = Clock::now();
  }
```

with

```cpp
  void OnCmd(const gz::msgs::Twist& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_ = {msg.linear().x(), msg.angular().z()};
    cmd_sim_ = sim_time_;
    cmd_wall_ = Clock::now();
  }

  /// The setpoint a wheel's motor follows this step, from its WheelSetpoint component: zero once it is older than
  /// cmd_timeout (0: never) on the plugin's clock (sim: the stamp's sim time; wall: when this plugin first saw that
  /// stamp), else clamped to max_speed, the motor controller's velocity limit; zero without a component.
  double Setpoint(Wheel& w, const gz::sim::EntityComponentManager& ecm, double t) const {
    const auto* cmd = ecm.Component<components::WheelSetpoint>(w.joint.Entity());
    if (!cmd) return 0.0;
    const components::Setpoint& s = cmd->Data();
    if (s.time != w.cmd_time) {
      w.cmd_time = s.time;
      w.cmd_wall = Clock::now();
    }
    const double age = wall_timeout_ ? Seconds(Clock::now() - w.cmd_wall) : t - Seconds(s.time);
    if (cmd_timeout_ > 0 && age > cmd_timeout_) return 0.0;
    return drive::Clamp(s.velocity, -max_speed_, max_speed_);
  }
```

- [ ] **Step 6: Build; the ROS-free runs are unchanged**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run sim-build 2>&1 | grep -E "warning|error|Summary"
cd rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_equivalence.Drives test_drivetrain.Response test_rover_sim 2>&1 | tail -3
```

Expected: no warning, `Summary: 1 package finished`; then `Ran 17 tests in ...s` (Drives 1, Response 3, test_rover_sim 13) and `OK`. No wheel has a `WheelSetpoint` without RoverSimSystem, so every run takes the twist path as before.

- [ ] **Step 7: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/plugins && git commit -q -F - <<'EOF'
PR 3: rover_sim: the wheel setpoint component, and the drivetrain follows it beside its twist input

plugins/rover_components.hh: WheelSetpoint (a wheel's speed setpoint, the sim time it was written, and whether
ros2_control wrote it) and WheelMotorCurrent, ECM components on the wheel joints that RoverSimSystem and
RoverDrivetrain share, registered by both libraries under one name. RoverDrivetrain follows a wheel's WheelSetpoint
when it has one (clamped to the top speed, zero once older than cmd_timeout: the motor controller's watchdog) and its
twist input otherwise, and writes each motor's current; nothing but RoverSimSystem creates the setpoint yet, so
every ROS-free run is unchanged. test_rover_components checks the components' equality and streams.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 4: RoverSimSystem, the simulated rover's ros2_control hardware

**Files:** Create `rover_sim/ros2_control/rover_sim_system.hpp`, `rover_sim/ros2_control/rover_sim_system.cpp`, `rover_sim/rover_sim_system.xml`, `rover_sim/ros2_control/test/test_rover_sim_system.cpp`. Modify `rover_sim/CMakeLists.txt`, `rover_sim/package.xml`.

The plugin lives in `ros2_control/`, not `plugins/`: CMake makes every `plugins/*.cpp` a Gazebo system plugin.

- [ ] **Step 1: The failing test**

Write `rover_sim/ros2_control/test/test_rover_sim_system.cpp`:

```cpp
// RoverSimSystem (rover_sim_system.cpp) without Gazebo running: an ECM with the rover's eight joints, the hardware
// info of rover_description's ros2_control block with the simulation's plugin, and the interfaces the controller
// manager would claim (initSim, then init and export, as gz_ros2_control's resource manager calls them).
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_components.hh"
#include "rover_sim_system.hpp"

namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace components = rover_sim::components;

const std::vector<std::string> kWheels = {"wheel_fl_joint", "wheel_rl_joint", "wheel_fr_joint", "wheel_rr_joint"};
const std::vector<std::string> kSensed = {"rocker_left_joint", "rocker_right_joint", "camera_pan_joint",
                                          "camera_tilt_joint"};

hardware_interface::InterfaceInfo Interface(const std::string& name) {
  hardware_interface::InterfaceInfo info{};
  info.name = name;
  info.size = 1;
  info.enable_limits = false;
  return info;
}

/// rover_description's ros2_control block (urdf/ros2_control.xacro) with the simulation's plugin.
hardware_interface::HardwareInfo RoverInfo() {
  hardware_interface::HardwareInfo info{};
  info.name = "rover";
  info.type = "system";
  info.hardware_plugin_name = "rover_sim/RoverSimSystem";
  for (const auto& name : kWheels) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.command_interfaces = {Interface("velocity")};
    joint.state_interfaces = {Interface("position"), Interface("velocity"), Interface("effort")};
    info.joints.push_back(joint);
  }
  for (const auto& name : kSensed) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.state_interfaces = {Interface("position"), Interface("velocity")};
    info.joints.push_back(joint);
  }
  return info;
}

class RoverSimSystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    for (const auto& name : kWheels) joints_[name] = Joint(name);
    for (const auto& name : kSensed) joints_[name] = Joint(name);
  }

  gz::sim::Entity Joint(const std::string& name) {
    const auto entity = ecm_.CreateEntity();
    ecm_.CreateComponent(entity, gz::sim::components::Joint());
    ecm_.CreateComponent(entity, gz::sim::components::Name(name));
    return entity;
  }

  /// What gz_ros2_control's resource manager does: initSim, then init and the interfaces' export.
  void Load() {
    rclcpp::Node::SharedPtr node;  // RoverSimSystem keeps it and never uses it
    ASSERT_TRUE(system_.initSim(node, joints_, RoverInfo(), ecm_, 100));
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = RoverInfo();
    params.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    params.logger = rclcpp::get_logger("test_rover_sim_system");
    ASSERT_EQ(system_.init(params), CallbackReturn::SUCCESS);
    for (const auto& state : system_.on_export_state_interfaces()) states_[state->get_name()] = state;
    for (const auto& command : system_.on_export_command_interfaces()) commands_[command->get_name()] = command;
  }

  double State(const std::string& name) { return states_.at(name)->get_optional<double>().value(); }

  void Command(const std::string& name, double value) { ASSERT_TRUE(commands_.at(name)->set_value(value, true)); }

  const components::Setpoint& Setpoint(const std::string& joint) {
    return ecm_.Component<components::WheelSetpoint>(joints_.at(joint))->Data();
  }

  gz::sim::EntityComponentManager ecm_;
  std::map<std::string, gz::sim::Entity> joints_;
  rover_sim::RoverSimSystem system_;
  std::map<std::string, hardware_interface::StateInterface::ConstSharedPtr> states_;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> commands_;
};

TEST_F(RoverSimSystemTest, ExportsTheBlocksInterfaces) {
  Load();
  EXPECT_EQ(states_.size(), 4u * 3u + 4u * 2u);
  EXPECT_EQ(commands_.size(), 4u);
  for (const auto& name : kWheels) {
    EXPECT_EQ(commands_.count(name + "/velocity"), 1u) << name;
    EXPECT_EQ(states_.count(name + "/effort"), 1u) << name;
  }
  for (const auto& name : kSensed) EXPECT_EQ(states_.count(name + "/position"), 1u) << name;
}

TEST_F(RoverSimSystemTest, MarksTheWheelsAsRos2ControlsAndGivesEveryJointItsState) {
  Load();
  for (const auto& name : kWheels) {
    EXPECT_TRUE(Setpoint(name).ros2_control) << name;
    EXPECT_EQ(Setpoint(name).velocity, 0.0) << name;
  }
  for (const auto& [name, entity] : joints_) {
    EXPECT_NE(ecm_.Component<gz::sim::components::JointPosition>(entity), nullptr) << name;
    EXPECT_NE(ecm_.Component<gz::sim::components::JointVelocity>(entity), nullptr) << name;
  }
  for (const auto& name : kSensed) {
    EXPECT_EQ(ecm_.Component<components::WheelSetpoint>(joints_.at(name)), nullptr) << name;
  }
}

TEST_F(RoverSimSystemTest, WriteHandsEachWheelItsCommandStampedWithTheTimeItIsGiven) {
  Load();
  Command("wheel_fl_joint/velocity", 2.5);
  Command("wheel_rl_joint/velocity", 2.5);
  Command("wheel_fr_joint/velocity", -1.25);
  // wheel_rr_joint's command is not set (NaN, as before a controller writes one): zero.
  const rclcpp::Time step(12, 345000000, RCL_ROS_TIME);
  ASSERT_EQ(system_.write(step, rclcpp::Duration::from_seconds(0.001)), return_type::OK);
  EXPECT_EQ(Setpoint("wheel_fl_joint").velocity, 2.5);
  EXPECT_EQ(Setpoint("wheel_rl_joint").velocity, 2.5);
  EXPECT_EQ(Setpoint("wheel_fr_joint").velocity, -1.25);
  EXPECT_EQ(Setpoint("wheel_rr_joint").velocity, 0.0);
  for (const auto& name : kWheels) {
    EXPECT_EQ(Setpoint(name).time, std::chrono::nanoseconds(12345000000)) << name;
    EXPECT_TRUE(Setpoint(name).ros2_control) << name;
  }
}

TEST_F(RoverSimSystemTest, ReadReportsTheJointsAndTheMotorCurrent) {
  Load();
  // Before the drivetrain's first step there is no current: effort 0.
  ASSERT_EQ(system_.read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  EXPECT_EQ(State("wheel_fl_joint/effort"), 0.0);
  ecm_.SetComponentData<gz::sim::components::JointPosition>(joints_.at("wheel_fl_joint"), {12.5});
  ecm_.SetComponentData<gz::sim::components::JointVelocity>(joints_.at("wheel_fl_joint"), {3.0});
  ecm_.CreateComponent(joints_.at("wheel_fl_joint"), components::WheelMotorCurrent(7.25));
  ecm_.SetComponentData<gz::sim::components::JointPosition>(joints_.at("rocker_left_joint"), {-0.125});
  ecm_.SetComponentData<gz::sim::components::JointVelocity>(joints_.at("camera_pan_joint"), {0.5});
  ASSERT_EQ(system_.read(rclcpp::Time(0, 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
            return_type::OK);
  EXPECT_EQ(State("wheel_fl_joint/position"), 12.5);
  EXPECT_EQ(State("wheel_fl_joint/velocity"), 3.0);
  EXPECT_EQ(State("wheel_fl_joint/effort"), 7.25);
  EXPECT_EQ(State("rocker_left_joint/position"), -0.125);
  EXPECT_EQ(State("camera_pan_joint/velocity"), 0.5);
  EXPECT_EQ(State("wheel_rr_joint/position"), 0.0);  // a joint component without a value yet
}

TEST_F(RoverSimSystemTest, InitSimRefusesAJointTheModelLacks) {
  joints_.erase("wheel_rr_joint");
  rclcpp::Node::SharedPtr node;
  EXPECT_FALSE(system_.initSim(node, joints_, RoverInfo(), ecm_, 100));
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
```

In `rover_sim/CMakeLists.txt` replace

```cmake
# C++ unit tests of the plugins' parts that need no Gazebo running:
# pixi run colcon-test, or ctest --test-dir <build directory>.
enable_testing()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/tests/CMakeLists.txt")
  add_subdirectory(plugins/tests)
endif()
```

with

```cmake
# RoverSimSystem: the simulated rover's ros2_control hardware (ros2_control/), which gz_ros2_control loads inside
# Gazebo through pluginlib (rover_sim_system.xml): each wheel's velocity command into the drivetrain's motor model
# (plugins/rover_components.hh), the joints' state and the motor currents back. It takes only gz_ros2_control's
# interface header: gz_ros2_control's library is the Gazebo system that loads this one.
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
set(gz_ros2_control_include "${gz_ros2_control_DIR}/../../../include/gz_ros2_control")
if(TARGET gz_ros2_control::gz_ros2_control-system)
  get_target_property(gz_ros2_control_include gz_ros2_control::gz_ros2_control-system INTERFACE_INCLUDE_DIRECTORIES)
endif()
add_library(rover_sim_system SHARED ros2_control/rover_sim_system.cpp)
target_include_directories(rover_sim_system PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ros2_control"
                           "${CMAKE_CURRENT_SOURCE_DIR}/plugins" ${gz_ros2_control_include})
target_link_libraries(rover_sim_system PUBLIC gz-sim8::gz-sim8 hardware_interface::hardware_interface
                      pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)
target_compile_options(rover_sim_system PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS rover_sim_system LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(gz_ros2_control rover_sim_system.xml)

# C++ unit tests of the plugins' parts that need no Gazebo running, and of RoverSimSystem against a bare ECM:
# pixi run colcon-test, or ctest --test-dir <build directory>.
enable_testing()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/tests/CMakeLists.txt")
  add_subdirectory(plugins/tests)
endif()
find_package(GTest REQUIRED)
add_executable(test_rover_sim_system ros2_control/test/test_rover_sim_system.cpp)
target_link_libraries(test_rover_sim_system PRIVATE rover_sim_system GTest::gtest)
target_compile_options(test_rover_sim_system PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME test_rover_sim_system COMMAND test_rover_sim_system)
```

In `rover_sim/package.xml` replace

```xml
  <depend>gz_common_vendor</depend>
```

with

```xml
  <depend>gz_common_vendor</depend>
  <!-- RoverSimSystem: the ros2_control hardware gz_ros2_control loads inside Gazebo -->
  <depend>gz_ros2_control</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <test_depend>gtest</test_depend>
```

- [ ] **Step 2: Run it to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run sim-build 2>&1 | grep -E "rover_sim_system|Summary" | head -4
```

Expected: CMake stops with `Cannot find source file: ros2_control/rover_sim_system.cpp` and `1 package failed: rover_sim`.

- [ ] **Step 3: RoverSimSystem**

Write `rover_sim/ros2_control/rover_sim_system.hpp`:

```cpp
// RoverSimSystem: the simulated rover's ros2_control hardware (design spec 2026-10-09, section 6.1). gz_ros2_control
// loads it inside Gazebo (pluginlib: rover_sim/RoverSimSystem, rover_sim_system.xml) for the ros2_control block of
// rover_sim.urdf.xacro with ros2_control:=true; the controller manager it runs calls write() at the start of every
// physics step (gz_ros2_control's PreUpdate) and read() and the controllers' update at the end of every 10th step
// (100 Hz, its PostUpdate).
// - write(): each wheel's velocity command [rad/s] into its joint's WheelSetpoint (plugins/rover_components.hh),
//   stamped with the time write() is given: ros2_control 4.47 passes the controller manager's clock, sim time from
//   the bridged /clock, a few ms behind the step (the drivetrain's watchdog counts from it, so the motors stop when
//   /clock stops). RoverDrivetrain's motor model follows the setpoint in the same step (its plugin comes after
//   gz_ros2_control's in the model; tests/test_ros2_control.py checks it). Not Gazebo's JointVelocityCmd: the
//   Physics system would enforce that as a velocity constraint and bypass the motors. A command not yet set (NaN,
//   until a controller writes one) is 0.
// - read(): every joint's position and velocity from Gazebo, and each wheel's motor current [A] as its effort
//   (RoverDrivetrain's WheelMotorCurrent; 0 until the drivetrain's first step).
// The interfaces are the block's (rover_description/urdf/ros2_control.xacro): wheels velocity command and position,
// velocity and effort state; rockers and camera head position and velocity.
#pragma once

#include <map>
#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>

namespace rover_sim {

class RoverSimSystem : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  using hardware_interface::SystemInterface::on_init;

  bool initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& joints,
               const hardware_interface::HardwareInfo& hardware_info, gz::sim::EntityComponentManager& ecm,
               unsigned int update_rate) override;
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  struct Joint {
    std::string name;
    gz::sim::Entity entity = gz::sim::kNullEntity;
    bool driven = false;  // a velocity command interface: a wheel
    std::string position, velocity, effort, command;  // the interfaces' full names
  };
  std::vector<Joint> joints_;
  gz::sim::EntityComponentManager* ecm_ = nullptr;
};

}  // namespace rover_sim
```

Write `rover_sim/ros2_control/rover_sim_system.cpp`:

```cpp
// rover_components.hh first: its component registration names gz::sim as `sim`, which gz_system_interface.hpp also
// declares as an alias at global scope.
#include "rover_components.hh"
#include "rover_sim_system.hpp"

#include <chrono>
#include <cmath>

#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rover_sim {

namespace {

rclcpp::Logger Logger() { return rclcpp::get_logger("RoverSimSystem"); }

double First(const std::vector<double>* values) { return values && !values->empty() ? (*values)[0] : 0.0; }

}  // namespace

bool RoverSimSystem::initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& joints,
                             const hardware_interface::HardwareInfo& hardware_info,
                             gz::sim::EntityComponentManager& ecm, unsigned int /*update_rate*/) {
  nh_ = model_nh;
  ecm_ = &ecm;
  joints_.clear();
  for (const auto& info : hardware_info.joints) {
    const auto found = joints.find(info.name);
    if (found == joints.end()) {
      RCLCPP_FATAL(Logger(), "joint '%s' of the ros2_control block is not a joint of the Gazebo model",
                   info.name.c_str());
      return false;
    }
    Joint joint;
    joint.name = info.name;
    joint.entity = found->second;
    joint.position = info.name + "/" + hardware_interface::HW_IF_POSITION;
    joint.velocity = info.name + "/" + hardware_interface::HW_IF_VELOCITY;
    joint.effort = info.name + "/" + hardware_interface::HW_IF_EFFORT;
    joint.command = info.name + "/" + hardware_interface::HW_IF_VELOCITY;
    for (const auto& command : info.command_interfaces) {
      if (command.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(Logger(), "joint '%s': RoverSimSystem takes velocity commands only, not '%s'",
                     info.name.c_str(), command.name.c_str());
        return false;
      }
      joint.driven = true;
    }
    if (!ecm.Component<gz::sim::components::JointPosition>(joint.entity)) {
      ecm.CreateComponent(joint.entity, gz::sim::components::JointPosition());
    }
    if (!ecm.Component<gz::sim::components::JointVelocity>(joint.entity)) {
      ecm.CreateComponent(joint.entity, gz::sim::components::JointVelocity());
    }
    if (joint.driven) {
      // The wheel is ros2_control's from now on (RoverDrivetrain ignores its setpoint topic); 0 until write().
      components::Setpoint setpoint;
      setpoint.ros2_control = true;
      ecm.CreateComponent(joint.entity, components::WheelSetpoint(setpoint));
    }
    joints_.push_back(joint);
  }
  return true;
}

hardware_interface::CallbackReturn RoverSimSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != joints_.size()) {
    RCLCPP_FATAL(get_logger(), "%zu joints in the ros2_control block, %zu found by initSim", info_.joints.size(),
                 joints_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  size_t wheels = 0;
  for (const auto& joint : joints_) wheels += joint.driven ? 1 : 0;
  RCLCPP_INFO(get_logger(), "RoverSimSystem: %zu joints, %zu wheels driven through RoverDrivetrain's motor model",
              joints_.size(), wheels);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverSimSystem::read(const rclcpp::Time& /*time*/,
                                                     const rclcpp::Duration& /*period*/) {
  for (const auto& joint : joints_) {
    const auto* position = ecm_->Component<gz::sim::components::JointPosition>(joint.entity);
    const auto* velocity = ecm_->Component<gz::sim::components::JointVelocity>(joint.entity);
    set_state(joint.position, First(position ? &position->Data() : nullptr));
    set_state(joint.velocity, First(velocity ? &velocity->Data() : nullptr));
    if (joint.driven) {
      const auto* current = ecm_->Component<components::WheelMotorCurrent>(joint.entity);
      set_state(joint.effort, current ? current->Data() : 0.0);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverSimSystem::write(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  const auto stamp = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::nanoseconds(time.nanoseconds()));
  for (const auto& joint : joints_) {
    if (!joint.driven) continue;
    auto* setpoint = ecm_->Component<components::WheelSetpoint>(joint.entity);
    if (!setpoint) continue;  // created by initSim; a world reset may have removed it
    const double command = get_command<double>(joint.command);
    setpoint->Data() = components::Setpoint{std::isfinite(command) ? command : 0.0, stamp, true};
  }
  return hardware_interface::return_type::OK;
}

}  // namespace rover_sim

PLUGINLIB_EXPORT_CLASS(rover_sim::RoverSimSystem, gz_ros2_control::GazeboSimSystemInterface)
```

Write `rover_sim/rover_sim_system.xml`:

```xml
<library path="rover_sim_system">
  <class name="rover_sim/RoverSimSystem" type="rover_sim::RoverSimSystem"
         base_class_type="gz_ros2_control::GazeboSimSystemInterface">
    <description>
      The simulated rover's ros2_control hardware: hands each wheel's velocity command to the RoverDrivetrain
      plugin's motor model and reports the joints' state and the motor currents (rover_sim/ros2_control).
    </description>
  </class>
</library>
```

- [ ] **Step 4: Run it to see it pass; pluginlib can find it**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run sim-build 2>&1 | grep -E "warning|error|Summary"
/Users/alarion239/.pixi/bin/pixi run bash -c 'ctest --test-dir ../build/rover_sim -R test_rover_sim_system --output-on-failure' | tail -3
cd /Users/alarion239/Desktop/hurc_ws/install/rover_sim && cat share/ament_index/resource_index/gz_ros2_control__pluginlib__plugin/rover_sim; echo; ls lib/librover_sim_system.*
```

Expected: no warning, `Summary: 1 package finished`; `100% tests passed, 0 tests failed out of 1`; `share/rover_sim/rover_sim_system.xml`; `lib/librover_sim_system.dylib`. The plan critic built this step's four files as written, against an environment locked as Task 2 locks it (gz_ros2_control 1.2.19, hardware_interface 4.47.0), with `-Wall -Wextra -Wpedantic`: no warning, the five test cases pass, and gz_ros2_control loaded the library from an install prefix on `AMENT_PREFIX_PATH` inside `gz sim -s`. A new binary's first run under ctest took 70 s on this Mac while macOS checked it; a second run takes 0.3 s. A warning that `on_init` hides an overload means the `using` line is missing; an include error for `gz_ros2_control/gz_system_interface.hpp` means the include directory came out wrong: print `gz_ros2_control_include` with a `message(STATUS ...)` and point it at the folder that holds `gz_ros2_control/gz_system_interface.hpp` (Task 2 printed it), then record the fix in Task 6's results.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/ros2_control rover_sim/rover_sim_system.xml rover_sim/CMakeLists.txt rover_sim/package.xml && git commit -q -F - <<'EOF'
PR 3: rover_sim: RoverSimSystem, the simulated rover's ros2_control hardware

A gz_ros2_control::GazeboSimSystemInterface plugin (pluginlib: rover_sim/RoverSimSystem) for rover_description's
ros2_control block: write() hands each wheel's velocity command to the drivetrain's motor model through the wheel
joint's WheelSetpoint, stamped with the time write() is given (ros2_control 4.47: the controller manager's clock, sim
time from the bridged /clock) (not Gazebo's JointVelocityCmd, which the Physics system would enforce as a velocity
constraint); read() reports every joint's position and velocity and each wheel's motor current as its effort. In
ros2_control/, not plugins/, whose files CMake builds as Gazebo systems. Its GoogleTest runs it against a bare ECM:
the interfaces, the wheels marked as ros2_control's, the commands stamped with the time given (NaN as 0), the state
read back, a joint the model lacks refused.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 5: The rover with ros2_control, its controllers, and the environment that finds them

**Files:** Create `rover_control/config/controllers.yaml`. Modify `rover_sim/urdf/rover_sim.urdf.xacro`, `rover_sim/tools/gen_rover.py`, `rover_sim/rover_config.py`, `rover_sim/gzenv.py`, `rover_sim/tests/simulate.py`, `rover_sim/tests/test_rover_model.py`, `rover_sim/tests/test_foundations.py`.

- [ ] **Step 1: The failing tests**

In `rover_sim/tests/test_rover_model.py` replace

```python
import re
import struct
import tempfile
```

with

```python
import re
import shutil
import struct
import subprocess
import tempfile
```

and append before the final `if __name__ == "__main__":`:

```python
class Ros2ControlVariant(unittest.TestCase):
    """build_urdf and build_sdf with ros2_control: the rover sim.launch.py runs (design spec 2026-10-09, 6.1 and 6.3):
    rover_description's ros2_control block with the hardware interface RoverSimSystem, and the gz_ros2_control
    system with rover_control's controllers.yaml just before the drivetrain, so that its PreUpdate, where the
    controller manager writes the setpoints, comes first in each step (tests/test_ros2_control.py)."""

    @classmethod
    def setUpClass(cls):
        cls.urdf = gen_rover.build_urdf(V, ros2_control=True)
        cls.sdf = gen_rover.build_sdf(V, ros2_control=True)
        cls.model = ET.fromstring(cls.sdf).find("model")

    def test_the_block_names_rover_sim_system(self):
        block = ET.fromstring(self.urdf).find("ros2_control")
        self.assertEqual(block.findtext("hardware/plugin"), "rover_sim/RoverSimSystem")
        commanded = [j.get("name") for j in block.findall("joint") if j.find("command_interface") is not None]
        self.assertEqual(commanded, [f"{wheel}_joint" for wheel in WHEELS])
        self.assertEqual(len(block.findall("joint")), 8)

    def test_gz_ros2_control_runs_just_before_the_drivetrain(self):
        plugins = self.model.findall("plugin")
        names = [p.get("name") for p in plugins]
        here = names.index("gz_ros2_control::GazeboSimROS2ControlPlugin")
        self.assertEqual(names[here + 1], "rover_sim::RoverDrivetrain")
        self.assertEqual(plugins[here].get("filename"), "gz_ros2_control-system")
        self.assertEqual(plugins[here].findtext("parameters"), str(rover_config.CONTROLLERS_YAML))
        self.assertTrue(rover_config.CONTROLLERS_YAML.is_file())

    def test_without_it_the_rover_has_neither_and_is_the_same(self):
        default = ET.fromstring(gen_rover.build_sdf(V)).find("model")
        self.assertNotIn("gz_ros2_control::GazeboSimROS2ControlPlugin", [p.get("name") for p in default.iter("plugin")])
        self.assertIsNone(ET.fromstring(gen_rover.build_urdf(V)).find("ros2_control"))
        ros = copy.deepcopy(self.model)
        ros.remove(ros.find("plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"))
        self.assertEqual(canonical(ros), canonical(default))

    def test_check_urdf_accepts_it(self):
        if shutil.which("check_urdf") is None:
            self.skipTest("check_urdf is not installed (liburdfdom-tools)")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "rover.urdf"
            path.write_text(self.urdf)
            result = subprocess.run(["check_urdf", str(path)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Successfully Parsed XML", result.stdout)

    def test_diffdrive_cannot_have_it(self):
        with self.assertRaises(Exception) as caught:
            gen_rover.build_urdf(DIFFDRIVE, ros2_control=True)
        self.assertIn("drivetrain mode physical", str(caught.exception))

    def test_gz_accepts_it(self):
        with temp_sdf(self.sdf) as path:
            result = gz_check(path)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
```

In `rover_sim/tests/test_foundations.py` replace

```python
        self.assertEqual(env["GZ_SIM_SYSTEM_PLUGIN_PATH"], str(SIM_DIR.parents[1] / "build" / "rover_sim"))
```

with

```python
        self.assertEqual(env["GZ_SIM_SYSTEM_PLUGIN_PATH"],
                         os.pathsep.join([str(SIM_DIR.parents[1] / "build" / "rover_sim"), "/env/lib"]))
```

replace

```python
            self.assertEqual(env["GZ_SIM_SYSTEM_PLUGIN_PATH"], str(Path(build).resolve()))
```

with

```python
            self.assertEqual(env["GZ_SIM_SYSTEM_PLUGIN_PATH"],
                             os.pathsep.join([str(Path(build).resolve()), "/env/lib"]))
```

and replace

```python
    def test_partition_and_ip_when_asked(self):
        env = gzenv.environment(base=self.BASE, partition="p", ip="127.0.0.1")
        self.assertEqual((env["GZ_PARTITION"], env["GZ_IP"]), ("p", "127.0.0.1"))
```

with

```python
    def test_partition_and_ip_when_asked(self):
        env = gzenv.environment(base=self.BASE, partition="p", ip="127.0.0.1")
        self.assertEqual((env["GZ_PARTITION"], env["GZ_IP"]), ("p", "127.0.0.1"))

    def test_gz_ros2_control_and_rover_sim_system_are_found(self):
        """gz_ros2_control's system plugin is in the environment's lib, which Gazebo does not search: last on the
        plugin path. RoverSimSystem is in the workspace's install/rover_sim, which pluginlib finds through
        AMENT_PREFIX_PATH: first, once colcon has installed it."""
        with tempfile.TemporaryDirectory() as install:
            env = gzenv.environment(base=dict(self.BASE, AMENT_PREFIX_PATH="/env"), install_dir=install)
            self.assertEqual(env["AMENT_PREFIX_PATH"], os.pathsep.join([str(Path(install).resolve()), "/env"]))
            twice = gzenv.environment(base=env, install_dir=install)
            self.assertEqual(twice["AMENT_PREFIX_PATH"], env["AMENT_PREFIX_PATH"])
            self.assertEqual(twice["GZ_SIM_SYSTEM_PLUGIN_PATH"].split(os.pathsep)[-1], "/env/lib")
            self.assertEqual(twice["GZ_SIM_SYSTEM_PLUGIN_PATH"].count("/env/lib"), 1)
        self.assertNotIn("AMENT_PREFIX_PATH", gzenv.environment(base=self.BASE, install_dir="/no/such/install"))
        self.assertEqual(gzenv.INSTALL_DIR, SIM_DIR.parents[1] / "install" / "rover_sim")

    def test_a_private_ros_domain_when_asked(self):
        env = gzenv.environment(base=self.BASE, ros_domain=63)
        self.assertEqual((env["ROS_DOMAIN_ID"], env["ROS_AUTOMATIC_DISCOVERY_RANGE"]), ("63", "LOCALHOST"))
        self.assertNotIn("ROS_DOMAIN_ID", gzenv.environment(base=self.BASE))
        self.assertEqual({gzenv.private_ros_domain(seed) for seed in range(1000)}, set(range(50, 100)))
```

- [ ] **Step 2: Run them to see them fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_rover_model.Ros2ControlVariant test_foundations.Environment 2>&1 | tail -3
```

Expected: `FAILED (errors=...)`: `build_urdf() got an unexpected keyword argument 'ros2_control'` and gzenv's missing `install_dir`, `INSTALL_DIR` and `private_ros_domain`.

- [ ] **Step 3: The controllers**

Write `rover_control/config/controllers.yaml`:

```yaml
# The rover's controllers, the same on the real and the simulated rover (design spec 2026-10-09, section 6): the
# controller manager's settings and the two controllers it runs, joint_state_broadcaster (/joint_states: wheels,
# rockers, camera head) and drive_controller, which takes /cmd_vel (geometry_msgs/TwistStamped) and publishes /odom
# and odom -> base_link. launch/controllers.launch.py starts both (another drive controller: controller:=<name>):
# rover.launch.py on the rover, with its hardware stub until the motor controllers are chosen, and rover_sim's
# sim.launch.py in the simulation, where gz_ros2_control runs the controller manager inside Gazebo and RoverSimSystem
# hands the wheel commands to the motor model.
#
# drive_controller is diff_drive_controller (skid steer: left = FL + RL, right = FR + RR). To try the team's own,
# change its type, or add the new controller beside it and start it with controller:=<name>. Its geometry and limits
# are rover_description/config/rover.yaml's, written out here (a YAML file cannot include another);
# test/test_controllers_yaml.cpp checks them against rover.yaml, so change both together.
# Units SI. Sources: (A) assumed, (M) measured or tuned in the simulation, (R) from a reference, as in rover.yaml.

controller_manager:
  ros__parameters:
    update_rate: 100                    # [Hz] the controllers' update, on the rover and in the simulation (A)
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    drive_controller:
      type: diff_drive_controller/DiffDriveController

drive_controller:
  ros__parameters:
    left_wheel_names: [wheel_fl_joint, wheel_rl_joint]
    right_wheel_names: [wheel_fr_joint, wheel_rr_joint]
    wheel_separation: 0.8               # [m] 2 x rover.yaml rocker.pivot_y
    wheel_radius: 0.15                  # [m] rover.yaml wheel.radius
    # The effective-track compensation of skid steer: the controller's kinematics and odometry use 0.8 m x this. 1 is
    # the raw response (realism design D24, Q10; the user's choice, 2026-10-06): odometry over-reports a turn in
    # place by 1 / the ground's turn ratio, 2.65 on packed ground.
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0
    odom_frame_id: odom
    base_frame_id: base_link
    publish_rate: 50.0                  # [Hz] /odom and odom -> base_link
    enable_odom_tf: true
    open_loop: false                    # odometry from the wheels' positions
    position_feedback: true
    # A command older than this counts as zero, so a commander that died cannot leave the rover driving (user
    # decision 2026-10-06). The station sends 20 Hz; a zero-stamped command is stamped on arrival (docs/topics.md).
    cmd_vel_timeout: 0.5                # [s]
    # The top speed, a wheel at rover.yaml wheel.top_speed, 20 rad/s: 3 m/s straight, 7.5 rad/s turning in place
    # (user decision 2026-10-07).
    linear.x.max_velocity: 3.0          # [m/s] top_speed x radius
    linear.x.min_velocity: -3.0
    angular.z.max_velocity: 7.5         # [rad/s] top_speed x radius / pivot_y
    angular.z.min_velocity: -7.5
    # The ramp: a wheel's speed changes by at most 8 rad/s^2 driving straight or turning in place, 1.2 m/s^2 (A: until
    # the driver team reports theirs, Q7); the simulation's ROS-free helpers ramp each wheel at the same rate
    # (rover_sim/config/sim.yaml wheel_cmd accel).
    linear.x.max_acceleration: 1.2      # [m/s^2] 8 rad/s^2 x radius
    linear.x.max_deceleration: -1.2
    angular.z.max_acceleration: 3.0     # [rad/s^2] 8 rad/s^2 x radius / pivot_y
    angular.z.max_deceleration: -3.0
```

- [ ] **Step 4: The wrapper's ros2_control option**

In `rover_sim/urdf/rover_sim.urdf.xacro` replace

```xml
     ruts, tread tyre), the opt-in dust and tyre compliance. tools/gen_rover.py expands it, converts it with
     `gz sdf -p` and writes models/rover/model.sdf (`pixi run sim-model`). gz_ros2_control comes with PR 3.
```

with

```xml
     ruts, tread tyre), the opt-in dust and tyre compliance. tools/gen_rover.py expands it, converts it with
     `gz sdf -p` and writes models/rover/model.sdf (`pixi run sim-model`). With ros2_control:=true it is the rover
     sim.launch.py runs: the description's ros2_control block with the hardware interface RoverSimSystem, and the
     gz_ros2_control system that runs the controller manager in Gazebo (gen_rover.build_urdf(..., ros2_control=True)).
```

replace

```xml
       ros2_control     the description's ros2_control block: false until PR 3 brings gz_ros2_control
```

with

```xml
       ros2_control     true: the description's ros2_control block and the gz_ros2_control system (default false:
                        the ROS-free rover, the worlds' model://rover and the tests')
       ros2_control_plugin  the block's hardware interface (default rover_sim/RoverSimSystem)
       controllers      with ros2_control: the controller manager's parameters, rover_control's
                        config/controllers.yaml (an absolute path)
```

replace

```xml
  <!-- PR 3 adds gz_ros2_control; until then the simulated rover has no ros2_control block. -->
  <xacro:arg name="ros2_control" default="false"/>
```

with

```xml
  <!-- Declared before the description declares them: the first declaration's default wins. -->
  <xacro:arg name="ros2_control" default="false"/>
  <xacro:arg name="ros2_control_plugin" default="rover_sim/RoverSimSystem"/>
  <xacro:arg name="controllers" default=""/>
```

and replace

```xml
  <xacro:if value="${physical}">
    <gazebo>
      <plugin filename="RoverDrivetrain" name="rover_sim::RoverDrivetrain">
```

with

```xml
  <!-- ros2_control (sim.launch.py's rover): gz_ros2_control runs the controller manager inside Gazebo with
       `controllers`, and the controller manager runs the description's ros2_control block through RoverSimSystem
       (ros2_control/rover_sim_system.cpp), which hands each wheel's velocity command to the drivetrain's motors.
       Before the drivetrain's plugin: Gazebo runs the plugins' PreUpdates in this order, so a setpoint the controller
       manager writes at the start of a step drives the motors in that step (tests/test_ros2_control.py). -->
  <xacro:if value="$(arg ros2_control)">
    <xacro:unless value="${physical}">
      ${xacro.fatal("ros2_control:=true needs sim.yaml drivetrain mode physical (RoverDrivetrain's motors)")}
    </xacro:unless>
    <xacro:if value="${xacro.arg('controllers') == ''}">
      ${xacro.fatal("ros2_control:=true needs controllers:= rover_control's controllers.yaml (absolute)")}
    </xacro:if>
    <gazebo>
      <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <parameters>$(arg controllers)</parameters>
      </plugin>
    </gazebo>
  </xacro:if>
  <xacro:if value="${physical}">
    <gazebo>
      <plugin filename="RoverDrivetrain" name="rover_sim::RoverDrivetrain">
```

- [ ] **Step 5: gen_rover builds it; rover_config names the controllers**

In `rover_sim/rover_config.py` replace

```python
SIM_YAML = SIM_DIR / "config" / "sim.yaml"
```

with

```python
SIM_YAML = SIM_DIR / "config" / "sim.yaml"
CONTROL_DIR = SIM_DIR.parent / "rover_control"  # the package beside this one: the controllers the ROS rover runs
CONTROLLERS_YAML = CONTROL_DIR / "config" / "controllers.yaml"
```

In `rover_sim/tools/gen_rover.py` replace

```python
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
```

with

```python
def build_urdf(variant=Variant(), ros2_control=False):
    """The variant's URDF: rover_sim.urdf.xacro expanded with its numbers. With ros2_control, the rover sim.launch.py
    runs: rover_description's ros2_control block with the hardware interface rover_sim/RoverSimSystem and the
    gz_ros2_control system with rover_control's config/controllers.yaml (rover_config.CONTROLLERS_YAML)."""
    import xacro  # here: only building a model needs it
    with tempfile.TemporaryDirectory(prefix="gen_rover_") as tmp:
        rover_yaml, sim_yaml = Path(tmp) / "rover.yaml", Path(tmp) / "sim.yaml"
        rover_yaml.write_text(yaml.safe_dump(variant.rover.plain(), sort_keys=False))
        sim_yaml.write_text(yaml.safe_dump(variant.sim.plain(), sort_keys=False))
        mappings = {"description_dir": str(rover_config.DESCRIPTION_DIR), "rover_yaml": str(rover_yaml),
                    "sim_yaml": str(sim_yaml), "surfaces_yaml": str(SURFACES_YAML)}
        if ros2_control:
            mappings.update(ros2_control="true", controllers=str(rover_config.CONTROLLERS_YAML))
        doc = xacro.process_file(str(XACRO), mappings=mappings)
    return doc.toprettyxml(indent="  ")


def _check_mode(variant):
    mode = variant.sim.drivetrain.mode
    if mode not in ("physical", "diffdrive"):
        raise ValueError(f"sim.yaml drivetrain mode {mode!r}: 'physical' or 'diffdrive'")


@functools.lru_cache(maxsize=64)
def build_sdf(variant=Variant(), ros2_control=False):
    """The variant's model as an SDF 1.11 document: build_urdf, then convert."""
    _check_mode(variant)
    return convert(build_urdf(variant, ros2_control), variant)


def convert(urdf, variant=Variant()):
    """A URDF that build_urdf made of `variant`, as an SDF 1.11 document: `gz sdf -p`, postprocess. sim.launch.py
    converts the URDF it publishes on /robot_description, so that Gazebo runs that rover, post-processed."""
    _check_mode(variant)
    with tempfile.TemporaryDirectory(prefix="gen_rover_") as tmp:
```

and in the module docstring replace

```python
build_sdf(variant) builds a variant of the rover (rover_config.Variant: some of rover.yaml's or sim.yaml's numbers
changed) without writing anything; the tests drive such variants. tests/test_equivalence.py checks that the default
rover and the variants the tests use are the frozen models gen_model.py wrote (tests/fixtures/rover_model).
```

with

```python
build_sdf(variant) builds a variant of the rover (rover_config.Variant: some of rover.yaml's or sim.yaml's numbers
changed) without writing anything; the tests drive such variants. build_urdf(variant, ros2_control=True) and convert()
build the rover with ros2_control that sim.launch.py runs (per launch: its controllers.yaml path is absolute).
tests/test_equivalence.py checks that the default rover and the variants the tests use are the frozen models
gen_model.py wrote (tests/fixtures/rover_model).
```

- [ ] **Step 6: gzenv finds gz_ros2_control and RoverSimSystem, and gives private ROS domains**

In `rover_sim/gzenv.py` replace

```python
- GZ_PARTITION, GZ_IP: only when asked (the tests run in a private partition
  on the loopback interface).
"""
```

with

```python
- GZ_SIM_SYSTEM_PLUGIN_PATH, last: the environment's lib, where
  gz_ros2_control's system plugin is (ros_gz_sim's own launch file adds
  LD_LIBRARY_PATH the same way); Gazebo does not search it.
- AMENT_PREFIX_PATH: the workspace's install/rover_sim first, once colcon has
  installed it (INSTALL_DIR): pluginlib finds RoverSimSystem there, which
  gz_ros2_control loads, without a shell that sourced install/.
- GZ_PARTITION, GZ_IP: only when asked (the tests run in a private partition
  on the loopback interface); ROS_DOMAIN_ID and
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: only when asked (the tests and the
  launcher's worlds take private_ros_domain()).
"""
```

replace

```python
BUILD_DIR = WORKSPACE / "build" / "rover_sim"  # where colcon builds this package
```

with

```python
BUILD_DIR = WORKSPACE / "build" / "rover_sim"  # where colcon builds this package
INSTALL_DIR = WORKSPACE / "install" / "rover_sim"  # where colcon installs it: RoverSimSystem, pluginlib's index
```

replace

```python
VARIABLES = ("GZ_SIM_RESOURCE_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH", "OGRE2_RESOURCE_PATH", "OGRE_RESOURCE_PATH",
             "GZ_RENDERING_RESOURCE_PATH", "GZ_PARTITION", "GZ_IP")


def environment(build_dir=None, *, partition=None, ip=None, base=None):
    """A copy of `base` (default os.environ) with the Gazebo variables set.
    build_dir: where the plugins are built (default BUILD_DIR); partition,
    ip: GZ_PARTITION and GZ_IP, left as they are when None."""
    env = dict(os.environ if base is None else base)
    build = Path(build_dir).resolve() if build_dir else BUILD_DIR
    for key, first in (("GZ_SIM_RESOURCE_PATH", SIM_DIR / "models"), ("GZ_SIM_SYSTEM_PLUGIN_PATH", build)):
        rest = [p for p in env.get(key, "").split(os.pathsep) if p and p != str(first)]
        env[key] = os.pathsep.join([str(first), *rest])
    prefix = Path(env.get("CONDA_PREFIX") or sys.prefix)
```

with

```python
VARIABLES = ("GZ_SIM_RESOURCE_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH", "OGRE2_RESOURCE_PATH", "OGRE_RESOURCE_PATH",
             "GZ_RENDERING_RESOURCE_PATH", "AMENT_PREFIX_PATH", "GZ_PARTITION", "GZ_IP", "ROS_DOMAIN_ID",
             "ROS_AUTOMATIC_DISCOVERY_RANGE")


def private_ros_domain(seed):
    """A ROS_DOMAIN_ID for runs that must not meet another ROS 2 graph on this machine (the tests, the launcher's
    worlds), from `seed` (a process id): 50-99, inside the 0-101 that keeps DDS's ports below the ephemeral ranges of
    Linux, macOS and Windows."""
    return 50 + seed % 50


def environment(build_dir=None, *, partition=None, ip=None, base=None, ros_domain=None, install_dir=None):
    """A copy of `base` (default os.environ) with the Gazebo variables set.
    build_dir: where the plugins are built (default BUILD_DIR); install_dir:
    where colcon installed this package (default INSTALL_DIR); partition, ip:
    GZ_PARTITION and GZ_IP, ros_domain: ROS_DOMAIN_ID (with LOCALHOST
    discovery), left as they are when None."""
    env = dict(os.environ if base is None else base)
    build = Path(build_dir).resolve() if build_dir else BUILD_DIR
    for key, first in (("GZ_SIM_RESOURCE_PATH", SIM_DIR / "models"), ("GZ_SIM_SYSTEM_PLUGIN_PATH", build)):
        rest = [p for p in env.get(key, "").split(os.pathsep) if p and p != str(first)]
        env[key] = os.pathsep.join([str(first), *rest])
    prefix = Path(env.get("CONDA_PREFIX") or sys.prefix)
    plugins = env["GZ_SIM_SYSTEM_PLUGIN_PATH"].split(os.pathsep)
    if str(prefix / "lib") not in plugins:
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join([*plugins, str(prefix / "lib")])
    install = Path(install_dir).resolve() if install_dir else INSTALL_DIR
    if install.is_dir():
        rest = [p for p in env.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p and p != str(install)]
        env["AMENT_PREFIX_PATH"] = os.pathsep.join([str(install), *rest])
```

and replace

```python
    if ip is not None:
        env["GZ_IP"] = ip
    return env
```

with

```python
    if ip is not None:
        env["GZ_IP"] = ip
    if ros_domain is not None:
        env["ROS_DOMAIN_ID"] = str(ros_domain)
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    return env
```

In `rover_sim/tests/simulate.py` replace

```python
# Set before Gazebo starts (gzenv): where model://rover and the plugins live,
# and a private transport partition so tests never talk to a running simulation.
os.environ.update(gzenv.environment(partition=f"rover_sim_test_{os.getpid()}",
                                    ip=os.environ.get("GZ_IP", "127.0.0.1")))
```

with

```python
# Set before Gazebo starts (gzenv): where model://rover and the plugins live,
# and a private transport partition and ROS domain so tests never talk to a
# running simulation or another ROS 2 graph (the ROS tests start rclpy after this).
os.environ.update(gzenv.environment(partition=f"rover_sim_test_{os.getpid()}",
                                    ip=os.environ.get("GZ_IP", "127.0.0.1"),
                                    ros_domain=gzenv.private_ros_domain(os.getpid())))
```

- [ ] **Step 7: Run them to see them pass; the default rover is unchanged**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_rover_model test_foundations.Environment test_rover_config test_equivalence.Equivalence 2>&1 | tail -3
```

Expected: `Ran 72 tests` (test_rover_model 46 + 6, Environment 8 + 2, test_rover_config 8, Equivalence 2) and `OK`. The default rover is byte for byte the tracked model (`TrackedModel`), and every frozen variant is still equal (`Equivalence`).

- [ ] **Step 8: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_control/config/controllers.yaml rover_sim/urdf/rover_sim.urdf.xacro rover_sim/tools/gen_rover.py rover_sim/rover_config.py rover_sim/gzenv.py rover_sim/tests/simulate.py rover_sim/tests/test_rover_model.py rover_sim/tests/test_foundations.py && git commit -q -F - <<'EOF'
PR 3: the rover with ros2_control, rover_control's controllers.yaml, and the environment that finds them

rover_sim.urdf.xacro with ros2_control:=true (gen_rover.build_urdf/build_sdf(..., ros2_control=True), and
gen_rover.convert for a URDF already built) adds rover_description's ros2_control block with the hardware interface
rover_sim/RoverSimSystem and the gz_ros2_control system with rover_control's config/controllers.yaml, just before the
drivetrain, so the controller manager's write() runs first in each step. The default rover, the worlds' and the
tests', is unchanged byte for byte. controllers.yaml: joint_state_broadcaster and drive_controller
(diff_drive_controller, left FL+RL, right FR+RR) with rover.yaml's geometry and limits, the 0.5 s command timeout, the
ramp at a wheel's 8 rad/s^2, 100 Hz. gzenv appends the environment's lib to GZ_SIM_SYSTEM_PLUGIN_PATH
(gz_ros2_control's system), puts install/rover_sim first on AMENT_PREFIX_PATH (pluginlib's RoverSimSystem), and
gives private ROS domains (private_ros_domain: 50-99), which the tests take through simulate.py.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 6: The prototype: RoverSimSystem inside Gazebo with the controllers, on this Mac

**Files:** Create `rover_sim/tests/rosstack.py`, `rover_sim/tests/test_ros2_control.py`; `$T/proto_stack.py`, `$T/odom_once.py`, `$T/split_check.sh`. Modify this plan (the results).

- [ ] **Step 1: The ROS test helpers**

Write `rover_sim/tests/rosstack.py`:

```python
"""ROS 2 runs for the tests: processes in sessions of their own, stopped with everything they started, and a probe
node in this process that publishes /cmd_vel and records what comes back, all in this process's private ROS domain
and gz partition (simulate.py sets both, and gzenv's Gazebo environment, when it is imported)."""
import contextlib
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import simulate  # noqa: F401  (sets GZ_PARTITION, GZ_IP, ROS_DOMAIN_ID and Gazebo's paths before rclpy starts)

import rclpy  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

import rover_config  # noqa: E402

STOP_TIMEOUT = 30.0  # [s] for a process group to end on SIGINT before what is left of it is killed


def start(cmd, log, env=None):
    """A process in a session of its own (so stop() reaches what it starts), its output in the file `log`."""
    with open(log, "w") as out:
        return subprocess.Popen([str(c) for c in cmd], stdout=out, stderr=subprocess.STDOUT, env=env or os.environ,
                                start_new_session=True)


def _alive(group):
    try:
        os.killpg(group, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def stop(process, timeout=STOP_TIMEOUT):
    """SIGINT to the process's group, SIGKILL to what is left of it after `timeout`; the process's exit code."""
    group = process.pid
    with contextlib.suppress(ProcessLookupError):
        os.killpg(group, signal.SIGINT)
    deadline = time.monotonic() + timeout
    while (process.poll() is None or _alive(group)) and time.monotonic() < deadline:
        time.sleep(0.2)
    if process.poll() is None or _alive(group):
        with contextlib.suppress(ProcessLookupError):
            os.killpg(group, signal.SIGKILL)
        process.wait()
    return process.returncode


def spawner(controller="drive_controller", extra_params=(), timeout=180):
    """The spawner that rover_control's launch/controllers.launch.py runs: joint_state_broadcaster and the drive
    controller from rover_control's config/controllers.yaml (then extra_params, which override it), the drive
    controller's topics on /cmd_vel and /odom."""
    files = [arg for path in (rover_config.CONTROLLERS_YAML, *extra_params) for arg in ("--param-file", str(path))]
    return ["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster", controller, *files,
            "--controller-manager", "/controller_manager", "--controller-manager-timeout", str(timeout),
            f"--controller-ros-args=-r /{controller}/cmd_vel:=/cmd_vel -r /{controller}/odom:=/odom"]


def shutdown():
    """rclpy off (a test module's tearDownModule)."""
    if rclpy.ok():
        rclpy.shutdown()


class Probe:
    """A node in this process: publishes TwistStamped commands on /cmd_vel with a zero stamp (the drive controller
    stamps them on arrival with its own clock, docs/topics.md) and records /odom, /joint_states and `topics`
    [(name, message type)] in `messages`."""

    def __init__(self, topics=()):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(f"rover_sim_test_{os.getpid()}")
        self.cmd = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.messages = {}
        for topic, kind in [("/odom", Odometry), ("/joint_states", JointState), *topics]:
            self.messages[topic] = []
            self.node.create_subscription(kind, topic, self.messages[topic].append, 100)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def sim_time(self):
        """The latest /odom's stamp [s]: sim time on the controller manager's clock (0 before the first)."""
        odom = self.messages["/odom"]
        return odom[-1].header.stamp.sec + odom[-1].header.stamp.nanosec * 1e-9 if odom else 0.0

    def drive_for(self, vx, wz, sim_seconds, timeout=60.0, rate=20.0, publish=True):
        """Publish the command `rate` times a second until /odom's stamps have advanced `sim_seconds` [s] of sim time,
        or `timeout` [s] of wall time has passed: a machine that runs the simulation slower than real time (CI's
        runners) drives as far as a fast one. publish=False only waits (the drive controller's timeout brakes)."""
        msg = TwistStamped()
        msg.twist.linear.x, msg.twist.angular.z = float(vx), float(wz)
        start, end = self.sim_time(), time.monotonic() + timeout
        while self.sim_time() - start < sim_seconds and time.monotonic() < end:
            if publish:
                self.cmd.publish(msg)
            time.sleep(1.0 / rate)

    def wait(self, condition, timeout, what, logs=()):
        """Wait until condition() is true; after `timeout` [s] fail with the tails of the files `logs`."""
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                tails = "\n".join(f"--- {log}\n{Path(log).read_text(errors='replace')[-3000:]}" for log in logs)
                raise AssertionError(f"no {what} within {timeout:.0f} s\n{tails}")
            time.sleep(0.2)

    def topics(self):
        """The names of the topics in the ROS graph."""
        return {name for name, _ in self.node.get_topic_names_and_types()}

    def close(self):
        self.executor.shutdown()
        self.thread.join(5)
        self.node.destroy_node()
```

- [ ] **Step 2: The test**

Write `rover_sim/tests/test_ros2_control.py`:

```python
#!/usr/bin/env python3
"""RoverSimSystem (ros2_control/rover_sim_system.cpp) inside Gazebo with the rover's controllers (design spec
2026-10-09, section 6.1), as sim.launch.py runs it: `gz sim -s` with the rover built with ros2_control,
robot_state_publisher, the /clock bridge and rover_control's controllers (joint_state_broadcaster, drive_controller),
in this process's private ROS domain and gz partition (rosstack, simulate). One run on flat ground in real time,
with the drivetrain's state every 1 ms step (stamped with the step's sim time) and the drive controller's messages
(stamped with the controller manager's clock: ros2_control 4.47 gives the controllers and the hardware that clock,
sim time from the bridged /clock, a few ms behind the step); the tests read it:
- gz_ros2_control loads RoverSimSystem and the controllers drive the wheels: /odom and the wheels move;
- the setpoint the controller manager writes at the start of a step is the one the motor model uses in that step;
- the joints' state and the motor currents come back on /joint_states.
The description of that rover is checked in test_rover_model.Ros2ControlVariant."""
import json
import tempfile
import unittest
from pathlib import Path

import yaml

import rosstack
from simulate import physical, world_sdf

import gen_rover  # noqa: E402  (worldfiles, through simulate, puts sim/ and sim/tools on the path)
import rover_config  # noqa: E402
from rover_config import ROVER  # noqa: E402
from urc import sdf  # noqa: E402

from geometry_msgs.msg import TwistStamped  # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402
from gz.transport13 import Node  # noqa: E402

WHEELS = ("fl", "rl", "fr", "rr")
SIDE = {"fl": 0, "rl": 0, "fr": 1, "rr": 1}  # index into (left, right)
JOINTS = {f"wheel_{w}_joint" for w in WHEELS} | {"rocker_left_joint", "rocker_right_joint", rover_config.PAN_JOINT,
                                                 rover_config.TILT_JOINT}
CMD_VEL_OUT = "/drive_controller/cmd_vel_out"
DRIVE = (0.3, 0.5)  # [m/s], [rad/s]: an arc to the left, every wheel forward, the right ones faster
SEPARATION, RADIUS = 2 * ROVER.rocker.pivot_y, ROVER.wheel.radius  # controllers.yaml's (rover_control checks them)
UPDATE_STEPS = 10  # gz_ros2_control updates the controllers every 10th 1 ms step (100 Hz): steps 10, 20, 30 ...


def tearDownModule():
    rosstack.shutdown()


def ms(stamp):
    """A ROS stamp, or a sim time [s], in whole milliseconds: the physics step's grid."""
    if hasattr(stamp, "sec"):
        return round(stamp.sec * 1000 + stamp.nanosec / 1e6)
    return round(stamp * 1000)


def close(got, want):
    """Equal within the drivetrain state's 6 significant digits."""
    return abs(got - want) <= 1e-5 * abs(want) + 1e-6


class RoverSimSystemInGazebo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="rover_ros2_control_")
        tmp = Path(cls.tmp.name)
        variant = physical(drivetrain={"state_rate": 1000.0})
        urdf = gen_rover.build_urdf(variant, ros2_control=True)
        (tmp / "rover").mkdir()
        (tmp / "rover" / "model.sdf").write_text(gen_rover.convert(urdf, variant))
        (tmp / "rover" / "model.config").write_text(sdf.model_config("rover", "The rover with ros2_control.",
                                                                     "rover_sim/tests/test_ros2_control.py"))
        world = tmp / "world.sdf"
        world.write_text(world_sdf(rover_uri=(tmp / "rover").as_uri()).replace(
            "<real_time_factor>0</real_time_factor>", "<real_time_factor>1</real_time_factor>"))
        (tmp / "rsp.yaml").write_text(yaml.safe_dump(
            {"robot_state_publisher": {"ros__parameters": {"robot_description": urdf, "use_sim_time": True}}}))
        (tmp / "test.yaml").write_text(yaml.safe_dump(
            {"drive_controller": {"ros__parameters": {"publish_limited_velocity": True}}}))
        cls.states = []
        node = Node()
        node.subscribe(StringMsg, rover_config.DRIVETRAIN_TOPIC, lambda m: cls.states.append(json.loads(m.data)))
        logs = [tmp / name for name in ("gz.log", "rsp.log", "bridge.log", "spawner.log")]
        processes = [
            rosstack.start(["gz", "sim", "-s", "-r", world], logs[0]),
            rosstack.start(["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args",
                            "--params-file", tmp / "rsp.yaml"], logs[1]),
            rosstack.start(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], logs[2]),
            rosstack.start(rosstack.spawner(extra_params=[tmp / "test.yaml"]), logs[3]),
        ]
        probe = rosstack.Probe(topics=[(CMD_VEL_OUT, TwistStamped)])
        try:
            probe.wait(lambda: probe.messages[CMD_VEL_OUT] and probe.cmd.get_subscription_count() > 0, 180,
                       "update of the drive controller", logs)
            cls.drive(probe)
        finally:
            cls.messages = {topic: list(found) for topic, found in probe.messages.items()}
            probe.close()
            for process in reversed(processes):
                rosstack.stop(process)
            node.unsubscribe(rover_config.DRIVETRAIN_TOPIC)
        cls.log = logs[0].read_text(errors="replace")

    @classmethod
    def drive(cls, probe):
        probe.drive_for(*DRIVE, sim_seconds=3.0)
        probe.drive_for(0.0, 0.0, sim_seconds=1.5)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_the_controllers_drive_the_rover(self):
        self.assertIn("RoverSimSystem: 8 joints, 4 wheels driven", self.log)
        self.assertTrue(self.messages[CMD_VEL_OUT], "no cmd_vel_out: the drive controller never updated")
        odom = self.messages["/odom"]
        self.assertTrue(odom, "no /odom")
        self.assertGreater(odom[-1].pose.pose.position.x, 0.3)
        self.assertGreater(odom[-1].pose.pose.position.y, 0.05)  # the arc turned left
        joints = self.messages["/joint_states"][-1]
        self.assertEqual(set(joints.name), JOINTS)
        position = dict(zip(joints.name, joints.position))
        # The arc's wheel commands: left (0.3 - 0.5 x 0.4) / 0.15 = 0.67 rad/s, right 3.33 rad/s, for 3 s of sim time
        # less the ramps: some 1.9 and 9.6 rad.
        for w in WHEELS:
            self.assertGreater(position[f"wheel_{w}_joint"], 0.5 if SIDE[w] == 0 else 5.0, w)
        self.assertGreater(position["wheel_fr_joint"], 3 * position["wheel_fl_joint"])

    def test_a_setpoint_written_at_a_step_drives_the_motors_in_that_step(self):
        """gz_ros2_control runs the controller manager's update in its PostUpdate of every 10th step, counted on sim
        time from 0 (steps 10, 20, 30 ...), and hands the new wheel commands to RoverSimSystem in its PreUpdate of
        the next step; the drivetrain, whose plugin comes after gz_ros2_control's, uses them in that same step. So
        every change of a setpoint in the drivetrain's state (every step) is in a step 11, 21, 31 ...: written and
        used in the same step (one step later they would be in steps 12, 22, 32 ...). The stamps of cmd_vel_out and
        /joint_states cannot show it: they are the controller manager's clock, a few ms behind the step."""
        sp = {ms(s["t"]): tuple(s["wheels"][w]["sp"] for w in WHEELS) for s in self.states}
        changes = [t for t in sorted(sp) if t - 1 in sp and sp[t] != sp[t - 1]]
        self.assertGreater(len(changes), 20)  # the ramps up and down: some 50 updates that change a command
        self.assertEqual(sorted({t % UPDATE_STEPS for t in changes}), [1], changes[:20])

    def test_the_joint_state_and_the_motor_currents_come_back(self):
        """/joint_states (joint_state_broadcaster) carries each wheel's motor current as its effort: the drivetrain's
        current of an update step (read() runs in gz_ros2_control's PostUpdate of steps 10, 20, 30 ..., after the
        drivetrain's PreUpdate of that step wrote it). A message's stamp is the controller manager's clock, a few ms
        behind the step, so the step is found near it: the steps within 50 ms whose four currents all equal the
        efforts. Where that is one step (the currents change from step to step while the rover drives), it is an
        update step."""
        currents = {ms(s["t"]): [s["wheels"][w]["i"] for w in WHEELS] for s in self.states}
        pinned = drawn = 0
        for message in self.messages["/joint_states"]:
            effort = dict(zip(message.name, message.effort))
            got = [effort[f"wheel_{w}_joint"] for w in WHEELS]
            stamp = ms(message.header.stamp)
            if not any(t in currents for t in range(stamp - 50, stamp + 51)):
                continue  # before the drivetrain's first state, or after its last
            steps = [t for t in range(stamp - 50, stamp + 51)
                     if t in currents and all(close(g, c) for g, c in zip(got, currents[t]))]
            self.assertTrue(steps, (stamp, got))
            if len(steps) == 1:
                self.assertEqual(steps[0] % UPDATE_STEPS, 0, (stamp, steps, got))
                pinned += 1
            drawn += max(abs(g) for g in got) > 1.0
        self.assertGreater(pinned, 50)
        self.assertGreater(drawn, 0, "the motors drew current")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run it: the prototype on this Mac**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run sim-build 2>&1 | tail -1
cd rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_ros2_control > $T/proto.log 2>&1; tail -8 $T/proto.log
```

Expected: `Summary: 1 package finished`; then the three tests `... ok`, `Ran 3 tests in <about 30>s`, `OK`. On a failure the message carries the tails of the four logs. Diagnose by the symptom before changing anything:
- `no update of the drive controller within 180 s` and `gz.log` says `Waiting RM to load and initialize hardware...` without end: the controller manager never got `/robot_description` (check `rsp.log`; the domain is the test process's, `ROS_DOMAIN_ID` 50-99).
- `gz.log` has `Failed to load system plugin [gz_ros2_control-system]`: `GZ_SIM_SYSTEM_PLUGIN_PATH` lacks the environment's lib (Task 5's gzenv).
- `The plugin failed to load ... rover_sim/RoverSimSystem`: pluginlib did not find it: `AMENT_PREFIX_PATH` lacks `install/rover_sim` (`pixi run sim-build` installs it).
- the timing test fails with the changes in steps 12, 22, 32 ... (`{2}` instead of `[1]`): the drivetrain ran before gz_ros2_control in the step (one step later than designed). The task allows step k + 1; record it, change the test to assert `[2]`, and say so in the results and in `rover_sim_system.hpp`'s comment. A mix of residues means setpoint changes that no controller update made (the watchdog: look for a stamp far behind the step, a stalled `/clock` bridge).
- the readback test finds no step whose currents equal a message's efforts: `read()` did not see the drivetrain's `WheelMotorCurrent` (the component registered twice as two types: `GZ_DEBUG_COMPONENT_FACTORY=true` prints the factory's registrations).

- [ ] **Step 4: Gazebo's GUI as a separate process beside the server**

Write `$T/proto_stack.py`:

```python
"""Write the rover with ros2_control, its flat world in real time and robot_state_publisher's parameters into a folder,
as tests/test_ros2_control.py does: python proto_stack.py <folder>."""
import sys
from pathlib import Path

sys.path.insert(0, "/Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests")
import yaml  # noqa: E402

from simulate import physical, world_sdf  # noqa: E402  (first: worldfiles puts rover_sim/ and its tools/ on the path)

import gen_rover  # noqa: E402
from urc import sdf  # noqa: E402

out = Path(sys.argv[1])
(out / "rover").mkdir(parents=True, exist_ok=True)
variant = physical()
urdf = gen_rover.build_urdf(variant, ros2_control=True)
(out / "rover" / "model.sdf").write_text(gen_rover.convert(urdf, variant))
(out / "rover" / "model.config").write_text(sdf.model_config("rover", "The rover with ros2_control.", "proto_stack.py"))
(out / "world.sdf").write_text(world_sdf(rover_uri=(out / "rover").as_uri()).replace(
    "<real_time_factor>0</real_time_factor>", "<real_time_factor>1</real_time_factor>"))
(out / "rsp.yaml").write_text(yaml.safe_dump(
    {"robot_state_publisher": {"ros__parameters": {"robot_description": urdf, "use_sim_time": True}}}))
```

Write `$T/odom_once.py`:

```python
"""Print /odom's position once (waiting up to 10 s)."""
import time

import rclpy
from nav_msgs.msg import Odometry

rclpy.init()
node = rclpy.create_node("odom_once")
got = []
node.create_subscription(Odometry, "/odom", got.append, 10)
end = time.time() + 10
while not got and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
print(f"odom x {got[-1].pose.pose.position.x:.3f} y {got[-1].pose.pose.position.y:.3f}" if got else "no /odom")
rclpy.shutdown()
```

Write `$T/split_check.sh`:

```bash
#!/bin/bash
# The rover with ros2_control in `gz sim -s`, Gazebo's GUI as a separate process (as sim.launch.py starts them; macOS
# cannot run both in one): the server loads RoverSimSystem, the controllers come up, /cmd_vel drives the rover while
# the GUI is open, and the GUI is still running after it. $1: the scripts' folder.
set -m
T="$1"; D="$T/split"; rm -rf "$D"; mkdir -p "$D"
SRC=/Users/alarion239/Desktop/hurc_ws/src
export GZ_IP=127.0.0.1 GZ_PARTITION=pr3_split ROS_DOMAIN_ID=63 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
eval "$(python "$SRC/rover_sim/gzenv.py")"
python "$T/proto_stack.py" "$D"
gz sim -s -r "$D/world.sdf" > "$D/server.log" 2>&1 & SERVER=$!
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$D/rsp.yaml" > "$D/rsp.log" 2>&1 & RSP=$!
ros2 run ros_gz_bridge parameter_bridge "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" > "$D/bridge.log" 2>&1 & BRIDGE=$!
ros2 run controller_manager spawner joint_state_broadcaster drive_controller \
  --param-file "$SRC/rover_control/config/controllers.yaml" --controller-manager-timeout 120 \
  "--controller-ros-args=-r /drive_controller/cmd_vel:=/cmd_vel -r /drive_controller/odom:=/odom" > "$D/spawner.log" 2>&1
echo "spawner exit $?"
gz sim -g > "$D/gui.log" 2>&1 & GUI=$!
sleep 8
ros2 topic pub --times 60 --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}}}" > /dev/null
python "$T/odom_once.py"
kill -0 $GUI 2>/dev/null && echo "gui running" || echo "gui exited"
kill -INT $GUI; sleep 2
kill -INT $SERVER $RSP $BRIDGE; wait 2>/dev/null
sleep 1; pgrep -f "$D/world.sdf" > /dev/null && echo "server still running" || echo "server stopped"
grep -E "RoverSimSystem:|Loading controller_manager" "$D/server.log" | head -3
grep -cE "Configured and activated" "$D/spawner.log"
grep -c "Failed to create component" "$D/gui.log"
```

Run it (a Gazebo window opens on this Mac for about 12 s):

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run bash $T/split_check.sh $T 2>&1 | tail -8
```

Expected: `spawner exit 0`; `odom x <above 1.0> y <near 0>` (0.5 m/s for 3 s, less the ramp and the slip); `gui running`; `server stopped`; the server's lines `Loading controller_manager` and `RoverSimSystem: 8 joints, 4 wheels driven through RoverDrivetrain's motor model`; `2` (both controllers activated); then the count of the GUI's `Failed to create component type [...]` lines, probably `8`. The server's full state, which the GUI asks for when it connects, carries the rover's own components, `WheelSetpoint` and `WheelMotorCurrent` on each wheel joint. The plan critic's probe saw both in the reply of `/world/<name>/state`. The GUI has not loaded the library that registers them, so it cannot create them. That is harmless (the GUI draws nothing from them): record the count, and the manual's paragraph on harmless log lines (Task 19) names the line. If `server still running` is printed, stop it yourself (`pkill -INT -f "$T/split/world.sdf"`): only what this step started.

- [ ] **Step 5: Record the results in this plan; the gate**

Append to the end of this plan file (`docs/superpowers/plans/2026-10-10-pr3-ros2-control.md`) a section with what was measured, in the plan's style:

```markdown
## Prototype results (Task 6, <date>)

Measured on the Apple M4 (macOS <version>), gz-sim 8.10, gz_ros2_control 1.2.19, ros2_control 4.47.0.
- `GZ_SIM_SYSTEM_PLUGIN_PATH` after activation: <Task 2's line>; gzenv appends the environment's lib.
- `test_ros2_control`: <the Ran/OK lines of $T/proto.log>; the controllers active <seconds> after the server started.
- Timing: a setpoint the controller manager writes at the start of step k drives the motors in step k: every one of
  the <count> setpoint changes in the drivetrain's state is in a step 11, 21, 31 ... (the controllers update in the
  PostUpdate of steps 10, 20, 30 ...); or: in step k + 1, with the reason found.
- Readback: /joint_states' wheel efforts equal the drivetrain's currents of an update step (<pinned> messages pinned to
  one step, every one a step 10, 20, 30 ...); the messages' stamps lag the step by <range> ms (the controller
  manager's clock, the bridged /clock).
- Server and GUI as separate processes: <split_check.sh's lines>.
- <anything that differed from this plan: an include path, a log line, a fix, and where it went>.
```

Gate: Parts B to E start only if `test_ros2_control` passes (with the timing at step k, or at k + 1 recorded) and `split_check.sh` printed `gui running` with `odom x` above 1.0. Otherwise stop here and report to the user: the failing lines, the logs under `$T`, and the options in this plan's Risks table.

- [ ] **Step 6: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/tests/rosstack.py rover_sim/tests/test_ros2_control.py docs/superpowers/plans/2026-10-10-pr3-ros2-control.md && git commit -q -F - <<'EOF'
PR 3: the prototype: RoverSimSystem inside Gazebo with the rover's controllers

tests/test_ros2_control.py runs the rover with ros2_control in `gz sim -s` with robot_state_publisher, the /clock
bridge and rover_control's controllers, in the test process's private ROS domain, and checks that gz_ros2_control
loads RoverSimSystem and the controllers drive the wheels, that a setpoint the controller manager writes at the start
of a step drives the motors in that step (every setpoint change in a step after the controllers' update step: 11,
21, 31 ...; the messages' stamps are the controller manager's clock, a few ms behind), and that the joints' state and
the motor currents of an update step come back on /joint_states. tests/rosstack.py: the ROS tests' processes and
probe node. The plan records the prototype's results on this Mac, the server and Gazebo's GUI as separate processes
among them.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

---

## Part B: the simulation side

Order: the ROS-free helpers first (Task 7), then every test and tool switches to them while the drivetrain still takes its twist (Task 8: the helpers publish the twist as before, and the setpoints the drivetrain will take), then the drivetrain takes only setpoints (Task 9). The suite is green after each task, and the traces of Task 1 stay bit for bit (Task 10).

### Task 7: wheel_cmd.py: the setpoints the drivetrain made of a twist

**Files:** Create `rover_sim/wheel_cmd.py`, `rover_sim/tests/test_wheel_cmd.py`. Modify `rover_sim/config/sim.yaml`, `rover_sim/rover_config.py`, `rover_sim/tests/test_rover_config.py`.

- [ ] **Step 1: The failing tests**

Write `rover_sim/tests/test_wheel_cmd.py`:

```python
#!/usr/bin/env python3
"""wheel_cmd.py, the ROS-free way to drive the simulated rover: the skid-steer formula and the ramp the drivetrain
applied before PR 3, what Driver and TwistDrive publish and when, and that the drive controller of the ROS stack ramps
each wheel at the same rate (rover_control/config/controllers.yaml). Nothing runs Gazebo; Task 10 of the PR 3 plan
compared whole runs bit for bit."""
import unittest

import yaml

from worldfiles import SIM_DIR  # noqa: F401  (puts rover_sim/ on the path)

import rover_config  # noqa: E402
import wheel_cmd  # noqa: E402
from rover_config import ROVER, SIM  # noqa: E402

TRACK, RADIUS, TOP = 2 * ROVER.rocker.pivot_y, ROVER.wheel.radius, ROVER.wheel.top_speed


class FakeNode:
    """advertise() as gz.transport13.Node has it, recording a copy of every message published as (topic, message)."""

    def __init__(self):
        self.sent = []

    def advertise(self, topic, kind):
        sent = self.sent

        class Publisher:
            def publish(self, msg):
                copy = kind()
                copy.CopyFrom(msg)
                sent.append((topic, copy))

        return Publisher()

    def of(self, topic):
        return [msg for t, msg in self.sent if t == topic]


class Setpoints(unittest.TestCase):
    def test_the_drivetrains_formula(self):
        vx, wz = 0.3, 0.2
        left, right = (vx - 1.0 * wz * TRACK / 2) / RADIUS, (vx - -1.0 * wz * TRACK / 2) / RADIUS
        self.assertEqual(wheel_cmd.setpoints(vx, wz), (left, left, right, right))
        self.assertEqual(len(set(wheel_cmd.setpoints(0.5, 0.0))), 1)  # straight: all alike
        fl, rl, fr, rr = wheel_cmd.setpoints(0.0, 1.0)
        self.assertEqual((fl, rl), (-fr, -rr))  # in place: the sides opposite
        self.assertAlmostEqual(fr, ROVER.rocker.pivot_y / RADIUS)
        self.assertEqual(wheel_cmd.WHEELS, ("fl", "rl", "fr", "rr"))


class RampSteps(unittest.TestCase):
    def test_at_most_accel_per_second(self):
        ramp = wheel_cmd.Ramp(accel=8.0)
        self.assertEqual(ramp.step(0.3, 0.0), (8.0 * 0.001,) * 4)  # 2 rad/s asked, 8 rad/s^2 for 1 ms
        for _ in range(248):
            values = ramp.step(0.3, 0.0)
        self.assertAlmostEqual(values[0], 249 * 0.008, places=9)
        for _ in range(10):
            values = ramp.step(0.3, 0.0)
        for value in values:
            self.assertAlmostEqual(value, 0.3 / RADIUS, places=12)  # arrived, and held

    def test_the_top_speed_first(self):
        ramp = wheel_cmd.Ramp(accel=0.0)
        for value in ramp.step(10.0, 0.0):  # 66.7 rad/s asked
            self.assertAlmostEqual(value, TOP, places=9)

    def test_without_accel_the_arithmetic_stays(self):
        """accel 0 is no limit, but the value still goes through SpeedLimiter's arithmetic, previous + (target -
        previous) / dt x dt, which may differ from the target in the last bit, as the drivetrain's did."""
        ramp = wheel_cmd.Ramp(accel=0.0)
        target = wheel_cmd.setpoints(0.37, 0.11)
        self.assertEqual(ramp.step(0.37, 0.11), tuple(0.0 + ((t - 0.0) / 0.001) * 0.001 for t in target))

    def test_one_rounding_as_libgz_math_computes_it(self):
        """previous + acceleration x dt is rounded once, as libgz-math7's SpeedLimiter, which ramped the drivetrain's
        setpoints until PR 3, computes it on osx-arm64 (a fused multiply-add; measured against the library,
        2026-10-10): a jump from a turn in place back to zero leaves the residue the drivetrain left, where two
        roundings give 0."""
        ramp = wheel_cmd.Ramp(accel=0.0)
        self.assertEqual(ramp.step(0.0, 1.0)[0].hex(), "-0x1.5555555555556p+1")
        fl, rl, fr, rr = ramp.step(0.0, 0.0)
        self.assertEqual((fl, rl), (float.fromhex("0x1.20c49ba5e354p-54"),) * 2)
        self.assertEqual((fr, rr), (-fl, -fl))

    def test_reset_starts_from_zero(self):
        ramp = wheel_cmd.Ramp(accel=8.0)
        ramp.step(0.3, 0.0)
        ramp.reset()
        self.assertEqual(ramp.values, (0.0,) * 4)


class DriverPublishes(unittest.TestCase):
    def test_nothing_before_the_first_command(self):
        node = FakeNode()
        driver = wheel_cmd.Driver(node)
        for _ in range(30):
            driver.step(force=True)
        self.assertEqual(node.sent, [])

    def test_every_change_and_the_beat(self):
        node = FakeNode()
        driver = wheel_cmd.Driver(node, accel=8.0)
        driver.command(0.3, 0.0)
        self.assertEqual([(m.linear.x, m.angular.z) for m in node.of(rover_config.CMD_VEL_TOPIC)], [(0.3, 0.0)])
        for _ in range(300):
            driver.step()
        wheels = node.of(rover_config.WHEEL_CMD_TOPIC)
        self.assertTrue(248 <= len(wheels) <= 255, len(wheels))  # the 250 steps of the ramp, not the held ones
        self.assertAlmostEqual(wheels[-1].data[0], 0.3 / RADIUS, places=9)
        driver.step(force=True)
        self.assertEqual(len(node.of(rover_config.WHEEL_CMD_TOPIC)), len(wheels) + 1)  # the beat repeats them

    def test_a_reset_waits_for_the_next_command(self):
        node = FakeNode()
        driver = wheel_cmd.Driver(node, accel=8.0)
        driver.command(0.3, 0.0)
        driver.step()
        driver.reset()
        count = len(node.sent)
        driver.step(force=True)
        self.assertEqual(len(node.sent), count)
        driver.command(0.3, 0.0)
        driver.step()
        self.assertEqual(node.of(rover_config.WHEEL_CMD_TOPIC)[-1].data[0], 8.0 * 0.001)  # from zero again


class TwistDriveSends(unittest.TestCase):
    def test_unramped_setpoints_and_the_twist(self):
        node = FakeNode()
        wheel_cmd.TwistDrive(node).twist(0.6, 0.8)
        self.assertEqual(tuple(node.of(rover_config.WHEEL_CMD_TOPIC)[0].data), wheel_cmd.setpoints(0.6, 0.8))
        self.assertEqual(node.of(rover_config.CMD_VEL_TOPIC)[0].angular.z, 0.8)


class DriveController(unittest.TestCase):
    def test_it_ramps_each_wheel_at_the_helpers_rate(self):
        """controllers.yaml's linear and angular acceleration limits are sim.yaml's wheel_cmd accel for a wheel
        driving straight and turning in place, and its kinematics have no track multiplier (1)."""
        params = yaml.safe_load(rover_config.CONTROLLERS_YAML.read_text())["drive_controller"]["ros__parameters"]
        accel, half = SIM.wheel_cmd.accel, ROVER.rocker.pivot_y
        self.assertAlmostEqual(params["linear.x.max_acceleration"] / RADIUS, accel, places=9)
        self.assertAlmostEqual(-params["linear.x.max_deceleration"] / RADIUS, accel, places=9)
        self.assertAlmostEqual(params["angular.z.max_acceleration"] * half / RADIUS, accel, places=9)
        self.assertAlmostEqual(-params["angular.z.max_deceleration"] * half / RADIUS, accel, places=9)
        self.assertEqual(params["wheel_separation_multiplier"], 1.0)


if __name__ == "__main__":
    unittest.main()
```

In `rover_sim/tests/test_rover_config.py` replace

```python
        self.assertEqual(rover_config.CMD_VEL_TOPIC, "/model/rover/cmd_vel")
```

with

```python
        self.assertEqual(rover_config.CMD_VEL_TOPIC, "/model/rover/cmd_vel")
        self.assertEqual(rover_config.WHEEL_CMD_TOPIC, "/model/rover/wheel_cmd")
```

- [ ] **Step 2: Run them to see them fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_wheel_cmd test_rover_config 2>&1 | tail -3
```

Expected: `FAILED (errors=...)`: `No module named 'wheel_cmd'` and `module 'rover_config' has no attribute 'WHEEL_CMD_TOPIC'`.

- [ ] **Step 3: sim.yaml's topic and ramp; wheel_cmd.py**

In `rover_sim/config/sim.yaml` replace

```yaml
# Gazebo's DiffDrive (drivetrain mode diffdrive). Lateral friction below longitudinal emulates tyre scrub; with equal
```

with

```yaml
# The ROS-free helpers (wheel_cmd.py: the tests, tools and scripts that speak twists to a rover without ROS) turn a
# twist into wheel setpoints as the drivetrain did before the ros2_control boundary (PR 3): skid steer with the
# geometric track, clamped to the top speed, each wheel's setpoint ramped at accel, so every run drives as it did. The
# ROS stack's drive controller ramps vx and wz at the same wheel rate (rover_control/config/controllers.yaml;
# tests/test_wheel_cmd.py checks it).
wheel_cmd:
  accel: 8.0                        # [rad/s^2] each wheel's setpoint, 1.2 m/s^2 (A: until the driver team's, Q7)

# Gazebo's DiffDrive (drivetrain mode diffdrive). Lateral friction below longitudinal emulates tyre scrub; with equal
```

and replace

```yaml
  cmd_vel: /model/rover/cmd_vel
```

with

```yaml
  cmd_vel: /model/rover/cmd_vel
  wheel_cmd: /model/rover/wheel_cmd             # the wheels' speed setpoints [rad/s]: gz.msgs.Double_V, fl rl fr rr
```

In `rover_sim/rover_config.py` replace

```python
CMD_VEL_TOPIC = SIM.topics.cmd_vel
```

with

```python
CMD_VEL_TOPIC = SIM.topics.cmd_vel
WHEEL_CMD_TOPIC = SIM.topics.wheel_cmd  # the drivetrain's wheel setpoints: gz.msgs.Double_V, fl rl fr rr (wheel_cmd.py)
```

Write `rover_sim/wheel_cmd.py`:

```python
"""Driving the simulated rover without ROS. The drivetrain's motors follow wheel speed setpoints
(plugins/rover_drivetrain.cpp; design spec 2026-10-09, section 6.1): on the rover with ros2_control, RoverSimSystem
writes them from the drive controller; without it they come on the drivetrain's gz-transport topic
rover_config.WHEEL_CMD_TOPIC, a gz.msgs.Double_V with one value per wheel in WHEELS order [rad/s].

The simulation's tests, tools and scripts speak body twists. This module turns a twist into the setpoints the
drivetrain itself made of it before the ros2_control boundary: each wheel at (vx - side x wz x track / 2) / radius
(side +1 left, -1 right; track 2 x rover.yaml rocker.pivot_y), clamped to the top speed and ramped at sim.yaml's
wheel_cmd accel, step by step with gz::math::SpeedLimiter's arithmetic as libgz-math7 computes it on osx-arm64 (one
rounding of previous + acceleration x dt, a fused multiply-add), so that a run drives as it did, to the bit. The ROS
stack has its own kinematics and ramp in the drive controller (rover_control/config/controllers.yaml), with the same
numbers (tests/test_wheel_cmd.py).

Driver feeds a run stepped in this process (gz.sim8.TestFixture); TwistDrive sends each twist as it comes, for a
caller whose twists are ramped already (the driver station slews its own).
"""
import math
from fractions import Fraction

from rover_config import CMD_VEL_TOPIC, ROVER, SIM, WHEEL_CMD_TOPIC

WHEELS = ("fl", "rl", "fr", "rr")  # the message's order: the drivetrain's <wheel> elements (urdf/rover_sim.urdf.xacro)
SIDES = (1.0, 1.0, -1.0, -1.0)  # +1 left, -1 right, in WHEELS order
STEP = 0.001  # [s] the worlds' physics step


def setpoints(vx, wz, rover=ROVER):
    """Each wheel's speed [rad/s] for the twist (vx [m/s], wz [rad/s]), in WHEELS order: the drivetrain's formula
    before PR 3, its operations in the same order, so the same doubles."""
    track, radius = 2 * rover.rocker.pivot_y, rover.wheel.radius
    return tuple((vx - side * wz * track / 2) / radius for side in SIDES)


def fused(previous, acceleration, dt):
    """previous + acceleration x dt rounded once to the nearest double, as libgz-math7's
    SpeedLimiter::LimitAcceleration computes it on osx-arm64 (compiled to a fused multiply-add, fmadd). Python 3.12
    has no math.fma: the exact rational sum, rounded once. With acceleration 0 the sum is previous, exactly."""
    if acceleration == 0.0:
        return previous + acceleration * dt
    return float(Fraction(previous) + Fraction(acceleration) * Fraction(dt))


class Ramp:
    """Each wheel's setpoint clamped to +-top_speed, then moved towards its target by at most accel [rad/s^2] (accel 0:
    no limit), as gz::math::SpeedLimiter's LimitVelocity and LimitAcceleration did in the drivetrain: the
    acceleration (target - previous) / dt is clamped, and the new setpoint is previous + acceleration x dt, rounded
    once (fused)."""

    def __init__(self, accel=SIM.wheel_cmd.accel, rover=ROVER):
        self.accel, self.top, self.rover = accel, rover.wheel.top_speed, rover
        self.values = (0.0,) * len(WHEELS)

    def step(self, vx, wz, dt=STEP):
        """The setpoints after a step of dt [s] towards the twist."""
        low, high = (-self.accel, self.accel) if self.accel > 0 else (-math.inf, math.inf)
        values = []
        for target, previous in zip(setpoints(vx, wz, self.rover), self.values):
            target = max(min(target, self.top), -self.top)
            values.append(fused(previous, max(min((target - previous) / dt, high), low), dt))
        self.values = tuple(values)
        return self.values

    def reset(self):
        self.values = (0.0,) * len(WHEELS)


def message(values):
    """The setpoints as the drivetrain's topic carries them."""
    from gz.msgs10.double_v_pb2 import Double_V  # here: the formula and the ramp need no Gazebo
    msg = Double_V()
    msg.data.extend(values)
    return msg


class Driver:
    """A twist schedule's setpoints for a run stepped in this process: command(vx, wz) when the twist changes
    (simulate sends one every 20 ms of sim time), step() once per physics step from the pre-update callback. step
    ramps (Ramp) and publishes the setpoints when they change, and with force (the 20 ms beat, so that the
    drivetrain's watchdog sees fresh ones), never before the first command. The drivetrain uses a setpoint published
    in a step from the next step on, as it used a twist. command also sends the twist as a gz.msgs.Twist on
    rover_config.CMD_VEL_TOPIC, which a rover with Gazebo's DiffDrive takes (sim.yaml drivetrain mode diffdrive) and
    the physical drivetrain ignores."""

    def __init__(self, node, accel=SIM.wheel_cmd.accel, rover=ROVER):
        from gz.msgs10.double_v_pb2 import Double_V
        from gz.msgs10.twist_pb2 import Twist
        self._wheels = node.advertise(WHEEL_CMD_TOPIC, Double_V)
        self._twists = node.advertise(CMD_VEL_TOPIC, Twist)
        self._twist = Twist()
        self.ramp = Ramp(accel, rover)
        self.twist = (0.0, 0.0)
        self._sent = None  # the setpoints last published; None: no command yet

    def command(self, vx, wz):
        self.twist = (vx, wz)
        self._twist.linear.x, self._twist.angular.z = vx, wz
        self._twists.publish(self._twist)
        if self._sent is None:
            self._sent = ()

    def step(self, force=False):
        if self._sent is None:
            return
        values = self.ramp.step(*self.twist)
        if force or values != self._sent:
            self._wheels.publish(message(values))
            self._sent = values

    def reset(self):
        """The world was reset: the ramp starts again from zero, and nothing goes out before the next command."""
        self.ramp.reset()
        self.twist = (0.0, 0.0)
        self._sent = None


class TwistDrive:
    """twist(vx, wz) as wheel setpoints without a ramp (setpoints), and as the twist for a DiffDrive rover: for a caller
    whose twists are ramped already, such as the driver station (station/drive.py slews its own), on a rover without
    ROS. The ROS stack takes the station's twist on /cmd_vel instead (station/ros_link.py)."""

    def __init__(self, node, rover=ROVER):
        from gz.msgs10.double_v_pb2 import Double_V
        from gz.msgs10.twist_pb2 import Twist
        self._wheels = node.advertise(WHEEL_CMD_TOPIC, Double_V)
        self._twists = node.advertise(CMD_VEL_TOPIC, Twist)
        self._twist = Twist()
        self.rover = rover

    def twist(self, vx, wz):
        self._twist.linear.x, self._twist.angular.z = vx, wz
        self._twists.publish(self._twist)
        self._wheels.publish(message(setpoints(vx, wz, self.rover)))
```

- [ ] **Step 4: Run them to see them pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_wheel_cmd test_rover_config test_rover_model.TrackedModel 2>&1 | tail -3
```

Expected: `Ran 23 tests` (11 + 8 + 4) and `OK`; the tracked model is unchanged (the xacro does not read the new keys yet).

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/wheel_cmd.py rover_sim/tests/test_wheel_cmd.py rover_sim/config/sim.yaml rover_sim/rover_config.py rover_sim/tests/test_rover_config.py && git commit -q -F - <<'EOF'
PR 3: rover_sim: wheel_cmd.py, the wheel setpoints the drivetrain made of a twist

The ROS-free way to drive the simulated rover once the drivetrain takes wheel setpoints (sim.yaml topics.wheel_cmd,
gz.msgs.Double_V, fl rl fr rr): setpoints() is the drivetrain's skid-steer formula, Ramp its gz::math::SpeedLimiter
arithmetic at sim.yaml's new wheel_cmd accel (8 rad/s^2), with previous + acceleration x dt rounded once as
libgz-math7 computes it on osx-arm64 (a fused multiply-add), Driver feeds a TestFixture run step by step (publishing on
a change and on the 20 ms beat, and the twist for a DiffDrive rover), TwistDrive sends an already-ramped twist as it
comes (the driver station). test_wheel_cmd checks them and that controllers.yaml ramps each wheel at the same rate.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 8: The tests, tools and station drive through wheel_cmd (the drivetrain still takes its twist)

**Files:** Modify `rover_sim/tests/simulate.py`, `rover_sim/tests/test_drivetrain.py`, `rover_sim/tests/test_render.py`, `rover_sim/tests/test_urc_terrain.py`, `rover_sim/tests/test_station.py`, `rover_sim/tests/test_fly_camera.py`, `rover_sim/station/link.py`.

Every place that published a twist on `/model/rover/cmd_vel` now goes through `wheel_cmd`, which publishes the same twists at the same steps (the drivetrain and DiffDrive still drive on them in this task) and the wheel setpoints the drivetrain will take in Task 9.

- [ ] **Step 1: simulate.py**

In `rover_sim/tests/simulate.py` replace

```python
Gazebo's Python TestFixture runs the server inside this process, so a run is
repeatable: commands go out over gz-transport every 20 ms of sim time, and
the state is read from the entity-component manager after the last step.
```

with

```python
Gazebo's Python TestFixture runs the server inside this process, so a run is
repeatable: the rover's command, a twist every 20 ms of sim time, goes out
over gz-transport as the wheel setpoints wheel_cmd.Driver makes of it step by
step (the physical drivetrain) and as the twist (Gazebo's DiffDrive), and the
state is read from the entity-component manager after the last step.
```

replace

```python
import rover_config  # noqa: E402
from rover_config import ROVER, Variant  # noqa: E402
from urc import sdf, terrain, terrains  # noqa: E402
```

with

```python
import rover_config  # noqa: E402
import wheel_cmd  # noqa: E402
from rover_config import ROVER, SIM, Variant  # noqa: E402
from urc import sdf, terrain, terrains  # noqa: E402
```

replace

```python
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
```

with

```python
from gz.msgs10.double_v_pb2 import Double_V  # noqa: E402
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
```

replace

```python
def simulate(seconds, extra="", spawn_z=0.02, cmd=(0.0, 0.0), subscribe=(), world=None, rover_uri=None, params=None,
             default_surface=None, solver=None, trace_every=0, publish_until=None):
    """Run for `seconds` of sim time and return the final State.

    cmd: (vx [m/s], wz [rad/s]) sent from the start, or a schedule
    [(t [s], vx, wz), ...] (twist_at), sent every 20 ms from the start and,
    with publish_until [s], only until then.
    subscribe: (topic, message class) pairs to record during the run.
    world: an SDF file to run instead of world_sdf(extra, spawn_z).
    rover_uri, params, default_surface, solver: see world_file.
    trace_every: record base_link every that many steps (State.trace).
    """
    with world_file(world, extra, spawn_z, rover_uri, params, default_surface, solver) as world_path:
        return _run(seconds, cmd, subscribe, world_path, trace_every, publish_until)


def _run(seconds, cmd, subscribe, world_path, trace_every=0, publish_until=None):
    node = Node()
    messages = {topic: [] for topic, _ in subscribe}
    for topic, msg_type in subscribe:
        node.subscribe(msg_type, topic, messages[topic].append)
    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
    scripted = bool(cmd) and isinstance(cmd[0], (tuple, list))
    twist = Twist()
    if not scripted:
        twist.linear.x, twist.angular.z = cmd
    handles = {}
```

with

```python
def simulate(seconds, extra="", spawn_z=0.02, cmd=(0.0, 0.0), subscribe=(), world=None, rover_uri=None, params=None,
             default_surface=None, solver=None, trace_every=0, publish_until=None, accel=None):
    """Run for `seconds` of sim time and return the final State.

    cmd: (vx [m/s], wz [rad/s]) sent from the start, or a schedule
    [(t [s], vx, wz), ...] (twist_at), sent every 20 ms from the start and,
    with publish_until [s], only until then: as the wheel setpoints
    wheel_cmd.Driver makes of it, step by step, and as the twist.
    subscribe: (topic, message class) pairs to record during the run.
    world: an SDF file to run instead of world_sdf(extra, spawn_z).
    rover_uri, params, default_surface, solver: see world_file.
    trace_every: record base_link every that many steps (State.trace).
    accel: the setpoints' ramp [rad/s^2]; default the wheel_cmd accel of
    `params`, else sim.yaml's (give it for a world built with a variant).
    """
    if accel is None:
        accel = (params.sim if params is not None else SIM).wheel_cmd.accel
    with world_file(world, extra, spawn_z, rover_uri, params, default_surface, solver) as world_path:
        return _run(seconds, cmd, subscribe, world_path, trace_every, publish_until, accel)


def _run(seconds, cmd, subscribe, world_path, trace_every=0, publish_until=None, accel=None):
    node = Node()
    messages = {topic: [] for topic, _ in subscribe}
    for topic, msg_type in subscribe:
        node.subscribe(msg_type, topic, messages[topic].append)
    driver = wheel_cmd.Driver(node, SIM.wheel_cmd.accel if accel is None else accel)
    scripted = bool(cmd) and isinstance(cmd[0], (tuple, list))
    handles = {}
```

replace

```python
        if publish_until is not None and info.iterations / 1000 > publish_until:
            return
        if info.iterations % 20 == 0:
            if scripted:
                twist.linear.x, twist.angular.z = twist_at(cmd, info.iterations / 1000)
                publisher.publish(twist)
            elif any(cmd):
                publisher.publish(twist)
```

with

```python
        if publish_until is not None and info.iterations / 1000 > publish_until:
            return
        beat = info.iterations % 20 == 0
        if beat and (scripted or any(cmd)):
            driver.command(*(twist_at(cmd, info.iterations / 1000) if scripted else cmd))
        driver.step(force=beat)
```

replace

```python
def follow(world, path, seconds, speed=0.8, tolerance=1.0, **driver):
    """Drive the rover in the SDF file `world` along a world (x, y) polyline
    with a pure-pursuit driver (pursue) on its ground truth, a command every
    20 ms of sim time, until it is within `tolerance` of the path's end
    (then it stops) or `seconds` of sim time have passed."""
    node = Node()
    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
    goal = tuple(path[-1])
    handles, trace, arrived = {}, [], []
    twist = Twist()

    def pre_update(info, ecm):
        if not handles:
            model = Model(World(world_entity(ecm)).model_by_name(ecm, "rover"))
            handles["base"] = Link(model.link_by_name(ecm, "base_link"))
        if info.iterations % 20:
            return
        x, y, _, _, _, yaw = _xyzrpy(handles["base"].world_pose(ecm))
        if not arrived and math.dist((x, y), goal) < tolerance:
            arrived.append(info.iterations / 1000)
        twist.linear.x, twist.angular.z = (0.0, 0.0) if arrived else pursue(path, x, y, yaw, speed, **driver)
        publisher.publish(twist)
```

with

```python
def follow(world, path, seconds, speed=0.8, tolerance=1.0, accel=None, **driver):
    """Drive the rover in the SDF file `world` along a world (x, y) polyline
    with a pure-pursuit driver (pursue) on its ground truth, a command every
    20 ms of sim time (wheel_cmd.Driver, ramped at accel, default sim.yaml's),
    until it is within `tolerance` of the path's end (then it stops) or
    `seconds` of sim time have passed."""
    node = Node()
    wheels = wheel_cmd.Driver(node, SIM.wheel_cmd.accel if accel is None else accel)
    goal = tuple(path[-1])
    handles, trace, arrived = {}, [], []

    def pre_update(info, ecm):
        if not handles:
            model = Model(World(world_entity(ecm)).model_by_name(ecm, "rover"))
            handles["base"] = Link(model.link_by_name(ecm, "base_link"))
        beat = info.iterations % 20 == 0
        if beat:
            x, y, _, _, _, yaw = _xyzrpy(handles["base"].world_pose(ecm))
            if not arrived and math.dist((x, y), goal) < tolerance:
                arrived.append(info.iterations / 1000)
            wheels.command(*((0.0, 0.0) if arrived else pursue(path, x, y, yaw, speed, **driver)))
        wheels.step(force=beat)
```

and replace

```python
    """Drive the rover from a thread of this process: twist_at(schedule, sim
    time) on its cmd_vel topic `rate` times a second, while any of the named
    worlds runs in this partition. Sim time comes from the worlds' statistics
    (10 Hz): /clock, sent every step, would add to the cost being measured.
    Nothing is sent before the first statistics message."""
    node = Node()
    now = []

    def on_stats(msg):
        now[:] = [msg.sim_time.sec + msg.sim_time.nsec * 1e-9]

    for name in worlds:
        node.subscribe(WorldStatistics, f"/world/{name}/stats", on_stats)
    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
    stop = threading.Event()

    def loop():
        twist = Twist()
        while not stop.wait(1.0 / rate):
            if now:
                twist.linear.x, twist.angular.z = twist_at(schedule, now[0])
                publisher.publish(twist)
```

with

```python
    """Drive the rover from a thread of this process: twist_at(schedule, sim
    time) `rate` times a second while any of the named worlds runs in this
    partition, as the twist (Gazebo's DiffDrive) and as wheel setpoints ramped
    over the sim time since the last (wheel_cmd.Ramp; the physical drivetrain).
    Sim time comes from the worlds' statistics (10 Hz): /clock, sent every
    step, would add to the cost being measured. Nothing is sent before the
    first statistics message."""
    node = Node()
    now = []

    def on_stats(msg):
        now[:] = [msg.sim_time.sec + msg.sim_time.nsec * 1e-9]

    for name in worlds:
        node.subscribe(WorldStatistics, f"/world/{name}/stats", on_stats)
    twists = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
    wheels = node.advertise(rover_config.WHEEL_CMD_TOPIC, Double_V)
    stop = threading.Event()

    def loop():
        twist, ramp, last = Twist(), wheel_cmd.Ramp(), None
        while not stop.wait(1.0 / rate):
            if now:
                t = now[0]
                twist.linear.x, twist.angular.z = twist_at(schedule, t)
                twists.publish(twist)
                dt = t - last if last is not None and t > last else wheel_cmd.STEP
                wheels.publish(wheel_cmd.message(ramp.step(twist.linear.x, twist.angular.z, dt)))
                last = t
```

- [ ] **Step 2: test_drivetrain's own drives**

In `rover_sim/tests/test_drivetrain.py` replace

```python
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
```

with nothing (delete the line), replace

```python
import rover_config  # noqa: E402
from rover_config import ROVER, SIM  # noqa: E402
```

with

```python
import rover_config  # noqa: E402
import wheel_cmd  # noqa: E402
from rover_config import ROVER, SIM  # noqa: E402
```

replace

```python
    raster = everywhere(ground, hf.n) if raster is None else raster
    with ground_world(hf, raster, rows, rover, lift=lift, ground_options=OPTIONS, terrain_extra=terrain_extra,
                      extra=extra, params=params or physical(contact={"dig": False}), solver=solver,
                      gravity=gravity) as world:
        s = simulate(seconds, world=world, cmd=cmd, trace_every=trace_every, publish_until=publish_until,
                     subscribe=[(rover_config.DRIVETRAIN_TOPIC, StringMsg), *subscribe])
```

with

```python
    raster = everywhere(ground, hf.n) if raster is None else raster
    params = params or physical(contact={"dig": False})
    with ground_world(hf, raster, rows, rover, lift=lift, ground_options=OPTIONS, terrain_extra=terrain_extra,
                      extra=extra, params=params, solver=solver, gravity=gravity) as world:
        s = simulate(seconds, world=world, cmd=cmd, trace_every=trace_every, publish_until=publish_until,
                     subscribe=[(rover_config.DRIVETRAIN_TOPIC, StringMsg), *subscribe],
                     accel=params.sim.wheel_cmd.accel)
```

and replace

```python
            fixture = TestFixture(world)
            node = Node()
            publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
            twist = Twist()
            twist.linear.x = 0.5
            commanding = [True]
            fixture.on_pre_update(lambda info, ecm: commanding[0] and info.iterations % 20 == 0
                                  and publisher.publish(twist))
            fixture.finalize()
```

with

```python
            fixture = TestFixture(world)
            node = Node()
            driver = wheel_cmd.Driver(node)
            commanding = [True]

            def drive(info, ecm):
                if commanding[0]:
                    beat = info.iterations % 20 == 0
                    if beat:
                        driver.command(0.5, 0.0)
                    driver.step(force=beat)

            fixture.on_pre_update(drive)
            fixture.finalize()
```

- [ ] **Step 3: The render tests' subprocesses**

In `rover_sim/tests/test_render.py` replace

```python
import rover_config  # noqa: E402
import viewers  # noqa: E402
from rover_config import ROVER, Variant  # noqa: E402
```

with

```python
import rover_config  # noqa: E402
import viewers  # noqa: E402
import wheel_cmd  # noqa: E402
from rover_config import ROVER, Variant  # noqa: E402
```

In `_drive_main` replace

```python
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.twist_pb2 import Twist
    from gz.msgs10.world_control_pb2 import WorldControl
```

with

```python
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.world_control_pb2 import WorldControl
```

and replace

```python
    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
    twist, handles, trace = Twist(), {}, []

    def pre_update(info, ecm):
        if not handles:
            handles["base"] = Link(Model(World(world_entity(ecm)).model_by_name(ecm, "rover")).link_by_name(
                ecm, "base_link"))
        if info.iterations % 20 == 0:
            vx = wz = 0.0
            for start, v, w in cfg["schedule"]:
                if info.iterations / 1000 >= start:
                    vx, wz = v, w
            twist.linear.x, twist.angular.z = vx, wz
            publisher.publish(twist)
```

with

```python
    driver = wheel_cmd.Driver(node)
    handles, trace, driven = {}, [], [0]  # driven: the phase the driver last drove in

    def pre_update(info, ecm):
        if not handles:
            handles["base"] = Link(Model(World(world_entity(ecm)).model_by_name(ecm, "rover")).link_by_name(
                ecm, "base_link"))
        if driven[0] != phase[0]:  # the world was reset: the ramp starts from zero again, as the drivetrain's did
            driven[0] = phase[0]
            driver.reset()
        beat = info.iterations % 20 == 0
        if beat:
            vx = wz = 0.0
            for start, v, w in cfg["schedule"]:
                if info.iterations / 1000 >= start:
                    vx, wz = v, w
            driver.command(vx, wz)
        driver.step(force=beat)
```

In `_render_main` replace

```python
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.twist_pb2 import Twist
    from gz.sim8 import TestFixture
```

with

```python
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.sim8 import TestFixture
```

and replace

```python
    if commands:
        publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
        twist = Twist()

        def pre_update(info, ecm):
            k, due = divmod(info.iterations, COMMAND_STEPS)
            if driving[0] and not due and k < len(commands):
                twist.linear.x, twist.angular.z = commands[k]
                publisher.publish(twist)

        fixture.on_pre_update(pre_update)
```

with

```python
    if commands:
        driver = wheel_cmd.Driver(node)

        def pre_update(info, ecm):
            k, due = divmod(info.iterations, COMMAND_STEPS)
            if not driving[0] or k >= len(commands):
                return
            if not due:
                driver.command(*commands[k])
            driver.step(force=not due)

        fixture.on_pre_update(pre_update)
```

- [ ] **Step 4: test_urc_terrain, test_station, test_fly_camera**

In `rover_sim/tests/test_urc_terrain.py` replace

```python
from rover_config import ROVER  # noqa: E402
```

with

```python
import wheel_cmd  # noqa: E402
from rover_config import ROVER  # noqa: E402
```

replace

```python
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
```

with nothing (delete the line), and replace

```python
    node = Node()
    publisher = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
    twist = Twist()
    rows, links = [], {}

    def pre(info, ecm):
        if not links:
            rover = Model(World(world_entity(ecm)).model_by_name(ecm, "rover"))
            links.update({name: Link(rover.link_by_name(ecm, name)) for name in WHEEL_LINKS})
        if info.iterations % 20 == 0:
            twist.linear.x = speed if info.iterations >= 1000 else 0.0
            publisher.publish(twist)
```

with

```python
    node = Node()
    driver = wheel_cmd.Driver(node)
    rows, links = [], {}

    def pre(info, ecm):
        if not links:
            rover = Model(World(world_entity(ecm)).model_by_name(ecm, "rover"))
            links.update({name: Link(rover.link_by_name(ecm, name)) for name in WHEEL_LINKS})
        beat = info.iterations % 20 == 0
        if beat:
            driver.command(speed if info.iterations >= 1000 else 0.0, 0.0)
        driver.step(force=beat)
```

In `rover_sim/tests/test_station.py` replace

```python
import viewers  # noqa: E402
from rover_config import ROVER, SIM  # noqa: E402
```

with

```python
import viewers  # noqa: E402
import wheel_cmd  # noqa: E402
from rover_config import ROVER, SIM  # noqa: E402
```

replace

```python
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
```

with nothing (delete the line), replace

```python
        node = Node()
        twist_pub = node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
        cmd_pub = node.advertise(viewers.CHASE_CMD_TOPIC, Vector3d)
        mode_pub = node.advertise(viewers.CHASE_MODE_TOPIC, StringMsg)
        twist = Twist()
        twist.linear.x, twist.angular.z = 0.6, 0.35
        zoom = math.log(1.5)
```

with

```python
        node = Node()
        driver = wheel_cmd.Driver(node)
        cmd_pub = node.advertise(viewers.CHASE_CMD_TOPIC, Vector3d)
        mode_pub = node.advertise(viewers.CHASE_MODE_TOPIC, StringMsg)
        zoom = math.log(1.5)
```

and replace

```python
            if info.iterations % 20 == 0:
                twist_pub.publish(twist)
            t = info.iterations / 1000
```

with

```python
            beat = info.iterations % 20 == 0
            if beat:
                driver.command(0.6, 0.35)
            driver.step(force=beat)
            t = info.iterations / 1000
```

In `rover_sim/tests/test_fly_camera.py` replace

```python
import rover_config  # noqa: E402  (worldfiles puts sim/ on the path)
```

with

```python
import wheel_cmd  # noqa: E402  (worldfiles puts sim/ on the path)
```

replace

```python
def fly(world, seconds, events=(), holds=(), track=(), sleeps=(), fixture_runs=None):
    """Run `world` (SDF text) for `seconds` of sim time in this process, then
    fixture_runs(fixture) if given. events: (t, topic, message) published
    once at sim time t; holds: (t0, t1, topic, message) published every 20 ms
    from t0 until before t1; neither after a world reset. track: other models
```

with

```python
def fly(world, seconds, events=(), holds=(), track=(), sleeps=(), fixture_runs=None, drive=()):
    """Run `world` (SDF text) for `seconds` of sim time in this process, then
    fixture_runs(fixture) if given. events: (t, topic, message) published
    once at sim time t; holds: (t0, t1, topic, message) published every 20 ms
    from t0 until before t1; drive: (t0, t1, vx, wz), the rover's twist from t0
    until before t1, as wheel_cmd.Driver's setpoints; none of them after a
    world reset. track: other models
```

replace

```python
    states = []
    node.subscribe(StringMsg, viewers.FLY_STATE_TOPIC, lambda m: states.append(json.loads(m.data)))
```

with

```python
    driver = wheel_cmd.Driver(node)
    states = []
    node.subscribe(StringMsg, viewers.FLY_STATE_TOPIC, lambda m: states.append(json.loads(m.data)))
```

replace

```python
        if info.iterations % 20 == 0:
            for t0, t1, topic, message in holds:
                if t0 - 5e-4 < t < t1 - 5e-4:
                    publishers[topic].publish(message)
```

with

```python
        beat = info.iterations % 20 == 0
        if beat:
            for t0, t1, topic, message in holds:
                if t0 - 5e-4 < t < t1 - 5e-4:
                    publishers[topic].publish(message)
        for t0, t1, vx, wz in drive:
            if t0 - 5e-4 < t < t1 - 5e-4:
                if beat:
                    driver.command(vx, wz)
                driver.step(force=beat)
```

and replace

```python
                holds=[(0.0, 9.0, rover_config.CMD_VEL_TOPIC, twist(0.6))], track=["rover"])
```

with

```python
                drive=[(0.0, 9.0, 0.6, 0.0)], track=["rover"])
```

- [ ] **Step 5: The station's link drives through an output it is given**

In `rover_sim/station/link.py` replace

```python
import rover_config
import viewers
```

with

```python
import rover_config
import viewers
import wheel_cmd
```

replace

```python
class GzLink:
    def __init__(self, world, node=None):
        self.world = world
        self.node = node or Node()
```

with

```python
class GzLink:
    """drive: where twist() goes, an object with twist(vx, wz): the station's main gives it the ROS stack's /cmd_vel
    (station/ros_link.py); by default, and in the tests, wheel_cmd.TwistDrive on the drivetrain's ROS-free topic."""

    def __init__(self, world, node=None, drive=None):
        self.world = world
        self.node = node or Node()
```

replace

```python
        self._twist = self.node.advertise(rover_config.CMD_VEL_TOPIC, Twist)
        self._led = self.node.advertise(rover_config.LED_TOPIC, StringMsg)
```

with

```python
        self.drive = drive or wheel_cmd.TwistDrive(self.node)
        self._led = self.node.advertise(rover_config.LED_TOPIC, StringMsg)
```

and replace

```python
    def twist(self, vx, wz):
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        self._twist.publish(msg)
```

with

```python
    def twist(self, vx, wz):
        self.drive.twist(vx, wz)
```

- [ ] **Step 6: The changed tests pass, and the traces are still Task 1's**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_wheel_cmd test_foundations test_rover_sim test_drivetrain.Response test_drivetrain.Interfaces test_station.ChaseCameraFlight test_station.EyeCameraRide test_station.DrivetrainReadout test_station.GzLinkState test_fly_camera.Flight test_urc_terrain.ProvingGroundPhysics test_equivalence.Drives > $T/task8.log 2>&1; tail -3 $T/task8.log
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run python $T/traces.py $T/traces_task8.npz > /dev/null 2>&1
/Users/alarion239/.pixi/bin/pixi run python -c "
import numpy as np
a, b = np.load('$T/traces_before.npz'), np.load('$T/traces_task8.npz')
print({k: bool(a[k].shape == b[k].shape and np.array_equal(a[k], b[k])) for k in a.files})"
```

Expected: `OK` (some skips where a world is not here); every trace `True` (`Interfaces` holds `test_state_message`, `test_command_timeout` and `test_reset`; `Flight` holds `test_follow_keeps_the_offset`). Then run the render tests that drive (they need the GPU; about 5 minutes):

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_render.DigCues test_render.Ruts > $T/task8_render.log 2>&1; tail -3 $T/task8_render.log
```

Expected: `OK` (test_render's known expected failures stay expected failures).

- [ ] **Step 7: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/tests rover_sim/station/link.py && git commit -q -F - <<'EOF'
PR 3: the tests, tools and station drive through wheel_cmd

simulate's runs, follow() and twist_publisher, test_drivetrain's reset, the render tests' subprocesses, the proving
ground's wheel poses, the chase and fly camera tests and the station's GzLink send the rover's twist through
wheel_cmd: the same twists at the same steps (the drivetrain and DiffDrive still drive on them) and the wheel
setpoints the drivetrain will take. simulate.simulate takes the setpoints' ramp from its variant, or as accel for a
world built with one. GzLink takes its drive output as an argument (by default wheel_cmd.TwistDrive). Task 1's
traces are unchanged bit for bit.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 9: The drivetrain takes only wheel setpoints

**Files:** Modify `rover_sim/plugins/rover_drivetrain.cpp`, `rover_sim/config/sim.yaml`, `rover_sim/urdf/rover_sim.urdf.xacro`, `rover_sim/rover_config.py`, `rover_sim/models/rover/model.sdf` (regenerated), `rover_sim/tests/test_rover_model.py`, `rover_sim/tests/test_equivalence.py`, `rover_sim/tests/fixtures/rover_model/README.md`, `rover_sim/tests/test_drivetrain.py`, `rover_sim/tests/test_ros2_control.py`.

- [ ] **Step 1: The failing tests**

In `rover_sim/tests/test_rover_model.py` replace

```python
        expect = {"topic": rover_config.CMD_VEL_TOPIC, "odom_topic": rover_config.ODOM_TOPIC,
```

with

```python
        expect = {"setpoint_topic": rover_config.WHEEL_CMD_TOPIC, "odom_topic": rover_config.ODOM_TOPIC,
```

and replace

```python
        numbers = {"cmd_timeout": 0.5, "track": 2 * ROVER.rocker.pivot_y, "radius": ROVER.wheel.radius,
                   "track_multiplier": 1.0, "motor/gear": 50.0, "motor/efficiency": 0.8,
                   "motor/current_limit": 20.0, "motor/voltage": 48.0, "driveline/backlash": ROVER.driveline.backlash,
                   "controller/kp": 4.0, "controller/ki": 160.0, "controller/accel": SIM.drivetrain.accel,
                   "controller/max_speed": ROVER.wheel.top_speed, "contact/v_stribeck": SIM.contact.v_stribeck,
                   "contact/stick_perp_ratio": 0.3, "contact/perp_ratio": 0.0}
        for tag, value in numbers.items():
            self.assertAlmostEqual(float(self.plugin.findtext(tag)), value, msg=tag)
```

with

```python
        numbers = {"cmd_timeout": 0.5, "track": 2 * ROVER.rocker.pivot_y, "radius": ROVER.wheel.radius,
                   "motor/gear": 50.0, "motor/efficiency": 0.8,
                   "motor/current_limit": 20.0, "motor/voltage": 48.0, "driveline/backlash": ROVER.driveline.backlash,
                   "controller/kp": 4.0, "controller/ki": 160.0,
                   "controller/max_speed": ROVER.wheel.top_speed, "contact/v_stribeck": SIM.contact.v_stribeck,
                   "contact/stick_perp_ratio": 0.3, "contact/perp_ratio": 0.0}
        for tag, value in numbers.items():
            self.assertAlmostEqual(float(self.plugin.findtext(tag)), value, msg=tag)
        for gone in ("topic", "track_multiplier", "controller/accel"):  # the drive controller's since PR 3
            self.assertIsNone(self.plugin.find(gone), gone)
```

In `rover_sim/tests/test_equivalence.py` replace

```python
section 5): tools/gen_rover.py's model of the description, and of each variant the tests build, against the frozen
models in fixtures/rover_model (written by gen_model.py at import/simulation, 9208047; fixtures/rover_model/README.md
lists them). SdfEquivalence checks the comparison itself; Drives runs the frozen and the generated rover side by
side in Gazebo. When the rover changes on purpose, refreeze the fixtures from gen_rover.py in the same commit and say
why (fixtures/rover_model/README.md)."""
```

with

```python
section 5): tools/gen_rover.py's model of the description, and of each variant the tests build, against the frozen
models in fixtures/rover_model (written by gen_model.py at import/simulation, 9208047; fixtures/rover_model/README.md
lists them), after ros2_control_boundary(), the one change since that is not physical. SdfEquivalence checks the
comparison itself; Drives runs the frozen and the generated rover side by side in Gazebo. When the rover changes on
purpose, refreeze the fixtures from gen_rover.py in the same commit and say why (fixtures/rover_model/README.md)."""
```

replace

```python
import gen_rover  # noqa: E402  (worldfiles puts sim/ and sim/tools on the path)
from rover_config import Variant  # noqa: E402
from urc import sdf  # noqa: E402
```

with

```python
import gen_rover  # noqa: E402  (worldfiles puts sim/ and sim/tools on the path)
import rover_config  # noqa: E402
from rover_config import Variant  # noqa: E402
from urc import sdf  # noqa: E402
```

replace

```python
        drivetrain={"cmd_timeout_clock": "sim", "accel": 0.0, "state_rate": 1000.0, "cmd_timeout": 0.0},
```

with

```python
        drivetrain={"cmd_timeout_clock": "sim", "state_rate": 1000.0, "cmd_timeout": 0.0},
```

replace

```python
def fixture(name):
    return (FIXTURES / f"{name}.sdf").read_text()
```

with

```python
def fixture(name):
    return (FIXTURES / f"{name}.sdf").read_text()


def ros2_control_boundary(text):
    """A frozen model as the generator writes it since the ros2_control boundary (design spec 2026-10-09, section 6.1;
    PR 3): RoverDrivetrain no longer turns a twist into wheel setpoints, so its twist topic, track multiplier and ramp
    (<topic>, <track_multiplier>, <controller><accel>) are gone, and it takes setpoints on <setpoint_topic>. Nothing
    physical changes; a DiffDrive model is left as it is."""
    root = ET.fromstring(text)
    for plugin in root.iter("plugin"):
        if plugin.get("filename") != "RoverDrivetrain":
            continue
        for parent, tag in ((plugin, "topic"), (plugin, "track_multiplier"), (plugin.find("controller"), "accel")):
            parent.remove(parent.find(tag))
        sdf.sub(plugin, "setpoint_topic", rover_config.WHEEL_CMD_TOPIC)
    return sdf.document(root)
```

replace

```python
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
```

with

```python
    def test_the_generated_rovers_are_the_frozen_ones(self):
        self.assertEqual(sorted(p.stem for p in FIXTURES.glob("*.sdf")), sorted(VARIANTS))
        for name, variant in VARIANTS.items():
            with self.subTest(name):
                self.assertEqual(differences(ros2_control_boundary(fixture(name)), gen_rover.build_sdf(variant)), [])

    def test_the_rover_is_the_tracked_model(self):
        """The tracked model://rover is the default rover (the comparison, not the bytes: gen_rover.py lays it out
        as the converter does)."""
        tracked = (SIM_DIR / "models" / "rover" / "model.sdf").read_text()
        self.assertEqual(differences(ros2_control_boundary(fixture("default")), tracked), [])

    def test_the_boundary_changes_only_the_drives_command_interface(self):
        """ros2_control_boundary changes four elements of a physical rover's drivetrain and nothing of a DiffDrive
        rover's."""
        self.assertEqual(len(differences(fixture("default"), ros2_control_boundary(fixture("default")))), 4)
        self.assertEqual(differences(fixture("diffdrive"), ros2_control_boundary(fixture("diffdrive"))), [])
```

In `rover_sim/tests/test_drivetrain.py` replace

```python
    def test_with_the_ramp(self):
        """rise = setpoint / accel +- 20 %, 0.33 s at wz 1 (plumbing: the ramp
        is the controller's, an assumption, design spec 6.3)."""
        rise, _ = self.times(physical(contact={"dig": False}))
        setpoint = 1.0 * C / ROVER.wheel.radius  # [rad/s] each wheel
        self.assertAlmostEqual(rise, setpoint / SIM.drivetrain.accel, delta=0.2 * setpoint / SIM.drivetrain.accel)

    def test_stop_without_the_ramp(self):
        """The motor, contact and tyre model alone stop the turn in < 40 ms (physics)."""
        self.assertLess(self.times(physical(contact={"dig": False}, drivetrain={"accel": 0.0}))[1], 0.040)
```

with

```python
    def test_with_the_ramp(self):
        """rise = setpoint / accel +- 20 %, 0.33 s at wz 1 (plumbing: the ramp
        is the setpoints' helper's, sim.yaml wheel_cmd accel, as the drive
        controller's; an assumption, design spec 6.3)."""
        rise, _ = self.times(physical(contact={"dig": False}))
        setpoint = 1.0 * C / ROVER.wheel.radius  # [rad/s] each wheel
        self.assertAlmostEqual(rise, setpoint / SIM.wheel_cmd.accel, delta=0.2 * setpoint / SIM.wheel_cmd.accel)

    def test_stop_without_the_ramp(self):
        """The motor, contact and tyre model alone stop the turn in < 40 ms (physics)."""
        self.assertLess(self.times(physical(contact={"dig": False}, wheel_cmd={"accel": 0.0}))[1], 0.040)
```

replace

```python
        self.assertLess(self.times(physical(contact={"dig": False}, drivetrain={"accel": 0.0}))[0], 0.060)
```

with

```python
        self.assertLess(self.times(physical(contact={"dig": False}, wheel_cmd={"accel": 0.0}))[0], 0.060)
```

replace

```python
        params = physical(contact={"dig": False}, drivetrain={"accel": 4.0},
```

with

```python
        params = physical(contact={"dig": False}, wheel_cmd={"accel": 4.0},
```

replace

```python
        (plumbing; with track_multiplier 1, as a real rover's odometry, design spec 6.8)."""
```

with

```python
        (plumbing; the drivetrain's no-slip odometry with the geometric track, as a real rover's, design spec 6.8)."""
```

replace

```python
    def test_command_timeout(self):
        """A command older than cmd_timeout counts as zero (0.5 s, the user's
        decision); cmd_timeout 0 holds the last command, as Gazebo's GUI Teleop
        needs. Sim clock here; the default clock is wall time."""
```

with

```python
    def test_command_timeout(self):
        """A setpoint older than cmd_timeout counts as zero (0.5 s, the user's
        decision); cmd_timeout 0 holds the last one. Sim clock here; the
        default clock is wall time. Watchdog has it to the step."""
```

replace

```python
        self.assertLess(stops.state.poses["base_link"][0], 0.75)  # 1.0 s + 0.5 s of the last command, the ramps
```

with

```python
        self.assertLess(stops.state.poses["base_link"][0], 0.75)  # 1.0 s + 0.5 s of the last setpoint, the ramp up
```

and insert before the line `@contextlib.contextmanager` that precedes `def captured_stderr(lines):`:

```python
class Watchdog(unittest.TestCase):
    """The motor controller's watchdog (sim.yaml drivetrain cmd_timeout, 0.5 s; design spec 2026-10-09, 6.1): a wheel
    setpoint older than cmd_timeout counts as zero, at once; cmd_timeout 0 holds the last one. Setpoints come from the
    ROS-free topic (simulate's wheel_cmd.Driver) until 1.0 s, the last in the step that ends at 1.000 s; the
    drivetrain's state of every step (sim clock) holds them through 1.500 s and has zero from 1.501 s."""

    @staticmethod
    def states(params):
        s = simulate(2.0, cmd=(0.5, 0.0), publish_until=1.0, params=params,
                     subscribe=[(rover_config.DRIVETRAIN_TOPIC, StringMsg)])
        states = [json.loads(m.data) for m in s.messages[rover_config.DRIVETRAIN_TOPIC]]
        return {round(state["t"] * 1000): state for state in states}

    def test_a_stale_setpoint_is_zero_at_once(self):
        states = self.states(physical(drivetrain={"state_rate": 1000.0}))
        for w in WHEELS:
            self.assertAlmostEqual(states[1500]["wheels"][w]["sp"], 0.5 / ROVER.wheel.radius, delta=1e-4)
            self.assertEqual(states[1501]["wheels"][w]["sp"], 0.0)
        self.assertEqual(states[1501]["cmd"], [0.0, 0.0])

    def test_cmd_timeout_zero_holds_the_last_setpoint(self):
        states = self.states(physical(drivetrain={"state_rate": 1000.0, "cmd_timeout": 0.0}))
        for w in WHEELS:
            self.assertAlmostEqual(states[1999]["wheels"][w]["sp"], 0.5 / ROVER.wheel.radius, delta=1e-4)


```

In `rover_sim/tests/test_ros2_control.py` replace

```python
import rosstack
from simulate import physical, world_sdf

import gen_rover  # noqa: E402  (worldfiles, through simulate, puts sim/ and sim/tools on the path)
import rover_config  # noqa: E402
```

with

```python
import rosstack
from simulate import physical, world_sdf

import gen_rover  # noqa: E402  (worldfiles, through simulate, puts sim/ and sim/tools on the path)
import rover_config  # noqa: E402
import wheel_cmd  # noqa: E402
```

replace

```python
from geometry_msgs.msg import TwistStamped  # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402
```

with

```python
from geometry_msgs.msg import TwistStamped  # noqa: E402
from gz.msgs10.double_v_pb2 import Double_V  # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg  # noqa: E402
```

replace

```python
        node.subscribe(StringMsg, rover_config.DRIVETRAIN_TOPIC, lambda m: cls.states.append(json.loads(m.data)))
        logs = [tmp / name for name in ("gz.log", "rsp.log", "bridge.log", "spawner.log")]
```

with

```python
        node.subscribe(StringMsg, rover_config.DRIVETRAIN_TOPIC, lambda m: cls.states.append(json.loads(m.data)))
        cls.wheel_cmd = node.advertise(rover_config.WHEEL_CMD_TOPIC, Double_V)  # early: it connects meanwhile
        logs = [tmp / name for name in ("gz.log", "rsp.log", "bridge.log", "spawner.log")]
```

replace

```python
    @classmethod
    def drive(cls, probe):
        probe.drive_for(*DRIVE, sim_seconds=3.0)
        probe.drive_for(0.0, 0.0, sim_seconds=1.5)
```

with

```python
    @classmethod
    def drive(cls, probe):
        probe.drive_for(*DRIVE, sim_seconds=1.5)
        for _ in range(3):  # the ROS-free setpoint topic, while ros2_control drives the wheels
            cls.wheel_cmd.publish(wheel_cmd.message((5.0,) * 4))
            probe.drive_for(*DRIVE, sim_seconds=0.1)
        probe.drive_for(*DRIVE, sim_seconds=1.2)
        probe.drive_for(0.0, 0.0, sim_seconds=1.5)
```

and insert before `if __name__ == "__main__":`:

```python
    def test_the_setpoint_topic_is_ignored(self):
        """While ros2_control drives the wheels RoverDrivetrain ignores its ROS-free setpoint topic, and says so once:
        the 5 rad/s sent there in the middle of the drive reach no motor."""
        self.assertEqual(self.log.count("is ignored: ros2_control's RoverSimSystem drives the wheels"), 1)
        self.assertEqual([s["t"] for s in self.states for w in WHEELS if abs(s["wheels"][w]["sp"] - 5.0) < 0.05], [])


```

- [ ] **Step 2: Run them to see them fail**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_rover_model.PhysicalVariant test_equivalence.Equivalence test_drivetrain.Watchdog test_ros2_control > $T/task9_red.log 2>&1; grep -E "^(FAIL|ERROR):|^Ran|^FAILED" $T/task9_red.log
```

Expected: `FAILED`, with `test_drivetrain_replaces_diffdrive` (no `setpoint_topic`), the two comparisons in `Equivalence` (the generated rover still has the twist interface; the new boundary test passes), both `Watchdog` tests (the drivetrain still takes its twist and ramps down after its timeout) and `test_the_setpoint_topic_is_ignored` failing.

- [ ] **Step 3: The drivetrain's setpoint input**

Edits in `rover_sim/plugins/rover_drivetrain.cpp`. Replace

```cpp
// - each wheel's speed setpoint: its joint's WheelSetpoint component (rover_components.hh) when it has one, which
//   ros2_control's RoverSimSystem (ros2_control/rover_sim_system.cpp) writes, clamped to <controller><max_speed> and
//   zero once older than <cmd_timeout> (the motor controller's watchdog); otherwise cmd_vel (vx, wz) ->
//   (vx -+ wz track track_multiplier / 2) / radius, left -, right +, through a ramp (gz::math::SpeedLimiter), a
//   command older than <cmd_timeout> counting as zero. Each motor's current goes into its joint's WheelMotorCurrent.
```

with

```cpp
// - each wheel's speed setpoint from its joint's WheelSetpoint component (rover_components.hh): ros2_control's
//   RoverSimSystem (ros2_control/rover_sim_system.cpp) writes it at the start of every step, or this plugin from the
//   newest message of its ROS-free topic <setpoint_topic> (gz.msgs.Double_V, one value per <wheel> in their order,
//   rad/s; ignored, saying so once, while ros2_control drives the wheels). A setpoint older than <cmd_timeout>
//   counts as zero (the motor controller's watchdog), and every one is clamped to <controller><max_speed>. The
//   twist-to-wheel kinematics and the ramp are the drive controller's (rover_control/config/controllers.yaml), or
//   wheel_cmd.py's for the ROS-free tools (design spec 2026-10-09, 6.1). Each motor's current goes into its joint's
//   WheelMotorCurrent.
```

replace

```cpp
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
```

with

```cpp
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
```

replace

```cpp
#include <gz/math/Pose3.hh>
#include <gz/math/SpeedLimiter.hh>
```

with

```cpp
#include <gz/math/Pose3.hh>
```

replace

```cpp
  gz::math::Quaterniond joint_frame;  // the joint's frame in the wheel's
  gz::math::SpeedLimiter ramp;
  double setpoint = 0.0;  // [rad/s] after the ramp
```

with

```cpp
  gz::math::Quaterniond joint_frame;  // the joint's frame in the wheel's
  double setpoint = 0.0;  // [rad/s] the motor follows this step: the fresh WheelSetpoint, clamped (0 when stale)
```

replace

```cpp
    track_ = get("track", track_) * get("track_multiplier", 1.0);
```

with

```cpp
    track_ = get("track", track_);  // the odometry's and the state's "cmd"
```

replace

```cpp
    accel_ = Child(sdf, "controller", "accel", 8.0);
    max_speed_ = Child(sdf, "controller", "max_speed", 10.0);
```

with

```cpp
    max_speed_ = Child(sdf, "controller", "max_speed", 10.0);
```

replace

```cpp
      w.ramp.SetMinVelocity(-max_speed_);
      w.ramp.SetMaxVelocity(max_speed_);
      if (accel_ > 0) {
        w.ramp.SetMinAcceleration(-accel_);
        w.ramp.SetMaxAcceleration(accel_);
      }
      if (const auto inertial = ecm.Component<gz::sim::components::Inertial>(w.link.Entity())) {
```

with

```cpp
      if (const auto inertial = ecm.Component<gz::sim::components::Inertial>(w.link.Entity())) {
```

replace

```cpp
    node_.Subscribe(text("topic", "/model/rover/cmd_vel"), &RoverDrivetrain::OnCmd, this);
```

with

```cpp
    setpoint_topic_ = text("setpoint_topic", "/model/rover/wheel_cmd");
    node_.Subscribe(setpoint_topic_, &RoverDrivetrain::OnSetpoints, this);
```

replace

```cpp
    double vx, wz, age;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sim_time_ = t;
      vx = cmd_[0];
      wz = cmd_[1];
      age = wall_timeout_ ? Seconds(Clock::now() - cmd_wall_) : t - cmd_sim_;
    }
    if (cmd_timeout_ > 0 && age > cmd_timeout_) vx = wz = 0.0;
    applied_cmd_ = {vx, wz};

    for (auto& w : wheels_) {
```

with

```cpp
    TakeTopicSetpoints(info, ecm);

    for (auto& w : wheels_) {
```

replace

```cpp
      if (ecm.Component<components::WheelSetpoint>(w.joint.Entity())) {
        w.setpoint = Setpoint(w, ecm, t);
      } else {
        double target = (vx - w.side * wz * track_ / 2) / radius_;
        w.ramp.LimitVelocity(target);
        w.ramp.LimitAcceleration(target, w.setpoint, info.dt);
        w.setpoint = target;
      }
      w.joint.SetForce(ecm, {w.motor.Step(motor_, w.setpoint, w.speed, dt)});
```

with

```cpp
      w.setpoint = Setpoint(w, ecm, t);
      w.joint.SetForce(ecm, {w.motor.Step(motor_, w.setpoint, w.speed, dt)});
```

replace

```cpp
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd_ = {0.0, 0.0};
    }
    for (auto& w : wheels_) {
      w.setpoint = w.spin = 0.0;
      w.cmd_time = std::chrono::steady_clock::duration::min();
      w.motor.Reset();
```

with

```cpp
    {
      std::lock_guard<std::mutex> lock(mutex_);
      topic_fresh_ = false;
    }
    for (auto& w : wheels_) {
      w.setpoint = w.spin = 0.0;
      w.cmd_time = std::chrono::steady_clock::duration::min();
      // A setpoint held from before the reset (cmd_timeout 0) must not drive the rover again: zero, stamped now.
      if (auto* cmd = ecm.Component<components::WheelSetpoint>(w.joint.Entity())) {
        cmd->Data() = components::Setpoint{0.0, info.simTime, cmd->Data().ros2_control};
      }
      w.motor.Reset();
```

replace

```cpp
  void OnCmd(const gz::msgs::Twist& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_ = {msg.linear().x(), msg.angular().z()};
    cmd_sim_ = sim_time_;
    cmd_wall_ = Clock::now();
  }
```

with

```cpp
  /// The setpoint topic's callback (on gz-transport's thread, or the publisher's for one in this process): keeps the
  /// newest message for the next PreUpdate, with the sim time it came at; one of another length is refused.
  void OnSetpoints(const gz::msgs::Double_V& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (static_cast<size_t>(msg.data_size()) != wheels_.size()) {
      if (!size_refused_) {
        gzerr << "RoverDrivetrain: " << setpoint_topic_ << " needs " << wheels_.size()
              << " setpoints (one per <wheel>, in their order), not " << msg.data_size() << "; refused\n";
        size_refused_ = true;
      }
      return;
    }
    topic_values_.assign(msg.data().begin(), msg.data().end());
    topic_time_ = sim_time_;
    topic_fresh_ = true;
  }

  /// The setpoint topic's newest message, if one came since the last step, into the wheels' WheelSetpoint
  /// components, stamped with the sim time it came at (the watchdog counts from then); dropped, saying so once,
  /// while ros2_control (RoverSimSystem) drives the wheels. At Gazebo's error level: `gz sim` prints nothing less at
  /// its default verbosity.
  void TakeTopicSetpoints(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) {
    std::vector<double> values;
    std::chrono::steady_clock::duration received{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sim_time_ = info.simTime;
      if (!topic_fresh_) return;
      topic_fresh_ = false;
      values.swap(topic_values_);
      received = topic_time_;
    }
    if (RosControlled(ecm)) {
      if (!topic_ignored_) {
        gzerr << "RoverDrivetrain: " << setpoint_topic_ << " is ignored: ros2_control's RoverSimSystem drives the "
              << "wheels\n";
        topic_ignored_ = true;
      }
      return;
    }
    for (size_t k = 0; k < wheels_.size(); ++k) {
      const components::Setpoint setpoint{values[k], received, false};
      const auto joint = wheels_[k].joint.Entity();
      if (auto* cmd = ecm.Component<components::WheelSetpoint>(joint)) {
        cmd->Data() = setpoint;
      } else {
        ecm.CreateComponent(joint, components::WheelSetpoint(setpoint));
      }
    }
  }

  /// Whether ros2_control drives the wheels: a wheel's setpoint says RoverSimSystem wrote it.
  bool RosControlled(const gz::sim::EntityComponentManager& ecm) const {
    for (const auto& w : wheels_) {
      const auto* cmd = ecm.Component<components::WheelSetpoint>(w.joint.Entity());
      if (cmd && cmd->Data().ros2_control) return true;
    }
    return false;
  }
```

replace

```cpp
    json << std::setprecision(6) << "{\"t\": " << t << ", \"cmd\": [" << applied_cmd_[0] << ", " << applied_cmd_[1]
         << "], \"wheels\": {";
```

with

```cpp
    // "cmd": the twist the setpoints in effect amount to, without slip (each side's mean; the odometry's track and
    // radius): what the motors are asked, whoever asks.
    double left = 0, right = 0, nl = 0, nr = 0;
    for (const auto& w : wheels_) {
      (w.side > 0 ? left : right) += w.setpoint;
      (w.side > 0 ? nl : nr) += 1;
    }
    left /= std::max(nl, 1.0);
    right /= std::max(nr, 1.0);
    json << std::setprecision(6) << "{\"t\": " << t << ", \"cmd\": [" << radius_ * (left + right) / 2 << ", "
         << radius_ * (right - left) / track_ << "], \"wheels\": {";
```

replace

```cpp
  double track_ = 0.8, radius_ = 0.15, accel_ = 8.0, max_speed_ = 10.0, rr_speed_ = 0.03;
```

with

```cpp
  double track_ = 0.8, radius_ = 0.15, max_speed_ = 10.0, rr_speed_ = 0.03;
```

replace

```cpp
  std::deque<Wheel> wheels_;  // in place: SpeedLimiter cannot be copied or moved
  gz::math::DiffDriveOdometry odometry_;
  std::array<double, 2> applied_cmd_{};
```

with

```cpp
  std::deque<Wheel> wheels_;
  gz::math::DiffDriveOdometry odometry_;
```

and replace

```cpp
  std::mutex mutex_;  // the command, written by gz-transport's thread
  std::array<double, 2> cmd_{};
  double sim_time_ = 0.0, cmd_sim_ = 0.0;
  Clock::time_point cmd_wall_ = Clock::now();
};
```

with

```cpp
  std::string setpoint_topic_;
  bool topic_ignored_ = false;  // said that ros2_control drives the wheels
  std::mutex mutex_;  // the setpoint topic's newest message, written by gz-transport's thread
  std::vector<double> topic_values_;
  std::chrono::steady_clock::duration sim_time_{}, topic_time_{};  // the latest PreUpdate's sim time; the message's
  bool topic_fresh_ = false, size_refused_ = false;
};
```

- [ ] **Step 4: sim.yaml, the wrapper, rover_config's docstring**

In `rover_sim/config/sim.yaml` replace

```yaml
# How the simulation models the rover and its ground: the drivetrain plugin's settings (its twist input, the stand-in
# motor controller, the motor model's integration), the contact model, the dig-in and its visual cues, the tyre
```

with

```yaml
# How the simulation models the rover and its ground: the drivetrain plugin's settings (its setpoint watchdog, the
# stand-in motor controller, the motor model's integration), the ROS-free helpers' ramp, the contact model, the dig-in
# and its visual cues, the tyre
```

replace

```yaml
  # The twist input (PR 3 moves it into the drive controller). A command older than cmd_timeout counts as zero, so a
  # commander that died cannot leave the rover driving (user decision 2026-10-06); 0 holds the last command, as
  # Gazebo's GUI Teleop needs. The clock: wall (a dead process) or sim (deterministic tests).
  cmd_timeout: 0.5                  # [s]
  cmd_timeout_clock: wall
  track_multiplier: 1.0             # effective-track compensation; 1 = the raw skid-steer response (D24, Q10)
  accel: 8.0                        # [rad/s^2] wheel setpoint ramp, 1.2 m/s^2 (A: until the driver team reports theirs, Q7)
```

with

```yaml
  # The motor controller's watchdog: a wheel setpoint older than cmd_timeout counts as zero, so a commander that died
  # cannot leave the rover driving (user decision 2026-10-06); 0 holds the last setpoint. The clock: wall (a dead
  # process) or sim (deterministic tests). The setpoints come from ros2_control (RoverSimSystem) or the ROS-free topic
  # topics.wheel_cmd; since PR 3 the twist-to-wheel kinematics, the effective-track compensation and the ramp are the
  # drive controller's (rover_control/config/controllers.yaml), and wheel_cmd's below for the ROS-free helpers.
  cmd_timeout: 0.5                  # [s]
  cmd_timeout_clock: wall
```

and replace

```yaml
  cmd_vel: /model/rover/cmd_vel
  wheel_cmd: /model/rover/wheel_cmd             # the wheels' speed setpoints [rad/s]: gz.msgs.Double_V, fl rl fr rr
```

with

```yaml
  cmd_vel: /model/rover/cmd_vel                 # Gazebo's DiffDrive only (drivetrain mode diffdrive): gz.msgs.Twist
  wheel_cmd: /model/rover/wheel_cmd             # the wheels' speed setpoints [rad/s]: gz.msgs.Double_V, fl rl fr rr
```

In `rover_sim/urdf/rover_sim.urdf.xacro` replace

```xml
      <plugin filename="RoverDrivetrain" name="rover_sim::RoverDrivetrain">
        <topic>${topics.cmd_vel}</topic>
```

with

```xml
      <plugin filename="RoverDrivetrain" name="rover_sim::RoverDrivetrain">
        <setpoint_topic>${topics.wheel_cmd}</setpoint_topic>
```

replace

```xml
        <radius>${rover.wheel.radius}</radius>
        <track_multiplier>${drive.track_multiplier}</track_multiplier>
```

with

```xml
        <radius>${rover.wheel.radius}</radius>
```

and replace

```xml
          <speed_filter>${drive.speed_filter}</speed_filter>
          <accel>${drive.accel}</accel>
```

with

```xml
          <speed_filter>${drive.speed_filter}</speed_filter>
```

In `rover_sim/rover_config.py` replace

```python
access (ROVER.wheel.radius, SIM.drivetrain.accel; YAML lists become tuples). Variant is a rover with some of those
```

with

```python
access (ROVER.wheel.radius, SIM.wheel_cmd.accel; YAML lists become tuples). Variant is a rover with some of those
```

- [ ] **Step 5: Build, regenerate the rover, and see the tests pass**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run sim-build 2>&1 | grep -E "warning|error|Summary"
/Users/alarion239/.pixi/bin/pixi run sim-model 2>&1 | tail -1
git diff --stat rover_sim/models && git diff rover_sim/models/rover/model.sdf | grep -E "^[-+] "
cd rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_rover_model test_equivalence test_drivetrain.Response test_drivetrain.SlowTurn test_drivetrain.Interfaces test_drivetrain.Watchdog test_ros2_control test_rover_sim > $T/task9.log 2>&1; tail -3 $T/task9.log
```

Expected: no warning, `Summary: 1 package finished`; `wrote config/surfaces.yaml, models/rover/model.sdf, ...`; one file changed, `rover_sim/models/rover/model.sdf`, with exactly these lines:

```
-      <topic>/model/rover/cmd_vel</topic>
+      <setpoint_topic>/model/rover/wheel_cmd</setpoint_topic>
-      <track_multiplier>1</track_multiplier>
-        <accel>8</accel>
```

then `OK` (with `SlowTurn`'s expected failure). `test_drivetrain.Interfaces.test_state_message` and `test_command_timeout` pass with their `assertEqual`s on `"cmd"` unchanged (Decision 7).

- [ ] **Step 6: The fixtures' README**

In `rover_sim/tests/fixtures/rover_model/README.md` replace

```markdown
| `test_physical.sdf` | `Params(drive=DriveParams(cmd_timeout_clock="sim", dig=False, mu_noise=0.0, backlash=0.0, current_limit=10.0, accel=0.0, state_rate=1000.0, cmd_timeout=0.0, default_surface="sand", tracks=TrackParams(report=1.0)))`: every number the tests change | `drivetrain={"cmd_timeout_clock": "sim", "accel": 0.0, "state_rate": 1000.0, "cmd_timeout": 0.0}, contact={"dig": False, "mu_noise": 0.0, "default_surface": "sand"}, driveline={"backlash": 0.0}, motor={"current_limit": 10.0}, ruts={"tracks": {"report": 1.0}}` |
```

with

```markdown
| `test_physical.sdf` | `Params(drive=DriveParams(cmd_timeout_clock="sim", dig=False, mu_noise=0.0, backlash=0.0, current_limit=10.0, accel=0.0, state_rate=1000.0, cmd_timeout=0.0, default_surface="sand", tracks=TrackParams(report=1.0)))`: every number the tests change | `drivetrain={"cmd_timeout_clock": "sim", "state_rate": 1000.0, "cmd_timeout": 0.0}, contact={"dig": False, "mu_noise": 0.0, "default_surface": "sand"}, driveline={"backlash": 0.0}, motor={"current_limit": 10.0}, ruts={"tracks": {"report": 1.0}}` (its `accel=0.0` was the drivetrain's ramp, one of the four elements below) |
```

and replace

```markdown
When the rover changes on purpose (real numbers from the mechanical team, a new sensor), the test fails as it should.
```

with

```markdown
Since PR 3 (the ros2_control boundary, 2026-10-10) the drivetrain takes wheel setpoints instead of a twist: its twist topic, track multiplier and ramp (`<topic>`, `<track_multiplier>`, `<controller><accel>`) are the drive controller's, and it reads `<setpoint_topic>`. Nothing physical changed, so these files stay what gen_model.py wrote, and `test_equivalence.ros2_control_boundary()` makes exactly that change to them before the comparison (a test checks that it changes those four elements and nothing else, and nothing of a DiffDrive model).

When the rover changes on purpose (real numbers from the mechanical team, a new sensor), the test fails as it should.
```

- [ ] **Step 7: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim && git commit -q -F - <<'EOF'
PR 3: rover_sim: the drivetrain takes only wheel setpoints

RoverDrivetrain follows each wheel's WheelSetpoint: RoverSimSystem's under ros2_control, else the newest message of
its ROS-free topic /model/rover/wheel_cmd (gz.msgs.Double_V, fl rl fr rr), stamped with the sim time it came at and
ignored, said once at Gazebo's error level, while ros2_control drives the wheels. The watchdog zeroes a setpoint older
than cmd_timeout at once; the clamp to the top speed stays. The twist input, its kinematics, track_multiplier and the
ramp are gone (the drive controller's, and wheel_cmd.py's for the ROS-free tools); the state message's "cmd" is the
twist the setpoints amount to. sim.yaml and the wrapper follow; the tracked rover changes in those four elements
only. test_equivalence compares the frozen models after ros2_control_boundary(), the one change that is not
physical; test_drivetrain's ramp settings move to wheel_cmd with the same numbers, and Watchdog checks the timeout
to the step; test_ros2_control checks that the topic is ignored under ros2_control.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 10: The calibration runs are bit for bit what they were; the full suite

**Files:** none in SRC; `$T/traces_after.npz`, `$T/suite_b.log`.

- [ ] **Step 1: The traces**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run python $T/traces.py $T/traces_after.npz > /dev/null 2>&1
/Users/alarion239/.pixi/bin/pixi run python -c "
import numpy as np
a, b = np.load('$T/traces_before.npz'), np.load('$T/traces_after.npz')
for k in a.files:
    same = a[k].shape == b[k].shape and np.array_equal(a[k], b[k])
    first = None if same else int(np.argmax(np.any(a[k][:len(b[k])] != b[k][:len(a[k])], axis=1)))
    print(k, same, '' if same else f'first differing row {first}: {a[k][first][:3]} vs {b[k][first][:3]}')"
```

Expected: seven lines, each `True`: the drivetrain follows the setpoints wheel_cmd.Driver ramps exactly as it ramped its twist (Decision 6), one step later as before. If a physical run differs (`model_rover`, `spin_stop`, `spin_stop_no_ramp`, `arc_sand`, `follow`, `sand_dig_spin`) while `diffdrive` is `True`, the drivetrain no longer runs before the tests' helper system in a step, or the ramp's arithmetic differs: check that the first differing row is where the first command starts to act (row about 20), then compare `wheel_cmd.Ramp` with gz-math 7.7.0's `SpeedLimiter::LimitAcceleration` (`$S/gz-math/src/SpeedLimiter.cc` if the reference clones are still there). The source does not show how the multiply-add is rounded: the plan critic disassembled the environment's `libgz-math7.dylib` (`llvm-objdump -d`, the symbol `SpeedLimiter17LimitAcceleration`) and found `fmadd`, which `wheel_cmd.fused` reproduces (Decision 6). A first difference in the row where a ramp reaches its target points there. Record what was found in the results; the suite's numbers (Step 2) decide.

- [ ] **Step 2: The full suite and the colcon tests**

Run in the background (about 16 minutes):

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run test > $T/suite_b.log 2>&1; echo "exit $?" >> $T/suite_b.log
```

Then:

```bash
T=/private/tmp/claude-502/rover-pr3
grep -E "tests passed|tests failed|^Ran |^OK|^FAILED|^exit" $T/suite_b.log
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain
```

Expected: `... out of 57` (rover_control, unchanged until Part C), `... out of 5` (rover_description), `100% tests passed, 0 tests failed out of 5` (rover_sim: the three plugin tests, `test_rover_components`, `test_rover_sim_system`); `Ran 577 tests` (551, plus 6 `Ros2ControlVariant`, 2 `Environment`, 4 `test_ros2_control`, 11 `test_wheel_cmd`, 2 `Watchdog`, 1 `Equivalence`), `OK (skipped=9, expected failures=2)`; `exit 0`; a clean tree (the suite's `sim-model` wrote the committed model). A timing test that fails under the machine's other load is rerun alone before it counts. Nothing to commit.

---

## Part C: rover_control

### Task 11: The tank kinematics replace the swerve library; controllers.yaml is checked against rover.yaml

**Files:** Create `rover_control/include/rover_control/tank_kinematics.hpp`, `rover_control/src/tank_kinematics.cpp`, `rover_control/test/test_tank_kinematics.cpp`, `rover_control/test/test_controllers_yaml.cpp`. Replace `rover_control/CMakeLists.txt`, `rover_control/package.xml`. Remove `rover_control/include/rover_driver/`, `rover_control/src/kinematics_2d.cpp`, `kinematics_3d.cpp`, `steering.cpp`, `rover_control/tests/`, `rover_control/tools/`. Modify `pixi.toml`, `pixi.lock`, `rover_description/config/rover.yaml` (a comment).

- [ ] **Step 1: The failing tests**

Write `rover_control/test/test_tank_kinematics.cpp`:

```cpp
// The rover's skid-steer kinematics (tank_kinematics.hpp): the geometry read from a rover.yaml, the inverse and forward
// kinematics, the saturation that keeps the turn radius; rover_description's own rover.yaml loads.
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "rover_control/tank_kinematics.hpp"

namespace {

using rover_control::LoadRoverYaml;
using rover_control::TankGeometry;
using rover_control::Twist;
using rover_control::WheelSpeeds;

/// A file in the test's temporary folder holding `body`; its path.
std::string Write(const std::string& name, const std::string& body) {
  const std::string path = ::testing::TempDir() + name;
  std::ofstream(path) << body;
  return path;
}

TankGeometry Geometry() {
  TankGeometry g;
  g.wheel_radius = 0.15;
  g.track = 0.8;
  g.wheelbase = 0.9;
  g.top_speed = 20.0;
  return g;
}

TEST(LoadRoverYaml, ReadsTheGeometry) {
  const TankGeometry g = LoadRoverYaml(Write("rover_a.yaml",
      "rocker: {pivot_y: 0.55, pivot_z: 0.3}\nwheel: {radius: 0.2, width: 0.1, dx: 0.6, dz: -0.1, top_speed: 15.0}\n"));
  EXPECT_DOUBLE_EQ(g.wheel_radius, 0.2);
  EXPECT_DOUBLE_EQ(g.track, 1.1);
  EXPECT_DOUBLE_EQ(g.wheelbase, 1.2);
  EXPECT_DOUBLE_EQ(g.top_speed, 15.0);
  EXPECT_DOUBLE_EQ(g.track_multiplier, 1.0);
}

TEST(LoadRoverYaml, LoadsTheDescriptionsRoverYaml) {
  const TankGeometry g = LoadRoverYaml(ROVER_YAML);
  EXPECT_NO_THROW(rover_control::Validate(g));
  EXPECT_GT(g.track, 2 * g.wheel_radius);
}

TEST(LoadRoverYaml, NamesWhatIsMissingOrWrong) {
  const std::string path = Write("rover_b.yaml", "wheel: {radius: 0.2, dx: 0.6, top_speed: 15.0}\n");
  try {
    LoadRoverYaml(path);
    FAIL() << "no exception";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("rocker.pivot_y"), std::string::npos) << e.what();
    EXPECT_NE(std::string(e.what()).find(path), std::string::npos) << e.what();
  }
  EXPECT_THROW(LoadRoverYaml(::testing::TempDir() + "no_such_rover.yaml"), std::runtime_error);
  const std::string negative =
      Write("rover_c.yaml", "rocker: {pivot_y: 0.5}\nwheel: {radius: -0.2, dx: 0.6, top_speed: 15.0}\n");
  EXPECT_THROW(LoadRoverYaml(negative), std::invalid_argument);
}

TEST(TankKinematics, StraightAndInPlace) {
  const TankGeometry g = Geometry();
  const WheelSpeeds straight = rover_control::ToWheels(g, {0.6, 0.0});
  EXPECT_DOUBLE_EQ(straight.left, 4.0);
  EXPECT_DOUBLE_EQ(straight.right, 4.0);
  const WheelSpeeds spin = rover_control::ToWheels(g, {0.0, 1.0});
  EXPECT_DOUBLE_EQ(spin.left, -0.4 / 0.15);
  EXPECT_DOUBLE_EQ(spin.right, 0.4 / 0.15);
}

TEST(TankKinematics, ForwardUndoesInverse) {
  const TankGeometry g = Geometry();
  for (const Twist t : {Twist{0.3, 0.2}, Twist{-0.5, 1.1}, Twist{1.2, -0.7}}) {
    const Twist back = rover_control::ToTwist(g, rover_control::ToWheels(g, t));
    EXPECT_NEAR(back.vx, t.vx, 1e-12);
    EXPECT_NEAR(back.wz, t.wz, 1e-12);
  }
}

TEST(TankKinematics, TheTrackMultiplierWidensTheTrack) {
  TankGeometry g = Geometry();
  g.track_multiplier = 1.5;
  EXPECT_DOUBLE_EQ(rover_control::ToWheels(g, {0.0, 1.0}).right, 0.6 / 0.15);
  EXPECT_NEAR(rover_control::ToTwist(g, rover_control::ToWheels(g, {0.2, 0.5})).wz, 0.5, 1e-12);
}

TEST(TankKinematics, SaturationKeepsTheTurnRadius) {
  const TankGeometry g = Geometry();
  const WheelSpeeds fast{10.0, 40.0};
  const WheelSpeeds limited = rover_control::Saturate(g, fast);
  EXPECT_DOUBLE_EQ(limited.right, 20.0);
  EXPECT_DOUBLE_EQ(limited.left, 5.0);
  const Twist a = rover_control::ToTwist(g, fast), b = rover_control::ToTwist(g, limited);
  EXPECT_NEAR(a.vx / a.wz, b.vx / b.wz, 1e-12);
  const WheelSpeeds slow{3.0, -2.0};
  EXPECT_DOUBLE_EQ(rover_control::Saturate(g, slow).left, 3.0);
  EXPECT_DOUBLE_EQ(rover_control::Saturate(g, slow).right, -2.0);
}

TEST(TankKinematics, ValidateNamesTheBadNumber) {
  TankGeometry g = Geometry();
  g.track = 0.0;
  try {
    rover_control::Validate(g);
    FAIL() << "no exception";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("track"), std::string::npos) << e.what();
  }
}

}  // namespace
```

Write `rover_control/test/test_controllers_yaml.cpp`:

```cpp
// rover_control's config/controllers.yaml: the controllers it names, and the drive controller's geometry and limits,
// which must be rover_description's rover.yaml's (the file writes them out: a YAML file cannot include another).
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "rover_control/tank_kinematics.hpp"

namespace {

const YAML::Node& Controllers() {
  static const YAML::Node root = YAML::LoadFile(CONTROLLERS_YAML);
  return root;
}

YAML::Node Drive() { return Controllers()["drive_controller"]["ros__parameters"]; }

double Number(const char* key) { return Drive()[key].as<double>(); }

TEST(ControllersYaml, TheControllerManagerRunsTheBroadcasterAndTheDriveController) {
  const YAML::Node manager = Controllers()["controller_manager"]["ros__parameters"];
  EXPECT_EQ(manager["joint_state_broadcaster"]["type"].as<std::string>(),
            "joint_state_broadcaster/JointStateBroadcaster");
  EXPECT_EQ(manager["drive_controller"]["type"].as<std::string>(), "diff_drive_controller/DiffDriveController");
  EXPECT_EQ(manager["update_rate"].as<int>(), 100);
}

TEST(ControllersYaml, LeftIsTheLeftWheelsRightTheRight) {
  EXPECT_EQ(Drive()["left_wheel_names"].as<std::vector<std::string>>(),
            (std::vector<std::string>{"wheel_fl_joint", "wheel_rl_joint"}));
  EXPECT_EQ(Drive()["right_wheel_names"].as<std::vector<std::string>>(),
            (std::vector<std::string>{"wheel_fr_joint", "wheel_rr_joint"}));
}

TEST(ControllersYaml, TheGeometryAndTheSpeedLimitsAreRoverYamls) {
  const rover_control::TankGeometry g = rover_control::LoadRoverYaml(ROVER_YAML);
  EXPECT_DOUBLE_EQ(Number("wheel_separation"), g.track);
  EXPECT_DOUBLE_EQ(Number("wheel_radius"), g.wheel_radius);
  EXPECT_DOUBLE_EQ(Number("wheel_separation_multiplier"), 1.0);
  const double top = g.top_speed * g.wheel_radius;  // [m/s] a wheel at its top speed
  EXPECT_NEAR(Number("linear.x.max_velocity"), top, 1e-9);
  EXPECT_NEAR(Number("linear.x.min_velocity"), -top, 1e-9);
  EXPECT_NEAR(Number("angular.z.max_velocity"), top / (g.track / 2), 1e-9);
  EXPECT_NEAR(Number("angular.z.min_velocity"), -top / (g.track / 2), 1e-9);
}

TEST(ControllersYaml, TheRampTurnsAWheelAtOneRate) {
  // A wheel's acceleration driving straight and turning in place is the same (8 rad/s^2: rover_sim's wheel_cmd accel,
  // which rover_sim's test_wheel_cmd checks).
  const rover_control::TankGeometry g = rover_control::LoadRoverYaml(ROVER_YAML);
  EXPECT_NEAR(Number("linear.x.max_acceleration") / g.wheel_radius,
              Number("angular.z.max_acceleration") * (g.track / 2) / g.wheel_radius, 1e-9);
  EXPECT_DOUBLE_EQ(Number("linear.x.max_deceleration"), -Number("linear.x.max_acceleration"));
  EXPECT_DOUBLE_EQ(Number("angular.z.max_deceleration"), -Number("angular.z.max_acceleration"));
}

TEST(ControllersYaml, ACommandTimesOutAfterHalfASecondAndOdometryIsOdomToBaseLink) {
  EXPECT_DOUBLE_EQ(Number("cmd_vel_timeout"), 0.5);
  EXPECT_EQ(Drive()["odom_frame_id"].as<std::string>(), "odom");
  EXPECT_EQ(Drive()["base_frame_id"].as<std::string>(), "base_link");
  EXPECT_TRUE(Drive()["enable_odom_tf"].as<bool>());
}

}  // namespace
```

- [ ] **Step 2: The package without the swerve code**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git rm -q -r rover_control/include/rover_driver rover_control/src rover_control/tests rover_control/tools && ls rover_control
```

Expected: `CMakeLists.txt README.md config package.xml test` (the swerve library, `rover_state`, `viz.py` and their tests stay in git history).

Write `rover_control/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20)
project(rover_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# MSVC (the Windows build) gets /W4: its -Wall would be /Wall, every warning of every header. It exports every
# symbol, so that a shared library has an import library.
if(MSVC)
  set(warnings /W4)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
else()
  set(warnings -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

# tank_kinematics: the rover's skid-steer kinematics, its geometry from rover_description's rover.yaml
# (include/rover_control/tank_kinematics.hpp).
add_library(tank_kinematics STATIC src/tank_kinematics.cpp)
target_include_directories(tank_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(tank_kinematics PUBLIC yaml-cpp::yaml-cpp)
set_target_properties(tank_kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(tank_kinematics PRIVATE ${warnings})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tank_kinematics EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

include(CTest)
if(BUILD_TESTING)
  # GoogleTest: the tank kinematics, and controllers.yaml against rover_description's rover.yaml.
  find_package(GTest REQUIRED)
  find_package(rover_description REQUIRED)
  add_executable(rover_control_tests
    test/test_tank_kinematics.cpp
    test/test_controllers_yaml.cpp
  )
  target_compile_definitions(rover_control_tests PRIVATE
    ROVER_YAML="${rover_description_DIR}/../config/rover.yaml"
    CONTROLLERS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/controllers.yaml")
  target_link_libraries(rover_control_tests PRIVATE tank_kinematics GTest::gtest_main)
  target_compile_options(rover_control_tests PRIVATE ${warnings})
  include(GoogleTest)
  gtest_discover_tests(rover_control_tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(tank_kinematics)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(yaml-cpp)
ament_package()
```

Write `rover_control/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_control</name>
  <version>0.0.0</version>
  <description>The rover's drive: the controllers the real and the simulated rover run (config/controllers.yaml: diff_drive_controller and joint_state_broadcaster), the real rover's launch file and its ros2_control hardware stub until the motor controllers are chosen, and a skid-steer kinematics library that reads rover_description's rover.yaml.</description>
  <maintainer email="amb30239@gmail.com">Alexander Belotserkovtsev</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>yaml-cpp</depend>
  <!-- rover.yaml: the tests' geometry, and the description rover.launch.py publishes -->
  <depend>rover_description</depend>

  <test_depend>gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

The maintainer and license lines are the existing file's, unchanged: all three packages carry the same license line, and choosing a license is the team's call, outside this PR.

In `pixi.toml` replace

```toml
# rover_control: the kinematics library and its tests
eigen = ">=5.0.1,<6"
gtest = ">=1.18.0,<2"
```

with

```toml
# rover_control: the tank kinematics' rover.yaml loader and the tests
gtest = ">=1.18.0,<2"
```

replace

```toml
# The colcon tests: rover_description's (xacro and check_urdf, the team's linters), rover_control's (GoogleTest,
# rover_state, viz.py) and rover_sim's three plugin test programs.
```

with

```toml
# The colcon tests: rover_description's (xacro and check_urdf, the team's linters), rover_control's (GoogleTest: the
# tank kinematics, controllers.yaml, the hardware stub; rover.launch.py with the stub) and rover_sim's C++ tests (the
# plugins' parts, the rover's ECM components, RoverSimSystem).
```

and delete

```toml
# rover_control's interactive view of the swerve kinematics.
control-viz = { cmd = "python rover_control/tools/viz.py", depends-on = ["build"] }
```

In `rover_description/config/rover.yaml` replace

```yaml
# design that chose it. The geometry is the swerve design's placeholder (rover_control's config.hpp, kRover) until
# the mechanical team has real numbers. To change the rover, edit this file and run `pixi run sim-model`
# (rover_description/README.md).
```

with

```yaml
# design that chose it. The geometry is the swerve design's placeholder (kRover of rover_control's old config.hpp,
# in git history) until the mechanical team has real numbers. To change the rover, edit this file and run
# `pixi run sim-model` (rover_description/README.md).
```

- [ ] **Step 3: Run the build to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run build 2>&1 | grep -E "tank_kinematics|Failed|Summary" | head -4
```

Expected: CMake stops with `Cannot find source file: src/tank_kinematics.cpp`, and `1 package failed: rover_control`.

- [ ] **Step 4: The library**

Write `rover_control/include/rover_control/tank_kinematics.hpp`:

```cpp
// Skid-steer (tank) kinematics of the rover: the left wheels (FL, RL) turn at one speed, the right ones (FR, RR) at
// another, and the body's forward speed and yaw rate follow from the two (design spec 2026-10-09, section 6). The
// geometry is rover_description's config/rover.yaml, read with LoadRoverYaml, or numbers a node takes from its ROS
// parameters. On the rover the drive controller (diff_drive_controller, config/controllers.yaml) does the same with
// the same numbers; this library is for the team's own drive code and tools.
#pragma once

#include <string>

namespace rover_control {

/// The rover's geometry as skid-steer kinematics needs it (SI units).
struct TankGeometry {
  double wheel_radius = 0.0;      ///< [m] rover.yaml wheel.radius
  double track = 0.0;             ///< [m] between the left and right wheel centres: 2 x rover.yaml rocker.pivot_y
  double wheelbase = 0.0;         ///< [m] between the front and rear wheel centres: 2 x rover.yaml wheel.dx
  double top_speed = 0.0;         ///< [rad/s] a wheel's top speed: rover.yaml wheel.top_speed
  double track_multiplier = 1.0;  ///< effective-track compensation of skid steer (1: the geometric track)
};

/// A body twist in base_link: forward speed [m/s] and yaw rate [rad/s], left positive.
struct Twist {
  double vx = 0.0;
  double wz = 0.0;
};

/// The wheels' speeds [rad/s], each side's two wheels alike.
struct WheelSpeeds {
  double left = 0.0;
  double right = 0.0;
};

/// Throws std::invalid_argument naming the first number that is not a positive finite number.
void Validate(const TankGeometry& geometry);

/// Inverse kinematics: (vx -+ wz x track x track_multiplier / 2) / wheel_radius, left -, right +.
WheelSpeeds ToWheels(const TankGeometry& geometry, const Twist& twist);

/// Forward kinematics, the inverse of ToWheels.
Twist ToTwist(const TankGeometry& geometry, const WheelSpeeds& wheels);

/// Both sides scaled by one factor so that the faster turns at most at top_speed: the same turn radius, slower.
WheelSpeeds Saturate(const TankGeometry& geometry, const WheelSpeeds& wheels);

/// The geometry in a rover.yaml (rover_description/config/rover.yaml): throws std::runtime_error naming the file and
/// the key when a number is missing or is not one, and std::invalid_argument (Validate) when one is not positive.
TankGeometry LoadRoverYaml(const std::string& path);

}  // namespace rover_control
```

Write `rover_control/src/tank_kinematics.cpp`:

```cpp
#include "rover_control/tank_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace rover_control {

namespace {

void Positive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string("tank geometry: ") + name + " must be a positive number");
  }
}

double Number(const YAML::Node& root, const std::string& path, const char* section, const char* key) {
  const std::string name = std::string(section) + "." + key;
  const YAML::Node group = root[section];
  if (!group || !group.IsMap() || !group[key] || !group[key].IsScalar()) {
    throw std::runtime_error(path + ": no number " + name);
  }
  try {
    return group[key].as<double>();
  } catch (const YAML::Exception&) {
    throw std::runtime_error(path + ": " + name + " is not a number");
  }
}

}  // namespace

void Validate(const TankGeometry& geometry) {
  Positive(geometry.wheel_radius, "wheel_radius");
  Positive(geometry.track, "track");
  Positive(geometry.wheelbase, "wheelbase");
  Positive(geometry.top_speed, "top_speed");
  Positive(geometry.track_multiplier, "track_multiplier");
}

WheelSpeeds ToWheels(const TankGeometry& geometry, const Twist& twist) {
  const double half = twist.wz * geometry.track * geometry.track_multiplier / 2.0;
  return {(twist.vx - half) / geometry.wheel_radius, (twist.vx + half) / geometry.wheel_radius};
}

Twist ToTwist(const TankGeometry& geometry, const WheelSpeeds& wheels) {
  return {geometry.wheel_radius * (wheels.left + wheels.right) / 2.0,
          geometry.wheel_radius * (wheels.right - wheels.left) / (geometry.track * geometry.track_multiplier)};
}

WheelSpeeds Saturate(const TankGeometry& geometry, const WheelSpeeds& wheels) {
  const double fastest = std::max(std::abs(wheels.left), std::abs(wheels.right));
  if (fastest <= geometry.top_speed) return wheels;
  const double scale = geometry.top_speed / fastest;
  return {wheels.left * scale, wheels.right * scale};
}

TankGeometry LoadRoverYaml(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error(path + ": " + e.what());
  }
  TankGeometry geometry;
  geometry.wheel_radius = Number(root, path, "wheel", "radius");
  geometry.track = 2.0 * Number(root, path, "rocker", "pivot_y");
  geometry.wheelbase = 2.0 * Number(root, path, "wheel", "dx");
  geometry.top_speed = Number(root, path, "wheel", "top_speed");
  Validate(geometry);
  return geometry;
}

}  // namespace rover_control
```

- [ ] **Step 5: Run the tests to see them pass; the lock without eigen**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run build 2>&1 | grep -E "warning|error|Summary"
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon test --packages-select rover_control --event-handlers console_cohesion+ 2>&1 | grep -E "tests passed|tests failed|Summary"
cd /Users/alarion239/Desktop/hurc_ws/src && cp pixi.lock $T/pixi.lock.task11 && /Users/alarion239/.pixi/bin/pixi lock 2>&1 | tail -1 && /Users/alarion239/.pixi/bin/pixi run python $T/lockdiff.py $T/pixi.lock.task11 pixi.lock
```

Expected: no warning, `Summary: 3 packages finished`; `100% tests passed, 0 tests failed out of 13` (8 kinematics, 5 controllers.yaml); the lock: `linux-64: 0 added, 0 removed`, `osx-arm64: 0 added, 0 removed`, `win-64: 0 added, 0 removed` (eigen stays, ros-jazzy-desktop needs it; `pixi lock` may rewrite nothing at all).

- [ ] **Step 6: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add -A rover_control pixi.toml pixi.lock rover_description/config/rover.yaml && git commit -q -F - <<'EOF'
PR 3: rover_control: the tank kinematics replace the swerve library

tank_kinematics (C++17, yaml-cpp): skid-steer inverse and forward kinematics with the effective-track multiplier, a
saturation that keeps the turn radius, and LoadRoverYaml, the geometry from rover_description's rover.yaml (a node
gives the same numbers from its ROS parameters). Its GoogleTest reads a rover.yaml with other numbers and the
description's own; test_controllers_yaml checks that controllers.yaml's drive controller has rover.yaml's geometry,
speed limits and one ramp rate for a wheel. The swerve design's library, rover_state, viz.py, their tests and
config.hpp's kRover are removed (they stay in git history), with the control-viz task and eigen in pixi.toml (the
lock keeps it: ros-jazzy-desktop needs it).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 12: The real rover's hardware stub

**Files:** Create `rover_control/include/rover_control/rover_hardware_stub.hpp`, `rover_control/src/rover_hardware_stub.cpp`, `rover_control/rover_hardware_stub.xml`, `rover_control/test/test_rover_hardware_stub.cpp`. Modify `rover_control/CMakeLists.txt`, `rover_control/package.xml`, `rover_description/urdf/rover.urdf.xacro` and `rover_description/urdf/ros2_control.xacro` (comments).

- [ ] **Step 1: The failing test**

Write `rover_control/test/test_rover_hardware_stub.cpp`:

```cpp
// RoverHardwareStub (rover_hardware_stub.cpp) without a controller manager: the interfaces of rover_description's
// ros2_control block, zero motion whatever it is commanded, and the line it logs for the commands it would send.
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rover_control/rover_hardware_stub.hpp"

namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using rover_control::RoverHardwareStub;

const std::vector<std::string> kWheels = {"wheel_fl_joint", "wheel_rl_joint", "wheel_fr_joint", "wheel_rr_joint"};
const std::vector<std::string> kSensed = {"rocker_left_joint", "rocker_right_joint", "camera_pan_joint",
                                          "camera_tilt_joint"};

hardware_interface::InterfaceInfo Interface(const std::string& name) {
  hardware_interface::InterfaceInfo info{};
  info.name = name;
  info.size = 1;
  info.enable_limits = false;
  return info;
}

/// rover_description's ros2_control block (urdf/ros2_control.xacro), as rover.launch.py loads it.
hardware_interface::HardwareInfo RoverInfo() {
  hardware_interface::HardwareInfo info{};
  info.name = "rover";
  info.type = "system";
  info.hardware_plugin_name = "rover_control/RoverHardwareStub";
  for (const auto& name : kWheels) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.command_interfaces = {Interface("velocity")};
    joint.state_interfaces = {Interface("position"), Interface("velocity"), Interface("effort")};
    info.joints.push_back(joint);
  }
  for (const auto& name : kSensed) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.state_interfaces = {Interface("position"), Interface("velocity")};
    info.joints.push_back(joint);
  }
  return info;
}

class RoverHardwareStubTest : public ::testing::Test {
 protected:
  void SetUp() override {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = RoverInfo();
    params.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    params.logger = rclcpp::get_logger("test_rover_hardware_stub");
    ASSERT_EQ(stub_.init(params), CallbackReturn::SUCCESS);
    for (const auto& state : stub_.on_export_state_interfaces()) states_[state->get_name()] = state;
    for (const auto& command : stub_.on_export_command_interfaces()) commands_[command->get_name()] = command;
  }

  RoverHardwareStub stub_;
  std::map<std::string, hardware_interface::StateInterface::ConstSharedPtr> states_;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> commands_;
};

TEST_F(RoverHardwareStubTest, ExportsTheBlocksInterfaces) {
  EXPECT_EQ(states_.size(), 4u * 3u + 4u * 2u);
  EXPECT_EQ(commands_.size(), 4u);
  for (const auto& name : kWheels) EXPECT_EQ(commands_.count(name + "/velocity"), 1u) << name;
}

TEST_F(RoverHardwareStubTest, ReportsZeroMotionWhateverItIsCommanded) {
  for (const auto& [name, command] : commands_) ASSERT_TRUE(command->set_value(3.0, true)) << name;
  ASSERT_EQ(stub_.write(rclcpp::Time(1, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  ASSERT_EQ(stub_.read(rclcpp::Time(1, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  for (const auto& [name, state] : states_) EXPECT_EQ(state->get_optional<double>().value(), 0.0) << name;
}

TEST_F(RoverHardwareStubTest, Activates) {
  EXPECT_EQ(stub_.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(WouldSend, NamesEachWheelsCommandOrNothingWhenAllAreZero) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(RoverHardwareStub::WouldSend({"wheel_fl_joint", "wheel_fr_joint"}, {2.0, -1.5}),
            "would send wheel_fl_joint 2.00, wheel_fr_joint -1.50 rad/s");
  EXPECT_EQ(RoverHardwareStub::WouldSend({"wheel_fl_joint", "wheel_fr_joint"}, {0.0, nan}), "");
  EXPECT_EQ(RoverHardwareStub::WouldSend({"wheel_fl_joint", "wheel_fr_joint"}, {nan, 0.5}),
            "would send wheel_fl_joint 0.00, wheel_fr_joint 0.50 rad/s");
}

}  // namespace
```

In `rover_control/CMakeLists.txt` replace

```cmake
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
```

with

```cmake
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
```

replace

```cmake
install(DIRECTORY include/ DESTINATION include)
install(TARGETS tank_kinematics EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
```

with

```cmake
# rover_hardware_stub: the real rover's ros2_control hardware until its motor controllers are chosen
# (pluginlib: rover_control/RoverHardwareStub, rover_hardware_stub.xml).
add_library(rover_hardware_stub SHARED src/rover_hardware_stub.cpp)
target_include_directories(rover_hardware_stub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(rover_hardware_stub PUBLIC hardware_interface::hardware_interface pluginlib::pluginlib
                      rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)
target_compile_options(rover_hardware_stub PRIVATE ${warnings})
pluginlib_export_plugin_description_file(hardware_interface rover_hardware_stub.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tank_kinematics rover_hardware_stub EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
```

replace

```cmake
  # GoogleTest: the tank kinematics, and controllers.yaml against rover_description's rover.yaml.
  find_package(GTest REQUIRED)
  find_package(rover_description REQUIRED)
  add_executable(rover_control_tests
    test/test_tank_kinematics.cpp
    test/test_controllers_yaml.cpp
  )
```

with

```cmake
  # GoogleTest: the tank kinematics, controllers.yaml against rover_description's rover.yaml, the hardware stub.
  find_package(GTest REQUIRED)
  find_package(rover_description REQUIRED)
  add_executable(rover_control_tests
    test/test_tank_kinematics.cpp
    test/test_controllers_yaml.cpp
    test/test_rover_hardware_stub.cpp
  )
```

replace

```cmake
  target_link_libraries(rover_control_tests PRIVATE tank_kinematics GTest::gtest_main)
```

with

```cmake
  target_link_libraries(rover_control_tests PRIVATE tank_kinematics rover_hardware_stub GTest::gtest_main)
```

and replace

```cmake
ament_export_libraries(tank_kinematics)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(yaml-cpp)
```

with

```cmake
ament_export_libraries(tank_kinematics rover_hardware_stub)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle yaml-cpp)
```

In `rover_control/package.xml` replace

```xml
  <depend>yaml-cpp</depend>
```

with

```xml
  <!-- the hardware stub, a ros2_control hardware plugin -->
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>yaml-cpp</depend>
```

- [ ] **Step 2: Run the build to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run build 2>&1 | grep -E "rover_hardware_stub|Failed|Summary" | head -4
```

Expected: `Cannot find source file: src/rover_hardware_stub.cpp`, `1 package failed: rover_control`.

- [ ] **Step 3: The stub**

Write `rover_control/include/rover_control/rover_hardware_stub.hpp`:

```cpp
// RoverHardwareStub: the real rover's ros2_control hardware until its motor controllers are chosen (design spec
// 2026-10-09, section 6). It exports the interfaces of rover_description's ros2_control block (wheels: velocity
// command, position, velocity and effort state; rockers and camera head: position and velocity), reports zero motion,
// and drives nothing: it says so when it starts and when it is activated, and logs the wheel commands it would send,
// at most once a second. rover.launch.py runs it (rover_description's rover.urdf.xacro names it by default). It
// activates, so that the controllers start on the rover's computer (the spec had it refuse; spec section 12).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace rover_control {

class RoverHardwareStub : public hardware_interface::SystemInterface {
 public:
  using hardware_interface::SystemInterface::on_init;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// "would send wheel_fl_joint 1.00, ... rad/s" for the joints' commands (one not set yet, NaN, as 0), or "" when
  /// every one is 0.
  static std::string WouldSend(const std::vector<std::string>& joints, const std::vector<double>& commands);

 private:
  std::vector<std::string> states_;     // every state interface's full name
  std::vector<std::string> commanded_;  // the joints with a velocity command
  int64_t logged_ = 0;                  // [ns] when write() last logged, on the controller manager's clock
  bool logged_once_ = false;
};

}  // namespace rover_control
```

Write `rover_control/src/rover_hardware_stub.cpp`:

```cpp
#include "rover_control/rover_hardware_stub.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rover_control {

namespace {
constexpr int64_t kLogPeriod = 1000000000;  // [ns] between two logs of the commands it would send
}  // namespace

hardware_interface::CallbackReturn RoverHardwareStub::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  states_.clear();
  commanded_.clear();
  for (const auto& joint : info_.joints) {
    for (const auto& state : joint.state_interfaces) states_.push_back(joint.name + "/" + state.name);
    for (const auto& command : joint.command_interfaces) {
      if (command.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(get_logger(), "joint '%s': RoverHardwareStub takes velocity commands only, not '%s'",
                     joint.name.c_str(), command.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      commanded_.push_back(joint.name);
    }
  }
  RCLCPP_WARN(get_logger(),
              "No motors are connected: RoverHardwareStub reports zero motion and drives nothing (the motor "
              "controllers are not chosen yet, rover_control/README.md); %zu joints, %zu wheels",
              info_.joints.size(), commanded_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverHardwareStub::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_WARN(get_logger(), "Active, but no motors are connected: the controllers run and no wheel turns");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverHardwareStub::read(const rclcpp::Time& /*time*/,
                                                        const rclcpp::Duration& /*period*/) {
  for (const auto& name : states_) set_state(name, 0.0);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverHardwareStub::write(const rclcpp::Time& time,
                                                         const rclcpp::Duration& /*period*/) {
  std::vector<double> commands;
  commands.reserve(commanded_.size());
  for (const auto& joint : commanded_) {
    commands.push_back(get_command<double>(joint + "/" + hardware_interface::HW_IF_VELOCITY));
  }
  const std::string line = WouldSend(commanded_, commands);
  if (!line.empty() && (!logged_once_ || time.nanoseconds() - logged_ >= kLogPeriod)) {
    RCLCPP_WARN(get_logger(), "No motors are connected: %s", line.c_str());
    logged_ = time.nanoseconds();
    logged_once_ = true;
  }
  return hardware_interface::return_type::OK;
}

std::string RoverHardwareStub::WouldSend(const std::vector<std::string>& joints, const std::vector<double>& commands) {
  bool moving = false;
  std::ostringstream line;
  line << std::fixed << std::setprecision(2) << "would send ";
  for (size_t k = 0; k < joints.size() && k < commands.size(); ++k) {
    const double value = std::isfinite(commands[k]) ? commands[k] : 0.0;
    moving = moving || value != 0.0;
    line << (k ? ", " : "") << joints[k] << ' ' << value;
  }
  line << " rad/s";
  return moving ? line.str() : std::string();
}

}  // namespace rover_control

PLUGINLIB_EXPORT_CLASS(rover_control::RoverHardwareStub, hardware_interface::SystemInterface)
```

Write `rover_control/rover_hardware_stub.xml`:

```xml
<library path="rover_hardware_stub">
  <class name="rover_control/RoverHardwareStub" type="rover_control::RoverHardwareStub"
         base_class_type="hardware_interface::SystemInterface">
    <description>
      The real rover's ros2_control hardware until its motor controllers are chosen: the description's interfaces,
      zero motion, and a log of the wheel commands it would send.
    </description>
  </class>
</library>
```

In `rover_description/urdf/rover.urdf.xacro` replace

```xml
       ros2_control_plugin  its hardware interface (default rover_control's real-hardware stub, which PR 3 adds)
```

with

```xml
       ros2_control_plugin  its hardware interface (default rover_control/RoverHardwareStub, the real rover's until
                            its motor controllers are chosen)
```

and in `rover_description/urdf/ros2_control.xacro` replace

```xml
     and velocity only. `plugin` is the hardware interface: rover_control's real-hardware stub by default
     (rover.urdf.xacro's ros2_control_plugin argument); the simulation sets its own (PR 3). Nothing loads this block
     yet: the controllers come with PR 3. -->
```

with

```xml
     and velocity only. `plugin` is the hardware interface: rover_control's real-hardware stub by default
     (rover.urdf.xacro's ros2_control_plugin argument); the simulation sets RoverSimSystem (rover_sim's
     rover_sim.urdf.xacro with ros2_control:=true). The controller manager loads it: rover_control's rover.launch.py
     on the rover, gz_ros2_control in the simulation. -->
```

- [ ] **Step 4: Run the tests to see them pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run build 2>&1 | grep -E "warning|error|Summary"
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon test --packages-select rover_control rover_description --event-handlers console_cohesion+ 2>&1 | grep -E "tests passed|tests failed|Summary"
cat install/rover_control/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/rover_control; echo
```

Expected: `Summary: 3 packages finished` without warnings from rover_control's own files (a warning from a ROS header under `-Wpedantic` is noted, not fixed); `100% tests passed, 0 tests failed out of 17` (rover_control: 13 + 4) and `... out of 5` (rover_description, the comments changed); `share/rover_control/rover_hardware_stub.xml`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_control rover_description/urdf && git commit -q -F - <<'EOF'
PR 3: rover_control: the real rover's hardware stub

RoverHardwareStub (pluginlib: rover_control/RoverHardwareStub), the hardware interface rover_description's
ros2_control block names by default until the motor controllers are chosen: it exports the block's interfaces,
reports zero position, velocity and effort, warns at start and on activation that no motors are connected, and logs
the wheel commands it would send, at most once a second. It activates (the spec had it refuse), so that the
controllers start on the rover's computer. Its GoogleTest checks the interfaces, the zero motion, the activation and
the logged line. rover_description's comments name it and RoverSimSystem.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 13: The controllers' launch file and the real rover's rover.launch.py

**Files:** Create `rover_control/launch/controllers.launch.py`, `rover_control/launch/rover.launch.py`, `rover_control/test/test_rover_launch.py`. Modify `rover_control/CMakeLists.txt`, `rover_control/package.xml`, `rover_sim/tests/rosstack.py`.

- [ ] **Step 1: The failing test**

Write `rover_control/test/test_rover_launch.py`:

```python
"""rover.launch.py, the real rover's ROS 2 stack, with the hardware stub (design spec 2026-10-09, 6.3), and
controllers.launch.py, which it shares with the simulation: what they start, and a run in a private ROS domain: the
controllers come up, /joint_states reports zero motion, a /cmd_vel command reaches the stub, which logs what it would
send, and /odom stays at the origin. CTest runs it with this package's install on AMENT_PREFIX_PATH
(CMakeLists.txt)."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
JOINTS = {'wheel_fl_joint', 'wheel_rl_joint', 'wheel_fr_joint', 'wheel_rr_joint', 'rocker_left_joint',
          'rocker_right_joint', 'camera_pan_joint', 'camera_tilt_joint'}
DOMAIN = str(50 + os.getpid() % 50)  # a private ROS domain, as rover_sim's tests take (gzenv.private_ros_domain)


def load(path, name):
    """Import a launch file."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def defaults(description):
    from launch.actions import DeclareLaunchArgument
    return {e.name: e.default_value[0].text for e in description.entities
            if isinstance(e, DeclareLaunchArgument)}


class Description(unittest.TestCase):

    def test_rover_launch_starts_the_description_the_manager_and_the_controllers(self):
        from launch.actions import IncludeLaunchDescription
        from launch_ros.actions import Node
        description = load(PACKAGE / 'launch' / 'rover.launch.py', 'rover_launch').generate_launch_description()
        found = defaults(description)
        self.assertEqual(found['controller'], 'drive_controller')
        self.assertEqual(found['hardware'], 'rover_control/RoverHardwareStub')
        self.assertTrue(found['rover_yaml'].endswith(os.path.join('config', 'rover.yaml')))
        self.assertEqual([e.node_executable for e in description.entities if isinstance(e, Node)],
                         ['robot_state_publisher', 'ros2_control_node'])
        self.assertEqual(len([e for e in description.entities
                              if isinstance(e, IncludeLaunchDescription)]), 1)

    def test_controllers_launch_spawns_both_with_the_contracts_topics(self):
        module = load(PACKAGE / 'launch' / 'controllers.launch.py', 'controllers_launch')
        found = defaults(module.generate_launch_description())
        self.assertEqual(found['controller'], 'drive_controller')
        self.assertEqual(found['controllers'], str(PACKAGE / 'config' / 'controllers.yaml'))
        self.assertEqual(module.spawner_arguments('team_drive', '/c.yaml', '60'), [
            'joint_state_broadcaster', 'team_drive', '--param-file', '/c.yaml',
            '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60',
            '--controller-ros-args=-r /team_drive/cmd_vel:=/cmd_vel -r /team_drive/odom:=/odom'])


class Run(unittest.TestCase):
    """ros2 launch rover_control rover.launch.py in a private ROS domain, on this machine only."""

    def test_the_stub_drives_nothing_and_says_so(self):
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        os.environ.update(ROS_DOMAIN_ID=DOMAIN, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        got = {'/joint_states': [], '/odom': []}
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'launch.log'
            with open(log, 'w') as out:
                launch = subprocess.Popen(['ros2', 'launch', 'rover_control', 'rover.launch.py'],
                                          stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('test_rover_launch')
            node.create_subscription(JointState, '/joint_states', got['/joint_states'].append, 10)
            node.create_subscription(Odometry, '/odom', got['/odom'].append, 10)
            cmd = node.create_publisher(TwistStamped, '/cmd_vel', 10)
            try:
                deadline = time.monotonic() + 90
                while not (got['/joint_states'] and got['/odom'] and cmd.get_subscription_count()):
                    self.assertLess(time.monotonic(), deadline, log.read_text())
                    rclpy.spin_once(node, timeout_sec=0.1)
                msg = TwistStamped()
                msg.twist.linear.x = 0.5
                end = time.monotonic() + 2.5
                while time.monotonic() < end:
                    cmd.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.05)
            finally:
                node.destroy_node()
                rclpy.shutdown()
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(30)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()
            text = log.read_text()
        self.assertIn('No motors are connected: RoverHardwareStub reports zero motion', text)
        self.assertIn('No motors are connected: would send wheel_fl_joint 3.33', text)  # 0.5 m/s / 0.15 m
        joints = got['/joint_states'][-1]
        self.assertEqual(set(joints.name), JOINTS)
        self.assertEqual(set(joints.position), {0.0})
        self.assertEqual(got['/odom'][-1].pose.pose.position.x, 0.0)


if __name__ == '__main__':
    unittest.main()
```

In `rover_control/CMakeLists.txt` replace

```cmake
install(DIRECTORY include/ DESTINATION include)
install(TARGETS tank_kinematics rover_hardware_stub EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
```

with

```cmake
install(DIRECTORY include/ DESTINATION include)
install(TARGETS tank_kinematics rover_hardware_stub EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# The controllers and the launch files: rover.launch.py, and controllers.launch.py, which rover_sim's sim.launch.py
# includes too (README.md).
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
```

and replace

```cmake
  include(GoogleTest)
  gtest_discover_tests(rover_control_tests)
endif()
```

with

```cmake
  include(GoogleTest)
  gtest_discover_tests(rover_control_tests)
  # rover.launch.py with the stub (unittest, as rover_description runs its own): this package's install first on
  # AMENT_PREFIX_PATH, so that ros2 launch and pluginlib find it.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_rover_launch COMMAND "${Python3_EXECUTABLE}" -m unittest -v test_rover_launch
           WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test")
  set_tests_properties(test_rover_launch PROPERTIES TIMEOUT 180
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH};PYTHONDONTWRITEBYTECODE=1")
endif()
```

In `rover_control/package.xml` replace

```xml
  <test_depend>gtest</test_depend>
```

with

```xml
  <!-- rover.launch.py and controllers.launch.py -->
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>diff_drive_controller</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>gtest</test_depend>
  <test_depend>rclpy</test_depend>
  <test_depend>geometry_msgs</test_depend>
  <test_depend>nav_msgs</test_depend>
  <test_depend>sensor_msgs</test_depend>
```

- [ ] **Step 2: Run it to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run build 2>&1 | grep -E "cannot find|Failed|Summary" | head -3
```

Expected: the install step stops, `file INSTALL cannot find ".../rover_control/launch": No such file or directory`, and `1 package failed: rover_control`: there are no launch files yet.

- [ ] **Step 3: The launch files**

Write `rover_control/launch/controllers.launch.py`:

```python
"""
The rover's controllers, spawned into a running controller manager: rover.launch.py's on the rover, the one
gz_ros2_control runs inside Gazebo in the simulation (rover_sim's sim.launch.py includes this file):

    ros2 launch rover_control controllers.launch.py

joint_state_broadcaster and the drive controller, their parameters from config/controllers.yaml, the drive
controller's ~/cmd_vel and ~/odom on the topic contract's /cmd_vel and /odom (docs/topics.md).

Arguments: controller (the drive controller in controllers.yaml, default drive_controller), controllers (the file,
default this package's config/controllers.yaml), timeout ([s] for the controller manager to come up, default 120: a
big world loads slowly).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = Path(__file__).resolve().parent.parent  # share/rover_control, or the package's source folder


def spawner_arguments(controller, controllers, timeout):
    """The spawner's arguments for joint_state_broadcaster and the drive controller `controller`."""
    return ['joint_state_broadcaster', controller, '--param-file', controllers,
            '--controller-manager', '/controller_manager', '--controller-manager-timeout', timeout,
            f'--controller-ros-args=-r /{controller}/cmd_vel:=/cmd_vel -r /{controller}/odom:=/odom']


def spawn(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    return [Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=spawner_arguments(arg('controller'), arg('controllers'), arg('timeout')))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='drive_controller',
                              description='the drive controller in controllers.yaml'),
        DeclareLaunchArgument('controllers',
                              default_value=str(PACKAGE / 'config' / 'controllers.yaml'),
                              description="the controllers' parameters"),
        DeclareLaunchArgument('timeout', default_value='120',
                              description='[s] to wait for the controller manager'),
        OpaqueFunction(function=spawn),
    ])
```

Write `rover_control/launch/rover.launch.py`:

```python
"""
The real rover's ROS 2 stack (design spec 2026-10-09, section 6.3):

    ros2 launch rover_control rover.launch.py

robot_state_publisher on rover_description's description (urdf/rover.urdf.xacro with its ros2_control block, whose
hardware interface is rover_control's RoverHardwareStub until the motor controllers are chosen: zero motion, and in
the log the commands it would send), ros2_control_node with config/controllers.yaml, and the controllers
(launch/controllers.launch.py): joint_state_broadcaster (/joint_states) and the drive controller, which takes /cmd_vel
(geometry_msgs/TwistStamped) and publishes /odom and odom -> base_link. The simulation runs the same controllers.yaml
(rover_sim's sim.launch.py).

Arguments: controller (the drive controller in controllers.yaml, default drive_controller), hardware (the
ros2_control hardware plugin, default rover_control/RoverHardwareStub), rover_yaml (the rover's numbers, default
rover_description's config/rover.yaml; a relative path is from the folder the launch runs in).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = Path(__file__).resolve().parent.parent  # share/rover_control, or the package's source folder


def generate_launch_description():
    description = get_package_share_directory('rover_description')
    controllers = str(PACKAGE / 'config' / 'controllers.yaml')
    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(description, 'urdf', 'rover.urdf.xacro'),
        ' rover_yaml:=', PathJoinSubstitution([os.getcwd(), LaunchConfiguration('rover_yaml')]),
        ' ros2_control_plugin:=', LaunchConfiguration('hardware')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='drive_controller',
                              description='the drive controller in controllers.yaml'),
        DeclareLaunchArgument('hardware', default_value='rover_control/RoverHardwareStub',
                              description='the ros2_control hardware plugin'),
        DeclareLaunchArgument('rover_yaml',
                              default_value=os.path.join(description, 'config', 'rover.yaml'),
                              description="the rover's numbers (a relative path: from the current "
                                          'folder)'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[controllers]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(PACKAGE / 'launch' / 'controllers.launch.py')),
            launch_arguments={'controller': LaunchConfiguration('controller'),
                              'controllers': controllers}.items()),
    ])
```

In `rover_sim/tests/rosstack.py` replace

```python
import contextlib
import os
```

with

```python
import contextlib
import importlib.util
import os
```

and replace

```python
def spawner(controller="drive_controller", extra_params=(), timeout=180):
    """The spawner that rover_control's launch/controllers.launch.py runs: joint_state_broadcaster and the drive
    controller from rover_control's config/controllers.yaml (then extra_params, which override it), the drive
    controller's topics on /cmd_vel and /odom."""
    files = [arg for path in (rover_config.CONTROLLERS_YAML, *extra_params) for arg in ("--param-file", str(path))]
    return ["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster", controller, *files,
            "--controller-manager", "/controller_manager", "--controller-manager-timeout", str(timeout),
            f"--controller-ros-args=-r /{controller}/cmd_vel:=/cmd_vel -r /{controller}/odom:=/odom"]
```

with

```python
def spawner(controller="drive_controller", extra_params=(), timeout=180):
    """The spawner that rover_control's launch/controllers.launch.py runs (its spawner_arguments), with rover_control's
    config/controllers.yaml and then extra_params, which override it."""
    spec = importlib.util.spec_from_file_location("controllers_launch",
                                                  rover_config.CONTROL_DIR / "launch" / "controllers.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    arguments = module.spawner_arguments(controller, str(rover_config.CONTROLLERS_YAML), str(timeout))
    arguments[4:4] = [arg for path in extra_params for arg in ("--param-file", str(path))]  # after controllers.yaml
    return ["ros2", "run", "controller_manager", "spawner", *arguments]
```

- [ ] **Step 4: Run them to see them pass; the stack by hand**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run build 2>&1 | tail -1
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon test --packages-select rover_control --event-handlers console_cohesion+ 2>&1 | grep -E "tests passed|tests failed|test_rover_launch"
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_ros2_control 2>&1 | tail -2
```

Expected: `Summary: 3 packages finished`; `100% tests passed, 0 tests failed out of 18` (17 GoogleTest cases and `test_rover_launch`, about 20 s); `test_ros2_control` `OK` (4 tests) with the spawner from `controllers.launch.py`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_control rover_sim/tests/rosstack.py && git commit -q -F - <<'EOF'
PR 3: rover_control: controllers.launch.py and the real rover's rover.launch.py

controllers.launch.py spawns joint_state_broadcaster and the drive controller (controller:=<name>) from
controllers.yaml into a running controller manager, the drive controller's ~/cmd_vel and ~/odom on /cmd_vel and
/odom; the real rover and the simulation both use it. rover.launch.py starts robot_state_publisher on
rover_description's description (hardware:=, default the stub), ros2_control_node with controllers.yaml, and the
controllers. test_rover_launch checks both descriptions and runs rover.launch.py in a private ROS domain: the
controllers come up, /joint_states is zero, a /cmd_vel command reaches the stub, which logs it, and /odom stays at the
origin. The ROS tests of rover_sim take their spawner from controllers.launch.py.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

---

## Part D: sim.launch.py, the station on /cmd_vel, drive and the launcher through the launch

### Task 14: sim.launch.py

**Files:** Create `rover_sim/launch/sim.launch.py`, `rover_sim/tests/test_sim_launch.py`. Modify `rover_sim/CMakeLists.txt`, `rover_sim/package.xml`, `rover_sim/bridge.yaml`.

- [ ] **Step 1: The failing tests**

Write `rover_sim/tests/test_sim_launch.py`:

```python
#!/usr/bin/env python3
"""launch/sim.launch.py (design spec 2026-10-09, 6.3): its arguments and what it starts (loaded, nothing started),
the world's copy with the rover swapped for the one with ros2_control, the sensor bridge's topics, and (SmokeTest) a
run of it headless."""
import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import yaml

import rosstack
from worldfiles import SIM_DIR, WORLDS

import gzenv  # noqa: E402  (worldfiles puts rover_sim/ on the path)

SIM_LAUNCH = SIM_DIR / "launch" / "sim.launch.py"
# The topic contract (design spec 6.2, docs/topics.md) the simulation provides.
CONTRACT = {"/cmd_vel", "/odom", "/joint_states", "/tf", "/tf_static", "/robot_description", "/imu", "/gps/fix",
            "/camera/image", "/camera/depth", "/camera/camera_info", "/clock", "/ground_truth/odom"}


def load():
    """The launch file as a module."""
    spec = importlib.util.spec_from_file_location("sim_launch", SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def tearDownModule():
    rosstack.shutdown()


class Arguments(unittest.TestCase):
    def test_the_five_arguments_and_their_defaults(self):
        from launch.actions import DeclareLaunchArgument
        module = load()
        for no_display, gui in ((False, "true"), (True, "false")):
            with mock.patch.object(module, "headless", lambda: no_display):
                entities = module.generate_launch_description().entities
            defaults = {e.name: e.default_value[0].text for e in entities if isinstance(e, DeclareLaunchArgument)}
            self.assertEqual(defaults, {"world": "rover_test", "gui": gui, "station": "false",
                                        "controller": "drive_controller", "sensors": "true"})

    def test_the_source_tree_from_here_or_from_the_installed_marker(self):
        module = load()
        self.assertEqual(module.source_dir(SIM_LAUNCH), SIM_DIR)
        with tempfile.TemporaryDirectory() as tmp:
            share = Path(tmp) / "share" / "rover_sim"
            (share / "launch").mkdir(parents=True)
            (share / "source_dir").write_text(f"{SIM_DIR}\n")
            self.assertEqual(module.source_dir(share / "launch" / "sim.launch.py"), SIM_DIR)

    def test_the_installed_launch_files_find_this_tree(self):
        share = gzenv.INSTALL_DIR / "share" / "rover_sim"
        self.assertEqual((share / "source_dir").read_text().strip(), str(SIM_DIR))
        for path in ("launch/sim.launch.py", "urdf/rover_sim.urdf.xacro", "config/sim.yaml"):
            self.assertTrue((share / path).is_file(), path)


class WorldCopy(unittest.TestCase):
    def test_the_rover_include_points_at_the_ros_rover_with_its_pose(self):
        module = load()
        text = (WORLDS / "rover_test.sdf").read_text()
        copy = module.ros_world(text, "file:///tmp/rover", sensors=False)
        includes = [i for i in ET.fromstring(copy).iter("include") if i.findtext("uri") == "file:///tmp/rover"]
        self.assertEqual(len(includes), 1)
        original = [i for i in ET.fromstring(text).iter("include") if i.findtext("uri") == "model://rover"][0]
        self.assertEqual(includes[0].findtext("pose"), original.findtext("pose"))
        self.assertNotIn("gz-sim-sensors-system", copy)
        self.assertIn("gz-sim-sensors-system", module.ros_world(text, "file:///tmp/rover"))

    def test_a_world_without_exactly_one_rover_is_refused(self):
        module = load()
        with self.assertRaises(ValueError):
            module.ros_world('<sdf version="1.11"><world name="w"/></sdf>', "file:///tmp/rover")

    def test_prepare_writes_the_ros_rover_and_the_copy(self):
        module = load()
        with tempfile.TemporaryDirectory() as tmp:
            copy, urdf = module.prepare(WORLDS / "rover_test.sdf", True, Path(tmp))
            model = ET.parse(Path(tmp) / "rover" / "model.sdf").getroot().find("model")
            self.assertEqual(model.get("name"), "rover")
            self.assertIn("gz_ros2_control::GazeboSimROS2ControlPlugin", [p.get("name") for p in model.iter("plugin")])
            self.assertEqual(ET.fromstring(urdf).findtext("ros2_control/hardware/plugin"), "rover_sim/RoverSimSystem")
            self.assertIn((Path(tmp) / "rover").as_uri(), copy.read_text())
            self.assertEqual(module.world_name(copy.read_text()), "rover_test")

    def test_what_it_starts(self):
        """The server on the copy, robot_state_publisher, the controllers and the bridge; the GUI and the station
        only when asked, the station attached to the world by its name."""
        from launch import LaunchContext
        from launch.actions import ExecuteProcess, IncludeLaunchDescription
        from launch_ros.actions import Node
        module = load()
        with tempfile.TemporaryDirectory() as tmp:
            for gui, station in (("false", "false"), ("true", "true")):
                run = Path(tmp) / gui
                with mock.patch.object(module, "mkdtemp", lambda prefix: str(run.mkdir() or run)):
                    context = LaunchContext()
                    context.launch_configurations.update(world="rover_test", gui=gui, station=station,
                                                         controller="drive_controller", sensors="false")
                    actions = module.launch_setup(context)

                def text(substitutions):
                    return "".join(s.perform(context) for s in substitutions)

                def command(action):
                    return [text(part) for part in action.cmd]

                # An ExecuteProcess's name and each part of its cmd are lists of substitutions (launch normalizes
                # them), not strings: performed, they are.
                processes = {text(a.name): command(a) for a in actions
                             if isinstance(a, ExecuteProcess) and not isinstance(a, Node)}
                expected = {"gz_server"} | ({"gz_gui", "station"} if gui == "true" else set())
                self.assertEqual(set(processes), expected)
                self.assertEqual(processes["gz_server"], ["gz", "sim", "-s", "-r", str(run / "rover_test.sdf")])
                self.assertEqual([a.node_executable for a in actions if isinstance(a, Node)],
                                 ["robot_state_publisher", "parameter_bridge"])
                self.assertEqual(len([a for a in actions if isinstance(a, IncludeLaunchDescription)]), 1)
                if station == "true":
                    self.assertEqual(processes["station"][-4:], ["--world-name", "rover_test", "--wait", "120"])


class SensorBridge(unittest.TestCase):
    def test_the_contracts_sensor_topics_and_none_of_the_controllers(self):
        """bridge.yaml bridges the sensors and the simulation's own topics; /cmd_vel, /odom, /joint_states and /tf
        are the controllers' and robot_state_publisher's."""
        names = {entry["ros_topic_name"] for entry in yaml.safe_load((SIM_DIR / "bridge.yaml").read_text())}
        self.assertLessEqual({"/imu", "/gps/fix", "/camera/image", "/camera/depth", "/camera/camera_info", "/clock",
                              "/ground_truth/odom"}, names)
        self.assertEqual(names & {"/cmd_vel", "/odom", "/joint_states", "/tf"}, set())


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run them to see them fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_sim_launch 2>&1 | tail -3
```

Expected: `FAILED (errors=...)`: `No such file or directory: .../launch/sim.launch.py`, and `SensorBridge` failing on `/cmd_vel`, `/odom`, `/joint_states` and `/tf`.

- [ ] **Step 3: The launch file**

Write `rover_sim/launch/sim.launch.py`:

```python
"""
The rover in a Gazebo world with its ROS 2 stack (design spec 2026-10-09, section 6.3):

    ros2 launch rover_sim sim.launch.py world:=urc_delivery

starts
- Gazebo's server on a copy of the world in which the rover is the one with ros2_control, at the world's rover pose
  (the mission sheet's start); the worlds themselves keep model://rover, the rover of every ROS-free run (the tests,
  the referee, the maps). With gui:=true Gazebo's GUI too, as a separate process (macOS cannot run both in one);
  closing it, or the server exiting, stops everything;
- robot_state_publisher on that rover's description (urdf/rover_sim.urdf.xacro with ros2_control): /robot_description,
  /tf_static and the joints' /tf;
- the controllers of rover_control's config/controllers.yaml in the controller manager that gz_ros2_control runs
  inside Gazebo (rover_control's launch/controllers.launch.py): joint_state_broadcaster (/joint_states) and the drive
  controller (controller:=<name>), which takes /cmd_vel (geometry_msgs/TwistStamped) and publishes /odom and
  odom -> base_link;
- the bridge of the sensors and of the simulation's own topics (bridge.yaml): /imu, /gps/fix, /camera/*, /clock,
  /ground_truth/odom and more (docs/topics.md);
- with station:=true the driver station, attached to the world (pixi run drive starts this launch itself, as its
  parent: it is the station).

Arguments: world (a name in rover_sim/worlds or an .sdf path; default rover_test), gui (default true where there is
a display), station (default false), controller (default drive_controller), sensors (false: without Gazebo's Sensors
system, so nothing renders and no GPU is needed; the cameras stay dark, the IMU and GNSS work).

The simulation runs from its source tree, rover_sim/ beside rover_description/ and rover_control/, as pixi run sim
does: the generated worlds and models are there, and gzenv.py sets Gazebo's environment from it. The installed copy
of this file finds the tree through share/rover_sim/source_dir, which CMakeLists.txt writes.
"""
import os
import re
import shutil
import sys
from pathlib import Path
from tempfile import mkdtemp

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def source_dir(here=None):
    """rover_sim's source tree: the folder above this file's when it runs from there, else the one CMake recorded in
    the installed share/rover_sim/source_dir."""
    package = Path(here or __file__).resolve().parent.parent
    if (package / "gzenv.py").is_file():
        return package
    return Path((package / "source_dir").read_text().strip())


SIM_DIR = source_dir()
sys.path[:0] = [str(SIM_DIR), str(SIM_DIR / "tools")]
import fetch_data  # noqa: E402
import gen_rover  # noqa: E402
import gzenv  # noqa: E402
import rover_config  # noqa: E402
from urc import sdf  # noqa: E402

WORLDS = SIM_DIR / "worlds"
ROVER_URI = "<uri>model://rover</uri>"
SENSORS = re.compile(r'<plugin filename="gz-sim-sensors-system".*?</plugin>', re.S)
CONTROLLERS_LAUNCH = rover_config.CONTROL_DIR / "launch" / "controllers.launch.py"
STATION_WAIT = 120  # [s] for the world to come up before an attached station gives up (a big world loads slowly)


def headless(platform=None, environ=None):
    """Whether there is no display to open a window on (Linux without X11 or Wayland)."""
    platform = sys.platform if platform is None else platform
    environ = os.environ if environ is None else environ
    return platform.startswith("linux") and not (environ.get("DISPLAY") or environ.get("WAYLAND_DISPLAY"))


def resolve_world(arg):
    """The .sdf of a name in rover_sim/worlds or of a path; a world that is not here says why."""
    path = Path(arg)
    if not path.is_file():
        path = WORLDS / f"{arg}.sdf"
    if not path.is_file():
        raise FileNotFoundError(f"no world {arg!r}: " + (fetch_data.world_missing(arg) or
                                                         "give an .sdf path or a name in rover_sim/worlds"))
    return path.resolve()


def world_name(text):
    return re.search(r'<world\s+name="([^"]+)"', text).group(1)


def ros_world(text, rover_uri, sensors=True):
    """The world `text` with its one model://rover include pointed at rover_uri (its name and pose kept), and without
    Gazebo's Sensors system unless `sensors`."""
    count = text.count(ROVER_URI)
    if count != 1:
        raise ValueError(f"the world includes model://rover {count} times: sim.launch.py swaps exactly one")
    text = text.replace(ROVER_URI, f"<uri>{rover_uri}</uri>")
    return text if sensors else SENSORS.sub("", text)


def prepare(world, sensors, directory):
    """The rover with ros2_control (gen_rover: the post-processed model the equivalence test vouches for, not Gazebo's
    own conversion of /robot_description) and the world's copy, written into `directory`: (the copy, the URDF)."""
    urdf = gen_rover.build_urdf(rover_config.Variant(), ros2_control=True)
    model = directory / "rover"
    model.mkdir()
    (model / "model.sdf").write_text(gen_rover.convert(urdf))
    (model / "model.config").write_text(sdf.model_config("rover", "The rover with ros2_control.",
                                                         "rover_sim/launch/sim.launch.py"))
    copy = directory / world.name
    copy.write_text(ros_world(world.read_text(), model.as_uri(), sensors))
    return copy, urdf


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = resolve_world(arg("world"))
    directory = Path(mkdtemp(prefix=f"rover_sim_launch_{world.stem}_"))
    copy, urdf = prepare(world, arg("sensors") == "true", directory)
    env = gzenv.environment()
    server = ExecuteProcess(cmd=["gz", "sim", "-s", "-r", str(copy)], env=env, output="screen", name="gz_server")
    actions = [
        LogInfo(msg=f"world: {copy} (a copy of {world}, the rover with ros2_control)"),
        server,
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[Shutdown(reason="Gazebo's server exited")])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: shutil.rmtree(directory, ignore_errors=True))])),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf, "use_sim_time": True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(CONTROLLERS_LAUNCH)), launch_arguments={
            "controller": arg("controller"), "controllers": str(rover_config.CONTROLLERS_YAML)}.items()),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(SIM_DIR / "bridge.yaml"), "use_sim_time": True}]),
    ]
    if arg("gui") == "true":
        gui = ExecuteProcess(cmd=["gz", "sim", "-g"], env=env, output="screen", name="gz_gui")
        actions += [gui, RegisterEventHandler(OnProcessExit(target_action=gui,
                                                            on_exit=[Shutdown(reason="Gazebo's GUI closed")]))]
    if arg("station") == "true":
        actions.append(ExecuteProcess(
            cmd=[sys.executable, str(SIM_DIR / "station" / "__main__.py"), "--world-name", world_name(copy.read_text()),
                 "--wait", str(STATION_WAIT)], env=env, output="screen", name="station"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="rover_test",
                              description="a name in rover_sim/worlds or an .sdf path"),
        DeclareLaunchArgument("gui", default_value="false" if headless() else "true",
                              description="Gazebo's GUI, a separate process"),
        DeclareLaunchArgument("station", default_value="false", description="the driver station, attached"),
        DeclareLaunchArgument("controller", default_value="drive_controller",
                              description="the drive controller in rover_control's controllers.yaml"),
        DeclareLaunchArgument("sensors", default_value="true",
                              description="Gazebo's Sensors system (false: nothing renders, no GPU needed)"),
        OpaqueFunction(function=launch_setup),
    ])
```

In `rover_sim/CMakeLists.txt` replace

```cmake
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

with

```cmake
# The launch files, with the wrapper xacro and sim.yaml they read. sim.launch.py runs the simulation from this source
# tree (its generated worlds and models are here, and gzenv.py sets Gazebo's environment from it): the installed copy
# finds the tree through share/rover_sim/source_dir.
install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}/
)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/source_dir" "${CMAKE_CURRENT_SOURCE_DIR}\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/source_dir" DESTINATION share/${PROJECT_NAME})
```

In `rover_sim/package.xml` replace

```xml
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
```

with

```xml
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <!-- launch/sim.launch.py: the controllers (rover_control's controllers.yaml and controllers.launch.py), the
       description's publisher; the station's /cmd_vel -->
  <exec_depend>rover_control</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>diff_drive_controller</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <test_depend>nav_msgs</test_depend>
  <test_depend>sensor_msgs</test_depend>
```

- [ ] **Step 4: The sensor bridge without the controllers' topics**

In `rover_sim/bridge.yaml` replace

```yaml
# ros_gz_bridge config for the rover (pixi run sim-bridge).
# gz topics are set in rover_sim/config/sim.yaml (topics).
- ros_topic_name: /cmd_vel
  gz_topic_name: /model/rover/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ
- ros_topic_name: /odom
  gz_topic_name: /model/rover/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
- ros_topic_name: /ground_truth/odom
```

with

```yaml
# ros_gz_bridge config for the rover: the sensor bridge launch/sim.launch.py runs (design spec 2026-10-09, 6.2;
# docs/topics.md). gz topics are set in rover_sim/config/sim.yaml (topics). /cmd_vel, /odom, /joint_states and /tf are
# not bridged: the drive controller, joint_state_broadcaster and robot_state_publisher provide them.
- ros_topic_name: /ground_truth/odom
```

replace

```yaml
- ros_topic_name: /joint_states
  gz_topic_name: /model/rover/joint_states
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: gz.msgs.Model
  direction: GZ_TO_ROS
- ros_topic_name: /imu
```

with

```yaml
- ros_topic_name: /imu
```

and replace

```yaml
- ros_topic_name: /tf
  gz_topic_name: /model/rover/tf
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
- ros_topic_name: /clock
```

with

```yaml
- ros_topic_name: /clock
```

- [ ] **Step 5: Run them to see them pass; the launch by hand**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run sim-build 2>&1 | tail -1
cd rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_sim_launch test_launch_files 2>&1 | tail -3
```

Expected: `Summary: 1 package finished`; `Ran 9 tests` (test_sim_launch's 8, without SmokeTest, which Task 17 adds; test_launch_files 1) and `OK`. Then run the launch headless for 40 s and look at what came up (domain 64):

```bash
T=/private/tmp/claude-502/rover-pr3
cat > $T/launch_check.sh <<'EOF'
#!/bin/bash
# sim.launch.py headless on rover_test for a while; what came up. $1: the scripts' folder.
set -m
export GZ_IP=127.0.0.1 GZ_PARTITION=pr3_launch ROS_DOMAIN_ID=64 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
SRC=/Users/alarion239/Desktop/hurc_ws/src
ros2 launch "$SRC/rover_sim/launch/sim.launch.py" world:=rover_test gui:=false sensors:=false > "$1/launch.log" 2>&1 &
PID=$!
sleep 30
ros2 topic pub --times 60 --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}}}" > /dev/null
python "$1/odom_once.py"
ros2 topic list --no-daemon | sort | tr '\n' ' '; echo
kill -INT -- -$PID; wait $PID 2>/dev/null; sleep 2
pgrep -f rover_sim_launch_rover_test > /dev/null && echo "still running" || echo "stopped"
grep -E "world: |RoverSimSystem:|Configured and activated|rror" "$1/launch.log" | head -8
EOF
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run bash $T/launch_check.sh $T
```

Expected: `odom x <above 1.0> y <near 0>` (sent with `ros2 topic pub`: one of the acceptance checks); a topic list with every topic of the contract (`/camera/camera_info /camera/depth /camera/image ... /clock /cmd_vel /gps/fix /ground_truth/odom /imu /joint_states /odom /robot_description /tf /tf_static`, among others); `stopped`; the log lines `world: /.../rover_sim_launch_rover_test_.../rover_test.sdf (a copy of ...)`, `RoverSimSystem: 8 joints, 4 wheels driven ...` and `Configured and activated joint_state_broadcaster`, `... drive_controller`, and no line with `rror` but those. If `still running` is printed, stop it yourself (`pkill -INT -f rover_sim_launch_rover_test`): only what this step started.

- [ ] **Step 6: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/launch/sim.launch.py rover_sim/tests/test_sim_launch.py rover_sim/CMakeLists.txt rover_sim/package.xml rover_sim/bridge.yaml && git commit -q -F - <<'EOF'
PR 3: rover_sim: sim.launch.py, the rover in a world with its ROS 2 stack

ros2 launch rover_sim sim.launch.py world:=<name> gui:= station:= controller:= sensors:= starts Gazebo's server on a
copy of the world whose one model://rover include points at the rover with ros2_control (built per launch by
gen_rover and post-processed: not Gazebo's own conversion of /robot_description), at the same pose; the worlds keep
model://rover for every ROS-free run. Gazebo's GUI runs as a separate process (gui:=true); its closing or the server's
exit stops the launch. robot_state_publisher publishes the same URDF, rover_control's controllers.launch.py starts the
controllers in the controller manager gz_ros2_control runs, and the sensor bridge (bridge.yaml, now without /cmd_vel,
/odom, /joint_states and /tf, which the controllers and robot_state_publisher provide) runs with use_sim_time.
station:=true attaches the driver station. sensors:=false drops the Sensors system for runs without a GPU. The launch
runs the simulation from its source tree; the installed copy finds it through share/rover_sim/source_dir, and
rover_sim installs urdf/ and config/ beside launch/.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 15: The driver station publishes /cmd_vel and starts the stack through sim.launch.py

**Files:** Create `rover_sim/station/ros_link.py`. Modify `rover_sim/station/__main__.py`, `rover_sim/tests/test_station.py`.

- [ ] **Step 1: The failing tests**

In `rover_sim/tests/test_station.py` replace

```python
import asyncio
import json
```

with

```python
import asyncio
import contextlib
import io
import json
```

replace

```python
import simulate
from simulate import world_sdf
```

with

```python
import rosstack
import simulate
from simulate import world_sdf
```

replace

```python
from station import __main__ as station_main  # noqa: E402
from station import link as gz_link  # noqa: E402
```

with

```python
from station import __main__ as station_main  # noqa: E402
from station import link as gz_link  # noqa: E402
from station import ros_link  # noqa: E402


def tearDownModule():
    rosstack.shutdown()
```

insert before `class WorldArgument(unittest.TestCase):`:

```python
class RosLink(unittest.TestCase):
    """station/ros_link.py, the station's /cmd_vel (docs/topics.md): TwistStamped with a zero stamp, which the drive
    controller stamps on arrival with its own clock, in this process's private ROS domain."""

    def test_twists_go_out_on_cmd_vel_with_a_zero_stamp(self):
        from geometry_msgs.msg import TwistStamped
        probe = rosstack.Probe(topics=[("/cmd_vel", TwistStamped)])  # first: it owns rclpy's context
        self.addCleanup(probe.close)
        cmd_vel = ros_link.CmdVel()
        self.addCleanup(cmd_vel.close)
        probe.wait(lambda: cmd_vel.subscribers() > 0, 10, "subscriber of the station's /cmd_vel")
        for _ in range(5):
            cmd_vel.twist(0.6, 0.8)
            time.sleep(0.05)
        probe.wait(lambda: probe.messages["/cmd_vel"], 5, "/cmd_vel message")
        msg = probe.messages["/cmd_vel"][-1]
        self.assertEqual((msg.twist.linear.x, msg.twist.angular.z), (0.6, 0.8))
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (0, 0))

    def test_it_says_so_when_nothing_listens(self):
        cmd_vel = ros_link.CmdVel()
        self.addCleanup(cmd_vel.close)
        for subscribers, said in ((0, 1), (1, 0)):
            lines = []
            with mock.patch.object(cmd_vel, "subscribers", return_value=subscribers):
                cmd_vel.warn_if_unheard(after=0.05, out=lines.append).join(5)
            self.assertEqual(len(lines), said)
        with mock.patch.object(cmd_vel, "subscribers", return_value=0):
            lines = []
            cmd_vel.warn_if_unheard(after=0.05, out=lines.append).join(5)
        self.assertIn("nothing subscribes to /cmd_vel", lines[0])


class StartsTheStack(unittest.TestCase):
    def test_a_world_starts_with_sim_launch_in_its_own_session(self):
        path = WORLDS / "rover_test.sdf"
        with mock.patch.object(station_main.subprocess, "Popen") as popen, \
                contextlib.redirect_stdout(io.StringIO()) as out:
            station_main.start_simulation(path)
        args, kwargs = popen.call_args
        self.assertEqual(args[0][1:], ["launch", str(SIM_DIR / "launch" / "sim.launch.py"), f"world:={path}",
                                       "gui:=false", "station:=false"])
        self.assertTrue(args[0][0].endswith("ros2"))
        self.assertTrue(kwargs["start_new_session"])
        self.assertIn(str(SIM_DIR / "models"), kwargs["env"]["GZ_SIM_RESOURCE_PATH"])  # gzenv's environment
        self.assertIn("headless", out.getvalue())


```

In class `StationProcess` replace

```python
    """pixi run drive as a process, in a private transport partition: it
    stops the world it started however it is stopped (DiffDrive would keep
    the last twist forever), one world has one station, and a world that
    cannot start ends the wait at once."""
```

with

```python
    """pixi run drive as a process, in a private transport partition and ROS
    domain: the world it starts runs with the rover's ROS 2 stack
    (sim.launch.py), which the station drives through the drive controller;
    it stops all of it however it is stopped, one world has one station, and
    a world that cannot start ends the wait at once."""
```

replace

```python
    def assert_world_stops(self, path, timeout=15.0):
```

with

```python
    @staticmethod
    def copy_of(process):
        """The world's copy that sim.launch.py runs (its line "world: <copy> (a copy of ...)")."""
        found = re.search(r"world: (\S+) \(a copy of", "".join(process.output))
        return found.group(1) if found else None

    def assert_world_stops(self, path, timeout=15.0):
```

replace

```python
        first = self.station(path)
        self.wait_for(first, "driver station on", 90)
```

with

```python
        first = self.station(path)
        self.wait_for(first, "driver station on", 120)
        copy = self.copy_of(first)
        self.assertIsNotNone(copy, "".join(first.output))
```

replace

```python
        first.send_signal(signal.SIGHUP)
        self.assertEqual(first.wait(30), 0, "".join(first.output))  # stopped by the station, not by the signal
        self.assert_world_stops(path)
```

with

```python
        first.send_signal(signal.SIGHUP)
        self.assertEqual(first.wait(30), 0, "".join(first.output))  # stopped by the station, not by the signal
        self.assert_world_stops(path)
        self.assert_world_stops(copy)  # the Gazebo server sim.launch.py ran
```

replace

```python
        path = self.world()
        process = self.station(path)
        self.wait_for(process, "driver station on", 90)
        url = re.search(r"driver station on (\S+)", "".join(process.output)).group(1)
        info, states = asyncio.run(self.fly_session(url))
```

with

```python
        path = self.world()
        process = self.station(path)
        self.wait_for(process, "driver station on", 120)
        url = re.search(r"driver station on (\S+)", "".join(process.output)).group(1)
        info, states = asyncio.run(self.fly_session(url))
```

and insert before `    def test_stopped_while_the_world_loads(self):`:

```python
    def test_drives_through_the_controllers(self):
        """pixi run drive <world> starts sim.launch.py, so the station's /cmd_vel drives the rover through the drive
        controller: W held for 3 s moves it forward (ground truth on the page, /odom in this process)."""
        path = self.world()
        process = self.station(path)
        self.wait_for(process, "driver station on", 120)
        probe = rosstack.Probe()
        self.addCleanup(probe.close)
        probe.wait(lambda: probe.messages["/odom"], 60, "/odom from the drive controller")
        url = re.search(r"driver station on (\S+)", "".join(process.output)).group(1)
        before, after = asyncio.run(self.drive_session(url))
        self.assertGreater(after["x"] - before["x"], 0.5, (before, after))
        self.assertGreater(probe.messages["/odom"][-1].pose.pose.position.x, 0.5)
        copy = self.copy_of(process)
        process.send_signal(signal.SIGTERM)
        self.assertEqual(process.wait(30), 0, "".join(process.output))
        self.assert_world_stops(path)
        self.assert_world_stops(copy)

    async def drive_session(self, url):
        """Hold W for 3 s of sim time as the page does (its input at 20 Hz; sim time from the telemetry's stats, so a
        runner slower than real time drives as far): the rover's pose before and after."""
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(url + "ws") as ws:
                latest, sim = {}, [0.0]

                async def receive():
                    async for msg in ws:
                        data = json.loads(msg.data)
                        if data["t"] == "telemetry" and data.get("pose"):
                            latest.update(data["pose"])
                        if data["t"] == "telemetry" and (data.get("stats") or {}).get("sim_time") is not None:
                            sim[0] = data["stats"]["sim_time"]

                async def hold(seconds, keys, timeout=60.0):
                    start, end = sim[0], time.monotonic() + timeout
                    while sim[0] - start < seconds and time.monotonic() < end:
                        await ws.send_json({"t": "input", "keys": keys, "axes": [0, 0]})
                        await asyncio.sleep(0.05)
                    return dict(latest)

                reader = asyncio.create_task(receive())
                deadline = time.monotonic() + 30
                while not (latest and sim[0]) and time.monotonic() < deadline:  # the first pose and sim time
                    await asyncio.sleep(0.05)
                before = await hold(1.0, [])
                await hold(3.0, ["KeyW"])
                after = await hold(1.0, [])
                reader.cancel()
        return before, after

```

- [ ] **Step 2: Run them to see them fail**

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_station.RosLink test_station.StartsTheStack test_station.StationProcess > $T/task15_red.log 2>&1; grep -E "^(FAIL|ERROR):|^Ran|^FAILED|ImportError" $T/task15_red.log | head
```

Expected: an import error, `cannot import name 'ros_link' from 'station'`.

- [ ] **Step 3: The station's /cmd_vel and its start of the stack**

Write `rover_sim/station/ros_link.py`:

```python
"""The station's ROS 2 side: the drive command, geometry_msgs/TwistStamped on /cmd_vel (the topic contract,
docs/topics.md), which the drive controller takes on the simulated rover as on the real one (rover_control's
config/controllers.yaml). The header's stamp stays zero: diff_drive_controller stamps a zero-stamped command with its
own clock when it arrives (sim time in the simulation), so its cmd_vel_timeout counts from then whatever clock the
station has; a wall-clock stamp under sim time would never time out."""
import os
import threading

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.signals import SignalHandlerOptions

CMD_VEL_TOPIC = "/cmd_vel"
UNHEARD = 10.0  # [s] after the start with nothing subscribed to /cmd_vel, the station says so


class CmdVel:
    """twist(vx, wz) on /cmd_vel: the drive output (GzLink's drive) of a station in the ROS stack. It needs no executor:
    the station publishes from its own threads, and nothing here subscribes."""

    def __init__(self):
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            # No rclpy signal handlers: the station stops on Ctrl-C, SIGTERM and SIGHUP itself and sends a zero twist
            # on its way out (server.Station.shutdown); rclpy's own handlers would shut its context down first, and
            # that publish would fail ("publisher's context is invalid").
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node(f"driver_station_{os.getpid()}")
        self._publisher = self.node.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self._msg = TwistStamped()

    def twist(self, vx, wz):
        if not rclpy.ok():
            return  # rclpy is down (another owner of the context shut it): nothing can go out
        self._msg.twist.linear.x, self._msg.twist.angular.z = float(vx), float(wz)
        self._publisher.publish(self._msg)

    def subscribers(self):
        """How many subscriptions to /cmd_vel the ROS graph has (the drive controller's, once it is up)."""
        return self._publisher.get_subscription_count()

    def warn_if_unheard(self, after=UNHEARD, out=print):
        """After `after` seconds say so, through out(line), if nothing subscribes to /cmd_vel: no drive controller,
        as in a world started without sim.launch.py. Returns the timer (a daemon thread)."""
        def check():
            if self.subscribers() == 0:
                out(f"nothing subscribes to {CMD_VEL_TOPIC} after {after:.0f} s: the drive controller is not up (it "
                    "comes with sim.launch.py: pixi run sim, pixi run drive <world>), so the rover will not move")

        timer = threading.Timer(after, check)
        timer.daemon = True
        timer.start()
        return timer

    def close(self):
        self.node.destroy_node()
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()
```

In `rover_sim/station/__main__.py` replace

```python
two views only a simulation has. With a world (a path to an .sdf, or a name in
sim/worlds such as urc_autonomy) and no simulation running, it starts the
world headless (`gz sim -s -r`, the environment of sim/run.sh), stops it on
exit (Ctrl-C, kill, or closing the terminal) and exits with it; without one
it attaches to the running simulation, and spawns its cameras again when
that is restarted. One world per GZ_PARTITION and one station per world: the
rover's topics are not world-scoped.
```

with

```python
two views only a simulation has. It drives the rover through the ROS 2 stack:
/cmd_vel (geometry_msgs/TwistStamped) into the drive controller
(station/ros_link.py). With a world (a path to an .sdf, or a name in
rover_sim/worlds such as urc_autonomy) and no simulation running, it starts
the world headless with that stack (launch/sim.launch.py gui:=false
station:=false, in gzenv's environment), stops it on exit (Ctrl-C, kill, or
closing the terminal) and exits with it; without one it attaches to the
running simulation (--wait: for up to that many seconds), and spawns its
cameras again when that is restarted. One world per GZ_PARTITION and ROS
domain, and one station per world: the rover's topics are not world-scoped.
```

replace

```python
SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
```

with

```python
SIM_DIR = Path(__file__).resolve().parents[1]
SIM_LAUNCH = SIM_DIR / "launch" / "sim.launch.py"
sys.path.insert(0, str(SIM_DIR))
```

replace

```python
from station import link, server  # noqa: E402
```

with

```python
from station import link, ros_link, server  # noqa: E402
```

replace

```python
# Ctrl-C, kill, the terminal closing. Each stops the station the same way: the
# rover gets a zero twist (DiffDrive keeps the last one forever) and a world
```

with

```python
# Ctrl-C, kill, the terminal closing. Each stops the station the same way: the
# rover gets a zero twist (the drive controller would drive on until its timeout) and a world
```

replace

```python
def start_simulation(path):
    """The world headless, in sim/run.sh's environment (gzenv)."""
    gz = shutil.which("gz") or str(Path(sys.prefix) / "bin" / "gz")
    print(f"starting {path.name} headless: gz sim -s -r", flush=True)
    return subprocess.Popen([gz, "sim", "-s", "-r", str(path)], env=gzenv.environment(), start_new_session=True)
```

with

```python
def start_simulation(path):
    """The world headless with the rover's ROS 2 stack: launch/sim.launch.py (gui:=false station:=false: this station
    is its driver station), in gzenv's environment and a session of its own (stop_simulation stops all of it)."""
    ros2 = shutil.which("ros2") or str(Path(sys.prefix) / "bin" / "ros2")
    print(f"starting {path.name} headless: ros2 launch rover_sim sim.launch.py", flush=True)
    return subprocess.Popen([ros2, "launch", str(SIM_LAUNCH), f"world:={path}", "gui:=false", "station:=false"],
                            env=gzenv.environment(), start_new_session=True)
```

replace

```python
            print(f"gz sim exited (code {process.returncode}): the station stops too", flush=True)
```

with

```python
            print(f"the simulation exited (code {process.returncode}): the station stops too", flush=True)
```

replace

```python
    parser.add_argument("--host", default="127.0.0.1")
```

with

```python
    parser.add_argument("--wait", type=float, default=DISCOVERY,
                        help="attached: wait up to this many seconds for the world (sim.launch.py station:=true)")
    parser.add_argument("--host", default="127.0.0.1")
```

replace

```python
    process = None
    try:
        running = link.wait_for_world(node, DISCOVERY)
```

with

```python
    process = cmd_vel = None
    try:
        # Attached (no world given), --wait gives a world that is still starting (sim.launch.py station:=true) time.
        running = link.wait_for_world(node, DISCOVERY if path else args.wait, None if path else name)
```

replace

```python
                sys.exit(f"{path.name} did not come up within {STARTUP:.0f} s" if code is None else
                         f"gz sim exited (code {code}) before {name} came up: see its errors above")
        print(f"world: {name}", flush=True)
        gz = link.GzLink(name, node)
```

with

```python
                sys.exit(f"{path.name} did not come up within {STARTUP:.0f} s" if code is None else
                         f"the simulation exited (code {code}) before {name} came up: see its errors above")
        print(f"world: {name}", flush=True)
        cmd_vel = ros_link.CmdVel()
        cmd_vel.warn_if_unheard(out=lambda line: print(f"warning: {line}", flush=True))
        gz = link.GzLink(name, node, drive=cmd_vel)
```

and replace

```python
    finally:
        if process is not None:
            stop_simulation(process)
```

with

```python
    finally:
        if cmd_vel is not None:
            cmd_vel.close()
        if process is not None:
            stop_simulation(process)
```

- [ ] **Step 4: Run them to see them pass**

Run in the background (about 6 minutes: four stations each start the stack):

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_station.RosLink test_station.StartsTheStack test_station.StationProcess test_station.WorldArgument test_station.EyeCameraRide test_station.DrivetrainReadout > $T/task15.log 2>&1; tail -4 $T/task15.log
```

Expected: `Ran 13 tests` (RosLink 2, StartsTheStack 1, StationProcess 5, WorldArgument 3, EyeCameraRide 1, DrivetrainReadout 1) and `OK`. `test_a_world_that_does_not_load` still ends within 30 s: the Gazebo server exits, sim.launch.py shuts down with it, and the station says `the simulation exited (code ...) before rover_test came up`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/station rover_sim/tests/test_station.py && git commit -q -F - <<'EOF'
PR 3: the driver station publishes /cmd_vel and starts the stack through sim.launch.py

station/ros_link.py: the station's drive output in the ROS stack, TwistStamped on /cmd_vel with a zero stamp (the
drive controller stamps it on arrival with its own clock, so its timeout holds under sim time), and a warning when
nothing subscribes to /cmd_vel 10 s after the start. pixi run drive <world> starts launch/sim.launch.py (gui:=false
station:=false) where it started gz sim, in its own session, and keeps its stop handling; attached, --wait gives a
world that is still starting time (sim.launch.py station:=true). Fly, Map, the drivetrain readout and the cameras keep
using Gazebo. StationProcess drives the rover through the controllers from the page's protocol and checks that the
launch's Gazebo server stops with the station.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 16: pixi run sim and the launcher through sim.launch.py

**Files:** Modify `rover_sim/run.sh`, `pixi.toml`, `rover_sim/launcher/__main__.py`, `rover_sim/tests/test_launcher.py`.

- [ ] **Step 1: The failing test**

In `rover_sim/tests/test_launcher.py` replace

```python
FAKE_STATION = textwrap.dedent("""\
    import signal, sys, time
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    print("\\x1b[1;33mWarning\\x1b[0m starting", flush=True)
```

with

```python
FAKE_STATION = textwrap.dedent("""\
    import os, signal, sys, time
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    print("\\x1b[1;33mWarning\\x1b[0m starting", flush=True)
    print("ROS domain", os.environ.get("ROS_DOMAIN_ID"), os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE"), flush=True)
```

and replace

```python
            self.assertIn("Warning starting", state["log"], "terminal colours are stripped")
```

with

```python
            self.assertIn("Warning starting", state["log"], "terminal colours are stripped")
            self.assertIn(f"ROS domain {self.launcher.domain} LOCALHOST", state["log"])  # its own ROS 2 graph
            self.assertTrue(50 <= self.launcher.domain < 100)
```

Run it: `cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_launcher 2>&1 | tail -3`. Expected: `FAILED (errors=1)`: `'Launcher' object has no attribute 'domain'`.

- [ ] **Step 2: The launcher's worlds in a ROS domain of their own**

In `rover_sim/launcher/__main__.py` replace

```python
station inside the page; Exit, or closing the page, stops both. One
simulation at a time, in a GZ_PARTITION of its own (on GZ_IP 127.0.0.1 unless
set, as the tests run), so it never meets a world started from a terminal or
by the tests.
"""
```

with

```python
station inside the page; Exit, or closing the page, stops both. The station
starts the world with the rover's ROS 2 stack (launch/sim.launch.py). One
simulation at a time, in a GZ_PARTITION and a ROS domain of its own
(gzenv.private_ros_domain; on GZ_IP 127.0.0.1 unless set, as the tests run),
so it never meets a world or a ROS 2 graph started from a terminal or by the
tests; the console names both (ROS_DOMAIN_ID=<n> ros2 topic list sees it).
"""
```

replace

```python
STATION = SIM_DIR / "station" / "__main__.py"
```

with

```python
STATION = SIM_DIR / "station" / "__main__.py"
sys.path.insert(0, str(SIM_DIR))
import gzenv  # noqa: E402
```

replace

```python
    def __init__(self):
        self.partition = f"rover_launcher_{os.getpid()}"
```

with

```python
    def __init__(self):
        self.partition = f"rover_launcher_{os.getpid()}"
        self.domain = gzenv.private_ros_domain(os.getpid())
```

and replace

```python
            print(f"starting {world} (GZ_PARTITION={self.partition})", flush=True)
            self.process = await asyncio.create_subprocess_exec(
                sys.executable, "-u", str(STATION), str(WORLDS / f"{world}.sdf"),
                "--no-browser", "--port", str(free_port()),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT, limit=1 << 20,
                env={"GZ_IP": "127.0.0.1", **os.environ, "GZ_PARTITION": self.partition, "PYTHONUNBUFFERED": "1"},
                start_new_session=True)  # the terminal's Ctrl-C reaches the launcher, which stops it
```

with

```python
            print(f"starting {world} (GZ_PARTITION={self.partition}, ROS_DOMAIN_ID={self.domain})", flush=True)
            self.process = await asyncio.create_subprocess_exec(
                sys.executable, "-u", str(STATION), str(WORLDS / f"{world}.sdf"),
                "--no-browser", "--port", str(free_port()),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT, limit=1 << 20,
                env={"GZ_IP": "127.0.0.1", **os.environ, "GZ_PARTITION": self.partition, "PYTHONUNBUFFERED": "1",
                     "ROS_DOMAIN_ID": str(self.domain), "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"},
                start_new_session=True)  # the terminal's Ctrl-C reaches the launcher, which stops it
```

- [ ] **Step 3: pixi run sim through the launch; the tasks**

In `rover_sim/run.sh` replace

```bash
# Start the rover simulation: pixi run sim [world] [--follow]
# world: a path to an .sdf, or a name in sim/worlds (e.g. urc_autonomy).
# --follow: the GUI camera follows the rover (drive it with pixi run drive).
# macOS cannot run Gazebo's server and GUI in one process, so the server runs
# in the background and the GUI in the foreground; closing the GUI stops both.
```

with

```bash
# Start the rover simulation with its ROS 2 stack: pixi run sim [world] [--follow]
# world: a path to an .sdf, or a name in rover_sim/worlds (e.g. urc_autonomy).
# --follow: the GUI camera follows the rover (drive it with pixi run drive, or
# ros2 topic pub /cmd_vel). launch/sim.launch.py starts Gazebo's server and GUI
# as separate processes (macOS cannot run both in one), the rover's controllers
# and the sensor bridge; closing the GUI stops everything.
```

and replace

```bash
if [[ "$(uname)" == "Darwin" ]]; then
  gz sim -s -r "$world" &
  server=$!
  look_at_rover &
  helper=$!
  trap 'kill "$server" "$helper" 2>/dev/null || true' EXIT
  gz sim -g
else
  look_at_rover &
  trap 'kill $! 2>/dev/null || true' EXIT
  gz sim -r "$world"
fi
```

with

```bash
look_at_rover &
helper=$!
trap 'kill "$helper" 2>/dev/null || true' EXIT
ros2 launch "$here/launch/sim.launch.py" "world:=$world" gui:=true
```

In `pixi.toml` replace

```toml
sim = { cmd = "bash rover_sim/run.sh", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
drive = { cmd = "python rover_sim/station/__main__.py", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
```

with

```toml
# The world in Gazebo's GUI with the rover's ROS 2 stack (rover_sim/launch/sim.launch.py): pixi run sim [world]
# [--follow]. /cmd_vel drives it: pixi run drive, or ros2 topic pub (README, Simulation).
sim = { cmd = "bash rover_sim/run.sh", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
# The driver station: with a world it starts sim.launch.py headless, without one it attaches to the running world.
drive = { cmd = "python rover_sim/station/__main__.py", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
```

and delete

```toml
sim-bridge = "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=rover_sim/bridge.yaml"
```

(sim.launch.py runs the bridge.)

- [ ] **Step 4: Run them to see them pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_launcher test_foundations.Environment 2>&1 | tail -3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi task list 2>&1 | grep -cE "sim-bridge|control-viz"
```

Expected: `Ran 15 tests` (test_launcher 5, Environment 10; `test_run_sh_stops_without_the_world` still stops on a world that is not here before any launch) and `OK`; `0`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/run.sh rover_sim/launcher/__main__.py rover_sim/tests/test_launcher.py pixi.toml && git commit -q -F - <<'EOF'
PR 3: pixi run sim and the launcher's worlds run the rover's ROS 2 stack

pixi run sim (run.sh) runs sim.launch.py gui:=true, Gazebo's server and GUI as separate processes on every platform,
keeping --follow; the launcher's tiles start the station, which starts sim.launch.py, in a GZ_PARTITION and a ROS
domain of their own (gzenv.private_ros_domain, LOCALHOST discovery), and the console names both. The sim-bridge task
is gone: sim.launch.py runs the bridge.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

---

## Part E: the smoke test, CI, documents

### Task 17: The launch smoke test, and CI

**Files:** Modify `rover_sim/tests/test_sim_launch.py`, `.github/workflows/ci.yml`.

- [ ] **Step 1: The test**

In `rover_sim/tests/test_sim_launch.py` replace

```python
import importlib.util
import tempfile
import unittest
```

with

```python
import importlib.util
import re
import subprocess
import tempfile
import unittest
```

and insert before `if __name__ == "__main__":`:

```python
class SmokeTest(unittest.TestCase):
    """sim.launch.py headless on rover_test (gui:=false sensors:=false: no GPU needed, as on CI's Linux runner), in this
    process's private gz partition and ROS domain (design spec 2026-10-09, 8: the smoke test): a /cmd_vel command moves
    /odom, the topic contract's topics are there, the drive controller's timeout brakes the rover, and SIGINT stops it
    all."""

    def test_cmd_vel_moves_odom(self):
        with tempfile.TemporaryDirectory(prefix="rover_smoke_") as tmp:
            log = Path(tmp) / "launch.log"
            launch = rosstack.start(["ros2", "launch", SIM_LAUNCH, "world:=rover_test", "gui:=false", "sensors:=false"],
                                    log)
            probe = rosstack.Probe()
            try:
                probe.wait(lambda: probe.messages["/odom"] and probe.cmd.get_subscription_count() > 0, 180,
                           "/odom and a subscriber of /cmd_vel", [log])
                start = probe.messages["/odom"][-1].pose.pose.position.x
                # On sim time (/odom's stamps), so that a runner slower than real time drives as far as this Mac.
                probe.drive_for(0.5, 0.0, sim_seconds=5.0, timeout=90)
                driven = probe.messages["/odom"][-1].pose.pose.position.x - start
                # No command for 2 s of sim time: the drive controller's 0.5 s timeout, then its 0.42 s ramp, brake.
                probe.drive_for(0.0, 0.0, sim_seconds=2.0, timeout=60, publish=False)
                coasting = probe.messages["/odom"][-1].twist.twist.linear.x
                topics = probe.topics()
            finally:
                probe.close()
                rosstack.stop(launch)
                text = log.read_text(errors="replace")
        copy = re.search(r"world: (\S+) \(a copy of", text)
        self.assertIsNotNone(copy, text[-3000:])
        self.assertGreater(driven, 1.5, text[-3000:])  # 0.5 m/s for 5 s of sim time, less the ramp and the slip
        self.assertLess(abs(coasting), 0.05)
        self.assertLessEqual(CONTRACT, topics)
        self.assertNotEqual(subprocess.run(["pgrep", "-f", copy.group(1)], capture_output=True).returncode, 0,
                            "Gazebo's server still runs")


```

Run it:

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_sim_launch.SmokeTest 2>&1 | tail -3
```

Expected: `Ran 1 test in <about 40>s` and `OK`. (It was written after the launch it tests, so it passes at once; Task 14's hand-run showed the same facts.)

- [ ] **Step 2: CI**

In `.github/workflows/ci.yml` replace

```yaml
    env:
      GZ_IP: 127.0.0.1
```

with

```yaml
    env:
      GZ_IP: 127.0.0.1
      # ROS 2 discovery stays on the runner's loopback; each test process also takes a ROS domain of its own.
      ROS_AUTOMATIC_DISCOVERY_RANGE: LOCALHOST
```

and replace

```yaml
      - run: pixi run test
```

with

```yaml
      # The colcon tests (rover.launch.py on the hardware stub among them) and the simulation's suite, which runs
      # RoverSimSystem inside Gazebo with the controllers and sim.launch.py headless (the smoke test): no GPU needed.
      - run: pixi run test
```

Check the file: `cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run python -c "import yaml; yaml.safe_load(open('.github/workflows/ci.yml')); print('ok')"`. Expected: `ok`.

- [ ] **Step 3: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/tests/test_sim_launch.py .github/workflows/ci.yml && git commit -q -F - <<'EOF'
PR 3: the sim.launch.py smoke test, in pixi run test and CI

test_sim_launch.SmokeTest runs sim.launch.py headless on rover_test (gui:=false sensors:=false, no GPU) in the test
process's private gz partition and ROS domain: /cmd_vel moves /odom by more than 1.5 m in 5 s of sim time (timed by
/odom's stamps, so a runner slower than real time is not failed), the topic contract's topics are all there, the drive
controller's timeout brakes the rover, and SIGINT stops Gazebo's server too. CI keeps ROS 2 discovery on the runner's
loopback.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 18: The full suite, the colcon tests, and the acceptance by hand

**Files:** none in SRC; `$T/suite_final.log`, `$T/tests_after.txt`, `$T/rover_check.sh`.

- [ ] **Step 1: Everything**

Run in the background (about 18 minutes):

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run test > $T/suite_final.log 2>&1; echo "exit $?" >> $T/suite_final.log
```

Then:

```bash
T=/private/tmp/claude-502/rover-pr3
grep -E "tests passed|tests failed|^Ran |^OK|^FAILED|^exit" $T/suite_final.log
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run python $T/list_tests.py rover_sim/tests > $T/tests_after.txt 2>/dev/null
comm -23 $T/tests_before.txt $T/tests_after.txt | wc -l; comm -13 $T/tests_before.txt $T/tests_after.txt | wc -l
git status --porcelain
```

Expected: `100% tests passed, 0 tests failed out of 18` (rover_control), `... out of 5` (rover_description), `... out of 5` (rover_sim); `Ran 590 tests` (Task 10's 577, plus test_sim_launch 9, test_station's RosLink 2, StartsTheStack 1 and the new StationProcess test), `OK (skipped=9, expected failures=2)`, `exit 0`; `0` tests gone and `39` new; a clean tree. Compare with Task 1's baseline lines: every test that passed there passes here.

- [ ] **Step 2: The acceptance by hand: /cmd_vel from `ros2 topic pub`, the real rover's launch with the stub**

Task 14's `$T/launch_check.sh` drove the simulated rover with `ros2 topic pub`; run it once more on the finished branch:

```bash
T=/private/tmp/claude-502/rover-pr3
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run bash $T/launch_check.sh $T 2>&1 | grep -E "odom x|stopped|still running"
```

Expected: `odom x <above 1.0> ...` and `stopped`. Then the real rover's launch, domain 65:

```bash
T=/private/tmp/claude-502/rover-pr3
cat > $T/rover_check.sh <<'EOF'
#!/bin/bash
# rover.launch.py with the hardware stub for a while: the controllers, a /cmd_vel, the stub's log. $1: the scripts' folder.
set -m
cd /Users/alarion239/Desktop/hurc_ws && source install/local_setup.bash
export ROS_DOMAIN_ID=65 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch rover_control rover.launch.py > "$1/rover.log" 2>&1 &
PID=$!
sleep 12
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers | grep -oE "name='[a-z_]+'|state='[a-z]+'" | tr '\n' ' '; echo
ros2 topic pub --times 40 --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}}}" > /dev/null
kill -INT -- -$PID; wait $PID 2>/dev/null
grep -oE "No motors are connected: (RoverHardwareStub reports zero motion|would send wheel_fl_joint [0-9.]+)" "$1/rover.log" | sort -u
EOF
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi run bash $T/rover_check.sh $T
```

Expected: `name='joint_state_broadcaster' state='active' name='drive_controller' state='active'` (in either order); then `No motors are connected: RoverHardwareStub reports zero motion` and lines `No motors are connected: would send wheel_fl_joint <value>`, the last `3.33`. Nothing to commit. CI on Linux and Windows runs once the user pushes (Task 21).

### Task 19: The documents

**Files:** Create `docs/topics.md`. Replace `rover_control/README.md`. Modify `README.md`, `rover_sim/README.md`, `rover_description/README.md`, `docs/sim/manual.md`, `docs/sim/design-notes.md`.

- [ ] **Step 1: The topic contract**

Write `docs/topics.md`:

```markdown
# Topic contract

The rover's programs talk to the rover, simulated or real, through these ROS 2 topics (design spec `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`, section 6.2). The names are the same on both, so a program that drives the simulated rover drives the real one.

| Topic | Type | Publisher | What it carries |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/TwistStamped | operator (the driver station), autonomy | The drive command: `twist.linear.x` [m/s], `twist.angular.z` [rad/s, left positive]. The drive controller limits it to 3 m/s and 7.5 rad/s and ramps it at 1.2 m/s² and 3 rad/s²; a command older than 0.5 s counts as zero, so publish at a rate (the station sends 20 Hz). Leave `header.stamp` zero: the controller stamps the command on arrival with its own clock. A stamp from the wall clock under the simulation's sim time would never time out |
| `/odom` | nav_msgs/Odometry | drive controller | Wheel odometry, `odom` → `base_link`, 50 Hz. Without slip, so it over-reports a turn in place by 1 / the ground's turn ratio (2.65 on packed ground) |
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster | The wheels (position, velocity, and effort: the motor current [A]), the rockers and the camera head (position, velocity); 100 Hz |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | drive controller (`odom` → `base_link`), robot_state_publisher (the joints and the frames `imu`, `gnss`, `camera`, `led`) | |
| `/robot_description` | std_msgs/String | robot_state_publisher | The URDF (transient local) |
| `/imu` | sensor_msgs/Imu | Gazebo bridge / IMU driver | 100 Hz, frame `base_link` |
| `/gps/fix` | sensor_msgs/NavSatFix | Gazebo bridge / GNSS driver | 10 Hz, frame `gnss` |
| `/camera/image`, `/camera/depth`, `/camera/camera_info` | sensor_msgs/Image, CameraInfo | Gazebo bridge / camera driver | The RGB-D camera on the pan-tilt head, 1280×720, 15 Hz, frame `camera`; in the simulation also `/camera/points` (PointCloud2) |
| `/clock` | rosgraph_msgs/Clock | Gazebo bridge (simulation only) | Sim time: the simulation's nodes run with `use_sim_time`, the controllers too (their clock is the controller manager's, which ros2_control also hands the simulated hardware). Without it nothing drives: a zero-stamped command would be stale on arrival, and the drivetrain's watchdog stops the motors when the clock stops |
| `/ground_truth/odom` | nav_msgs/Odometry | Gazebo bridge (simulation only) | The true pose, `world` → `base_link`, 50 Hz |

Joints: `wheel_fl_joint`, `wheel_rl_joint`, `wheel_fr_joint`, `wheel_rr_joint`, `rocker_left_joint`, `rocker_right_joint`, `camera_pan_joint`, `camera_tilt_joint`. The controllers are `rover_control/config/controllers.yaml`'s, the same on both rovers (`rover_control/README.md`): `drive_controller` (diff_drive_controller, left = FL + RL, right = FR + RR) and `joint_state_broadcaster`.

On the simulated rover (`pixi run sim` running), in `pixi shell`:

    ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}, angular: {z: 0.2}}}"
    ros2 topic echo /odom

## The simulation's own topics

`rover_sim/launch/sim.launch.py` also bridges these (`rover_sim/bridge.yaml`): `/drivetrain` (std_msgs/String, the drivetrain's state as JSON at 50 Hz: each wheel's setpoint, speed, motor current, torque, voltage, saturation, slip, load, ground and dig factor; `cmd`, the twist the setpoints amount to), `/camera/pan` and `/camera/tilt` (Float64 target angles [rad], to the head), `/led` (the status LED: `red`, `blue`, `green`, `off`), and the referee's and the mission props' topics (`/urc/score`, `/urc/radio`, `/urc/display`, `/urc/launch_key`, `/astronaut/command`, `/astronaut/speech`, `/lander/joint_states`; `docs/sim/manual.md`).

In Gazebo only (`rover_sim/config/sim.yaml`, topics): `/model/rover/wheel_cmd` (gz.msgs.Double_V, the four wheels' speed setpoints in rad/s, fl rl fr rr), where the simulation's tests and tools drive the rover without ROS (`rover_sim/wheel_cmd.py`), ignored while ros2_control drives the wheels; the drivetrain's own no-slip odometry `/model/rover/odometry` and `/model/rover/tf`; `/model/rover/joint_states`; `/model/rover/ground_truth`; the driver station's cameras.
```

- [ ] **Step 2: rover_control's README**

Write `rover_control/README.md`:

````markdown
# Rover Control
The rover's drive, the same on the real and the simulated rover (design spec `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`, section 6). Programs publish `/cmd_vel` (geometry_msgs/TwistStamped); the controllers turn it into each wheel's velocity command, which ros2_control hands to the hardware: on the rover a stub until the motor controllers are chosen, in the simulation `rover_sim`'s `RoverSimSystem`. The topics are in [docs/topics.md](../docs/topics.md).

| Path | What it is |
|---|---|
| `config/controllers.yaml` | The controller manager (100 Hz) and its controllers: `joint_state_broadcaster` (`/joint_states`) and `drive_controller`, diff_drive_controller for skid steer (left FL + RL, right FR + RR; `/odom` and `odom` → `base_link`; 3 m/s and 7.5 rad/s, ramped at 1.2 m/s² and 3 rad/s²; a 0.5 s command timeout). Its geometry is `rover_description/config/rover.yaml`'s, written out; a test checks the two against each other |
| `launch/controllers.launch.py` | Spawns both into a running controller manager, the drive controller's topics on `/cmd_vel` and `/odom`; `rover.launch.py` and `rover_sim`'s `sim.launch.py` include it |
| `launch/rover.launch.py` | The real rover: robot_state_publisher on `rover_description`'s description, `ros2_control_node` with `controllers.yaml`, and the controllers |
| `include/rover_control/rover_hardware_stub.hpp`, `src/rover_hardware_stub.cpp`, `rover_hardware_stub.xml` | `rover_control/RoverHardwareStub`, the real rover's ros2_control hardware until the motor controllers are chosen: the description's interfaces, zero motion, and in the log the wheel commands it would send |
| `include/rover_control/tank_kinematics.hpp`, `src/tank_kinematics.cpp` | `tank_kinematics`: skid-steer inverse and forward kinematics, a saturation that keeps the turn radius, and `LoadRoverYaml`, the geometry from `rover.yaml` (a node gives the same numbers from its ROS parameters) |
| `test/` | GoogleTest (the kinematics, `controllers.yaml` against `rover.yaml`, the stub) and `test_rover_launch.py` (`rover.launch.py` with the stub, in a private ROS domain): `pixi run colcon-test` |

### Run the real rover's stack
From the workspace root, in `pixi shell`, after `colcon build`:

```bash
source install/local_setup.bash
ros2 launch rover_control rover.launch.py        # controller:=<name>, hardware:=<plugin>, rover_yaml:=<file>
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}}}"
```

The stub warns that no motors are connected and logs the commands it would send (`would send wheel_fl_joint 2.00, ...`, at most once a second); `/joint_states` and `/odom` stay at zero.

### Change the drive controller
Change `drive_controller`'s `type` in `config/controllers.yaml`, or add another controller beside it and start it with `controller:=<name>` (`rover.launch.py`, and `sim.launch.py` in the simulation). Its parameters go under its name in the same file. Keep the geometry `rover.yaml`'s: `test_controllers_yaml.cpp` checks `drive_controller`'s.

### When the motor controllers are chosen
Write their ros2_control hardware interface (a `hardware_interface::SystemInterface` with the stub's interfaces), export it with pluginlib, and give it as `hardware:=<package>/<Class>`, or make it `rover_description`'s default (`urdf/rover.urdf.xacro`'s `ros2_control_plugin`).

The swerve design's kinematics library (`rover_driver`), `rover_state` and `viz.py` were removed when the drive became skid steer; they are in git history before PR 3.
````

- [ ] **Step 3: The README, rover_sim's and rover_description's**

In `README.md` replace

```markdown
2. `pixi run sim` builds the simulation's plugins, generates the rover and the worlds and opens Gazebo on the test ground (`pixi run sim urc_autonomy` opens a URC mission world)
3. `pixi run drive urc_autonomy` drives the rover from the browser; `pixi run launcher` shows every world as a tile that starts it with its driver station
```

with

```markdown
2. `pixi run sim` builds the simulation's plugins, generates the rover and the worlds and opens Gazebo on the test ground with the rover's ROS 2 stack: `rover_sim/launch/sim.launch.py` runs the controllers, robot_state_publisher and the sensor bridge (`pixi run sim urc_autonomy` opens a URC mission world)
3. `pixi run drive urc_autonomy` drives the rover from the browser; `pixi run launcher` shows every world as a tile that starts it with its driver station. Programs drive the simulated rover as they will drive the real one, through `/cmd_vel` (geometry_msgs/TwistStamped) into the controllers: in `pixi shell`, `ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}}}"`. The topics are listed in [docs/topics.md](docs/topics.md)
```

replace

```markdown
### Tests
- `pixi run test` runs the colcon tests of `rover_description` (xacro and `check_urdf` on the description, the team's linters), `rover_control` and `rover_sim` (C++) and the simulation's suite: about 15 minutes on an Apple M4. Tests that render skip on a machine without a GPU and say why
```

with

```markdown
### The real rover
`ros2 launch rover_control rover.launch.py` (in a shell that sourced `install/`, Usage above) starts the real rover's stack: robot_state_publisher on `rover_description`'s description and the same controllers as the simulation (`rover_control/config/controllers.yaml`). Until the motor controllers are chosen its hardware interface is a stub that reports zero motion and logs the wheel commands it would send (`rover_control/README.md`).

### Tests
- `pixi run test` runs the colcon tests of `rover_description` (xacro and `check_urdf` on the description, the team's linters), `rover_control` (GoogleTest, and `rover.launch.py` on the hardware stub) and `rover_sim` (C++, `RoverSimSystem` among them) and the simulation's suite, which also runs `RoverSimSystem` inside Gazebo with the controllers and `sim.launch.py` headless (the smoke test): about 18 minutes on an Apple M4. Tests that render skip on a machine without a GPU and say why
```

replace

```markdown
- `rover_sim`: Contains SDFs, URDFs, launch configurations, and any other tools that help with simulating the rover in Gazebo. Today it holds the URC simulation: Gazebo plugins for the drivetrain and motors, the rocker differential and the driver station's cameras; the rover's Gazebo model, built from `rover_description` and `config/sim.yaml` (how the simulation models the rover) with `urdf/rover_sim.urdf.xacro`; the URC 2027 mission worlds and their generator; a browser driver station, a launcher and a referee (macOS and Linux)
- `rover_control`: The rover's drive code. For now the swerve design's kinematics library `rover_driver` with its tests and tools, superseded by tank drive; the ros2_control work replaces it
```

with

```markdown
- `rover_sim`: Contains SDFs, URDFs, launch configurations, and any other tools that help with simulating the rover in Gazebo. Today it holds the URC simulation: Gazebo plugins for the drivetrain and motors, the rocker differential and the driver station's cameras; `RoverSimSystem`, the simulated rover's ros2_control hardware, which hands the controllers' wheel commands to the motor model; `launch/sim.launch.py`, the rover in a world with its ROS 2 stack; the rover's Gazebo model, built from `rover_description` and `config/sim.yaml` (how the simulation models the rover) with `urdf/rover_sim.urdf.xacro`; the URC 2027 mission worlds and their generator; a browser driver station, a launcher and a referee (macOS and Linux)
- `rover_control`: The rover's drive: the controllers both the real and the simulated rover run (`config/controllers.yaml`: diff_drive_controller and joint_state_broadcaster, started by `launch/controllers.launch.py`), the real rover's `launch/rover.launch.py` with its ros2_control hardware stub until the motor controllers are chosen, and `tank_kinematics`, a skid-steer kinematics library that reads `rover_description`'s `rover.yaml`
```

and replace

```markdown
Beside the packages, `docs/` holds the design specs and plans (`docs/superpowers`), the simulation's manual (`docs/sim`) and the research behind it (`docs/research`).
```

with

```markdown
Beside the packages, `docs/` holds the topic contract (`docs/topics.md`), the design specs and plans (`docs/superpowers`), the simulation's manual (`docs/sim`) and the research behind it (`docs/research`).
```

In `rover_sim/README.md` replace

```markdown
pixi run sim                    # build the plugins, generate rover and worlds, open Gazebo (rover_test)
```

with

```markdown
pixi run sim                    # build the plugins, generate rover and worlds, open Gazebo with the ROS 2 stack (rover_test)
```

replace

```markdown
| `gzenv.py`, `run.sh`, `bridge.yaml` | The Gazebo environment (the one place it is set), `pixi run sim`, the ROS 2 bridge's topics |
```

with

```markdown
| `ros2_control/`, `rover_sim_system.xml` | `RoverSimSystem`, the simulated rover's ros2_control hardware: gz_ros2_control loads it inside Gazebo, and it hands each wheel's velocity command to the drivetrain's motor model (`plugins/rover_components.hh`) and reports the joints and the motor currents |
| `launch/`, `gzenv.py`, `run.sh`, `bridge.yaml` | `sim.launch.py` (below), the Gazebo environment (the one place it is set), `pixi run sim`, the sensor bridge's topics |
| `wheel_cmd.py` | The ROS-free way to drive the simulated rover: a twist as the wheel setpoints the drivetrain takes on `/model/rover/wheel_cmd` (the tests and tools) |
```

replace

```markdown
### Launch Files
- `gazebo.launch.py`: Start Gazebo using the correct method for your operating system. Set the `world` argument to change which SDF is loaded
```

with

```markdown
### Launch Files
- `sim.launch.py`: the rover in a world with its ROS 2 stack: `ros2 launch rover_sim sim.launch.py world:=<name> gui:=true|false station:=true|false controller:=<name> sensors:=true|false`. Gazebo's server (and its GUI as a separate process), robot_state_publisher, the controllers of `rover_control/config/controllers.yaml` in the controller manager gz_ros2_control runs inside Gazebo, the sensor bridge (`bridge.yaml`) and, with `station:=true`, the driver station. `pixi run sim`, `pixi run drive <world>` and the launcher start it. The worlds include the ROS-free rover (`model://rover`, which the tests drive through `/model/rover/wheel_cmd`); the launch runs a copy of the world whose rover has ros2_control, at the same pose
- `gazebo.launch.py`: Start Gazebo using the correct method for your operating system. Set the `world` argument to change which SDF is loaded
```

and replace

```markdown
Open these worlds with `pixi run sim <world>`: `gazebo.launch.py` cannot load them yet (the installed package has neither the rover's model nor the worlds, and nothing sets Gazebo's resource path from `install/`).
```

with

```markdown
Open these worlds with `pixi run sim <world>` or `sim.launch.py`; `gazebo.launch.py` cannot load them (it sets neither Gazebo's resource path nor its plugin path, which `gzenv.py` sets for `sim.launch.py`).
```

In `rover_description/README.md` replace

```markdown
The numbers are placeholders until the mechanical team has real ones: the geometry is the swerve design's (`rover_control`'s `config.hpp`, which keeps its own copy until the drive code reads this file), the motors a validated simulation set.
```

with

```markdown
The numbers are placeholders until the mechanical team has real ones: the geometry is the swerve design's, the motors a validated simulation set. `rover_control`'s kinematics library reads this file (`LoadRoverYaml`), and `rover_control/config/controllers.yaml` writes the drive controller's geometry out, checked against it by a test.
```

replace

```markdown
Its hardware plugin is the argument `ros2_control_plugin`, by default `rover_control/RoverHardwareStub`, which a later pull request adds; nothing loads the block yet |
```

with

```markdown
Its hardware plugin is the argument `ros2_control_plugin`: by default `rover_control/RoverHardwareStub`, the real rover's stub (`rover_control`'s `rover.launch.py` loads it); the simulation sets `rover_sim/RoverSimSystem` |
```

and replace

```markdown
3. `pixi run test`. A changed number fails `rover_sim`'s `test_equivalence`, which holds the rover as it was before this package described it: refreeze its fixtures in the same commit and say why (`rover_sim/tests/fixtures/rover_model/README.md`).
```

with

```markdown
3. `pixi run test`. A changed number fails `rover_sim`'s `test_equivalence`, which holds the rover as it was before this package described it: refreeze its fixtures in the same commit and say why (`rover_sim/tests/fixtures/rover_model/README.md`). A changed wheel radius, track or top speed also fails `rover_control`'s `test_controllers_yaml`: write the new numbers into `rover_control/config/controllers.yaml`.
```

- [ ] **Step 4: The simulation's manual and design notes**

In `docs/sim/manual.md` replace

```text
pixi run sim                    # build the plugins (colcon), regenerate rover and worlds, open Gazebo (rover_test)
pixi run drive urc_autonomy     # or drive from the browser: starts the world headless, opens the station page
pixi run launcher               # every world as a tile: a click starts it with its station
pixi run sim-bridge             # second terminal: ROS 2 <-> Gazebo topics
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

with

```text
pixi run sim                    # build the plugins (colcon), regenerate rover and worlds, open Gazebo with the ROS 2 stack (rover_test)
pixi run drive urc_autonomy     # or drive from the browser: starts the world and the stack headless, opens the station page
pixi run launcher               # every world as a tile: a click starts it with its station
pixi run ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.3}, angular: {z: 0.2}}}'
```

replace

```markdown
The rover stops 0.5 s after its last command (the drivetrain's command
timeout), so publish `cmd_vel` at a rate, as the line above and the station
do. Gazebo GUI's Teleop plugin (⋮ menu → Teleop, topic
`/model/rover/cmd_vel`) sends one message per button press: in button mode
the rover moves for 0.5 s; use its keyboard mode, or build the rover with
`cmd_timeout: 0.0` (`rover_sim/config/sim.yaml`, drivetrain), which holds the last command.

On macOS Gazebo cannot run its server and GUI in one process, so `rover_sim/run.sh`
starts the server in the background and the GUI in the foreground; closing
the GUI stops both.
```

with

```markdown
The rover drives through the controllers, as the real rover will
(`docs/topics.md`): `/cmd_vel` (geometry_msgs/TwistStamped) into the drive
controller (`rover_control/config/controllers.yaml`), which ramps it and
brakes 0.5 s after the last command (its `cmd_vel_timeout`), so publish at a
rate, as the line above and the station do; behind it the drivetrain's own
watchdog stops the motors 0.5 s after their last setpoint. Gazebo GUI's Teleop
plugin speaks Gazebo's twist, which nothing takes now; use the station or
`ros2 topic pub`.

`rover_sim/run.sh` runs `rover_sim/launch/sim.launch.py` (`ros2 launch
rover_sim sim.launch.py world:=<name> gui:= station:= controller:=
sensors:=`): Gazebo's server and its GUI as separate processes (macOS cannot
run both in one; closing the GUI stops everything), robot_state_publisher,
the controllers in the controller manager gz_ros2_control runs inside
Gazebo, and the sensor bridge (`bridge.yaml`). The worlds include the ROS-free
rover (`model://rover`); the launch runs a copy of the world with the rover
that has ros2_control, at the same pose.

The controllers' clock is the bridged `/clock` (sim time): without the
bridge nothing drives. Harmless lines in the launch's log:
`Velocity command timed out. Braking.` once a second while no command comes,
`Received TwistStamped with zero timestamp` once (the stamp is left zero on
purpose, `docs/topics.md`), once each `Executor is not available during
hardware component initialization for 'rover'` and robot_state_publisher's
KDL warning about the root link's inertia, and from Gazebo's GUI
`Failed to create component type [...]` for the rover's own components
(`WheelSetpoint`, `WheelMotorCurrent`), which the GUI does not load.
```

replace

```markdown
| `bridge.yaml` | ROS 2 bridge topics |
| `run.sh`, `gzenv.py` | `pixi run sim`: server and GUI, camera on the rover; `gzenv.py` is the one place the Gazebo environment is set |
| `station/` | Driver station: `__main__.py` (CLI), `server.py` (web app), `link.py` (gz-transport), `drive.py` (keys → twist, deadman, the look, the fly camera), `video.py` (frames → JPEG), `minimap.py` (hillshade), `static/` (page) |
```

with

```markdown
| `bridge.yaml` | The sensor bridge's topics (sim.launch.py runs it) |
| `run.sh`, `gzenv.py`, `launch/sim.launch.py` | `pixi run sim`: sim.launch.py with the GUI, camera on the rover; `gzenv.py` is the one place the Gazebo environment is set |
| `ros2_control/`, `rover_sim_system.xml` | `RoverSimSystem`, the simulated rover's ros2_control hardware, which gz_ros2_control loads inside Gazebo |
| `wheel_cmd.py` | A twist as the wheel setpoints the drivetrain takes without ROS (the tests and tools) |
| `station/` | Driver station: `__main__.py` (CLI), `server.py` (web app), `link.py` (gz-transport), `ros_link.py` (`/cmd_vel`), `drive.py` (keys → twist, deadman, the look, the fly camera), `video.py` (frames → JPEG), `minimap.py` (hillshade), `static/` (page) |
```

replace

```markdown
| ROS 2 (via `sim-bridge`) | Gazebo | Type |
|---|---|---|
| `/cmd_vel` → | `/model/rover/cmd_vel` | Twist (vx, wz); a command older than 0.5 s counts as zero (`sim.yaml` drivetrain `cmd_timeout`, wall clock) |
| `/odom` | `/model/rover/odometry` | wheel odometry, `odom` → `base_link`, 50 Hz; no slip, so it over-reports a turn in place (2.65× on packed ground) |
| `/ground_truth/odom` | `/model/rover/ground_truth` | true pose, `world` → `base_link` (see Known limitations: an occasional yaw-rate spike) |
| `/drivetrain` | `/model/rover/drivetrain` | String, JSON at 50 Hz: per wheel setpoint, speed, current, torque, voltage, saturation, slip, load, ground, dig factor |
| `/joint_states` | `/model/rover/joint_states` | 2 rockers, 4 wheels, camera pan and tilt |
```

with

```markdown
The rover's ROS 2 topics, the same on the real rover, are the topic contract
(`docs/topics.md`): `/cmd_vel` into the drive controller, `/odom` and
`/joint_states` from the controllers, `/tf` from them and robot_state_publisher.
`sim.launch.py` bridges these from Gazebo:

| ROS 2 (sim.launch.py's bridge) | Gazebo | Type |
|---|---|---|
| `/ground_truth/odom` | `/model/rover/ground_truth` | true pose, `world` → `base_link` (see Known limitations: an occasional yaw-rate spike) |
| `/drivetrain` | `/model/rover/drivetrain` | String, JSON at 50 Hz: per wheel setpoint, speed, current, torque, voltage, saturation, slip, load, ground, dig factor; `cmd`, the twist the setpoints amount to |
```

replace

```markdown
| `/tf` | `/model/rover/tf` | `odom` → `base_link` |
| `/clock` | `/clock` | sim time |
```

with

```markdown
| `/clock` | `/clock` | sim time |
```

replace

```markdown
| – | `/model/rover/link/rocker_{left,right}/particle_emitter/dust_r{l,r}/cmd` | ParticleEmitter: the drivetrain sets the dust behind the rear wheels (10 Hz); only with `sim.yaml`'s `dust` on, off by default (no dust, nothing on these topics) |
```

with

```markdown
| – | `/model/rover/wheel_cmd` | Double_V, the wheels' speed setpoints [rad/s], fl rl fr rr: the ROS-free way to drive (the tests and tools, `wheel_cmd.py`); ignored while ros2_control drives the wheels |
| – | `/model/rover/odometry`, `/model/rover/tf` | the drivetrain's own no-slip odometry, `odom` → `base_link`, 50 Hz |
| – | `/model/rover/joint_states` | the joints as Gazebo has them |
| – | `/model/rover/link/rocker_{left,right}/particle_emitter/dust_r{l,r}/cmd` | ParticleEmitter: the drivetrain sets the dust behind the rear wheels (10 Hz); only with `sim.yaml`'s `dust` on, off by default (no dust, nothing on these topics) |
```

replace

```markdown
`rover_sim/config/sim.yaml`: the drivetrain's settings (its twist input, the
stand-in motor controller's PI tuning, the motor model's substeps), the
contact model, dig-in, the visual cues, dust and the tyre model. Edit either,
```

with

```markdown
`rover_sim/config/sim.yaml`: the drivetrain's settings (its setpoint watchdog,
the stand-in motor controller's PI tuning, the motor model's substeps), the
ROS-free helpers' ramp, the contact model, dig-in, the visual cues, dust and
the tyre model; the drive controller's are `rover_control/config/controllers.yaml`.
Edit either,
```

replace

```markdown
- `world`: a path to an `.sdf` or a name in `rover_sim/worlds`. With a world and
  nothing running, the station starts `gz sim -s -r <world>` with `run.sh`'s
  environment, stops it on exit and exits when it exits. A world that fails
  to load ends the wait at once ("gz sim exited (code N) before <world> came up").
- Without `world` it attaches to the running simulation (it finds
  `/world/<name>/clock` on the transport); `--world-name` picks one.
```

with

```markdown
- `world`: a path to an `.sdf` or a name in `rover_sim/worlds`. With a world and
  nothing running, the station starts the world with the rover's ROS 2 stack
  (`ros2 launch rover_sim sim.launch.py world:=<world> gui:=false station:=false`,
  in `gzenv.py`'s environment), stops it on exit and exits when it exits. A world
  that fails to load ends the wait at once ("the simulation exited (code N)
  before <world> came up").
- Without `world` it attaches to the running simulation (it finds
  `/world/<name>/clock` on the transport); `--world-name` picks one, `--wait`
  waits up to that many seconds for it (`sim.launch.py station:=true` starts
  the station so).
```

replace

```markdown
- While the station has control it publishes `/model/rover/cmd_vel` at 20 Hz
  and `blue` on `/model/rover/led` at 2 Hz (rule 1.e.ii: teleoperation).
```

with

```markdown
- While the station has control it publishes `/cmd_vel` (TwistStamped, the
  stamp left zero for the drive controller; `station/ros_link.py`) at 20 Hz and
  `blue` on `/model/rover/led` at 2 Hz (rule 1.e.ii: teleoperation). If nothing
  subscribes to `/cmd_vel` 10 s after the start (a world started without
  `sim.launch.py`), it says so.
```

replace

```markdown
  ignored. A SIGKILLed station stops the rover through the drivetrain's
  command timeout (0.5 s).
- One world per `GZ_PARTITION`, one station per world: the rover's topics are
  not world-scoped. Run separate worlds with `GZ_PARTITION=<name>`; the
  station warns when several share its partition.
```

with

```markdown
  ignored. A SIGKILLed station stops the rover through the drive controller's
  command timeout (0.5 s).
- One world per `GZ_PARTITION` and ROS domain, one station per world: the
  rover's topics are not world-scoped. Run separate worlds with
  `GZ_PARTITION=<name>` and `ROS_DOMAIN_ID=<n>`; the station warns when several
  share its partition.
```

replace

```text
pixi run sim-bridge                # for ROS 2
```

with nothing (delete the line), and replace

```markdown
- **The rover stops after half a second:** the drivetrain's command timeout;
  publish `cmd_vel` at a rate (the station sends 20 Hz), or use
  `cmd_timeout: 0.0` (`sim.yaml`, drivetrain) for Gazebo's GUI Teleop.
```

with

```markdown
- **The rover stops after half a second:** the drive controller's command
  timeout (`cmd_vel_timeout`, `rover_control/config/controllers.yaml`);
  publish `/cmd_vel` at a rate (the station sends 20 Hz). A command stamped
  with the wall clock under sim time never times out: leave the stamp zero.
- **The rover does not move:** nothing drives it through ROS. Start the world
  with `pixi run sim` or `pixi run drive <world>` (sim.launch.py brings the
  controllers; `ros2 control list_controllers` lists them active).
```

In `docs/sim/design-notes.md` replace

```markdown
- **Setpoints**: (vx ∓ wz·track·track_multiplier/2)/r per side, through a ramp
  (`gz::math::SpeedLimiter`, 8 rad/s², 1.2 m/s² (A) until the driver team
  reports its controller's ramp; `sim.yaml` `drivetrain: accel`) and clamped to the top speed
  (`rover.yaml` `wheel: top_speed`).
  `track_multiplier` is 1.0: the raw skid-steer response, which the team's
  driver compensates (the user's choice; Clearpath ships 1.875 for Husky).
```

with

```markdown
- **Setpoints**: one per wheel, from its joint's `WheelSetpoint` component
  (`plugins/rover_components.hh`; design spec 2026-10-09, 6.1): ros2_control's
  `RoverSimSystem` writes it at the start of every step from the drive
  controller's command, and the motors use it in that step
  (`tests/test_ros2_control.py`); without ros2_control the drivetrain takes it
  from its ROS-free topic `/model/rover/wheel_cmd` (`gz.msgs.Double_V`, fl rl fr
  rr). Clamped to the top speed (`rover.yaml` `wheel: top_speed`). The skid-steer
  kinematics (vx ∓ wz·track/2)/r and the ramp (8 rad/s² for a wheel, 1.2 m/s²
  (A) until the driver team reports its controller's) are the drive
  controller's (`rover_control/config/controllers.yaml`), and `wheel_cmd.py`'s
  for the tests and tools (`sim.yaml` `wheel_cmd: accel`, with
  gz::math::SpeedLimiter's arithmetic, so every run drives as it did before
  PR 3). The effective-track multiplier is 1.0 (`wheel_separation_multiplier`):
  the raw skid-steer response, which the team's driver compensates (the user's
  choice; Clearpath ships 1.875 for Husky).
```

replace

```markdown
- **Command timeout**: a command older than 0.5 s (wall clock) counts as
  zero, so a commander that dies cannot leave the rover driving
  (`sim.yaml` drivetrain `cmd_timeout`; 0 holds the last command; tests use the sim
  clock).
- **Reset**: `ISystemReset` clears the command, motors, ramps, dig-in and
  odometry, puts the tyres back at their SDF poses and clears the ruts and
  pits.
```

with

```markdown
- **Watchdog**: a setpoint older than 0.5 s (wall clock) counts as zero, at
  once, as a motor controller's watchdog stops its motor, so a commander that
  dies cannot leave the rover driving (`sim.yaml` drivetrain `cmd_timeout`; 0
  holds the last setpoint; tests use the sim clock). The drive controller has
  its own 0.5 s command timeout in front of it.
- **Reset**: `ISystemReset` zeroes the setpoints and clears the motors, dig-in
  and odometry, puts the tyres back at their SDF poses and clears the ruts and
  pits.
```

- [ ] **Step 5: Check the links and commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && grep -rn "sim-bridge\|control-viz\|rover_driver\b\|config\.hpp" README.md rover_sim/README.md rover_control/README.md rover_description/README.md docs/sim docs/topics.md | grep -v "git history"
grep -c "/model/rover/cmd_vel" docs/sim/manual.md
```

Expected: no lines; `0`. Then:

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add docs/topics.md README.md rover_sim/README.md rover_control/README.md rover_description/README.md docs/sim/manual.md docs/sim/design-notes.md && git commit -q -F - <<'EOF'
PR 3: docs: the topic contract, the ros2_control boundary in the READMEs and the manual

docs/topics.md: the topic contract of design spec 6.2 (the same names on the simulated and the real rover), the stamp
rule for /cmd_vel, and the simulation's own ROS and Gazebo topics. The README's Simulation, The real rover, Tests and
packages, rover_sim's README (sim.launch.py, RoverSimSystem, wheel_cmd.py), rover_control's README (the controllers,
the launch files, the stub, the kinematics library, how to change the drive controller), rover_description's (who
reads rover.yaml, the plugin's default), and the simulation's manual and design notes (the drive through the
controllers, the watchdog, the station's /cmd_vel, the bridge without sim-bridge) say how it is now.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 20: The spec's as-built notes

**Files:** Modify `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`.

- [ ] **Step 1: Section 12**

Append to the spec:

```markdown

## 12. As built (PR 3, 2026-10-10)

PR 3 (`docs/superpowers/plans/2026-10-10-pr3-ros2-control.md`) built section 6 with these differences from the text above, each decided in that plan:

1. **The worlds keep including the rover** (6.1). Every ROS-free path (the suite's runs, the referee, the maps, the realism report) loads the worlds as they are, and gz_ros2_control in a model blocks Gazebo's start until `/robot_description` arrives. `sim.launch.py` runs a copy of the world whose one `model://rover` include points at the rover with ros2_control, built per launch from the URDF it publishes and post-processed by `gen_rover` (PR 2's Decision 1), at the include's pose, the mission sheet's start. No spawn service is involved.
2. **The drive controller's numbers are written out in `rover_control/config/controllers.yaml`** (6), and `rover_control`'s `test_controllers_yaml` checks them against `rover.yaml`. The controllers are named `joint_state_broadcaster` and `drive_controller` (diff_drive_controller); `controller:=<name>` starts another drive controller from the file.
3. **The hardware stub activates** (6): it reports zero motion and logs the commands it would send, so that the controllers start on the rover's computer; it does not refuse to activate.
4. **The ramp** (6.1): the drive controller ramps vx and wz (1.2 m/s², 3 rad/s²), each a wheel's 8 rad/s² driving straight or turning in place. The simulation's ROS-free helpers (`rover_sim/wheel_cmd.py`) turn their twists into wheel setpoints with the drivetrain's former formula and ramp (`sim.yaml` `wheel_cmd.accel`), so the calibration runs are bit for bit what they were. The drivetrain keeps the top-speed clamp and its watchdog, which zeroes a stale setpoint at once.
5. **Timing**: the controller manager's `write()` (gz_ros2_control's PreUpdate) runs before the drivetrain's PreUpdate in every step because its plugin comes first in the model, so a setpoint written at the start of step k drives the motors in step k (`rover_sim/tests/test_ros2_control.py`, from the steps in which the setpoints change; the plan's Prototype results). ros2_control 4.47 hands the hardware and the controllers the controller manager's clock, not the step's time: sim time from the bridged `/clock`, a few ms behind the step. RoverSimSystem stamps the setpoints with it, so the drivetrain's watchdog also stops the motors when `/clock` stops, the one case diff_drive_controller's own timeout misses because its clock stops with it. Without the `/clock` bridge nothing drives.
6. **The ROS-free setpoint topic** (`/model/rover/wheel_cmd`) is ignored while ros2_control drives the wheels, with a message at Gazebo's error level, which `gz sim` shows at its default verbosity.
7. **`pixi run drive <world>` and the launcher start `sim.launch.py` through the station** (6.3), which stays its parent and keeps its stop handling (`ros2 launch` does not handle SIGHUP); `sim.launch.py station:=true` starts the station attached. The launcher's worlds and the tests take ROS domains of their own (50-99).
8. **The GUI is a separate process on every platform** (6.3), and `sensors:=false` runs without a GPU (the smoke test).
9. **The station's `/cmd_vel` carries a zero stamp**, which the drive controller replaces with its own clock's time on arrival (`docs/topics.md`).
10. **The drivetrain state's `cmd`** is the twist that the wheel setpoints in effect amount to.
11. **Dependencies**: `rover_control` and pixi name `controller_manager`, `hardware_interface`, `diff_drive_controller`, `joint_state_broadcaster` and `ros2controlcli`, not the `ros2_control` and `ros2_controllers` metapackages.
```

- [ ] **Step 2: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add docs/superpowers/specs/2026-10-09-rover-software-integration-design.md && git commit -q -F - <<'EOF'
Spec: as built in PR 3 (the ros2_control boundary)

Section 12 records where PR 3 differs from section 6: the worlds keep the rover and the launch swaps it, the drive
controller's numbers written out and tested, the stub activates, the two ramps, the step timing, the ignored topic's
message level, the station as the launch's parent, the GUI as a separate process everywhere, the zero stamp, the state
message's cmd, the packages named.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 21: Hand-off

**Files:** none.

- [ ] **Step 1: The branch, complete and not pushed**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain && git log --oneline pr2/rover-description..HEAD
git branch -vv | grep pr3/ros2-control; git ls-remote --heads origin pr3/ros2-control | wc -l
cd /Users/alarion239/Desktop/Rover && git status --porcelain | wc -l
```

Expected: a clean tree; the plan's two commits and the commits of Tasks 2 to 9, 11 to 17, 19 and 20 (pixi; the component; RoverSimSystem; the ROS rover; the prototype; wheel_cmd; the helpers; the drivetrain; the kinematics; the stub; the launch files; sim.launch.py; the station; sim and the launcher; the smoke test; the documents; the spec); `pr3/ros2-control` without an upstream; `0` (not on GitHub); the Rover repository untouched (`0`).

- [ ] **Step 2: Remove the temporary files**

```bash
rm -rf /private/tmp/claude-502/rover-pr3 /private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/pr3src /private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/pr3lock
ls /private/tmp/claude-502 | grep -c rover-pr3
```

Expected: `0`. No ROS daemon was started (`--no-daemon`, `ros2 topic pub`, rclpy scripts); `pgrep -fl "ros2-daemon|_ros2_daemon"` shows at most the one on domain 87, which is not ours.

- [ ] **Step 3: Report to the user**

Report: the commits; the prototype's results (Task 6: RoverSimSystem loaded by gz_ros2_control in `gz sim -s` on macOS, the GUI as a separate process, the step timing, the readback); the traces bit for bit (Task 10); the counts (Task 1's baseline and Task 18's: colcon 18, 5, 5; the suite 590, 9 skipped, 2 expected failures; 39 new tests, none gone); the acceptance by hand (Task 14 and 18: `ros2 topic pub` drives the simulated rover, rover.launch.py starts with the stub and both controllers active, the station drives through the controllers in `test_station`); the decisions that differ from the spec (section 12); what was not checked here: CI on Linux and Windows (the first push), Gazebo's GUI under `pixi run sim` beyond Task 6's separate-process check, the stack on the rover's own computer; the lock: PR 3 changes `pixi.toml` and `pixi.lock` (31 packages) as PR 2 and the data pull request do, and the lock is `merge=binary`, so whichever merges after another merges `pixi.toml` by hand and regenerates the lock with `pixi lock` (never by merging the lock's text), then checks with a lock diff that only the pull requests' packages were added; and the decision that needs the user: the push of `pr3/ros2-control` and its pull request, which builds on PR 2 (`pr2/rover-description`) and should be opened after PR 2 merges, or against `pr2/rover-description` until then.

---

## Acceptance (spec 9.2, row 3; the task's list)

| Criterion | Where it is shown |
|---|---|
| A prototype first: RoverSimSystem hands setpoints to RoverDrivetrain inside Gazebo through a new rover_sim ECM component (not JointVelocityCmd), on this Mac; timing proven and documented; gz_ros2_control loads it with the server/GUI split; controllers drive the wheels | Tasks 3 to 6; `test_ros2_control` (timing, readback, driving); `split_check.sh`; the Prototype results section |
| RoverDrivetrain takes four wheel setpoints; kinematics, track multiplier and ramp in the drive controller's config; motor model, PI, current limit, backlash, the watchdog, contact, dig-in, cues, odometry, ground truth and the state stay | Task 9; `test_drivetrain.Watchdog`; the full suite (Task 18) |
| The ROS-free topic `/model/rover/wheel_cmd` feeds the same component, ignored with a warning under RoverSimSystem | Task 9; `test_ros2_control.test_the_setpoint_topic_is_ignored` |
| The test helpers turn their twists into setpoints with the plugin's formula; every calibration test keeps its numbers; the full suite passes | Tasks 7, 8, 10 (traces bit for bit), 18 |
| rover_control: controllers.yaml (diff_drive_controller, left FL+RL, right FR+RR, rover.yaml's geometry, limits, timeout, /odom and odom → base_link; joint_state_broadcaster) | Task 5; `test_controllers_yaml` (Task 11) |
| The hardware stub (same interfaces, zero motion, logs commands, says no motors are connected) | Task 12; `test_rover_hardware_stub`; `test_rover_launch` |
| rover.launch.py (robot_state_publisher + ros2_control_node + controllers.yaml) starts with the stub | Task 13; `test_rover_launch`; Task 18 Step 2 |
| A tank-drive kinematics library with GoogleTest reading rover.yaml; the swerve code dropped | Task 11 |
| package.xml and CMake dependencies | Tasks 4, 11 to 14 (Decision 10) |
| sim.launch.py world:= gui:= station:= controller:= (server, GUI separate, robot_state_publisher on rover_sim.urdf.xacro with gz_ros2_control and RoverSimSystem, the post-processed model at the start pose, controllers from rover_control, the sensor bridge, the station) | Task 14; `test_sim_launch`; Decision 12 |
| The station publishes /cmd_vel through rclpy; Fly, Map, the readout and the referee keep Gazebo; drive and the launcher start sim.launch.py; the station works attached | Tasks 15, 16; `test_station.RosLink`, `StationProcess.test_drives_through_the_controllers`, `test_launcher` |
| Tests: RoverSimSystem, the watchdog, the kinematics, check_urdf, the smoke test in pixi run test and CI | Tasks 4, 6, 9, 11, 5 (`test_check_urdf_accepts_it`), 17 |
| Driving from the station and from `ros2 topic pub /cmd_vel` through the controllers | `StationProcess.test_drives_through_the_controllers`; Tasks 14 and 18 (`launch_check.sh`) |
| Docs: the topic contract in docs/, the READMEs, spec as-built notes | Tasks 19, 20 |

## Risks

| Risk | Handling |
|---|---|
| gz_ros2_control cannot load RoverSimSystem inside `gz sim -s` on macOS (rclcpp in the server, pluginlib's index, the system plugin's path) | Part A proves it before anything depends on it; Task 6 names the three likely causes by their log lines; if it cannot be made to load, the implementer stops and reports (options: run the server under ros_gz_sim's `gzserver` node, or a ROS node outside Gazebo that bridges the wheel setpoints over gz-transport, which the timing test would then measure) |
| The drivetrain runs before gz_ros2_control in a step (a setpoint used one step late) | The plugins' order makes it the same step; `test_ros2_control` checks it to the step against diff_drive_controller's stamped output; one step late is allowed by the task and would be recorded (Task 6 Step 3 says how) |
| The traces are not bit for bit (the tests' helper system no longer runs after the drivetrain, or the ramp's arithmetic differs) | Task 10 locates the first differing row; the suite's assertions, unchanged, decide; the finding goes into the results |
| The component registered from two libraries (RoverDrivetrain, rover_sim_system) is two types in the ECM | gz-sim 8's factory registers a component per library under one name by design; `test_ros2_control` would fail at once (no setpoint reaches the motors) |
| A wall-clock-stamped /cmd_vel never times out under sim time | The station leaves the stamp zero; `docs/topics.md` and the manual's troubleshooting say so; `test_station.RosLink` checks the zero stamp |
| The controllers come up after the station is ready, or not at all in a world started without the launch | The station warns after 10 s without a /cmd_vel subscriber; the tests wait for /odom and the subscriber before driving |
| ROS 2 traffic between test runs, the launcher's worlds and other work on this machine | Private ROS domains (50-99) with LOCALHOST discovery everywhere; the manual runs use 63-66; no ROS daemon is started |
| The suite gets slower (the ROS tests start Gazebo and the stack; four station tests start sim.launch.py) | About 3 minutes more (Task 18 measures it); the smoke test is the spec's 30-second class |
| CI's Linux runner and Windows build differ from this Mac (DDS on the runner, MSVC with hardware_interface's headers) | LOCALHOST discovery; rover_control exports all symbols on MSVC and builds its tests only where BUILD_TESTING is on (not on Windows); the first push shows it, and the user decides the push |
| robot_state_publisher's KDL warning and diff_drive_controller's "zero timestamp" warning in every launch's log | Both harmless (PR 2's Decision 14; the zero stamp is the design) |
| More log noise in the simulation's log: diff_drive_controller warns `Velocity command timed out. Braking.` once a second while no command comes (from 0.5 s after its activation), and the hardware component warns once `Executor is not available during hardware component initialization for 'rover'. Skipping node creation!` (gz_ros2_control 1.2.19 does not pass ros2_control 4.47 the executor); Gazebo's GUI does not load the rover's own components in the server's state (the probe saw them in `/world/<name>/state`): Task 6 measured that it skips them silently at its default verbosity and warns once per type at `-v 2`, `Component type [<id>] has not been registered in this process, so it can't be deserialized.`, not the `Failed to create component type [...]` this row first guessed | Seen or traced in the plan critic's probe and Task 6, and harmless (Task 6 Step 4 counts the GUI's lines); `docs/sim/manual.md` names them beside the rover's ROS stack, so nobody chases them |
| ros2_control 4.47 gives the hardware and the controllers the controller manager's clock (the bridged `/clock`, 1 to 16 ms behind the step in the probe), not the time gz_ros2_control passes | Measured by the plan critic's probe (Decisions 2 and 3): the timing and readback tests find the steps from the drivetrain's per-step state, not from the messages' stamps; RoverSimSystem's stamps lag the step by those few ms, far below the watchdog's 0.5 s, and freeze with `/clock`, which stops the motors |
| `test_ros2_control` and the smoke test on a runner that simulates slower than real time | Both drive on sim time (`rosstack.Probe.drive_for`, timed by `/odom`'s stamps) with wall-clock caps; the thresholds leave room for the ramps and the slip |
| The ROS-free helpers' ramp is not the plugin's to the bit (libgz-math7 fuses the multiply-add on osx-arm64) | `wheel_cmd.fused` rounds once, as `fmadd` does; `test_wheel_cmd.RampSteps.test_one_rounding_as_libgz_math_computes_it` pins a residue measured from the library |
| The pixi lock conflicts with PR 2's and the data pull request's | `merge=binary`; whichever merges later regenerates with `pixi lock` after merging `pixi.toml` by hand (Task 21's report) |
| The drive controller's numbers drift from rover.yaml | `test_controllers_yaml` fails and names the number; rover_description's README says to change both |
| A world reset reloads gz_ros2_control (1.2.19 has no ISystemReset), whose new controller manager has no controllers (found by the PR 3 review) | `sim.launch.py` spawns the controllers again each time RoverSimSystem comes up after the first; the smoke test resets the world and drives again |
| gz_ros2_control cannot load RoverSimSystem now and then on macOS (`dlclose(0x0): invalid handle`, a race in rcutils' loader; once in some 170 starts, the review) | RoverSimSystem links only the libraries it uses (a short search of the loaded images); a spawner that fails stops `sim.launch.py`, saying why; the manual's Known limitations. Not retried in the tests |
| The spawner's lock file in `~/.ros/locks` is every spawner's on the machine, whatever the ROS domain (the review) | `sim.launch.py`'s spawners, `test_ros2_control`'s and `test_rover_launch`'s get a ROS_HOME of their own (`gzenv.private_ros_home`) |
| A stop before the controllers are active takes 5 to 11 s and ends in SIGKILL (gz_ros2_control's destructor waits for its controller manager; the review) | Upstream behaviour; the manual's Known limitations; the station's 10 s and the launcher's 25 s bound it |
| `pixi run sim` and `drive` use domain 0 with ROS 2's subnet discovery (the review) | Documented with the remedy (`docs/topics.md`, the manual); the default is the user's decision (Review fixes) |

## The plan critic's checks (2026-10-10)

Checked on this Mac against a scratch environment locked as Task 2 locks it: gz_ros2_control 1.2.19, ros2_control 4.47.0, ros2_controllers 4.42.1 and gz-sim 8.10. The scratch environment has been deleted. Not checked here: the real rover model in Gazebo (Task 6 does that), Gazebo's GUI as a separate process (Task 6 Step 4), and Linux.

- **Task 4:** the four files, as written, build with `-Wall -Wextra -Wpedantic` without a warning, and the five test cases pass.
- **The prototype, with a stand-in for the drivetrain:**
  - A stand-in for the drivetrain's command path ran after gz_ros2_control in a four-wheel model under `gz sim -s`, with robot_state_publisher, the `/clock` bridge, and the spawner arguments of `controllers.launch.py` on this plan's `controllers.yaml`.
  - gz_ros2_control loaded RoverSimSystem from an install prefix on `AMENT_PREFIX_PATH`.
  - Both controllers configured and activated, and zero-stamped TwistStamped commands moved `/odom`.
  - The stand-in system plugin read the `WheelSetpoint` that RoverSimSystem created: two libraries register the component, and the ECM sees one type.
  - All 42 setpoint changes fell in steps that are 1 mod 10, so the setpoints are handed over in the same step (Decision 3).
- **The clock:** ros2_control 4.47 gives the hardware's `write()` and the controllers the controller manager's clock, which is sim time from the bridged `/clock`. In the probe it ran 1 to 16 ms behind the step, and `cmd_vel_out` stamps were 8 to 15 ms apart (Decision 2). `test_ros2_control`'s timing and readback tests therefore find the steps from the drivetrain's per-step state, not from the messages' stamps.
- **The ramp:** libgz-math7's `SpeedLimiter::LimitAcceleration` is compiled to `fmadd` on osx-arm64. `wheel_cmd.fused` matched the library over four ramps of 4000 steps each. Two roundings differed from it in 19 steps of the spin-stop ramp (Decision 6).
- **The station's signals:** with rclpy's default signal handlers, a publish after SIGTERM fails with `RCLError: publisher's context is invalid`. With `SignalHandlerOptions.NO` it goes out (Decision 14).
- **Task 14's test:** launch's `ExecuteProcess.name` is a list of substitutions, which cannot be a dict key. The test now performs it first.
- **The GUI:** the server's full state (`/world/<name>/state`) carries `WheelSetpoint` and `WheelMotorCurrent` on the wheel joints. Gazebo's GUI does not load the library that registers them (Task 6 Step 4, Risks).
- **diff_drive_controller in the log:** it warns `Velocity command timed out. Braking.` once a second while no command comes.

## Prototype results (Task 6, 2026-10-10)

Measured on the Apple M4 (macOS 26.5.1), gz-sim 8.10.0, gz_ros2_control 1.2.19, ros2_control 4.47.0, ros2_controllers
4.42.1. The gate is passed: Parts B to E may start.
- `GZ_SIM_SYSTEM_PLUGIN_PATH` after activation: `GZ_SIM_SYSTEM_PLUGIN_PATH=[]` (empty: no activation script sets it);
  gzenv appends the environment's lib, and Gazebo found `gz_ros2_control-system` there.
- `test_ros2_control`: `Ran 3 tests in 9.724s`, `OK`; three more runs of the same stack also passed (10.7 s and
  10.3 s, and the measuring run below). The controllers were updating and `/cmd_vel` had its subscriber 4.1 s after
  the server started (wall clock: the server's start, `/robot_description`, the hardware, both controllers).
- Timing: a setpoint the controller manager writes at the start of step k drives the motors in step k. Every one of
  the 51 setpoint changes in the drivetrain's per-step state is in a step 11, 21, 31 ... (the first at 1601, 1611,
  1621 ms); the controllers update in the PostUpdate of steps 10, 20, 30 ... None was one step late.
- Readback: `/joint_states`' wheel efforts equal the drivetrain's motor currents of an update step. Of 517 messages,
  411 match exactly one step within 50 ms of their stamp, and every one of those is a step 10, 20, 30 ...; the
  stamps lag that step by 0 to 8 ms (mean 3.2 ms), the controller manager's clock from the bridged `/clock`.
  `cmd_vel_out`'s stamps are 5 to 17 ms apart (mostly 11 and 12 ms). After 3 s of the arc and 1.5 s of stop the
  wheels had turned 2.03 rad (left) and 10.03 rad (right).
- Server and GUI as separate processes (`split_check.sh`, domain 63, its world with the SceneBroadcaster: see below):
  `spawner exit 0`, `odom x 1.597 y 0.000`, `gui running`, `server stopped`, the server's `Loading
  controller_manager` and `RoverSimSystem: 8 joints, 4 wheels driven through RoverDrivetrain's motor model`, `2`
  controllers activated, and `0` GUI lines `Failed to create component`, not the 8 this plan guessed. Three more
  split runs (one without the SceneBroadcaster, one that queried the state, one with the GUI at `-v 2`) gave
  `odom x` 1.594, 1.573 and 1.710 and the same other lines.
- The GUI and the rover's components: the server's state (`/world/test/state`, queried during a split run) carries
  `WheelSetpoint` and `WheelMotorCurrent` on the four wheel joints (4 entries each). The GUI skips both types it has
  no registration for. At its default verbosity it prints nothing about them. At `-v 2` it warns once per type:
  `Component type [15200978981940006955] has not been registered in this process, so it can't be deserialized.`
  (WheelSetpoint) and the same for `[6913242256637207411]` (WheelMotorCurrent). The ids are gz-common's `hash64`
  of the registered names. The manual's paragraph on harmless log lines (Task 19) and the Risks row should name this
  warning instead of `Failed to create component type`. The GUI also printed gz-rendering's
  `Unable to load Ogre Plugin[.../lib/OGRE-Next]` twice, which `docs/sim/gazebo-lessons.md` already documents as
  harmless.
- The server's log has the lines the Risks table names: `Executor is not available during hardware component
  initialization for 'rover'. Skipping node creation!` once, `Velocity command timed out. Braking.` about once a
  second while no command comes, and diff_drive_controller's one-time `Received TwistStamped with zero timestamp,
  setting it to current time`. It also has gz_ros2_control's `Desired controller update period (0.01 s) is slower
  than the gazebo simulation period (0.001 s)` once, which is the design (100 Hz on the 1 ms step).

What differed from this plan:
- `tests/rosstack.py`'s `stop()`: the first run failed in `setUpClass`'s cleanup with `PermissionError` from
  `os.killpg`. The spawner exits once both controllers are active, and macOS refuses a signal to a process group
  whose only member is an unreaped zombie with EPERM. The exception also stopped the cleanup before it reached the
  other processes; I stopped them by hand. `stop()` now reaps the process first and treats EPERM like a group that
  has ended.
- `split_check.sh`: (1) it stops the background jobs by process group (`kill -INT -$pid`, under `set -m`), because
  `ros2 run` waits for its child and does not pass on a signal sent only to itself; (2) the world written by
  `proto_stack.py` adds the SceneBroadcaster and UserCommands systems. The test world (`simulate.world_sdf`) has
  neither, and every real world has both. Without the SceneBroadcaster the GUI has no scene to load, so the first run
  (0 component lines too) showed only that the GUI process starts beside the server.
- Task 4 Step 2 failed at configure with `'.../rover_sim/rover_sim_system.xml' does not exist`
  (pluginlib_export_plugin_description_file checks the file before CMake looks for the missing source). Step 4 built
  without a compiler warning. Configuring the package now prints CMake author, policy and deprecation warnings from
  the environment's own config files (fastrtps' `FindTinyXML2` case, realtime_tools' CMP0167, the deprecated
  tl_expected), which `find_package(hardware_interface)` and `rclcpp` pull in; they are not this package's. The new
  test binary's first run took 13 s, not 70 s.
- Task 2: the lock is the planning session's byte for byte (31, 31 and 29 packages added, none removed or updated).
  Six of the added packages per platform are conda-forge's, not RoboStack `_21` builds: cpp-expected, filelock,
  jinja2, markupsafe, pygraphviz, typeguard. The commit message says so.
- Task 5's expected test counts held (`Ran 72 tests`); `check_urdf` is installed, so `test_check_urdf_accepts_it` ran.
- For later tasks: Task 13's `test_rover_launch.py` stops its launch with a bare `os.killpg(launch.pid, ...)`. If the
  launch has already ended there, macOS raises the same `PermissionError` and hides the test's own failure; reap it
  first (`launch.poll()`) or use `rosstack.stop`'s rule.

## Review fixes (2026-10-10)

The PR 3 review's confirmed findings, each with what was done. Commits start `PR 3 review fixes:`.

| Finding | Done |
|---|---|
| A world reset leaves the ROS stack without controllers | `sim.launch.py` spawns them again (`RespawnAfterReset`, on RoverSimSystem's line in the server's output); the smoke test resets `rover_test` and drives again; the manual and `docs/topics.md` |
| The watchdog on the ros2_control path has no test | `test_ros2_control` stops the `/clock` bridge (SIGSTOP) mid-drive: the setpoints drop from the arc's to zero at once, 0.502 s later (M) |
| `librover_sim_system.dylib` lacks the link option of the other plugins | `-dead_strip_dylibs` / `--as-needed` for it and its test: 0 `*__rosidl_generator_py` libraries, 18 lines of `otool -L` where 274 (M) |
| RoverSimSystem did not load once (`dlclose(0x0)`) | The link option; `sim.launch.py` stops and says why when a spawner fails (`stop_if_a_spawner_fails`); the manual's Known limitations. The tests do not retry a stack that fails this way |
| "Every run drives as before PR 3" is too broad | `design-notes.md`, `sim.yaml`, `wheel_cmd.py` and `simulate.twist_publisher` say which runs: those that keep commanding (every calibration run); a run that stops commanding stops harder (0.558 m where 0.651 m, the review's numbers), the cost runs ramp coarser |
| The spawner's lock is shared by every stack on the machine | A ROS_HOME of their own for the spawners of `sim.launch.py`, `test_ros2_control` and `test_rover_launch`, the logs left where they went |
| `test_rover_launch` hard-codes 3.33 rad/s | 0.5 m/s over the installed `controllers.yaml`'s `wheel_radius` |
| The stub logs `-0.00` and nothing when the wheels stop | 0.00 below 0.005 rad/s; one `would send 0 to every wheel` line when they are back at zero; formatted only when logged; a GoogleTest captures the lines, `test_rover_launch` checks the stop |
| Ctrl-C of `rover.launch.py` prints two ERROR lines | `rover_control/README.md` names them (pal_statistics' `context cannot be slept with`, and the second SIGINT's `exit code -2`) |
| A stop during start-up takes 10 s and ends in SIGKILL | The manual's Known limitations and a Risks row; upstream, not changed |
| `pixi run drive` and `sim` use domain 0 with subnet discovery | Documented, with the remedy (`ROS_DOMAIN_ID`, or `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), in `docs/topics.md` and the manual. The default was not changed: one skeptic refuted the finding; the spec has ROS tools on other computers (RViz on Windows beside Gazebo in WSL2, the rover's computer); pixi's ros_environment hook sets `SUBNET` when unset, so a LOCALHOST default for the simulation could not tell an explicit opt-out. The user decides |
| The station's warning timer raises after `close()` | `CmdVel` cancels its timers in `close()`, and a lock keeps them and the station's threads off the destroyed node; a test fails on the old code |
| Camera images reach ROS at 2 to 4 Hz on this Mac | `docs/topics.md`: about 11 Hz to a reliable subscriber on an idle M4, under 4 Hz on a busy one, none best effort (M); subscribe reliable |
| The bridge converts point clouds with no subscriber | The camera's four entries are `lazy: true`: no Gazebo subscriber until a ROS reader, 138 images in 12 s with one (M) |
| `rover_description/README.md` says CI cannot catch a stale model | It says CI's `git diff --exit-code` fails on one |
| Spec section 12 does not say the prototype ran on macOS only | Item 12 |
| `station.js` still names `/model/rover/cmd_vel` | Its header says `/cmd_vel` through `station/ros_link.py` |
| The manual's harmless lines miss three warnings | The three, the drive controller's `Can't accept new commands`, and the bridge's occasional `mutex lock failed` on a stop; `docs/topics.md` says the rockers' and the head's efforts are NaN |
| `rover.launch.py --show-args` offers a dead `controllers` | `rover.launch.py` declares `controllers` and hands it to `ros2_control_node` and the spawner; a test performs the include's arguments |
| Spec section 12 does not say `run.sh` stays | Item 13 |
