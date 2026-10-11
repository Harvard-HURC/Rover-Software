# Rover Control
The rover's drive, the same on the real and the simulated rover (design spec `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`, section 6). Programs publish `/cmd_vel` (geometry_msgs/TwistStamped); the controllers turn it into each wheel's velocity command, which ros2_control hands to the hardware: on the rover a stub until the motor controllers are chosen, in the simulation `rover_sim`'s `RoverSimSystem`. The topics are in [docs/topics.md](../docs/topics.md).

| Path | What it is |
|---|---|
| `config/controllers.yaml` | The controller manager (100 Hz) and its controllers: `joint_state_broadcaster` (`/joint_states`) and `drive_controller`, diff_drive_controller for skid steer (left FL + RL, right FR + RR; `/odom` and `odom` → `base_link`; 3 m/s and 7.5 rad/s, ramped at 1.2 m/s² and 3 rad/s²; a 0.5 s command timeout). Its geometry is `rover_description/config/rover.yaml`'s, written out; a test checks the two against each other |
| `launch/controllers.launch.py` | Spawns both into a running controller manager, the drive controller's topics on `/cmd_vel` and `/odom`; `rover.launch.py` and `rover_sim`'s `sim.launch.py` include it |
| `launch/rover.launch.py` | The real rover: robot_state_publisher on `rover_description`'s description, `ros2_control_node` with `controllers.yaml` (or `controllers:=<file>`), and the controllers |
| `include/rover_control/rover_hardware_stub.hpp`, `src/rover_hardware_stub.cpp`, `rover_hardware_stub.xml` | `rover_control/RoverHardwareStub`, the real rover's ros2_control hardware until the motor controllers are chosen: the description's interfaces, zero motion, and in the log the wheel commands it would send |
| `include/rover_control/tank_kinematics.hpp`, `src/tank_kinematics.cpp` | `tank_kinematics`: skid-steer inverse and forward kinematics, a saturation that keeps the turn radius, and `LoadRoverYaml`, the geometry from `rover.yaml` (a node gives the same numbers from its ROS parameters) |
| `test/` | GoogleTest (the kinematics, `controllers.yaml` against `rover.yaml`, the stub) and `test_rover_launch.py` (`rover.launch.py` with the stub, in a private ROS domain): `pixi run colcon-test` |

### Run the real rover's stack
From the workspace root, in `pixi shell`, after `colcon build`:

```bash
source install/local_setup.bash        # local_setup.zsh in zsh
ros2 launch rover_control rover.launch.py        # controller:=<name>, controllers:=<file>, hardware:=<plugin>, rover_yaml:=<file>
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}}}"
```

The stub warns that no motors are connected and logs the commands it would send (`would send wheel_fl_joint 2.00, ...`, rounded to 0.01 rad/s: at once when the wheels start, at most once a second while they turn, and `would send 0 to every wheel` once when they are back at zero); `/joint_states` and `/odom` stay at zero. The drive controller also warns `Velocity command timed out. Braking.` about once a second while no command comes, and once `Received TwistStamped with zero timestamp`: the stamp is left zero on purpose (`docs/topics.md`). Ctrl-C prints two harmless errors from the controller manager's statistics thread as it shuts down, `[controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!`, and sometimes `[ERROR] [ros2_control_node-2]: process has died [... exit code -2 ...]`: under `ros2 launch` one Ctrl-C reaches the node twice (`docs/sim/gazebo-lessons.md`).

### Change the drive controller
Add another controller beside `drive_controller` in `config/controllers.yaml` (its `type` under `controller_manager`, its parameters under its name, its geometry `rover.yaml`'s) and start it with `controller:=<name>` (`rover.launch.py`, and `sim.launch.py` in the simulation). `rover.launch.py` reads the installed `controllers.yaml`, so run `colcon build` (or `pixi run build`) after editing it; `sim.launch.py` reads the source tree's. Changing `drive_controller`'s `type` instead fails the tests written for diff_drive_controller: `test_controllers_yaml.cpp` checks the type and its parameters, and `rover_sim`'s `test_wheel_cmd.py` and `test_ros2_control.py` read its acceleration limits, `publish_limited_velocity` and `/drive_controller/cmd_vel_out`; change them with it. [CONTRIBUTING.md](../CONTRIBUTING.md#swap-the-drive-controller) has the details.

### Tests
`pixi run colcon-test` runs them with the other packages' colcon tests; for this package alone, in `pixi shell` in the workspace root after `colcon build`: `colcon test --packages-select rover_control`, then `colcon test-result --verbose`. 19 tests, about 7 s on an Apple M4 (M, 2026-10-10): the GoogleTest cases of the kinematics, `controllers.yaml` against `rover.yaml` and the stub, and `test_rover_launch.py`, which runs `rover.launch.py` with the stub in a private ROS domain and checks the wheel commands the stub logs.

### Drive it from a program
Publish `/cmd_vel` and read `/odom` ([docs/topics.md](../docs/topics.md)); [CONTRIBUTING.md](../CONTRIBUTING.md#write-a-program-that-drives-the-rover) has a short rclpy program, run against this stack and the simulation, and the steps for a drive controller of the team's own ([Write a drive controller](../CONTRIBUTING.md#write-a-drive-controller)).

### On the rover's computer
The rover's computer builds this package and `rover_description` only, without Gazebo or a GPU (design spec, section 7). Neither way below has run on a rover's computer yet (2026-10-10): the computer is not chosen.
- **pixi**, as on a laptop ([CONTRIBUTING.md](../CONTRIBUTING.md#set-up)), then in `pixi shell` in the workspace root `colcon build --packages-select rover_description rover_control`. `pixi.toml` lists `linux-64`; a Jetson needs `linux-aarch64` added and the lock resolved for it. `pixi install` also installs the simulation's libraries, about 5 GB in all.
- **apt** ROS 2 Jazzy on Ubuntu 24.04: in the workspace root, after `source /opt/ros/jazzy/setup.bash`, `rosdep install --from-paths src/rover_description src/rover_control --ignore-src -y`, then `colcon build --packages-select rover_description rover_control`.

Then `source install/local_setup.bash` and `ros2 launch rover_control rover.launch.py` (above).

### When the motor controllers are chosen
Write their ros2_control hardware interface (a `hardware_interface::SystemInterface` with the stub's interfaces), export it with pluginlib, and give it as `hardware:=<package>/<Class>`, or make it `rover_description`'s default (`urdf/rover.urdf.xacro`'s `ros2_control_plugin`).

Link it as `CMakeLists.txt` links the stub, with only the libraries it uses (`-dead_strip_dylibs` on macOS, `--as-needed` on Linux). The ROS packages' exported link lines name their messages' Python extension libraries (`*__rosidl_generator_py`), which need libpython: linked in, they keep `ros2_control_node`, which has no libpython, from loading the plugin, and on macOS a test program fails to start with `dyld: symbol not found in flat namespace '_PyExc_RuntimeError'`.

The swerve design's kinematics library (`rover_driver`), `rover_state` and `viz.py` were removed when the drive became skid steer; they are in git history before PR 3.
