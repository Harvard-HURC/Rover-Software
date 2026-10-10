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

The stub warns that no motors are connected and logs the commands it would send (`would send wheel_fl_joint 2.00, ...`, at most once a second, rounded to 0.01 rad/s); `/joint_states` and `/odom` stay at zero. The drive controller also warns `Velocity command timed out. Braking.` about once a second while no command comes, and once `Received TwistStamped with zero timestamp`: the stamp is left zero on purpose (`docs/topics.md`).

### Change the drive controller
Change `drive_controller`'s `type` in `config/controllers.yaml`, or add another controller beside it and start it with `controller:=<name>` (`rover.launch.py`, and `sim.launch.py` in the simulation). Its parameters go under its name in the same file. Keep the geometry `rover.yaml`'s: `test_controllers_yaml.cpp` checks `drive_controller`'s.

### When the motor controllers are chosen
Write their ros2_control hardware interface (a `hardware_interface::SystemInterface` with the stub's interfaces), export it with pluginlib, and give it as `hardware:=<package>/<Class>`, or make it `rover_description`'s default (`urdf/rover.urdf.xacro`'s `ros2_control_plugin`).

Link it as `CMakeLists.txt` links the stub, with only the libraries it uses (`-dead_strip_dylibs` on macOS, `--as-needed` on Linux). The ROS packages' exported link lines name their messages' Python extension libraries (`*__rosidl_generator_py`), which need libpython: linked in, they keep `ros2_control_node`, which has no libpython, from loading the plugin, and on macOS a test program fails to start with `dyld: symbol not found in flat namespace '_PyExc_RuntimeError'`.

The swerve design's kinematics library (`rover_driver`), `rover_state` and `viz.py` were removed when the drive became skid steer; they are in git history before PR 3.
