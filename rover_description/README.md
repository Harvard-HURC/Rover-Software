# Rover Description
This package primarily contains the URDF files that define the rover's hardware in a way that other ROS packages can use.

Every physical number of the rover is in one file, `config/rover.yaml`; the description, the simulation (`rover_sim`) and RViz read it. The numbers are placeholders until the mechanical team has real ones: the geometry is the swerve design's, the motors a validated simulation set. `rover_control`'s kinematics library reads this file (`LoadRoverYaml`), and `rover_control/config/controllers.yaml` writes the drive controller's geometry out, checked against it by a test.

### What is where
| Path | What it is |
|---|---|
| `config/rover.yaml` | Every physical number, with its unit and source ((A) assumed, (M) measured or tuned, (R) from a reference): chassis, rockers, the differential's spring and damper, wheels and the top speed, the motors and drivelines, the tyres, the IMU, GNSS, the camera and its pan-tilt head, the LED, and the look (masts, colours) |
| `urdf/rover.urdf.xacro` | The description built from `rover.yaml`: links with their masses and inertias, joints with limits and damping, collisions and visuals, and the frames `imu`, `gnss`, `camera` and `led` |
| `urdf/inertials.xacro` | The inertias of a box, a wheel and a rocker |
| `urdf/ros2_control.xacro` | The `ros2_control` block: the four wheels take a velocity command and report position, velocity and effort; the rockers and the camera head report position and velocity. Its hardware plugin is the argument `ros2_control_plugin`: by default `rover_control/RoverHardwareStub`, the real rover's stub (`rover_control`'s `rover.launch.py` loads it); the simulation sets `rover_sim/RoverSimSystem` |
| `meshes/wheel.glb`, `meshes/gen_wheel_mesh.py` | The tread tyre (a unit cylinder the description scales to the wheel) and the script that writes it from `rover.yaml`'s colours |
| `launch/view.launch.py`, `rviz/view.rviz` | The rover in RViz |
| `test/test_description.py` | Its tests: `check_urdf`, the joint tree, the numbers, the `ros2_control` block, the mesh, the launch file (`colcon test`, with the team's linters) |

The description names no simulator. `rover_sim/urdf/rover_sim.urdf.xacro` adds Gazebo's sensors and plugins to it, and `rover_sim/config/sim.yaml` holds how the simulation models the rover (contact, dig-in, its motor controller's tuning, the visual cues).

### Change the rover
1. Edit `config/rover.yaml`; keep the unit and say where the number comes from.
2. `pixi run sim-model` builds the simulated rover again (`rover_sim/models/rover/model.sdf`); `pixi run sim`, `drive` and `test` do it for you. The generated models are tracked: commit what it rewrites with `rover.yaml` (`git status` lists it: `model.sdf`, and the tread mesh's copy or the station's cameras when they change). CI regenerates them before its tests, so it cannot tell a stale one.
3. `pixi run test`. A changed number fails `rover_sim`'s `test_equivalence`, which holds the rover as it was before this package described it: refreeze its fixtures in the same commit and say why (`rover_sim/tests/fixtures/rover_model/README.md`). A changed wheel radius, track or top speed also fails `rover_control`'s `test_controllers_yaml`: write the new numbers into `rover_control/config/controllers.yaml`.

A new frame, joint or shape goes into `urdf/rover.urdf.xacro`, its numbers into `rover.yaml`. Check the result in `pixi shell`: `xacro rover_description/urdf/rover.urdf.xacro > /tmp/rover.urdf && check_urdf /tmp/rover.urdf`. After changing the tread tyre's colours run `python rover_description/meshes/gen_wheel_mesh.py`, then `pixi run sim-model`.

### View it in RViz
From the workspace root, in `pixi shell`:

```bash
colcon build --packages-select rover_description
source install/local_setup.bash        # local_setup.zsh in zsh
ros2 launch rover_description view.launch.py
```

The sliders move the joints. Arguments: `gui:=false` (no sliders), `rviz:=false`, `rover_yaml:=<file>` (another set of numbers; a relative path is from the folder you launch in). Without a display (Linux without `DISPLAY` or `WAYLAND_DISPLAY`) `gui` and `rviz` default to false and the launch says so. `robot_state_publisher` warns that KDL does not support a root link with an inertia; the warning is harmless, and a massless root link would rename the simulated rover's links.

### Use it from another package
`xacro $(ros2 pkg prefix rover_description)/share/rover_description/urdf/rover.urdf.xacro` expands it; the arguments are `rover_yaml` (default the package's `config/rover.yaml`; give another file's absolute path, as xacro reads a relative one from the package's `urdf/`), `ros2_control` (`true` or `false`) and `ros2_control_plugin`.
