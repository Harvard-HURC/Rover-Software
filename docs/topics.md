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

Joints: `wheel_fl_joint`, `wheel_rl_joint`, `wheel_fr_joint`, `wheel_rr_joint`, `rocker_left_joint`, `rocker_right_joint`, `camera_pan_joint`, `camera_tilt_joint`. The controllers are `rover_control/config/controllers.yaml`'s, the same on both rovers (`rover_control/README.md`): `drive_controller` (diff_drive_controller, left = FL + RL, right = FR + RR) and `joint_state_broadcaster`. On the real rover the hardware interface is a stub until the motor controllers are chosen: `/joint_states` and `/odom` stay at zero there.

On the simulated rover (`pixi run sim` running), in `pixi shell`:

    ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}, angular: {z: 0.2}}}"
    ros2 topic echo /odom

`pixi run sim` and `pixi run drive` use the shell's ROS domain and gz partition (`ROS_DOMAIN_ID`, `GZ_PARTITION`). The launcher runs each world in a domain and partition of its own, as the tests do; its console names them (`ROS_DOMAIN_ID=<n> ros2 topic list` then sees the world's topics).

## The simulation's own topics

`rover_sim/launch/sim.launch.py` also bridges these (`rover_sim/bridge.yaml`): `/drivetrain` (std_msgs/String, the drivetrain's state as JSON at 50 Hz: each wheel's setpoint, speed, motor current, torque, voltage, saturation, slip, load, ground and dig factor; `cmd`, the twist the setpoints amount to), `/camera/pan` and `/camera/tilt` (Float64 target angles [rad], to the head), `/led` (the status LED: `red`, `blue`, `green`, `off`), and the referee's and the mission props' topics (`/urc/score`, `/urc/radio`, `/urc/display`, `/urc/launch_key`, `/astronaut/command`, `/astronaut/speech`, `/lander/joint_states`; `docs/sim/manual.md`).

In Gazebo only (`rover_sim/config/sim.yaml`, topics): `/model/rover/wheel_cmd` (gz.msgs.Double_V, the four wheels' speed setpoints in rad/s, fl rl fr rr), where the simulation's tests and tools drive the rover without ROS (`rover_sim/wheel_cmd.py`), ignored while ros2_control drives the wheels; the drivetrain's own no-slip odometry `/model/rover/odometry` and `/model/rover/tf`; `/model/rover/joint_states`; `/model/rover/ground_truth`; the driver station's cameras.
