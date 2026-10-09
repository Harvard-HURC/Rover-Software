# Rover Control
The rover's drive code. For now it holds `rover_driver`, the kinematics library of the swerve design (2D and 3D inverse and forward kinematics, steering). The rover switched to tank drive on 2026-10-05, so the library is superseded; it stays until the ros2_control work replaces it with the drive controller's configuration, the hardware interface and `rover.launch.py` (design spec `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`, section 6).

- `include/rover_driver/`, `src/`: the library (C++17, Eigen)
- `tools/rover_state.cpp`: the kinematics of one command as JSON; `tools/viz.py` draws them (`pixi run control-viz`)
- `tests/`: GoogleTest and CTest tests (`pixi run colcon-test`)
