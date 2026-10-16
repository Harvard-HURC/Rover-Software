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
