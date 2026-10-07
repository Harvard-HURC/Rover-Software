#pragma once

#include <array>

#include <Eigen/Core>

#include "rover_driver/config.hpp"

namespace rover_driver {

template <class T>
using PerWheel = std::array<T, kNumWheels>;

// Planar body velocity command: linear [m/s] and yaw rate [rad/s].
struct BodyTwist2d {
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
};

// Body angular velocity from the IMU gyro [rad/s], body frame.
struct Gyro {
  double wx = 0.0;
  double wy = 0.0;
  double wz = 0.0;
};

// Rocker encoder readings: angles [rad] and rates [rad/s] relative to the body.
struct SuspensionState {
  double q_left = 0.0;
  double q_right = 0.0;
  double dq_left = 0.0;
  double dq_right = 0.0;
};

struct ModuleCommand {
  double steer_angle = 0.0;  // [rad]
  double wheel_rate = 0.0;   // [rad/s]
  bool steer_defined = false;
};

struct ModuleCommand3d : ModuleCommand {
  double contact_angle = 0.0;  // [rad]
};

struct ModuleMeasurement {
  double steer_angle = 0.0;  // [rad]
  double wheel_rate = 0.0;   // [rad/s]
};

struct Fk2dResult {
  BodyTwist2d twist;
  double residual_rms = 0.0;  // [m/s]
};

struct Fk3dResult {
  Eigen::Vector3d v_body = Eigen::Vector3d::Zero();  // [m/s], body frame
  PerWheel<double> contact_angle{};
  double residual_rms = 0.0;  // [m/s]
  bool converged = false;
  int iterations = 0;  // accepted Levenberg-Marquardt steps
};

// What the encoders would read if the modules tracked these commands exactly.
template <class Command>
PerWheel<ModuleMeasurement> to_measurements(const PerWheel<Command>& commands) noexcept {
  PerWheel<ModuleMeasurement> out{};
  for (int i = 0; i < kNumWheels; ++i) {
    out[i] = {commands[i].steer_angle, commands[i].wheel_rate};
  }
  return out;
}

}  // namespace rover_driver
