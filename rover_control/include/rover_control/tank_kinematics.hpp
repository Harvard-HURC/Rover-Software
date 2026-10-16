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
