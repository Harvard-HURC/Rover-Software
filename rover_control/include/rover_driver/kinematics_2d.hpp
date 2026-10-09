#pragma once

#include <Eigen/Core>

#include "rover_driver/config.hpp"
#include "rover_driver/types.hpp"

namespace rover_driver {

// Wheel-center (x, y) in the body frame with both rockers at zero angle.
Eigen::Vector2d nominal_wheel_xy(int wheel, const RoverGeometry& g = kRover) noexcept;

// Flat-ground swerve IK: body twist -> per-module steering angle and wheel rate.
// Modules slower than g.steer_undefined_speed get steer_defined = false and
// zero angle/rate.
PerWheel<ModuleCommand> inverse_2d(const BodyTwist2d& cmd,
                                   const RoverGeometry& g = kRover) noexcept;

// Flat-ground swerve FK: least-squares body twist from all modules
// (Kelly & Seegmiller 2015, eq. 75, zero steering offset). residual_rms is the
// RMS of the 8 velocity-equation residuals [m/s] - a slip indicator.
Fk2dResult forward_2d(const PerWheel<ModuleMeasurement>& meas,
                      const RoverGeometry& g = kRover) noexcept;

}  // namespace rover_driver
