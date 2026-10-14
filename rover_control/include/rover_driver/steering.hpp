#pragma once

#include "config.hpp"
#include "types.hpp"
#include "rover_driver/config.hpp"
#include "rover_driver/types.hpp"

namespace rover_driver {

// Fits IK commands into the steering range [-g.steer_limit, g.steer_limit].
// A module moves its wheel the same way steered at psi + k*pi for any integer
// k, with the wheel rate negated for odd k: the wheel faces backwards and spins
// backwards. Of the angles in range, the one nearest the module's current angle
// (from its encoder) is chosen, so a module never swings further than needed
// and, where the range allows both, keeps the side it is on. Because
// g.steer_limit > pi/2, every direction has at least one angle in range; in
// particular reversing keeps the wheels straight and a turn in place needs at
// most about 50 degrees.
//
// Modules with steer_defined == false hold their current angle (clamped to the
// range; 0 if it is not finite) with zero wheel rate, instead of returning to 0.
// Commands with a non-finite steering angle pass through unchanged.
PerWheel<ModuleCommand> fit_steer_range(const PerWheel<ModuleCommand>& cmd,
                                        const PerWheel<double>& current_steer,
                                        const RoverGeometry& g = kRover) noexcept;

// As above for inverse_3d output. Reversing a module also negates its contact
// angle: the wheel-center velocity keeps its vertical part while its direction
// along the module x axis flips. (The fork-spin term in the wheel rate changes
// sign with the axle, so the whole wheel rate is negated.)
PerWheel<ModuleCommand3d> fit_steer_range(const PerWheel<ModuleCommand3d>& cmd,
                                          const PerWheel<double>& current_steer,
                                          const RoverGeometry& g = kRover) noexcept;

}  // namespace rover_driver
