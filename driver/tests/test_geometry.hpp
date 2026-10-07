#pragma once

#include "rover_driver/config.hpp"

namespace rover_driver::test {

// Worked-example geometry from the design discussion: wheels at (±0.5, ±0.4).
inline constexpr RoverGeometry kExample{
    /*wheel_radius=*/0.15,
    /*pivot=*/{{{0.0, 0.4, 0.35}, {0.0, -0.4, 0.35}}},
    /*wheel_offset=*/
    {{{0.5, 0.0, -0.2}, {0.5, 0.0, -0.2}, {-0.5, 0.0, -0.2}, {-0.5, 0.0, -0.2}}},
    /*steer_undefined_speed=*/1e-6,
    /*steer_limit=*/2.356,
    /*fk3d_eta_prior_weight=*/1e-3,
    /*fk3d_vz_prior_weight=*/1.0,
    /*fk3d_max_contact_angle=*/1.4,
    /*fk3d_max_iterations=*/100,
    /*fk3d_tolerance=*/1e-12,
};

constexpr double kPi = 3.14159265358979323846;

}  // namespace rover_driver::test
