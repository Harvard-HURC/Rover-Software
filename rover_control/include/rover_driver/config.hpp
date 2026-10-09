#pragma once

#include <array>

#include <Eigen/Core>

namespace rover_driver {

enum Wheel : int { kFL = 0, kFR = 1, kRL = 2, kRR = 3 };
enum Side : int { kLeft = 0, kRight = 1 };

constexpr int kNumWheels = 4;
constexpr Side kWheelSide[kNumWheels] = {kLeft, kRight, kLeft, kRight};

// constexpr-friendly 3-vector (Eigen types cannot be constexpr).
struct Vec3 {
  double x;
  double y;
  double z;
};

inline Eigen::Vector3d to_eigen(const Vec3& v) noexcept { return {v.x, v.y, v.z}; }

// Body frame: x forward, y left, z up. The body origin is the point whose
// velocity is commanded and estimated; all positions are relative to it.
struct RoverGeometry {
  double wheel_radius;
  // Rocker pivot positions in the body frame, indexed by Side.
  std::array<Vec3, 2> pivot;
  // Wheel-center position relative to its rocker pivot, in the rocker frame
  // (equal to the body-frame offset when the rocker angle is zero).
  std::array<Vec3, kNumWheels> wheel_offset;
  // Below this horizontal wheel speed [m/s] the steering angle is undefined.
  double steer_undefined_speed;
  // Steering joint range [rad]: modules reach [-steer_limit, steer_limit].
  // Must exceed pi/2 so every wheel direction fits, forward or reversed.
  double steer_limit;
  // Weight [m/s per rad] pulling contact angles to zero in forward_3d.
  double fk3d_eta_prior_weight;
  // Weight [dimensionless] pulling body vertical velocity to zero in forward_3d:
  // the body is assumed to pitch and roll about its origin, as inverse_3d does.
  // Wheel odometry observes v_z only through rotation, so without it slip is
  // explained as vertical motion. In least-squares terms it is the wheel
  // velocity noise divided by the expected spread of v_z. The price: real v_z,
  // e.g. while pitching about the rear axle as the front wheels climb a step,
  // is underestimated and the rest lands in the contact angles and
  // residual_rms (see forward_3d). Lower weights shrink that bias but pass more
  // wheel noise into v_z.
  double fk3d_vz_prior_weight;
  // Largest |contact angle| [rad] forward_3d may report; keep it below pi/2.
  // At pi/2 the wheel center moves along module z, and beyond it opposite to
  // the way the wheel spins, so a wheel turning against its motion (reversed,
  // slipping, miswired) could be explained with no residual. A wheel meeting a
  // step of height h with its rocker level has contact angle acos(1 - h / R).
  double fk3d_max_contact_angle;
  int fk3d_max_iterations;
  double fk3d_tolerance;
};

// PLACEHOLDER numbers until the mechanical team provides the real geometry.
// Origin: on the ground, midway between the wheels.
inline constexpr RoverGeometry kRover{
    /*wheel_radius=*/0.15,
    /*pivot=*/{{{0.0, 0.40, 0.35}, {0.0, -0.40, 0.35}}},
    /*wheel_offset=*/
    {{{0.45, 0.0, -0.20}, {0.45, 0.0, -0.20}, {-0.45, 0.0, -0.20}, {-0.45, 0.0, -0.20}}},
    /*steer_undefined_speed=*/1e-3,
    /*steer_limit=*/2.356,  // 135 deg
    /*fk3d_eta_prior_weight=*/1e-3,
    /*fk3d_vz_prior_weight=*/1.0,
    /*fk3d_max_contact_angle=*/1.4,  // 80 deg: a step of 0.83 R
    /*fk3d_max_iterations=*/50,
    /*fk3d_tolerance=*/1e-10,
};

static_assert(kRover.steer_limit > 1.5707963267948966, "steer_limit must exceed pi/2");

}  // namespace rover_driver
