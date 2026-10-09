#pragma once

#include <Eigen/Core>

#include "rover_driver/config.hpp"
#include "rover_driver/types.hpp"

namespace rover_driver {

Eigen::Matrix3d rot_y(double angle) noexcept;
Eigen::Matrix3d rot_z(double angle) noexcept;

// Kinematic chain body -> rocker -> wheel center for one wheel, all in body
// coordinates (Kelly & Seegmiller 2015, transport theorem).
struct WheelChain {
  Eigen::Vector3d center;        // wheel-center position
  Eigen::Vector3d velocity;      // wheel-center velocity w.r.t. the world
  Eigen::Matrix3d rocker_rot;    // rocker frame -> body frame
  Eigen::Vector3d rocker_omega;  // rocker angular velocity w.r.t. the world
};

// v_body, w_body: body linear/angular velocity w.r.t. the world, body frame.
WheelChain wheel_chain(int wheel, const Eigen::Vector3d& v_body, const Eigen::Vector3d& w_body,
                       const SuspensionState& s, const RoverGeometry& g = kRover) noexcept;

// Terrain-aware IK (Toupet et al. 2020 generalised to swerve). Body velocity is
// (cmd.vx, cmd.vy, 0) - vertical assumed zero - and angular velocity is
// (gyro.wx, gyro.wy, cmd.wz). Steering follows the wheel-center velocity in
// the rocker frame; wheel rate removes the fork's own rotation about the axle.
// Not modelled: fork rotation about the rolling direction (body roll rate, or
// pitch/rocker rate while crabbing) tilts the wheel about its contact point and
// moves the center sideways without slip, so steering toward that motion then
// commands a small lateral slip.
PerWheel<ModuleCommand3d> inverse_3d(const BodyTwist2d& cmd, const Gyro& gyro,
                                     const SuspensionState& s,
                                     const RoverGeometry& g = kRover) noexcept;

// Terrain-aware FK: body linear velocity and per-wheel contact angles that best
// explain the measured steering angles and wheel rates (least squares), given
// the gyro's full angular velocity and the rocker encoders. Levenberg-Marquardt
// with the exact Hessian, initialised from forward_2d.
//
// Wheel speeds see v_z only through the front/rear and left/right differences
// that rotation causes, and not at all without rotation, so two priors apply:
// - v_z toward zero (g.fk3d_vz_prior_weight): the body pitches and rolls about
//   its origin, as inverse_3d assumes. Without it slip is explained as
//   vertical motion. Real v_z is underestimated, and the contact angles and
//   residual_rms absorb the rest. Example, kRover pitching nose-up at 0.3 rad/s
//   about the rear axle (front wheels climbing a step) while driving at
//   0.3 m/s: the origin moves at (0.3, 0, 0.135) m/s; on noise-free data
//   forward_3d reports (0.32, 0, 0.058), the level rear wheels as descending
//   at 0.27 rad, the front contact angles 0.20 rad low, and residual_rms
//   0.019 m/s.
// - contact angles toward zero (g.fk3d_eta_prior_weight). It only decides the
//   angle of a wheel whose rolling speed times velocity is below about the
//   weight squared, i.e. a wheel stopped to within about 1 mm/s by default.
//   A contact angle is meaningful only when its wheel moves well above encoder
//   noise; for a stopped or slow wheel it follows the noise.
// Contact angles are limited to +-g.fk3d_max_contact_angle. Otherwise a wheel
// turning against its motion (reversed, slipping, miswired) could be explained
// as rolling with an angle near pi, with no residual.
//
// The contact angles make the cost non-convex, and LM returns a stationary
// point, not necessarily the lowest one:
// - On inconsistent data (a reversed or slipping wheel) it can stop in a local
//   minimum whose v_body is far from the best fit's. residual_rms is then
//   large, so treat a result with a large residual_rms as untrustworthy.
// - The forward_2d start ignores the rocker angles, so it suits rockers far
//   from vertical. On random consistent motions it found the true one for
//   |q| <= 1.25 rad. From about 1.3 rad, with the rockers turned opposite
//   ways, some ended in a wrong minimum (about 6 in 10 000 near pi/2) with
//   v_body off by up to 1.8 m/s and residual_rms as small as 0.001 m/s. Real
//   rockers stay far below that.
//
// converged: a stationary point was reached (to within rounding), possibly
// with contact angles at the limit. False at the iteration limit, when the
// steps stall short of one (wheel rates beyond about 1e10 rad/s), and when the
// cost is not finite (non-finite input, or input so large that it overflows):
// then v_body, the contact angles and residual_rms are NaN.
// iterations: accepted LM steps. residual_rms: RMS of the 12 wheel-velocity
// rows [m/s], the consistency check. It includes what the priors and the limit
// leave unexplained, so it can be nonzero on consistent data (above). The
// rolling model omits the lateral wheel-center motion noted at inverse_3d, so
// under a body roll rate that motion shows up in v_body and residual_rms.
Fk3dResult forward_3d(const PerWheel<ModuleMeasurement>& meas, const Gyro& gyro,
                      const SuspensionState& s, const RoverGeometry& g = kRover) noexcept;

}  // namespace rover_driver
