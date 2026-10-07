#include "rover_driver/kinematics_3d.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rover_driver/kinematics_2d.hpp"

namespace rover_driver {

namespace {

constexpr int kUnknowns = 3 + kNumWheels;               // v_body, contact angles
constexpr int kKinematicRows = 3 * kNumWheels;          // wheel velocity equations
constexpr int kVzPriorRow = kKinematicRows + kNumWheels;  // after contact-angle priors
constexpr int kRows = kVzPriorRow + 1;
using State = Eigen::Matrix<double, kUnknowns, 1>;
using Hessian = Eigen::Matrix<double, kUnknowns, kUnknowns>;
using Residual = Eigen::Matrix<double, kRows, 1>;
using Jacobian = Eigen::Matrix<double, kRows, kUnknowns>;

// When no step reduces the cost, x counts as converged only if every gradient
// entry g_k = J_k . r is at most this fraction of its bound |J_k| |r|. Where
// rounding stops the steps at a minimum it is below 1e-7.
constexpr double kStallTolerance = 1e-6;

// Direction of the wheel-center velocity in module coordinates.
Eigen::Vector3d rolling_direction(double contact_angle) noexcept {
  return {std::cos(contact_angle), 0.0, std::sin(contact_angle)};
}

// Per-wheel quantities that do not depend on the unknowns.
struct WheelTerms {
  Eigen::Matrix3d module_rot;  // module frame -> body frame
  Eigen::Vector3d rotation_velocity;  // wheel-center velocity from rotations only
  double rolling_speed;               // R * (wheel rate + fork spin)
};

PerWheel<WheelTerms> wheel_terms(const PerWheel<ModuleMeasurement>& meas,
                                 const Eigen::Vector3d& w_body, const SuspensionState& s,
                                 const RoverGeometry& g) noexcept {
  PerWheel<WheelTerms> terms{};
  for (int i = 0; i < kNumWheels; ++i) {
    const WheelChain c = wheel_chain(i, Eigen::Vector3d::Zero(), w_body, s, g);
    terms[i].module_rot = c.rocker_rot * rot_z(meas[i].steer_angle);
    const double fork_spin = terms[i].module_rot.col(1).dot(c.rocker_omega);
    terms[i].rotation_velocity = c.velocity;
    terms[i].rolling_speed = g.wheel_radius * (meas[i].wheel_rate + fork_spin);
  }
  return terms;
}

// Residual per wheel (module coords): M^T (v + c) - s * [cos eta, 0, sin eta],
// then prior rows eta_weight * eta and vz_weight * v_z.
void evaluate(const State& x, const PerWheel<WheelTerms>& terms, double eta_weight,
              double vz_weight, Residual& r, Jacobian* jacobian) noexcept {
  const Eigen::Vector3d v = x.head<3>();
  if (jacobian != nullptr) {
    jacobian->setZero();
  }
  for (int i = 0; i < kNumWheels; ++i) {
    const WheelTerms& t = terms[i];
    const double eta = x(3 + i);
    r.segment<3>(3 * i) = t.module_rot.transpose() * (v + t.rotation_velocity) -
                          t.rolling_speed * rolling_direction(eta);
    r(kKinematicRows + i) = eta_weight * eta;
    if (jacobian != nullptr) {
      jacobian->block<3, 3>(3 * i, 0) = t.module_rot.transpose();
      jacobian->block<3, 1>(3 * i, 3 + i) =
          t.rolling_speed * Eigen::Vector3d(std::sin(eta), 0.0, -std::cos(eta));
      (*jacobian)(kKinematicRows + i, 3 + i) = eta_weight;
    }
  }
  r(kVzPriorRow) = vz_weight * v.z();
  if (jacobian != nullptr) {
    (*jacobian)(kVzPriorRow, 2) = vz_weight;
  }
}

}  // namespace

Eigen::Matrix3d rot_y(double angle) noexcept {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Eigen::Matrix3d r;
  r << c, 0.0, s,
       0.0, 1.0, 0.0,
       -s, 0.0, c;
  return r;
}

Eigen::Matrix3d rot_z(double angle) noexcept {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Eigen::Matrix3d r;
  r << c, -s, 0.0,
       s, c, 0.0,
       0.0, 0.0, 1.0;
  return r;
}

WheelChain wheel_chain(int wheel, const Eigen::Vector3d& v_body, const Eigen::Vector3d& w_body,
                       const SuspensionState& s, const RoverGeometry& g) noexcept {
  assert(wheel >= 0 && wheel < kNumWheels);
  const Side side = kWheelSide[wheel];
  const double q = side == kLeft ? s.q_left : s.q_right;
  const double dq = side == kLeft ? s.dq_left : s.dq_right;
  const Eigen::Vector3d pivot = to_eigen(g.pivot[side]);

  WheelChain c;
  c.rocker_rot = rot_y(q);
  const Eigen::Vector3d arm = c.rocker_rot * to_eigen(g.wheel_offset[wheel]);
  c.center = pivot + arm;
  c.rocker_omega = w_body + dq * Eigen::Vector3d::UnitY();
  c.velocity = v_body + w_body.cross(pivot) + c.rocker_omega.cross(arm);
  return c;
}

PerWheel<ModuleCommand3d> inverse_3d(const BodyTwist2d& cmd, const Gyro& gyro,
                                     const SuspensionState& s, const RoverGeometry& g) noexcept {
  const Eigen::Vector3d v_body(cmd.vx, cmd.vy, 0.0);
  const Eigen::Vector3d w_body(gyro.wx, gyro.wy, cmd.wz);

  PerWheel<ModuleCommand3d> out{};
  for (int i = 0; i < kNumWheels; ++i) {
    const WheelChain c = wheel_chain(i, v_body, w_body, s, g);
    const Eigen::Vector3d u = c.rocker_rot.transpose() * c.velocity;  // rocker frame
    const double horizontal = std::hypot(u.x(), u.y());
    if (horizontal < g.steer_undefined_speed) {
      continue;
    }
    const double psi = std::atan2(u.y(), u.x());
    const Eigen::Vector3d axle = c.rocker_rot * rot_z(psi) * Eigen::Vector3d::UnitY();
    const double fork_spin = axle.dot(c.rocker_omega);

    out[i].steer_angle = psi;
    out[i].wheel_rate = c.velocity.norm() / g.wheel_radius - fork_spin;
    out[i].steer_defined = true;
    out[i].contact_angle = std::atan2(u.z(), horizontal);
  }
  return out;
}

Fk3dResult forward_3d(const PerWheel<ModuleMeasurement>& meas, const Gyro& gyro,
                      const SuspensionState& s, const RoverGeometry& g) noexcept {
  const Eigen::Vector3d w_body(gyro.wx, gyro.wy, gyro.wz);
  const PerWheel<WheelTerms> terms = wheel_terms(meas, w_body, s, g);
  const double eta_prior = g.fk3d_eta_prior_weight;
  const double vz_prior = g.fk3d_vz_prior_weight;
  const double eta_max = g.fk3d_max_contact_angle;

  const Fk2dResult initial = forward_2d(meas, g);
  State x = State::Zero();
  x(0) = initial.twist.vx;
  x(1) = initial.twist.vy;

  Fk3dResult out;
  Residual r;
  Jacobian jacobian;
  double lambda = 1e-3;
  // Each pass tests x for convergence, then takes at most one accepted LM step;
  // out.iterations counts accepted steps.
  while (true) {
    evaluate(x, terms, eta_prior, vz_prior, r, &jacobian);
    const double cost = r.squaredNorm();
    if (!std::isfinite(cost)) {
      // Non-finite input, or input so large the cost overflows: no estimate.
      // Never reported as converged, and every output is NaN, including those
      // the forward_2d start would leave finite (it ignores gyro and rockers).
      x.setConstant(std::numeric_limits<double>::quiet_NaN());
      break;
    }
    State gradient = jacobian.transpose() * r;
    // Exact Hessian of cost / 2. The residual is linear in v_body, so J^T J
    // lacks only the curvature of each wheel's rolling term along its contact
    // angle. Without it, steps are poor whenever a wheel's rolling speed does
    // not match its motion (slip, or noise on a stopped rover) and the solve
    // crawls to the iteration limit.
    Hessian hessian = jacobian.transpose() * jacobian;
    for (int i = 0; i < kNumWheels; ++i) {
      hessian(3 + i, 3 + i) +=
          terms[i].rolling_speed * r.segment<3>(3 * i).dot(rolling_direction(x(3 + i)));
    }
    // Contact angles live in [-eta_max, eta_max]. One held at its bound by a
    // gradient pointing outward is fixed for this step (projected gradient).
    for (int i = 0; i < kNumWheels; ++i) {
      const int k = 3 + i;
      if ((x(k) >= eta_max && gradient(k) < 0.0) || (x(k) <= -eta_max && gradient(k) > 0.0)) {
        gradient(k) = 0.0;
        hessian.row(k).setZero();
        hessian.col(k).setZero();
        hessian(k, k) = 1.0;
      }
    }
    if (gradient.lpNorm<Eigen::Infinity>() < g.fk3d_tolerance) {
      out.converged = true;
      break;
    }
    if (out.iterations >= g.fk3d_max_iterations) {
      break;
    }

    // Damping runs up to a cap relative to the Hessian's largest entry, which
    // bounds its eigenvalues (Gershgorin: |eigenvalue| <= 7 * that entry), so
    // the damped matrix is positive definite by the cap. A fixed cap failed at
    // large rates: the contact-angle curvature grows as (R * wheel rate)^2, so
    // from about 1e7 rad/s the damped matrix stayed indefinite up to the cap.
    const double max_lambda = 1e12 * hessian.cwiseAbs().maxCoeff();
    lambda = std::min(lambda, max_lambda);
    bool improved = false;
    bool small_step = false;
    while (true) {
      const Eigen::LLT<Hessian> damped(hessian + lambda * Hessian::Identity());
      if (damped.info() == Eigen::Success) {
        State candidate = x + damped.solve(-gradient);
        for (int i = 0; i < kNumWheels; ++i) {
          candidate(3 + i) = std::clamp(candidate(3 + i), -eta_max, eta_max);
        }
        Residual candidate_r;
        evaluate(candidate, terms, eta_prior, vz_prior, candidate_r, nullptr);
        if (candidate_r.squaredNorm() < cost) {
          small_step = (candidate - x).norm() < g.fk3d_tolerance;
          x = candidate;
          lambda = std::max(lambda * 0.3, 1e-12);
          improved = true;
          break;
        }
      }
      // Not positive definite yet (the step might not descend), or no descent.
      if (lambda >= max_lambda) {
        break;
      }
      lambda = std::min(lambda * 10.0, max_lambda);
    }
    if (!improved) {
      // No damping up to the cap reduces the cost. At a minimum that is
      // rounding, and the gradient is tiny next to its bound. Otherwise the
      // steps stalled: at wheel rates beyond about 1e10 rad/s the contact-angle
      // rows outweigh the velocity rows by more than double precision resolves.
      const State bound =
          kStallTolerance * std::sqrt(cost) * jacobian.colwise().norm().transpose();
      out.converged = (gradient.array().abs() <= bound.array()).all();
      break;
    }
    ++out.iterations;
    if (small_step) {
      out.converged = true;
      break;
    }
  }

  evaluate(x, terms, eta_prior, vz_prior, r, nullptr);
  out.v_body = x.head<3>();
  for (int i = 0; i < kNumWheels; ++i) {
    out.contact_angle[i] = x(3 + i);
  }
  out.residual_rms = std::sqrt(r.head<kKinematicRows>().squaredNorm() / kKinematicRows);
  return out;
}

}  // namespace rover_driver
