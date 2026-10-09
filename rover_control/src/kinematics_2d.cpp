#include "rover_driver/kinematics_2d.hpp"

#include <cassert>
#include <cmath>

#include <Eigen/Dense>

namespace rover_driver {

Eigen::Vector2d nominal_wheel_xy(int wheel, const RoverGeometry& g) noexcept {
  assert(wheel >= 0 && wheel < kNumWheels);
  const Vec3& p = g.pivot[kWheelSide[wheel]];
  const Vec3& a = g.wheel_offset[wheel];
  return {p.x + a.x, p.y + a.y};
}

PerWheel<ModuleCommand> inverse_2d(const BodyTwist2d& cmd, const RoverGeometry& g) noexcept {
  PerWheel<ModuleCommand> out{};
  for (int i = 0; i < kNumWheels; ++i) {
    const Eigen::Vector2d r = nominal_wheel_xy(i, g);
    const double vx = cmd.vx - cmd.wz * r.y();
    const double vy = cmd.vy + cmd.wz * r.x();
    const double speed = std::hypot(vx, vy);
    if (speed < g.steer_undefined_speed) {
      continue;
    }
    out[i].steer_angle = std::atan2(vy, vx);
    out[i].wheel_rate = speed / g.wheel_radius;
    out[i].steer_defined = true;
  }
  return out;
}

Fk2dResult forward_2d(const PerWheel<ModuleMeasurement>& meas, const RoverGeometry& g) noexcept {
  // Each module: R * rate * [cos psi, sin psi] = [vx - wz*y, vy + wz*x].
  Eigen::Matrix<double, 2 * kNumWheels, 3> a;
  Eigen::Matrix<double, 2 * kNumWheels, 1> b;
  for (int i = 0; i < kNumWheels; ++i) {
    const Eigen::Vector2d r = nominal_wheel_xy(i, g);
    const double speed = g.wheel_radius * meas[i].wheel_rate;
    a.row(2 * i) << 1.0, 0.0, -r.y();
    a.row(2 * i + 1) << 0.0, 1.0, r.x();
    b(2 * i) = speed * std::cos(meas[i].steer_angle);
    b(2 * i + 1) = speed * std::sin(meas[i].steer_angle);
  }
  const Eigen::Vector3d x = a.colPivHouseholderQr().solve(b);

  Fk2dResult out;
  out.twist = {x(0), x(1), x(2)};
  out.residual_rms = std::sqrt((a * x - b).squaredNorm() / (2 * kNumWheels));
  return out;
}

}  // namespace rover_driver
