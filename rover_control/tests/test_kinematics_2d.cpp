#include <cmath>

#include <gtest/gtest.h>

#include "rover_driver/kinematics_2d.hpp"
#include "test_geometry.hpp"

using namespace rover_driver;
using rover_driver::test::kExample;
using rover_driver::test::kPi;

TEST(NominalWheelXy, IsPivotPlusOffset) {
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kFL, kExample).x(), 0.5);
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kFL, kExample).y(), 0.4);
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kRR, kExample).x(), -0.5);
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kRR, kExample).y(), -0.4);

  // kExample has pivot.x == 0 and wheel_offset.y == 0, so it cannot tell a
  // sum from picking one term. Here every pivot and offset component is
  // nonzero and distinct, and the pivots differ per side.
  constexpr RoverGeometry kSkewed{
      /*wheel_radius=*/0.15,
      /*pivot=*/{{{0.05, 0.40, 0.35}, {-0.03, -0.41, 0.33}}},
      /*wheel_offset=*/
      {{{0.50, 0.02, -0.20}, {0.47, -0.02, -0.21}, {-0.52, 0.04, -0.19}, {-0.48, -0.05, -0.22}}},
      /*steer_undefined_speed=*/1e-6,
      /*steer_limit=*/2.356,
      /*fk3d_eta_prior_weight=*/1e-3,
      /*fk3d_vz_prior_weight=*/1.0,
      /*fk3d_max_contact_angle=*/1.4,
      /*fk3d_max_iterations=*/100,
      /*fk3d_tolerance=*/1e-12,
  };
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kFL, kSkewed).x(), 0.55);   // 0.05 + 0.50
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kFL, kSkewed).y(), 0.42);   // 0.40 + 0.02
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kFR, kSkewed).x(), 0.44);   // -0.03 + 0.47
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kFR, kSkewed).y(), -0.43);  // -0.41 - 0.02
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kRL, kSkewed).x(), -0.47);  // 0.05 - 0.52
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kRL, kSkewed).y(), 0.44);   // 0.40 + 0.04
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kRR, kSkewed).x(), -0.51);  // -0.03 - 0.48
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kRR, kSkewed).y(), -0.46);  // -0.41 - 0.05
}

TEST(NominalWheelXy, DefaultGeometryMatchesWheelNames) {
  EXPECT_GT(nominal_wheel_xy(kFL).x(), 0.0);
  EXPECT_GT(nominal_wheel_xy(kFL).y(), 0.0);
  EXPECT_GT(nominal_wheel_xy(kFR).x(), 0.0);
  EXPECT_LT(nominal_wheel_xy(kFR).y(), 0.0);
  EXPECT_LT(nominal_wheel_xy(kRL).x(), 0.0);
  EXPECT_GT(nominal_wheel_xy(kRL).y(), 0.0);
  EXPECT_LT(nominal_wheel_xy(kRR).x(), 0.0);
  EXPECT_LT(nominal_wheel_xy(kRR).y(), 0.0);
}

TEST(Inverse2d, StraightAheadDrivesAllWheelsForwardAtSameRate) {
  const auto cmd = inverse_2d({1.0, 0.0, 0.0}, kExample);
  for (const auto& m : cmd) {
    EXPECT_TRUE(m.steer_defined);
    EXPECT_NEAR(m.steer_angle, 0.0, 1e-12);
    EXPECT_NEAR(m.wheel_rate, 1.0 / 0.15, 1e-12);
  }

  // kExample shares kRover's wheel radius, so it cannot tell g.wheel_radius
  // from kRover.wheel_radius. Use a different radius here.
  RoverGeometry big_wheels = kExample;
  big_wheels.wheel_radius = 0.2;
  for (const auto& m : inverse_2d({1.0, 0.0, 0.0}, big_wheels)) {
    EXPECT_NEAR(m.wheel_rate, 1.0 / 0.2, 1e-12);
  }
}

TEST(Inverse2d, StrafeLeftPointsAllWheelsLeft) {
  const auto cmd = inverse_2d({0.0, 1.0, 0.0}, kExample);
  for (const auto& m : cmd) {
    EXPECT_TRUE(m.steer_defined);
    EXPECT_NEAR(m.steer_angle, kPi / 2.0, 1e-12);
    EXPECT_NEAR(m.wheel_rate, 1.0 / 0.15, 1e-12);
  }
}

TEST(Inverse2d, TurnInPlaceKeepsWheelsTangent) {
  const auto cmd = inverse_2d({0.0, 0.0, 1.0}, kExample);
  for (int i = 0; i < kNumWheels; ++i) {
    const Eigen::Vector2d r = nominal_wheel_xy(i, kExample);
    const Eigen::Vector2d dir(std::cos(cmd[i].steer_angle), std::sin(cmd[i].steer_angle));
    EXPECT_NEAR(dir.dot(r), 0.0, 1e-12) << "wheel " << i;
    EXPECT_GT(r.x() * dir.y() - r.y() * dir.x(), 0.0) << "wheel " << i << " not CCW";
    EXPECT_NEAR(cmd[i].wheel_rate, r.norm() / 0.15, 1e-12) << "wheel " << i;
  }
}

TEST(Inverse2d, WorkedExampleForwardWhileTurningLeft) {
  const auto cmd = inverse_2d({1.0, 0.0, 0.5}, kExample);
  EXPECT_NEAR(cmd[kFL].steer_angle, std::atan2(0.25, 0.8), 1e-12);
  EXPECT_NEAR(cmd[kFR].steer_angle, std::atan2(0.25, 1.2), 1e-12);
  EXPECT_NEAR(cmd[kRL].steer_angle, std::atan2(-0.25, 0.8), 1e-12);
  EXPECT_NEAR(cmd[kRR].steer_angle, std::atan2(-0.25, 1.2), 1e-12);
  EXPECT_NEAR(cmd[kFL].steer_angle * 180.0 / kPi, 17.354, 1e-3);
  EXPECT_NEAR(cmd[kFL].wheel_rate * 0.15, 0.8382, 1e-4);
  EXPECT_NEAR(cmd[kFR].wheel_rate * 0.15, 1.2258, 1e-4);
  EXPECT_NEAR(cmd[kRL].wheel_rate, cmd[kFL].wheel_rate, 1e-12);
  EXPECT_NEAR(cmd[kRR].wheel_rate, cmd[kFR].wheel_rate, 1e-12);
}

TEST(Inverse2d, ZeroCommandLeavesSteeringUndefined) {
  const auto cmd = inverse_2d({0.0, 0.0, 0.0}, kExample);
  for (const auto& m : cmd) {
    EXPECT_FALSE(m.steer_defined);
    EXPECT_EQ(m.steer_angle, 0.0);
    EXPECT_EQ(m.wheel_rate, 0.0);
  }
}

TEST(Inverse2d, SteerUndefinedSpeedGatesEachWheelStrictly) {
  // Yaw at 1 rad/s about a point next to FL (0.5, 0.4): with
  // cmd = {0.4 + eps, -0.5, 1}, FL moves at (eps, 0) while FR, RL and RR
  // move at 0.8 m/s or more. kExample's threshold is 1e-6.
  {
    const auto cmd = inverse_2d({0.4 + 5e-7, -0.5, 1.0}, kExample);
    EXPECT_FALSE(cmd[kFL].steer_defined);
    EXPECT_EQ(cmd[kFL].steer_angle, 0.0);
    EXPECT_EQ(cmd[kFL].wheel_rate, 0.0);
    for (const int i : {kFR, kRL, kRR}) {
      EXPECT_TRUE(cmd[i].steer_defined) << "wheel " << i;
      EXPECT_GT(cmd[i].wheel_rate, 0.8 / 0.15 - 1e-9) << "wheel " << i;
    }
  }
  {
    // FL speed 5e-5 is above kExample's 1e-6 but below kRover's 1e-3, so this
    // also checks that the threshold comes from g.
    const auto cmd = inverse_2d({0.4 + 5e-5, -0.5, 1.0}, kExample);
    EXPECT_TRUE(cmd[kFL].steer_defined);
    EXPECT_NEAR(cmd[kFL].steer_angle, 0.0, 1e-12);
    EXPECT_NEAR(cmd[kFL].wheel_rate, 5e-5 / 0.15, 1e-12);
  }
  {
    // Only speeds strictly below the threshold are undefined. 0.25 is exact in
    // binary and hypot(x, 0) == |x|, so every wheel sits exactly on it.
    RoverGeometry gate = kExample;
    gate.steer_undefined_speed = 0.25;
    for (const auto& m : inverse_2d({0.25, 0.0, 0.0}, gate)) {
      EXPECT_TRUE(m.steer_defined);
      EXPECT_EQ(m.steer_angle, 0.0);
    }
    for (const auto& m : inverse_2d({0.25 - 1e-9, 0.0, 0.0}, gate)) {
      EXPECT_FALSE(m.steer_defined);
      EXPECT_EQ(m.wheel_rate, 0.0);
    }
  }
}

TEST(Forward2d, RoundTripsInverse2d) {
  const BodyTwist2d twists[] = {
      {0.3, -0.2, 0.7}, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.2}, {-0.5, 0.4, 0.1}};
  for (const auto& twist : twists) {
    const auto fk = forward_2d(to_measurements(inverse_2d(twist, kExample)), kExample);
    EXPECT_NEAR(fk.twist.vx, twist.vx, 1e-12);
    EXPECT_NEAR(fk.twist.vy, twist.vy, 1e-12);
    EXPECT_NEAR(fk.twist.wz, twist.wz, 1e-12);
    EXPECT_LT(fk.residual_rms, 1e-12);
  }
}

TEST(Forward2d, ScalesWheelRateByGeometryRadius) {
  // Built by hand, independent of inverse_2d: every module steered straight
  // ahead at 5 rad/s. kExample shares kRover's wheel radius, so it cannot tell
  // g.wheel_radius from kRover.wheel_radius; a 0.2 m wheel can.
  PerWheel<ModuleMeasurement> meas{};
  for (auto& m : meas) {
    m = {0.0, 1.0 / 0.2};
  }
  RoverGeometry big_wheels = kExample;
  big_wheels.wheel_radius = 0.2;
  const auto fk = forward_2d(meas, big_wheels);
  EXPECT_NEAR(fk.twist.vx, 1.0, 1e-12);
  EXPECT_NEAR(fk.twist.vy, 0.0, 1e-12);
  EXPECT_NEAR(fk.twist.wz, 0.0, 1e-12);
  EXPECT_LT(fk.residual_rms, 1e-12);

  EXPECT_NEAR(forward_2d(meas, kExample).twist.vx, 0.15 / 0.2, 1e-12);
}

TEST(Forward2d, InconsistentWheelShowsResidual) {
  auto meas = to_measurements(inverse_2d({1.0, 0.0, 0.0}, kExample));
  meas[kFL].wheel_rate *= 1.5;  // front-left spins 50 % fast (slipping)
  const auto fk = forward_2d(meas, kExample);

  // Every module reads steer 0; the wheel speeds are (1.5, 1, 1, 1) m/s.
  // kExample's wheels sit at (+-0.5, +-0.4), so sum x_i = sum y_i = 0 and the
  // normal equations are diagonal:
  //   A^T A = diag(4, 4, 4 * (0.5^2 + 0.4^2)) = diag(4, 4, 1.64),
  //   A^T b = (1.5 + 1 + 1 + 1, 0, -0.4 * 1.5 + 0.4 - 0.4 + 0.4) = (4.5, 0, -0.2).
  // So vx = 9/8, vy = 0 and wz = -0.2 / 1.64 = -5/41: the fast left wheel
  // yaws the estimate clockwise. In units of 1/328 m/s the residuals A*x - b
  // are (-107, 25, 57, 25) along x and (-20, -20, 20, 20) along y for
  // (FL, FR, RL, RR), so the RMS over all 8 is sqrt(17548 / 8) / 328 = 0.14279.
  EXPECT_NEAR(fk.twist.vx, 9.0 / 8.0, 1e-12);
  EXPECT_NEAR(fk.twist.vy, 0.0, 1e-12);
  EXPECT_NEAR(fk.twist.wz, -5.0 / 41.0, 1e-12);
  EXPECT_NEAR(fk.residual_rms, std::sqrt(17548.0 / 8.0) / 328.0, 1e-12);
}
