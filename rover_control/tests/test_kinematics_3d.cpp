#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "rover_driver/kinematics_2d.hpp"
#include "rover_driver/kinematics_3d.hpp"
#include "test_geometry.hpp"

using namespace rover_driver;
using rover_driver::test::kExample;
using rover_driver::test::kPi;

namespace {

// World position of a wheel center at time t, for a body that starts at the
// world origin with identity attitude and moves with constant body twist
// (v_b, w_b) while the rockers move at constant rates.
Eigen::Vector3d world_wheel_center(int wheel, double t, const Eigen::Vector3d& v_b,
                                   const Eigen::Vector3d& w_b, const SuspensionState& s0) {
  SuspensionState s = s0;
  s.q_left += s0.dq_left * t;
  s.q_right += s0.dq_right * t;
  const Eigen::Matrix3d r =
      Eigen::AngleAxisd(w_b.norm() * t, w_b.normalized()).toRotationMatrix();
  const WheelChain chain = wheel_chain(wheel, v_b, w_b, s, kExample);
  return v_b * t + r * chain.center;
}

// Uniform noise in [-1, 1]. std::mt19937's output sequence is fixed by the
// standard (the std distributions are not), so the frames are portable.
class Noise {
 public:
  explicit Noise(unsigned seed) : rng_(seed) {}
  double operator()() { return 2.0 * (static_cast<double>(rng_()) / 4294967295.0) - 1.0; }

 private:
  std::mt19937 rng_;
};

// forward_3d's cost as the spec defines it, built from wheel_chain: squared
// wheel-velocity residuals (module coordinates) plus the prior rows.
double fk3d_cost(const Eigen::Vector3d& v, const PerWheel<double>& eta,
                 const PerWheel<ModuleMeasurement>& meas, const Gyro& gyro,
                 const SuspensionState& s, const RoverGeometry& g) {
  const Eigen::Vector3d w(gyro.wx, gyro.wy, gyro.wz);
  double cost = std::pow(g.fk3d_vz_prior_weight * v.z(), 2);
  for (int i = 0; i < kNumWheels; ++i) {
    const WheelChain c = wheel_chain(i, v, w, s, g);
    const Eigen::Matrix3d m = c.rocker_rot * rot_z(meas[i].steer_angle);
    const double rolling = g.wheel_radius * (meas[i].wheel_rate + m.col(1).dot(c.rocker_omega));
    const Eigen::Vector3d e = m.transpose() * c.velocity -
                              rolling * Eigen::Vector3d(std::cos(eta[i]), 0.0, std::sin(eta[i]));
    cost += e.squaredNorm() + std::pow(g.fk3d_eta_prior_weight * eta[i], 2);
  }
  return cost;
}

}  // namespace

TEST(Rotation, RotZTurnsXTowardY) {
  EXPECT_NEAR((rot_z(kPi / 2.0) * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitY()).norm(),
              0.0, 1e-12);
  EXPECT_NEAR((rot_z(kPi / 2.0) * Eigen::Vector3d::UnitY() + Eigen::Vector3d::UnitX()).norm(),
              0.0, 1e-12);
  for (double angle : {0.7, -1.3}) {
    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_TRUE(rot_z(angle).isApprox(expected, 1e-12)) << "angle " << angle;
  }
}

TEST(Rotation, RotYIsRightHandedAboutY) {
  // Right-handed about +y: x turns toward -z and z turns toward +x.
  EXPECT_NEAR((rot_y(kPi / 2.0) * Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitZ()).norm(),
              0.0, 1e-12);
  EXPECT_NEAR((rot_y(kPi / 2.0) * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitX()).norm(),
              0.0, 1e-12);
  for (double angle : {0.7, -1.3}) {
    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    EXPECT_TRUE(rot_y(angle).isApprox(expected, 1e-12)) << "angle " << angle;
  }
}

TEST(WheelChain, CenterAtZeroRockerAngleIsPivotPlusOffset) {
  const WheelChain c = wheel_chain(kFR, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                   SuspensionState{}, kExample);
  EXPECT_TRUE(c.center.isApprox(Eigen::Vector3d(0.5, -0.4, 0.15)));
}

TEST(WheelChain, PositiveRockerAngleLowersFrontWheel) {
  SuspensionState s;
  s.q_left = 0.1;
  const WheelChain front = wheel_chain(kFL, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                       s, kExample);
  const WheelChain rear = wheel_chain(kRL, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                      s, kExample);
  EXPECT_LT(front.center.z(), 0.15);
  EXPECT_GT(rear.center.z(), 0.15);
}

TEST(WheelChain, RockerAngleMovesOnlyItsOwnWheels) {
  struct Case {
    SuspensionState s;
    int front, rear, other_front, other_rear;
  };
  const Case cases[] = {
      {{0.1, 0.0, 0.0, 0.0}, kFL, kRL, kFR, kRR},
      {{0.0, 0.1, 0.0, 0.0}, kFR, kRR, kFL, kRL},
  };
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  for (const auto& c : cases) {
    EXPECT_LT(wheel_chain(c.front, zero, zero, c.s, kExample).center.z(), 0.15);
    EXPECT_GT(wheel_chain(c.rear, zero, zero, c.s, kExample).center.z(), 0.15);
    EXPECT_NEAR(wheel_chain(c.other_front, zero, zero, c.s, kExample).center.z(), 0.15, 1e-12);
    EXPECT_NEAR(wheel_chain(c.other_rear, zero, zero, c.s, kExample).center.z(), 0.15, 1e-12);
  }
}

TEST(WheelChain, EachWheelUsesItsOwnRockerAngle) {
  SuspensionState s;
  s.q_left = 0.1;
  s.q_right = -0.3;
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  for (int i = 0; i < kNumWheels; ++i) {
    const double q = kWheelSide[i] == kLeft ? 0.1 : -0.3;
    const double a = (i == kFL || i == kFR) ? 0.5 : -0.5;
    const double y = kWheelSide[i] == kLeft ? 0.4 : -0.4;
    // Rotating the offset (a, 0, -0.2) by q about +y gives
    // (a cos q - 0.2 sin q, 0, -a sin q - 0.2 cos q); the pivot is (0, y, 0.35).
    const Eigen::Vector3d expected(a * std::cos(q) - 0.2 * std::sin(q), y,
                                   0.35 - a * std::sin(q) - 0.2 * std::cos(q));
    const WheelChain c = wheel_chain(i, zero, zero, s, kExample);
    EXPECT_NEAR((c.center - expected).norm(), 0.0, 1e-12) << "wheel " << i;
    EXPECT_TRUE(c.rocker_rot.isApprox(
        Eigen::AngleAxisd(q, Eigen::Vector3d::UnitY()).toRotationMatrix(), 1e-12))
        << "wheel " << i;
  }
}

TEST(WheelChain, RockerRateMovesOnlyItsOwnWheels) {
  // Body still, rockers level, one rocker turning at 1 rad/s about +y: a wheel
  // on it moves at UnitY x (a, 0, -0.2) = (-0.2, 0, -a); the others stay still.
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  for (int moving : {kLeft, kRight}) {
    SuspensionState s;
    if (moving == kLeft) {
      s.dq_left = 1.0;
    } else {
      s.dq_right = 1.0;
    }
    for (int i = 0; i < kNumWheels; ++i) {
      const bool on_moving = kWheelSide[i] == moving;
      const double a = (i == kFL || i == kFR) ? 0.5 : -0.5;
      const Eigen::Vector3d expected_v = on_moving ? Eigen::Vector3d(-0.2, 0.0, -a) : zero;
      const Eigen::Vector3d expected_w = on_moving ? Eigen::Vector3d(0.0, 1.0, 0.0) : zero;
      const WheelChain c = wheel_chain(i, zero, zero, s, kExample);
      EXPECT_NEAR((c.velocity - expected_v).norm(), 0.0, 1e-12)
          << "moving side " << moving << ", wheel " << i;
      EXPECT_NEAR((c.rocker_omega - expected_w).norm(), 0.0, 1e-12)
          << "moving side " << moving << ", wheel " << i;
    }
  }
}

TEST(WheelChain, RockerOmegaIsBodyRatePlusOwnRockerRate) {
  const Eigen::Vector3d v_b(0.7, -0.2, 0.05);
  const Eigen::Vector3d w_b(0.1, -0.3, 0.4);
  SuspensionState s;
  s.q_left = 0.12;
  s.q_right = -0.2;
  s.dq_left = 0.3;
  s.dq_right = -0.7;
  for (int i = 0; i < kNumWheels; ++i) {
    const double dq = kWheelSide[i] == kLeft ? 0.3 : -0.7;
    const Eigen::Vector3d expected = w_b + Eigen::Vector3d(0.0, dq, 0.0);
    const WheelChain c = wheel_chain(i, v_b, w_b, s, kExample);
    EXPECT_NEAR((c.rocker_omega - expected).norm(), 0.0, 1e-12) << "wheel " << i;
  }
}

TEST(WheelChain, VelocityMatchesFiniteDifferenceOfPosition) {
  struct Case {
    Eigen::Vector3d v_b;
    Eigen::Vector3d w_b;
    SuspensionState s;
  };
  const Case cases[] = {
      {{0.7, -0.2, 0.05}, {0.1, -0.3, 0.4}, {0.12, -0.12, 0.3, -0.3}},
      {{-0.3, 0.5, -0.1}, {-0.2, 0.25, -0.6}, {-0.2, 0.15, -0.5, 0.2}},
      {{0.0, 0.0, 0.0}, {0.4, 0.1, 0.0}, {0.05, -0.05, 0.0, 0.0}},
      {{0.2, 0.1, 0.0}, {0.1, 0.2, -0.3}, {0.3, -0.25, 0.0, 0.4}},
  };
  const double h = 1e-6;
  for (const auto& c : cases) {
    for (int i = 0; i < kNumWheels; ++i) {
      const Eigen::Vector3d fd = (world_wheel_center(i, h, c.v_b, c.w_b, c.s) -
                                  world_wheel_center(i, -h, c.v_b, c.w_b, c.s)) /
                                 (2.0 * h);
      const WheelChain chain = wheel_chain(i, c.v_b, c.w_b, c.s, kExample);
      EXPECT_NEAR(chain.velocity.x(), fd.x(), 1e-7) << "wheel " << i;
      EXPECT_NEAR(chain.velocity.y(), fd.y(), 1e-7) << "wheel " << i;
      EXPECT_NEAR(chain.velocity.z(), fd.z(), 1e-7) << "wheel " << i;
    }
  }
}

TEST(Inverse3d, MatchesInverse2dWhenLevelAndStill) {
  const BodyTwist2d twists[] = {
      {1.0, 0.0, 0.5}, {0.3, -0.2, 0.7}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, 0.0}};
  for (const auto& geometry : {kRover, kExample}) {
    for (const auto& twist : twists) {
      const auto c2 = inverse_2d(twist, geometry);
      const auto c3 = inverse_3d(twist, Gyro{0.0, 0.0, twist.wz}, SuspensionState{}, geometry);
      for (int i = 0; i < kNumWheels; ++i) {
        EXPECT_EQ(c3[i].steer_defined, c2[i].steer_defined) << "wheel " << i;
        EXPECT_NEAR(c3[i].steer_angle, c2[i].steer_angle, 1e-12) << "wheel " << i;
        EXPECT_NEAR(c3[i].wheel_rate, c2[i].wheel_rate, 1e-12) << "wheel " << i;
        EXPECT_NEAR(c3[i].contact_angle, 0.0, 1e-12) << "wheel " << i;
      }
    }
  }
}

TEST(Inverse3d, FrontLeftClimbingSpeedsUpAndTiltsContact) {
  SuspensionState s;
  s.dq_left = -0.5;  // left rocker pitching nose-up: FL climbs, RL descends
  const auto cmd = inverse_3d({0.3, 0.0, 0.0}, Gyro{}, s, kExample);
  EXPECT_GT(cmd[kFL].contact_angle, 0.1);
  EXPECT_LT(cmd[kRL].contact_angle, -0.1);
  EXPECT_NEAR(cmd[kFR].contact_angle, 0.0, 1e-12);
  EXPECT_NEAR(cmd[kRR].contact_angle, 0.0, 1e-12);
  EXPECT_GT(cmd[kFL].wheel_rate, cmd[kFR].wheel_rate);
  EXPECT_NEAR(cmd[kFR].wheel_rate, 0.3 / kExample.wheel_radius, 1e-12);
  EXPECT_NEAR(cmd[kRR].wheel_rate, 0.3 / kExample.wheel_radius, 1e-12);

  // Exact values. Left wheel centers move at
  // (0.3, 0, 0) + (-0.5 y) x (+-0.5, 0, -0.2) = (0.4, 0, +-0.25). The fork turns
  // at -0.5 rad/s about the axle, so relative to the fork the wheel must spin
  // 0.5 rad/s faster than |v|/R to roll without slip.
  const double left_rate = std::hypot(0.4, 0.25) / kExample.wheel_radius + 0.5;
  EXPECT_NEAR(cmd[kFL].steer_angle, 0.0, 1e-12);
  EXPECT_NEAR(cmd[kRL].steer_angle, 0.0, 1e-12);
  EXPECT_NEAR(cmd[kFL].contact_angle, std::atan2(0.25, 0.4), 1e-12);
  EXPECT_NEAR(cmd[kRL].contact_angle, std::atan2(-0.25, 0.4), 1e-12);
  EXPECT_NEAR(cmd[kFL].wheel_rate, left_rate, 1e-12);
  EXPECT_NEAR(cmd[kRL].wheel_rate, left_rate, 1e-12);
}

TEST(Inverse3d, StaticEqualRockerAnglesTiltContactOnly) {
  SuspensionState s;
  s.q_left = 0.2;
  s.q_right = 0.2;
  const auto cmd = inverse_3d({0.4, 0.0, 0.0}, Gyro{}, s, kExample);
  for (int i = 0; i < kNumWheels; ++i) {
    EXPECT_NEAR(cmd[i].steer_angle, 0.0, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].wheel_rate, 0.4 / kExample.wheel_radius, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].contact_angle, 0.2, 1e-12) << "wheel " << i;
  }
}

TEST(Inverse3d, StaticTwistTiltsContactPerSide) {
  SuspensionState s;
  s.q_left = 0.2;
  s.q_right = -0.2;
  const auto cmd = inverse_3d({0.4, 0.0, 0.0}, Gyro{}, s, kExample);
  for (int i = 0; i < kNumWheels; ++i) {
    const double expected = kWheelSide[i] == kLeft ? 0.2 : -0.2;
    EXPECT_NEAR(cmd[i].steer_angle, 0.0, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].wheel_rate, 0.4 / kExample.wheel_radius, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].contact_angle, expected, 1e-12) << "wheel " << i;
  }
}

TEST(Inverse3d, SteersInTheRockerFrame) {
  // Both rockers tilted 0.3 rad while crabbing. The wheel-center velocity
  // (0.4, 0.3, 0) in body coordinates is (0.4 cos 0.3, 0.3, 0.4 sin 0.3) in the
  // rocker frame, so the steering angle is not the body-frame atan2(0.3, 0.4).
  SuspensionState s;
  s.q_left = 0.3;
  s.q_right = 0.3;
  const auto cmd = inverse_3d({0.4, 0.3, 0.0}, Gyro{}, s, kExample);
  const double ux = 0.4 * std::cos(0.3);
  const double uz = 0.4 * std::sin(0.3);
  for (int i = 0; i < kNumWheels; ++i) {
    EXPECT_TRUE(cmd[i].steer_defined) << "wheel " << i;
    EXPECT_NEAR(cmd[i].steer_angle, std::atan2(0.3, ux), 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].wheel_rate, 0.5 / kExample.wheel_radius, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].contact_angle, std::atan2(uz, std::hypot(ux, 0.3)), 1e-12)
        << "wheel " << i;
  }
}

TEST(Inverse3d, GyroPitchRateTiltsContactAndSlowsWheels) {
  // Body pitching nose-down at wy = 0.2 rad/s while driving at 0.3 m/s, rockers
  // level and still. Wheel centers are at (+-0.5, +-0.4, 0.15), so
  // v = (0.3, 0, 0) + 0.2 y x (x_i, y_i, 0.15) = (0.33, 0, -0.1) at the front and
  // (0.33, 0, 0.1) at the rear. The fork turns with the body at +0.2 rad/s about
  // the axle, so relative to it the wheel spins 0.2 rad/s slower than |v|/R.
  const auto cmd =
      inverse_3d({0.3, 0.0, 0.0}, Gyro{0.0, 0.2, 0.0}, SuspensionState{}, kExample);
  const double rate = std::hypot(0.33, 0.1) / kExample.wheel_radius - 0.2;
  for (int i = 0; i < kNumWheels; ++i) {
    const bool front = i == kFL || i == kFR;
    EXPECT_NEAR(cmd[i].steer_angle, 0.0, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].wheel_rate, rate, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].contact_angle, std::atan2(front ? -0.1 : 0.1, 0.33), 1e-12)
        << "wheel " << i;
  }
}

TEST(Inverse3d, CommandsRollWithoutSlipInGeneralMotion) {
  // Nonzero roll/pitch rates, yaw, crabbing, rocker angles and rocker rates.
  // Checks the commands against the rolling model directly: the wheel-center
  // velocity lies along the rolling direction t of the commanded module, and
  // the contact point (-R n from the center) has no velocity along t.
  struct Case {
    BodyTwist2d cmd;
    Gyro gyro;
    SuspensionState s;
  };
  const Case cases[] = {
      {{0.5, 0.2, 0.3}, {0.15, -0.25, 0.0}, {0.2, -0.1, 0.4, -0.3}},
      {{-0.3, 0.4, -0.6}, {-0.3, 0.1, 0.0}, {-0.25, 0.3, -0.2, 0.5}},
      {{0.2, -0.5, 0.8}, {0.05, 0.3, 0.0}, {0.1, 0.15, 0.0, -0.6}},
  };
  for (const auto& g : {kRover, kExample}) {
    for (const auto& c : cases) {
      const auto cmd = inverse_3d(c.cmd, c.gyro, c.s, g);
      const Eigen::Vector3d v_body(c.cmd.vx, c.cmd.vy, 0.0);
      const Eigen::Vector3d w_body(c.gyro.wx, c.gyro.wy, c.cmd.wz);
      for (int i = 0; i < kNumWheels; ++i) {
        const bool left = kWheelSide[i] == kLeft;
        const double q = left ? c.s.q_left : c.s.q_right;
        const double dq = left ? c.s.dq_left : c.s.dq_right;
        const Eigen::Vector3d fork_omega = w_body + dq * Eigen::Vector3d::UnitY();
        // Center velocity from wheel_chain, which is checked against finite differences.
        const Eigen::Vector3d v = wheel_chain(i, v_body, w_body, c.s, g).velocity;

        ASSERT_TRUE(cmd[i].steer_defined) << "wheel " << i;
        const Eigen::Matrix3d m = rot_y(q) * rot_z(cmd[i].steer_angle);  // module -> body
        const double eta = cmd[i].contact_angle;
        const Eigen::Vector3d t = m * Eigen::Vector3d(std::cos(eta), 0.0, std::sin(eta));
        const Eigen::Vector3d n = m * Eigen::Vector3d(-std::sin(eta), 0.0, std::cos(eta));
        const Eigen::Vector3d axle = m.col(1);

        EXPECT_NEAR(v.dot(axle), 0.0, 1e-12) << "wheel " << i;
        EXPECT_NEAR(v.dot(n), 0.0, 1e-12) << "wheel " << i;
        EXPECT_GT(v.dot(t), 0.0) << "wheel " << i;

        const Eigen::Vector3d w_wheel = fork_omega + cmd[i].wheel_rate * axle;
        const Eigen::Vector3d v_contact = v + w_wheel.cross(-g.wheel_radius * n);
        EXPECT_NEAR(v_contact.dot(t), 0.0, 1e-12) << "wheel " << i;
      }
    }
  }
}

TEST(Inverse3d, SteeringUndefinedBelowThresholdInTheRockerFrame) {
  // Left rocker at q = 0.3 turning at dq = -0.5 rad/s. In the rocker frame the
  // offsets (+-0.5, 0, -0.2) move at dq * y x offset = dq * (-0.2, 0, -+0.5),
  // and the body velocity (vx, vy, 0) is (vx cos q, vy, vx sin q). With
  // vx = 0.2 dq / cos q the rocker-x parts cancel, so both left wheels move
  // along their rocker's z axis (steering axis) at 0.22 and -0.28 m/s, plus vy
  // along rocker y. Only vy may decide whether steering is defined, although
  // their body-frame horizontal speed stays above 0.06 m/s.
  RoverGeometry g = kExample;
  g.steer_undefined_speed = 0.01;
  const double q = 0.3;
  SuspensionState s;
  s.q_left = q;
  s.dq_left = -0.5;
  const double vx = 0.2 * s.dq_left / std::cos(q);
  for (double vy : {0.0, 0.005, 0.02}) {  // below, below and above the threshold
    const auto cmd = inverse_3d({vx, vy, 0.0}, Gyro{}, s, g);
    for (int i : {kFL, kRL}) {
      SCOPED_TRACE(testing::Message() << "vy " << vy << ", wheel " << i);
      const Eigen::Vector3d v =
          wheel_chain(i, Eigen::Vector3d(vx, vy, 0.0), Eigen::Vector3d::Zero(), s, g).velocity;
      const Eigen::Vector3d u = rot_y(q).transpose() * v;  // rocker frame
      ASSERT_NEAR(u.x(), 0.0, 1e-15);
      ASSERT_GT(std::abs(u.z()), 0.2);
      ASSERT_GT(std::hypot(v.x(), v.y()), 0.06);
      if (vy < g.steer_undefined_speed) {
        EXPECT_FALSE(cmd[i].steer_defined);
        EXPECT_EQ(cmd[i].steer_angle, 0.0);
        EXPECT_EQ(cmd[i].wheel_rate, 0.0);
        EXPECT_EQ(cmd[i].contact_angle, 0.0);
      } else {
        // Steered along rocker y, so the axle is along rocker -x, normal to the
        // rocker's rotation: no fork spin.
        EXPECT_TRUE(cmd[i].steer_defined);
        EXPECT_NEAR(cmd[i].steer_angle, kPi / 2.0, 1e-12);
        EXPECT_NEAR(cmd[i].wheel_rate, std::hypot(vy, u.z()) / g.wheel_radius, 1e-12);
        EXPECT_NEAR(cmd[i].contact_angle, std::atan2(u.z(), vy), 1e-12);
      }
    }
    EXPECT_TRUE(cmd[kFR].steer_defined);  // right rocker level: moves at (vx, vy, 0)
    EXPECT_TRUE(cmd[kRR].steer_defined);
  }
}

TEST(Forward3d, RoundTripsInverse3d) {
  struct Case {
    BodyTwist2d twist;
    Gyro gyro;  // gyro.wz must equal twist.wz for a consistent round trip
    SuspensionState s;
  };
  const Case cases[] = {
      {{0.4, 0.1, 0.3}, {0.05, -0.08, 0.3}, {0.1, -0.1, 0.2, -0.2}},
      {{1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, -0.5, 0.0}},
      {{-0.3, 0.6, -0.4}, {-0.1, 0.1, -0.4}, {-0.15, 0.15, 0.1, -0.1}},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(testing::Message() << "case " << (&c - cases));
    const auto ik = inverse_3d(c.twist, c.gyro, c.s, kExample);
    const auto fk = forward_3d(to_measurements(ik), c.gyro, c.s, kExample);
    // The contact-angle prior shifts each contact angle by up to about
    // (prior / wheel speed)^2, ~1e-5 rad here (the slowest wheel moves at
    // ~0.26 m/s), and v_body by ~1e-6 m/s. These tolerances hold for these
    // cases, not as a general bound.
    EXPECT_TRUE(fk.converged);
    EXPECT_NEAR(fk.v_body.x(), c.twist.vx, 1e-5);
    EXPECT_NEAR(fk.v_body.y(), c.twist.vy, 1e-5);
    EXPECT_NEAR(fk.v_body.z(), 0.0, 1e-5);
    for (int i = 0; i < kNumWheels; ++i) {
      EXPECT_NEAR(fk.contact_angle[i], ik[i].contact_angle, 1e-4) << "wheel " << i;
    }
    EXPECT_LT(fk.residual_rms, 1e-5);
  }
}

TEST(Forward3d, StationaryRoverConvergesToZero) {
  const auto fk = forward_3d(PerWheel<ModuleMeasurement>{}, Gyro{}, SuspensionState{}, kExample);
  EXPECT_TRUE(fk.converged);
  EXPECT_NEAR(fk.v_body.norm(), 0.0, 1e-12);
  for (double eta : fk.contact_angle) {
    EXPECT_NEAR(eta, 0.0, 1e-12);
  }
}

TEST(Forward3d, InconsistentWheelShowsResidual) {
  auto meas = to_measurements(inverse_3d({1.0, 0.0, 0.0}, Gyro{}, SuspensionState{}, kExample));
  meas[kFL].wheel_rate *= 1.5;
  const auto fk = forward_3d(meas, Gyro{}, SuspensionState{}, kExample);
  EXPECT_GT(fk.residual_rms, 0.01);
}

TEST(Forward3d, SlipIsNotExplainedAsVerticalMotion) {
  // Driving straight at 1 m/s while pitching slowly, FL slipping 10%. Wheel
  // speeds see v_z only through the small front/rear difference the pitch rate
  // causes, so without the v_z prior LM tilts every contact angle together and
  // reports mostly vertical motion (v = (0.17, 0, 0.96) at wy = 0.01, contact
  // angles at their 1.4 rad limit). With zero gyro the forward_2d start is
  // already stationary, hence the pitch rate.
  for (double wy : {0.01, 0.1}) {
    SCOPED_TRACE(testing::Message() << "wy " << wy);
    const Gyro gyro{0.0, wy, 0.0};
    auto meas = to_measurements(inverse_3d({1.0, 0.0, 0.0}, gyro, SuspensionState{}, kExample));
    meas[kFL].wheel_rate *= 0.9;
    const auto fk = forward_3d(meas, gyro, SuspensionState{}, kExample);
    EXPECT_TRUE(fk.converged);
    EXPECT_GT(fk.iterations, 0);
    EXPECT_NEAR(fk.v_body.x(), 0.975, 0.005);  // mean wheel speed
    EXPECT_NEAR(fk.v_body.y(), 0.0, 1e-9);
    EXPECT_LT(std::abs(fk.v_body.z()), 0.01);
    for (int i = 0; i < kNumWheels; ++i) {
      EXPECT_LT(std::abs(fk.contact_angle[i]), 0.1) << "wheel " << i;
    }
    EXPECT_GT(fk.residual_rms, 0.01);
  }
}

TEST(Forward3d, PriorKeepsNearlyStoppedWheelContactAngleNearZero) {
  // Pivoting about FL while pitching at 2 mrad/s: FL's center only moves
  // (0.0003, 0, -0.001) m/s. Its wheel reads zero (stalled), so the rolling
  // speed is just the fork spin, R * wy = 3e-4 m/s. Speed times velocity is
  // below the prior weight squared, so the prior, not this noise-level motion,
  // decides eta_FL; without it eta_FL follows the motion to -1.2 rad.
  const BodyTwist2d twist{0.2, -0.25, 0.5};
  const Gyro gyro{0.0, 0.002, 0.5};
  auto meas = to_measurements(inverse_3d(twist, gyro, SuspensionState{}, kExample));
  meas[kFL].wheel_rate = 0.0;
  const auto fk = forward_3d(meas, gyro, SuspensionState{}, kExample);
  EXPECT_TRUE(fk.converged);
  EXPECT_LT(std::abs(fk.contact_angle[kFL]), 0.5);
  EXPECT_NEAR(fk.v_body.x(), twist.vx, 1e-3);
  EXPECT_NEAR(fk.v_body.y(), twist.vy, 1e-3);
}

TEST(Forward3d, IterationLimitReportsNotConverged) {
  const Gyro gyro{0.05, -0.08, 0.3};
  const SuspensionState s{0.1, -0.1, 0.2, -0.2};
  const auto meas = to_measurements(inverse_3d({0.4, 0.1, 0.3}, gyro, s, kExample));
  const auto full = forward_3d(meas, gyro, s, kExample);
  ASSERT_TRUE(full.converged);
  ASSERT_GT(full.iterations, 1);

  // Exactly enough steps: the last iterate is still tested, so it converges.
  RoverGeometry g = kExample;
  g.fk3d_max_iterations = full.iterations;
  const auto enough = forward_3d(meas, gyro, s, g);
  EXPECT_TRUE(enough.converged);
  EXPECT_EQ(enough.iterations, full.iterations);
  EXPECT_NEAR((enough.v_body - full.v_body).norm(), 0.0, 1e-15);

  // One step short: the limit is hit.
  g.fk3d_max_iterations = full.iterations - 1;
  const auto cut = forward_3d(meas, gyro, s, g);
  EXPECT_FALSE(cut.converged);
  EXPECT_EQ(cut.iterations, full.iterations - 1);
}

TEST(Forward3d, NonFiniteInputIsNotConverged) {
  // Every output is NaN: also v_body and the contact angles when only the gyro
  // or the rockers are bad (the forward_2d start reads neither, so it stays
  // finite), and when finite input is so large that the cost overflows.
  const auto consistent =
      to_measurements(inverse_3d({1.0, 0.0, 0.0}, Gyro{}, SuspensionState{}, kExample));
  const auto expect_no_estimate = [](const Fk3dResult& fk, const char* source) {
    SCOPED_TRACE(source);
    EXPECT_FALSE(fk.converged);
    EXPECT_TRUE(fk.v_body.array().isNaN().all()) << fk.v_body.transpose();
    for (double eta : fk.contact_angle) {
      EXPECT_TRUE(std::isnan(eta)) << eta;
    }
    EXPECT_TRUE(std::isnan(fk.residual_rms)) << fk.residual_rms;
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (double bad : {nan, inf, 1e300}) {
    SCOPED_TRACE(testing::Message() << "bad " << bad);
    auto meas = consistent;
    meas[kFL].wheel_rate = bad;
    expect_no_estimate(forward_3d(meas, Gyro{}, SuspensionState{}, kExample), "wheel rate");
    expect_no_estimate(forward_3d(consistent, Gyro{bad, 0.0, 0.0}, SuspensionState{}, kExample),
                       "gyro");
    expect_no_estimate(
        forward_3d(consistent, Gyro{}, SuspensionState{0.0, 0.0, 0.0, bad}, kExample),
        "rocker rate");
  }
  for (double bad : {nan, inf}) {  // a finite angle only turns the rocker
    SCOPED_TRACE(testing::Message() << "bad " << bad);
    expect_no_estimate(
        forward_3d(consistent, Gyro{}, SuspensionState{bad, 0.0, 0.0, 0.0}, kExample),
        "rocker angle");
  }
}

TEST(Forward3d, SolutionScalesWithHugeRates) {
  // Without the contact-angle prior the cost is homogeneous: scaling every
  // speed and rate by k scales v_body by k and keeps the contact angles. A
  // reversed FL wheel makes the contact-angle curvature strongly negative at
  // the start, and it grows as (R * wheel rate)^2. With a fixed damping cap of
  // 1e12 the damped Hessian stayed indefinite up to the cap from k = 1e7 on,
  // so the forward_2d start (zero contact angles) was reported as converged
  // after 0 steps.
  RoverGeometry g = kExample;
  g.fk3d_eta_prior_weight = 0.0;
  const auto solve = [&g](double k) {
    const Gyro gyro{0.05 * k, -0.08 * k, 0.3 * k};
    const SuspensionState s{0.1, -0.1, 0.2 * k, -0.2 * k};
    auto meas = to_measurements(inverse_3d({0.4 * k, 0.1 * k, 0.3 * k}, gyro, s, g));
    meas[kFL].wheel_rate *= -1.0;
    meas[kRL].wheel_rate *= 1.3;
    return forward_3d(meas, gyro, s, g);
  };
  const Fk3dResult base = solve(1.0);
  ASSERT_TRUE(base.converged);
  ASSERT_GT(base.residual_rms, 0.05);  // inconsistent data, as intended
  const auto matches_base = [&base](const Fk3dResult& fk, double k) {
    bool same = (fk.v_body / k - base.v_body).norm() < 1e-9 &&
                std::abs(fk.residual_rms / k - base.residual_rms) < 1e-9;
    for (int i = 0; i < kNumWheels; ++i) {
      same = same && std::abs(fk.contact_angle[i] - base.contact_angle[i]) < 1e-9;
    }
    return same;
  };
  for (double k : {1e7, 1e9}) {
    SCOPED_TRACE(testing::Message() << "k " << k);
    const Fk3dResult fk = solve(k);
    EXPECT_TRUE(fk.converged);
    EXPECT_GT(fk.iterations, 0);
    EXPECT_TRUE(matches_base(fk, k)) << "v_body / k " << (fk.v_body / k).transpose();
  }
  // From about k = 1e11 the contact-angle rows outweigh the velocity rows by
  // more than double precision resolves and the steps stall near the start.
  // Such a result must not be reported as converged.
  for (double k : {1e11, 1e12}) {
    SCOPED_TRACE(testing::Message() << "k " << k);
    const Fk3dResult fk = solve(k);
    EXPECT_TRUE(!fk.converged || matches_base(fk, k))
        << "converged at v_body / k " << (fk.v_body / k).transpose();
  }
}

TEST(Forward3d, VzPriorUnderestimatesPitchAboutRearAxle) {
  // Front wheels climbing a step: the body pitches nose-up at 0.3 rad/s about
  // the rear contact line while the rear wheels roll on level ground. The
  // measurements come from inverse_3d on a copy of kExample whose origin is that
  // line (pivots 0.5 m further forward), where the body moves at (0.3, 0, 0).
  // At kExample's origin it moves at (0.3, 0, 0) + (0, -0.3, 0) x (0.5, 0, 0)
  // = (0.3, 0, 0.15).
  RoverGeometry rear_origin = kExample;
  for (Vec3& p : rear_origin.pivot) {
    p.x += 0.5;
  }
  const Gyro gyro{0.0, -0.3, 0.0};
  const auto ik = inverse_3d({0.3, 0.0, 0.0}, gyro, SuspensionState{}, rear_origin);
  const auto meas = to_measurements(ik);
  ASSERT_NEAR(ik[kRL].contact_angle, 0.0, 1e-12);
  ASSERT_NEAR(ik[kRR].contact_angle, 0.0, 1e-12);

  // Without the v_z prior, rotation makes this motion fully observable.
  RoverGeometry no_vz_prior = kExample;
  no_vz_prior.fk3d_vz_prior_weight = 0.0;
  const auto exact = forward_3d(meas, gyro, SuspensionState{}, no_vz_prior);
  EXPECT_TRUE(exact.converged);
  EXPECT_NEAR(exact.v_body.x(), 0.3, 1e-5);
  EXPECT_NEAR(exact.v_body.y(), 0.0, 1e-5);
  EXPECT_NEAR(exact.v_body.z(), 0.15, 1e-5);
  for (int i = 0; i < kNumWheels; ++i) {
    EXPECT_NEAR(exact.contact_angle[i], ik[i].contact_angle, 1e-4) << "wheel " << i;
  }
  EXPECT_LT(exact.residual_rms, 1e-5);

  // The default prior assumes pitching about the origin: it keeps under half
  // of v_z (0.069), the level rear wheels appear to descend (-0.28 rad), the
  // front ones climb 0.2 rad less, and residual_rms (0.022) flags consistent
  // data. Changing fk3d_vz_prior_weight moves this trade-off.
  const auto biased = forward_3d(meas, gyro, SuspensionState{}, kExample);
  EXPECT_TRUE(biased.converged);
  EXPECT_GT(biased.v_body.z(), 0.0);
  EXPECT_LT(biased.v_body.z(), 0.075);
  EXPECT_LT(biased.contact_angle[kRL], -0.2);
  EXPECT_LT(biased.contact_angle[kRR], -0.2);
  EXPECT_LT(biased.contact_angle[kFL], ik[kFL].contact_angle - 0.15);
  EXPECT_GT(biased.residual_rms, 0.01);
}

TEST(Forward3d, WheelTurningAgainstItsMotionShowsResidual) {
  // Driving straight at 0.5 m/s while pitching, FL's rate reversed (sign fault)
  // or 1.5x (slip). Unbounded, the reversed wheel was explained as rolling
  // backward with contact angle 3.09 and residual_rms 0.004 at wy = 0.05.
  for (double wy : {0.05, 0.3}) {
    const Gyro gyro{0.0, wy, 0.0};
    const auto consistent =
        to_measurements(inverse_3d({0.5, 0.0, 0.0}, gyro, SuspensionState{}, kExample));
    for (double factor : {-1.0, 1.5}) {
      SCOPED_TRACE(testing::Message() << "wy " << wy << ", FL rate x " << factor);
      auto meas = consistent;
      meas[kFL].wheel_rate *= factor;
      const auto fk = forward_3d(meas, gyro, SuspensionState{}, kExample);
      EXPECT_TRUE(fk.converged);
      EXPECT_GT(fk.residual_rms, 0.05);
      for (int i = 0; i < kNumWheels; ++i) {
        EXPECT_LE(std::abs(fk.contact_angle[i]), kExample.fk3d_max_contact_angle) << "wheel " << i;
      }
    }
  }
}

TEST(Forward3d, ConvergesWithinContactAngleLimitOnNoisyInput) {
  // Alternating frames: a stopped rover whose encoders read noise, and driving
  // with up to 30% wheel slip plus sensor noise. Rolling speeds that disagree
  // with the wheel motion made plain Gauss-Newton crawl: with kRover's limit of
  // 50 iterations, 12 of these stopped and 8 of these driving frames hit it.
  // Unbounded, contact angles followed the noise past 1.4 rad in 92 of the
  // stopped frames.
  for (const auto& g : {kRover, kExample}) {
    Noise u(1);
    for (int frame = 0; frame < 200; ++frame) {
      SCOPED_TRACE(testing::Message() << "frame " << frame << ", max iterations "
                                      << g.fk3d_max_iterations);
      PerWheel<ModuleMeasurement> meas{};
      Gyro gyro;
      SuspensionState s;
      if (frame % 2 == 0) {
        for (auto& m : meas) {
          m.steer_angle = 3.0 * u();
          m.wheel_rate = 0.02 * u();
        }
        gyro = {0.01 * u(), 0.01 * u(), 0.01 * u()};
        s = {0.3 * u(), 0.3 * u(), 0.01 * u(), 0.01 * u()};
      } else {
        const BodyTwist2d twist{u(), u(), u()};
        gyro = {0.2 * u(), 0.2 * u(), twist.wz};
        s = {0.3 * u(), 0.3 * u(), 0.3 * u(), 0.3 * u()};
        meas = to_measurements(inverse_3d(twist, gyro, s, g));
        for (auto& m : meas) {
          m.wheel_rate *= 1.0 + 0.3 * u();
          m.steer_angle += 0.02 * u();
        }
        gyro.wx += 0.01 * u();
        gyro.wy += 0.01 * u();
        gyro.wz += 0.01 * u();
      }
      const auto fk = forward_3d(meas, gyro, s, g);
      EXPECT_TRUE(fk.converged);
      for (int i = 0; i < kNumWheels; ++i) {
        EXPECT_LE(std::abs(fk.contact_angle[i]), g.fk3d_max_contact_angle) << "wheel " << i;
      }
    }
  }
}

TEST(Forward3d, ReturnsStationaryPointOfItsCost) {
  // Central differences of the cost built independently from wheel_chain must
  // vanish at the solution, in every unknown including v_z (where the v_z
  // prior acts). Inputs: slip while pitching slowly, where the priors matter
  // most, and noisy driving frames with interior contact angles.
  struct Case {
    PerWheel<ModuleMeasurement> meas;
    Gyro gyro;
    SuspensionState s;
  };
  std::vector<Case> cases;
  for (double wy : {0.01, 0.1}) {
    const Gyro gyro{0.0, wy, 0.0};
    auto meas = to_measurements(inverse_3d({1.0, 0.0, 0.0}, gyro, SuspensionState{}, kExample));
    meas[kFL].wheel_rate *= 0.9;
    cases.push_back({meas, gyro, SuspensionState{}});
  }
  Noise u(2);
  for (int k = 0; k < 4; ++k) {
    const BodyTwist2d twist{u(), u(), u()};
    Gyro gyro{0.2 * u(), 0.2 * u(), twist.wz};
    const SuspensionState s{0.3 * u(), 0.3 * u(), 0.3 * u(), 0.3 * u()};
    auto meas = to_measurements(inverse_3d(twist, gyro, s, kExample));
    for (auto& m : meas) {
      m.wheel_rate *= 1.0 + 0.1 * u();
      m.steer_angle += 0.02 * u();
    }
    gyro.wy += 0.01 * u();
    cases.push_back({meas, gyro, s});
  }

  const double h = 1e-6;
  for (const auto& c : cases) {
    SCOPED_TRACE(testing::Message() << "case " << (&c - cases.data()));
    const auto fk = forward_3d(c.meas, c.gyro, c.s, kExample);
    ASSERT_TRUE(fk.converged);
    for (double eta : fk.contact_angle) {
      ASSERT_LT(std::abs(eta), kExample.fk3d_max_contact_angle);  // interior minimum
    }
    for (int k = 0; k < 3 + kNumWheels; ++k) {
      Eigen::Vector3d v_plus = fk.v_body;
      Eigen::Vector3d v_minus = fk.v_body;
      PerWheel<double> eta_plus = fk.contact_angle;
      PerWheel<double> eta_minus = fk.contact_angle;
      if (k < 3) {
        v_plus(k) += h;
        v_minus(k) -= h;
      } else {
        eta_plus[k - 3] += h;
        eta_minus[k - 3] -= h;
      }
      const double gradient =
          (fk3d_cost(v_plus, eta_plus, c.meas, c.gyro, c.s, kExample) -
           fk3d_cost(v_minus, eta_minus, c.meas, c.gyro, c.s, kExample)) /
          (2.0 * h);
      EXPECT_NEAR(gradient, 0.0, 1e-8) << "unknown " << k;
    }
  }
}
