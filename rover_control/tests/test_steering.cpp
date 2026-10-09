#include <cmath>
#include <limits>
#include <random>

#include <gtest/gtest.h>

#include "rover_driver/kinematics_2d.hpp"
#include "rover_driver/kinematics_3d.hpp"
#include "rover_driver/steering.hpp"
#include "test_geometry.hpp"

using namespace rover_driver;
using rover_driver::test::kExample;
using rover_driver::test::kPi;

namespace {

constexpr PerWheel<double> kStraight{};  // every module currently at 0

PerWheel<ModuleCommand> single(double steer_angle, double wheel_rate) {
  PerWheel<ModuleCommand> cmd{};
  for (auto& m : cmd) {
    m = {steer_angle, wheel_rate, true};
  }
  return cmd;
}

PerWheel<double> all(double angle) { return {angle, angle, angle, angle}; }

}  // namespace

TEST(FitSteerRange, ReversingKeepsWheelsStraight) {
  const auto ik = inverse_2d({-1.0, 0.0, 0.0}, kExample);
  for (const auto& m : ik) {
    EXPECT_NEAR(std::abs(m.steer_angle), kPi, 1e-12);  // raw IK turns every module around
  }
  for (const auto& m : fit_steer_range(ik, kStraight, kExample)) {
    EXPECT_TRUE(m.steer_defined);
    EXPECT_NEAR(m.steer_angle, 0.0, 1e-12);
    EXPECT_NEAR(m.wheel_rate, -1.0 / 0.15, 1e-12);
  }
}

TEST(FitSteerRange, TurnInPlaceFromStraight) {
  // kExample's wheels sit at (+-0.5, +-0.4). Spinning at +1 rad/s, FL moves
  // along (-0.4, 0.5), at atan2(0.5, -0.4) = 128.7 deg: in range, but facing
  // backwards at -51.3 deg is nearer to straight. Each wheel ends 51.3 deg from
  // straight; the left wheels, which move backwards, reversed.
  const BodyTwist2d spin{0.0, 0.0, 1.0};
  const auto fit = fit_steer_range(inverse_2d(spin, kExample), kStraight, kExample);
  const double speed = std::hypot(0.5, 0.4) / 0.15;
  EXPECT_NEAR(fit[kFL].steer_angle, std::atan2(-0.5, 0.4), 1e-12);
  EXPECT_NEAR(fit[kFL].wheel_rate, -speed, 1e-12);
  EXPECT_NEAR(fit[kFR].steer_angle, std::atan2(0.5, 0.4), 1e-12);
  EXPECT_NEAR(fit[kFR].wheel_rate, speed, 1e-12);
  EXPECT_NEAR(fit[kRL].steer_angle, std::atan2(-0.5, -0.4) + kPi, 1e-12);
  EXPECT_NEAR(fit[kRL].wheel_rate, -speed, 1e-12);
  EXPECT_NEAR(fit[kRR].steer_angle, std::atan2(-0.5, 0.4), 1e-12);
  EXPECT_NEAR(fit[kRR].wheel_rate, speed, 1e-12);

  const auto fk = forward_2d(to_measurements(fit), kExample);
  EXPECT_NEAR(fk.twist.vx, 0.0, 1e-12);
  EXPECT_NEAR(fk.twist.vy, 0.0, 1e-12);
  EXPECT_NEAR(fk.twist.wz, 1.0, 1e-12);
}

TEST(FitSteerRange, PicksAngleNearestCurrent) {
  const double limit = kExample.steer_limit;  // 135 deg
  {
    // Straight sideways: +90 forward or -90 reversed, whichever side it is on.
    const auto near_plus = fit_steer_range(single(kPi / 2, 2.0), all(1.4), kExample);
    EXPECT_NEAR(near_plus[kFL].steer_angle, kPi / 2, 1e-12);
    EXPECT_EQ(near_plus[kFL].wheel_rate, 2.0);
    const auto near_minus = fit_steer_range(single(kPi / 2, 2.0), all(-1.4), kExample);
    EXPECT_NEAR(near_minus[kFL].steer_angle, -kPi / 2, 1e-12);
    EXPECT_EQ(near_minus[kFL].wheel_rate, -2.0);
  }
  {
    // 2.0 rad is in range, but from straight 2.0 - pi is nearer.
    const auto from_straight = fit_steer_range(single(2.0, 2.0), kStraight, kExample);
    EXPECT_NEAR(from_straight[kFL].steer_angle, 2.0 - kPi, 1e-12);
    EXPECT_EQ(from_straight[kFL].wheel_rate, -2.0);
    const auto from_side = fit_steer_range(single(2.0, 2.0), all(2.3), kExample);
    EXPECT_EQ(from_side[kFL].steer_angle, 2.0);
    EXPECT_EQ(from_side[kFL].wheel_rate, 2.0);
  }
  {
    // -2.5 rad is beyond the limit: only the reversed angle is left, even
    // right next to the forward one.
    ASSERT_GT(2.5, limit);
    const auto fit = fit_steer_range(single(-2.5, 2.0), all(-limit), kExample);
    EXPECT_NEAR(fit[kFL].steer_angle, -2.5 + kPi, 1e-12);
    EXPECT_EQ(fit[kFL].wheel_rate, -2.0);
  }
}

TEST(FitSteerRange, RangeBeyondHalfTurnUsesFullTurns) {
  RoverGeometry wide = kExample;
  wide.steer_limit = 4.0;
  // -2.5 + 2 pi = 3.78 is in range and nearest 3.5; a full turn is not a reversal.
  const auto fit = fit_steer_range(single(-2.5, 2.0), all(3.5), wide);
  EXPECT_NEAR(fit[kFL].steer_angle, -2.5 + 2 * kPi, 1e-12);
  EXPECT_EQ(fit[kFL].wheel_rate, 2.0);
}

TEST(FitSteerRange, RandomTwistsStayInRangeAndRoundTrip) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  auto u = [&] { return dist(rng); };
  const double limit = kExample.steer_limit;
  for (int trial = 0; trial < 2000; ++trial) {
    const BodyTwist2d twist{u(), u(), 2.0 * u()};
    const PerWheel<double> current{3.0 * u(), 3.0 * u(), 3.0 * u(), 3.0 * u()};
    const auto fit = fit_steer_range(inverse_2d(twist, kExample), current, kExample);
    for (int i = 0; i < kNumWheels; ++i) {
      const double a = fit[i].steer_angle;
      ASSERT_LE(std::abs(a), limit) << "trial " << trial << " wheel " << i;
      // No other in-range angle is nearer the current one.
      for (double other : {a - kPi, a + kPi}) {
        if (std::abs(other) <= limit) {
          ASSERT_LE(std::abs(a - current[i]), std::abs(other - current[i]))
              << "trial " << trial << " wheel " << i;
        }
      }
    }
    const auto fk = forward_2d(to_measurements(fit), kExample);
    ASSERT_NEAR(fk.twist.vx, twist.vx, 1e-12) << "trial " << trial;
    ASSERT_NEAR(fk.twist.vy, twist.vy, 1e-12) << "trial " << trial;
    ASSERT_NEAR(fk.twist.wz, twist.wz, 1e-12) << "trial " << trial;
  }
}

TEST(FitSteerRange, UndefinedModulesHoldCurrentAngle) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto ik = inverse_2d({0.0, 0.0, 0.0}, kExample);
  const auto fit = fit_steer_range(ik, {0.7, -3.0, 3.0, nan}, kExample);
  for (const auto& m : fit) {
    EXPECT_FALSE(m.steer_defined);
    EXPECT_EQ(m.wheel_rate, 0.0);
  }
  EXPECT_EQ(fit[kFL].steer_angle, 0.7);
  EXPECT_EQ(fit[kFR].steer_angle, -kExample.steer_limit);  // clamped
  EXPECT_EQ(fit[kRL].steer_angle, kExample.steer_limit);
  EXPECT_EQ(fit[kRR].steer_angle, 0.0);
}

TEST(FitSteerRange, NonFiniteAnglePassesThrough) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto fit = fit_steer_range(single(nan, 2.0), kStraight, kExample);
  EXPECT_TRUE(std::isnan(fit[kFL].steer_angle));
  EXPECT_EQ(fit[kFL].wheel_rate, 2.0);
}

TEST(FitSteerRange, Reversed3dCommandsRoundTripForward3d) {
  struct Case {
    BodyTwist2d twist;
    Gyro gyro;  // gyro.wz must equal twist.wz for a consistent round trip
    SuspensionState s;
  };
  const Case cases[] = {
      {{0.4, 0.1, 0.3}, {0.05, -0.08, 0.3}, {0.1, -0.1, 0.2, -0.2}},
      {{-1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, -0.5, 0.0}},
      {{-0.3, 0.6, -0.4}, {-0.1, 0.1, -0.4}, {-0.15, 0.15, 0.1, -0.1}},
      {{0.0, 0.0, 0.8}, {0.1, 0.0, 0.8}, {0.2, -0.2, 0.3, -0.3}},
  };
  int reversals = 0;
  for (const auto& c : cases) {
    SCOPED_TRACE(testing::Message() << "case " << (&c - cases));
    const auto ik = inverse_3d(c.twist, c.gyro, c.s, kExample);
    const auto fit = fit_steer_range(ik, kStraight, kExample);
    for (int i = 0; i < kNumWheels; ++i) {
      ASSERT_TRUE(ik[i].steer_defined);
      EXPECT_LE(std::abs(fit[i].steer_angle), kPi / 2 + 1e-12) << "wheel " << i;
      if (fit[i].wheel_rate * ik[i].wheel_rate < 0.0) {
        ++reversals;
        EXPECT_EQ(fit[i].contact_angle, -ik[i].contact_angle) << "wheel " << i;
      } else {
        EXPECT_EQ(fit[i].contact_angle, ik[i].contact_angle) << "wheel " << i;
      }
    }
    // Tolerances as in Forward3d.RoundTripsInverse3d.
    const auto fk = forward_3d(to_measurements(fit), c.gyro, c.s, kExample);
    EXPECT_TRUE(fk.converged);
    EXPECT_NEAR(fk.v_body.x(), c.twist.vx, 1e-5);
    EXPECT_NEAR(fk.v_body.y(), c.twist.vy, 1e-5);
    EXPECT_NEAR(fk.v_body.z(), 0.0, 1e-5);
    for (int i = 0; i < kNumWheels; ++i) {
      EXPECT_NEAR(fk.contact_angle[i], fit[i].contact_angle, 1e-4) << "wheel " << i;
    }
    EXPECT_LT(fk.residual_rms, 1e-5);
  }
  EXPECT_GE(reversals, 6);  // reversing and spinning cases reverse several modules
}
