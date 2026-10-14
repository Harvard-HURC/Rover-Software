// The rover's skid-steer kinematics (tank_kinematics.hpp): the geometry read from a rover.yaml, the inverse and forward
// kinematics, the saturation that keeps the turn radius; rover_description's own rover.yaml loads.
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "rover_control/tank_kinematics.hpp"

namespace {

using rover_control::LoadRoverYaml;
using rover_control::TankGeometry;
using rover_control::Twist;
using rover_control::WheelSpeeds;

/// A file in the test's temporary folder holding `body`; its path.
std::string Write(const std::string& name, const std::string& body) {
  const std::string path = ::testing::TempDir() + name;
  std::ofstream(path) << body;
  return path;
}

TankGeometry Geometry() {
  TankGeometry g;
  g.wheel_radius = 0.15;
  g.track = 0.8;
  g.wheelbase = 0.9;
  g.top_speed = 20.0;
  return g;
}

TEST(LoadRoverYaml, ReadsTheGeometry) {
  const TankGeometry g = LoadRoverYaml(Write("rover_a.yaml",
      "rocker: {pivot_y: 0.55, pivot_z: 0.3}\nwheel: {radius: 0.2, width: 0.1, dx: 0.6, dz: -0.1, top_speed: 15.0}\n"));
  EXPECT_DOUBLE_EQ(g.wheel_radius, 0.2);
  EXPECT_DOUBLE_EQ(g.track, 1.1);
  EXPECT_DOUBLE_EQ(g.wheelbase, 1.2);
  EXPECT_DOUBLE_EQ(g.top_speed, 15.0);
  EXPECT_DOUBLE_EQ(g.track_multiplier, 1.0);
}

TEST(LoadRoverYaml, LoadsTheDescriptionsRoverYaml) {
  const TankGeometry g = LoadRoverYaml(ROVER_YAML);
  EXPECT_NO_THROW(rover_control::Validate(g));
  EXPECT_GT(g.track, 2 * g.wheel_radius);
}

TEST(LoadRoverYaml, NamesWhatIsMissingOrWrong) {
  const std::string path = Write("rover_b.yaml", "wheel: {radius: 0.2, dx: 0.6, top_speed: 15.0}\n");
  try {
    LoadRoverYaml(path);
    FAIL() << "no exception";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("rocker.pivot_y"), std::string::npos) << e.what();
    EXPECT_NE(std::string(e.what()).find(path), std::string::npos) << e.what();
  }
  EXPECT_THROW(LoadRoverYaml(::testing::TempDir() + "no_such_rover.yaml"), std::runtime_error);
  const std::string negative =
      Write("rover_c.yaml", "rocker: {pivot_y: 0.5}\nwheel: {radius: -0.2, dx: 0.6, top_speed: 15.0}\n");
  EXPECT_THROW(LoadRoverYaml(negative), std::invalid_argument);
}

TEST(TankKinematics, StraightAndInPlace) {
  const TankGeometry g = Geometry();
  const WheelSpeeds straight = rover_control::ToWheels(g, {0.6, 0.0});
  EXPECT_DOUBLE_EQ(straight.left, 4.0);
  EXPECT_DOUBLE_EQ(straight.right, 4.0);
  const WheelSpeeds spin = rover_control::ToWheels(g, {0.0, 1.0});
  EXPECT_DOUBLE_EQ(spin.left, -0.4 / 0.15);
  EXPECT_DOUBLE_EQ(spin.right, 0.4 / 0.15);
}

TEST(TankKinematics, ForwardUndoesInverse) {
  const TankGeometry g = Geometry();
  for (const Twist t : {Twist{0.3, 0.2}, Twist{-0.5, 1.1}, Twist{1.2, -0.7}}) {
    const Twist back = rover_control::ToTwist(g, rover_control::ToWheels(g, t));
    EXPECT_NEAR(back.vx, t.vx, 1e-12);
    EXPECT_NEAR(back.wz, t.wz, 1e-12);
  }
}

TEST(TankKinematics, TheTrackMultiplierWidensTheTrack) {
  TankGeometry g = Geometry();
  g.track_multiplier = 1.5;
  EXPECT_DOUBLE_EQ(rover_control::ToWheels(g, {0.0, 1.0}).right, 0.6 / 0.15);
  EXPECT_NEAR(rover_control::ToTwist(g, rover_control::ToWheels(g, {0.2, 0.5})).wz, 0.5, 1e-12);
}

TEST(TankKinematics, SaturationKeepsTheTurnRadius) {
  const TankGeometry g = Geometry();
  const WheelSpeeds fast{10.0, 40.0};
  const WheelSpeeds limited = rover_control::Saturate(g, fast);
  EXPECT_DOUBLE_EQ(limited.right, 20.0);
  EXPECT_DOUBLE_EQ(limited.left, 5.0);
  const Twist a = rover_control::ToTwist(g, fast), b = rover_control::ToTwist(g, limited);
  EXPECT_NEAR(a.vx / a.wz, b.vx / b.wz, 1e-12);
  const WheelSpeeds slow{3.0, -2.0};
  EXPECT_DOUBLE_EQ(rover_control::Saturate(g, slow).left, 3.0);
  EXPECT_DOUBLE_EQ(rover_control::Saturate(g, slow).right, -2.0);
}

TEST(TankKinematics, ValidateNamesTheBadNumber) {
  TankGeometry g = Geometry();
  g.track = 0.0;
  try {
    rover_control::Validate(g);
    FAIL() << "no exception";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("track"), std::string::npos) << e.what();
  }
}

}  // namespace
