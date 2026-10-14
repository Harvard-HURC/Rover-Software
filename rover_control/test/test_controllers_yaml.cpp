// rover_control's config/controllers.yaml: the controllers it names, and the drive controller's geometry and limits,
// which must be rover_description's rover.yaml's (the file writes them out: a YAML file cannot include another).
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "rover_control/tank_kinematics.hpp"

namespace {

const YAML::Node& Controllers() {
  static const YAML::Node root = YAML::LoadFile(CONTROLLERS_YAML);
  return root;
}

YAML::Node Drive() { return Controllers()["drive_controller"]["ros__parameters"]; }

double Number(const char* key) { return Drive()[key].as<double>(); }

TEST(ControllersYaml, TheControllerManagerRunsTheBroadcasterAndTheDriveController) {
  const YAML::Node manager = Controllers()["controller_manager"]["ros__parameters"];
  EXPECT_EQ(manager["joint_state_broadcaster"]["type"].as<std::string>(),
            "joint_state_broadcaster/JointStateBroadcaster");
  EXPECT_EQ(manager["drive_controller"]["type"].as<std::string>(), "diff_drive_controller/DiffDriveController");
  EXPECT_EQ(manager["update_rate"].as<int>(), 100);
}

TEST(ControllersYaml, LeftIsTheLeftWheelsRightTheRight) {
  EXPECT_EQ(Drive()["left_wheel_names"].as<std::vector<std::string>>(),
            (std::vector<std::string>{"wheel_fl_joint", "wheel_rl_joint"}));
  EXPECT_EQ(Drive()["right_wheel_names"].as<std::vector<std::string>>(),
            (std::vector<std::string>{"wheel_fr_joint", "wheel_rr_joint"}));
}

TEST(ControllersYaml, TheGeometryAndTheSpeedLimitsAreRoverYamls) {
  const rover_control::TankGeometry g = rover_control::LoadRoverYaml(ROVER_YAML);
  EXPECT_DOUBLE_EQ(Number("wheel_separation"), g.track);
  EXPECT_DOUBLE_EQ(Number("wheel_radius"), g.wheel_radius);
  EXPECT_DOUBLE_EQ(Number("wheel_separation_multiplier"), 1.0);
  const double top = g.top_speed * g.wheel_radius;  // [m/s] a wheel at its top speed
  EXPECT_NEAR(Number("linear.x.max_velocity"), top, 1e-9);
  EXPECT_NEAR(Number("linear.x.min_velocity"), -top, 1e-9);
  EXPECT_NEAR(Number("angular.z.max_velocity"), top / (g.track / 2), 1e-9);
  EXPECT_NEAR(Number("angular.z.min_velocity"), -top / (g.track / 2), 1e-9);
}

TEST(ControllersYaml, TheRampTurnsAWheelAtOneRate) {
  // A wheel's acceleration driving straight and turning in place is the same (8 rad/s^2: rover_sim's wheel_cmd accel,
  // which rover_sim's test_wheel_cmd checks).
  const rover_control::TankGeometry g = rover_control::LoadRoverYaml(ROVER_YAML);
  EXPECT_NEAR(Number("linear.x.max_acceleration") / g.wheel_radius,
              Number("angular.z.max_acceleration") * (g.track / 2) / g.wheel_radius, 1e-9);
  EXPECT_DOUBLE_EQ(Number("linear.x.max_deceleration"), -Number("linear.x.max_acceleration"));
  EXPECT_DOUBLE_EQ(Number("angular.z.max_deceleration"), -Number("angular.z.max_acceleration"));
}

TEST(ControllersYaml, ACommandTimesOutAfterHalfASecondAndOdometryIsOdomToBaseLink) {
  EXPECT_DOUBLE_EQ(Number("cmd_vel_timeout"), 0.5);
  EXPECT_EQ(Drive()["odom_frame_id"].as<std::string>(), "odom");
  EXPECT_EQ(Drive()["base_frame_id"].as<std::string>(), "base_link");
  EXPECT_TRUE(Drive()["enable_odom_tf"].as<bool>());
}

}  // namespace
