// RoverHardwareStub (rover_hardware_stub.cpp) without a controller manager: the interfaces of rover_description's
// ros2_control block, zero motion whatever it is commanded, and the line it logs for the commands it would send.
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rover_control/rover_hardware_stub.hpp"

namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using rover_control::RoverHardwareStub;

const std::vector<std::string> kWheels = {"wheel_fl_joint", "wheel_rl_joint", "wheel_fr_joint", "wheel_rr_joint"};
const std::vector<std::string> kSensed = {"rocker_left_joint", "rocker_right_joint", "camera_pan_joint",
                                          "camera_tilt_joint"};

hardware_interface::InterfaceInfo Interface(const std::string& name) {
  hardware_interface::InterfaceInfo info{};
  info.name = name;
  info.size = 1;
  info.enable_limits = false;
  return info;
}

/// rover_description's ros2_control block (urdf/ros2_control.xacro), as rover.launch.py loads it.
hardware_interface::HardwareInfo RoverInfo() {
  hardware_interface::HardwareInfo info{};
  info.name = "rover";
  info.type = "system";
  info.hardware_plugin_name = "rover_control/RoverHardwareStub";
  for (const auto& name : kWheels) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.command_interfaces = {Interface("velocity")};
    joint.state_interfaces = {Interface("position"), Interface("velocity"), Interface("effort")};
    info.joints.push_back(joint);
  }
  for (const auto& name : kSensed) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.state_interfaces = {Interface("position"), Interface("velocity")};
    info.joints.push_back(joint);
  }
  return info;
}

class RoverHardwareStubTest : public ::testing::Test {
 protected:
  void SetUp() override {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = RoverInfo();
    params.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    params.logger = rclcpp::get_logger("test_rover_hardware_stub");
    ASSERT_EQ(stub_.init(params), CallbackReturn::SUCCESS);
    for (const auto& state : stub_.on_export_state_interfaces()) states_[state->get_name()] = state;
    for (const auto& command : stub_.on_export_command_interfaces()) commands_[command->get_name()] = command;
  }

  RoverHardwareStub stub_;
  std::map<std::string, hardware_interface::StateInterface::ConstSharedPtr> states_;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> commands_;
};

TEST_F(RoverHardwareStubTest, ExportsTheBlocksInterfaces) {
  EXPECT_EQ(states_.size(), 4u * 3u + 4u * 2u);
  EXPECT_EQ(commands_.size(), 4u);
  for (const auto& name : kWheels) EXPECT_EQ(commands_.count(name + "/velocity"), 1u) << name;
}

TEST_F(RoverHardwareStubTest, ReportsZeroMotionWhateverItIsCommanded) {
  for (const auto& [name, command] : commands_) ASSERT_TRUE(command->set_value(3.0, true)) << name;
  ASSERT_EQ(stub_.write(rclcpp::Time(1, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  ASSERT_EQ(stub_.read(rclcpp::Time(1, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  for (const auto& [name, state] : states_) EXPECT_EQ(state->get_optional<double>().value(), 0.0) << name;
}

TEST_F(RoverHardwareStubTest, Activates) {
  EXPECT_EQ(stub_.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(WouldSend, NamesEachWheelsCommandOrNothingWhenAllAreZero) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(RoverHardwareStub::WouldSend({"wheel_fl_joint", "wheel_fr_joint"}, {2.0, -1.5}),
            "would send wheel_fl_joint 2.00, wheel_fr_joint -1.50 rad/s");
  EXPECT_EQ(RoverHardwareStub::WouldSend({"wheel_fl_joint", "wheel_fr_joint"}, {0.0, nan}), "");
  EXPECT_EQ(RoverHardwareStub::WouldSend({"wheel_fl_joint", "wheel_fr_joint"}, {nan, 0.5}),
            "would send wheel_fl_joint 0.00, wheel_fr_joint 0.50 rad/s");
}

}  // namespace
