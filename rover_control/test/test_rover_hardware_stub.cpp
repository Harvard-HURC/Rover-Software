// RoverHardwareStub (rover_hardware_stub.cpp) without a controller manager: the interfaces of rover_description's
// ros2_control block, zero motion whatever it is commanded, and the lines it logs for the commands it would send.
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rcutils/logging.h>

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
  // The drive controller's ramp leaves residues such as -1e-17: 0.00, not -0.00, and alone they are nothing.
  EXPECT_EQ(RoverHardwareStub::WouldSend({"wheel_fl_joint", "wheel_fr_joint"}, {-1e-17, 0.16}),
            "would send wheel_fl_joint 0.00, wheel_fr_joint 0.16 rad/s");
  EXPECT_EQ(RoverHardwareStub::WouldSend({"wheel_fl_joint", "wheel_fr_joint"}, {-0.001, 0.004}), "");
}

std::vector<std::string>* g_logged = nullptr;  // where Capture puts the lines logged meanwhile

void Capture(const rcutils_log_location_t* /*location*/, int /*severity*/, const char* /*name*/,
             rcutils_time_point_value_t /*timestamp*/, const char* format, va_list* args) {
  if (!g_logged) return;
  va_list copy;
  va_copy(copy, *args);
  char line[1024];
  std::vsnprintf(line, sizeof line, format, copy);
  va_end(copy);
  g_logged->push_back(line);
}

TEST_F(RoverHardwareStubTest, LogsTheCommandsAtOnceThenEverySecondAndOnceBackAtZero) {
  std::vector<std::string> logged;
  g_logged = &logged;
  const auto saved = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(Capture);
  const auto write = [&](double seconds, double command) {
    for (const auto& name : kWheels) ASSERT_TRUE(commands_[name + "/velocity"]->set_value(command, true)) << name;
    ASSERT_EQ(stub_.write(rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME),
                          rclcpp::Duration::from_seconds(0.01)),
              return_type::OK);
  };
  write(1.00, 0.0);     // standing: nothing
  write(1.01, 2.0);     // the wheels start: at once
  write(1.50, 2.5);     // within a second of the last line: nothing
  write(2.02, 3.0);     // a second later
  write(2.03, -1e-17);  // the ramp's residue: back at zero, once
  write(2.04, 0.0);
  write(2.05, 1.0);     // they start again: at once
  rcutils_logging_set_output_handler(saved);
  g_logged = nullptr;
  const auto send = [](const std::string& value) {
    return "No motors are connected: would send wheel_fl_joint " + value + ", wheel_rl_joint " + value +
           ", wheel_fr_joint " + value + ", wheel_rr_joint " + value + " rad/s";
  };
  EXPECT_EQ(logged, (std::vector<std::string>{send("2.00"), send("3.00"),
                                              "No motors are connected: would send 0 to every wheel", send("1.00")}));
}

}  // namespace
