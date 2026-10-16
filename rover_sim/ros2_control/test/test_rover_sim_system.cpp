// RoverSimSystem (rover_sim_system.cpp) without Gazebo running: an ECM with the rover's eight joints, the hardware
// info of rover_description's ros2_control block with the simulation's plugin, and the interfaces the controller
// manager would claim (initSim, then init and export, as gz_ros2_control's resource manager calls them).
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_components.hh"
#include "rover_sim_system.hpp"

namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace components = rover_sim::components;

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

/// rover_description's ros2_control block (urdf/ros2_control.xacro) with the simulation's plugin.
hardware_interface::HardwareInfo RoverInfo() {
  hardware_interface::HardwareInfo info{};
  info.name = "rover";
  info.type = "system";
  info.hardware_plugin_name = "rover_sim/RoverSimSystem";
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

class RoverSimSystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    for (const auto& name : kWheels) joints_[name] = Joint(name);
    for (const auto& name : kSensed) joints_[name] = Joint(name);
  }

  gz::sim::Entity Joint(const std::string& name) {
    const auto entity = ecm_.CreateEntity();
    ecm_.CreateComponent(entity, gz::sim::components::Joint());
    ecm_.CreateComponent(entity, gz::sim::components::Name(name));
    return entity;
  }

  /// What gz_ros2_control's resource manager does: initSim, then init and the interfaces' export.
  void Load() {
    rclcpp::Node::SharedPtr node;  // RoverSimSystem keeps it and never uses it
    ASSERT_TRUE(system_.initSim(node, joints_, RoverInfo(), ecm_, 100));
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = RoverInfo();
    params.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    params.logger = rclcpp::get_logger("test_rover_sim_system");
    ASSERT_EQ(system_.init(params), CallbackReturn::SUCCESS);
    for (const auto& state : system_.on_export_state_interfaces()) states_[state->get_name()] = state;
    for (const auto& command : system_.on_export_command_interfaces()) commands_[command->get_name()] = command;
  }

  double State(const std::string& name) { return states_.at(name)->get_optional<double>().value(); }

  void Command(const std::string& name, double value) { ASSERT_TRUE(commands_.at(name)->set_value(value, true)); }

  const components::Setpoint& Setpoint(const std::string& joint) {
    return ecm_.Component<components::WheelSetpoint>(joints_.at(joint))->Data();
  }

  gz::sim::EntityComponentManager ecm_;
  std::map<std::string, gz::sim::Entity> joints_;
  rover_sim::RoverSimSystem system_;
  std::map<std::string, hardware_interface::StateInterface::ConstSharedPtr> states_;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> commands_;
};

TEST_F(RoverSimSystemTest, ExportsTheBlocksInterfaces) {
  Load();
  EXPECT_EQ(states_.size(), 4u * 3u + 4u * 2u);
  EXPECT_EQ(commands_.size(), 4u);
  for (const auto& name : kWheels) {
    EXPECT_EQ(commands_.count(name + "/velocity"), 1u) << name;
    EXPECT_EQ(states_.count(name + "/effort"), 1u) << name;
  }
  for (const auto& name : kSensed) EXPECT_EQ(states_.count(name + "/position"), 1u) << name;
}

TEST_F(RoverSimSystemTest, MarksTheWheelsAsRos2ControlsAndGivesEveryJointItsState) {
  Load();
  for (const auto& name : kWheels) {
    EXPECT_TRUE(Setpoint(name).ros2_control) << name;
    EXPECT_EQ(Setpoint(name).velocity, 0.0) << name;
  }
  for (const auto& [name, entity] : joints_) {
    EXPECT_NE(ecm_.Component<gz::sim::components::JointPosition>(entity), nullptr) << name;
    EXPECT_NE(ecm_.Component<gz::sim::components::JointVelocity>(entity), nullptr) << name;
  }
  for (const auto& name : kSensed) {
    EXPECT_EQ(ecm_.Component<components::WheelSetpoint>(joints_.at(name)), nullptr) << name;
  }
}

TEST_F(RoverSimSystemTest, WriteHandsEachWheelItsCommandStampedWithTheTimeItIsGiven) {
  Load();
  Command("wheel_fl_joint/velocity", 2.5);
  Command("wheel_rl_joint/velocity", 2.5);
  Command("wheel_fr_joint/velocity", -1.25);
  // wheel_rr_joint's command is not set (NaN, as before a controller writes one): zero.
  const rclcpp::Time step(12, 345000000, RCL_ROS_TIME);
  ASSERT_EQ(system_.write(step, rclcpp::Duration::from_seconds(0.001)), return_type::OK);
  EXPECT_EQ(Setpoint("wheel_fl_joint").velocity, 2.5);
  EXPECT_EQ(Setpoint("wheel_rl_joint").velocity, 2.5);
  EXPECT_EQ(Setpoint("wheel_fr_joint").velocity, -1.25);
  EXPECT_EQ(Setpoint("wheel_rr_joint").velocity, 0.0);
  for (const auto& name : kWheels) {
    EXPECT_EQ(Setpoint(name).time, std::chrono::nanoseconds(12345000000)) << name;
    EXPECT_TRUE(Setpoint(name).ros2_control) << name;
  }
}

TEST_F(RoverSimSystemTest, ReadReportsTheJointsAndTheMotorCurrent) {
  Load();
  // Before the drivetrain's first step there is no current: effort 0.
  ASSERT_EQ(system_.read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  EXPECT_EQ(State("wheel_fl_joint/effort"), 0.0);
  ecm_.SetComponentData<gz::sim::components::JointPosition>(joints_.at("wheel_fl_joint"), {12.5});
  ecm_.SetComponentData<gz::sim::components::JointVelocity>(joints_.at("wheel_fl_joint"), {3.0});
  ecm_.CreateComponent(joints_.at("wheel_fl_joint"), components::WheelMotorCurrent(7.25));
  ecm_.SetComponentData<gz::sim::components::JointPosition>(joints_.at("rocker_left_joint"), {-0.125});
  ecm_.SetComponentData<gz::sim::components::JointVelocity>(joints_.at("camera_pan_joint"), {0.5});
  ASSERT_EQ(system_.read(rclcpp::Time(0, 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
            return_type::OK);
  EXPECT_EQ(State("wheel_fl_joint/position"), 12.5);
  EXPECT_EQ(State("wheel_fl_joint/velocity"), 3.0);
  EXPECT_EQ(State("wheel_fl_joint/effort"), 7.25);
  EXPECT_EQ(State("rocker_left_joint/position"), -0.125);
  EXPECT_EQ(State("camera_pan_joint/velocity"), 0.5);
  EXPECT_EQ(State("wheel_rr_joint/position"), 0.0);  // a joint component without a value yet
}

TEST_F(RoverSimSystemTest, InitSimRefusesAJointTheModelLacks) {
  joints_.erase("wheel_rr_joint");
  rclcpp::Node::SharedPtr node;
  EXPECT_FALSE(system_.initSim(node, joints_, RoverInfo(), ecm_, 100));
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
