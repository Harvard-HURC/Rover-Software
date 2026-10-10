// RoverSimSystem: the simulated rover's ros2_control hardware (design spec 2026-10-09, section 6.1). gz_ros2_control
// loads it inside Gazebo (pluginlib: rover_sim/RoverSimSystem, rover_sim_system.xml) for the ros2_control block of
// rover_sim.urdf.xacro with ros2_control:=true; the controller manager it runs calls write() at the start of every
// physics step (gz_ros2_control's PreUpdate) and read() and the controllers' update at the end of every 10th step
// (100 Hz, its PostUpdate).
// - write(): each wheel's velocity command [rad/s] into its joint's WheelSetpoint (plugins/rover_components.hh),
//   stamped with the time write() is given: ros2_control 4.47 passes the controller manager's clock, sim time from
//   the bridged /clock, a few ms behind the step (the drivetrain's watchdog counts from it, so the motors stop when
//   /clock stops). RoverDrivetrain's motor model follows the setpoint in the same step (its plugin comes after
//   gz_ros2_control's in the model; tests/test_ros2_control.py checks it). Not Gazebo's JointVelocityCmd: the
//   Physics system would enforce that as a velocity constraint and bypass the motors. A command not yet set (NaN,
//   until a controller writes one) is 0.
// - read(): every joint's position and velocity from Gazebo, and each wheel's motor current [A] as its effort
//   (RoverDrivetrain's WheelMotorCurrent; 0 until the drivetrain's first step).
// The interfaces are the block's (rover_description/urdf/ros2_control.xacro): wheels velocity command and position,
// velocity and effort state; rockers and camera head position and velocity.
#pragma once

#include <map>
#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>

namespace rover_sim {

class RoverSimSystem : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  using hardware_interface::SystemInterface::on_init;

  bool initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& joints,
               const hardware_interface::HardwareInfo& hardware_info, gz::sim::EntityComponentManager& ecm,
               unsigned int update_rate) override;
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  struct Joint {
    std::string name;
    gz::sim::Entity entity = gz::sim::kNullEntity;
    bool driven = false;  // a velocity command interface: a wheel
    std::string position, velocity, effort, command;  // the interfaces' full names
  };
  std::vector<Joint> joints_;
  gz::sim::EntityComponentManager* ecm_ = nullptr;
};

}  // namespace rover_sim
