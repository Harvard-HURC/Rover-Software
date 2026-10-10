// RoverHardwareStub: the real rover's ros2_control hardware until its motor controllers are chosen (design spec
// 2026-10-09, section 6). It exports the interfaces of rover_description's ros2_control block (wheels: velocity
// command, position, velocity and effort state; rockers and camera head: position and velocity), reports zero motion,
// and drives nothing: it says so when it starts and when it is activated, and logs the wheel commands it would send,
// at most once a second. rover.launch.py runs it (rover_description's rover.urdf.xacro names it by default). It
// activates, so that the controllers start on the rover's computer (the spec had it refuse; spec section 12).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace rover_control {

class RoverHardwareStub : public hardware_interface::SystemInterface {
 public:
  using hardware_interface::SystemInterface::on_init;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// "would send wheel_fl_joint 1.00, ... rad/s" for the joints' commands (one not set yet, NaN, as 0), or "" when
  /// every one is 0.
  static std::string WouldSend(const std::vector<std::string>& joints, const std::vector<double>& commands);

 private:
  std::vector<std::string> states_;     // every state interface's full name
  std::vector<std::string> commanded_;  // the joints with a velocity command
  int64_t logged_ = 0;                  // [ns] when write() last logged, on the controller manager's clock
  bool logged_once_ = false;
};

}  // namespace rover_control
