#include "rover_control/rover_hardware_stub.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rover_control {

namespace {
constexpr int64_t kLogPeriod = 1000000000;  // [ns] between two logs of the commands it would send
}  // namespace

hardware_interface::CallbackReturn RoverHardwareStub::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  states_.clear();
  commanded_.clear();
  for (const auto& joint : info_.joints) {
    for (const auto& state : joint.state_interfaces) states_.push_back(joint.name + "/" + state.name);
    for (const auto& command : joint.command_interfaces) {
      if (command.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(get_logger(), "joint '%s': RoverHardwareStub takes velocity commands only, not '%s'",
                     joint.name.c_str(), command.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      commanded_.push_back(joint.name);
    }
  }
  RCLCPP_WARN(get_logger(),
              "No motors are connected: RoverHardwareStub reports zero motion and drives nothing (the motor "
              "controllers are not chosen yet, rover_control/README.md); %zu joints, %zu wheels",
              info_.joints.size(), commanded_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverHardwareStub::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_WARN(get_logger(), "Active, but no motors are connected: the controllers run and no wheel turns");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverHardwareStub::read(const rclcpp::Time& /*time*/,
                                                        const rclcpp::Duration& /*period*/) {
  for (const auto& name : states_) set_state(name, 0.0);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverHardwareStub::write(const rclcpp::Time& time,
                                                         const rclcpp::Duration& /*period*/) {
  std::vector<double> commands;
  commands.reserve(commanded_.size());
  for (const auto& joint : commanded_) {
    commands.push_back(get_command<double>(joint + "/" + hardware_interface::HW_IF_VELOCITY));
  }
  const std::string line = WouldSend(commanded_, commands);
  if (!line.empty() && (!logged_once_ || time.nanoseconds() - logged_ >= kLogPeriod)) {
    RCLCPP_WARN(get_logger(), "No motors are connected: %s", line.c_str());
    logged_ = time.nanoseconds();
    logged_once_ = true;
  }
  return hardware_interface::return_type::OK;
}

std::string RoverHardwareStub::WouldSend(const std::vector<std::string>& joints, const std::vector<double>& commands) {
  bool moving = false;
  std::ostringstream line;
  line << std::fixed << std::setprecision(2) << "would send ";
  for (size_t k = 0; k < joints.size() && k < commands.size(); ++k) {
    const double value = std::isfinite(commands[k]) ? commands[k] : 0.0;
    moving = moving || value != 0.0;
    line << (k ? ", " : "") << joints[k] << ' ' << value;
  }
  line << " rad/s";
  return moving ? line.str() : std::string();
}

}  // namespace rover_control

PLUGINLIB_EXPORT_CLASS(rover_control::RoverHardwareStub, hardware_interface::SystemInterface)
