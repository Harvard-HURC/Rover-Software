#include "rover_control/rover_hardware_stub.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rover_control {

namespace {
constexpr int64_t kLogPeriod = 1000000000;  // [ns] between two logs of the commands it would send
constexpr double kShown = 0.005;            // [rad/s] a command below this shows as 0.00, and counts as 0

// A command as the log shows it: 0 when it is not set yet (NaN) or rounds to 0.00 (the drive controller's ramp leaves
// residues like -1e-17, which would print as -0.00).
double Shown(double command) { return std::isfinite(command) && std::abs(command) >= kShown ? command : 0.0; }
}  // namespace

hardware_interface::CallbackReturn RoverHardwareStub::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  states_.clear();
  commanded_.clear();
  command_interfaces_.clear();
  for (const auto& joint : info_.joints) {
    for (const auto& state : joint.state_interfaces) states_.push_back(joint.name + "/" + state.name);
    for (const auto& command : joint.command_interfaces) {
      if (command.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(get_logger(), "joint '%s': RoverHardwareStub takes velocity commands only, not '%s'",
                     joint.name.c_str(), command.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      commanded_.push_back(joint.name);
      command_interfaces_.push_back(joint.name + "/" + command.name);
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
  commands_.clear();
  bool moving = false;
  for (const auto& name : command_interfaces_) {
    commands_.push_back(get_command<double>(name));
    moving = moving || Shown(commands_.back()) != 0.0;
  }
  // The commands at once when the wheels start, then at most once a second while they turn, and one line when they
  // are back at zero, so that the log's last line is what the wheels were last told. The line is only formatted when
  // it is logged: write() runs at the controller manager's rate.
  if (moving && (!moving_ || time.nanoseconds() - logged_ >= kLogPeriod)) {
    RCLCPP_WARN(get_logger(), "No motors are connected: %s", WouldSend(commanded_, commands_).c_str());
    logged_ = time.nanoseconds();
  } else if (!moving && moving_) {
    RCLCPP_WARN(get_logger(), "No motors are connected: would send 0 to every wheel");
    logged_ = time.nanoseconds();
  }
  moving_ = moving;
  return hardware_interface::return_type::OK;
}

std::string RoverHardwareStub::WouldSend(const std::vector<std::string>& joints, const std::vector<double>& commands) {
  bool moving = false;
  std::ostringstream line;
  line << std::fixed << std::setprecision(2) << "would send ";
  for (size_t k = 0; k < joints.size() && k < commands.size(); ++k) {
    const double value = Shown(commands[k]);
    moving = moving || value != 0.0;
    line << (k ? ", " : "") << joints[k] << ' ' << value;
  }
  line << " rad/s";
  return moving ? line.str() : std::string();
}

}  // namespace rover_control

PLUGINLIB_EXPORT_CLASS(rover_control::RoverHardwareStub, hardware_interface::SystemInterface)
