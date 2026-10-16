// rover_components.hh first: its component registration names gz::sim as `sim`, which gz_system_interface.hpp also
// declares as an alias at global scope.
#include "rover_components.hh"
#include "rover_sim_system.hpp"

#include <chrono>
#include <cmath>

#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rover_sim {

namespace {

rclcpp::Logger Logger() { return rclcpp::get_logger("RoverSimSystem"); }

double First(const std::vector<double>* values) { return values && !values->empty() ? (*values)[0] : 0.0; }

}  // namespace

bool RoverSimSystem::initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& joints,
                             const hardware_interface::HardwareInfo& hardware_info,
                             gz::sim::EntityComponentManager& ecm, unsigned int /*update_rate*/) {
  nh_ = model_nh;
  ecm_ = &ecm;
  joints_.clear();
  for (const auto& info : hardware_info.joints) {
    const auto found = joints.find(info.name);
    if (found == joints.end()) {
      RCLCPP_FATAL(Logger(), "joint '%s' of the ros2_control block is not a joint of the Gazebo model",
                   info.name.c_str());
      return false;
    }
    Joint joint;
    joint.name = info.name;
    joint.entity = found->second;
    joint.position = info.name + "/" + hardware_interface::HW_IF_POSITION;
    joint.velocity = info.name + "/" + hardware_interface::HW_IF_VELOCITY;
    joint.effort = info.name + "/" + hardware_interface::HW_IF_EFFORT;
    joint.command = info.name + "/" + hardware_interface::HW_IF_VELOCITY;
    for (const auto& command : info.command_interfaces) {
      if (command.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(Logger(), "joint '%s': RoverSimSystem takes velocity commands only, not '%s'",
                     info.name.c_str(), command.name.c_str());
        return false;
      }
      joint.driven = true;
    }
    if (!ecm.Component<gz::sim::components::JointPosition>(joint.entity)) {
      ecm.CreateComponent(joint.entity, gz::sim::components::JointPosition());
    }
    if (!ecm.Component<gz::sim::components::JointVelocity>(joint.entity)) {
      ecm.CreateComponent(joint.entity, gz::sim::components::JointVelocity());
    }
    if (joint.driven) {
      // The wheel is ros2_control's from now on (RoverDrivetrain ignores its setpoint topic); 0 until write().
      components::Setpoint setpoint;
      setpoint.ros2_control = true;
      ecm.CreateComponent(joint.entity, components::WheelSetpoint(setpoint));
    }
    joints_.push_back(joint);
  }
  return true;
}

hardware_interface::CallbackReturn RoverSimSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != joints_.size()) {
    RCLCPP_FATAL(get_logger(), "%zu joints in the ros2_control block, %zu found by initSim", info_.joints.size(),
                 joints_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  size_t wheels = 0;
  for (const auto& joint : joints_) wheels += joint.driven ? 1 : 0;
  RCLCPP_INFO(get_logger(), "RoverSimSystem: %zu joints, %zu wheels driven through RoverDrivetrain's motor model",
              joints_.size(), wheels);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverSimSystem::read(const rclcpp::Time& /*time*/,
                                                     const rclcpp::Duration& /*period*/) {
  for (const auto& joint : joints_) {
    const auto* position = ecm_->Component<gz::sim::components::JointPosition>(joint.entity);
    const auto* velocity = ecm_->Component<gz::sim::components::JointVelocity>(joint.entity);
    set_state(joint.position, First(position ? &position->Data() : nullptr));
    set_state(joint.velocity, First(velocity ? &velocity->Data() : nullptr));
    if (joint.driven) {
      const auto* current = ecm_->Component<components::WheelMotorCurrent>(joint.entity);
      set_state(joint.effort, current ? current->Data() : 0.0);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverSimSystem::write(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  const auto stamp = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::nanoseconds(time.nanoseconds()));
  for (const auto& joint : joints_) {
    if (!joint.driven) continue;
    auto* setpoint = ecm_->Component<components::WheelSetpoint>(joint.entity);
    if (!setpoint) continue;  // created by initSim; a world reset may have removed it
    const double command = get_command<double>(joint.command);
    setpoint->Data() = components::Setpoint{std::isfinite(command) ? command : 0.0, stamp, true};
  }
  return hardware_interface::return_type::OK;
}

}  // namespace rover_sim

PLUGINLIB_EXPORT_CLASS(rover_sim::RoverSimSystem, gz_ros2_control::GazeboSimSystemInterface)
