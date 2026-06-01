#include "aristo_hardware_interface/aristo.hpp"
#include "aristo_hardware_interface/utils/actuator_config_loader.hpp"
#include "plato_hardware_interface/utils/parameter_utils.hpp"

#include <exception>
#include <limits>
#include <string>
#include <utility>
#include <pluginlib/class_list_macros.hpp>

namespace aristo_hardware_interface
{

hardware_interface::CallbackReturn AristoHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != aristo_hand::Hand::kNumActuators) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AristoHardware"),
      "Expected %zu joints for Aristo hand, got %zu",
      aristo_hand::Hand::kNumActuators,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto zeroing_it = info_.hardware_parameters.find("zeroing");
  if (zeroing_it != info_.hardware_parameters.end()) {
    try {
      zeroing_requested_ =
        plato_hardware_interface::utils::parse_bool_parameter(zeroing_it->second, "zeroing");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AristoHardware"),
        "Invalid Aristo hardware parameter: %s",
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  std::string actuator_config_yaml_path_override;
  const auto actuator_config_path_it =
    info_.hardware_parameters.find("actuator_config_yaml_path");
  if (actuator_config_path_it != info_.hardware_parameters.end()) {
    actuator_config_yaml_path_override = actuator_config_path_it->second;
  }

  try {
    auto actuator_configs = actuator_config_yaml_path_override.empty() ?
      aristo_actuator::load_aristo_actuator_configs() :
      aristo_actuator::load_aristo_actuator_configs(actuator_config_yaml_path_override);
    hand_ = std::make_unique<aristo_hand::Hand>(std::move(actuator_configs));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AristoHardware"),
      "Failed to initialize Aristo hand: %s",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AristoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AristoHardware"), "Configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AristoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 3);
  auto & joint_states = hand_->joint_states();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_name = info_.joints[i].name;
    state_interfaces.emplace_back(
      joint_name, hardware_interface::HW_IF_POSITION, &joint_states.position_at(i));
    state_interfaces.emplace_back(
      joint_name, hardware_interface::HW_IF_VELOCITY, &joint_states.velocity_at(i));
    state_interfaces.emplace_back(
      joint_name, hardware_interface::HW_IF_EFFORT, &joint_states.effort_at(i));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AristoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size() * 5);
  auto & joint_commands = hand_->joint_commands();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_name = info_.joints[i].name;
    command_interfaces.emplace_back(
      joint_name, hardware_interface::HW_IF_POSITION, &joint_commands.position_at(i));
    command_interfaces.emplace_back(
      joint_name, hardware_interface::HW_IF_VELOCITY, &joint_commands.velocity_at(i));
    command_interfaces.emplace_back(
      joint_name, hardware_interface::HW_IF_EFFORT, &joint_commands.effort_at(i));
    command_interfaces.emplace_back(joint_name, "stiffness", &joint_commands.stiffness_at(i));
    command_interfaces.emplace_back(joint_name, "damping", &joint_commands.damping_at(i));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AristoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hand_->reset_joint_commands(0.0);
  if (zeroing_requested_) {
    RCLCPP_INFO(
      rclcpp::get_logger("AristoHardware"),
      "Aristo embedded zeroing requested.");
  }
  if (!hand_->enable(zeroing_requested_)) {
    RCLCPP_WARN(
      rclcpp::get_logger("AristoHardware"),
      zeroing_requested_ ?
      "One or more Aristo actuators failed to enable/embedded-zero. Continuing activation." :
      "One or more Aristo actuators failed to enable. Continuing activation.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AristoHardware"), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AristoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!hand_->disable()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AristoHardware"),
      "Failed to disable one or more Aristo actuators.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  hand_->reset_joint_commands(std::numeric_limits<double>::quiet_NaN());
  RCLCPP_INFO(rclcpp::get_logger("AristoHardware"), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AristoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!hand_->read()) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AristoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!hand_->write()) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace aristo_hardware_interface

PLUGINLIB_EXPORT_CLASS(aristo_hardware_interface::AristoHardware, hardware_interface::SystemInterface)
