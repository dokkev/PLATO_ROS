#include "plato_hardware_interface/plato.hpp"
#include "plato_hardware_interface/utils/plato_hand_config_loader.hpp"
#include "plato_hardware_interface/utils/parameter_utils.hpp"

#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace plato_hardware_interface
{

hardware_interface::CallbackReturn PlatoHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != plato_hand::Hand::kNumJoints) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PlatoHardware"),
      "Expected %zu joints for Plato hand, got %zu",
      plato_hand::Hand::kNumJoints,
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
        rclcpp::get_logger("PlatoHardware"),
        "Invalid Plato hardware parameter: %s",
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  try {
    // Hand constructor validates the actuator config count and initializes all RobotIO buffers.
    hand_ = std::make_unique<plato_hand::Hand>(plato_hand::load_default_plato_hand_config());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PlatoHardware"),
      "Failed to initialize Plato hand: %s",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PlatoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PlatoHardware"), "Configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PlatoHardware::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> PlatoHardware::export_command_interfaces()
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

hardware_interface::CallbackReturn PlatoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hand_->reset_joint_commands(std::numeric_limits<double>::quiet_NaN());
  hand_->enable(zeroing_requested_);

  RCLCPP_INFO(rclcpp::get_logger("PlatoHardware"), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PlatoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hand_->disable();
  hand_->reset_joint_commands(std::numeric_limits<double>::quiet_NaN());
  RCLCPP_INFO(rclcpp::get_logger("PlatoHardware"), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PlatoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!hand_->read()) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PlatoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!hand_->write_joint_commands()) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace plato_hardware_interface

PLUGINLIB_EXPORT_CLASS(plato_hardware_interface::PlatoHardware, hardware_interface::SystemInterface)
