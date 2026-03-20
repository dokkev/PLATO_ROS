#include "aristo_hardware_interface/aristo.hpp"

#include <exception>
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

  try {
    hand_ = std::make_unique<aristo_hand::Hand>();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AristoHardware"),
      "Failed to initialize Aristo hand: %s",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  hand_->initialize_joint_buffers(aristo_hand::Hand::kNumActuators, 0.0);
  ft_sensor_states_.resize(aristo_hand::Hand::kNumFtSensors);

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
  state_interfaces.reserve(info_.joints.size() * 3 + hand_->get_num_ft_sensors() * 6);
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

  for (size_t i = 0; i < hand_->get_num_ft_sensors(); ++i) {
    const std::string sensor_name = "ft_sensor" + std::to_string(i + 1);
    state_interfaces.emplace_back(sensor_name, "force.x", &ft_sensor_states_[i].force.x);
    state_interfaces.emplace_back(sensor_name, "force.y", &ft_sensor_states_[i].force.y);
    state_interfaces.emplace_back(sensor_name, "force.z", &ft_sensor_states_[i].force.z);
    state_interfaces.emplace_back(sensor_name, "torque.x", &ft_sensor_states_[i].torque.x);
    state_interfaces.emplace_back(sensor_name, "torque.y", &ft_sensor_states_[i].torque.y);
    state_interfaces.emplace_back(sensor_name, "torque.z", &ft_sensor_states_[i].torque.z);
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
  hand_->reset_joint_commands();
  hand_->enable();
  hand_->set_current_position_as_zero();
  RCLCPP_INFO(rclcpp::get_logger("AristoHardware"), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AristoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hand_->reset_joint_commands();
  hand_->disable();
  RCLCPP_INFO(rclcpp::get_logger("AristoHardware"), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AristoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  hand_->poll_can_bus();
  hand_->read_joint_states();
  hand_->update_ft_sensor_states(ft_sensor_states_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AristoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  hand_->write_joint_commands();
  return hardware_interface::return_type::OK;
}

}  // namespace aristo_hardware_interface

PLUGINLIB_EXPORT_CLASS(aristo_hardware_interface::AristoHardware, hardware_interface::SystemInterface)
