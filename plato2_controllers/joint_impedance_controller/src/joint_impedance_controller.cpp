#include "joint_impedance_controller/joint_impedance_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace joint_impedance_controller
{

JointImpedanceController::JointImpedanceController()
: controller_interface::ControllerInterface(), rt_command_ptr_(nullptr), joints_command_subscriber_(nullptr)
{
}

controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<joint_impedance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception &e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  dof_ = params_.joints.size();
  current_positions_.resize(dof_, 0.0);
  current_velocities_.resize(dof_, 0.0);

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "Configuration successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, params_.joints, "effort", ordered_interfaces) ||
    dof_ != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu", dof_, ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Activation successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto &joint_name : params_.joints)
  {
    command_interfaces_config.names.push_back(joint_name + "/effort");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration JointImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto &joint_name : params_.joints)
  {
    state_interfaces_config.names.push_back(joint_name + "/position");
    state_interfaces_config.names.push_back(joint_name + "/velocity");
  }

  return state_interfaces_config;
}

controller_interface::return_type JointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->position.size() != dof_ ||
      (*joint_commands)->velocity.size() != dof_ ||
      (*joint_commands)->stiffness.size() != dof_ ||
      (*joint_commands)->damping.size() != dof_ ||
      (*joint_commands)->torque_ff.size() != dof_)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Command size mismatch. Expected size: %zu, received: position (%zu), velocity (%zu), stiffness (%zu), damping (%zu), torque_ff (%zu)",
      dof_,
      (*joint_commands)->position.size(),
      (*joint_commands)->velocity.size(),
      (*joint_commands)->stiffness.size(),
      (*joint_commands)->damping.size(),
      (*joint_commands)->torque_ff.size());
    return controller_interface::return_type::ERROR;
  }

  for (size_t index = 0; index < dof_; ++index)
  {
    double position_error = (*joint_commands)->position[index] - current_positions_[index];
    double velocity_error = (*joint_commands)->velocity[index] - current_velocities_[index];
    double torque_command =
      (*joint_commands)->stiffness[index] * position_error +
      (*joint_commands)->damping[index] * velocity_error +
      (*joint_commands)->torque_ff[index];

    command_interfaces_[index].set_value(torque_command);
  }

  return controller_interface::return_type::OK;
}

}  // namespace joint_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(joint_impedance_controller::JointImpedanceController, controller_interface::ControllerInterface)
