#include "plato2_controllers/joint_impedance_controller.hpp"

#include <string>
#include <vector>
#include "rclcpp/logging.hpp"

namespace joint_impedance_controller
{
JointImpedanceController::JointImpedanceController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
  command_interface_types_ = {"effort"};
  state_interface_types_ = {"position", "velocity", "effort"};

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    command_interfaces_config.names.push_back(joint_name + "/effort");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration JointImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      state_interfaces_config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "Joint Impedance Controller configured successfully.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Joint Impedance Controller activated successfully.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  // No command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  // Check command size matches
  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value((*joint_commands)->data[i]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace joint_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_impedance_controller::JointImpedanceController, controller_interface::ControllerInterface)
