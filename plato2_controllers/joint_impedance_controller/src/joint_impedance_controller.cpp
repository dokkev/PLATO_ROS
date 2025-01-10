#include "joint_impedance_controller/joint_impedance_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rclcpp/version.h"

namespace joint_impedance_controller
{

JointImpedanceController::JointImpedanceController()
: controller_interface::ControllerInterface(), rt_command_ptr_(nullptr), joints_command_subscriber_(nullptr)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  dof_ = params_.joints.size();

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "Configuration successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

// controller_interface::InterfaceConfiguration JointImpedanceController::command_interface_configuration() const
// {


//   return command_interfaces_config;
// }

// controller_interface::InterfaceConfiguration JointImpedanceController::state_interface_configuration() const
// {



//   return conf;
// }

controller_interface::return_type JointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{


  return controller_interface::return_type::OK;
}

}  // namespace joint_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(joint_impedance_controller::JointImpedanceController, controller_interface::ControllerInterface)
