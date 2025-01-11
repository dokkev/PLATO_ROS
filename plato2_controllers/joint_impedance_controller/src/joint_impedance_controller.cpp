#include "joint_impedance_controller/joint_impedance_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace joint_impedance_controller
{

JointImpedanceController::JointImpedanceController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter listener was not initialized");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  // Validate parameters
  if (params_.joints.empty()) 
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Store parameters
  joint_names_ = params_.joints;

  // Setup command interface types with effort interface
  command_interface_types_.clear();
  for (const auto & joint : joint_names_)
  {
    command_interface_types_.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }

  // Initialize state vectors
  const size_t num_joints = joint_names_.size();
  positions_.resize(num_joints, 0.0);
  velocities_.resize(num_joints, 0.0);
  efforts_.resize(num_joints, 0.0);

  // Log parameters
  RCLCPP_INFO(get_node()->get_logger(), "Configured joints: ");
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (i < 2)
    {
      RCLCPP_INFO(
        get_node()->get_logger(), 
        "Joint '%s': DIRECT POSITION CONTROL", 
        joint_names_[i].c_str());
    }
    else
    {
      RCLCPP_INFO(
        get_node()->get_logger(), 
        "Joint '%s': stiffness=%.2f, damping=%.2f", 
        joint_names_[i].c_str(),
        params_.impedance.joints_map[joint_names_[i]].stiffness,
        params_.impedance.joints_map[joint_names_[i]].damping);
    }
  }

  // Create command subscriber
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Check command interfaces
  if (command_interfaces_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu",
      joint_names_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get ordered interfaces
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Command interface configuration mismatch");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset command buffer and zero commands
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  for (auto & command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Get the currently commanded values
  auto impedance_commands = rt_command_ptr_.readFromRT();

  // Read current states
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    positions_[i] = state_interfaces_[i * 3].get_value();
    velocities_[i] = state_interfaces_[i * 3 + 1].get_value();
    efforts_[i] = state_interfaces_[i * 3 + 2].get_value();
  }

  // No command received yet
  if (!impedance_commands || !(*impedance_commands))
  {
    return controller_interface::return_type::OK;
  }

  // Validate message size
  if ((*impedance_commands)->position.size() != joint_names_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Command size (%zu) does not match number of joints (%zu)",
      (*impedance_commands)->position.size(), joint_names_.size());
    return controller_interface::return_type::ERROR;
  }

  // Apply control for each joint
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (i < 2)  // First two joints (0 and 1): direct position control
    {
      command_interfaces_[i].set_value((*impedance_commands)->position[i]);
    }
    else  // Other joints: impedance control
    {
      // Calculate impedance control terms
      const double position_error = (*impedance_commands)->position[i] - positions_[i];
      const double velocity_error = (*impedance_commands)->velocity.empty() ? 
        -velocities_[i] : (*impedance_commands)->velocity[i] - velocities_[i];
      
      // Get stiffness and damping values (use defaults if not provided)
      const double stiffness = (*impedance_commands)->stiffness.empty() ? 
        params_.impedance.joints_map[joint_names_[i]].stiffness : (*impedance_commands)->stiffness[i];
      const double damping = (*impedance_commands)->damping.empty() ? 
        params_.impedance.joints_map[joint_names_[i]].damping : (*impedance_commands)->damping[i];
      const double effort_ff = (*impedance_commands)->effort_ff.empty() ? 
        0.0 : (*impedance_commands)->effort_ff[i];
      
      // Compute control command
      const double effort_cmd = 
        stiffness * position_error +  // Spring term
        damping * velocity_error +    // Damper term
        effort_ff;                    // Feedforward term

      command_interfaces_[i].set_value(effort_cmd);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.clear();
  
  // Add state interfaces for the joints
  for (const auto & joint : joint_names_)
  {
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  
  return state_interfaces_config;
}

}  // namespace joint_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_impedance_controller::JointImpedanceController, controller_interface::ControllerInterface)