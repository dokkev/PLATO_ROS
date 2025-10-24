#include "joint_impedance_controller/joint_impedance_controller.hpp"

#include <memory>
#include <string>
#include <vector>
#include <cmath>

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

////////////////////////////////////////////////////////////////////////

controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter listener was not initialized");
    return controller_interface::CallbackReturn::ERROR;
  }

  params_ = param_listener_->get_params();

  if (params_.joints.empty()) 
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_names_ = params_.joints;
  const size_t num_joints = joint_names_.size();

  // Initialize state vectors
  positions_.resize(num_joints, 0.0);
  velocities_.resize(num_joints, 0.0);
  efforts_.resize(num_joints, 0.0);

  // Create command subscriber with VOLATILE QoS
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", qos,
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  // Create state publisher
  publisher_ = get_node()->create_publisher<StateMsg>(
    "~/controller_state", rclcpp::SystemDefaultsQoS());

  state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<StateMsg>>(publisher_);

  // Pre-allocate message fields to avoid runtime allocations
  state_publisher_->lock();
  auto& msg = state_publisher_->msg_;
  msg.joint_names = joint_names_;
  msg.position_desired.resize(num_joints, 0.0);
  msg.velocity_desired.resize(num_joints, 0.0);
  msg.position_actual.resize(num_joints, 0.0);
  msg.velocity_actual.resize(num_joints, 0.0);
  msg.position_error.resize(num_joints, 0.0);
  msg.velocity_error.resize(num_joints, 0.0);
  msg.stiffness.resize(num_joints, 0.0);
  msg.damping.resize(num_joints, 0.0);
  msg.effort_ff.resize(num_joints, 0.0);
  msg.effort_fb.resize(num_joints, 0.0);
  msg.effort_desired.resize(num_joints, 0.0);
  msg.effort_actual.resize(num_joints, 0.0);
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "Configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////

controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto num_joints = joint_names_.size();

  // Clear and reserve interface vectors (prevents reallocation)
  ordered_position_command_interfaces_.clear();
  ordered_velocity_command_interfaces_.clear();
  ordered_effort_command_interfaces_.clear();
  ordered_stiffness_command_interfaces_.clear();
  ordered_damping_command_interfaces_.clear();

  ordered_position_command_interfaces_.reserve(num_joints);
  ordered_velocity_command_interfaces_.reserve(num_joints);
  ordered_effort_command_interfaces_.reserve(num_joints);
  ordered_stiffness_command_interfaces_.reserve(num_joints);
  ordered_damping_command_interfaces_.reserve(num_joints);

  // Gather command interfaces in order
  for (const auto & joint_name : joint_names_)
  {
    const std::string pos_name = joint_name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string vel_name = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    const std::string eff_name = joint_name + "/" + hardware_interface::HW_IF_EFFORT;
    const std::string stiff_name = joint_name + "/stiffness";
    const std::string damp_name = joint_name + "/damping";

    for (auto & command_interface : command_interfaces_)
    {
      const auto& iface_name = command_interface.get_name();
      
      if (iface_name == pos_name) {
        ordered_position_command_interfaces_.emplace_back(command_interface);
      } else if (iface_name == vel_name) {
        ordered_velocity_command_interfaces_.emplace_back(command_interface);
      } else if (iface_name == eff_name) {
        ordered_effort_command_interfaces_.emplace_back(command_interface);
      } else if (iface_name == stiff_name) {
        ordered_stiffness_command_interfaces_.emplace_back(command_interface);
      } else if (iface_name == damp_name) {
        ordered_damping_command_interfaces_.emplace_back(command_interface);
      }
    }
  }

  // Validate all interfaces were found
  if (ordered_position_command_interfaces_.size() != num_joints ||
      ordered_velocity_command_interfaces_.size() != num_joints ||
      ordered_effort_command_interfaces_.size() != num_joints ||
      ordered_stiffness_command_interfaces_.size() != num_joints ||
      ordered_damping_command_interfaces_.size() != num_joints)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Not all command interfaces found!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Activated successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  
  // Zero all commands
  const size_t num_joints = joint_names_.size();
  for (size_t i = 0; i < num_joints; ++i)
  {
    ordered_position_command_interfaces_[i].get().set_value(0.0);
    ordered_velocity_command_interfaces_[i].get().set_value(0.0);
    ordered_effort_command_interfaces_[i].get().set_value(0.0);
    ordered_stiffness_command_interfaces_[i].get().set_value(0.0);
    ordered_damping_command_interfaces_[i].get().set_value(0.0);
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////

void JointImpedanceController::read_state_interfaces()
{
  const size_t num_joints = joint_names_.size();
  for (size_t i = 0; i < num_joints; ++i)
  {
    positions_[i] = state_interfaces_[i * 3].get_value();
    velocities_[i] = state_interfaces_[i * 3 + 1].get_value();
    efforts_[i] = state_interfaces_[i * 3 + 2].get_value();
  }
}

////////////////////////////////////////////////////////////////////////

controller_interface::return_type JointImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Read state interfaces
  read_state_interfaces();

  // Get commands from realtime buffer
  auto impedance_commands = rt_command_ptr_.readFromRT();

  if (!impedance_commands || !(*impedance_commands))
  {
    return controller_interface::return_type::OK;
  }

  const auto& commands = **impedance_commands;
  const size_t num_joints = joint_names_.size();

  // Validate command size
  if (commands.position.size() != num_joints)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Command size mismatch: %zu expected, got %zu",
      num_joints, commands.position.size());
    return controller_interface::return_type::ERROR;
  }

  // Write commands to hardware
  for (size_t i = 0; i < num_joints; ++i)
  {
    ordered_position_command_interfaces_[i].get().set_value(commands.position[i]);
    ordered_velocity_command_interfaces_[i].get().set_value(commands.velocity[i]);
    ordered_effort_command_interfaces_[i].get().set_value(commands.effort_ff[i]);
    ordered_stiffness_command_interfaces_[i].get().set_value(commands.stiffness[i]);
    ordered_damping_command_interfaces_[i].get().set_value(commands.damping[i]);
  }

  // Publish state
  publish_state(time, commands);

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  command_interfaces_config.names.clear();
  for (const auto & joint : joint_names_)
  {
    command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    command_interfaces_config.names.push_back(joint + "/stiffness");
    command_interfaces_config.names.push_back(joint + "/damping");
  }

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

////////////////////////////////////////////////////////////////////////

void JointImpedanceController::publish_state(const rclcpp::Time & time, const CmdType& command)
{
  if (!state_publisher_->trylock()) {
    return;
  }

  auto& msg = state_publisher_->msg_;
  msg.header.stamp = time;

  // Assign states (no allocation, vectors already sized)
  msg.position_actual = positions_;
  msg.velocity_actual = velocities_;
  msg.effort_actual = efforts_;
  msg.position_desired = command.position;
  msg.velocity_desired = command.velocity;

  state_publisher_->unlockAndPublish();
}

////////////////////////////////////////////////////////////////////////

}  // namespace joint_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_impedance_controller::JointImpedanceController, controller_interface::ControllerInterface)