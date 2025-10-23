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
  const size_t num_joints = joint_names_.size();

  // Initialize state vectors
  positions_.resize(num_joints, 0.0);
  velocities_.resize(num_joints, 0.0);
  efforts_.resize(num_joints, 0.0);

  // Create command subscriber
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  
  // Create State Publisher
  publisher_ = get_node()->create_publisher<plato2_interfaces::msg::ImpedanceControllerState>(
    "~/controller_state", rclcpp::SystemDefaultsQoS());

  state_publisher_ = std::make_unique<StatePublisher>(publisher_);
  state_publisher_->lock();

  state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.position_desired.resize(num_joints, 0.0);
  state_publisher_->msg_.velocity_desired.resize(num_joints, 0.0);
  state_publisher_->msg_.position_actual.resize(num_joints, 0.0);
  state_publisher_->msg_.velocity_actual.resize(num_joints, 0.0);
  state_publisher_->msg_.position_error.resize(num_joints, 0.0);
  state_publisher_->msg_.velocity_error.resize(num_joints, 0.0);
  
  state_publisher_->msg_.stiffness.resize(num_joints, 0.0);
  state_publisher_->msg_.damping.resize(num_joints, 0.0);
  state_publisher_->msg_.effort_ff.resize(num_joints, 0.0);
  state_publisher_->msg_.effort_fb.resize(num_joints, 0.0);
  state_publisher_->msg_.effort_desired.resize(num_joints, 0.0);
  state_publisher_->msg_.effort_actual.resize(num_joints, 0.0);
  
  state_publisher_->msg_.force_x.resize(3, 0.0);
  state_publisher_->msg_.force_y.resize(3, 0.0);
  state_publisher_->msg_.force_z.resize(3, 0.0);
  state_publisher_->msg_.force_norm.resize(3, 0.0);

  state_publisher_->unlock();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Clear and reserve interfaces
  ordered_position_command_interfaces_.clear();
  ordered_velocity_command_interfaces_.clear();
  ordered_effort_command_interfaces_.clear();
  ordered_stiffness_command_interfaces_.clear();
  ordered_damping_command_interfaces_.clear();

  const auto num_joints = joint_names_.size();
  ordered_position_command_interfaces_.reserve(num_joints);
  ordered_velocity_command_interfaces_.reserve(num_joints);
  ordered_effort_command_interfaces_.reserve(num_joints);
  ordered_stiffness_command_interfaces_.reserve(num_joints);
  ordered_damping_command_interfaces_.reserve(num_joints);

  for (const auto & joint_name : joint_names_)
  {
    for (auto & command_interface : command_interfaces_)
    {
      if (command_interface.get_name() == joint_name + "/" + hardware_interface::HW_IF_POSITION)
      {
        ordered_position_command_interfaces_.emplace_back(command_interface);
      }
      if (command_interface.get_name() == joint_name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        ordered_velocity_command_interfaces_.emplace_back(command_interface);
      }
      if (command_interface.get_name() == joint_name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        ordered_effort_command_interfaces_.emplace_back(command_interface);
      }
      if (command_interface.get_name() == joint_name + "/stiffness")
      {
        ordered_stiffness_command_interfaces_.emplace_back(command_interface);
      }
      if (command_interface.get_name() == joint_name + "/damping")
      {
        ordered_damping_command_interfaces_.emplace_back(command_interface);
      }
    }
  }

  // Reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  // Check if all interfaces were found
  if (ordered_position_command_interfaces_.size() != num_joints ||
      ordered_velocity_command_interfaces_.size() != num_joints ||
      ordered_effort_command_interfaces_.size() != num_joints ||
      ordered_stiffness_command_interfaces_.size() != num_joints ||
      ordered_damping_command_interfaces_.size() != num_joints)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Some command interfaces were not found!");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset command buffer and zero commands
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    ordered_position_command_interfaces_[i].get().set_value(0.0);
    ordered_velocity_command_interfaces_[i].get().set_value(0.0);
    ordered_effort_command_interfaces_[i].get().set_value(0.0);
    ordered_stiffness_command_interfaces_[i].get().set_value(0.0);
    ordered_damping_command_interfaces_[i].get().set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto impedance_commands = rt_command_ptr_.readFromRT();

  // No command received yet
  if (!impedance_commands || !(*impedance_commands))
  {
    return controller_interface::return_type::OK;
  }

  const auto& commands = **impedance_commands;

  // Validate message size
  if (commands.position.size() != joint_names_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Command size (%zu) does not match number of joints (%zu)",
      commands.position.size(), joint_names_.size());
    return controller_interface::return_type::ERROR;
  }

  // Pass commands to hardware
  for (size_t i = 0; i < joint_names_.size(); ++i)
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void JointImpedanceController::publish_state(const rclcpp::Time & time, const CmdType& command)
{
  if (state_publisher_->trylock()){
    auto& msg = state_publisher_->msg_;
    msg.header.stamp = time;

    // State assignments
    msg.position_actual = positions_;
    msg.velocity_actual = velocities_;
    msg.effort_actual = efforts_;
    
    msg.position_desired = command.position;
    msg.velocity_desired = command.velocity;
    
    // Calculate end-effector forces
    // Finger 1: joints [2,3]
    const Jacobian Jinv1 = get_Jinv(positions_[2], positions_[3]);
    msg.force_x[0] = Jinv1.j11 * efforts_[2] + Jinv1.j12 * efforts_[3];
    msg.force_z[0] = Jinv1.j21 * efforts_[2] + Jinv1.j22 * efforts_[3];
    msg.force_y[0] = 0.0;
    msg.force_norm[0] = std::sqrt(msg.force_x[0]*msg.force_x[0] + msg.force_z[0]*msg.force_z[0]);
  
    // Finger 2: joints [4,5]
    const Jacobian Jinv2 = get_Jinv(positions_[4], positions_[5]);
    msg.force_x[1] = Jinv2.j11 * efforts_[4] + Jinv2.j12 * efforts_[5];
    msg.force_z[1] = Jinv2.j21 * efforts_[4] + Jinv2.j22 * efforts_[5];
    msg.force_y[1] = 0.0;
    msg.force_norm[1] = std::sqrt(msg.force_x[1]*msg.force_x[1] + msg.force_z[1]*msg.force_z[1]);
  
    // Finger 3: joints [6,7]
    const Jacobian Jinv3 = get_Jinv(positions_[6], positions_[7]);
    msg.force_x[2] = Jinv3.j11 * efforts_[6] + Jinv3.j12 * efforts_[7];
    msg.force_z[2] = Jinv3.j21 * efforts_[6] + Jinv3.j22 * efforts_[7];
    msg.force_y[2] = 0.0;
    msg.force_norm[2] = std::sqrt(msg.force_x[2]*msg.force_x[2] + msg.force_z[2]*msg.force_z[2]);
  
    state_publisher_->unlockAndPublish();
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////

JointImpedanceController::Jacobian JointImpedanceController::get_J(double theta1, double theta2) const
{
  Jacobian J;
  J.j11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2);
  J.j12 = -L2 * sin(theta1 + theta2);
  J.j21 = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
  J.j22 = L2 * cos(theta1 + theta2);

  return J;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

JointImpedanceController::Jacobian JointImpedanceController::get_Jinv(double theta1, double theta2) const
{
  // First get regular Jacobian
  Jacobian J = get_J(theta1, theta2);
  
  // Calculate determinant
  double det = J.j11*J.j22 - J.j12*J.j21;

  const double eps = 1e-6;
  if (std::abs(det) < eps) {
    det = eps * (det >= 0 ? 1 : -1);
  }
  
  // Create inverse Jacobian
  Jacobian Jinv;
  Jinv.j11 =  J.j22/det;
  Jinv.j12 = -J.j12/det;
  Jinv.j21 = -J.j21/det;
  Jinv.j22 =  J.j11/det;
  
  return Jinv;
}


}  // namespace joint_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_impedance_controller::JointImpedanceController, controller_interface::ControllerInterface)