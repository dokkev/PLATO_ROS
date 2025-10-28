#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

#include "plato2_hardware_interface/plato2.hpp"

namespace plato2_hardware_interface {

hardware_interface::CallbackReturn
PLATO2Hardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize hand with pcan_interface_
  hand_ = std::make_unique<plato2_hand::Hand>(pcan_interface_);

  // Initialize command and state vectors
  const size_t num_joints = info_.joints.size();
  joint_position_commands_.resize(num_joints, 0.0);
  joint_velocity_commands_.resize(num_joints, 0.0);
  joint_effort_commands_.resize(num_joints, 0.0);
  joint_stiffness_commands_.resize(num_joints, 0.0);
  joint_damping_commands_.resize(num_joints, 0.0);
  
  joint_position_states_.resize(num_joints, 0.0);
  joint_velocity_states_.resize(num_joints, 0.0);
  joint_effort_states_.resize(num_joints, 0.0);

  // Initialize FT sensor states vector
  ft_sensor_states_.resize(hand_->get_num_ft_sensors());

  return hardware_interface::CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////////////

hardware_interface::CallbackReturn
PLATO2Hardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"),"Configuring ...setting all joint state to 0..");

  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////////////

std::vector<hardware_interface::StateInterface>
PLATO2Hardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Reserve space for joint and FT sensor interfaces
  state_interfaces.reserve(info_.joints.size() * 3 + (hand_->get_num_ft_sensors() * 6));

  // Joint state interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &joint_velocity_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &joint_effort_states_[i]));
  }


  // Initialize FT sensor states
  for (size_t i = 0; i < hand_->get_num_ft_sensors(); ++i) {
    std::string sensor_name = "ft_sensor" + std::to_string(i + 1);
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "force.x", &ft_sensor_states_[i].force.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "force.y", &ft_sensor_states_[i].force.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "force.z", &ft_sensor_states_[i].force.z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "torque.x", &ft_sensor_states_[i].torque.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "torque.y", &ft_sensor_states_[i].torque.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "torque.z", &ft_sensor_states_[i].torque.z));
}

  return state_interfaces;
}

/////////////////////////////////////////////////////////////////////////

std::vector<hardware_interface::CommandInterface>
PLATO2Hardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // We'll expose 5 command interfaces per joint: position, velocity, effort,
  // stiffness (kp) and damping (kd). Reserve accordingly.
  command_interfaces.reserve(info_.joints.size() * 5);
  position_command_interface_names_.reserve(info_.joints.size());
  velocity_command_interface_names_.reserve(info_.joints.size());
  effort_command_interface_names_.reserve(info_.joints.size());
  stiffness_command_interface_names_.reserve(info_.joints.size());
  damping_command_interface_names_.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
  // Position
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[i].name, hardware_interface::HW_IF_POSITION,
    &joint_position_commands_[i]));
  position_command_interface_names_.push_back(command_interfaces.back().get_name());

  // Velocity
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
    &joint_velocity_commands_[i]));
  velocity_command_interface_names_.push_back(command_interfaces.back().get_name());

  // Effort
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
    &joint_effort_commands_[i]));
  effort_command_interface_names_.push_back(command_interfaces.back().get_name());

  // Stiffness (Kp)
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[i].name, "stiffness", &joint_stiffness_commands_[i]));
  stiffness_command_interface_names_.push_back(command_interfaces.back().get_name());

  // Damping (Kd)
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[i].name, "damping", &joint_damping_commands_[i]));
  damping_command_interface_names_.push_back(command_interfaces.back().get_name());
  }

  return command_interfaces;
}

/////////////////////////////////////////////////////////////////////////

hardware_interface::CallbackReturn
PLATO2Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"),
              "Activating ...please wait...");

  // Initialize all command vectors to zero in a single loop
  const size_t num_joints = joint_position_commands_.size();
  for (size_t i = 0; i < num_joints; ++i) {
    joint_position_commands_[i] = 0.0;
    joint_velocity_commands_[i] = 0.0;
    joint_effort_commands_[i] = 0.0;
    joint_stiffness_commands_[i] = 0.0;
    joint_damping_commands_[i] = 0.0;
  }

  // set current position as zero
  // hand_->set_current_position_as_zero();

  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////////////

hardware_interface::CallbackReturn
PLATO2Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/////////////////////////////////////////////////////////////////////////

hardware_interface::return_type
PLATO2Hardware::read(const rclcpp::Time &time,
                    const rclcpp::Duration &period) {

  // Read joint states from the hardware
  hand_->update_joint_states(joint_position_states_, 
                             joint_velocity_states_, 
                             joint_effort_states_);

  // Read FT sensor data
  // hand_->update_ft_sensor_states(ft_sensor_states_);

  return hardware_interface::return_type::OK;
}

/////////////////////////////////////////////////////////////////////////

hardware_interface::return_type
PLATO2Hardware::write(const rclcpp::Time &time,
                     const rclcpp::Duration & /*period*/) {

  // Set recommended impedance values for thumb joints
  joint_stiffness_commands_[0] = 3.0;
  joint_damping_commands_[0] = 0.20;
  joint_stiffness_commands_[1] = 0.0;
  joint_damping_commands_[1] = 0.10;

  // Send impedance commands to the hardware
  hand_->set_impedance_command(joint_position_commands_, 
                               joint_velocity_commands_,
                               joint_stiffness_commands_,
                               joint_damping_commands_,
                               joint_effort_commands_);

  return hardware_interface::return_type::OK;
}

} // namespace plato2_hardware_interface

/////////////////////////////////////////////////////////////////////////

PLUGINLIB_EXPORT_CLASS(plato2_hardware_interface::PLATO2Hardware,
                      hardware_interface::SystemInterface)