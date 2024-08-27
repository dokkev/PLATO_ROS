// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

#include "plato_hardware_interface/plato.hpp"

namespace plato_hardware_interface {

void PLATOHardware::set_zero_command(std::vector<double> &command) {
  for (size_t i = 0; i < command.size(); ++i) {
    command[i] = 0.000;
  }
}

void PLATOHardware::set_zero_torque_command(std::vector<double> &command) {
  for (size_t i = 0; i < command.size(); ++i) {
    command[i] = 200.0;
  }
}

void PLATOHardware::set_zero_states(std::vector<double> &joint_states) {
  for (size_t i = 0; i < joint_states.size(); ++i) {
    joint_states[i] = 0.0;
  }
}

void PLATOHardware::stop() {

  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"),
              "Deactivating ...Setting all commands to zero...");

  // set all command effort to 0
  set_zero_command(motor_effort_commands_);
  socket_can_.send_can(motor_effort_commands_);


  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully Stopped!");
}

std::vector<double> PLATOHardware::compute_velocity(
                      const rclcpp::Duration &period, 
                      std::vector<double> joint_position_states_,
                      std::vector<double> joint_position_states_prev_){

  std::vector<double> joint_velocity_states(joint_position_states_.size(), 0.0);

  for (size_t i = 0; i < joint_velocity_states.size(); ++i) {
    joint_velocity_states[i]= (joint_position_states_[i] - joint_position_states_prev_[i]) / period.seconds();
  }

  return joint_velocity_states;
                                      
}

std::vector<double> PLATOHardware::lpf_filter(
                                double alpha,
                               std::vector<double> states,
                               std::vector<double> states_prev){
 
 
  std::vector<double> filtered_states(states.size(), 0.0);
  for (size_t i = 0; i < filtered_states.size(); ++i) {

    filtered_states[i] = alpha * states[i] + (1 - alpha) * states_prev[i];

  }

  return filtered_states;

}
    

hardware_interface::CallbackReturn
PLATOHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize all Joint Vectors
  joint_position_commands_.resize(info_.joints.size(),
                                  std::numeric_limits<double>::quiet_NaN());

  joint_effort_commands_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());
  joint_effort_commands_prev_.resize(info_.joints.size(),
                                     std::numeric_limits<double>::quiet_NaN());

  joint_position_states_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());
  joint_position_states_prev_.resize(info_.joints.size(),
                                     std::numeric_limits<double>::quiet_NaN());
  joint_position_states_raw_.resize(info_.joints.size(),
                                      std::numeric_limits<double>::quiet_NaN());


  joint_velocity_states_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());
  joint_velocity_states_prev_.resize(info_.joints.size(),
                                     std::numeric_limits<double>::quiet_NaN());
  joint_velocity_states_raw_.resize(info_.joints.size(),
                                      std::numeric_limits<double>::quiet_NaN());      

  joint_effort_states_.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());

  // Initialize all Motor Vectors
  motor_effort_commands_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());
  motor_position_states_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());
  motor_position_states_prev_.resize(info_.joints.size(),
                                     std::numeric_limits<double>::quiet_NaN());

  // Shutdown protocol
  rclcpp::on_shutdown(std::bind(&PLATOHardware::stop, this));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PLATOHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"),
              "Configuring ...setting all joint state to 0..");
  set_zero_states(joint_position_states_);
  set_zero_states(joint_position_states_prev_);
  set_zero_states(joint_position_states_raw_);
  set_zero_states(joint_velocity_states_);
  set_zero_states(joint_velocity_states_prev_);
  set_zero_states(joint_velocity_states_raw_);
  set_zero_states(joint_effort_states_);

  PLATOHardware::set_zero_torque_command(motor_position_commands_);
  // PLATOHardware::set_zero_torque_command(joint_position_commands_prev_);
  PLATOHardware::set_zero_command(joint_effort_commands_);
  PLATOHardware::set_zero_command(joint_effort_commands_prev_);

  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PLATOHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.reserve(info_.joints.size() * 3);
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

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PLATOHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  effort_command_interface_names_.reserve(info_.joints.size());
  position_command_interface_names_.reserve(info_.joints.size());


  // Position Command Interface
  // for (size_t i = 0; i < info_.joints.size(); ++i) {
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       info_.joints[i].name, hardware_interface::HW_IF_POSITION,
  //       &joint_position_commands_[i]));
  //   position_command_interface_names_.push_back(
  //       command_interfaces.back().get_name());
  // }

  // Effort Command Interface
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &joint_effort_commands_[i]));
    effort_command_interface_names_.push_back(command_interfaces.back().get_name());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
PLATOHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"),
              "Activating ...please wait...");

  // TODO: Add Gravity Compensation

  // send the zero effort command to the motor
  // PLATOHardware::set_zero_command(joint_effort_commands_);

  // send the zero position command to the motor
  PLATOHardware::set_zero_torque_command(motor_position_commands_);

  // init CAN

  socket_can_.init();
  can_error_.resize(info_.joints.size(), false);

  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PLATOHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"),
              "Deactivating ...Setting all commands to zero...");

  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully deactivated!");

  // send the zero position command to the motor
  PLATOHardware::set_zero_torque_command(motor_position_commands_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PLATOHardware::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) {
  rclcpp::on_shutdown(std::bind(&PLATOHardware::stop, this));

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PLATOHardware::read(const rclcpp::Time &/*time*/,
                    const rclcpp::Duration &period) {

  // repeat 3 times to ensure the motor position is read correctly
  for (unsigned int i = 0; i < 3; i++) {
    // Read the motor position over CAN
    socket_can_.receive_can(motor_position_states_);



    // Adjust the motor position with Offset and Direction
    motor_direction_.convert_motor_to_joint_position(
        motor_position_states_, joint_position_states_);
  }

  //update velocity
   // Define the filter coefficient (alpha)
  const double alpha = 0.01;

  // Calculate and filter the joint velocities
  for (size_t i = 0; i < joint_velocity_states_.size(); i++) {
    // Calculate the raw velocity
    double raw_velocity = (joint_position_states_[i] - joint_position_states_prev_[i]) / period.seconds();

    // Apply the low-pass filter to the velocity
    joint_velocity_states_[i] = alpha * raw_velocity + (1 - alpha) * joint_velocity_states_prev_[i];
  }
  

  // update the previous joint position states
  joint_position_states_prev_ = joint_position_states_;
  joint_velocity_states_prev_ = joint_velocity_states_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PLATOHardware::write(const rclcpp::Time & /*time*/,
                     const rclcpp::Duration & /*period*/) {



  // Adjust joint commands -> motor commands with direction and offset
  motor_direction_.convert_joint_to_motor_effort(joint_effort_commands_,
                                                   motor_effort_commands_);
  // send the motor commands over CAN                                              
  socket_can_.send_can(motor_effort_commands_);
  



  return hardware_interface::return_type::OK;
}

} // namespace plato_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plato_hardware_interface::PLATOHardware,
                       hardware_interface::SystemInterface)
