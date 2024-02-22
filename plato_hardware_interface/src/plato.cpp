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

namespace plato_hardware_interface
{



hardware_interface::CallbackReturn PLATOHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize all Joint Vectors
  joint_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_effort_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  joint_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_effort_states_.resize(info_.joints.size(),   std::numeric_limits<double>::quiet_NaN());

  // Initialize all Motor Vectors
  motor_effort_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  motor_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // init CAN
  socket_can_.init();

  // make commands and states all zero for testing
  // for (size_t i = 0; i < joint_effort_commands_.size(); ++i){
  //   joint_effort_commands_[i] = 0.0;
  //   joint_effort_states_[i] = 0.0;
  //   joint_position_commands_[i] = 0.0;
  //   joint_velocity_states_[i] = 0.0;
  // }


  rclcpp::on_shutdown(std::bind(&PLATOHardware::stop, this));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PLATOHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{




  PLATOHardware::set_zero_joint_states(joint_position_states_);
  PLATOHardware::set_zero_joint_states(joint_velocity_states_);
  PLATOHardware::set_zero_joint_states(joint_effort_states_);
  
  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully configured!");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PLATOHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.reserve(info_.joints.size() * 3);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_position_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_effort_states_[i]));
  }


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PLATOHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  effort_command_interface_names_.reserve(info_.joints.size());

  for (size_t i=0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_effort_commands_[i]));
    effort_command_interface_names_.push_back(command_interfaces.back().get_name());
  }

  


  return command_interfaces;
}

hardware_interface::CallbackReturn PLATOHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("PLATOHardware"), "Activating ...please wait...");



    // joint_effort_commands_ = joint_effort_states_;


    // TODO: Add Gravity Compensation


  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PLATOHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(
    rclcpp::get_logger("PLATOHardware"), "Deactivating ...Setting all commands to zero...");


    RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully deactivated!");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PLATOHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/) 
{
  rclcpp::on_shutdown(std::bind(&PLATOHardware::stop, this));

  return CallbackReturn::SUCCESS;

}

hardware_interface::return_type PLATOHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // Get unadjusted motor encoder angles from CAN bus

  socket_can_.receive_can_rx_msg(motor_position_states_);
  motor_direction_.convert_motor_to_joint_position(motor_position_states_, joint_position_states_);

  //print motor_position_states_
  // RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Motor Position States: {}", motor_position_states_);
  // Adjust motor encoder angles with offset and direction
  // motor_direction_.convert_motor_to_joint_position(motor_position_states_, joint_position_states_);

  // print joint states
  // for (size_t i = 0; i < joint_position_states_.size(); ++i) {
  //     RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Moint %zu position: %f", i, motor_position_states_[i]);
  // }




  // RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PLATOHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  PLATOHardware::set_zero_joint_states(joint_effort_commands_);


  return hardware_interface::return_type::OK;
}

}  // namespace plato_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plato_hardware_interface::PLATOHardware, hardware_interface::SystemInterface)
