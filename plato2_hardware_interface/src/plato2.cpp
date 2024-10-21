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

#include "plato2_hardware_interface/plato2.hpp"

namespace plato2_hardware_interface {


hardware_interface::CallbackReturn
PLATO2Hardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize all Joint Vectors
  joint_position_commands_.resize(info_.joints.size(),
                                  std::numeric_limits<double>::quiet_NaN());

  joint_effort_commands_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());

  joint_position_states_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());

  joint_velocity_states_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());

  joint_effort_states_.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PLATO2Hardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"),
              "Configuring ...setting all joint state to 0..");


  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PLATO2Hardware::export_state_interfaces() {
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
PLATO2Hardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  effort_command_interface_names_.reserve(info_.joints.size());
  position_command_interface_names_.reserve(info_.joints.size());


  // Position Command Interface
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_position_commands_[i]));
    position_command_interface_names_.push_back(
        command_interfaces.back().get_name());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
PLATO2Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"),
              "Activating ...please wait...");


  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Successfully activated!");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PLATO2Hardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {


  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Successfully deactivated!");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PLATO2Hardware::read(const rclcpp::Time &/*time*/,
                    const rclcpp::Duration &period) {


  // Initialize all Joint Vectors to 0
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_position_states_[i] = 0.0;
    joint_velocity_states_[i] = 0.0;
    joint_effort_states_[i] = 0.0; 
  }

  hand_.update_states();


  // for (int i = 0; i < 20; i++) {
  //   // Read the CAN bus
    // TPCANMsg msg;
    // if (pcan_interface_.get_buffer_message(msg)) {
  //     // RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Received message from CAN");
  //     pcan_interface_.print_message(msg);
  // }


  // #endif

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PLATO2Hardware::write(const rclcpp::Time &time,
                     const rclcpp::Duration & /*period*/) {
            
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_position_commands_[i] = 0.0;
  }


  // TPCANMsg msg;
  // msg.ID = 0x11;
  // control_msg_.stop_motor(msg);
  // pcan_interface_.send_message(msg);


  // msg.ID = 0x12;
  // control_msg_.stop_control(msg);
  // pcan_interface_.send_message(msg);


  // msg.ID = 0x13;
  // control_msg_.stop_motor(msg);
  // pcan_interface_.send_message(msg);
  

  // TPCANMsg msg2;
  // msg2.ID = 0x14;
  // control_msg_.stop_motor(msg2);
  // pcan_interface_.send_message(msg2);
  
  // TPCANMsg msg3;
  // msg3.ID = 0x15;
  // control_msg_.stop_motor(msg3);
  // pcan_interface_.send_message(msg3);


  // msg.ID = 0x16;
  // control_msg_.stop_motor(msg);
  // pcan_interface_.send_message(msg);

  // msg.ID = 0x17;
  // control_msg_.stop_motor(msg);
  // pcan_interface_.send_message(msg);

  // msg.ID = 0x18;
  // control_msg_.stop_motor(msg);
  // pcan_interface_.send_message(msg);

  // print the time
  // RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Current time: %f", time.seconds());









  return hardware_interface::return_type::OK;
}

} // namespace plato2_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plato2_hardware_interface::PLATO2Hardware,
                       hardware_interface::SystemInterface)
