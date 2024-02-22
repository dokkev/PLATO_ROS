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

#ifndef PLATO_HARDWARE_INTERFACE__PLATO_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO_HPP_

#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <limits>



#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/joint_state.h>

#include "plato_hardware_interface/plato_common.hpp"
#include "plato_hardware_interface/plato_motor_direction.hpp"
#include "plato_hardware_interface/plato_socket_can.hpp"

#include "plato_hardware_interface/visibility_control.h"

namespace plato_hardware_interface
{

class PLATOHardware : public hardware_interface::SystemInterface 
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PLATOHardware);

    PLATO_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

    PLATO_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    PLATO_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    PLATO_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    PLATO_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    PLATO_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State & previous_state) override;
    
    PLATO_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    PLATO_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    PLATO_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;


    private:
      /// The size of this vector is (standard_interfaces_.size() x nr_joints)
      std::vector<double> motor_effort_commands_;
      std::vector<double> motor_position_states_;

      std::vector<double> joint_position_commands_;
      std::vector<double> joint_effort_commands_;
      
      std::vector<double> joint_position_states_;
      std::vector<double> joint_velocity_states_;
      std::vector<double> joint_effort_states_;
      std::vector<double> ft_sensor_states_;

      std::vector<std::string> effort_command_interface_names_;

      plato_socket_can::PlatoSocketCAN socket_can_;

      plato_motor_direction::PlatoMotorDirection motor_direction_;

     

      // Set either command or joint_states to all 0 for test
      void set_zero_command(std::vector<double>& command){
        for (size_t i = 0; i < command.size(); ++i) {
          command[i] = 0.0;
        }
      }
      void set_zero_joint_states(std::vector<double>& joint_states){
        for (size_t i = 0; i < joint_states.size(); ++i) {
          joint_states[i] = 0.0;
        }
      }

      void stop(){
        set_zero_command(motor_effort_commands_);
        RCLCPP_INFO(
        rclcpp::get_logger("PLATOHardware"), "Deactivating ...Setting all commands to zero...");
        // set all command effort to 0


        RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully deactivated!");
      }



      




};


}  // namespace plato_hardware_interface

#endif  // PLATO_HARDWARE_INTERFACE__PLATO_HPP_
