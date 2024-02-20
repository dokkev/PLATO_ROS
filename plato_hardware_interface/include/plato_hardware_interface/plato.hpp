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

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/joint_state.h>

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
      std::vector<double> joint_position_command_;
      std::vector<double> joint_effort_command_;
      
      std::vector<double> joint_position_;
      std::vector<double> joint_velocity_;
      std::vector<double> joint_effort_;
      std::vector<double> ft_states_;

      // CAN Subscriber
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
      // rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_state_sub_; //TODO: Read Force Torque Sensor Data
      // CAN publisher
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
      rclcpp::Node::SharedPtr node_;
      sensor_msgs::msg::JointState latest_joint_state_;

      /// Use standard interfaces for joints because they are relevant for dynamic behavior
      std::array<std::string, 4> standard_interfaces_ = { hardware_interface::HW_IF_POSITION,
                                                          hardware_interface::HW_IF_VELOCITY,
                                                          hardware_interface::HW_IF_ACCELERATION,
                                                          hardware_interface::HW_IF_EFFORT };


};


}  // namespace plato_hardware_interface

#endif  // PLATO_HARDWARE_INTERFACE__PLATO_HPP_
