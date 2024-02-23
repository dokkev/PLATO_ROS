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
#include "plato_hardware_interface/plato_common.hpp"


namespace plato_hardware_interface
{



void PLATOHardware::set_zero_command(std::vector<double>& command){
  for (size_t i = 0; i < command.size(); ++i) {
    command[i] = 0.0;
  }
}

void PLATOHardware::set_zero_states(std::vector<double>& joint_states){
  for (size_t i = 0; i < joint_states.size(); ++i) {
    joint_states[i] = 0.0;
  }
}

void PLATOHardware::set_can_id_map(){
  rx_to_joint_ = {
    {MOTOR_0_CAN_RX_ID, 0}, {MOTOR_1_CAN_RX_ID, 1}, {MOTOR_2_CAN_RX_ID, 2}, 
    {MOTOR_3_CAN_RX_ID, 3}, {MOTOR_4_CAN_RX_ID, 4}, {MOTOR_5_CAN_RX_ID, 5}, 
    {MOTOR_6_CAN_RX_ID, 6}, {MOTOR_7_CAN_RX_ID, 7}, {MOTOR_8_CAN_RX_ID, 8}
  };

  joint_to_tx_ = {
    {0, MOTOR_0_CAN_TX_ID}, {1, MOTOR_1_CAN_TX_ID}, {2, MOTOR_2_CAN_TX_ID}, 
    {3, MOTOR_3_CAN_TX_ID}, {4, MOTOR_4_CAN_TX_ID}, {5, MOTOR_5_CAN_TX_ID}, 
    {6, MOTOR_6_CAN_TX_ID}, {7, MOTOR_7_CAN_TX_ID}, {8, MOTOR_8_CAN_TX_ID}
  };

}

void PLATOHardware::stop(){
  set_zero_command(motor_effort_commands_);
  RCLCPP_INFO(
  rclcpp::get_logger("PLATOHardware"), "Deactivating ...Setting all commands to zero...");
  // set all command effort to 0



  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully Stopped!");
}

void PLATOHardware::can_frame_callback(const can_msgs::msg::Frame::SharedPtr msg){
  // convert the can frame to motor position with thread safety
  if (rx_to_joint_.find(msg->id) != rx_to_joint_.end()){
    motor_position_states_[rx_to_joint_[msg->id]] = *reinterpret_cast<double*>(msg->data.data());
  }
}


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

  // Initialize the Joint (Motor) to CAN ID mapping
  PLATOHardware::set_can_id_map();

  // Node
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=plato_hardware_interface"+ info_.name});


  // Node
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=plato_hardware_interface"+ info_.name});

  node_ = rclcpp::Node::make_shared("_", options);

  can_publisher_ = node_->create_publisher<can_msgs::msg::Frame>("to_can_bus", rclcpp::QoS(10));

  can_subscriber_ = node_->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", rclcpp::QoS(10),
    std::bind(&PLATOHardware::can_frame_callback, this, std::placeholders::_1));


  rclcpp::on_shutdown(std::bind(&PLATOHardware::stop, this));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PLATOHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Configuring ...setting all joint state to 0..");
  PLATOHardware::set_zero_states(joint_position_states_);
  PLATOHardware::set_zero_states(joint_velocity_states_);
  PLATOHardware::set_zero_states(joint_effort_states_);

  PLATOHardware::set_zero_command(joint_position_commands_);
  
  RCLCPP_INFO(rclcpp::get_logger("PLATOHardware"), "Successfully configured!");

    // init CAN
  // socket_can_.init();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PLATOHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

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
  // command_interfaces.reserve(info_.joints.size());
  // effort_command_interface_names_.reserve(info_.joints.size());

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

  RCLCPP_INFO(
    rclcpp::get_logger("PLATOHardware"), "Activating ...please wait...");





    // TODO: Add Gravity Compensation

    // send the zero effort command to the motor
    PLATOHardware::set_zero_command(joint_effort_commands_);


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

  if (rclcpp::ok()){
    rclcpp::spin_some(node_);
  }

  // thread safe method to convert motor position to joint position



  motor_direction_.convert_motor_to_joint_position(motor_position_states_, joint_position_states_);



  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PLATOHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  // Convert the joint position commands to motor position commands with direction
  motor_direction_.convert_joint_to_motor_effort(joint_effort_commands_, motor_effort_commands_);

  // Send the motor position commands over CAN
  can_msgs::msg::Frame frame;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = 8;

  // use the joint_id_to_can_tx_id mapping
  for (size_t i = 0; i < motor_effort_commands_.size(); ++i) {
    std::memcpy(frame.data.data(), &motor_effort_commands_[i], sizeof(double));
    frame.id = joint_to_tx_[i];

    if (rclcpp::ok()){
      can_publisher_->publish(frame);
    }

  }
  return hardware_interface::return_type::OK;

} 

#include "pluginlib/class_list_macros.hpp"
}  // namespace plato_hardware_interface
PLUGINLIB_EXPORT_CLASS(plato_hardware_interface::PLATOHardware, hardware_interface::SystemInterface)
