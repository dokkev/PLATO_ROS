#ifndef JOINT_IMPEDANCE_CONTROLLER__JOINT_IMPEDANCE_CONTROLLER_HPP_
#define JOINT_IMPEDANCE_CONTROLLER__JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <vector>
#include <string>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace joint_impedance_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;

class JointImpedanceController : public controller_interface::ControllerInterface
{
public:
  JointImpedanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<std::string> state_interface_types_;
  std::vector<std::string> command_interface_types_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;
};

}  // namespace joint_impedance_controller

#endif  // JOINT_IMPEDANCE_CONTROLLER__JOINT_IMPEDANCE_CONTROLLER_HPP_
