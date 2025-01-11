#ifndef JOINT_IMPEDANCE_CONTROLLER__JOINT_IMPEDANCE_CONTROLLER_HPP_
#define JOINT_IMPEDANCE_CONTROLLER__JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "joint_impedance_controller_parameters.hpp"

namespace joint_impedance_controller
{

using CmdType = std_msgs::msg::Float64MultiArray;

/**
 * \brief Joint impedance controller for a set of joints.
 *
 * This class implements impedance control for a set of joints.
 *
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 */
class JointImpedanceController : public controller_interface::ControllerInterface
{
public:
  JointImpedanceController();

  controller_interface::CallbackReturn on_init() override;
  
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
    
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
    
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  
  // Real-time buffer for commands
  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  // Parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

}  // namespace joint_impedance_controller

#endif  // JOINT_IMPEDANCE_CONTROLLER__JOINT_IMPEDANCE_CONTROLLER_HPP_