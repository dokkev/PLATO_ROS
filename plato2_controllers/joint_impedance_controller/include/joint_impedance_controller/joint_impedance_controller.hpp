#ifndef JOINT_IMPEDANCE_CONTROLLER__JOINT_IMPEDANCE_CONTROLLER_HPP_
#define JOINT_IMPEDANCE_CONTROLLER__JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "plato2_interfaces/msg/impedance_commands.hpp"
#include "plato2_interfaces/msg/impedance_controller_state.hpp"

// Include generated parameter header
#include <joint_impedance_controller/joint_impedance_controller_parameters.hpp>

namespace joint_impedance_controller
{

using CmdType = plato2_interfaces::msg::ImpedanceCommands;

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

  void publish_state(const rclcpp::Time & time, const CmdType& command);

protected:
  std::vector<std::string> joint_names_;

  // State interfaces
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;

  // Command interfaces - to be passed to hardware
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_position_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_velocity_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_effort_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_stiffness_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_damping_command_interfaces_;

  // Real-time buffer for commands
  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  // State Publisher
  using ControllerStateMsg = plato2_interfaces::msg::ImpedanceControllerState;
  using StatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  using StatePublisherPtr = std::unique_ptr<StatePublisher>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr publisher_;
  StatePublisherPtr state_publisher_;

  //QoS
  // Link Parameters for finger jacobian

  const double L1 = 0.06;
  const double L2 = 0.06;

  struct Jacobian{
    double j11, j12;
    double j21, j22;
  };

  Jacobian get_J(double theta1, double theta2) const;
  Jacobian get_Jinv(double theta1, double theta2) const;



  // jacobian matrix


  

  // Parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

}  // namespace joint_impedance_controller

#endif  // JOINT_IMPEDANCE_CONTROLLER__JOINT_IMPEDANCE_CONTROLLER_HPP_