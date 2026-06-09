#ifndef PLATO_ROS_CONTROLLER__PLATO_ROS_CONTROLLER_HPP_
#define PLATO_ROS_CONTROLLER__PLATO_ROS_CONTROLLER_HPP_

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "plato_robot_system/control/plato_control_architecture.hpp"
#include "plato_robot_system/sensor/nari_touch.hpp"
#include "plato_interfaces/msg/impedance_controller_state.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sdr_grasp_msgs/msg/tactile.hpp"

namespace plato_ros_controller
{

class PlatoRosController : public controller_interface::ControllerInterface
{
public:
  PlatoRosController();

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
  virtual controller_interface::CallbackReturn configure_control_architecture(
    plato_robot_system::ControlArchitecture & architecture);
  virtual void register_robot_states(
    plato_robot_system::ControlArchitecture & architecture,
    plato_robot_system::RobotSystem & robot);
  virtual void configure_initial_state(
    plato_robot_system::ControlArchitecture & architecture);
  virtual void filter_command(plato_robot_system::RobotCommand * command) const;

private:
  using TactileMsg = sdr_grasp_msgs::msg::Tactile;
  using TactileSensorVector = plato_robot_system::TactileSensorVector;
  static constexpr std::array<const char *, 2> kTactileFrameNames{
    "thumb_distal_tactile",
    "index_distal_tactile"};

  void read_state_interfaces();
  bool assign_command_interfaces();
  TactileSensorVector current_tactile_sensors();
  void tactile_callback(std::size_t index, const TactileMsg::SharedPtr msg);
  plato_robot_system::sensor::NARITouchSample convert_tactile_msg(
    std::size_t index,
    const TactileMsg & msg) const;
  plato_robot_system::sensor::NARITouchContactState convert_contact_state(int state) const;
  rclcpp::Time tactile_sample_time(const TactileMsg & msg) const;
  const char * tactile_frame_name(std::size_t index) const;
  void publish_controller_state(
    const rclcpp::Time & time,
    const plato_robot_system::RobotCommand & command);
  void write_command(const plato_robot_system::RobotCommand & command);

  std::vector<std::string> joint_names_;
  std::vector<std::string> tactile_topics_;
  bool compute_impedance_torque_{false};

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    position_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    velocity_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    effort_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    stiffness_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    damping_command_interfaces_;

  std::shared_ptr<plato_robot_system::RobotSystem> robot_;
  plato_robot_system::ControlArchitecture control_architecture_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<TactileSensorVector>> rt_tactile_ptr_;

  rclcpp::Publisher<plato_interfaces::msg::ImpedanceControllerState>::SharedPtr
    controller_state_pub_;
  std::vector<rclcpp::Subscription<TactileMsg>::SharedPtr> tactile_subs_;
  std::vector<plato_robot_system::sensor::NARITouch> nari_touch_;
  std::vector<bool> tactile_stream_seen_;
};

}  // namespace plato_ros_controller

#endif  // PLATO_ROS_CONTROLLER__PLATO_ROS_CONTROLLER_HPP_
