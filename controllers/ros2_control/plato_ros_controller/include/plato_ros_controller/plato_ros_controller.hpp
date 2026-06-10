#ifndef PLATO_ROS_CONTROLLER__PLATO_ROS_CONTROLLER_HPP_
#define PLATO_ROS_CONTROLLER__PLATO_ROS_CONTROLLER_HPP_

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "aristo_controller/config/aristo_config.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "plato_robot_system/control/plato_control_architecture.hpp"
#include "plato_robot_system/sensor/nari_touch.hpp"
#include "plato_interfaces/msg/impedance_controller_state.hpp"
#include "plato_interfaces/srv/request_state.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sdr_grasp_msgs/msg/tactile.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace aristo_controller::state_machines
{
class GraspTeleopState;
class JointTeleopState;
}  // namespace aristo_controller::state_machines

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
  using GraspTeleopMsg = std_msgs::msg::Float64MultiArray;
  using GraspForceReferenceMsg = std_msgs::msg::Float64;
  using GraspForceReferenceValidMsg = std_msgs::msg::Bool;
  using JointTeleopMsg = std_msgs::msg::Float64MultiArray;
  struct GraspTeleopCommand
  {
    double u{1.0};
    double phi{0.0};
    double desired_force_n{0.0};
    bool has_desired_force{false};
  };
  struct JointTeleopCommand
  {
    Eigen::VectorXd target_jpos;
  };
  using RequestStateSrv = plato_interfaces::srv::RequestState;
  static constexpr std::array<const char *, 2> kTactileFrameNames{
    "thumb_distal_tactile",
    "index_distal_tactile"};

  void read_state_interfaces();
  bool assign_state_interfaces();
  bool assign_command_interfaces();
  TactileSensorVector current_tactile_sensors();
  void tactile_callback(std::size_t index, const TactileMsg::SharedPtr msg);
  void joint_teleop_command_callback(const JointTeleopMsg::SharedPtr msg);
  void sync_joint_teleop_input();
  void grasp_teleop_command_callback(const GraspTeleopMsg::SharedPtr msg);
  void grasp_force_reference_callback(const GraspForceReferenceMsg::SharedPtr msg);
  void grasp_force_reference_valid_callback(const GraspForceReferenceValidMsg::SharedPtr msg);
  void sync_grasp_teleop_input();
  plato_robot_system::sensor::NARITouchSample convert_tactile_msg(
    std::size_t index,
    const TactileMsg & msg) const;
  plato_robot_system::sensor::NARITouchContactState convert_contact_state(int state) const;
  rclcpp::Time tactile_sample_time(const TactileMsg & msg) const;
  const char * tactile_frame_name(std::size_t index) const;
  void publish_controller_state(
    const rclcpp::Time & time,
    const plato_robot_system::RobotCommand & command);
  void request_state_callback(
    std::shared_ptr<RequestStateSrv::Request> request,
    std::shared_ptr<RequestStateSrv::Response> response);
  void apply_pending_state_request();
  bool configure_from_control_config(
    plato_robot_system::ControlArchitecture & architecture,
    plato_robot_system::RobotSystem & robot);
  void configure_identity_joint_mapping(std::size_t num_joints);
  bool configure_model_joint_mapping(const plato_robot_system::RobotSystem & robot);
  plato_robot_system::RobotCommand make_safe_hold_command(
    const plato_robot_system::RobotState & state) const;
  bool write_safe_hold_command(const plato_robot_system::RobotState & state);
  void write_zero_command();
  Eigen::VectorXd map_joint_positions_to_model_q(
    const Eigen::VectorXd & joint_positions,
    const plato_robot_system::RobotSystem & robot) const;
  Eigen::VectorXd map_joint_values_to_model_v(const Eigen::VectorXd & joint_values) const;
  void write_command(const plato_robot_system::RobotCommand & command);

  std::vector<std::string> joint_names_;
  std::vector<std::string> tactile_topics_;
  std::string joint_teleop_command_topic_;
  std::string grasp_teleop_command_topic_;
  std::string grasp_force_reference_topic_;
  std::string grasp_force_reference_valid_topic_;
  std::string control_config_yaml_path_;
  bool fixed_thumb_{false};

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
  std::vector<Eigen::Index> model_q_indices_;
  std::vector<Eigen::Index> model_v_indices_;
  Eigen::VectorXd model_q_;
  Eigen::VectorXd model_qdot_;
  Eigen::VectorXd model_tau_;
  plato_robot_system::RobotCommand filtered_command_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    position_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    velocity_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    effort_state_interfaces_;

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
  aristo_controller::state_machines::JointTeleopState * joint_teleop_state_{nullptr};
  aristo_controller::state_machines::GraspTeleopState * grasp_teleop_state_{nullptr};
  std::atomic<plato_robot_system::StateId> pending_requested_state_id_{-1};
  std::atomic<double> grasp_force_reference_n_{0.0};
  std::atomic<bool> grasp_force_reference_valid_{false};

  realtime_tools::RealtimeBuffer<std::shared_ptr<TactileSensorVector>> rt_tactile_ptr_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<JointTeleopCommand>>
    rt_joint_teleop_command_ptr_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<GraspTeleopCommand>>
    rt_grasp_teleop_command_ptr_;

  rclcpp::Publisher<plato_interfaces::msg::ImpedanceControllerState>::SharedPtr
    controller_state_pub_;
  rclcpp::Service<RequestStateSrv>::SharedPtr request_state_srv_;
  std::vector<rclcpp::Subscription<TactileMsg>::SharedPtr> tactile_subs_;
  rclcpp::Subscription<JointTeleopMsg>::SharedPtr joint_teleop_command_sub_;
  rclcpp::Subscription<GraspTeleopMsg>::SharedPtr grasp_teleop_command_sub_;
  rclcpp::Subscription<GraspForceReferenceMsg>::SharedPtr grasp_force_reference_sub_;
  rclcpp::Subscription<GraspForceReferenceValidMsg>::SharedPtr
    grasp_force_reference_valid_sub_;
  std::mutex nari_touch_mutex_;
  std::vector<plato_robot_system::sensor::NARITouch> nari_touch_;
  std::vector<bool> tactile_stream_seen_;
};

}  // namespace plato_ros_controller

#endif  // PLATO_ROS_CONTROLLER__PLATO_ROS_CONTROLLER_HPP_
