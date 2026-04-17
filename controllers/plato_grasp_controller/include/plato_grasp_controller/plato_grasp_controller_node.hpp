#ifndef PLATO_GRASP_CONTROLLER__PLATO_GRASP_CONTROLLER_NODE_HPP_
#define PLATO_GRASP_CONTROLLER__PLATO_GRASP_CONTROLLER_NODE_HPP_

#include <optional>
#include <memory>
#include <string>

#include "plato_grasp_controller/plato_grasp_planner.hpp"
#include "plato_grasp_controller/plato_grasp_impedance_handler.hpp"
#include "plato_grasp_controller/plato_grasp_task_runner.hpp"
#include "plato_interfaces/msg/impedance_commands.hpp"
#include "plato_interfaces/srv/save_joint_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace plato_grasp_controller
{

class PlatoGraspControllerNode : public rclcpp::Node
{
public:
  explicit PlatoGraspControllerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void handle_motion_state_command(const std_msgs::msg::String::SharedPtr msg);
  void handle_task_command(const std_msgs::msg::String::SharedPtr msg);
  void handle_task_update();
  void handle_save_joint_position(
    const std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Request> request,
    std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Response> response);
  bool publish_motion_plan(
    const std::string & pos_preset_name,
    bool use_current_position,
    double impedance_level,
    std::string * error_out);
  bool publish_grasp(
    const GraspPlanConfig & grasp_plan,
    double impedance_level,
    std::string * error_out);

  int joint_count_;
  std::string joint_state_topic_;
  std::string motion_state_topic_;
  std::string task_topic_;
  std::string trajectory_goal_topic_;
  std::string saved_joint_positions_yaml_path_;
  std::string plan_config_yaml_path_;
  std::string impedance_preset_yaml_path_;
  double manual_motion_impedance_level_;
  double task_update_rate_hz_;
  std::unique_ptr<PlatoGraspPlanner> planner_;
  std::unique_ptr<PlatoGraspImpedanceHandler> impedance_handler_;
  std::unique_ptr<PlatoGraspTaskRunner> task_runner_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_state_sub_;
  rclcpp::Publisher<plato_interfaces::msg::ImpedanceCommands>::SharedPtr trajectory_goal_pub_;
  rclcpp::Service<plato_interfaces::srv::SaveJointPosition>::SharedPtr save_joint_position_srv_;
  rclcpp::TimerBase::SharedPtr task_update_timer_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_task_update_time_{0, 0, RCL_STEADY_TIME};
  bool has_last_task_update_time_{false};
};

}  // namespace plato_grasp_controller

#endif  // PLATO_GRASP_CONTROLLER__PLATO_GRASP_CONTROLLER_NODE_HPP_
