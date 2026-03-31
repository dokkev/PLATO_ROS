#include <memory>
#include <string>
#include <utility>

#include "plato_grasp_controller/plato_grasp_planner.hpp"
#include "plato_interfaces/srv/save_joint_position.hpp"
#include "plato_utils/joint_position_storage.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace plato_grasp_controller
{

class JointPositionSaverNode : public rclcpp::Node
{
public:
  explicit JointPositionSaverNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("joint_position_saver", options),
    joint_state_topic_(this->declare_parameter<std::string>(
        "joint_state_topic",
        "/plato2/joint_states")),
    planner_(std::make_unique<PlatoGraspPlanner>(
        this->declare_parameter<int>("joint_count", 8),
        this->declare_parameter<std::string>(
          "saved_joint_positions_yaml_path",
          plato::storage::default_joint_position_yaml_path("plato_grasp_controller"))))
  {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      10,
      std::bind(&JointPositionSaverNode::handle_joint_state, this, std::placeholders::_1));

    save_joint_position_srv_ =
      this->create_service<plato_interfaces::srv::SaveJointPosition>(
      "~/save_joint_position",
      std::bind(
        &JointPositionSaverNode::handle_save_joint_position,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Joint position saver listening on %s and serving %s",
      joint_state_topic_.c_str(),
      save_joint_position_srv_->get_service_name());
    RCLCPP_INFO(
      this->get_logger(),
      "Saving joint positions to %s",
      planner_->saved_joint_positions_yaml_path().c_str());
  }

private:
  void handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    planner_->update_joint_state(msg->name, msg->position);
  }

  void handle_save_joint_position(
    const std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Request> request,
    std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Response> response)
  {
    response->saved_path = planner_->saved_joint_positions_yaml_path();

    std::string saved_name;
    std::string error_message;
    const bool success = planner_->save_current_joint_position(
      request->name,
      &saved_name,
      &error_message);

    response->success = success;
    response->saved_name = saved_name;
    if (success) {
      response->message = "Saved current joint positions to YAML.";
      RCLCPP_INFO(
        this->get_logger(),
        "Saved joint position '%s' to %s",
        saved_name.c_str(),
        response->saved_path.c_str());
      return;
    }

    response->message = error_message;
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
  }

  std::string joint_state_topic_;
  std::unique_ptr<PlatoGraspPlanner> planner_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<plato_interfaces::srv::SaveJointPosition>::SharedPtr save_joint_position_srv_;
};

}  // namespace plato_grasp_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plato_grasp_controller::JointPositionSaverNode>());
  rclcpp::shutdown();
  return 0;
}
