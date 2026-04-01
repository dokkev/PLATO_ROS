#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plato_interfaces/srv/save_joint_position.hpp"
#include "plato_utils/joint_position_storage.hpp"
#include "plato_utils/joint_state_ordering.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace plato
{
namespace tools
{

class JointPositionSaverNode : public rclcpp::Node
{
public:
  explicit JointPositionSaverNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("joint_position_saver", options),
    joint_count_(plato::joint_state::clamp_joint_count(
        this->declare_parameter<int>("joint_count", 8))),
    joint_state_topic_(this->declare_parameter<std::string>(
        "joint_state_topic",
        "/plato2/joint_states")),
    saved_joint_positions_yaml_path_(this->declare_parameter<std::string>(
        "saved_joint_positions_yaml_path",
        plato::storage::default_joint_position_yaml_path("plato_grasp_controller"))),
    last_positions_(static_cast<size_t>(joint_count_), 0.0)
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
      saved_joint_positions_yaml_path_.c_str());
  }

private:
  void handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::vector<double> ordered_positions(static_cast<size_t>(joint_count_), 0.0);
    const auto valid_size = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < valid_size; ++i) {
      const int joint_index = plato::joint_state::joint_index_for_name(msg->name[i], joint_count_);
      if (joint_index >= 0) {
        ordered_positions[static_cast<size_t>(joint_index)] = msg->position[i];
      }
    }

    last_positions_ = std::move(ordered_positions);
    has_joint_state_ = true;
  }

  void handle_save_joint_position(
    const std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Request> request,
    std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Response> response)
  {
    response->saved_path = saved_joint_positions_yaml_path_;

    if (!has_joint_state_) {
      response->success = false;
      response->message = "Current joint positions have not been received yet.";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    std::string saved_name;
    std::string error_message;
    const bool success = plato::storage::save_joint_position_yaml(
      saved_joint_positions_yaml_path_,
      request->name,
      plato::joint_state::ordered_joint_names(joint_count_),
      last_positions_,
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

  int joint_count_;
  std::string joint_state_topic_;
  std::string saved_joint_positions_yaml_path_;
  std::vector<double> last_positions_;
  bool has_joint_state_ = false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<plato_interfaces::srv::SaveJointPosition>::SharedPtr save_joint_position_srv_;
};

}  // namespace tools
}  // namespace plato

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plato::tools::JointPositionSaverNode>());
  rclcpp::shutdown();
  return 0;
}
