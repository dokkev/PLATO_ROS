#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "aristo_grasp_controller/aristo_grasp_controller.hpp"
#include "aristo_grasp_controller/aristo_kinematics.hpp"
#include "plato_interfaces/srv/save_joint_position.hpp"
#include "plato_utils/joint_position_storage.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
constexpr std::array<const char *, 8> jointNames{
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};

std::string default_urdf_path()
{
  return ament_index_cpp::get_package_share_directory("plato_description") +
         "/urdf/generated/aristo.urdf";
}
}  // namespace

class AristoGraspNode : public rclcpp::Node
{
public:
  AristoGraspNode()
  : Node("aristo_grasp_node"),
    urdf_path_(declare_parameter<std::string>("urdf_path", default_urdf_path())),
    command_topic_(declare_parameter<std::string>(
        "command_topic", "/plato2/aristo_grasp_controller/commands")),
    joint_state_topic_(declare_parameter<std::string>("joint_state_topic", "/plato2/joint_states")),
    minimal_force_topic_(declare_parameter<std::string>(
        "minimal_force_topic", "/object_state/minimal_force")),
    measured_force_topic_(declare_parameter<std::string>(
        "measured_force_topic", "/object_state/measured_force")),
    output_topic_(declare_parameter<std::string>(
        "output_topic", "/plato2/joint_impedance_trajectory_controller/commands")),
    joint_positions_yaml_path_(declare_parameter<std::string>(
        "joint_positions_yaml_path",
        plato::storage::default_joint_position_yaml_path("aristo_grasp_controller")))
  {
    using std::placeholders::_1;

    auto kinematics = std::make_shared<aristo_grasp_controller::AristoKinematics>(urdf_path_);
    controller_ = std::make_unique<aristo_grasp_controller::AristoGraspController>(kinematics);

    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_, 10, std::bind(&AristoGraspNode::command_callback, this, _1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AristoGraspNode::joint_state_callback, this, _1));

    minimal_force_sub_ = create_subscription<std_msgs::msg::Float32>(
      minimal_force_topic_, 10, std::bind(&AristoGraspNode::minimal_force_callback, this, _1));

    measured_force_sub_ = create_subscription<std_msgs::msg::Float64>(
      measured_force_topic_, 10, std::bind(&AristoGraspNode::measured_force_callback, this, _1));

    position_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(output_topic_, 10);

    save_joint_position_srv_ = create_service<plato_interfaces::srv::SaveJointPosition>(
      "~/save_joint_position",
      std::bind(
        &AristoGraspNode::save_joint_position_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "aristo_grasp_node ready");
    RCLCPP_INFO(get_logger(), "Loaded Pinocchio model from %s", urdf_path_.c_str());
    RCLCPP_INFO(get_logger(), "Command topic: %s", command_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output topic: %s", output_topic_.c_str());
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::vector<double> positions(jointNames.size(), 0.0);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      for (size_t joint_idx = 0; joint_idx < jointNames.size(); ++joint_idx) {
        if (msg->name[i] == jointNames[joint_idx]) {
          if (i < msg->position.size()) {
            positions[joint_idx] = msg->position[i];
          }
          break;
        }
      }
    }
    last_positions_ = positions;
  }

  void minimal_force_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (msg) {
      minimal_force_ = static_cast<double>(msg->data);
    }
  }

  void measured_force_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (msg) {
      measured_force_ = msg->data;
    }
  }

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::array<double, 3> commands{0.5, 0.0, 0.0};
    if (msg && !msg->data.empty()) {
      commands[0] = msg->data[0];
      if (msg->data.size() > 1) {
        commands[1] = msg->data[1];
      }
      commands[2] = msg->data.size() > 2 ? msg->data[2] : minimal_force_;
    }

    const auto & positions = last_positions_.empty() ? zero_positions_ : last_positions_;
    controller_->update(commands, positions, measured_force_);

    std_msgs::msg::Float64MultiArray command_msg;
    command_msg.data = controller_->commands();
    position_pub_->publish(command_msg);
  }

  void save_joint_position_callback(
    const std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Request> request,
    std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Response> response)
  {
    response->saved_path = joint_positions_yaml_path_;

    if (last_positions_.size() != jointNames.size()) {
      response->success = false;
      response->message = "Current joint positions have not been received yet.";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    const std::vector<std::string> joint_names(jointNames.begin(), jointNames.end());
    std::string saved_name;
    std::string error_message;
    const bool success = plato::storage::save_joint_position_yaml(
      joint_positions_yaml_path_,
      request->name,
      joint_names,
      last_positions_,
      &saved_name,
      &error_message);

    response->success = success;
    response->saved_name = saved_name;
    response->message = success ? "Saved current joint positions to YAML." : error_message;

    if (success) {
      RCLCPP_INFO(
        get_logger(),
        "Saved joint positions as '%s' to %s",
        response->saved_name.c_str(),
        response->saved_path.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to save joint positions to %s: %s",
        response->saved_path.c_str(),
        response->message.c_str());
    }
  }

  std::unique_ptr<aristo_grasp_controller::AristoGraspController> controller_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr minimal_force_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measured_force_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
  rclcpp::Service<plato_interfaces::srv::SaveJointPosition>::SharedPtr save_joint_position_srv_;

  std::vector<double> last_positions_;
  const std::vector<double> zero_positions_ = std::vector<double>(jointNames.size(), 0.0);
  double minimal_force_ = 0.0;
  double measured_force_ = 0.0;
  std::string urdf_path_;
  std::string command_topic_;
  std::string joint_state_topic_;
  std::string minimal_force_topic_;
  std::string measured_force_topic_;
  std::string output_topic_;
  std::string joint_positions_yaml_path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AristoGraspNode>());
  rclcpp::shutdown();
  return 0;
}
