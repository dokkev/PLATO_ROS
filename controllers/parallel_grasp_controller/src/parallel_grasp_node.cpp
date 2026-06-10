#include "parallel_grasp_controller/parallel_grasp_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "plato_interfaces/srv/save_joint_position.hpp"
#include "plato_utils/joint_position_storage.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
constexpr std::array<const char *, 8> kJointNames{
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};
}  // namespace

class ParallelGraspNode : public rclcpp::Node
{
public:
  ParallelGraspNode()
  : Node("parallel_grasp_node"),
    controller_(),
    joint_positions_yaml_path_(this->declare_parameter<std::string>(
        "joint_positions_yaml_path",
        plato::storage::default_joint_position_yaml_path("parallel_grasp_controller")))
  {
    using std::placeholders::_1;

    command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/plato2/parallel_grasp_controller/commands", 10,
      std::bind(&ParallelGraspNode::command_callback, this, _1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/plato2/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&ParallelGraspNode::joint_state_callback, this, _1));

    target_normal_force_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/grasp_force_reference/target_normal_force_n", 10,
      std::bind(&ParallelGraspNode::target_normal_force_callback, this, _1));

    reference_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/grasp_force_reference/reference_valid", 10,
      std::bind(&ParallelGraspNode::reference_valid_callback, this, _1));

    measured_normal_force_min_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/grasp_force_reference/measured_normal_force_min_n", 10,
      std::bind(&ParallelGraspNode::measured_normal_force_min_callback, this, _1));

    measured_normal_force_avg_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/grasp_force_reference/measured_normal_force_avg_n", 10,
      std::bind(&ParallelGraspNode::measured_normal_force_avg_callback, this, _1));

    position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/plato2/joint_impedance_trajectory_controller/commands", 10);

    save_joint_position_srv_ = this->create_service<plato_interfaces::srv::SaveJointPosition>(
      "~/save_joint_position",
      std::bind(
        &ParallelGraspNode::save_joint_position_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "parallel_grasp_node ready");
    RCLCPP_INFO(this->get_logger(), "Expecting commands: [u, phi, f] where:");
    RCLCPP_INFO(this->get_logger(), "  u   = grasp distance [0,1] (0=closed, 1=open)");
    RCLCPP_INFO(this->get_logger(), "  phi = contact angle [0,1] (0=parallel, 1=flexed)");
    RCLCPP_INFO(
      this->get_logger(),
      "  f   = desired force (optional; omitted value uses valid grasp force reference)");
    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing to /grasp_force_reference target/valid/measured force topics");
    RCLCPP_INFO(
      this->get_logger(),
      "Joint position save service available at %s/save_joint_position",
      this->get_fully_qualified_name());
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Build a position vector ordered as joint1..joint8; default to 0.
    std::vector<double> positions(kJointNames.size(), 0.0);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      for (size_t idx = 0; idx < kJointNames.size(); ++idx) {
        if (msg->name[i] == kJointNames[idx]) {
          if (i < msg->position.size()) {
            positions[idx] = msg->position[i];
          }
          break;
        }
      }
    }
    last_positions_ = positions;
  }

  void target_normal_force_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (msg) {
      if (!std::isfinite(msg->data) || msg->data < 0.0) {
        target_normal_force_n_ = 0.0;
        reference_valid_ = false;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Ignoring invalid target_normal_force_n reference");
        return;
      }
      target_normal_force_n_ = msg->data;
      static int callback_counter = 0;
      if (++callback_counter % 100 == 0) {
        std::cout << "[FORCE REFERENCE CALLBACK] target_normal_force_n="
                  << target_normal_force_n_ << std::endl;
      }
    }
  }

  void reference_valid_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg) {
      reference_valid_ = msg->data;
    }
  }

  void measured_normal_force_min_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (msg) {
      measured_normal_force_min_n_ = std::isfinite(msg->data) ? std::max(0.0, msg->data) : 0.0;
      static int callback_counter = 0;
      if (++callback_counter % 100 == 0) {
        std::cout << "[MEASURED FORCE CALLBACK] min_n="
                  << measured_normal_force_min_n_ << std::endl;
      }
    }
  }

  void measured_normal_force_avg_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (msg) {
      measured_normal_force_avg_n_ = std::isfinite(msg->data) ? std::max(0.0, msg->data) : 0.0;
      static int callback_counter = 0;
      if (++callback_counter % 100 == 0) {
        std::cout << "[MEASURED FORCE CALLBACK] avg_n="
                  << measured_normal_force_avg_n_ << std::endl;
      }
    }
  }

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Parse commands: [u_d, u_phi, f_d]
    std::array<double, 3> commands = {0.5, 0.0, 0.0};  // Default: mid-open, parallel, no force

    if (msg && !msg->data.empty()) {
      commands[0] = msg->data[0];  // u_d: grasp distance
      if (msg->data.size() > 1) commands[1] = msg->data[1];  // u_phi: contact angle
      if (msg->data.size() > 2) {
        commands[2] = msg->data[2];  // f_d: desired force (from command)
      } else {
        // If no force command is provided, use the reference generator only
        // when it reports that the scalar force reference is valid.
        commands[2] = reference_valid_ ? target_normal_force_n_ : 0.0;
      }
    }

    // Debug: print inputs every 100 cycles
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0) {
      std::cout << "[NODE] commands=[" << commands[0] << ", " << commands[1] << ", " << commands[2]
                << "] reference_valid=" << reference_valid_
                << " target_normal_force_n=" << target_normal_force_n_
                << " measured_min_n=" << measured_normal_force_min_n_
                << " measured_avg_n=" << measured_normal_force_avg_n_ << std::endl;
    }

    const auto & positions = last_positions_.empty() ? zero_positions_ : last_positions_;
    controller_.update(commands, positions, measured_normal_force_min_n_);
    const auto & target = controller_.get_commands();

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = target;
    position_pub_->publish(cmd_msg);
  }

  void save_joint_position_callback(
    const std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Request> request,
    std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Response> response)
  {
    response->saved_path = joint_positions_yaml_path_;

    if (last_positions_.size() != kJointNames.size()) {
      response->success = false;
      response->message = "Current joint positions have not been received yet.";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    const std::vector<std::string> joint_names(kJointNames.begin(), kJointNames.end());
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
    if (success) {
      response->message = "Saved current joint positions to YAML.";
      RCLCPP_INFO(
        this->get_logger(),
        "Saved joint positions as '%s' to %s",
        response->saved_name.c_str(),
        response->saved_path.c_str());
    } else {
      response->message = error_message;
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to save joint positions to %s: %s",
        response->saved_path.c_str(),
        response->message.c_str());
    }
  }

  ParallelGraspController controller_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_normal_force_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reference_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measured_normal_force_min_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measured_normal_force_avg_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
  rclcpp::Service<plato_interfaces::srv::SaveJointPosition>::SharedPtr save_joint_position_srv_;

  std::vector<double> last_positions_;
  const std::vector<double> zero_positions_ = std::vector<double>(kJointNames.size(), 0.0);
  double target_normal_force_n_ = 0.0;
  double measured_normal_force_min_n_ = 0.0;
  double measured_normal_force_avg_n_ = 0.0;
  bool reference_valid_ = false;
  std::string joint_positions_yaml_path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParallelGraspNode>());
  rclcpp::shutdown();
  return 0;
}
