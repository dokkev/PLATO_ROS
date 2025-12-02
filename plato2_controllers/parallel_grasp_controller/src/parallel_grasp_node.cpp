#include "parallel_grasp_controller/parallel_grasp_controller.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
    controller_()
  {
    using std::placeholders::_1;

    command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/plato2/parallel_grasp_controller/commands", 10,
      std::bind(&ParallelGraspNode::command_callback, this, _1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/plato2/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&ParallelGraspNode::joint_state_callback, this, _1));

    position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/plato2/plato2_position_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(), "parallel_grasp_node ready");
    RCLCPP_INFO(this->get_logger(), "Expecting commands: [u, phi] (force ignored here):");
    RCLCPP_INFO(this->get_logger(), "  u   = grasp distance [0,1] (0=closed, 1=open)");
    RCLCPP_INFO(this->get_logger(), "  phi = contact angle [0,1] (0=parallel, 1=flexed)");
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

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Parse commands: [u_d, u_phi]
    std::array<double, 3> commands = {0.5, 0.0, 0.0};  // Default: mid-open, parallel

    if (msg && !msg->data.empty()) {
      commands[0] = msg->data[0];  // u_d: grasp distance
      if (msg->data.size() > 1) commands[1] = msg->data[1];  // u_phi: contact angle
      // Force input ignored; commands[2] stays 0
    }

    const auto & positions = last_positions_.empty() ? zero_positions_ : last_positions_;
    controller_.update(commands, positions);
    const auto & target = controller_.get_commands();

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = target;
    position_pub_->publish(cmd_msg);
  }

  ParallelGraspController controller_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;

  std::vector<double> last_positions_;
  const std::vector<double> zero_positions_ = std::vector<double>(kJointNames.size(), 0.0);
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParallelGraspNode>());
  rclcpp::shutdown();
  return 0;
}
