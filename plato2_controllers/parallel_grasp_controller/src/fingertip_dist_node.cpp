#include "parallel_grasp_controller/fingertip_dist_controller.hpp"

#include <array>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

namespace
{
constexpr std::array<const char *, 8> kJointNames{
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};
}  // namespace

class FingertipDistNode : public rclcpp::Node
{
public:
  FingertipDistNode()
  : Node("fingertip_dist_node"),
    controller_()
  {
    using std::placeholders::_1;

    const int initial_thumb_state = this->declare_parameter<int>("initial_thumb_state", 1);
    const double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 50.0);
    state_transition_duration_ = this->declare_parameter<double>("state_transition_duration", 0.8);
    try {
      controller_.set_state(initial_thumb_state);
    } catch (const std::out_of_range& ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid initial_thumb_state %d: %s. Falling back to state 1.",
        initial_thumb_state,
        ex.what());
      controller_.set_state(1);
    }

    command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/plato2/fingertip_dist_cmd", 10,
      std::bind(&FingertipDistNode::command_callback, this, _1));

    thumb_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/plato2/thumb_state", 10,
      std::bind(&FingertipDistNode::state_callback, this, _1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/plato2/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&FingertipDistNode::joint_state_callback, this, _1));

    position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/plato2/joint_position_controller/commands", 10);

    if (publish_rate_hz > 0.0) {
      publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz),
        std::bind(&FingertipDistNode::timer_callback, this));
    }

    RCLCPP_INFO(this->get_logger(), "fingertip_dist_node ready");
    RCLCPP_INFO(
      this->get_logger(),
      "Expecting /plato2/fingertip_dist_cmd: [t_i_dist, t_m_dist, T_IP, I_PIP, M_PIP]");
    RCLCPP_INFO(this->get_logger(), "T_IP is ignored; thumb posture is controlled by /plato2/thumb_state");
    RCLCPP_INFO(this->get_logger(), "All command values are normalized to [0, 1]");
    RCLCPP_INFO(this->get_logger(), "State subscriber: /plato2/thumb_state (0=poking, 1=index pinch, 2=middle pinch)");
    RCLCPP_INFO(this->get_logger(), "Initial state: %d", initial_thumb_state);
    RCLCPP_INFO(this->get_logger(), "Publishing latest target at %.1f Hz", publish_rate_hz);
    RCLCPP_INFO(this->get_logger(), "State transition duration: %.2f s", state_transition_duration_);
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
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

  void state_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    try {
      controller_.set_state(msg->data);
      RCLCPP_INFO(this->get_logger(), "Fingertip distance state set to %d", msg->data);
      has_target_ = true;
      start_state_transition();
    } catch (const std::out_of_range& ex) {
      RCLCPP_WARN(this->get_logger(), "Ignoring invalid state %d: %s", msg->data, ex.what());
    }
  }

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::array<double, 5> commands = {1.0, 1.0, 0.5, 0.0, 0.0};
    if (msg) {
      const size_t count = std::min(commands.size(), msg->data.size());
      for (size_t i = 0; i < count; ++i) {
        commands[i] = msg->data[i];
      }
    }

    last_commands_ = commands;
    has_target_ = true;
    if (trajectory_active_) {
      trajectory_goal_ = compute_current_target();
    } else {
      publish_current_target();
    }
  }

  void timer_callback()
  {
    if (!has_target_) {
      return;
    }
    if (trajectory_active_) {
      publish_trajectory_sample();
      return;
    }
    publish_current_target();
  }

  std::vector<double> compute_current_target()
  {
    const auto& positions = last_positions_.empty() ? zero_positions_ : last_positions_;
    controller_.update(last_commands_, positions);
    return controller_.get_commands();
  }

  void publish_command(const std::vector<double>& command)
  {
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = command;
    position_pub_->publish(cmd_msg);
    last_published_target_ = command;
  }

  void publish_current_target()
  {
    publish_command(compute_current_target());
  }

  void start_state_transition()
  {
    trajectory_start_ = last_published_target_;
    if (trajectory_start_.empty()) {
      trajectory_start_ = last_positions_.empty() ? zero_positions_ : last_positions_;
    }

    trajectory_goal_ = compute_current_target();
    if (state_transition_duration_ <= 0.0) {
      trajectory_active_ = false;
      publish_command(trajectory_goal_);
      return;
    }

    trajectory_start_time_ = std::chrono::steady_clock::now();
    trajectory_active_ = true;
    publish_trajectory_sample();
  }

  void publish_trajectory_sample()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - trajectory_start_time_).count();
    const double u = std::clamp(elapsed / state_transition_duration_, 0.0, 1.0);
    const double s = u * u * (3.0 - 2.0 * u);

    const size_t n = std::max(trajectory_start_.size(), trajectory_goal_.size());
    std::vector<double> sample(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
      const double start = i < trajectory_start_.size() ? trajectory_start_[i] : 0.0;
      const double goal = i < trajectory_goal_.size() ? trajectory_goal_[i] : start;
      sample[i] = start + (goal - start) * s;
    }

    publish_command(sample);
    if (u >= 1.0) {
      trajectory_active_ = false;
      publish_command(trajectory_goal_);
    }
  }

  FingertipDistController controller_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr thumb_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::array<double, 5> last_commands_ = {1.0, 1.0, 0.5, 0.0, 0.0};
  std::vector<double> last_positions_;
  std::vector<double> last_published_target_;
  std::vector<double> trajectory_start_;
  std::vector<double> trajectory_goal_;
  const std::vector<double> zero_positions_ = std::vector<double>(kJointNames.size(), 0.0);
  std::chrono::steady_clock::time_point trajectory_start_time_;
  double state_transition_duration_ = 0.8;
  bool has_target_ = false;
  bool trajectory_active_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingertipDistNode>());
  rclcpp::shutdown();
  return 0;
}
