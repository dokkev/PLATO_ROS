#include <memory>
#include <vector>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "plato_interfaces/msg/impedance_commands.hpp"
#include "joint_impedance_controller/impedance_trajectory_controller.hpp"
#include "joint_impedance_controller/impedance_gain_handler.hpp"

class ImpedanceTrajectoryControllerNode : public rclcpp::Node {
public:
  ImpedanceTrajectoryControllerNode()
    : Node("impedance_trajectory_controller_node"),
      controller_(8),
      gain_handler_(*this),
      steady_clock_(RCL_STEADY_TIME) {
    const auto position_topic = this->declare_parameter<std::string>(
        "position_command_topic", "/plato2/joint_impedance_trajectory_controller/commands");
    const auto joint_state_topic = this->declare_parameter<std::string>(
        "joint_state_topic", "/plato2/joint_states");
    const auto impedance_topic = this->declare_parameter<std::string>(
        "impedance_command_topic", "/plato2/joint_impedance_controller/commands");
    default_goal_duration_sec_ = this->declare_parameter<double>("default_goal_duration_sec", 0.25);
    const double control_rate_hz = this->declare_parameter<double>("control_rate_hz", 100.0);

    position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        position_topic, 10,
        std::bind(&ImpedanceTrajectoryControllerNode::positionCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic, rclcpp::SensorDataQoS(),
        std::bind(&ImpedanceTrajectoryControllerNode::jointStateCallback, this, std::placeholders::_1));

    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    impedance_pub_ = this->create_publisher<plato_interfaces::msg::ImpedanceCommands>(
        impedance_topic, qos_profile);

    const double safe_rate_hz = std::max(control_rate_hz, 1.0);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / safe_rate_hz));
    update_timer_ = this->create_wall_timer(
        period, std::bind(&ImpedanceTrajectoryControllerNode::updateLoop, this));

    RCLCPP_INFO(this->get_logger(),
                "impedance_trajectory_controller_node started (rate=%.1fHz, goal_duration=%.3fs)",
                safe_rate_hz, default_goal_duration_sec_);
  }

private:
  void positionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!msg) {
      return;
    }
    if (msg->data.empty()) {
      controller_.holdPosition();
      return;
    }
    controller_.setGoal(msg->data, default_goal_duration_sec_);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg) {
      return;
    }
    controller_.setMeasuredState(msg->position, msg->velocity);
  }

  void updateLoop() {
    const auto now = steady_clock_.now();
    double dt_sec = 0.0;
    if (has_last_update_time_) {
      dt_sec = (now - last_update_time_).seconds();
    }
    last_update_time_ = now;
    has_last_update_time_ = true;

    auto gains = gain_handler_.getGains();
    controller_.setGains(gains.stiffness, gains.damping);
    const auto impedance_cmd = controller_.update(dt_sec);

    plato_interfaces::msg::ImpedanceCommands msg_out;
    msg_out.position = impedance_cmd.position;
    msg_out.velocity = impedance_cmd.velocity;
    msg_out.stiffness = impedance_cmd.stiffness;
    msg_out.damping = impedance_cmd.damping;
    msg_out.effort_ff = impedance_cmd.effort_ff;
    impedance_pub_->publish(msg_out);
  }

  ImpedanceTrajectoryController controller_;
  impedance_trajectory_controller::ImpedanceGainHandler gain_handler_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<plato_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  rclcpp::Clock steady_clock_;
  rclcpp::Time last_update_time_{0, 0, RCL_STEADY_TIME};
  bool has_last_update_time_{false};
  double default_goal_duration_sec_{0.25};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImpedanceTrajectoryControllerNode>());
  rclcpp::shutdown();
  return 0;
}
