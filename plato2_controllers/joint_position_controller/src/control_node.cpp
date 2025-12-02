#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "plato2_interfaces/msg/impedance_commands.hpp"
#include "joint_position_controller/joint_position_controller.hpp"
#include "joint_position_controller/impedance_gain_handler.hpp"

class JointPositionControllerNode : public rclcpp::Node {
public:
  JointPositionControllerNode()
    : Node("joint_position_controller_node"),
      controller_(this->declare_parameter<double>("interpolation_alpha", 0.1),
                  this->declare_parameter<double>("kp_force", 0.5),
                  this->declare_parameter<double>("ki_force", 0.1),
                  this->declare_parameter<double>("force_i_limit", 5.0)),
      gain_handler_(*this)
  {
    position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/plato2/plato2_position_controller/commands", 10,
        std::bind(&JointPositionControllerNode::position_callback, this, std::placeholders::_1));

    desired_force_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        this->declare_parameter<std::string>("desired_force_topic", "/object_state/minimal_force"),
        10, std::bind(&JointPositionControllerNode::desired_force_cb, this, std::placeholders::_1));

    measured_force_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        this->declare_parameter<std::string>("measured_force_topic", "/tactile/force_measured"),
        10, std::bind(&JointPositionControllerNode::measured_force_cb, this, std::placeholders::_1));

    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    impedance_pub_ = this->create_publisher<plato2_interfaces::msg::ImpedanceCommands>(
        "/plato2/joint_impedance_controller/commands", qos_profile);

    RCLCPP_INFO(this->get_logger(), "joint_position_controller_node started");
  }

private:
  void position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const std::vector<double> position_cmd = msg ? msg->data : std::vector<double>();
    auto gains = gain_handler_.getGains();
    controller_.setGains(gains.stiffness, gains.damping);
    auto impedance_cmd = controller_.process(position_cmd, desired_force_, measured_force_, contact_from_estimator_);

    plato2_interfaces::msg::ImpedanceCommands msg_out;
    msg_out.position = impedance_cmd.position;
    msg_out.velocity = impedance_cmd.velocity;
    msg_out.stiffness = impedance_cmd.stiffness;
    msg_out.damping = impedance_cmd.damping;
    msg_out.effort_ff = impedance_cmd.effort_ff;
    impedance_pub_->publish(msg_out);
  }

  void desired_force_cb(const std_msgs::msg::Float64::SharedPtr msg) {
    desired_force_ = msg ? msg->data : 0.0;
    contact_from_estimator_ = desired_force_ > 0.0;
  }

  void measured_force_cb(const std_msgs::msg::Float64::SharedPtr msg) {
    measured_force_ = msg ? msg->data : 0.0;
  }

  JointPositionController controller_;
  joint_position_controller::ImpedanceGainHandler gain_handler_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr desired_force_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measured_force_sub_;
  rclcpp::Publisher<plato2_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_pub_;

  double desired_force_{0.0};
  double measured_force_{0.0};
  bool contact_from_estimator_{false};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
