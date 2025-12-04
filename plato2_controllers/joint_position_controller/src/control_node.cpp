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
      controller_(this->declare_parameter<double>("interpolation_alpha", 0.1)),
      gain_handler_(*this)
  {
    position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/plato2/joint_position_controller/commands", 10,
        std::bind(&JointPositionControllerNode::position_callback, this, std::placeholders::_1));

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
    auto impedance_cmd = controller_.process(position_cmd);

    plato2_interfaces::msg::ImpedanceCommands msg_out;
    msg_out.position = impedance_cmd.position;
    msg_out.velocity = impedance_cmd.velocity;
    msg_out.stiffness = impedance_cmd.stiffness;
    msg_out.damping = impedance_cmd.damping;
    msg_out.effort_ff = impedance_cmd.effort_ff;
    impedance_pub_->publish(msg_out);
  }

  JointPositionController controller_;
  joint_position_controller::ImpedanceGainHandler gain_handler_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_sub_;
  rclcpp::Publisher<plato2_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
