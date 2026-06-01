#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace maestro_teleop
{

class Float8ArrayToImpedanceTrajectory : public rclcpp::Node
{
public:
  Float8ArrayToImpedanceTrajectory()
  : Node("float8array_to_impedance_trajectory")
  {
    source_topic_ = declare_parameter<std::string>(
      "source_topic", "/plato2/joint_impedance_controller/commands_float8array_HY");
    target_topic_ = declare_parameter<std::string>(
      "target_topic", "/plato2/joint_impedance_trajectory_controller/commands");
    expected_size_ = declare_parameter<int>("expected_size", 8);

    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(target_topic_, 10);
    subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      source_topic_,
      10,
      std::bind(&Float8ArrayToImpedanceTrajectory::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Relaying %s -> %s through joint impedance trajectory LPF",
      source_topic_.c_str(),
      target_topic_.c_str());
  }

private:
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (expected_size_ > 0 && msg->data.size() != static_cast<std::size_t>(expected_size_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Ignoring %s with %zu values; expected %d.",
        source_topic_.c_str(),
        msg->data.size(),
        expected_size_);
      return;
    }

    std_msgs::msg::Float64MultiArray out;
    out.data = msg->data;
    publisher_->publish(out);
  }

  std::string source_topic_;
  std::string target_topic_;
  int expected_size_{8};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

}  // namespace maestro_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<maestro_teleop::Float8ArrayToImpedanceTrajectory>());
  rclcpp::shutdown();
  return 0;
}
