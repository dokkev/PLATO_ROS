#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace maestro_teleop
{

class FingertipDistToParallelGrasp : public rclcpp::Node
{
public:
  FingertipDistToParallelGrasp()
  : Node("fingertip_dist_to_parallel_grasp")
  {
    source_topic_ = declare_parameter<std::string>(
      "source_topic", "/plato2/fingertip_dist_cmd");
    target_topic_ = declare_parameter<std::string>(
      "target_topic", "/plato2/parallel_grasp_controller/commands");

    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(target_topic_, 10);
    subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      source_topic_,
      10,
      std::bind(&FingertipDistToParallelGrasp::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Relaying %s [0,3] -> %s [t-i_dis, I_PIP]",
      source_topic_.c_str(),
      target_topic_.c_str());
  }

private:
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() <= kIndexPipIndex) {
      const auto size = msg ? msg->data.size() : 0U;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Ignoring fingertip_dist_cmd with %zu values; expected at least %zu.",
        size,
        kIndexPipIndex + 1U);
      return;
    }

    std_msgs::msg::Float64MultiArray out;
    out.data = {msg->data[kThumbIndexDistanceIndex], msg->data[kIndexPipIndex]};
    publisher_->publish(out);
  }

  static constexpr std::size_t kThumbIndexDistanceIndex = 0U;
  static constexpr std::size_t kIndexPipIndex = 3U;

  std::string source_topic_;
  std::string target_topic_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

}  // namespace maestro_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<maestro_teleop::FingertipDistToParallelGrasp>());
  rclcpp::shutdown();
  return 0;
}
