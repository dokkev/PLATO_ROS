#ifndef PLATO_STATE_ESTIMATOR_REFERENCE_GRASP_FORCE_GENERATOR_NODE_HPP
#define PLATO_STATE_ESTIMATOR_REFERENCE_GRASP_FORCE_GENERATOR_NODE_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sdr_grasp_msgs/msg/tactile.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "plato_state_estimator/reference_grasp_force_generator.hpp"

namespace plato_state_estimator
{

// ROS wrapper for ReferenceGraspForceGenerator.
//
// The node publishes a scalar normal force reference plus diagnostics. It does
// not estimate full object state or make task/state-machine decisions.
class ReferenceGraspForceGeneratorNode : public rclcpp::Node
{
public:
  ReferenceGraspForceGeneratorNode();
  ~ReferenceGraspForceGeneratorNode() = default;

private:
  void tactile0Callback(const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> msg);
  void tactile1Callback(const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> msg);
  void updateTimerCallback();

  TactileData convertTactileMsg(
    const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> & msg) const;
  ReferenceGraspForceConfig loadConfigFromParameters();
  void publishOutput(const ReferenceGraspForceOutput & output);

  rclcpp::Subscription<sdr_grasp_msgs::msg::Tactile>::SharedPtr tactile0_sub_;
  rclcpp::Subscription<sdr_grasp_msgs::msg::Tactile>::SharedPtr tactile1_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_normal_force_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reference_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measured_min_force_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measured_avg_force_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr shear_translation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr shear_rotation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slip_state_pub_;

  // Deprecated compatibility topics.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr deprecated_minimal_force_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr deprecated_measured_force_pub_;

  rclcpp::TimerBase::SharedPtr update_timer_;

  std::unique_ptr<ReferenceGraspForceGenerator> generator_;

  std::shared_ptr<sdr_grasp_msgs::msg::Tactile> tactile0_msg_;
  std::shared_ptr<sdr_grasp_msgs::msg::Tactile> tactile1_msg_;

  rclcpp::Time last_update_time_;
  double update_rate_{100.0};
  SlipState last_logged_slip_state_{SlipState::NO_CONTACT};
  bool has_logged_slip_state_{false};
};

}  // namespace plato_state_estimator

#endif  // PLATO_STATE_ESTIMATOR_REFERENCE_GRASP_FORCE_GENERATOR_NODE_HPP
