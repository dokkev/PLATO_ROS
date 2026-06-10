#include "plato_state_estimator/reference_grasp_force_generator_node.hpp"

#include <chrono>
#include <functional>

namespace plato_state_estimator
{
namespace
{

template<typename MessageT>
void publishScalar(
  const typename rclcpp::Publisher<MessageT>::SharedPtr & publisher,
  const double value)
{
  MessageT msg;
  msg.data = value;
  publisher->publish(msg);
}

}  // namespace

ReferenceGraspForceGeneratorNode::ReferenceGraspForceGeneratorNode()
: Node("reference_grasp_force_generator")
{
  const auto config = loadConfigFromParameters();
  generator_.reset(new ReferenceGraspForceGenerator(config));

  tactile0_sub_ = this->create_subscription<sdr_grasp_msgs::msg::Tactile>(
    "/tactile_0/tactile_states",
    10,
    std::bind(
      &ReferenceGraspForceGeneratorNode::tactile0Callback,
      this,
      std::placeholders::_1));

  tactile1_sub_ = this->create_subscription<sdr_grasp_msgs::msg::Tactile>(
    "/tactile_1/tactile_states",
    10,
    std::bind(
      &ReferenceGraspForceGeneratorNode::tactile1Callback,
      this,
      std::placeholders::_1));

  target_normal_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/grasp_force_reference/target_normal_force_n", 10);
  reference_valid_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/grasp_force_reference/reference_valid", 10);
  measured_min_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/grasp_force_reference/measured_normal_force_min_n", 10);
  measured_avg_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/grasp_force_reference/measured_normal_force_avg_n", 10);
  shear_translation_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/grasp_force_reference/shear_translation_mm", 10);
  shear_rotation_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/grasp_force_reference/shear_rotation_rad", 10);
  slip_state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/grasp_force_reference/slip_state", 10);

  deprecated_minimal_force_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "/object_state/minimal_force", 10);
  deprecated_measured_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/object_state/measured_force", 10);

  const double safe_update_rate = update_rate_ > 0.0 ? update_rate_ : 100.0;
  const auto update_period = std::chrono::duration<double>(1.0 / safe_update_rate);
  update_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(update_period),
    std::bind(&ReferenceGraspForceGeneratorNode::updateTimerCallback, this));

  last_update_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Reference grasp force generator initialized");
  RCLCPP_INFO(this->get_logger(), "  Update rate: %.1f Hz", update_rate_);
  RCLCPP_INFO(
    this->get_logger(),
    "  Translational slip threshold: %.3f mm",
    config.translational_slip_threshold_mm);
  RCLCPP_INFO(
    this->get_logger(),
    "  Rotational slip threshold: %.3f rad",
    config.rotational_slip_threshold_rad);
  RCLCPP_INFO(
    this->get_logger(),
    "  Force limits: [%.3f, %.3f] N",
    config.min_contact_force_n,
    config.max_force_limit_n);
}

ReferenceGraspForceConfig ReferenceGraspForceGeneratorNode::loadConfigFromParameters()
{
  ReferenceGraspForceConfig config;

  this->declare_parameter<double>("update_rate", update_rate_);

  this->declare_parameter<double>("base_force_n", config.base_force_n);
  this->declare_parameter<double>("min_contact_force_n", config.min_contact_force_n);
  this->declare_parameter<double>("max_force_limit_n", config.max_force_limit_n);

  this->declare_parameter<double>("trans_deadband_mm", config.trans_deadband_mm);
  this->declare_parameter<double>("rot_deadband_rad", config.rot_deadband_rad);

  this->declare_parameter<double>(
    "translational_slip_threshold_mm",
    config.translational_slip_threshold_mm);
  this->declare_parameter<double>(
    "rotational_slip_threshold_rad",
    config.rotational_slip_threshold_rad);

  this->declare_parameter<double>("k_trans_n_per_mm", config.k_trans_n_per_mm);
  this->declare_parameter<double>("d_trans_n_per_mm_s", config.d_trans_n_per_mm_s);
  this->declare_parameter<double>("k_rot_n_per_rad", config.k_rot_n_per_rad);
  this->declare_parameter<double>("d_rot_n_per_rad_s", config.d_rot_n_per_rad_s);

  this->declare_parameter<double>(
    "tactile0_shear_x_sign",
    config.tactile0_shear_x_sign);
  this->declare_parameter<double>(
    "tactile0_shear_y_sign",
    config.tactile0_shear_y_sign);
  this->declare_parameter<double>(
    "tactile0_shear_theta_sign",
    config.tactile0_shear_theta_sign);
  this->declare_parameter<double>(
    "tactile1_shear_x_sign",
    config.tactile1_shear_x_sign);
  this->declare_parameter<double>(
    "tactile1_shear_y_sign",
    config.tactile1_shear_y_sign);
  this->declare_parameter<double>(
    "tactile1_shear_theta_sign",
    config.tactile1_shear_theta_sign);

  update_rate_ = this->get_parameter("update_rate").as_double();

  config.base_force_n = this->get_parameter("base_force_n").as_double();
  config.min_contact_force_n = this->get_parameter("min_contact_force_n").as_double();
  config.max_force_limit_n = this->get_parameter("max_force_limit_n").as_double();

  config.trans_deadband_mm = this->get_parameter("trans_deadband_mm").as_double();
  config.rot_deadband_rad = this->get_parameter("rot_deadband_rad").as_double();

  config.translational_slip_threshold_mm =
    this->get_parameter("translational_slip_threshold_mm").as_double();
  config.rotational_slip_threshold_rad =
    this->get_parameter("rotational_slip_threshold_rad").as_double();

  config.k_trans_n_per_mm = this->get_parameter("k_trans_n_per_mm").as_double();
  config.d_trans_n_per_mm_s = this->get_parameter("d_trans_n_per_mm_s").as_double();
  config.k_rot_n_per_rad = this->get_parameter("k_rot_n_per_rad").as_double();
  config.d_rot_n_per_rad_s = this->get_parameter("d_rot_n_per_rad_s").as_double();

  config.tactile0_shear_x_sign =
    this->get_parameter("tactile0_shear_x_sign").as_double();
  config.tactile0_shear_y_sign =
    this->get_parameter("tactile0_shear_y_sign").as_double();
  config.tactile0_shear_theta_sign =
    this->get_parameter("tactile0_shear_theta_sign").as_double();
  config.tactile1_shear_x_sign =
    this->get_parameter("tactile1_shear_x_sign").as_double();
  config.tactile1_shear_y_sign =
    this->get_parameter("tactile1_shear_y_sign").as_double();
  config.tactile1_shear_theta_sign =
    this->get_parameter("tactile1_shear_theta_sign").as_double();

  return config;
}

void ReferenceGraspForceGeneratorNode::tactile0Callback(
  const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> msg)
{
  tactile0_msg_ = msg;
}

void ReferenceGraspForceGeneratorNode::tactile1Callback(
  const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> msg)
{
  tactile1_msg_ = msg;
}

void ReferenceGraspForceGeneratorNode::updateTimerCallback()
{
  if (!tactile0_msg_ || !tactile1_msg_) {
    return;
  }

  const auto current_time = this->now();
  double dt_sec = (current_time - last_update_time_).seconds();
  last_update_time_ = current_time;

  if (dt_sec <= 0.0 || dt_sec > 1.0) {
    dt_sec = update_rate_ > 0.0 ? 1.0 / update_rate_ : 0.01;
  }

  const TactileData tactile0 = convertTactileMsg(tactile0_msg_);
  const TactileData tactile1 = convertTactileMsg(tactile1_msg_);
  const ReferenceGraspForceOutput output = generator_->update(tactile0, tactile1, dt_sec);

  publishOutput(output);

  if (!has_logged_slip_state_ || output.slip_state != last_logged_slip_state_) {
    RCLCPP_INFO(this->get_logger(), "Slip diagnostic state: %s", toString(output.slip_state));
    last_logged_slip_state_ = output.slip_state;
    has_logged_slip_state_ = true;
  }
}

TactileData ReferenceGraspForceGeneratorNode::convertTactileMsg(
  const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> & msg) const
{
  TactileData data;
  if (!msg) {
    return data;
  }

  data.contact_state = msg->contact_state;
  data.shear_x = msg->shear_displacement.x;
  data.shear_y = msg->shear_displacement.y;
  data.shear_theta = msg->shear_displacement.theta;

  data.force_x = 0.0;
  data.force_y = 0.0;
  data.force_z = msg->force.z;

  data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9;
  return data;
}

void ReferenceGraspForceGeneratorNode::publishOutput(
  const ReferenceGraspForceOutput & output)
{
  publishScalar<std_msgs::msg::Float64>(
    target_normal_force_pub_,
    output.target_normal_force_n);

  std_msgs::msg::Bool valid_msg;
  valid_msg.data = output.reference_valid;
  reference_valid_pub_->publish(valid_msg);

  publishScalar<std_msgs::msg::Float64>(
    measured_min_force_pub_,
    output.measured_normal_force_min_n);
  publishScalar<std_msgs::msg::Float64>(
    measured_avg_force_pub_,
    output.measured_normal_force_avg_n);
  publishScalar<std_msgs::msg::Float64>(
    shear_translation_pub_,
    output.shear_translation_mm);
  publishScalar<std_msgs::msg::Float64>(
    shear_rotation_pub_,
    output.shear_rotation_rad);

  std_msgs::msg::String slip_msg;
  slip_msg.data = toString(output.slip_state);
  slip_state_pub_->publish(slip_msg);

  if (output.reference_valid) {
    std_msgs::msg::Float32 deprecated_force_msg;
    deprecated_force_msg.data = static_cast<float>(output.target_normal_force_n);
    deprecated_minimal_force_pub_->publish(deprecated_force_msg);
  }

  publishScalar<std_msgs::msg::Float64>(
    deprecated_measured_force_pub_,
    output.measured_normal_force_min_n);
}

}  // namespace plato_state_estimator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<plato_state_estimator::ReferenceGraspForceGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
