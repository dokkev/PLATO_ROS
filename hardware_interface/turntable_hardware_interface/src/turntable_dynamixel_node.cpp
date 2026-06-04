#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "yaml-cpp/yaml.h"

#include "turntable_hardware_interface/dynamixel.hpp"
#include "turntable_hardware_interface/min_jerk_traj.hpp"

namespace turntable_hardware_interface
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;
constexpr uint8_t kPositionControlMode = 3;

template<typename T>
T yaml_value(const YAML::Node & node, const std::string & key, const T & default_value)
{
  if (!node || !node[key]) {
    return default_value;
  }
  return node[key].as<T>();
}

std::string default_config_file()
{
  return ament_index_cpp::get_package_share_directory("turntable_hardware_interface") +
         "/config/turntable_dynamixel.yaml";
}

}  // namespace

class TurntableDynamixelNode : public rclcpp::Node
{
public:
  TurntableDynamixelNode()
  : Node("turntable_dynamixel_node")
  {
    const auto config_file = declare_parameter<std::string>("config_file", default_config_file());
    position_topic_ = declare_parameter<std::string>("position_topic", "~/position");
    velocity_topic_ = declare_parameter<std::string>("velocity_topic", "~/velocity");
    desired_position_topic_ =
      declare_parameter<std::string>("desired_position_topic", "~/desired_position");

    load_config_(config_file);

    dynamixel_ = std::make_unique<Dynamixel>(dynamixel_config_);
    if (!dynamixel_->open()) {
      throw std::runtime_error(dynamixel_->last_error());
    }
    if (set_position_control_mode_on_start_) {
      try {
        dynamixel_->set_torque_enabled(false);
        dynamixel_->set_operating_mode(kPositionControlMode);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to set turntable Dynamixel position-control mode: %s",
          ex.what());
      }
    }
    if (enable_torque_on_start_) {
      try {
        dynamixel_->set_torque_enabled(true);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to enable turntable Dynamixel torque: %s",
          ex.what());
      }
    }

    position_pub_ = create_publisher<std_msgs::msg::Float64>(position_topic_, 10);
    velocity_pub_ = create_publisher<std_msgs::msg::Float64>(velocity_topic_, 10);
    desired_position_sub_ = create_subscription<std_msgs::msg::Float64>(
      desired_position_topic_,
      10,
      std::bind(&TurntableDynamixelNode::desired_position_callback_, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / command_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TurntableDynamixelNode::control_loop_, this));

    RCLCPP_INFO(
      get_logger(),
      "Turntable Dynamixel node started (id=%u, baudrate=%d, duration=%.3fs, command_rate=%.1fHz)",
      dynamixel_config_.id,
      dynamixel_config_.baudrate,
      min_jerk_duration_sec_,
      command_rate_hz_);
  }

private:
  void load_config_(const std::string & config_file)
  {
    const YAML::Node config = YAML::LoadFile(config_file);
    const YAML::Node dynamixel = config["dynamixel"];
    const YAML::Node min_jerk = config["min_jerk"];

    dynamixel_config_.device_name =
      yaml_value<std::string>(dynamixel, "device_name", dynamixel_config_.device_name);
    dynamixel_config_.baudrate =
      yaml_value<int>(dynamixel, "baudrate", dynamixel_config_.baudrate);
    dynamixel_config_.id =
      static_cast<uint8_t>(yaml_value<int>(dynamixel, "servo_id", dynamixel_config_.id));
    dynamixel_config_.protocol_version =
      yaml_value<float>(dynamixel, "protocol_version", dynamixel_config_.protocol_version);
    dynamixel_config_.operating_mode_address =
      static_cast<uint16_t>(yaml_value<int>(
        dynamixel, "operating_mode_address", dynamixel_config_.operating_mode_address));
    dynamixel_config_.torque_enable_address =
      static_cast<uint16_t>(yaml_value<int>(
        dynamixel, "torque_enable_address", dynamixel_config_.torque_enable_address));
    dynamixel_config_.goal_position_address =
      static_cast<uint16_t>(yaml_value<int>(
        dynamixel, "goal_position_address", dynamixel_config_.goal_position_address));
    dynamixel_config_.present_velocity_address =
      static_cast<uint16_t>(yaml_value<int>(
        dynamixel, "present_velocity_address", dynamixel_config_.present_velocity_address));
    dynamixel_config_.present_position_address =
      static_cast<uint16_t>(yaml_value<int>(
        dynamixel, "present_position_address", dynamixel_config_.present_position_address));
    position_units_per_revolution_ =
      yaml_value<double>(
      dynamixel,
      "position_units_per_revolution",
      position_units_per_revolution_);
    velocity_unit_rad_per_sec_ =
      yaml_value<double>(
      dynamixel,
      "velocity_unit_rad_per_sec",
      velocity_unit_rad_per_sec_);
    enable_torque_on_start_ =
      yaml_value<bool>(dynamixel, "enable_torque_on_start", enable_torque_on_start_);
    set_position_control_mode_on_start_ =
      yaml_value<bool>(
      dynamixel,
      "set_position_control_mode_on_start",
      set_position_control_mode_on_start_);

    min_jerk_duration_sec_ =
      yaml_value<double>(min_jerk, "duration_sec", min_jerk_duration_sec_);
    command_rate_hz_ =
      yaml_value<double>(
      min_jerk,
      "command_rate_hz",
      yaml_value<double>(
        config["control"],
        "update_rate_hz",
        yaml_value<double>(min_jerk, "update_rate_hz", command_rate_hz_)));
    completion_tolerance_rad_ =
      yaml_value<double>(
      min_jerk,
      "completion_tolerance_rad",
      completion_tolerance_rad_);

    if (command_rate_hz_ <= 0.0) {
      throw std::runtime_error("min_jerk.command_rate_hz must be positive.");
    }
    if (min_jerk_duration_sec_ <= 0.0) {
      throw std::runtime_error("min_jerk.duration_sec must be positive.");
    }
    if (completion_tolerance_rad_ < 0.0) {
      throw std::runtime_error("min_jerk.completion_tolerance_rad must be non-negative.");
    }
    if (position_units_per_revolution_ <= 0.0) {
      throw std::runtime_error("dynamixel.position_units_per_revolution must be positive.");
    }
    if (velocity_unit_rad_per_sec_ <= 0.0) {
      throw std::runtime_error("dynamixel.velocity_unit_rad_per_sec must be positive.");
    }
  }

  void desired_position_callback_(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (traj_active_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Ignoring desired position %.3f rad while turntable trajectory is active.",
        msg->data);
      return;
    }

    double start_position_rad = 0.0;
    try {
      start_position_rad = read_position_rad_();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Cannot start turntable trajectory: %s",
        ex.what());
      return;
    }

    target_position_rad_ = msg->data;
    trajectory_.reset(start_position_rad, target_position_rad_, min_jerk_duration_sec_);
    traj_start_time_ = now();
    traj_active_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Started turntable min-jerk trajectory: %.6f rad -> %.6f rad over %.3fs",
      start_position_rad,
      target_position_rad_,
      min_jerk_duration_sec_);
  }

  void control_loop_()
  {
    double current_position_rad = 0.0;
    double current_velocity_rad_per_sec = 0.0;
    try {
      current_position_rad = read_position_rad_();
      current_velocity_rad_per_sec = read_velocity_rad_per_sec_();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Failed to read turntable Dynamixel state: %s",
        ex.what());
      return;
    }

    publish_state_(current_position_rad, current_velocity_rad_per_sec);

    if (!traj_active_) {
      return;
    }

    const double elapsed_sec = (now() - traj_start_time_).seconds();
    const auto sample = trajectory_.sample(elapsed_sec);
    try {
      dynamixel_->set_desired_position(rad_to_raw_position_(sample.position));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Failed to command turntable Dynamixel: %s",
        ex.what());
      return;
    }

    const double position_error_rad = std::abs(current_position_rad - target_position_rad_);
    if (elapsed_sec >= trajectory_.duration_sec() &&
      position_error_rad <= completion_tolerance_rad_)
    {
      try {
        dynamixel_->set_desired_position(rad_to_raw_position_(target_position_rad_));
      } catch (const std::exception & ex) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Failed to command final turntable Dynamixel target: %s",
          ex.what());
        return;
      }
      traj_active_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Turntable trajectory complete at position %.6f rad (target %.6f rad, error %.6f rad, tolerance %.6f rad).",
        current_position_rad,
        target_position_rad_,
        position_error_rad,
        completion_tolerance_rad_);
    }
  }

  double read_position_rad_()
  {
    return raw_position_to_rad_(static_cast<double>(dynamixel_->get_current_position()));
  }

  double read_velocity_rad_per_sec_()
  {
    return raw_velocity_to_rad_per_sec_(static_cast<double>(dynamixel_->get_current_velocity()));
  }

  void publish_state_(double position, double velocity)
  {
    std_msgs::msg::Float64 position_msg;
    position_msg.data = position;
    position_pub_->publish(position_msg);

    std_msgs::msg::Float64 velocity_msg;
    velocity_msg.data = velocity;
    velocity_pub_->publish(velocity_msg);
  }

  double raw_position_to_rad_(double raw_position) const
  {
    return raw_position * kTwoPi / position_units_per_revolution_;
  }

  int32_t rad_to_raw_position_(double position_rad) const
  {
    return static_cast<int32_t>(std::llround(
             position_rad * position_units_per_revolution_ /
             kTwoPi));
  }

  double raw_velocity_to_rad_per_sec_(double raw_velocity) const
  {
    return raw_velocity * velocity_unit_rad_per_sec_;
  }

  Dynamixel::Config dynamixel_config_;
  std::unique_ptr<Dynamixel> dynamixel_;
  MinJerkTraj trajectory_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr desired_position_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string position_topic_;
  std::string velocity_topic_;
  std::string desired_position_topic_;

  double min_jerk_duration_sec_ = 1.0;
  double command_rate_hz_ = 100.0;
  double completion_tolerance_rad_ = 0.01;
  double position_units_per_revolution_ = 4096.0;
  double velocity_unit_rad_per_sec_ = 0.0239808239;
  double target_position_rad_ = 0.0;
  rclcpp::Time traj_start_time_;
  bool traj_active_ = false;
  bool set_position_control_mode_on_start_ = true;
  bool enable_torque_on_start_ = true;
};

}  // namespace turntable_hardware_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turntable_hardware_interface::TurntableDynamixelNode>());
  rclcpp::shutdown();
  return 0;
}
