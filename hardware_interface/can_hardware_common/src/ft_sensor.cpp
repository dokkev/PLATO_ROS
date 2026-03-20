#include "can_hardware_common/ft_sensor.hpp"

#include <rclcpp/rclcpp.hpp>

namespace sensor
{

namespace
{
void decode_force(const TPCANMsg & msg, Eigen::Vector3f & force)
{
  constexpr float kForceScale = 0.001f;
  constexpr float kForceBias = -30.0f;
  const uint8_t * data = msg.DATA;

  force.x() = (uint16_t(data[0]) * 256u + uint16_t(data[1])) * kForceScale + kForceBias;
  force.y() = (uint16_t(data[2]) * 256u + uint16_t(data[3])) * kForceScale + kForceBias;
  force.z() = (uint16_t(data[4]) * 256u + uint16_t(data[5])) * kForceScale + kForceBias;
}

void decode_torque(const TPCANMsg & msg, Eigen::Vector3f & torque)
{
  constexpr float kTorqueScale = 1e-5f;
  constexpr float kTorqueBias = -0.3f;
  const uint8_t * data = msg.DATA;

  torque.x() = (uint16_t(data[0]) * 256u + uint16_t(data[1])) * kTorqueScale + kTorqueBias;
  torque.y() = (uint16_t(data[2]) * 256u + uint16_t(data[3])) * kTorqueScale + kTorqueBias;
  torque.z() = (uint16_t(data[4]) * 256u + uint16_t(data[5])) * kTorqueScale + kTorqueBias;
}
}  // namespace

FTSensor::FTSensor(const Config & config)
: config_(config),
  alpha_(0.06f),
  calibration_samples_(100),
  force_bias_calibrated_(false),
  torque_bias_calibrated_(false),
  bias_samples_force_(0),
  bias_samples_torque_(0)
{
  RCLCPP_INFO(
    rclcpp::get_logger("can_hardware_common"),
    "FT Sensor initialized - force ID: 0x%X, torque ID: 0x%X",
    config_.force_rx_id, config_.torque_rx_id);
}

void FTSensor::apply_low_pass_filter(States & states)
{
  states.force_filtered =
    alpha_ * (states.force_raw - bias_force_) + (1.0f - alpha_) * states.force_filtered;
  states.torque_filtered =
    alpha_ * (states.torque_raw - bias_torque_) + (1.0f - alpha_) * states.torque_filtered;
}

void FTSensor::update_bias()
{
  if (!force_bias_calibrated_ && bias_samples_force_ >= calibration_samples_) {
    bias_force_ = bias_accum_force_ / static_cast<float>(bias_samples_force_);
    force_bias_calibrated_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("can_hardware_common"),
      "FT Sensor 0x%X force bias calibrated: [%.4f, %.4f, %.4f]",
      config_.force_rx_id,
      bias_force_.x(), bias_force_.y(), bias_force_.z());
  }

  if (!torque_bias_calibrated_ && bias_samples_torque_ >= calibration_samples_) {
    bias_torque_ = bias_accum_torque_ / static_cast<float>(bias_samples_torque_);
    torque_bias_calibrated_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("can_hardware_common"),
      "FT Sensor 0x%X torque bias calibrated: [%.4f, %.4f, %.4f]",
      config_.torque_rx_id,
      bias_torque_.x(), bias_torque_.y(), bias_torque_.z());
  }
}

void FTSensor::process_message(const TPCANMsg & msg)
{
  if (msg.ID == config_.force_rx_id) {
    decode_force(msg, states_.force_raw);

    if (!force_bias_calibrated_) {
      bias_accum_force_ += states_.force_raw;
      ++bias_samples_force_;
      update_bias();
    }
  } else if (msg.ID == config_.torque_rx_id) {
    decode_torque(msg, states_.torque_raw);

    if (!torque_bias_calibrated_) {
      bias_accum_torque_ += states_.torque_raw;
      ++bias_samples_torque_;
      update_bias();
    }
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("can_hardware_common"),
      "FTSensor received unexpected CAN ID: 0x%X", msg.ID);
    return;
  }

  if (force_bias_calibrated_ && torque_bias_calibrated_) {
    apply_low_pass_filter(states_);
  }
}

}  // namespace sensor
