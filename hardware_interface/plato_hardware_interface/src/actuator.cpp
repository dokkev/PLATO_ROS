#include "plato_hardware_interface/actuator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "plato_hardware_interface/can_protocol.hpp"

namespace plato_actuator
{

namespace
{
can_hardware_common::ActuatorCoreConfig make_core_config(const Config & config)
{
  can_hardware_common::ActuatorCoreConfig core;
  core.can_tx_id = config.static_config.can_tx_id;
  core.can_rx_id = config.static_config.can_rx_id;
  core.position_offset = config.position_offset;
  core.direction = config.static_config.direction;
  core.torque_constant = config.static_config.torque_constant;
  core.gear_ratio = config.static_config.gear_ratio;
  return core;
}

void validate_direction(const StaticConfig & config)
{
  if (config.direction != 1 && config.direction != -1) {
    throw std::invalid_argument("Plato actuator direction must be +1 or -1");
  }
}
}  // namespace

Actuator::Actuator(const Config & config)
: static_config_(config.static_config),
  position_offset_(config.position_offset),
  protocol_(std::make_unique<CANProtocol>(make_core_config(config)))
{
  validate_direction(static_config_);
}

Actuator::Actuator(Actuator &&) noexcept = default;

Actuator::~Actuator() = default;

actuator::TxCommand Actuator::enable_motor()
{
  return protocol_->make_enable_motor_command();
}

actuator::TxCommand Actuator::disable_motor()
{
  return protocol_->make_disable_motor_command();
}

actuator::TxCommand Actuator::stop_control()
{
  return protocol_->make_stop_control_command();
}

actuator::TxCommand Actuator::set_joint_torque(float joint_torque)
{
  joint_torque = clamp_torque_near_bounds_(joint_torque);
  if (std::isfinite(static_config_.limits.effort_limit)) {
    joint_torque = std::clamp(
      joint_torque, -static_config_.limits.effort_limit, static_config_.limits.effort_limit);
  }

  float motor_torque = map_joint_to_motor_frame_(joint_torque) * kMotorTorqueScale;
  motor_torque = std::clamp(motor_torque, -kMotorTorqueLimit, kMotorTorqueLimit);

  return protocol_->make_torque_command(motor_torque);
}

actuator::TxCommand Actuator::set_joint_position(float joint_position, uint32_t duration_ms)
{
  if (std::isfinite(static_config_.limits.position_limit_min) &&
    std::isfinite(static_config_.limits.position_limit_max))
  {
    joint_position = std::clamp(
      joint_position,
      static_config_.limits.position_limit_min,
      static_config_.limits.position_limit_max);
  }

  return protocol_->make_position_command(joint_position, duration_ms);
}

actuator::TxCommand Actuator::set_servo_position(float joint_position, uint32_t current_milliamps)
{
  if (std::isfinite(static_config_.limits.position_limit_min) &&
    std::isfinite(static_config_.limits.position_limit_max))
  {
    joint_position = std::clamp(
      joint_position,
      static_config_.limits.position_limit_min,
      static_config_.limits.position_limit_max);
  }

  return protocol_->make_servo_position_command(joint_position, current_milliamps);
}

actuator::TxCommand Actuator::set_servo_hold(float joint_position)
{
  return set_servo_position(joint_position, static_config_.servo_current_milliamps);
}

actuator::TxCommand Actuator::set_servo_idle(float joint_position)
{
  return set_servo_position(joint_position, 0);
}

bool Actuator::set_current_position_as_zero()
{
  if (!is_initialized_) {
    return false;
  }

  position_offset_ = motor_position_;
  protocol_->set_position_offset(position_offset_);
  state_.position = 0.0f;
  return true;
}

void Actuator::process_message(const TPCANMsg & msg)
{
  if (msg.ID != get_rx_id()) {
    return;
  }

  const auto decoded = protocol_->decode(msg);
  if (!decoded) {
    return;
  }

  apply_decoded_feedback_(*decoded);
}

float Actuator::clamp_torque_near_bounds_(float joint_torque) const
{
  if (!static_config_.soft_stop_enabled) {
    return joint_torque;
  }

  if (!is_initialized_) {
    return joint_torque;
  }

  if (!std::isfinite(static_config_.limits.position_limit_min) ||
    !std::isfinite(static_config_.limits.position_limit_max))
  {
    return joint_torque;
  }

  const float min_limit_threshold = static_config_.limits.position_limit_min + kJointLimitSafetyMargin;
  const float max_limit_threshold = static_config_.limits.position_limit_max - kJointLimitSafetyMargin;

  if (state_.position < min_limit_threshold && joint_torque < 0.0f) {
    float norm_dist =
      (state_.position - static_config_.limits.position_limit_min) / kJointLimitSafetyMargin;
    norm_dist = std::clamp(norm_dist, 0.0f, 1.0f);
    return joint_torque * norm_dist * norm_dist;
  }

  if (state_.position > max_limit_threshold && joint_torque > 0.0f) {
    float norm_dist =
      (static_config_.limits.position_limit_max - state_.position) / kJointLimitSafetyMargin;
    norm_dist = std::clamp(norm_dist, 0.0f, 1.0f);
    return joint_torque * norm_dist * norm_dist;
  }

  return joint_torque;
}

float Actuator::map_joint_to_motor_frame_(float joint_value, bool apply_offset) const
{
  const float direction = static_cast<float>(static_config_.direction);
  return apply_offset ?
         (joint_value * direction) + position_offset_ :
         joint_value * direction;
}

float Actuator::map_motor_to_joint_frame_(float motor_value, bool apply_offset) const
{
  const float direction = static_cast<float>(static_config_.direction);
  return apply_offset ?
         (motor_value - position_offset_) * direction :
         motor_value * direction;
}

void Actuator::apply_decoded_feedback_(const can_hardware_common::DecodedFeedback & decoded)
{
  if (decoded.motor_position) {
    motor_position_ = *decoded.motor_position;
  }

  if (decoded.has_state) {
    state_ = decoded.state;
    is_initialized_ = true;
  }

  if (decoded.temperature) {
    status_.temperature = *decoded.temperature;
  }

  if (decoded.in_oc_mode) {
    status_.in_oc_mode = *decoded.in_oc_mode;
  }

  if (decoded.has_fault) {
    status_.has_fault = *decoded.has_fault;
  }

  if (decoded.motor_enabled) {
    motor_enabled_ = *decoded.motor_enabled;
  }
}

}  // namespace plato_actuator
