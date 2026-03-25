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
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kSecondsPerMinute = 60.0f;

void validate_direction(const Config & config)
{
  if (config.core.direction != 1 && config.core.direction != -1) {
    throw std::invalid_argument("Plato actuator direction must be +1 or -1");
  }
}
}  // namespace

Actuator::Actuator(const Config & config)
: config_(config),
  protocol_(std::make_unique<can_protocol::SteadywinProtocol>(config.core))
{
  validate_direction(config_);
}

Actuator::Actuator(Actuator &&) noexcept = default;

Actuator & Actuator::operator=(Actuator &&) noexcept = default;

Actuator::~Actuator() = default;

can_hardware_common::CommandRequest Actuator::to_request_(
  uint32_t key, const actuator::TxCommand & cmd) const
{
  can_hardware_common::CommandRequest req;
  req.key = key;
  req.frame = cmd.frame;
  req.reply.expected_rx_id = config_.core.can_rx_id;
  req.reply.expected_opcode = cmd.expected_response_opcode;
  req.reply.success_byte = 0x00;
  return req;
}

can_hardware_common::CommandRequest Actuator::make_enable_request(uint32_t key) const
{
  // const_cast needed because protocol is not const-correct (it mutates internal msg buffers).
  return to_request_(key, const_cast<Actuator *>(this)->enable_motor());
}

can_hardware_common::CommandRequest Actuator::make_disable_request(uint32_t key) const
{
  return to_request_(key, const_cast<Actuator *>(this)->disable_motor());
}

can_hardware_common::CommandRequest Actuator::make_torque_request(
  uint32_t key, float joint_torque)
{
  return to_request_(key, set_joint_torque(joint_torque));
}

can_hardware_common::CommandRequest Actuator::make_servo_hold_request(
  uint32_t key, float joint_position)
{
  return to_request_(key, set_servo_hold(joint_position));
}

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
  if (std::isfinite(config_.limits.effort_limit)) {
    joint_torque = std::clamp(
      joint_torque, -config_.limits.effort_limit, config_.limits.effort_limit);
  }

  float motor_torque = map_joint_to_motor_frame_(joint_torque) * kMotorTorqueScale;
  motor_torque = std::clamp(motor_torque, -kMotorTorqueLimit, kMotorTorqueLimit);

  return protocol_->make_torque_command(motor_torque);
}

actuator::TxCommand Actuator::set_joint_position(float joint_position, uint32_t duration_ms)
{
  if (std::isfinite(config_.limits.position_limit_min) &&
    std::isfinite(config_.limits.position_limit_max))
  {
    joint_position = std::clamp(
      joint_position,
      config_.limits.position_limit_min,
      config_.limits.position_limit_max);
  }

  return protocol_->make_position_command(
    map_joint_to_motor_frame_(joint_position, true),
    duration_ms);
}

actuator::TxCommand Actuator::set_servo_position(float joint_position, uint32_t current_milliamps)
{
  if (std::isfinite(config_.limits.position_limit_min) &&
    std::isfinite(config_.limits.position_limit_max))
  {
    joint_position = std::clamp(
      joint_position,
      config_.limits.position_limit_min,
      config_.limits.position_limit_max);
  }

  return protocol_->make_servo_position_command(
    map_joint_to_motor_frame_(joint_position, true),
    current_milliamps);
}

actuator::TxCommand Actuator::set_servo_hold(float joint_position)
{
  return set_servo_position(joint_position, config_.servo_current_milliamps);
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

  config_.core.position_offset = motor_position_;
  state_.position = 0.0f;
  return true;
}

void Actuator::process_message(const TPCANMsg & msg)
{
  if (msg.ID != get_rx_id()) {
    return;
  }

  protocol_->process_message(msg, *this);
}

void Actuator::apply_motor_feedback(
  float motor_position,
  float motor_velocity,
  float motor_torque,
  bool position_has_offset,
  bool velocity_is_rpm)
{
  set_motor_position_raw(motor_position);

  if (velocity_is_rpm) {
    motor_velocity = motor_velocity * kTwoPi / kSecondsPerMinute;
  }

  state_.position = map_motor_to_joint_frame_(motor_position, position_has_offset);
  state_.velocity = map_motor_to_joint_frame_(motor_velocity);
  state_.torque = map_motor_to_joint_frame_(motor_torque);
  is_initialized_ = true;
}

float Actuator::clamp_torque_near_bounds_(float joint_torque) const
{
  if (!is_initialized_) {
    return joint_torque;
  }

  if (!std::isfinite(config_.limits.position_limit_min) ||
    !std::isfinite(config_.limits.position_limit_max))
  {
    return joint_torque;
  }

  const float min_limit_threshold = config_.limits.position_limit_min + kJointLimitSafetyMargin;
  const float max_limit_threshold = config_.limits.position_limit_max - kJointLimitSafetyMargin;

  if (state_.position < min_limit_threshold && joint_torque < 0.0f) {
    float norm_dist = (state_.position - config_.limits.position_limit_min) / kJointLimitSafetyMargin;
    norm_dist = std::clamp(norm_dist, 0.0f, 1.0f);
    return joint_torque * norm_dist * norm_dist;
  }

  if (state_.position > max_limit_threshold && joint_torque > 0.0f) {
    float norm_dist = (config_.limits.position_limit_max - state_.position) / kJointLimitSafetyMargin;
    norm_dist = std::clamp(norm_dist, 0.0f, 1.0f);
    return joint_torque * norm_dist * norm_dist;
  }

  return joint_torque;
}

float Actuator::map_joint_to_motor_frame_(float joint_value, bool apply_offset) const
{
  const float direction = static_cast<float>(config_.core.direction);
  return apply_offset ?
         (joint_value * direction) + config_.core.position_offset :
         joint_value * direction;
}

float Actuator::map_motor_to_joint_frame_(float motor_value, bool apply_offset) const
{
  const float direction = static_cast<float>(config_.core.direction);
  return apply_offset ?
         (motor_value - config_.core.position_offset) * direction :
         motor_value * direction;
}

}  // namespace plato_actuator
