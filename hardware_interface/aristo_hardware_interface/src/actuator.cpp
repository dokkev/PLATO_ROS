#include "aristo_hardware_interface/actuator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "aristo_hardware_interface/can_protocol.hpp"
#include <rclcpp/rclcpp.hpp>

namespace aristo_actuator
{

namespace
{
auto logger() { return rclcpp::get_logger("aristo_actuator"); }

rclcpp::Clock & log_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

void validate_direction(const Config & config)
{
  if (config.core.direction != 1 && config.core.direction != -1) {
    throw std::invalid_argument("Aristo actuator direction must be +1 or -1");
  }
}
}  // namespace

Actuator::Actuator(const Config & config)
: config_(config),
  protocol_(std::make_unique<CANProtocol>(config.core))
{
  validate_direction(config_);
}

Actuator::Actuator(Actuator &&) noexcept = default;

Actuator & Actuator::operator=(Actuator &&) noexcept = default;

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

actuator::TxCommand Actuator::set_current_position_as_zero()
{
  return protocol_->make_zero_position_command();
}

actuator::TxCommand Actuator::set_default_can_limits()
{
  return protocol_->make_default_can_limits_command();
}

actuator::TxCommand Actuator::read_motor_params()
{
  return protocol_->make_read_motor_params_command();
}

actuator::TxCommand Actuator::read_can_limits()
{
  return protocol_->make_read_can_limits_command();
}

actuator::TxCommand Actuator::read_state()
{
  return protocol_->make_read_state_command();
}

actuator::TxCommand Actuator::set_joint_torque(float joint_torque)
{
  joint_torque = clamp_torque_near_bounds_(joint_torque);
  if (std::isfinite(config_.limits.effort_limit)) {
    joint_torque = std::clamp(joint_torque, -config_.limits.effort_limit, config_.limits.effort_limit);
  }

  return protocol_->make_torque_command(joint_torque);
}

std::optional<actuator::TxCommand> Actuator::set_joint_impedance(
  const can_hardware_common::ActuatorTarget & joint_target_in)
{
  can_hardware_common::ActuatorTarget joint_target = joint_target_in;
  clamp_impedance_target_(joint_target);
  determine_current_state_();

  joint_target.torque = smooth_torque_cmd_(joint_target.torque);
  filter_soft_limit_command_(joint_target);

  auto command = protocol_->make_impedance_command(joint_target);
  if (!command) {
    return std::nullopt;
  }

  return command;
}

void Actuator::process_rx_frame(const TPCANMsg & msg)
{
  if (msg.ID != get_rx_id()) {
    return;
  }

  if (protocol_->try_update_motor_params(msg)) {
    motor_params_ = protocol_->motor_params();
    has_motor_params_ = true;
    return;
  }

  if (protocol_->try_update_can_limits(msg)) {
    active_limits_ = protocol_->active_limits();
    has_active_limits_ = true;
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
  if (!has_feedback_) {
    return joint_torque;
  }

  if (!std::isfinite(config_.limits.position_limit_min) ||
    !std::isfinite(config_.limits.position_limit_max))
  {
    return joint_torque;
  }

  const float min_limit_threshold = config_.limits.position_limit_min + kJointLimitSafetyMargin;
  const float max_limit_threshold = config_.limits.position_limit_max - kJointLimitSafetyMargin;

  if (feedback_.position < min_limit_threshold && joint_torque < 0.0f) {
    float norm_dist = (feedback_.position - config_.limits.position_limit_min) / kJointLimitSafetyMargin;
    norm_dist = std::clamp(norm_dist, 0.0f, 1.0f);
    return joint_torque * norm_dist * norm_dist;
  }

  if (feedback_.position > max_limit_threshold && joint_torque > 0.0f) {
    float norm_dist = (config_.limits.position_limit_max - feedback_.position) / kJointLimitSafetyMargin;
    norm_dist = std::clamp(norm_dist, 0.0f, 1.0f);
    return joint_torque * norm_dist * norm_dist;
  }

  return joint_torque;
}

void Actuator::determine_current_state_()
{
  if (!has_feedback_) {
    control_state_ = SoftLimitState::kOperational;
    return;
  }

  if (!std::isfinite(config_.limits.position_limit_min) ||
    !std::isfinite(config_.limits.position_limit_max))
  {
    control_state_ = SoftLimitState::kOperational;
    return;
  }

  const float q_meas = feedback_.position;
  const float q_min = config_.limits.position_limit_min;
  const float q_max = config_.limits.position_limit_max;

  if (q_meas < q_min || q_meas > q_max)
  {
    control_state_ = SoftLimitState::kOverLimit;
  } else if (control_state_ == SoftLimitState::kLowerLimit) {
    if (q_meas > q_min + kSoftLimitExitMargin) {
      control_state_ = SoftLimitState::kOperational;
    }
  } else if (control_state_ == SoftLimitState::kUpperLimit) {
    if (q_meas < q_max - kSoftLimitExitMargin) {
      control_state_ = SoftLimitState::kOperational;
    }
  } else if (q_meas < q_min + kSoftLimitEnterMargin) {
    control_state_ = SoftLimitState::kLowerLimit;
  } else if (q_meas > q_max - kSoftLimitEnterMargin) {
    control_state_ = SoftLimitState::kUpperLimit;
  } else {
    control_state_ = SoftLimitState::kOperational;
  }
}

void Actuator::clamp_impedance_target_(can_hardware_common::ActuatorTarget & joint_target) const
{
  if (std::isfinite(config_.limits.position_limit_min) &&
    std::isfinite(config_.limits.position_limit_max))
  {
    joint_target.position = std::clamp(
      joint_target.position, config_.limits.position_limit_min, config_.limits.position_limit_max);
  }
  if (std::isfinite(config_.limits.velocity_limit)) {
    joint_target.velocity = std::clamp(
      joint_target.velocity, -config_.limits.velocity_limit, config_.limits.velocity_limit);
  }
  if (std::isfinite(config_.limits.effort_limit)) {
    joint_target.torque = std::clamp(
      joint_target.torque, -config_.limits.effort_limit, config_.limits.effort_limit);
  }
  if (std::isfinite(config_.limits.stiffness_limit)) {
    joint_target.stiffness = std::clamp(
      joint_target.stiffness, 0.0f, config_.limits.stiffness_limit);
  }
  if (std::isfinite(config_.limits.damping_limit)) {
    joint_target.damping = std::clamp(
      joint_target.damping, 0.0f, config_.limits.damping_limit);
  }
}

void Actuator::filter_soft_limit_command_(can_hardware_common::ActuatorTarget & joint_target)
{
  if (!has_feedback_) {
    return;
  }

  bool clipped = false;
  switch (control_state_) {
    case SoftLimitState::kLowerLimit:
      if (joint_target.position < feedback_.position) {
        joint_target.position = feedback_.position;
        clipped = true;
      }
      if (joint_target.velocity < 0.0f) {
        joint_target.velocity = 0.0f;
        clipped = true;
      }
      if (joint_target.torque < 0.0f) {
        joint_target.torque = 0.0f;
        clipped = true;
      }
      if (clipped) {
        RCLCPP_WARN_THROTTLE(
          logger(),
          log_clock(),
          1000,
          "Aristo actuator 0x%X lower soft limit: outward command clipped",
          get_tx_id());
      }
      break;
    case SoftLimitState::kUpperLimit:
      if (joint_target.position > feedback_.position) {
        joint_target.position = feedback_.position;
        clipped = true;
      }
      if (joint_target.velocity > 0.0f) {
        joint_target.velocity = 0.0f;
        clipped = true;
      }
      if (joint_target.torque > 0.0f) {
        joint_target.torque = 0.0f;
        clipped = true;
      }
      if (clipped) {
        RCLCPP_WARN_THROTTLE(
          logger(),
          log_clock(),
          1000,
          "Aristo actuator 0x%X upper soft limit: outward command clipped",
          get_tx_id());
      }
      break;
    case SoftLimitState::kOverLimit:
      joint_target.position = feedback_.position;
      joint_target.velocity = 0.0f;
      joint_target.torque = 0.0f;
      joint_target.stiffness = 0.0f;
      joint_target.damping = kOverLimitDampingMNmPerRadS;
      reset_torque_cmd_smoothing_(0.0f);
      RCLCPP_WARN_THROTTLE(
        logger(),
        log_clock(),
        1000,
        "Aristo actuator 0x%X over position limit: damping-only safety mode active",
        get_tx_id());
      break;
    case SoftLimitState::kOperational:
      break;
  }
}

float Actuator::smooth_torque_cmd_(float torque)
{
  if (!std::isfinite(torque)) {
    return torque;
  }

  const float smoothing = config_.torque_cmd_smoothing;
  if (smoothing <= 0.0f || !has_smoothed_torque_cmd_) {
    reset_torque_cmd_smoothing_(torque);
    return torque;
  }

  smoothed_torque_cmd_ =
    smoothing * smoothed_torque_cmd_ + (1.0f - smoothing) * torque;
  return smoothed_torque_cmd_;
}

void Actuator::reset_torque_cmd_smoothing_(float torque)
{
  smoothed_torque_cmd_ = torque;
  has_smoothed_torque_cmd_ = true;
}

float Actuator::smooth_torque_meas_(float effort)
{
  if (!std::isfinite(effort)) {
    return effort;
  }

  const float smoothing = config_.torque_meas_smoothing;
  if (smoothing <= 0.0f || !has_smoothed_torque_meas_) {
    reset_torque_meas_smoothing_(effort);
    return effort;
  }

  smoothed_torque_meas_ =
    smoothing * smoothed_torque_meas_ + (1.0f - smoothing) * effort;
  return smoothed_torque_meas_;
}

void Actuator::reset_torque_meas_smoothing_(float effort)
{
  smoothed_torque_meas_ = effort;
  has_smoothed_torque_meas_ = true;
}

void Actuator::apply_decoded_feedback_(const can_hardware_common::DecodedFeedback & decoded)
{
  if (decoded.has_state) {
    feedback_ = decoded.state;
    feedback_.torque = smooth_torque_meas_(feedback_.torque);
    has_feedback_ = true;
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

}  // namespace aristo_actuator
