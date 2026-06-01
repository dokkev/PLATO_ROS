#include "aristo_hardware_interface/can_protocol.hpp"

#include <cstring>
#include <stdexcept>

namespace aristo_actuator
{

namespace
{
constexpr uint8_t kCmdReadStates = 0xF1;

bool is_state_frame(const TPCANMsg & frame)
{
  return frame.LEN == 8 || (frame.LEN >= 7 && frame.DATA[0] == kCmdReadStates);
}
}  // namespace

CANProtocol::CANProtocol(
  const can_hardware_common::ActuatorCoreConfig & config)
: config_(config),
  encoder_(config.gear_ratio, config.can_tx_id),
  decoder_(),
  onoff_msg_(make_message_(config.can_tx_id, 8)),
  cmd_msg_(make_message_(config.can_tx_id, 8)),
  config_msg_(make_message_(config.can_tx_id, 7))
{
  validate_direction_(config_);
}

std::optional<actuator::TxCommand> CANProtocol::make_impedance_command(
  const can_hardware_common::ActuatorTarget & joint_target)
{
  encoder_.set_impedance(
    cmd_msg_,
    map_joint_to_motor_frame_(joint_target.position),
    map_joint_to_motor_frame_(joint_target.velocity),
    joint_target.stiffness,
    joint_target.damping,
    map_joint_to_motor_frame_(joint_target.torque));
  return actuator::TxCommand{cmd_msg_};
}

actuator::TxCommand CANProtocol::make_torque_command(float joint_torque)
{
  encoder_.set_impedance(
    cmd_msg_,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    map_joint_to_motor_frame_(joint_torque));
  return actuator::TxCommand{cmd_msg_};
}

std::optional<can_hardware_common::DecodedFeedback> CANProtocol::decode(
  const TPCANMsg & frame)
{
  if (frame.MSGTYPE != PCAN_MESSAGE_STANDARD || !is_state_frame(frame)) {
    return std::nullopt;
  }

  float motor_position = 0.0f;
  float motor_velocity = 0.0f;
  float motor_torque = 0.0f;
  bool in_oc_mode = false;
  bool has_fault = false;
  float unused_kp = 0.0f;
  float unused_kd = 0.0f;

  decoder_.get_states(
    frame,
    motor_position,
    motor_velocity,
    unused_kp,
    unused_kd,
    motor_torque,
    in_oc_mode,
    has_fault);

  can_hardware_common::DecodedFeedback decoded;
  decoded.has_state = true;
  decoded.state.position = map_motor_to_joint_frame_(motor_position);
  decoded.state.velocity = map_motor_to_joint_frame_(motor_velocity);
  decoded.state.torque = map_motor_to_joint_frame_(motor_torque);
  decoded.in_oc_mode = in_oc_mode;
  decoded.has_fault = has_fault;
  decoded.motor_enabled = in_oc_mode && !has_fault;
  return decoded;
}

actuator::TxCommand CANProtocol::make_enable_motor_command()
{
  encoder_.start_motor(onoff_msg_);
  return actuator::TxCommand{onoff_msg_};
}

actuator::TxCommand CANProtocol::make_disable_motor_command()
{
  encoder_.stop_motor(onoff_msg_);
  return actuator::TxCommand{onoff_msg_};
}

actuator::TxCommand CANProtocol::make_stop_control_command()
{
  encoder_.stop_control(onoff_msg_);
  return actuator::TxCommand{onoff_msg_};
}

actuator::TxCommand CANProtocol::make_zero_position_command()
{
  encoder_.set_zero_position(onoff_msg_);
  return actuator::TxCommand{onoff_msg_};
}

actuator::TxCommand CANProtocol::make_default_can_limits_command()
{
  encoder_.set_default_can_limits(config_msg_);
  return actuator::TxCommand{config_msg_};
}

TPCANMsg CANProtocol::make_message_(uint32_t can_id, uint8_t len)
{
  TPCANMsg msg;
  std::memset(&msg, 0, sizeof(msg));
  msg.ID = can_id;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = len > 8 ? 8 : len;
  return msg;
}

void CANProtocol::validate_direction_(
  const can_hardware_common::ActuatorCoreConfig & config)
{
  if (config.direction != 1 && config.direction != -1) {
    throw std::invalid_argument("Aristo actuator direction must be +1 or -1");
  }
}

float CANProtocol::map_joint_to_motor_frame_(float joint_value, bool apply_offset) const
{
  const float direction = static_cast<float>(config_.direction);
  return apply_offset ?
         (joint_value * direction) + config_.position_offset :
         joint_value * direction;
}

float CANProtocol::map_motor_to_joint_frame_(float motor_value, bool apply_offset) const
{
  const float direction = static_cast<float>(config_.direction);
  return apply_offset ?
         (motor_value - config_.position_offset) * direction :
         motor_value * direction;
}

}  // namespace aristo_actuator
