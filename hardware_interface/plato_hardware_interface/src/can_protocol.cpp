#include "plato_hardware_interface/can_protocol.hpp"

#include <chrono>
#include <cstring>

#include <rclcpp/rclcpp.hpp>

#include "plato_hardware_interface/actuator.hpp"

namespace can_protocol
{

namespace
{
auto logger() { return rclcpp::get_logger("can_protocol"); }

rclcpp::Clock & throttle_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

constexpr uint8_t kAckFrameLength = 2;
constexpr uint8_t kStateFrameLength = 8;

bool is_supported_rx_frame(const TPCANMsg & msg)
{
  return msg.MSGTYPE == PCAN_MESSAGE_STANDARD;
}

const char * command_name(uint8_t command_byte)
{
  switch (command_byte) {
    case CommandByte::START_MOTOR:
      return "START_MOTOR";
    case CommandByte::STOP_MOTOR:
      return "STOP_MOTOR";
    case CommandByte::STOP_CONTROL:
      return "STOP_CONTROL";
    case CommandByte::TORQUE_CONTROL:
      return "TORQUE_CONTROL";
    case CommandByte::SPEED_CONTROL:
      return "SPEED_CONTROL";
    case CommandByte::POSITION_CONTROL:
      return "POSITION_CONTROL";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

bool MsgDecoder::get_result(uint8_t error_byte)
{
  switch (error_byte) {
    case ResultByte::SUCCESS:
      return true;
    case ResultByte::FAILURE:
      RCLCPP_ERROR(logger(), "Motor driver: FAILURE");
      return false;
    case ResultByte::FAILURE_UNKNOWN_COMMAND:
      RCLCPP_ERROR(logger(), "Motor driver: UNKNOWN COMMAND");
      return false;
    case ResultByte::FAILURE_UNKNOWN_ID:
      RCLCPP_ERROR(logger(), "Motor driver: UNKNOWN ID");
      return false;
    case ResultByte::FAILURE_READ_ONLY_REGISTER:
      RCLCPP_ERROR(logger(), "Motor driver: READ ONLY REGISTER");
      return false;
    case ResultByte::FAILURE_UNKNOWN_REGISTER:
      RCLCPP_ERROR(logger(), "Motor driver: UNKNOWN REGISTER");
      return false;
    default:
      RCLCPP_ERROR(logger(), "Motor driver: unknown error byte 0x%02X", error_byte);
      return false;
  }
}

void MsgEncoder::start_motor(TPCANMsg & msg)
{
  msg.DATA[0] = CommandByte::START_MOTOR;
}

void MsgEncoder::stop_motor(TPCANMsg & msg)
{
  msg.DATA[0] = CommandByte::STOP_MOTOR;
}

void MsgEncoder::stop_control(TPCANMsg & msg)
{
  msg.DATA[0] = CommandByte::STOP_CONTROL;
}

void MsgEncoder::set_torque(TPCANMsg & msg, float torque, uint32_t duration)
{
  msg.DATA[0] = CommandByte::TORQUE_CONTROL;
  can_hardware_common::can_protocol_helpers::encode_float_le(msg, torque, 1);
  can_hardware_common::can_protocol_helpers::encode_u24_le(msg, duration, 5);
}

void MsgEncoder::set_position(TPCANMsg & msg, float position, uint32_t duration)
{
  msg.DATA[0] = CommandByte::POSITION_CONTROL;
  can_hardware_common::can_protocol_helpers::encode_float_le(msg, position, 1);
  can_hardware_common::can_protocol_helpers::encode_u24_le(msg, duration, 5);
}

void MsgDecoder::get_states(
  const TPCANMsg & msg,
  uint8_t & temperature,
  float & position,
  float & velocity,
  float & torque) const
{
  temperature = msg.DATA[2];

  const uint16_t pos_int = (static_cast<uint16_t>(msg.DATA[4]) << 8) | msg.DATA[3];
  position = pos_int * 25.0f / 65535.0f - 12.5f;

  const uint16_t velocity_int =
    (static_cast<uint16_t>(msg.DATA[5]) << 4) | ((msg.DATA[6] & 0xF0) >> 4);
  velocity = velocity_int * 130.0f / 4095.0f - 65.0f;

  const uint16_t torque_int = ((msg.DATA[6] & 0x0F) << 8) | msg.DATA[7];
  torque = torque_int * torque_scale_ - torque_offset_;
}

SteadywinProtocol::SteadywinProtocol(
  const can_hardware_common::ActuatorCoreConfig & config)
: tx_id_(config.can_tx_id),
  decoder_(config.gear_ratio, config.torque_constant),
  onoff_msg_(make_message_(config.can_tx_id, 8)),
  cmd_msg_(make_message_(config.can_tx_id, 8))
{
}

actuator::TxCommand SteadywinProtocol::make_enable_motor_command()
{
  MsgEncoder::start_motor(onoff_msg_);
  return {onoff_msg_, std::chrono::milliseconds(100)};
}

actuator::TxCommand SteadywinProtocol::make_disable_motor_command()
{
  MsgEncoder::stop_motor(onoff_msg_);
  return {onoff_msg_, std::chrono::milliseconds(100)};
}

actuator::TxCommand SteadywinProtocol::make_stop_control_command()
{
  MsgEncoder::stop_control(onoff_msg_);
  return {onoff_msg_};
}

actuator::TxCommand SteadywinProtocol::make_torque_command(float motor_torque)
{
  MsgEncoder::set_torque(cmd_msg_, motor_torque, 0);
  return {cmd_msg_};
}

actuator::TxCommand SteadywinProtocol::make_position_command(
  float motor_position,
  uint32_t duration)
{
  MsgEncoder::set_position(cmd_msg_, motor_position, duration);
  return {cmd_msg_};
}

actuator::TxCommand SteadywinProtocol::make_servo_position_command(
  float motor_position,
  uint32_t current_milliamps)
{
  MsgEncoder::set_position(cmd_msg_, motor_position, current_milliamps);
  return {cmd_msg_};
}

void SteadywinProtocol::process_message(const TPCANMsg & msg, plato_actuator::Actuator & actuator)
{
  if (!is_supported_rx_frame(msg)) {
    RCLCPP_WARN_THROTTLE(
      logger(), throttle_clock(), 1000,
      "Ignoring unsupported Plato CAN frame type 0x%02X for actuator ID 0x%02X",
      msg.MSGTYPE, tx_id_);
    return;
  }

  if (msg.LEN < kAckFrameLength) {
    RCLCPP_WARN_THROTTLE(
      logger(), throttle_clock(), 1000,
      "Ignoring short Plato CAN frame for actuator ID 0x%02X: LEN=%u",
      tx_id_, msg.LEN);
    return;
  }

  switch (msg.DATA[0]) {
    case CommandByte::POSITION_CONTROL:
    case CommandByte::SPEED_CONTROL:
    case CommandByte::TORQUE_CONTROL:
      if (msg.LEN < kStateFrameLength) {
        RCLCPP_WARN_THROTTLE(
          logger(), throttle_clock(), 1000,
          "Ignoring short Plato state reply for actuator ID 0x%02X: LEN=%u",
          tx_id_, msg.LEN);
        return;
      }
      if (MsgDecoder::get_result(msg.DATA[1])) {
        uint8_t temperature = 0;
        float motor_position = 0.0f;
        float motor_velocity = 0.0f;
        float motor_torque = 0.0f;
        decoder_.get_states(msg, temperature, motor_position, motor_velocity, motor_torque);
        actuator.set_status_temperature(temperature);
        actuator.apply_motor_feedback(
          motor_position,
          motor_velocity,
          motor_torque,
          true,
          true);
      } else {
        RCLCPP_ERROR(logger(), "Actuator ID 0x%02X control failed", tx_id_);
      }
      break;
    case CommandByte::START_MOTOR:
      if (MsgDecoder::get_result(msg.DATA[1])) {
        actuator.set_motor_enabled(true);
        RCLCPP_INFO(
          logger(),
          "Actuator RX 0x%02X acknowledged %s",
          msg.ID, command_name(msg.DATA[0]));
      } else {
        RCLCPP_ERROR(
          logger(),
          "Actuator RX 0x%02X returned failure for %s",
          msg.ID, command_name(msg.DATA[0]));
      }
      break;
    case CommandByte::STOP_MOTOR:
    case CommandByte::STOP_CONTROL:
      if (MsgDecoder::get_result(msg.DATA[1])) {
        actuator.set_motor_enabled(false);
        RCLCPP_INFO(
          logger(),
          "Actuator RX 0x%02X acknowledged %s",
          msg.ID, command_name(msg.DATA[0]));
      } else {
        RCLCPP_ERROR(
          logger(),
          "Actuator RX 0x%02X returned failure for %s",
          msg.ID, command_name(msg.DATA[0]));
      }
      break;
    default:
      break;
  }
}

TPCANMsg SteadywinProtocol::make_message_(uint32_t can_id, uint8_t len)
{
  TPCANMsg msg;
  std::memset(&msg, 0, sizeof(msg));
  msg.ID = can_id;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = len > 8 ? 8 : len;
  return msg;
}

}  // namespace can_protocol
