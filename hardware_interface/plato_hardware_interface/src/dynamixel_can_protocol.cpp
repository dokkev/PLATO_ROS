#include "plato_hardware_interface/dynamixel_can_protocol.hpp"

#include <cmath>
#include <cstddef>
#include <cstring>

#include "can_hardware_common/utils/can_helper.hpp"

namespace plato_hardware_interface::dynamixel_can_protocol
{
namespace
{

constexpr float kPositionRawMaximum = 65535.0f;
constexpr float kPackedStateRawMaximum = 4095.0f;
constexpr float kPositionMinimumRad = -12.5f;
constexpr float kPositionRangeRad = 25.0f;
constexpr float kVelocityMinimumRpm = -65.0f;
constexpr float kVelocityRangeRpm = 130.0f;
constexpr float kRadiansPerRevolution = 2.0f * 3.14159265358979323846f;
constexpr float kSecondsPerMinute = 60.0f;

TPCANMsg make_standard_frame(uint32_t can_id, uint8_t len)
{
  TPCANMsg frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.ID = can_id;
  frame.MSGTYPE = PCAN_MESSAGE_STANDARD;
  frame.LEN = len;
  return frame;
}

void encode_u16_le(TPCANMsg & frame, uint16_t value, std::size_t offset)
{
  frame.DATA[offset] = value & 0xFF;
  frame.DATA[offset + 1] = (value >> 8) & 0xFF;
}

std::optional<Command> decode_command(uint8_t command_byte)
{
  switch (command_byte) {
    case static_cast<uint8_t>(Command::kEnable):
      return Command::kEnable;
    case static_cast<uint8_t>(Command::kDisable):
      return Command::kDisable;
    case static_cast<uint8_t>(Command::kSetPosition):
      return Command::kSetPosition;
    default:
      return std::nullopt;
  }
}

std::optional<Result> decode_result(uint8_t result_byte)
{
  switch (result_byte) {
    case static_cast<uint8_t>(Result::kSuccess):
      return Result::kSuccess;
    case static_cast<uint8_t>(Result::kFailure):
      return Result::kFailure;
    case static_cast<uint8_t>(Result::kMotorDisabled):
      return Result::kMotorDisabled;
    default:
      return std::nullopt;
  }
}

TPCANMsg make_lifecycle_command(uint32_t mcu_can_id, uint8_t servo_id, Command command)
{
  auto frame = make_standard_frame(mcu_can_id, kLifecycleCommandLength);
  frame.DATA[0] = static_cast<uint8_t>(command);
  frame.DATA[1] = servo_id;
  return frame;
}

}  // namespace

TPCANMsg make_enable_command(uint32_t mcu_can_id, uint8_t servo_id)
{
  return make_lifecycle_command(mcu_can_id, servo_id, Command::kEnable);
}

TPCANMsg make_disable_command(uint32_t mcu_can_id, uint8_t servo_id)
{
  return make_lifecycle_command(mcu_can_id, servo_id, Command::kDisable);
}

TPCANMsg make_position_command(
  uint32_t mcu_can_id,
  uint8_t servo_id,
  float position_rad,
  uint16_t current_milliamps)
{
  auto frame = make_standard_frame(mcu_can_id, kPositionCommandLength);
  frame.DATA[0] = static_cast<uint8_t>(Command::kSetPosition);
  frame.DATA[1] = servo_id;
  can_hardware_common::can_protocol_helpers::encode_float_le(frame, position_rad, 2);
  encode_u16_le(frame, current_milliamps, 6);
  return frame;
}

std::optional<Response> decode_response(const TPCANMsg & frame)
{
  if (frame.MSGTYPE != PCAN_MESSAGE_STANDARD || frame.LEN < kLifecycleResponseLength) {
    return std::nullopt;
  }

  const auto command = decode_command(frame.DATA[0]);
  const auto result = decode_result(frame.DATA[2]);
  if (!command || !result) {
    return std::nullopt;
  }

  return Response{*command, frame.DATA[1], *result};
}

std::optional<Response> decode_lifecycle_response(const TPCANMsg & frame)
{
  return decode_response(frame);
}

std::optional<StateFeedback> decode_state_feedback(
  const TPCANMsg & frame,
  float torque_constant,
  float gear_ratio)
{
  const auto response = decode_response(frame);
  if (
    !response || response->command != Command::kSetPosition ||
    frame.LEN < kStateResponseLength)
  {
    return std::nullopt;
  }

  const float torque_range = 450.0f * torque_constant * gear_ratio;
  if (!std::isfinite(torque_range) || torque_range <= 0.0f) {
    return std::nullopt;
  }

  const uint16_t position_raw =
    static_cast<uint16_t>(frame.DATA[3]) |
    (static_cast<uint16_t>(frame.DATA[4]) << 8);
  const uint16_t velocity_raw =
    (static_cast<uint16_t>(frame.DATA[5]) << 4) |
    ((static_cast<uint16_t>(frame.DATA[6]) >> 4) & 0x0FU);
  const uint16_t torque_raw =
    (static_cast<uint16_t>(frame.DATA[6] & 0x0FU) << 8) |
    static_cast<uint16_t>(frame.DATA[7]);

  const float position =
    static_cast<float>(position_raw) * kPositionRangeRad / kPositionRawMaximum +
    kPositionMinimumRad;
  const float velocity_rpm =
    static_cast<float>(velocity_raw) * kVelocityRangeRpm / kPackedStateRawMaximum +
    kVelocityMinimumRpm;
  const float torque =
    static_cast<float>(torque_raw) * torque_range / kPackedStateRawMaximum -
    (225.0f * torque_constant * gear_ratio);

  return StateFeedback{
    position,
    velocity_rpm * kRadiansPerRevolution / kSecondsPerMinute,
    torque};
}

bool is_success(Result result)
{
  return result == Result::kSuccess;
}

const char * command_name(Command command)
{
  switch (command) {
    case Command::kEnable:
      return "ENABLE";
    case Command::kDisable:
      return "DISABLE";
    case Command::kSetPosition:
      return "SET_POSITION";
    default:
      return "UNKNOWN";
  }
}

}  // namespace plato_hardware_interface::dynamixel_can_protocol
