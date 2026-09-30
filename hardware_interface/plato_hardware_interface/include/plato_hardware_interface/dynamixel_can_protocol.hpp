#ifndef PLATO_HARDWARE_INTERFACE__DYNAMIXEL_CAN_PROTOCOL_HPP_
#define PLATO_HARDWARE_INTERFACE__DYNAMIXEL_CAN_PROTOCOL_HPP_

#include <cstdint>
#include <optional>

#include <PCANBasic.h>

#include "plato_hardware_interface/gim3505_protocol.hpp"

namespace plato_hardware_interface::dynamixel_can_protocol
{

enum class Command : uint8_t
{
  kEnable = plato_hardware_interface::gim3505_protocol::CommandByte::START_MOTOR,
  kDisable = plato_hardware_interface::gim3505_protocol::CommandByte::STOP_MOTOR,
  kSetPosition = plato_hardware_interface::gim3505_protocol::CommandByte::POSITION_CONTROL,
};

enum class Result : uint8_t
{
  kSuccess = plato_hardware_interface::gim3505_protocol::ResultByte::SUCCESS,
  kFailure = plato_hardware_interface::gim3505_protocol::ResultByte::FAILURE,
  kMotorDisabled = 0x02,
};

struct Response
{
  Command command;
  uint8_t servo_id = 0;
  Result result = Result::kFailure;
};

struct StateFeedback
{
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
};

constexpr uint8_t kLifecycleCommandLength = 2;
constexpr uint8_t kLifecycleResponseLength = 3;
constexpr uint8_t kPositionCommandLength = 8;
constexpr uint8_t kStateResponseLength = 8;

// Command frame:
//   DATA[0] = Command
//   DATA[1] = Dynamixel servo/channel ID
//   DATA[2..] = command-specific payload
//
// Response frame:
//   DATA[0] = echoed Command
//   DATA[1] = Dynamixel servo/channel ID
//   DATA[2] = Result
//   For POSITION_CONTROL responses, DATA[3..7] contains packed state feedback.
//   DATA[3..4] = uint16 position, mapped from [-12.5, 12.5] rad.
//   DATA[5..6] = 12-bit velocity, mapped from [-65, 65] RPM.
//   DATA[6..7] = 12-bit torque, mapped using torque_constant and gear_ratio.
//
// Position command payload:
//   DATA[2..5] = float32 goal position, little-endian, radians
//   DATA[6..7] = uint16 current limit, little-endian, milliamps
TPCANMsg make_enable_command(uint32_t mcu_can_id, uint8_t servo_id);
TPCANMsg make_disable_command(uint32_t mcu_can_id, uint8_t servo_id);
TPCANMsg make_position_command(
  uint32_t mcu_can_id,
  uint8_t servo_id,
  float position_rad,
  uint16_t current_milliamps);

std::optional<Response> decode_response(const TPCANMsg & frame);
std::optional<Response> decode_lifecycle_response(const TPCANMsg & frame);
std::optional<StateFeedback> decode_state_feedback(
  const TPCANMsg & frame,
  float torque_constant,
  float gear_ratio);

bool is_success(Result result);
const char * command_name(Command command);

}  // namespace plato_hardware_interface::dynamixel_can_protocol

#endif  // PLATO_HARDWARE_INTERFACE__DYNAMIXEL_CAN_PROTOCOL_HPP_
