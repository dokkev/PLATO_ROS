#ifndef PLATO_HARDWARE_INTERFACE__DYNAMIXEL_CAN_PROTOCOL_HPP_
#define PLATO_HARDWARE_INTERFACE__DYNAMIXEL_CAN_PROTOCOL_HPP_

#include <cstdint>
#include <optional>

#include <PCANBasic.h>

#include "plato_hardware_interface/utils/can_ids.hpp"

namespace plato_hardware_interface::dynamixel_can_protocol
{

enum class Command : uint8_t
{
  kEnable = CommandByte::START_MOTOR,
  kDisable = CommandByte::STOP_MOTOR,
  kSetPosition = CommandByte::POSITION_CONTROL,
};

enum class Result : uint8_t
{
  kSuccess = ResultByte::SUCCESS,
  kFailure = ResultByte::FAILURE,
  kMotorDisabled = 0x02,
};

struct Response
{
  Command command;
  uint8_t servo_id = 0;
  Result result = Result::kFailure;
};

constexpr uint8_t kLifecycleCommandLength = 2;
constexpr uint8_t kLifecycleResponseLength = 3;
constexpr uint8_t kPositionCommandLength = 8;

// Command frame:
//   DATA[0] = Command
//   DATA[1] = Dynamixel servo/channel ID
//   DATA[2..] = command-specific payload
//
// Response frame:
//   DATA[0] = echoed Command
//   DATA[1] = Dynamixel servo/channel ID
//   DATA[2] = Result
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

bool is_success(Result result);
const char * command_name(Command command);

}  // namespace plato_hardware_interface::dynamixel_can_protocol

#endif  // PLATO_HARDWARE_INTERFACE__DYNAMIXEL_CAN_PROTOCOL_HPP_
