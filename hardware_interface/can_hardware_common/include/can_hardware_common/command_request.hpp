#ifndef CAN_HARDWARE_COMMON__COMMAND_REQUEST_HPP_
#define CAN_HARDWARE_COMMON__COMMAND_REQUEST_HPP_

#include <cstdint>
#include <optional>

#include "can_hardware_common/can_frame_types.hpp"

namespace can_hardware_common
{

struct ResponseExpectation
{
  bool enabled = false;
  uint32_t expected_rx_id = 0;
  std::optional<uint8_t> expected_opcode;
  std::optional<uint8_t> expected_result_byte;

  bool matches(const RxFrame & rx_frame) const
  {
    if (!enabled) {
      return true;
    }
    if (rx_frame.ID != expected_rx_id) {
      return false;
    }
    if (expected_opcode.has_value()) {
      if (rx_frame.LEN < 1 || rx_frame.DATA[0] != *expected_opcode) {
        return false;
      }
    }
    if (expected_result_byte.has_value()) {
      if (rx_frame.LEN < 2 || rx_frame.DATA[1] != *expected_result_byte) {
        return false;
      }
    }
    return true;
  }
};

struct CommandRequest
{
  uint32_t key = 0;
  TxFrame tx_frame{};
  ResponseExpectation response{};
  bool latest_only = true;
};

}  // namespace can_hardware_common

#endif  // CAN_HARDWARE_COMMON__COMMAND_REQUEST_HPP_
