#ifndef CAN_HARDWARE_COMMON__ACTUATOR_HPP_
#define CAN_HARDWARE_COMMON__ACTUATOR_HPP_

#include <chrono>
#include <cstdint>
#include <limits>

#include "PCANBasic.h"

namespace can_hardware_common
{

struct ActuatorTarget
{
  float position = 0.0f;
  float velocity = 0.0f;
  float stiffness = 0.0f;
  float damping = 0.0f;
  float torque = 0.0f;
};

struct ActuatorState
{
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
};

struct ActuatorStatus
{
  uint8_t temperature = 0;
  bool in_oc_mode = false;
  bool has_fault = false;
};

struct ActuatorCoreConfig
{
  uint8_t can_tx_id = 0;
  uint8_t can_rx_id = 0;
  float position_offset = 0.0f;
  int8_t direction = 1;
  float torque_constant = 0.0f;
  float gear_ratio = 0.0f;
};

// ── Reply/retry specification for CAN commands ──

struct ReplySpec
{
  bool enabled = true;
  uint32_t expected_rx_id = 0;
  uint8_t expected_opcode = 0;
  uint8_t success_byte = 0x00;
};

struct RetryPolicy
{
  std::chrono::microseconds holdoff = std::chrono::microseconds(500);
  std::size_t max_attempts = 0;  // 0 = unlimited within service budget
  bool latest_only = true;
};

struct CommandRequest
{
  uint32_t key = 0;
  TPCANMsg frame{};
  ReplySpec reply{};
  RetryPolicy retry{};
};

}  // namespace can_hardware_common

namespace actuator
{

struct Limits
{
  float position_limit_max = std::numeric_limits<float>::infinity();
  float position_limit_min = -std::numeric_limits<float>::infinity();
  float velocity_limit = std::numeric_limits<float>::infinity();
  float effort_limit = std::numeric_limits<float>::infinity();
  float stiffness_limit = std::numeric_limits<float>::infinity();
  float damping_limit = std::numeric_limits<float>::infinity();
};

struct Config
{
  can_hardware_common::ActuatorCoreConfig core;
  Limits limits;
};

// Legacy TxCommand kept for protocol layer. Actuator wraps this into CommandRequest.
struct TxCommand
{
  TPCANMsg frame{};
  uint8_t expected_response_opcode = 0;
};

}  // namespace actuator

#endif  // CAN_HARDWARE_COMMON__ACTUATOR_HPP_
