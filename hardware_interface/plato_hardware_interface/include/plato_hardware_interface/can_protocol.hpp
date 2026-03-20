#ifndef PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_
#define PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_

#include <cstdint>
#include "PCANBasic.h"

#include "can_hardware_common/actuator.hpp"
#include "can_hardware_common/utils/can_helper.hpp"
#include "plato_hardware_interface/utils/can_ids.hpp"

namespace plato_actuator
{
class Actuator;
}

namespace can_protocol
{

// Legacy Plato v2 default used for GIM3505 position-control commands.
constexpr uint32_t kDefaultQddPositionDurationMs = 10;

class MsgEncoder
{
public:
  static void start_motor(TPCANMsg & msg);
  static void stop_motor(TPCANMsg & msg);
  static void stop_control(TPCANMsg & msg);
  static void set_torque(TPCANMsg & msg, float torque, uint32_t duration);
  static void set_position(TPCANMsg & msg, float position, uint32_t duration);
};

class MsgDecoder
{
public:
  MsgDecoder(float gear_ratio, float torque_constant)
  : torque_scale_(450.0f * torque_constant * gear_ratio / 4095.0f),
    torque_offset_(225.0f * torque_constant * gear_ratio)
  {
  }

  static bool get_result(uint8_t error_byte);

  void get_states(
    const TPCANMsg & msg,
    uint8_t & temperature,
    float & position,
    float & velocity,
    float & torque) const;

private:
  float torque_scale_;
  float torque_offset_;
};

class SteadywinProtocol
{
public:
  explicit SteadywinProtocol(const can_hardware_common::ActuatorCoreConfig & config);

  actuator::TxCommand make_enable_motor_command();
  actuator::TxCommand make_disable_motor_command();
  actuator::TxCommand make_stop_control_command();
  actuator::TxCommand make_torque_command(float motor_torque);
  actuator::TxCommand make_position_command(
    float motor_position,
    uint32_t duration = kDefaultQddPositionDurationMs);
  actuator::TxCommand make_servo_position_command(float motor_position, uint32_t current_milliamps = 0);
  void process_message(const TPCANMsg & msg, plato_actuator::Actuator & actuator);

private:
  static TPCANMsg make_message_(uint32_t can_id, uint8_t len);

  uint8_t tx_id_;
  MsgDecoder decoder_;
  TPCANMsg onoff_msg_;
  TPCANMsg cmd_msg_;
};

}  // namespace can_protocol

#endif  // PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_
