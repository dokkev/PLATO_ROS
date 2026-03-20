#ifndef ARISTO_HARDWARE_INTERFACE__MIT_CAN_PROTOCOL_HPP_
#define ARISTO_HARDWARE_INTERFACE__MIT_CAN_PROTOCOL_HPP_

#include <cstdint>
#include <optional>

#include "PCANBasic.h"

#include "can_hardware_common/actuator.hpp"
#include "can_hardware_common/utils/can_helper.hpp"

namespace aristo_actuator
{
class Actuator;
}

namespace mit_can_protocol
{

inline constexpr float KP_MAX = 500.0f;
inline constexpr float KD_MAX = 5.0f;
inline constexpr float POS_MAX = 95.5f;
inline constexpr float VEL_MAX = 45.0f;
inline constexpr float T_MAX = 18.0f;

class MsgEncoder
{
public:
  MsgEncoder(float gear_ratio, uint8_t tx_id)
  : gear_ratio_(gear_ratio), tx_id_(tx_id)
  {
  }

  void set_can_limits(
    TPCANMsg & msg,
    float pos_max_rad,
    float vel_max_rps,
    float tq_max_nm,
    bool set_pos,
    bool set_vel,
    bool set_tq);
  void set_default_can_limits(TPCANMsg & msg);
  void set_zero_position(TPCANMsg & msg);
  void start_motor(TPCANMsg & msg);
  void stop_motor(TPCANMsg & msg);
  void stop_control(TPCANMsg & msg);
  void clear_fault(TPCANMsg & msg);
  void set_impedance(
    TPCANMsg & msg,
    float position_rad,
    float velocity_rps,
    float kp,
    float kd,
    float torque_nm);

private:
  float gear_ratio_;
  uint8_t tx_id_;
};

class MsgDecoder
{
public:
  void get_states(
    const TPCANMsg & msg,
    float & position,
    float & velocity,
    float & kp,
    float & kd,
    float & torque,
    bool & in_oc_mode,
    bool & has_fault) const;
  void get_limits(
    const TPCANMsg & msg,
    float & pos_max_rad,
    float & vel_max_rps,
    float & tq_max_nm) const;
};

class MITProtocol
{
public:
  explicit MITProtocol(const can_hardware_common::ActuatorCoreConfig & config);

  actuator::TxCommand make_enable_motor_command();
  actuator::TxCommand make_disable_motor_command();
  actuator::TxCommand make_stop_control_command();
  actuator::TxCommand make_torque_command(float motor_torque);
  std::optional<actuator::TxCommand> make_impedance_command(
    const can_hardware_common::ActuatorTarget & motor_target);
  actuator::TxCommand make_zero_position_command();
  actuator::TxCommand make_default_can_limits_command();
  void process_message(const TPCANMsg & msg, aristo_actuator::Actuator & actuator);

private:
  static TPCANMsg make_message_(uint32_t can_id, uint8_t len);

  uint8_t tx_id_;
  MsgEncoder encoder_;
  MsgDecoder decoder_;
  TPCANMsg onoff_msg_;
  TPCANMsg cmd_msg_;
  TPCANMsg config_msg_;
};

}  // namespace mit_can_protocol

#endif  // ARISTO_HARDWARE_INTERFACE__MIT_CAN_PROTOCOL_HPP_
