#ifndef ARISTO_HARDWARE_INTERFACE__MIT_CAN_PROTOCOL_HPP_
#define ARISTO_HARDWARE_INTERFACE__MIT_CAN_PROTOCOL_HPP_

#include <cstdint>
#include "PCANBasic.h"

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

}  // namespace mit_can_protocol

#endif  // ARISTO_HARDWARE_INTERFACE__MIT_CAN_PROTOCOL_HPP_
