#ifndef ARISTO_HARDWARE_INTERFACE__MIT_CAN_PROTOCOL_HPP_
#define ARISTO_HARDWARE_INTERFACE__MIT_CAN_PROTOCOL_HPP_

#include <cstdint>
#include <optional>
#include "PCANBasic.h"

namespace mit_can_protocol
{

inline constexpr float KP_MAX = 500.0f;
inline constexpr float KD_MAX = 5.0f;
inline constexpr float POS_MAX = 95.5f;
inline constexpr float VEL_MAX = 45.0f;
inline constexpr float T_MAX = 18.0f;

struct MitLimits
{
  float pos_max_rad = POS_MAX;
  float vel_max_rad_s = VEL_MAX;
  float t_max_nm = T_MAX;
};

struct MotorParams
{
  uint8_t pole_pairs = 0;
  float torque_constant_nm_per_a = 0.0f;
  uint8_t gear_ratio = 0;
};

class MsgEncoder
{
public:
  explicit MsgEncoder(uint8_t tx_id)
  : tx_id_(tx_id)
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
  void read_can_limits(TPCANMsg & msg);
  void read_motor_params(TPCANMsg & msg);
  void read_states(TPCANMsg & msg);
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
  void set_active_limits(const MitLimits & limits);
  const MitLimits & active_limits() const { return active_limits_; }

private:
  uint8_t tx_id_;
  MitLimits active_limits_{};
};

class MsgDecoder
{
public:
  bool get_states(
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
  std::optional<MitLimits> get_limits(const TPCANMsg & msg) const;
  bool get_motor_params(const TPCANMsg & msg, MotorParams & params) const;
  void set_active_limits(const MitLimits & limits) { active_limits_ = limits; }
  const MitLimits & active_limits() const { return active_limits_; }

private:
  MitLimits active_limits_{};
};

}  // namespace mit_can_protocol

#endif  // ARISTO_HARDWARE_INTERFACE__MIT_CAN_PROTOCOL_HPP_
