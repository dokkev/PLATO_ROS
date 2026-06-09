#ifndef ARISTO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_
#define ARISTO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_

#include <optional>

#include <PCANBasic.h>

#include "aristo_hardware_interface/mit_can_protocol.hpp"
#include "can_hardware_common/actuator_protocol.hpp"

namespace aristo_actuator
{

inline constexpr float mNmToNm = 1.0e-3f;
inline constexpr float nmToMNm = 1.0e3f;
inline constexpr float driverKt = 0.52f;
inline constexpr float motorKt = 0.52f / 8.0f;
inline constexpr float ktCmdScale = driverKt / motorKt;
inline constexpr float ktFbScale = motorKt / driverKt;
inline constexpr float cmdEffortScale = mNmToNm * ktCmdScale;
inline constexpr float fbEffortScale = nmToMNm * ktFbScale;
inline constexpr uint8_t driverGear = 8;

class CANProtocol : public can_hardware_common::ActuatorProtocol
{
public:
  explicit CANProtocol(const can_hardware_common::ActuatorCoreConfig & config);

  std::optional<actuator::TxCommand> make_impedance_command(
    const can_hardware_common::ActuatorTarget & joint_target) override;
  actuator::TxCommand make_torque_command(float joint_torque) override;
  std::optional<can_hardware_common::DecodedFeedback> decode(
    const TPCANMsg & frame) override;

  actuator::TxCommand make_enable_motor_command();
  actuator::TxCommand make_disable_motor_command();
  actuator::TxCommand make_stop_control_command();
  actuator::TxCommand make_zero_position_command();
  actuator::TxCommand make_default_can_limits_command();
  actuator::TxCommand make_read_motor_params_command();
  actuator::TxCommand make_read_can_limits_command();
  actuator::TxCommand make_read_state_command();
  bool try_update_motor_params(const TPCANMsg & frame);
  bool try_update_can_limits(const TPCANMsg & frame);
  const mit_can_protocol::MotorParams & motor_params() const { return motor_params_; }
  const mit_can_protocol::MitLimits & active_limits() const { return active_limits_; }
  bool has_motor_params() const { return has_motor_params_; }
  bool has_active_limits() const { return has_active_limits_; }

private:
  static TPCANMsg make_message_(uint32_t can_id, uint8_t len);
  static void validate_direction_(const can_hardware_common::ActuatorCoreConfig & config);

  float to_protocol_(float joint_value, bool apply_offset = false) const;
  float from_protocol_(float protocol_value, bool apply_offset = false) const;

  can_hardware_common::ActuatorCoreConfig config_;
  mit_can_protocol::MsgEncoder encoder_;
  mit_can_protocol::MsgDecoder decoder_;
  TPCANMsg onoff_msg_{};
  TPCANMsg cmd_msg_{};
  TPCANMsg config_msg_{};
  TPCANMsg read_msg_{};
  mit_can_protocol::MotorParams motor_params_{};
  mit_can_protocol::MitLimits active_limits_{};
  bool has_motor_params_ = false;
  bool has_active_limits_ = false;
};

}  // namespace aristo_actuator

#endif  // ARISTO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_
