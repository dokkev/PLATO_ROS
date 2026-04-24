#ifndef ARISTO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_
#define ARISTO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_

#include <optional>

#include "aristo_hardware_interface/mit_can_protocol.hpp"
#include "can_hardware_common/actuator_protocol.hpp"

namespace aristo_actuator
{

class CANProtocol : public can_hardware_common::ActuatorProtocol
{
public:
  explicit CANProtocol(const can_hardware_common::ActuatorCoreConfig & config);

  std::optional<actuator::TxCommand> make_impedance_command(
    const can_hardware_common::ActuatorTarget & joint_target) override;
  actuator::TxCommand make_torque_command(float joint_torque) override;
  std::optional<can_hardware_common::DecodedFeedback> decode(
    const can_hardware_common::RxFrame & frame) override;

  actuator::TxCommand make_enable_motor_command();
  actuator::TxCommand make_disable_motor_command();
  actuator::TxCommand make_stop_control_command();
  actuator::TxCommand make_zero_position_command();
  actuator::TxCommand make_default_can_limits_command();

private:
  static TPCANMsg make_message_(uint32_t can_id, uint8_t len);
  static void validate_direction_(const can_hardware_common::ActuatorCoreConfig & config);

  float map_joint_to_motor_frame_(float joint_value, bool apply_offset = false) const;
  float map_motor_to_joint_frame_(float motor_value, bool apply_offset = false) const;

  can_hardware_common::ActuatorCoreConfig config_;
  mit_can_protocol::MsgEncoder encoder_;
  mit_can_protocol::MsgDecoder decoder_;
  TPCANMsg onoff_msg_{};
  TPCANMsg cmd_msg_{};
  TPCANMsg config_msg_{};
};

}  // namespace aristo_actuator

#endif  // ARISTO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_
