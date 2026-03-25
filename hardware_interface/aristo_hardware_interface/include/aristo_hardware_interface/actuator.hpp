#ifndef ARISTO_HARDWARE_INTERFACE__ACTUATOR_HPP_
#define ARISTO_HARDWARE_INTERFACE__ACTUATOR_HPP_

#include <memory>
#include <optional>

#include "aristo_hardware_interface/hardware_config/actuator_config.hpp"
#include "can_hardware_common/actuator.hpp"

namespace mit_can_protocol
{
class MITProtocol;
}

namespace aristo_actuator
{

struct Config
{
  can_hardware_common::ActuatorCoreConfig core;
  actuator::Limits limits;
};

class Actuator
{
public:
  explicit Actuator(const Config & config);
  Actuator(const Actuator &) = delete;
  Actuator & operator=(const Actuator &) = delete;
  Actuator(Actuator &&) noexcept;
  Actuator & operator=(Actuator &&) noexcept;
  ~Actuator();

  actuator::TxCommand enable_motor();
  actuator::TxCommand disable_motor();
  actuator::TxCommand stop_control();
  actuator::TxCommand set_current_position_as_zero();
  actuator::TxCommand set_default_can_limits();

  actuator::TxCommand set_joint_torque(float joint_torque);
  std::optional<actuator::TxCommand> set_joint_impedance(
    const can_hardware_common::ActuatorTarget & joint_target);

  void process_message(const TPCANMsg & msg);

  const Config & get_config() const { return config_; }
  const can_hardware_common::ActuatorState & get_feedback() const { return feedback_; }
  const can_hardware_common::ActuatorStatus & get_status() const { return status_; }

  uint32_t get_tx_id() const { return config_.core.can_tx_id; }
  uint32_t get_rx_id() const { return config_.core.can_rx_id; }
  float get_motor_position() const { return motor_position_; }
  bool is_enabled() const { return motor_enabled_; }
  bool has_feedback() const { return has_feedback_; }

  void set_motor_enabled(bool enabled) { motor_enabled_ = enabled; }
  void set_fault_flags(bool in_oc_mode, bool has_fault)
  {
    status_.in_oc_mode = in_oc_mode;
    status_.has_fault = has_fault;
    motor_enabled_ = in_oc_mode && !has_fault;
  }
  void set_motor_position_raw(float motor_position) { motor_position_ = motor_position; }
  void apply_motor_feedback(
    float motor_position,
    float motor_velocity,
    float motor_torque,
    bool position_has_offset,
    bool velocity_is_rpm);

private:
  enum class SoftLimitState
  {
    kOperational,
    kUpperLimit,
    kLowerLimit,
    kOverLimit,
  };

  float clamp_torque_near_bounds_(float joint_torque) const;
  void determine_current_state_();
  void clamp_impedance_target_(can_hardware_common::ActuatorTarget & joint_target) const;
  float map_joint_to_motor_frame_(float joint_value, bool apply_offset = false) const;
  float map_motor_to_joint_frame_(float motor_value, bool apply_offset = false) const;
  static can_hardware_common::ActuatorTarget make_motor_impedance_target_(
    const can_hardware_common::ActuatorTarget & joint_target,
    const Actuator & actuator);

  Config config_;
  std::unique_ptr<mit_can_protocol::MITProtocol> protocol_;
  can_hardware_common::ActuatorState feedback_;
  can_hardware_common::ActuatorStatus status_;
  float motor_position_ = 0.0f;
  bool motor_enabled_ = false;
  bool has_feedback_ = false;
  SoftLimitState control_state_ = SoftLimitState::kOperational;

  static constexpr float kJointLimitSafetyMargin = 0.05f;
  static constexpr float kSoftLimitMargin = 0.174f;
  static constexpr float kSoftLimitHysteresis = 0.02f;
};

namespace detail
{

inline can_hardware_common::ActuatorCoreConfig make_base_config(
  uint8_t tx_id,
  uint8_t rx_id,
  float offset,
  char direction,
  float torque_constant,
  float gear_ratio)
{
  return {tx_id, rx_id, offset, direction, torque_constant, gear_ratio};
}

inline actuator::Limits make_limits(
  float position_limit_max,
  float position_limit_min,
  float velocity_limit,
  float effort_limit,
  float stiffness_limit,
  float damping_limit)
{
  actuator::Limits limits;
  limits.position_limit_max = position_limit_max;
  limits.position_limit_min = position_limit_min;
  limits.velocity_limit = velocity_limit;
  limits.effort_limit = effort_limit;
  limits.stiffness_limit = stiffness_limit;
  limits.damping_limit = damping_limit;
  return limits;
}

}  // namespace detail

namespace ActuatorConfigFactory
{

inline Config create_thumb_roll_config()
{
  return {
    detail::make_base_config(
      MotorTxID::MOTOR1,
      MotorRxID::MOTOR1,
      MotorOffset::MOTOR1,
      MotorDirection::MOTOR1,
      GIM3505::TORQUE_CONSTANT,
      GIM3505::GEAR_RATIO),
    detail::make_limits(
      JointPositionLimit::THUMB_ROLL_MAX,
      JointPositionLimit::THUMB_ROLL_MIN,
      JointVelocityLimit::THUMB_ROLL,
      JointEffortLimit::THUMB_ROLL,
      JointStiffnessLimit::THUMB_ROLL,
      JointDampingLimit::THUMB_ROLL)
  };
}

inline Config create_thumb_yaw_config()
{
  return {
    detail::make_base_config(
      MotorTxID::MOTOR2,
      MotorRxID::MOTOR2,
      MotorOffset::MOTOR2,
      MotorDirection::MOTOR2,
      GIM3505::TORQUE_CONSTANT,
      GIM3505::GEAR_RATIO),
    detail::make_limits(
      JointPositionLimit::THUMB_YAW_MAX,
      JointPositionLimit::THUMB_YAW_MIN,
      JointVelocityLimit::THUMB_YAW,
      JointEffortLimit::THUMB_YAW,
      JointStiffnessLimit::THUMB_YAW,
      JointDampingLimit::THUMB_YAW)
  };
}

inline Config create_mcp_config(uint8_t motor_num)
{
  const uint8_t tx_id = MotorTxID::MOTOR1 + motor_num - 1;
  const uint8_t rx_id = MotorRxID::MOTOR1 + motor_num - 1;
  const float * offsets = &MotorOffset::MOTOR1;
  const char * directions = &MotorDirection::MOTOR1;

  return {
    detail::make_base_config(
      tx_id,
      rx_id,
      offsets[motor_num - 1],
      directions[motor_num - 1],
      GIM3505::TORQUE_CONSTANT,
      GIM3505::GEAR_RATIO),
    detail::make_limits(
      JointPositionLimit::MCP_MAX,
      JointPositionLimit::MCP_MIN,
      JointVelocityLimit::MCP,
      JointEffortLimit::MCP,
      JointStiffnessLimit::MCP,
      JointDampingLimit::MCP)
  };
}

inline Config create_pip_config(uint8_t motor_num)
{
  const uint8_t tx_id = MotorTxID::MOTOR1 + motor_num - 1;
  const uint8_t rx_id = MotorRxID::MOTOR1 + motor_num - 1;
  const float * offsets = &MotorOffset::MOTOR1;
  const char * directions = &MotorDirection::MOTOR1;

  return {
    detail::make_base_config(
      tx_id,
      rx_id,
      offsets[motor_num - 1],
      directions[motor_num - 1],
      GIM3505::TORQUE_CONSTANT,
      GIM3505::GEAR_RATIO),
    detail::make_limits(
      JointPositionLimit::PIP_MAX,
      JointPositionLimit::PIP_MIN,
      JointVelocityLimit::PIP,
      JointEffortLimit::PIP,
      JointStiffnessLimit::PIP,
      JointDampingLimit::PIP)
  };
}

}  // namespace ActuatorConfigFactory

}  // namespace aristo_actuator

#endif  // ARISTO_HARDWARE_INTERFACE__ACTUATOR_HPP_
