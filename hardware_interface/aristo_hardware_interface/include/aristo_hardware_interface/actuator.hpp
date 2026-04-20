#ifndef ARISTO_HARDWARE_INTERFACE__ACTUATOR_HPP_
#define ARISTO_HARDWARE_INTERFACE__ACTUATOR_HPP_

#include <cstdint>
#include <memory>
#include <optional>

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

}  // namespace aristo_actuator

#endif  // ARISTO_HARDWARE_INTERFACE__ACTUATOR_HPP_
