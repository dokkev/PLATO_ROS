#ifndef ARISTO_HARDWARE_INTERFACE__ACTUATOR_HPP_
#define ARISTO_HARDWARE_INTERFACE__ACTUATOR_HPP_

#include <cstdint>
#include <memory>
#include <optional>

#include "can_hardware_common/actuator.hpp"

namespace aristo_actuator
{

class CANProtocol;

struct Config
{
  can_hardware_common::ActuatorCoreConfig core;
  actuator::Limits limits;
  float torque_smoothing = 1.0f;
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

  void process_rx_frame(const TPCANMsg & msg);

  const Config & get_config() const { return config_; }
  const can_hardware_common::ActuatorState & get_feedback() const { return feedback_; }
  const can_hardware_common::ActuatorStatus & get_status() const { return status_; }

  uint32_t get_tx_id() const { return config_.core.can_tx_id; }
  uint32_t get_rx_id() const { return config_.core.can_rx_id; }
  bool is_enabled() const { return motor_enabled_; }
  bool has_feedback() const { return has_feedback_; }

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
  float smooth_impedance_torque_(float torque);
  void reset_torque_smoothing_(float torque);
  void apply_decoded_feedback_(const can_hardware_common::DecodedFeedback & decoded);

  Config config_;
  std::unique_ptr<CANProtocol> protocol_;
  can_hardware_common::ActuatorState feedback_;
  can_hardware_common::ActuatorStatus status_;
  bool motor_enabled_ = false;
  bool has_feedback_ = false;
  float smoothed_impedance_torque_ = 0.0f;
  bool has_smoothed_impedance_torque_ = false;
  SoftLimitState control_state_ = SoftLimitState::kOperational;

  static constexpr float kJointLimitSafetyMargin = 0.05f;
  static constexpr float kSoftLimitMargin = 0.174f;
  static constexpr float kSoftLimitHysteresis = 0.02f;
};

}  // namespace aristo_actuator

#endif  // ARISTO_HARDWARE_INTERFACE__ACTUATOR_HPP_
