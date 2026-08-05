#ifndef ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_
#define ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

#include "aristo_hardware_interface/actuator.hpp"
#include "aristo_hardware_interface/aristo_model.hpp"
#include "aristo_hardware_interface/aristo_protocol.hpp"
#include "can_hardware_common/can_bus.hpp"
#include "can_hardware_common/core/can_hand_base.hpp"

namespace aristo_hand
{

class Hand : public can_hardware_common::core::CanHandBase
{
public:
  static constexpr size_t kNumActuators = 8;

  struct ActuatorActivationStatus
  {
    size_t index = 0;
    uint32_t tx_id = 0;
    uint32_t rx_id = 0;
    bool enabled = false;
    bool has_feedback = false;
    bool in_oc_mode = false;
    bool has_fault = false;
    bool has_motor_params = false;
    bool has_active_limits = false;
    mit_can_protocol::MotorParams motor_params{};
    mit_can_protocol::MitLimits active_limits{};
  };

  explicit Hand(std::vector<aristo_actuator::Config> actuator_configs);
  Hand(const Hand &) = delete;
  Hand & operator=(const Hand &) = delete;
  Hand(Hand &&) = delete;
  Hand & operator=(Hand &&) = delete;
  ~Hand() = default;

  bool write_joint_commands() { return write(); }

  size_t get_num_actuators() const { return kNumActuators; }
  std::vector<ActuatorActivationStatus> actuator_activation_statuses() const;

private:
  using LifecyclePlan = can_hardware_common::core::LifecyclePlan;
  using WritePlan = can_hardware_common::core::WritePlan;

  static constexpr std::chrono::microseconds kDirectTxInterFrameGap{250};
  static constexpr std::chrono::microseconds kDirectTxFrameTimeout{10000};
  static constexpr std::chrono::milliseconds kRxStaleTimeout{20};
  static constexpr std::chrono::milliseconds kStartupMetadataTimeout{500};
  static constexpr std::chrono::milliseconds kStartupMetadataRetryInterval{50};

  bool update_measurements_() override;
  void refresh_state_snapshot_() override;
  void build_ready_write_plan_(WritePlan & plan) override;
  bool execute_write_plan_(const WritePlan & plan) override;
  LifecyclePlan build_lifecycle_plan_(can_hardware_common::core::LifecycleOperation operation) override;
  bool execute_lifecycle_plan_(const LifecyclePlan & plan) override;
  bool execute_direct_frames_(
    const std::vector<TPCANMsg> & frames,
    std::chrono::microseconds timeout);
  bool query_startup_metadata_();
  bool confirm_mit_mode_(std::chrono::milliseconds timeout);
  bool wait_for_motor_params_(
    std::size_t actuator_index,
    const TPCANMsg & request_frame,
    std::chrono::milliseconds timeout);
  bool wait_for_active_limits_(
    std::size_t actuator_index,
    const TPCANMsg & request_frame,
    std::chrono::milliseconds timeout);
  bool startup_metadata_ready_() const;
  bool actuator_has_motor_params_(std::size_t actuator_index) const;
  bool actuator_has_active_limits_(std::size_t actuator_index) const;
  bool actuator_in_mit_mode_(std::size_t actuator_index) const;
  can_hardware_common::CanBus::RxPollResult poll_can_bus();
  TPCANStatus send_frame_blocking_(const TPCANMsg & frame, std::chrono::microseconds timeout);
  void mark_rx_frame_();
  bool has_fresh_rx_(std::chrono::steady_clock::time_point now) const;

  can_hardware_common::CanBus transport_;
  mutable std::mutex state_mutex_;
  AristoProtocol protocol_;
  AristoModel model_;
  std::vector<aristo_actuator::Actuator> actuators_;
  std::vector<aristo_actuator::Config> actuator_configs_;
  can_hardware_common::CanBus::RxPollResult last_rx_result_{};
  std::chrono::steady_clock::time_point last_rx_time_{};
  bool has_observed_rx_ = false;
};

}  // namespace aristo_hand

#endif  // ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_
