#ifndef ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_
#define ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_

#include <chrono>
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

  explicit Hand(std::vector<aristo_actuator::Config> actuator_configs);
  Hand(const Hand &) = delete;
  Hand & operator=(const Hand &) = delete;
  Hand(Hand &&) = delete;
  Hand & operator=(Hand &&) = delete;
  ~Hand() = default;

  bool write_joint_commands() { return write(); }

  size_t get_num_actuators() const { return kNumActuators; }

private:
  using LifecyclePlan = can_hardware_common::core::LifecyclePlan;
  using WritePlan = can_hardware_common::core::WritePlan;

  static constexpr std::chrono::microseconds kDirectTxInterFrameGap{100};
  static constexpr std::chrono::microseconds kDirectTxFrameTimeout{2000};
  static constexpr std::chrono::milliseconds kRxStaleTimeout{20};

  bool update_measurements_() override;
  void refresh_state_snapshot_() override;
  void build_ready_write_plan_(WritePlan & plan) override;
  bool execute_write_plan_(const WritePlan & plan) override;
  LifecyclePlan build_lifecycle_plan_(can_hardware_common::core::LifecycleOperation operation) override;
  bool execute_lifecycle_plan_(const LifecyclePlan & plan) override;
  bool execute_direct_frames_(
    const std::vector<TPCANMsg> & frames,
    std::chrono::microseconds timeout);
  can_hardware_common::CanBus::RxPollResult poll_can_bus();
  TPCANStatus send_frame_blocking_(const TPCANMsg & frame, std::chrono::microseconds timeout);
  void print_hardware_info_(const char * actuator_total_label = "Total Actuators") const;
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
