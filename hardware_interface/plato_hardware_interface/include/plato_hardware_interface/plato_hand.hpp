#ifndef PLATO_HARDWARE_INTERFACE__PLATO_HAND_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO_HAND_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

#include "can_hardware_common/can_transport.hpp"
#include "can_hardware_common/command_scheduler.hpp"
#include "can_hardware_common/robot.hpp"
#include "plato_hardware_interface/actuator.hpp"
#include "plato_hardware_interface/five_bar_linkage.hpp"
#include "plato_hardware_interface/plato_hand_config.hpp"

namespace plato_hand
{

class Hand : public can_hardware_common::RobotIO
{
public:
  static constexpr size_t kNumJoints = 8;
  static constexpr size_t kNumActuators = 8;

  explicit Hand(PlatoHandConfig config);
  Hand(const Hand &) = delete;
  Hand & operator=(const Hand &) = delete;
  Hand(Hand &&) = delete;
  Hand & operator=(Hand &&) = delete;
  ~Hand();

  bool enable(bool automatic_zeroing = false);
  bool disable();
  bool read();
  bool write_joint_commands();

  void print_motor_positions();

private:
  using SteadyClock = std::chrono::steady_clock;
  using TransactionResult = can_hardware_common::CanCommandScheduler::TransactionResult;

  static constexpr size_t kThumbRollIndex = 0;
  static constexpr size_t kThumbYawIndex = 1;
  static constexpr size_t kThumbMcpIndex = 2;
  static constexpr size_t kServoWriteDivisor = 10;
  // Spread geared-joint torque TX across cycles to reduce per-cycle write latency at high rates.
  static constexpr size_t kTorqueWriteStride = 2;
  static constexpr std::chrono::microseconds kDirectTxInterFrameGap{100};
  static constexpr std::chrono::microseconds kDirectTxFrameTimeout{500};
  static constexpr std::chrono::milliseconds kRxStaleTimeout{20};
  // Thumb servo channels can acknowledge lifecycle commands around ~17 ms on hardware.
  // Keep timeout comfortably above that to avoid false startup timeouts.
  static constexpr std::chrono::microseconds kResponseTimeout{25000};
  static constexpr std::size_t kLifecycleCommandRetries = 2;
  static constexpr size_t kZeroingProbeRounds = 3;

  bool send_frame_blocking_(const TPCANMsg & frame, std::chrono::microseconds timeout);
  bool zero_actuators_();
  void update_joint_states_locked_();
  void mark_rx_frame_();
  bool has_fresh_rx_(SteadyClock::time_point now) const;
  void configure_transport_simulator_(bool bypass_hardware);
  std::vector<TPCANMsg> simulate_tx_frame_(const TPCANMsg & tx_frame);
  static TPCANMsg make_ack_frame_(uint32_t rx_id, uint8_t opcode, uint8_t result = 0x00);
  static TPCANMsg make_state_frame_(
    uint32_t rx_id,
    uint8_t opcode,
    uint8_t temperature,
    float motor_position,
    float motor_velocity_rpm,
    float motor_torque);
  static float decode_float_le_(const TPCANMsg & frame, size_t offset);

  struct SimActuatorState
  {
    float motor_position = 0.0F;
    float motor_velocity_rpm = 0.0F;
    float motor_torque = 0.0F;
    bool motor_enabled = false;
    uint8_t temperature = 30;
  };

  can_hardware_common::CanTransport transport_;
  FiveBarLinkage::Transmission transmission_;
  can_hardware_common::CanCommandScheduler scheduler_;
  mutable std::mutex state_mutex_;
  mutable std::mutex simulator_state_mutex_;
  std::vector<plato_actuator::Actuator> actuators_;
  std::vector<SimActuatorState> simulator_states_;

  std::string actuator_offset_yaml_path_;
  std::vector<plato_actuator::Config> actuator_configs_;
  size_t write_cycle_count_ = 0;
  SteadyClock::time_point last_rx_time_{};
  size_t rx_frame_count_ = 0;
  size_t stale_write_cycle_count_ = 0;
  bool has_observed_rx_ = false;
  bool transport_simulator_enabled_ = false;
  bool disable_on_destruction_ = true;
  bool enable_requested_ = false;
  bool disable_requested_ = false;
};

}  // namespace plato_hand

#endif  // PLATO_HARDWARE_INTERFACE__PLATO_HAND_HPP_
