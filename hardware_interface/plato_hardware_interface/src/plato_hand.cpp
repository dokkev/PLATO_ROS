#include "plato_hardware_interface/plato_hand.hpp"

#include <array>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "plato_hardware_interface/utils/actuator_offset_loader.hpp"

namespace plato_hand
{

namespace
{
constexpr double kInvalidStateValue = std::numeric_limits<double>::quiet_NaN();
constexpr double kDefaultJointStateValue = 0.0;

auto logger() { return rclcpp::get_logger("plato_hardware_interface"); }

rclcpp::Clock & throttle_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

static_assert(
  Hand::kNumJoints == FiveBarLinkage::Transmission::kNumJoints,
  "Plato hand joint dimension must match five-bar transmission");
static_assert(
  Hand::kNumActuators == FiveBarLinkage::Transmission::kNumActuators,
  "Plato hand actuator dimension must match five-bar transmission");
}  // namespace

// ════════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════════════════

Hand::Hand(PlatoHandConfig config)
: transmission_(config.linkage_config),
  scheduler_(transport_),  // must follow transport_ and transmission_ in member order
  actuator_offset_yaml_path_(std::move(config.actuator_offset_yaml_path)),
  actuator_configs_(std::move(config.actuator_configs))
{
  if (actuator_configs_.size() != kNumActuators) {
    throw std::invalid_argument(
            "Plato hand expects exactly " + std::to_string(kNumActuators) +
            " actuator configs, got " + std::to_string(actuator_configs_.size()));
  }

  initialize_joint_buffers(kNumJoints, kDefaultJointStateValue);
  initialize_actuator_buffers(kNumActuators, kInvalidStateValue);

  actuators_.reserve(kNumActuators);
  for (const auto & cfg : actuator_configs_) {
    actuators_.emplace_back(cfg);
  }

  transport_.add_rx_observer([this](const TPCANMsg & frame) {
    (void)frame;
    mark_rx_frame_();
  });

  // Three RX observers: freshness timestamp, actuator state parsing, scheduler matching.
  transport_.add_rx_observer([this](const TPCANMsg & frame) {
    if (frame.MSGTYPE != PCAN_MESSAGE_STANDARD || frame.LEN < 2) {
      return;
    }
    for (size_t i = 0; i < kNumActuators; ++i) {
      if (actuators_[i].get_rx_id() == frame.ID) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        actuators_[i].process_message(frame);
        return;
      }
    }
  });

  transport_.add_rx_observer([this](const TPCANMsg & frame) {
    scheduler_.observe_rx(frame);
  });

  RCLCPP_INFO(logger(), "================== Actuators Info ===================");
  RCLCPP_INFO(logger(), "Total Number of Actuators: %zu", kNumActuators);
  for (size_t i = 0; i < kNumActuators; ++i) {
    RCLCPP_INFO(
      logger(), "Actuator %zu - TX: 0x%02X, RX: 0x%02X",
      i + 1, actuators_[i].get_tx_id(), actuators_[i].get_rx_id());
  }

  // Deterministic paced direct TX path for write_joint_commands().
  transport_.set_min_inter_frame_gap(config.direct_tx_inter_frame_gap);
  RCLCPP_INFO(
    logger(),
    "Direct TX inter-frame gap: %ld us",
    static_cast<long>(config.direct_tx_inter_frame_gap.count()));
}

bool Hand::enable(bool automatic_zeroing)
{
  bool success = true;

  for (size_t i = 0; i < kNumActuators; ++i) {
    const auto req = actuators_[i].make_enable_request(static_cast<uint32_t>(i));
    const auto res = scheduler_.execute_blocking(
      req, kResponseTimeout, kLifecycleCommandRetries);
    if (res.status != TransactionResult::Status::kConfirmed) {
      RCLCPP_ERROR(logger(), "Enable actuator %zu: %s",
        i + 1, TransactionResult::status_label(res.status));
      success = false;
    }
  }
  if (automatic_zeroing) {
    success = zero_actuators_() && success;
  }

  return success;
}

bool Hand::disable()
{
  bool success = true;

  for (size_t i = 0; i < kNumActuators; ++i) {
    const auto req = actuators_[i].make_disable_request(static_cast<uint32_t>(i));
    const auto res = scheduler_.execute_blocking(
      req, kResponseTimeout, kLifecycleCommandRetries);
    if (res.status != TransactionResult::Status::kConfirmed) {
      RCLCPP_ERROR(logger(), "Disable actuator %zu: %s",
        i + 1, TransactionResult::status_label(res.status));
      success = false;
    }
  }

  return success;
}

bool Hand::read()
{
  const auto rx = transport_.process_rx();
  if (rx.is_bus_error()) {
    RCLCPP_WARN_THROTTLE(
      logger(), throttle_clock(), 1000, "CAN receive error: status 0x%X", rx.status);
    return false;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  update_joint_states_locked_();
  return true;
}

bool Hand::write_joint_commands()
{
  std::array<TPCANMsg, kNumActuators> tx_frames{};
  size_t tx_count = 0;

  // 1) Validate command snapshot
  // 2) Joint->actuator mapping
  // 3) Build direct TX frames
  // 4) Capture desired-command history (delivery-agnostic)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto joint_cmd = joint_command_view();

    if (!joint_cmd.all_finite()) {
      RCLCPP_WARN_THROTTLE(
        logger(), throttle_clock(), 1000, "Skipping write: non-finite joint commands.");
      ++write_cycle_count_;
      return true;
    }

    transmission_.joint_to_actuator(
      joint_commands_, actuator_states_, joint_states_, actuator_commands_);
    const auto actuator_cmd = actuator_command_view();

    if ((write_cycle_count_ % kServoWriteDivisor) == 0) {
      tx_frames[tx_count++] = actuators_[kThumbRollIndex].set_servo_hold(
        static_cast<float>(actuator_cmd.position(kThumbRollIndex))).frame;
      tx_frames[tx_count++] = actuators_[kThumbYawIndex].set_servo_hold(
        static_cast<float>(actuator_cmd.position(kThumbYawIndex))).frame;
    }

    for (size_t i = kThumbMcpIndex; i < kNumActuators; ++i) {
      const size_t geared_index = i - kThumbMcpIndex;
      const size_t phase = write_cycle_count_ % kTorqueWriteStride;
      if ((geared_index % kTorqueWriteStride) != phase) {
        continue;
      }
      const Eigen::Index idx = static_cast<Eigen::Index>(i);
      tx_frames[tx_count++] = actuators_[i].set_joint_torque(
        static_cast<float>(actuator_cmd.effort(idx))).frame;
    }

    ++write_cycle_count_;
    capture_joint_commands();
    capture_actuator_commands();
  }

  const auto now = SteadyClock::now();
  if (!has_fresh_rx_(now)) {
    // Fast recovery path: try one RX drain only when freshness check fails.
    const auto rx_probe = transport_.process_rx();
    if (rx_probe.is_bus_error()) {
      RCLCPP_WARN_THROTTLE(
        logger(),
        throttle_clock(),
        1000,
        "CAN receive error while probing stale RX: status 0x%X",
        rx_probe.status);
      return false;
    }

    if (has_fresh_rx_(SteadyClock::now())) {
      stale_write_cycle_count_ = 0;
    } else {
      ++stale_write_cycle_count_;
      const long rx_age_ms = has_observed_rx_
        ? static_cast<long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_time_).count())
        : -1L;
      RCLCPP_WARN_THROTTLE(
        logger(),
        throttle_clock(),
        1000,
        "Skipping direct TX: RX stale (age=%ld ms, stale_cycles=%zu, total_rx=%zu)",
        rx_age_ms,
        stale_write_cycle_count_,
        rx_frame_count_);
      return true;
    }
  } else {
    stale_write_cycle_count_ = 0;
  }

  for (size_t i = 0; i < tx_count; ++i) {
    if (!send_frame_blocking_(tx_frames[i], kDirectTxFrameTimeout)) {
      return false;
    }
  }

  return true;
}

bool Hand::send_frame_blocking_(const TPCANMsg & frame, std::chrono::microseconds timeout)
{
  const auto now = SteadyClock::now();
  const auto next_send_time = transport_.next_send_time();
  if (next_send_time > now) {
    const auto wait = std::chrono::duration_cast<std::chrono::microseconds>(next_send_time - now);
    if (wait > timeout) {
      RCLCPP_WARN_THROTTLE(
        logger(),
        throttle_clock(),
        1000,
        "CAN direct TX pacing timeout on ID 0x%X (wait=%ldus)",
        frame.ID,
        static_cast<long>(wait.count()));
      return false;
    }
    std::this_thread::sleep_until(next_send_time);
  }

  const TPCANStatus tx_status = transport_.send_if_ready(frame);
  if (tx_status == PCAN_ERROR_OK) {
    return true;
  }
  if (tx_status == PCAN_ERROR_QXMTFULL) {
    RCLCPP_WARN_THROTTLE(
      logger(),
      throttle_clock(),
      1000,
      "CAN direct TX dropped by pacing on ID 0x%X",
      frame.ID);
    return false;
  }

  RCLCPP_WARN_THROTTLE(
    logger(),
    throttle_clock(),
    1000,
    "CAN direct TX error on ID 0x%X: status 0x%X",
    frame.ID,
    tx_status);
  return false;
}

// ════════════════════════════════════════════════════════════════════════════
//  Helpers
// ════════════════════════════════════════════════════════════════════════════

bool Hand::zero_actuators_()
{
  bool success = true;

  for (size_t round = 0; round < kZeroingProbeRounds; ++round) {
    for (size_t i = kThumbMcpIndex; i < kNumActuators; ++i) {
      const auto req = actuators_[i].make_torque_request(static_cast<uint32_t>(i), 0.0f);
      const auto res = scheduler_.execute_blocking(
        req, kResponseTimeout, kLifecycleCommandRetries);
      if (res.status != TransactionResult::Status::kConfirmed) {
        RCLCPP_WARN(
          logger(),
          "Zeroing probe failed for actuator %zu: %s",
          i + 1,
          TransactionResult::status_label(res.status));
        success = false;
      }
    }
  }

  plato_actuator::PositionOffsets offsets;
  offsets.reserve(kNumActuators);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < kNumActuators; ++i) {
      const bool is_servo = (i == kThumbRollIndex || i == kThumbYawIndex);

      if (is_servo) {
        offsets.push_back(actuator_configs_[i].core.position_offset);
        continue;
      }

      if (!actuators_[i].is_initialized()) {
        RCLCPP_WARN(
          logger(),
          "Zeroing skipped for actuator %zu: no valid feedback; keeping previous offset.",
          i + 1);
        offsets.push_back(actuator_configs_[i].core.position_offset);
        success = false;
        continue;
      }

      if (!actuators_[i].set_current_position_as_zero()) {
        RCLCPP_WARN(
          logger(),
          "Zeroing failed for actuator %zu: unable to set current position as zero.",
          i + 1);
        offsets.push_back(actuator_configs_[i].core.position_offset);
        success = false;
        continue;
      }

      actuator_configs_[i].core.position_offset = actuators_[i].get_position_offset();
      offsets.push_back(actuators_[i].get_position_offset());
    }

    update_joint_states_locked_();
  }

  try {
    plato_actuator::save_plato_actuator_position_offsets(offsets, actuator_offset_yaml_path_);
    if (success) {
      RCLCPP_INFO(logger(), "Zeroing complete, offsets saved.");
    } else {
      RCLCPP_WARN(logger(), "Zeroing completed with warnings; offsets were saved.");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Zeroing completed with warnings and failed to save offsets: %s", e.what());
    success = false;
  }

  return success;
}

void Hand::update_joint_states_locked_()
{
  bool all_initialized = true;
  for (size_t i = 0; i < kNumActuators; ++i) {
    if (!actuators_[i].is_initialized()) {
      all_initialized = false;
      continue;
    }
    const auto & state = actuators_[i].get_state();
    actuator_states_.position_at(i) = state.position;
    actuator_states_.velocity_at(i) = state.velocity;
    actuator_states_.effort_at(i) = state.torque;
  }

  if (!all_initialized) {
    return;
  }

  can_hardware_common::RobotIO::JointState joint_state_candidate = joint_states_;
  transmission_.actuator_to_joint(actuator_states_, joint_state_candidate);
  if (!joint_state_candidate.const_view().all_finite()) {
    RCLCPP_WARN_THROTTLE(
      logger(),
      throttle_clock(),
      1000,
      "Skipping joint-state update: transmission output contains non-finite values.");
    return;
  }

  // Preserve storage addresses exported via ros2_control state interfaces.
  // Reassigning joint_states_ would invalidate those pointers.
  auto dst = joint_states_.view();
  const auto src = joint_state_candidate.const_view();
  dst.position = src.position;
  dst.velocity = src.velocity;
  dst.effort = src.effort;
}

void Hand::mark_rx_frame_()
{
  last_rx_time_ = SteadyClock::now();
  has_observed_rx_ = true;
  ++rx_frame_count_;
}

bool Hand::has_fresh_rx_(SteadyClock::time_point now) const
{
  if (!has_observed_rx_) {
    return false;
  }
  return (now - last_rx_time_) <= kRxStaleTimeout;
}

void Hand::print_motor_positions()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < kNumActuators; ++i) {
    RCLCPP_INFO(logger(), "J%zu Motor Position: %.4f", i + 1, actuators_[i].get_motor_position());
  }
}

}  // namespace plato_hand
