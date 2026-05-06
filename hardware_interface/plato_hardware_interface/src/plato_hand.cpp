#include "plato_hardware_interface/plato_hand.hpp"

#include <algorithm>
#include <cmath>
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
constexpr float kInvalidStateValue = std::numeric_limits<float>::quiet_NaN();
constexpr double kDefaultJointStateValue = 0.0;
auto logger() { return rclcpp::get_logger("plato_hardware_interface"); }

rclcpp::Clock & throttle_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

rclcpp::Clock & snapshot_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

using WritePlan = can_hardware_common::core::WritePlan;

bool is_nonfatal_read_status(TPCANStatus status)
{
  return status == PCAN_ERROR_OK || status == PCAN_ERROR_QRCVEMPTY;
}

static_assert(
  Hand::kNumJoints == FiveBarLinkage::Transmission::kNumJoints,
  "Plato hand joint dimension must match five-bar transmission");
static_assert(
  Hand::kNumActuators == FiveBarLinkage::Transmission::kNumActuators,
  "Plato hand actuator dimension must match five-bar transmission");

FiveBarLinkage::Transmission::JointArray build_joint_effort_limits(
  const std::vector<plato_actuator::Config> & actuator_configs)
{
  if (actuator_configs.size() != Hand::kNumActuators) {
    throw std::invalid_argument(
            "Plato hand expects exactly " + std::to_string(Hand::kNumActuators) +
            " actuator configs when building effort limits, got " +
            std::to_string(actuator_configs.size()));
  }

  FiveBarLinkage::Transmission::JointArray limits =
    {};
  limits.fill(std::numeric_limits<float>::infinity());
  for (size_t i = 0; i < actuator_configs.size(); ++i) {
    limits[i] = actuator_configs[i].static_config.effort_limit_nm;
  }
  return limits;
}

std::vector<plato_actuator::StaticConfig> extract_static_configs(
  const std::vector<plato_actuator::Config> & actuator_configs)
{
  std::vector<plato_actuator::StaticConfig> static_configs;
  static_configs.reserve(actuator_configs.size());
  for (const auto & config : actuator_configs) {
    static_configs.push_back(config.static_config);
  }
  return static_configs;
}

std::vector<float> extract_position_offsets(
  const std::vector<plato_actuator::Config> & actuator_configs)
{
  std::vector<float> position_offsets;
  position_offsets.reserve(actuator_configs.size());
  for (const auto & config : actuator_configs) {
    position_offsets.push_back(config.position_offset);
  }
  return position_offsets;
}

}  // namespace

// ════════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════════════════

Hand::Hand(PlatoHandConfig config)
: transmission_(config.linkage_config, build_joint_effort_limits(config.actuator_configs)),
  protocol_(),
  model_(transmission_),
  actuator_offset_yaml_path_(std::move(config.actuator_offset_yaml_path)),
  actuator_static_configs_(extract_static_configs(config.actuator_configs)),
  actuator_position_offsets_(extract_position_offsets(config.actuator_configs)),
  direct_tx_frame_timeout_(config.direct_tx_inter_frame_gap * 4),
  servo_stiffness_scale_(config.servo_stiffness_scale)
{
  if (actuator_static_configs_.size() != kNumActuators) {
    throw std::invalid_argument(
            "Plato hand expects exactly " + std::to_string(kNumActuators) +
            " actuator configs, got " + std::to_string(actuator_static_configs_.size()));
  }

  initialize_joint_buffers(kNumJoints, kDefaultJointStateValue);
  initialize_actuator_buffers(kNumActuators, kInvalidStateValue);
  state_snapshot_.resize(kNumJoints, kNumActuators);

  actuators_.reserve(kNumActuators);
  for (const auto & cfg : config.actuator_configs) {
    actuators_.emplace_back(cfg);
  }

  disable_on_destruction_ = config.disable_on_destruction;

  transport_.add_rx_observer([this](const TPCANMsg & frame) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (protocol_.process_rx_frame(frame, actuators_)) {
      mark_rx_frame_();
    }
  });

  RCLCPP_INFO(logger(), "================== Actuators Info ===================");
  RCLCPP_INFO(logger(), "Total Number of Actuators: %zu", kNumActuators);
  for (size_t i = 0; i < kNumActuators; ++i) {
    RCLCPP_INFO(
      logger(), "Actuator %zu - TX: 0x%02X, RX: 0x%02X",
      i + 1, actuators_[i].get_tx_id(), actuators_[i].get_rx_id());
  }

  // Deterministic paced direct TX path for write().
  transport_.set_tx_gap(config.direct_tx_inter_frame_gap);
  RCLCPP_INFO(
    logger(),
    "Direct TX inter-frame gap: %ld us",
    static_cast<long>(config.direct_tx_inter_frame_gap.count()));
  RCLCPP_INFO(
    logger(),
    "Direct TX frame timeout: %ld us",
    static_cast<long>(direct_tx_frame_timeout_.count()));
}

Hand::~Hand()
{
  if (!disable_on_destruction_ || !enable_requested_ || disable_requested_) {
    return;
  }

  try {
    if (!disable()) {
      RCLCPP_WARN(
        logger(),
        "Destructor auto-disable: one or more actuators did not acknowledge STOP_MOTOR.");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger(), "Destructor auto-disable raised exception: %s", e.what());
  } catch (...) {
    RCLCPP_WARN(logger(), "Destructor auto-disable raised unknown exception.");
  }
}

bool Hand::update_measurements_()
{
  const auto rx = transport_.poll_rx();
  last_rx_healthy_ = is_nonfatal_read_status(rx.status);
  if (!last_rx_healthy_) {
    RCLCPP_WARN_THROTTLE(
      logger(), throttle_clock(), 1000, "CAN receive error: status 0x%X", rx.status);
    return false;
  }

  return true;
}

void Hand::refresh_state_snapshot_()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  model_.update_joint_states(actuators_, joint_commands_, actuator_states_, joint_states_);

  state_snapshot_.stamp = snapshot_clock().now();
  state_snapshot_.has_fresh_rx = has_fresh_rx_(SteadyClock::now());
  state_snapshot_.calibrated = actuator_position_offsets_.size() == kNumActuators;
  state_snapshot_.sensors_ok = true;
  state_snapshot_.actuators_ready = model_.actuators_ready(actuators_);
  state_snapshot_.transport_healthy = last_rx_healthy_;
  state_snapshot_.lifecycle_busy = false;
  state_snapshot_.model_ready =
    actuators_.size() == kNumActuators &&
    actuator_static_configs_.size() == kNumActuators;
  if (state_snapshot_.actuator_states.size() != kNumActuators) {
    state_snapshot_.resize(kNumJoints, kNumActuators);
  }
  std::copy_n(joint_states_.position_data(), kNumJoints, state_snapshot_.joint_position.begin());
  std::copy_n(joint_states_.velocity_data(), kNumJoints, state_snapshot_.joint_velocity.begin());
  std::copy_n(joint_states_.effort_data(), kNumJoints, state_snapshot_.joint_effort.begin());
  model_.copy_feedback_snapshot(actuators_, state_snapshot_);
}

can_hardware_common::core::LifecyclePlan Hand::build_lifecycle_plan_(
  can_hardware_common::core::LifecycleOperation operation)
{
  LifecyclePlan plan;
  plan.operation = operation;
  plan.ready = true;

  if (operation == can_hardware_common::core::LifecycleOperation::kZero) {
    plan.dispatch_policy = can_hardware_common::core::DispatchPolicy::kCustomExecution;
    return plan;
  }

  plan.dispatch_policy = can_hardware_common::core::DispatchPolicy::kDirectFrames;
  switch (operation) {
    case can_hardware_common::core::LifecycleOperation::kEnable:
      protocol_.append_enable_frames(actuators_, plan.direct_frames);
      break;
    case can_hardware_common::core::LifecycleOperation::kDisable:
      protocol_.append_disable_frames(actuators_, plan.direct_frames);
      break;
    case can_hardware_common::core::LifecycleOperation::kZero:
      break;
  }

  return plan;
}

bool Hand::execute_lifecycle_plan_(const LifecyclePlan & plan)
{
  if (plan.operation == can_hardware_common::core::LifecycleOperation::kZero) {
    return execute_zero_lifecycle_();
  }

  return execute_standard_lifecycle_(plan);
}

bool Hand::execute_standard_lifecycle_(const LifecyclePlan & plan)
{
  const bool enabling = plan.operation == can_hardware_common::core::LifecycleOperation::kEnable;
  if (enabling) {
    enable_requested_ = true;
    disable_requested_ = false;
  } else {
    disable_requested_ = true;
  }

  for (const auto & frame : plan.direct_frames) {
    if (!send_frame_blocking_(frame, kResponseTimeout)) {
      return false;
    }
  }

  return true;
}

bool Hand::execute_zero_lifecycle_()
{
  return zero_actuators_();
}

void Hand::build_ready_write_plan_(WritePlan & plan)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    model_.joint_to_actuator_commands(
      joint_commands_, actuator_states_, joint_states_, actuator_commands_);
    protocol_.append_write_frames(
      actuators_,
      actuator_commands_,
      write_cycle_count_,
      servo_stiffness_scale_,
      plan.direct_frames);
    ++write_cycle_count_;
  }

  plan.dispatch_policy = can_hardware_common::core::DispatchPolicy::kDirectFrames;
}

bool Hand::execute_write_plan_(const WritePlan & plan)
{
  for (const auto & frame : plan.direct_frames) {
    if (!send_frame_blocking_(frame, direct_tx_frame_timeout_)) {
      return false;
    }
  }

  return true;
}

bool Hand::send_frame_blocking_(const TPCANMsg & frame, std::chrono::microseconds timeout)
{
  const auto now = SteadyClock::now();
  const auto next_send_time = transport_.next_tx_time();
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

  const TPCANStatus tx_status = transport_.send_tx_frame(frame);
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

bool Hand::poll_rx_for_(std::chrono::microseconds budget)
{
  const auto deadline = SteadyClock::now() + budget;
  bool healthy = true;

  while (SteadyClock::now() < deadline) {
    const auto rx = transport_.poll_rx();
    last_rx_healthy_ = is_nonfatal_read_status(rx.status);
    healthy = healthy && last_rx_healthy_;
    if (!last_rx_healthy_) {
      RCLCPP_WARN_THROTTLE(
        logger(), throttle_clock(), 1000, "CAN receive error: status 0x%X", rx.status);
      return false;
    }

    if (rx.processed_frames == 0U) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }

  return healthy;
}

// ════════════════════════════════════════════════════════════════════════════
//  Helpers
// ════════════════════════════════════════════════════════════════════════════

bool Hand::zero_actuators_()
{
  const bool probe_success = run_zeroing_probe_rounds_();

  std::vector<float> offsets;
  const bool capture_success = capture_zero_offsets_(offsets);
  const bool save_success = persist_zero_offsets_(offsets, probe_success && capture_success);
  return probe_success && capture_success && save_success;
}

bool Hand::run_zeroing_probe_rounds_()
{
  bool success = true;
  for (size_t round = 0; round < kZeroingProbeRounds; ++round) {
    for (size_t i = kThumbMcpIndex; i < kNumActuators; ++i) {
      if (!send_frame_blocking_(actuators_[i].set_joint_torque(0.0f).frame, kResponseTimeout)) {
        RCLCPP_WARN(
          logger(),
          "Zeroing probe direct TX failed for actuator %zu",
          i + 1);
        success = false;
      }
    }
    success = poll_rx_for_(kResponseTimeout) && success;
  }

  success = poll_rx_for_(kResponseTimeout) && success;
  return success;
}

bool Hand::capture_zero_offsets_(std::vector<float> & offsets)
{
  bool success = true;
  offsets.clear();
  offsets.reserve(kNumActuators);

  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < kNumActuators; ++i) {
    const bool is_servo = (i == kThumbRollIndex || i == kThumbYawIndex);

    if (is_servo) {
      // Preserve the existing offsets for joint1/joint2 thumb servo channels during zeroing.
      offsets.push_back(actuator_position_offsets_[i]);
      continue;
    }

    if (!actuators_[i].is_initialized()) {
      RCLCPP_WARN(
        logger(),
        "Zeroing skipped for actuator %zu: no valid feedback; keeping previous offset.",
        i + 1);
      offsets.push_back(actuator_position_offsets_[i]);
      success = false;
      continue;
    }

    if (!actuators_[i].set_current_position_as_zero()) {
      RCLCPP_WARN(
        logger(),
        "Zeroing failed for actuator %zu: unable to set current position as zero.",
        i + 1);
      offsets.push_back(actuator_position_offsets_[i]);
      success = false;
      continue;
    }

    actuator_position_offsets_[i] = actuators_[i].get_position_offset();
    offsets.push_back(actuator_position_offsets_[i]);
  }

  model_.update_joint_states(actuators_, joint_commands_, actuator_states_, joint_states_);
  return success;
}

bool Hand::persist_zero_offsets_(const std::vector<float> & offsets, bool zeroing_success) const
{
  try {
    plato_actuator::save_plato_actuator_position_offsets(offsets, actuator_offset_yaml_path_);
    if (zeroing_success) {
      RCLCPP_INFO(logger(), "Zeroing complete, offsets saved.");
    } else {
      RCLCPP_WARN(logger(), "Zeroing completed with warnings; offsets were saved.");
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Zeroing completed with warnings and failed to save offsets: %s", e.what());
    return false;
  }
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
