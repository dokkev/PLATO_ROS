#include "aristo_hardware_interface/aristo_hand.hpp"
#include "aristo_hardware_interface/can_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace aristo_hand
{

namespace
{
using LifecyclePlan = can_hardware_common::core::LifecyclePlan;
using WritePlan = can_hardware_common::core::WritePlan;

auto logger() { return rclcpp::get_logger("aristo_hardware_interface"); }

rclcpp::Clock & snapshot_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

bool is_nonfatal_read_status(TPCANStatus status)
{
  return status == PCAN_ERROR_OK || status == PCAN_ERROR_QRCVEMPTY;
}

void log_motor_param_sanity(std::size_t actuator_index, const mit_can_protocol::MotorParams & params)
{
  constexpr float kKtTol = 1e-3f;
  if (std::abs(params.torque_constant_nm_per_a - aristo_actuator::driverKt) > kKtTol)
  {
    RCLCPP_WARN(
      logger(),
      "Aristo actuator %zu reports torque_constant_nm_per_a=%.6f, expected %.6f",
      actuator_index + 1,
      params.torque_constant_nm_per_a,
      aristo_actuator::driverKt);
  }

  if (params.gear_ratio != aristo_actuator::driverGear) {
    RCLCPP_WARN(
      logger(),
      "Aristo actuator %zu reports driver gear_ratio=%u, expected %u",
      actuator_index + 1,
      params.gear_ratio,
      aristo_actuator::driverGear);
  }
}

void log_active_limit_sanity(std::size_t actuator_index, const mit_can_protocol::MitLimits & limits)
{
  if (!std::isfinite(limits.t_max_nm) || limits.t_max_nm <= 0.0f) {
    RCLCPP_WARN(
      logger(),
      "Aristo actuator %zu reports invalid MIT T_Max=%.6f Nm",
      actuator_index + 1,
      limits.t_max_nm);
    return;
  }

  const float max_cmd_mnm = limits.t_max_nm / aristo_actuator::cmdEffortScale;
  if (std::abs(limits.t_max_nm - mit_can_protocol::T_MAX) > 1e-3f) {
    RCLCPP_WARN(
      logger(),
      "Aristo actuator %zu active MIT T_Max=%.3f Nm differs from default %.3f Nm; ROS command clamp is %.1f mNm",
      actuator_index + 1,
      limits.t_max_nm,
      mit_can_protocol::T_MAX,
      max_cmd_mnm);
  }
}

void accumulate_poll_result(
  can_hardware_common::CanBus::RxPollResult & aggregate,
  const can_hardware_common::CanBus::RxPollResult & update)
{
  aggregate.processed_frames += update.processed_frames;

  if (is_nonfatal_read_status(aggregate.status) && !is_nonfatal_read_status(update.status))
  {
    aggregate.status = update.status;
  } else if (
    aggregate.status == PCAN_ERROR_QRCVEMPTY &&
    update.status == PCAN_ERROR_OK)
  {
    aggregate.status = PCAN_ERROR_OK;
  }
}
}  // namespace

Hand::Hand(std::vector<aristo_actuator::Config> actuator_configs)
: actuator_configs_(std::move(actuator_configs))
{
  if (actuator_configs_.size() != kNumActuators) {
    throw std::invalid_argument(
            "Aristo hand expects exactly " + std::to_string(kNumActuators) +
            " actuator configs, got " + std::to_string(actuator_configs_.size()));
  }

  initialize_joint_buffers(kNumActuators, 0.0);
  initialize_actuator_buffers(kNumActuators, 0.0);
  state_snapshot_.resize(kNumActuators, kNumActuators);

  actuators_.reserve(actuator_configs_.size());
  for (const auto & config : actuator_configs_) {
    actuators_.emplace_back(config);
  }

  protocol_.initialize_rx_dispatch(actuators_);
  transport_.add_rx_observer([this](const TPCANMsg & frame) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (protocol_.process_rx_frame(frame, actuators_)) {
      mark_rx_frame_();
    }
  });
  transport_.set_tx_gap(kDirectTxInterFrameGap);
}

can_hardware_common::CanBus::RxPollResult Hand::poll_can_bus()
{
  auto poll_result = transport_.poll_rx();
  if (poll_result.processed_frames > 0 && poll_result.status == PCAN_ERROR_OK) {
    const auto extra_poll_result = transport_.poll_rx();
    if (extra_poll_result.processed_frames > 0 || extra_poll_result.status != PCAN_ERROR_QRCVEMPTY) {
      accumulate_poll_result(poll_result, extra_poll_result);
    }
  }

  return poll_result;
}

TPCANStatus Hand::send_frame_blocking_(
  const TPCANMsg & frame,
  std::chrono::microseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  TPCANStatus last_status = PCAN_ERROR_QXMTFULL;

  while (std::chrono::steady_clock::now() < deadline) {
    const auto next_send = transport_.next_tx_time();
    const auto now = std::chrono::steady_clock::now();
    if (next_send > now) {
      if (next_send >= deadline) {
        return PCAN_ERROR_QXMTFULL;
      }
      std::this_thread::sleep_until(next_send);
      continue;
    }

    const TPCANStatus status = transport_.send_tx_frame(frame);
    if (status == PCAN_ERROR_OK) {
      return status;
    }
    if (status != PCAN_ERROR_QXMTFULL) {
      return status;
    }

    last_status = status;
    (void)poll_can_bus();
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }

  return last_status;
}

std::vector<Hand::ActuatorActivationStatus> Hand::actuator_activation_statuses() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<ActuatorActivationStatus> statuses;
  statuses.reserve(actuators_.size());

  for (size_t i = 0; i < actuators_.size(); ++i) {
    const auto & actuator = actuators_[i];
    const auto & status = actuator.get_status();
    statuses.push_back(ActuatorActivationStatus{
      i,
      actuator.get_tx_id(),
      actuator.get_rx_id(),
      actuator.is_enabled(),
      actuator.has_feedback(),
      status.in_oc_mode,
      status.has_fault,
      actuator.has_motor_params(),
      actuator.has_active_limits(),
      actuator.motor_params(),
      actuator.active_limits()});
  }

  return statuses;
}

bool Hand::update_measurements_()
{
  last_rx_result_ = poll_can_bus();
  if (!is_nonfatal_read_status(last_rx_result_.status)) {
    RCLCPP_WARN(
      logger(),
      "CAN read failed while polling Aristo hand (status=0x%X)",
      last_rx_result_.status);
    return false;
  }

  return true;
}

void Hand::refresh_state_snapshot_()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  model_.update_joint_states(actuators_, actuator_states_, joint_states_);
  state_snapshot_.stamp = snapshot_clock().now();
  state_snapshot_.has_fresh_rx = has_fresh_rx_(std::chrono::steady_clock::now());
  state_snapshot_.calibrated = actuator_configs_.size() == kNumActuators;
  state_snapshot_.sensor_status = {};
  state_snapshot_.sensors_ok = true;
  state_snapshot_.actuators_ready = model_.actuators_ready(actuators_);
  state_snapshot_.transport_healthy = is_nonfatal_read_status(last_rx_result_.status);
  state_snapshot_.lifecycle_busy = false;
  state_snapshot_.model_ready =
    actuators_.size() == kNumActuators &&
    actuator_configs_.size() == kNumActuators;
  if (state_snapshot_.actuator_states.size() != kNumActuators) {
    state_snapshot_.resize(kNumActuators, kNumActuators);
  }
  can_hardware_common::RobotIO::copy_joint_state_to_snapshot(joint_states_, state_snapshot_);
  model_.copy_feedback_snapshot(actuators_, state_snapshot_);
}

can_hardware_common::core::LifecyclePlan Hand::build_lifecycle_plan_(
  can_hardware_common::core::LifecycleOperation operation)
{
  LifecyclePlan plan;
  plan.operation = operation;
  plan.ready = true;
  plan.dispatch_policy = can_hardware_common::core::DispatchPolicy::kDirectFrames;

  switch (operation) {
    case can_hardware_common::core::LifecycleOperation::kEnable:
      protocol_.append_enable_frames(actuators_, plan.direct_frames);
      break;
    case can_hardware_common::core::LifecycleOperation::kDisable:
      protocol_.append_disable_frames(actuators_, plan.direct_frames);
      break;
    case can_hardware_common::core::LifecycleOperation::kZero:
      protocol_.append_zero_frames(actuators_, plan.direct_frames);
      break;
  }

  return plan;
}

bool Hand::execute_lifecycle_plan_(const LifecyclePlan & plan)
{
  if (plan.operation == can_hardware_common::core::LifecycleOperation::kEnable) {
    if (!query_startup_metadata_()) {
      return false;
    }

    if (!execute_direct_frames_(plan.direct_frames, kDirectTxFrameTimeout)) {
      return false;
    }

    return confirm_mit_mode_(kStartupMetadataTimeout);
  }

  return execute_direct_frames_(plan.direct_frames, kDirectTxFrameTimeout);
}

void Hand::build_ready_write_plan_(WritePlan & plan)
{
  std::vector<can_hardware_common::ActuatorTarget> impedance_targets;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    model_.build_impedance_targets(
      actuators_, joint_commands_, actuator_commands_, impedance_targets);
    protocol_.append_impedance_frames(actuators_, impedance_targets, plan.direct_frames);
  }

  plan.dispatch_policy = can_hardware_common::core::DispatchPolicy::kDirectFrames;
}

bool Hand::execute_write_plan_(const WritePlan & plan)
{
  return execute_direct_frames_(plan.direct_frames, kDirectTxFrameTimeout);
}

bool Hand::execute_direct_frames_(
  const std::vector<TPCANMsg> & frames,
  std::chrono::microseconds timeout)
{
  for (const auto & frame : frames) {
    if (aristo_hand::is_dangerous_all_zero_mit_command(frame)) {
      RCLCPP_FATAL(
        logger(),
        "Refusing to send dangerous all-zero MIT command frame ID=0x%X len=%u",
        frame.ID,
        frame.LEN);
      return false;
    }

    const TPCANStatus status = send_frame_blocking_(frame, timeout);
    if (status != PCAN_ERROR_OK) {
      const auto diagnostics = can_hardware_common::CanBus::format_diagnostics(
        transport_.get_diagnostics());
      RCLCPP_ERROR(
        logger(),
        "Failed to send Aristo CAN frame ID=0x%X len=%u status=%s diagnostics={%s}",
        frame.ID,
        frame.LEN,
        can_hardware_common::CanBus::bus_status_string(status).c_str(),
        diagnostics.c_str());
      return false;
    }
    (void)poll_can_bus();
  }
  return true;
}

bool Hand::query_startup_metadata_()
{
  bool all_motor_params_ready = true;
  for (std::size_t i = 0; i < actuators_.size(); ++i) {
    TPCANMsg frame{};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      frame = actuators_[i].read_motor_params().frame;
    }

    if (!wait_for_motor_params_(i, frame, kStartupMetadataTimeout)) {
      RCLCPP_ERROR(
        logger(),
        "Timed out waiting for Aristo actuator %zu motor params.",
        i + 1);
      all_motor_params_ready = false;
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      log_motor_param_sanity(i, actuators_[i].motor_params());
    }
  }

  bool all_limits_ready = true;
  for (std::size_t i = 0; i < actuators_.size(); ++i) {
    TPCANMsg frame{};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      frame = actuators_[i].read_can_limits().frame;
    }

    if (!wait_for_active_limits_(i, frame, kStartupMetadataTimeout)) {
      RCLCPP_ERROR(
        logger(),
        "Timed out waiting for Aristo actuator %zu MIT limits.",
        i + 1);
      all_limits_ready = false;
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      log_active_limit_sanity(i, actuators_[i].active_limits());
    }
  }

  return all_motor_params_ready && all_limits_ready;
}

bool Hand::confirm_mit_mode_(std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    std::vector<TPCANMsg> query_frames;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (std::size_t i = 0; i < actuators_.size(); ++i) {
        if (!actuator_in_mit_mode_(i)) {
          query_frames.push_back(actuators_[i].read_state().frame);
        }
      }

      if (query_frames.empty()) {
        return true;
      }
    }

    if (!execute_direct_frames_(query_frames, kDirectTxFrameTimeout)) {
      return false;
    }
    (void)poll_can_bus();

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const bool all_in_mit_mode = std::all_of(
        actuators_.begin(),
        actuators_.end(),
        [](const auto & actuator) { return actuator.is_enabled(); });
      if (all_in_mit_mode) {
        return true;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  RCLCPP_ERROR(
    logger(),
    "Timed out waiting for all Aristo actuators to report MIT/OC mode.");
  return false;
}

bool Hand::wait_for_motor_params_(
  std::size_t actuator_index,
  const TPCANMsg & request_frame,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto next_request_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_request_time) {
      const TPCANStatus status = send_frame_blocking_(request_frame, kDirectTxFrameTimeout);
      if (status != PCAN_ERROR_OK) {
        return false;
      }
      next_request_time = now + kStartupMetadataRetryInterval;
    }

    (void)poll_can_bus();

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (actuator_has_motor_params_(actuator_index)) {
        return true;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  return actuator_has_motor_params_(actuator_index);
}

bool Hand::wait_for_active_limits_(
  std::size_t actuator_index,
  const TPCANMsg & request_frame,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto next_request_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_request_time) {
      const TPCANStatus status = send_frame_blocking_(request_frame, kDirectTxFrameTimeout);
      if (status != PCAN_ERROR_OK) {
        return false;
      }
      next_request_time = now + kStartupMetadataRetryInterval;
    }

    (void)poll_can_bus();

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (actuator_has_active_limits_(actuator_index)) {
        return true;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  return actuator_has_active_limits_(actuator_index);
}

bool Hand::startup_metadata_ready_() const
{
  return std::all_of(
    actuators_.begin(),
    actuators_.end(),
    [](const auto & actuator) {
      return actuator.has_motor_params() && actuator.has_active_limits();
    });
}

bool Hand::actuator_has_motor_params_(std::size_t actuator_index) const
{
  return actuator_index < actuators_.size() && actuators_[actuator_index].has_motor_params();
}

bool Hand::actuator_has_active_limits_(std::size_t actuator_index) const
{
  return actuator_index < actuators_.size() && actuators_[actuator_index].has_active_limits();
}

bool Hand::actuator_in_mit_mode_(std::size_t actuator_index) const
{
  return actuator_index < actuators_.size() && actuators_[actuator_index].is_enabled();
}

void Hand::mark_rx_frame_()
{
  last_rx_time_ = std::chrono::steady_clock::now();
  has_observed_rx_ = true;
}

bool Hand::has_fresh_rx_(std::chrono::steady_clock::time_point now) const
{
  if (!has_observed_rx_) {
    return false;
  }
  return (now - last_rx_time_) <= kRxStaleTimeout;
}

}  // namespace aristo_hand
