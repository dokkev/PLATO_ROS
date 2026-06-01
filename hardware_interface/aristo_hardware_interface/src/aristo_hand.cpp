#include "aristo_hardware_interface/aristo_hand.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
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

  print_hardware_info_();
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
    const TPCANStatus status = transport_.send_tx_frame(frame);
    if (status == PCAN_ERROR_OK) {
      return status;
    }
    if (status != PCAN_ERROR_QXMTFULL) {
      return status;
    }

    last_status = status;
    const auto next_send = transport_.next_tx_time();
    const auto now = std::chrono::steady_clock::now();
    if (next_send > now) {
      const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(next_send - now);
      std::this_thread::sleep_for(std::min(std::chrono::microseconds(100), remaining));
    } else {
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  }

  return last_status;
}

void Hand::print_hardware_info_(const char * actuator_total_label) const
{
  std::cout << "================== Actuators Info ===================" << std::endl;
  std::cout << "[INFO] " << actuator_total_label << ": " << actuators_.size() << std::endl;

  for (size_t i = 0; i < actuators_.size(); ++i) {
    std::cout << "[INFO] Actuator " << (i + 1)
              << " - TX: 0x" << std::hex << actuators_[i].get_tx_id()
              << ", RX: 0x" << actuators_[i].get_rx_id() << std::dec << std::endl;
  }
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
    if (send_frame_blocking_(frame, timeout) != PCAN_ERROR_OK) {
      return false;
    }
  }
  return true;
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
