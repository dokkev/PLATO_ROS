#include "plato_hardware_interface/plato_hand.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "plato_hardware_interface/utils/actuator_offset_loader.hpp"

namespace plato_hand
{

namespace
{
constexpr float kSteadywinTorqueScale = 8.0f;
constexpr float kSteadywinTorqueLimit = 9.8f;
constexpr double kInvalidStateValue = std::numeric_limits<double>::quiet_NaN();
constexpr size_t kMaxTransientReadFailures = 5;
constexpr size_t kZeroingProbeRetryPeriodCycles = 10;

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

void accumulate_poll_result(
  can_hardware_common::CanBusManager::PollResult & aggregate,
  const can_hardware_common::CanBusManager::PollResult & update)
{
  aggregate.processed_frames += update.processed_frames;
  aggregate.hit_frame_budget = aggregate.hit_frame_budget || update.hit_frame_budget;

  const auto is_nonfatal_status = [](TPCANStatus status) {
      return status == PCAN_ERROR_OK || status == PCAN_ERROR_QRCVEMPTY;
    };

  if (is_nonfatal_status(aggregate.read_status) && !is_nonfatal_status(update.read_status)) {
    aggregate.read_status = update.read_status;
  } else if (
    aggregate.read_status == PCAN_ERROR_QRCVEMPTY &&
    update.read_status == PCAN_ERROR_OK)
  {
    aggregate.read_status = PCAN_ERROR_OK;
  }
}
}  // namespace

Hand::Hand(PlatoHandConfig config)
: transmission_(config.linkage_config),
  actuator_offset_yaml_path_(std::move(config.actuator_offset_yaml_path)),
  actuator_configs_(std::move(config.actuator_configs))
{
  if (actuator_configs_.size() != kNumActuators) {
    throw std::invalid_argument(
            "Plato hand expects exactly " + std::to_string(kNumActuators) +
            " actuator configs, got " + std::to_string(actuator_configs_.size()));
  }

  initialize_joint_buffers(kNumJoints, kInvalidStateValue);
  initialize_actuator_buffers(kNumActuators, kInvalidStateValue);

  actuators_.reserve(kNumActuators);
  for (const auto & config : actuator_configs_) {
    actuators_.emplace_back(config);
  }

  initialize_rx_dispatch_table_();

  print_actuator_info_();
}

bool Hand::enable_all_actuators()
{
  TPCANStatus first_error = PCAN_ERROR_OK;
  for (auto & actuator : actuators_) {
    const TPCANStatus status = send_command_(actuator.enable_motor());
    if (first_error == PCAN_ERROR_OK && status != PCAN_ERROR_OK) {
      first_error = status;
    }
  }

  if (first_error != PCAN_ERROR_OK) {
    RCLCPP_ERROR_THROTTLE(
      logger(), throttle_clock(), 1000,
      "Failed to enable one or more Plato actuators (transport status 0x%X)",
      first_error);
    return false;
  }

  return true;
}

bool Hand::disable_all_actuators()
{
  TPCANStatus first_error = PCAN_ERROR_OK;
  for (auto & actuator : actuators_) {
    const TPCANStatus status = send_command_(actuator.disable_motor());
    if (first_error == PCAN_ERROR_OK && status != PCAN_ERROR_OK) {
      first_error = status;
    }
  }

  if (first_error != PCAN_ERROR_OK) {
    RCLCPP_ERROR_THROTTLE(
      logger(), throttle_clock(), 1000,
      "Failed to disable one or more Plato actuators (transport status 0x%X)",
      first_error);
    return false;
  }

  return true;
}

void Hand::print_hardware_info_(const char * actuator_total_label) const
{
  auto logger = rclcpp::get_logger("plato_hardware_interface");
  RCLCPP_INFO(logger, "================== Actuators Info ===================");
  RCLCPP_INFO(logger, "%s: %zu", actuator_total_label, actuators_.size());

  for (size_t i = 0; i < actuators_.size(); ++i) {
    RCLCPP_INFO(
      logger, "Actuator %zu - TX: 0x%02X, RX: 0x%02X",
      i + 1, actuators_[i].get_tx_id(), actuators_[i].get_rx_id());
  }
}

void Hand::initialize_rx_dispatch_table_()
{
  for (size_t actuator_index = 0; actuator_index < kNumActuators; ++actuator_index) {
    rx_dispatch_table_[actuator_index] = {
      actuators_[actuator_index].get_rx_id(),
      actuator_index};
  }

  std::sort(
    rx_dispatch_table_.begin(),
    rx_dispatch_table_.end(),
    [](const RxDispatchEntry & lhs, const RxDispatchEntry & rhs) {
      return lhs.rx_id < rhs.rx_id;
    });
}

void Hand::dispatch_rx_frame_static_(void * context, const TPCANMsg & frame)
{
  static_cast<Hand *>(context)->dispatch_rx_frame_(frame);
}

void Hand::dispatch_rx_frame_(const TPCANMsg & frame)
{
  if (frame.MSGTYPE != PCAN_MESSAGE_STANDARD) {
    return;
  }

  const auto entry_it = std::lower_bound(
    rx_dispatch_table_.begin(),
    rx_dispatch_table_.end(),
    frame.ID,
    [](const RxDispatchEntry & entry, uint32_t rx_id) {
      return entry.rx_id < rx_id;
    });
  if (entry_it == rx_dispatch_table_.end() || entry_it->rx_id != frame.ID) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  actuators_[entry_it->actuator_index].process_message(frame);
}

void Hand::enable(bool automatic_zeroing)
{
  consecutive_read_failures_ = 0;
  zeroing_pending_ = automatic_zeroing;
  zeroing_probe_cooldown_cycles_ = 0;
  (void)enable_all_actuators();

  if (zeroing_pending_) {
    RCLCPP_INFO(
      logger(),
      "Automatic Plato zeroing requested; waiting for a full actuator feedback snapshot.");
  }
}

void Hand::disable()
{
  consecutive_read_failures_ = 0;
  zeroing_pending_ = false;
  zeroing_probe_cooldown_cycles_ = 0;
  (void)disable_all_actuators();
}

bool Hand::read()
{
  auto poll_result = can_bus_manager_.poll_once({this, &Hand::dispatch_rx_frame_static_});
  if (poll_result.hit_frame_budget && poll_result.read_status == PCAN_ERROR_OK) {
    const auto extra_poll_result =
      can_bus_manager_.poll_once({this, &Hand::dispatch_rx_frame_static_});
    accumulate_poll_result(poll_result, extra_poll_result);
  }

  if (
    poll_result.read_status != PCAN_ERROR_OK &&
    poll_result.read_status != PCAN_ERROR_QRCVEMPTY)
  {
    ++consecutive_read_failures_;
    RCLCPP_WARN_THROTTLE(
      logger(), throttle_clock(), 1000,
      "Plato CAN receive failed with status 0x%X (%zu consecutive failures)",
      poll_result.read_status, consecutive_read_failures_);
    if (consecutive_read_failures_ >= kMaxTransientReadFailures) {
      return false;
    }
  } else {
    consecutive_read_failures_ = 0;
  }

  read_joint_states_();

  if (zeroing_pending_) {
    if (has_zeroing_feedback_()) {
      const ZeroingResult zeroing_result = set_current_position_as_zero_(true);
      if (zeroing_result != ZeroingResult::kFailed) {
        zeroing_pending_ = false;
        zeroing_probe_cooldown_cycles_ = 0;
        if (zeroing_result == ZeroingResult::kRuntimeOnly) {
          RCLCPP_ERROR(
            logger(),
            "Automatic Plato zeroing applied in memory, but persisting actuator offsets failed.");
        }
      } else {
        zeroing_probe_cooldown_cycles_ = kZeroingProbeRetryPeriodCycles;
        RCLCPP_ERROR(
          logger(),
          "Automatic Plato zeroing failed after feedback became available.");
      }
    } else if (zeroing_probe_cooldown_cycles_ == 0) {
      request_feedback_probe_();
      zeroing_probe_cooldown_cycles_ = kZeroingProbeRetryPeriodCycles;
    } else {
      --zeroing_probe_cooldown_cycles_;
    }
  }

  return true;
}

bool Hand::write_joint_commands()
{
  enum class WritePreconditionFailure
  {
    kNone,
    kInvalidJointCommand,
    kMissingActuatorFeedback,
  };

  bool feedback_probe_needed = false;
  WritePreconditionFailure precondition_failure = WritePreconditionFailure::kNone;

  std::array<actuator::TxCommand, kNumActuators> commands{};
  size_t command_count = 0;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto joint_command = joint_command_view();

    if (!joint_command.all_finite()) {
      precondition_failure = WritePreconditionFailure::kInvalidJointCommand;
    } else {
      bool missing_geared_feedback = false;
      for (size_t i = kThumbMcpIndex; i < kNumActuators; ++i) {
        if (
          !std::isfinite(actuator_states_.position_at(i)) ||
          !std::isfinite(actuator_states_.velocity_at(i)) ||
          !std::isfinite(actuator_states_.effort_at(i)))
        {
          missing_geared_feedback = true;
          break;
        }
      }

      if (missing_geared_feedback) {
        feedback_probe_needed = true;
        precondition_failure = WritePreconditionFailure::kMissingActuatorFeedback;
      }
    }

    if (precondition_failure == WritePreconditionFailure::kNone) {
      transmission_.joint_to_actuator(joint_commands_, actuator_states_, joint_states_, actuator_commands_);
      const auto actuator_command = actuator_command_view();

      commands[command_count++] =
        actuators_[kThumbRollIndex].set_servo_position(
          static_cast<float>(actuator_command.position(kThumbRollIndex)));
      commands[command_count++] =
        actuators_[kThumbYawIndex].set_servo_position(
          static_cast<float>(actuator_command.position(kThumbYawIndex)));

      for (size_t i = kThumbMcpIndex; i < kNumActuators; ++i) {
        const Eigen::Index index = static_cast<Eigen::Index>(i);
        float actuator_torque = static_cast<float>(actuator_command.effort(index)) *
          kSteadywinTorqueScale;
        actuator_torque = std::clamp(actuator_torque, -kSteadywinTorqueLimit, kSteadywinTorqueLimit);
        commands[command_count++] = actuators_[i].set_joint_torque(actuator_torque);
      }
    }
  }

  if (feedback_probe_needed) {
    request_feedback_probe_();
  }

  if (precondition_failure != WritePreconditionFailure::kNone) {
    switch (precondition_failure) {
      case WritePreconditionFailure::kInvalidJointCommand:
        RCLCPP_WARN_THROTTLE(
          logger(), throttle_clock(), 1000,
          "Skipping Plato write because the joint command buffer contains non-finite values.");
        break;
      case WritePreconditionFailure::kMissingActuatorFeedback:
        RCLCPP_WARN_THROTTLE(
          logger(), throttle_clock(), 1000,
          "Skipping Plato write because actuator feedback is incomplete; sent a feedback probe.");
        break;
      case WritePreconditionFailure::kNone:
        break;
    }
    return true;
  }

  TPCANStatus first_error = PCAN_ERROR_OK;
  for (size_t i = 0; i < command_count; ++i) {
    const TPCANStatus status = send_command_(commands[i]);
    if (first_error == PCAN_ERROR_OK && status != PCAN_ERROR_OK) {
      first_error = status;
    }
  }

  if (first_error == PCAN_ERROR_OK) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    capture_joint_commands();
    capture_actuator_commands();
    return true;
  }

  RCLCPP_ERROR_THROTTLE(
    logger(), throttle_clock(), 1000,
    "Plato write path failed with transport status 0x%X",
    first_error);
  return false;
}

TPCANStatus Hand::send_command_(const actuator::TxCommand & command)
{
  return can_bus_manager_.send_frame(command.frame, command.post_send_delay);
}

void Hand::read_joint_states_()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  joint_states_.fill(kInvalidStateValue);

  bool has_full_geared_feedback = true;
  for (size_t i = 0; i < kNumActuators; ++i) {
    actuator_states_.position_at(i) = kInvalidStateValue;
    actuator_states_.velocity_at(i) = kInvalidStateValue;
    actuator_states_.effort_at(i) = kInvalidStateValue;

    if (!actuators_[i].has_feedback()) {
      if (i >= kThumbMcpIndex) {
        has_full_geared_feedback = false;
      }
      continue;
    }

    const auto & feedback = actuators_[i].get_feedback();
    actuator_states_.position_at(i) = feedback.position;
    actuator_states_.velocity_at(i) = feedback.velocity;
    actuator_states_.effort_at(i) = feedback.torque;
  }

  if (!has_full_geared_feedback) {
    return;
  }

  transmission_.actuator_to_joint(actuator_states_, joint_states_);
}

bool Hand::has_zeroing_feedback_() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return std::all_of(
    actuators_.begin() + static_cast<std::ptrdiff_t>(kThumbMcpIndex),
    actuators_.end(),
    [](const auto & actuator) { return actuator.has_feedback(); });
}

void Hand::request_feedback_probe_()
{
  std::vector<actuator::TxCommand> commands;
  commands.reserve(kNumActuators - kThumbMcpIndex);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t actuator_index = 0; actuator_index < actuators_.size(); ++actuator_index) {
      auto & actuator = actuators_[actuator_index];
      if (actuator.has_feedback()) {
        continue;
      }

      if (actuator_index == kThumbRollIndex || actuator_index == kThumbYawIndex) {
        continue;
      }

      // Steadywin only returns state on control replies, so use a zero-torque command as a
      // non-driving feedback probe for geared joints that have not reported yet.
      commands.push_back(actuator.set_joint_torque(0.0f));
    }
  }

  TPCANStatus first_error = PCAN_ERROR_OK;
  for (const auto & command : commands) {
    const TPCANStatus status = send_command_(command);
    if (first_error == PCAN_ERROR_OK && status != PCAN_ERROR_OK) {
      first_error = status;
    }
  }

  if (first_error != PCAN_ERROR_OK) {
    RCLCPP_WARN_THROTTLE(
      logger(), throttle_clock(), 1000,
      "Plato feedback probe failed with transport status 0x%X",
      first_error);
  }
}

Hand::ZeroingResult Hand::set_current_position_as_zero_(bool persist_offsets)
{
  plato_actuator::PositionOffsets offsets;
  offsets.reserve(kNumActuators);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < kNumActuators; ++i) {
      if (i == kThumbRollIndex || i == kThumbYawIndex) {
        continue;
      }

      if (!actuators_[i].has_feedback()) {
        RCLCPP_WARN(
          logger(),
          "Cannot zero actuator %zu because no feedback has been received yet",
          i + 1);
        return ZeroingResult::kFailed;
      }
    }

    for (size_t i = 0; i < kNumActuators; ++i) {
      if (i == kThumbRollIndex || i == kThumbYawIndex) {
        offsets.push_back(actuator_configs_[i].core.position_offset);

        if (actuators_[i].has_feedback()) {
          const auto & feedback = actuators_[i].get_feedback();
          actuator_states_.position_at(i) = feedback.position;
          actuator_states_.velocity_at(i) = feedback.velocity;
          actuator_states_.effort_at(i) = feedback.torque;
        }
        continue;
      }

      if (!actuators_[i].set_current_position_as_zero()) {
        RCLCPP_WARN(
          logger(),
          "Cannot zero actuator %zu because software zeroing preconditions changed unexpectedly",
          i + 1);
        return ZeroingResult::kFailed;
      }

      actuator_configs_[i].core.position_offset = actuators_[i].get_position_offset();
      offsets.push_back(actuators_[i].get_position_offset());

      const auto & feedback = actuators_[i].get_feedback();
      actuator_states_.position_at(i) = feedback.position;
      actuator_states_.velocity_at(i) = feedback.velocity;
      actuator_states_.effort_at(i) = feedback.torque;
    }

    transmission_.actuator_to_joint(actuator_states_, joint_states_);
  }

  if (persist_offsets) {
    try {
      plato_actuator::save_plato_actuator_position_offsets(offsets, actuator_offset_yaml_path_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Failed to persist Plato actuator offsets: %s", e.what());
      return ZeroingResult::kRuntimeOnly;
    }
  }

  RCLCPP_INFO(logger(), "Set current Plato actuator positions as software zero");
  return persist_offsets ? ZeroingResult::kRuntimeAndPersisted : ZeroingResult::kRuntimeOnly;
}

void Hand::print_motor_positions()
{
  auto logger = rclcpp::get_logger("plato_hardware_interface");
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < actuators_.size(); ++i) {
    RCLCPP_INFO(logger, "J%zu Motor Position: %.4f", i + 1, actuators_[i].get_motor_position());
  }
}

void Hand::print_actuator_info_() const
{
  print_hardware_info_("Total Number of Actuators");
}

}  // namespace plato_hand
