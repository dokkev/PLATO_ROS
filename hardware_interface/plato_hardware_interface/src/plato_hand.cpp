#include "plato_hardware_interface/plato_hand.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
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
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kSecondsPerMinute = 60.0f;
constexpr double kMaxServoCurrentCommandMilliamps =
  static_cast<double>(std::numeric_limits<uint32_t>::max());
constexpr double kMaxDerivedThumbServoCurrentMilliamps = 2000.0;

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

bool is_thumb_servo_index(size_t index)
{
  return index == 0 || index == 1;
}

bool tolerate_thumb_lifecycle_result(
  const can_hardware_common::CanCommandScheduler::TransactionResult & result,
  bool enabling)
{
  if (result.status == can_hardware_common::CanCommandScheduler::TransactionResult::Status::kConfirmed) {
    return true;
  }
  if (result.status == can_hardware_common::CanCommandScheduler::TransactionResult::Status::kTimeout) {
    return true;
  }
  if (
    enabling &&
    result.status == can_hardware_common::CanCommandScheduler::TransactionResult::Status::kRejected &&
    result.result_byte == ResultByte::FAILURE)
  {
    return true;
  }
  return false;
}

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
    FiveBarLinkage::Transmission::JointArray::Constant(
    std::numeric_limits<float>::infinity());
  for (size_t i = 0; i < actuator_configs.size(); ++i) {
    limits(static_cast<Eigen::Index>(i)) = actuator_configs[i].static_config.effort_limit_nm;
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

uint32_t clamp_servo_current_command(double servo_current_milliamps)
{
  const double clamped =
    std::clamp(servo_current_milliamps, 0.0, kMaxServoCurrentCommandMilliamps);
  return static_cast<uint32_t>(std::llround(clamped));
}

TPCANMsg make_servo_position_command(
  plato_actuator::Actuator & actuator,
  double actuator_position,
  double stiffness,
  double servo_stiffness_scale)
{
  const auto joint_position = static_cast<float>(actuator_position);
  double resolved_servo_current_milliamps = std::numeric_limits<double>::quiet_NaN();
  if (std::isfinite(stiffness) && servo_stiffness_scale > 0.0) {
    resolved_servo_current_milliamps = std::clamp(
      std::abs(stiffness) * servo_stiffness_scale,
      0.0,
      kMaxDerivedThumbServoCurrentMilliamps);
  }

  if (!std::isfinite(resolved_servo_current_milliamps)) {
    return actuator.set_servo_hold(joint_position).frame;
  }

  const auto current_command = clamp_servo_current_command(resolved_servo_current_milliamps);
  if (current_command == 0U) {
    return actuator.set_servo_idle(joint_position).frame;
  }

  return actuator.set_servo_position(joint_position, current_command).frame;
}
}  // namespace

// ════════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════════════════

Hand::Hand(PlatoHandConfig config)
: transmission_(config.linkage_config, build_joint_effort_limits(config.actuator_configs)),
  scheduler_(transport_),  // must follow transport_ and transmission_ in member order
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

  actuators_.reserve(kNumActuators);
  for (const auto & cfg : config.actuator_configs) {
    actuators_.emplace_back(cfg);
  }

  transport_simulator_enabled_ = config.enable_transport_simulator;
  disable_on_destruction_ = config.disable_on_destruction;
  simulator_states_.assign(kNumActuators, SimActuatorState{});
  if (transport_simulator_enabled_) {
    configure_transport_simulator_(config.transport_simulator_bypass_hardware);
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
  RCLCPP_INFO(
    logger(),
    "Direct TX frame timeout: %ld us",
    static_cast<long>(direct_tx_frame_timeout_.count()));
  if (transport_simulator_enabled_) {
    RCLCPP_WARN(
      logger(),
      "Plato hand transport simulator enabled (bypass_hardware=%s)",
      config.transport_simulator_bypass_hardware ? "true" : "false");
  }
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

bool Hand::enable(bool automatic_zeroing)
{
  enable_requested_ = true;
  disable_requested_ = false;
  bool success = true;

  for (size_t i = 0; i < kNumActuators; ++i) {
    const auto req = actuators_[i].make_enable_request(static_cast<uint32_t>(i));
    const auto res = scheduler_.execute_blocking(
      req, kResponseTimeout, kLifecycleCommandRetries);
    if (is_thumb_servo_index(i) && tolerate_thumb_lifecycle_result(res, true)) {
      actuators_[i].set_motor_enabled(true);
      if (res.status != TransactionResult::Status::kConfirmed) {
        RCLCPP_WARN(
          logger(),
          "Enable thumb servo actuator %zu tolerated without strict confirmation: %s (result_byte=0x%02X)",
          i + 1,
          TransactionResult::status_label(res.status),
          res.result_byte);
      }
      continue;
    }
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
  disable_requested_ = true;
  bool success = true;

  for (size_t i = 0; i < kNumActuators; ++i) {
    const auto req = actuators_[i].make_disable_request(static_cast<uint32_t>(i));
    const auto res = scheduler_.execute_blocking(
      req, kResponseTimeout, kLifecycleCommandRetries);
    if (is_thumb_servo_index(i) && tolerate_thumb_lifecycle_result(res, false)) {
      actuators_[i].set_motor_enabled(false);
      if (res.status != TransactionResult::Status::kConfirmed) {
        RCLCPP_WARN(
          logger(),
          "Disable thumb servo actuator %zu tolerated without strict confirmation: %s (result_byte=0x%02X)",
          i + 1,
          TransactionResult::status_label(res.status),
          res.result_byte);
      }
      continue;
    }
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
  bool have_initialized_actuator_feedback = false;

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

    have_initialized_actuator_feedback = std::any_of(
      actuators_.begin(),
      actuators_.end(),
      [](const auto & actuator) { return actuator.is_initialized(); });

    if ((write_cycle_count_ % kServoWriteDivisor) == 0) {
      tx_frames[tx_count++] = make_servo_position_command(
        actuators_[kThumbRollIndex],
        actuator_cmd.position(kThumbRollIndex),
        actuator_cmd.stiffness(kThumbRollIndex),
        servo_stiffness_scale_);
      tx_frames[tx_count++] = make_servo_position_command(
        actuators_[kThumbYawIndex],
        actuator_cmd.position(kThumbYawIndex),
        actuator_cmd.stiffness(kThumbYawIndex),
        servo_stiffness_scale_);
    }

    for (size_t i = kThumbMcpIndex; i < kNumActuators; ++i) {
      const Eigen::Index idx = static_cast<Eigen::Index>(i);
      tx_frames[tx_count++] = actuators_[i].set_joint_torque(
        static_cast<float>(actuator_cmd.effort(idx))).frame;
    }

    ++write_cycle_count_;
    capture_joint_commands();
    capture_actuator_commands();
  }

  const auto now = SteadyClock::now();
  if (have_initialized_actuator_feedback && !has_fresh_rx_(now)) {
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
        "RX stale; attempting direct TX recovery (age=%ld ms, stale_cycles=%zu, total_rx=%zu)",
        rx_age_ms,
        stale_write_cycle_count_,
        rx_frame_count_);
    }
  } else {
    stale_write_cycle_count_ = 0;
  }

  for (size_t i = 0; i < tx_count; ++i) {
    if (!send_frame_blocking_(tx_frames[i], direct_tx_frame_timeout_)) {
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

void Hand::configure_transport_simulator_(bool bypass_hardware)
{
  transport_.set_tx_simulator(
    [this](const TPCANMsg & tx_frame) {
      return simulate_tx_frame_(tx_frame);
    },
    bypass_hardware);
}

std::vector<TPCANMsg> Hand::simulate_tx_frame_(const TPCANMsg & tx_frame)
{
  if (tx_frame.MSGTYPE != PCAN_MESSAGE_STANDARD || tx_frame.LEN < 1) {
    return {};
  }

  std::lock_guard<std::mutex> lock(simulator_state_mutex_);

  size_t actuator_index = kNumActuators;
  for (size_t i = 0; i < kNumActuators; ++i) {
    if (actuators_[i].get_tx_id() == tx_frame.ID) {
      actuator_index = i;
      break;
    }
  }
  if (actuator_index >= kNumActuators) {
    return {};
  }

  auto & sim = simulator_states_[actuator_index];
  const uint32_t rx_id = actuators_[actuator_index].get_rx_id();
  const uint8_t opcode = tx_frame.DATA[0];

  switch (opcode) {
    case CommandByte::START_MOTOR:
      sim.motor_enabled = true;
      sim.motor_velocity_rpm = 0.0f;
      sim.motor_torque = 0.0f;
      return {make_ack_frame_(rx_id, opcode)};
    case CommandByte::STOP_MOTOR:
    case CommandByte::STOP_CONTROL:
      sim.motor_enabled = false;
      sim.motor_velocity_rpm = 0.0f;
      sim.motor_torque = 0.0f;
      return {make_ack_frame_(rx_id, opcode)};
    case CommandByte::TORQUE_CONTROL: {
        if (!sim.motor_enabled) {
          return {make_ack_frame_(rx_id, opcode, ResultByte::FAILURE)};
        }

        const float commanded_torque = decode_float_le_(tx_frame, 1);
        sim.motor_torque = std::clamp(commanded_torque, -9.8f, 9.8f);
        sim.motor_velocity_rpm =
          std::clamp(0.90f * sim.motor_velocity_rpm + sim.motor_torque * 8.0f, -65.0f, 65.0f);
        sim.motor_position += (sim.motor_velocity_rpm * kTwoPi / kSecondsPerMinute) * 0.002f;
        return {
          make_state_frame_(
            rx_id,
            opcode,
            sim.temperature,
            sim.motor_position,
            sim.motor_velocity_rpm,
            sim.motor_torque)};
      }
    case CommandByte::POSITION_CONTROL: {
        if (!sim.motor_enabled) {
          return {make_ack_frame_(rx_id, opcode, ResultByte::FAILURE)};
        }

        const float target_position = decode_float_le_(tx_frame, 1);
        const float delta = target_position - sim.motor_position;
        sim.motor_velocity_rpm =
          std::clamp(delta * (25.0f * kSecondsPerMinute / kTwoPi), -65.0f, 65.0f);
        sim.motor_position = target_position;
        sim.motor_torque = std::clamp(delta * 0.5f, -9.8f, 9.8f);
        return {
          make_state_frame_(
            rx_id,
            opcode,
            sim.temperature,
            sim.motor_position,
            sim.motor_velocity_rpm,
            sim.motor_torque)};
      }
    default:
      break;
  }

  return {};
}

TPCANMsg Hand::make_ack_frame_(uint32_t rx_id, uint8_t opcode, uint8_t result)
{
  TPCANMsg frame{};
  frame.ID = rx_id;
  frame.MSGTYPE = PCAN_MESSAGE_STANDARD;
  frame.LEN = 2;
  frame.DATA[0] = opcode;
  frame.DATA[1] = result;
  return frame;
}

TPCANMsg Hand::make_state_frame_(
  uint32_t rx_id,
  uint8_t opcode,
  uint8_t temperature,
  float motor_position,
  float motor_velocity_rpm,
  float motor_torque)
{
  TPCANMsg frame{};
  frame.ID = rx_id;
  frame.MSGTYPE = PCAN_MESSAGE_STANDARD;
  frame.LEN = 8;
  frame.DATA[0] = opcode;
  frame.DATA[1] = ResultByte::SUCCESS;
  frame.DATA[2] = temperature;

  const float clamped_position = std::clamp(motor_position, -12.5f, 12.5f);
  const uint16_t position_raw = static_cast<uint16_t>(std::lround(
      (clamped_position + 12.5f) * 65535.0f / 25.0f));
  frame.DATA[3] = static_cast<uint8_t>(position_raw & 0xFF);
  frame.DATA[4] = static_cast<uint8_t>((position_raw >> 8) & 0xFF);

  const float clamped_velocity = std::clamp(motor_velocity_rpm, -65.0f, 65.0f);
  const uint16_t velocity_raw = static_cast<uint16_t>(std::lround(
      (clamped_velocity + 65.0f) * 4095.0f / 130.0f));
  frame.DATA[5] = static_cast<uint8_t>((velocity_raw >> 4) & 0xFF);

  const float clamped_torque = std::clamp(motor_torque, -9.8f, 9.8f);
  const uint16_t torque_raw = static_cast<uint16_t>(std::lround(
      2048.0f + clamped_torque * (2047.0f / 9.8f)));
  frame.DATA[6] = static_cast<uint8_t>(((velocity_raw & 0x0F) << 4) | ((torque_raw >> 8) & 0x0F));
  frame.DATA[7] = static_cast<uint8_t>(torque_raw & 0xFF);

  return frame;
}

float Hand::decode_float_le_(const TPCANMsg & frame, size_t offset)
{
  if (offset + sizeof(float) > frame.LEN) {
    return 0.0f;
  }

  float value = 0.0f;
  std::memcpy(&value, &frame.DATA[offset], sizeof(float));
  return value;
}

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
  bool used_fallback_feedback = false;
  std::string fallback_feedback_ids;
  for (size_t i = 0; i < kNumActuators; ++i) {
    if (!actuators_[i].is_initialized()) {
      if (i == kThumbRollIndex || i == kThumbYawIndex) {
        // Thumb servo channels may not report continuous state feedback on hold commands.
        // Preserve joint-state updates by mirroring their commanded hold position.
        actuator_states_.position_at(i) = joint_commands_.position_at(i);
        actuator_states_.velocity_at(i) = 0.0;
        actuator_states_.effort_at(i) = 0.0;
        continue;
      }

      const bool have_cached_feedback =
        std::isfinite(actuator_states_.position_at(i)) &&
        std::isfinite(actuator_states_.velocity_at(i)) &&
        std::isfinite(actuator_states_.effort_at(i));
      if (!have_cached_feedback) {
        actuator_states_.position_at(i) = 0.0;
        actuator_states_.velocity_at(i) = 0.0;
        actuator_states_.effort_at(i) = 0.0;
      }

      used_fallback_feedback = true;
      if (!fallback_feedback_ids.empty()) {
        fallback_feedback_ids += ", ";
      }
      fallback_feedback_ids += std::to_string(i + 1);
      if (!have_cached_feedback) {
        fallback_feedback_ids += "*";
      }
      continue;
    }
    const auto & state = actuators_[i].get_state();
    actuator_states_.position_at(i) = state.position;
    actuator_states_.velocity_at(i) = state.velocity;
    actuator_states_.effort_at(i) = state.torque;
  }

  if (used_fallback_feedback) {
    RCLCPP_WARN_THROTTLE(
      logger(),
      throttle_clock(),
      1000,
      "Joint-state update using fallback feedback for actuator(s) [%s] ('*' means zero fallback; others use cached state)",
      fallback_feedback_ids.c_str());
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
