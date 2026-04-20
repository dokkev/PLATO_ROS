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

using LifecyclePlan = can_hardware_common::core::LifecyclePlan;
using WritePlan = can_hardware_common::core::WritePlan;

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

}  // namespace

// ════════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════════════════

Hand::Hand(PlatoHandConfig config)
: transmission_(config.linkage_config, build_joint_effort_limits(config.actuator_configs)),
  protocol_(),
  model_(transmission_),
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
  state_snapshot_.resize(kNumJoints, kNumActuators);

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
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (protocol_.process_rx_frame(frame, actuators_)) {
      mark_rx_frame_();
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

  // Deterministic paced direct TX path for write().
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

bool Hand::update_measurements_()
{
  const auto rx = transport_.process_rx();
  last_rx_healthy_ = !rx.is_bus_error();
  if (rx.is_bus_error()) {
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

  const auto joint_state = joint_state_view();
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
  state_snapshot_.joint_position = joint_state.position.matrix();
  state_snapshot_.joint_velocity = joint_state.velocity.matrix();
  state_snapshot_.joint_effort = joint_state.effort.matrix();

  if (state_snapshot_.actuator_states.size() != kNumActuators) {
    state_snapshot_.resize(kNumJoints, kNumActuators);
  }
  model_.copy_feedback_snapshot(actuators_, state_snapshot_);
}

can_hardware_common::core::LifecyclePlan Hand::build_lifecycle_plan_(
  can_hardware_common::core::LifecycleOperation operation)
{
  return protocol_.build_lifecycle_plan(actuators_, operation);
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
  apply_lifecycle_request_flags_(enabling);

  bool success = true;
  for (size_t i = 0; i < plan.scheduled_requests.size(); ++i) {
    const auto res = scheduler_.execute_blocking(
      plan.scheduled_requests[i], kResponseTimeout, kLifecycleCommandRetries);
    success = handle_standard_lifecycle_result_(i, res, enabling) && success;
  }

  return success;
}

bool Hand::execute_zero_lifecycle_()
{
  return zero_actuators_();
}

bool Hand::handle_standard_lifecycle_result_(
  std::size_t actuator_index,
  const TransactionResult & result,
  bool enabling)
{
  if (is_thumb_servo_index(actuator_index) && tolerate_thumb_lifecycle_result(result, enabling)) {
    actuators_[actuator_index].set_motor_enabled(enabling);
    if (result.status != TransactionResult::Status::kConfirmed) {
      RCLCPP_WARN(
        logger(),
        "%s thumb servo actuator %zu tolerated without strict confirmation: %s (result_byte=0x%02X)",
        enabling ? "Enable" : "Disable",
        actuator_index + 1,
        TransactionResult::status_label(result.status),
        result.result_byte);
    }
    return true;
  }

  if (result.status != TransactionResult::Status::kConfirmed) {
    RCLCPP_ERROR(
      logger(),
      "%s actuator %zu: %s",
      enabling ? "Enable" : "Disable",
      actuator_index + 1,
      TransactionResult::status_label(result.status));
    return false;
  }

  return true;
}

void Hand::apply_lifecycle_request_flags_(bool enabling)
{
  if (enabling) {
    enable_requested_ = true;
    disable_requested_ = false;
    return;
  }

  disable_requested_ = true;
}

void Hand::build_ready_write_plan_(WritePlan & plan)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    model_.joint_to_actuator_commands(
      joint_commands_, actuator_states_, joint_states_, actuator_commands_);
    const auto actuator_cmd = actuator_command_view();
    plan.computed_actuator_command.capture(actuator_cmd);
    protocol_.append_write_frames(
      actuators_, actuator_cmd, write_cycle_count_, servo_stiffness_scale_, plan.direct_frames);
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
