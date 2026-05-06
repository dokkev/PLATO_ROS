#include "plato_hardware_interface/plato_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "plato_hardware_interface/dynamixel_can_protocol.hpp"

namespace plato_hand
{
namespace
{
constexpr std::size_t kThumbRollIndex = 0;
constexpr std::size_t kThumbYawIndex = 1;
constexpr std::size_t kThumbMcpIndex = 2;
constexpr std::size_t kServoWriteDivisor = 1;
constexpr double kMaxServoCurrentCommandMilliamps =
  static_cast<double>(std::numeric_limits<uint32_t>::max());
constexpr double kMaxDerivedThumbServoCurrentMilliamps = 2000.0;

uint8_t dynamixel_servo_id_for_index(std::size_t actuator_index)
{
  return static_cast<uint8_t>(actuator_index + 1U);
}

uint32_t clamp_servo_current_command(double servo_current_milliamps)
{
  const double clamped =
    std::clamp(servo_current_milliamps, 0.0, kMaxServoCurrentCommandMilliamps);
  return static_cast<uint32_t>(std::llround(clamped));
}

TPCANMsg make_servo_position_command(
  plato_actuator::Actuator & actuator,
  std::size_t actuator_index,
  float actuator_position,
  float stiffness,
  double servo_stiffness_scale)
{
  double resolved_servo_current_milliamps = std::numeric_limits<double>::quiet_NaN();
  if (std::isfinite(stiffness) && servo_stiffness_scale > 0.0) {
    resolved_servo_current_milliamps = std::clamp(
      std::abs(stiffness) * servo_stiffness_scale,
      0.0,
      kMaxDerivedThumbServoCurrentMilliamps);
  }

  if (!std::isfinite(resolved_servo_current_milliamps)) {
    resolved_servo_current_milliamps = actuator.get_static_config().servo_current_milliamps;
  }

  const auto current_command = clamp_servo_current_command(resolved_servo_current_milliamps);
  return plato_hardware_interface::dynamixel_can_protocol::make_position_command(
    actuator.get_tx_id(),
    dynamixel_servo_id_for_index(actuator_index),
    actuator_position,
    static_cast<uint16_t>(current_command));
}

}  // namespace

void PlatoProtocol::append_enable_frames(
  std::vector<plato_actuator::Actuator> & actuators,
  std::vector<TPCANMsg> & direct_frames) const
{
  const auto gim_count = actuators.size() > kThumbMcpIndex ? actuators.size() - kThumbMcpIndex : 0U;
  direct_frames.reserve(direct_frames.size() + gim_count + kThumbMcpIndex);
  if (actuators.size() > kThumbRollIndex) {
    direct_frames.push_back(plato_hardware_interface::dynamixel_can_protocol::make_enable_command(
      actuators[kThumbRollIndex].get_tx_id(),
      dynamixel_servo_id_for_index(kThumbRollIndex)));
  }
  if (actuators.size() > kThumbYawIndex) {
    direct_frames.push_back(plato_hardware_interface::dynamixel_can_protocol::make_enable_command(
      actuators[kThumbYawIndex].get_tx_id(),
      dynamixel_servo_id_for_index(kThumbYawIndex)));
  }
  for (std::size_t i = kThumbMcpIndex; i < actuators.size(); ++i) {
    direct_frames.push_back(actuators[i].enable_motor().frame);
  }
}

void PlatoProtocol::append_disable_frames(
  std::vector<plato_actuator::Actuator> & actuators,
  std::vector<TPCANMsg> & direct_frames) const
{
  const auto gim_count = actuators.size() > kThumbMcpIndex ? actuators.size() - kThumbMcpIndex : 0U;
  direct_frames.reserve(direct_frames.size() + gim_count + kThumbMcpIndex);
  if (actuators.size() > kThumbRollIndex) {
    direct_frames.push_back(plato_hardware_interface::dynamixel_can_protocol::make_disable_command(
      actuators[kThumbRollIndex].get_tx_id(),
      dynamixel_servo_id_for_index(kThumbRollIndex)));
  }
  if (actuators.size() > kThumbYawIndex) {
    direct_frames.push_back(plato_hardware_interface::dynamixel_can_protocol::make_disable_command(
      actuators[kThumbYawIndex].get_tx_id(),
      dynamixel_servo_id_for_index(kThumbYawIndex)));
  }
  for (std::size_t i = kThumbMcpIndex; i < actuators.size(); ++i) {
    direct_frames.push_back(actuators[i].disable_motor().frame);
  }
}

bool PlatoProtocol::process_rx_frame(
  const TPCANMsg & frame,
  std::vector<plato_actuator::Actuator> & actuators) const
{
  if (frame.MSGTYPE != PCAN_MESSAGE_STANDARD || frame.LEN < 2) {
    return false;
  }

  const bool is_thumb_servo_response =
    actuators.size() > kThumbYawIndex &&
    (frame.ID == actuators[kThumbRollIndex].get_rx_id() ||
    frame.ID == actuators[kThumbYawIndex].get_rx_id());
  if (is_thumb_servo_response) {
    const auto response =
      plato_hardware_interface::dynamixel_can_protocol::decode_response(frame);
    if (response) {
      const auto actuator_index = static_cast<std::size_t>(response->servo_id - 1U);
      if (actuator_index <= kThumbYawIndex) {
        const bool enabled =
          response->command == plato_hardware_interface::dynamixel_can_protocol::Command::kEnable &&
          plato_hardware_interface::dynamixel_can_protocol::is_success(response->result);
        const bool disabled =
          response->command == plato_hardware_interface::dynamixel_can_protocol::Command::kDisable &&
          plato_hardware_interface::dynamixel_can_protocol::is_success(response->result);
        if (enabled || disabled) {
          actuators[actuator_index].set_motor_enabled(enabled);
        }
        return true;
      }
    }

    return frame.DATA[0] == CommandByte::POSITION_CONTROL;
  }

  for (auto & actuator : actuators) {
    if (actuator.get_rx_id() != frame.ID) {
      continue;
    }
    actuator.process_message(frame);
    return true;
  }

  return false;
}

void PlatoProtocol::append_write_frames(
  std::vector<plato_actuator::Actuator> & actuators,
  const ActuatorCommand & actuator_cmd,
  std::size_t write_cycle_count,
  double servo_stiffness_scale,
  std::vector<TPCANMsg> & direct_frames) const
{
  direct_frames.reserve(actuators.size());

  if ((write_cycle_count % kServoWriteDivisor) == 0) {
    direct_frames.push_back(make_servo_position_command(
      actuators[kThumbRollIndex],
      kThumbRollIndex,
      actuator_cmd.position_at(kThumbRollIndex),
      actuator_cmd.stiffness_at(kThumbRollIndex),
      servo_stiffness_scale));
    direct_frames.push_back(make_servo_position_command(
      actuators[kThumbYawIndex],
      kThumbYawIndex,
      actuator_cmd.position_at(kThumbYawIndex),
      actuator_cmd.stiffness_at(kThumbYawIndex),
      servo_stiffness_scale));
  }

  for (std::size_t i = kThumbMcpIndex; i < actuators.size(); ++i) {
    direct_frames.push_back(
      actuators[i].set_joint_torque(static_cast<float>(actuator_cmd.effort_at(i))).frame);
  }
}

}  // namespace plato_hand
