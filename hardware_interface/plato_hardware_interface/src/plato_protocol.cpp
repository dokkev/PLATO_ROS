#include "plato_hardware_interface/plato_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

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

void PlatoProtocol::append_enable_frames(
  std::vector<plato_actuator::Actuator> & actuators,
  std::vector<TPCANMsg> & direct_frames) const
{
  direct_frames.reserve(direct_frames.size() + actuators.size());
  for (auto & actuator : actuators) {
    direct_frames.push_back(actuator.enable_motor().frame);
  }
}

void PlatoProtocol::append_disable_frames(
  std::vector<plato_actuator::Actuator> & actuators,
  std::vector<TPCANMsg> & direct_frames) const
{
  direct_frames.reserve(direct_frames.size() + actuators.size());
  for (auto & actuator : actuators) {
    direct_frames.push_back(actuator.disable_motor().frame);
  }
}

bool PlatoProtocol::process_rx_frame(
  const TPCANMsg & frame,
  std::vector<plato_actuator::Actuator> & actuators) const
{
  if (frame.MSGTYPE != PCAN_MESSAGE_STANDARD || frame.LEN < 2) {
    return false;
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
  const ActuatorCommandView & actuator_cmd,
  std::size_t write_cycle_count,
  double servo_stiffness_scale,
  std::vector<TPCANMsg> & direct_frames) const
{
  direct_frames.reserve(actuators.size());

  if ((write_cycle_count % kServoWriteDivisor) == 0) {
    direct_frames.push_back(make_servo_position_command(
      actuators[kThumbRollIndex],
      actuator_cmd.position(kThumbRollIndex),
      actuator_cmd.stiffness(kThumbRollIndex),
      servo_stiffness_scale));
    direct_frames.push_back(make_servo_position_command(
      actuators[kThumbYawIndex],
      actuator_cmd.position(kThumbYawIndex),
      actuator_cmd.stiffness(kThumbYawIndex),
      servo_stiffness_scale));
  }

  for (std::size_t i = kThumbMcpIndex; i < actuators.size(); ++i) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    direct_frames.push_back(
      actuators[i].set_joint_torque(static_cast<float>(actuator_cmd.effort(idx))).frame);
  }
}

}  // namespace plato_hand
