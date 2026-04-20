#include "aristo_hardware_interface/aristo_protocol.hpp"

#include <algorithm>

namespace aristo_hand
{

AristoProtocol::LifecyclePlan AristoProtocol::build_lifecycle_plan(
  std::vector<aristo_actuator::Actuator> & actuators,
  can_hardware_common::core::LifecycleOperation operation) const
{
  LifecyclePlan plan;
  plan.operation = operation;
  plan.ready = true;
  plan.dispatch_policy = can_hardware_common::core::DispatchPolicy::kDirectFrames;
  plan.direct_frames.reserve(actuators.size());

  for (auto & actuator : actuators) {
    switch (operation) {
      case can_hardware_common::core::LifecycleOperation::kEnable:
        plan.direct_frames.push_back(actuator.enable_motor().frame);
        break;
      case can_hardware_common::core::LifecycleOperation::kDisable:
        plan.direct_frames.push_back(actuator.disable_motor().frame);
        break;
      case can_hardware_common::core::LifecycleOperation::kZero:
        plan.direct_frames.push_back(actuator.set_current_position_as_zero().frame);
        break;
    }
  }

  return plan;
}

void AristoProtocol::initialize_rx_dispatch(
  const std::vector<aristo_actuator::Actuator> & actuators,
  const std::vector<sensor::FTSensor> & ft_sensors)
{
  rx_dispatch_table_.clear();
  rx_dispatch_table_.reserve(actuators.size() + (2 * ft_sensors.size()));

  for (std::size_t actuator_index = 0; actuator_index < actuators.size(); ++actuator_index) {
    rx_dispatch_table_.push_back(
      {actuators[actuator_index].get_rx_id(), DispatchTargetKind::kActuator, actuator_index});
  }

  for (std::size_t sensor_index = 0; sensor_index < ft_sensors.size(); ++sensor_index) {
    rx_dispatch_table_.push_back(
      {ft_sensors[sensor_index].get_force_rx_id(), DispatchTargetKind::kForceSensor, sensor_index});
    rx_dispatch_table_.push_back(
      {ft_sensors[sensor_index].get_torque_rx_id(), DispatchTargetKind::kTorqueSensor, sensor_index});
  }

  std::sort(
    rx_dispatch_table_.begin(),
    rx_dispatch_table_.end(),
    [](const RxDispatchEntry & lhs, const RxDispatchEntry & rhs) {
      return lhs.rx_id < rhs.rx_id;
    });
}

bool AristoProtocol::process_rx_frame(
  const TPCANMsg & frame,
  std::vector<aristo_actuator::Actuator> & actuators,
  std::vector<sensor::FTSensor> & ft_sensors) const
{
  if (frame.MSGTYPE != PCAN_MESSAGE_STANDARD) {
    return false;
  }

  const auto entry_it = std::lower_bound(
    rx_dispatch_table_.begin(),
    rx_dispatch_table_.end(),
    frame.ID,
    [](const RxDispatchEntry & entry, uint32_t rx_id) {
      return entry.rx_id < rx_id;
    });
  if (entry_it == rx_dispatch_table_.end() || entry_it->rx_id != frame.ID) {
    return false;
  }

  switch (entry_it->target_kind) {
    case DispatchTargetKind::kActuator:
      actuators[entry_it->target_index].process_message(frame);
      break;
    case DispatchTargetKind::kForceSensor:
    case DispatchTargetKind::kTorqueSensor:
      ft_sensors[entry_it->target_index].process_message(frame);
      break;
  }

  return true;
}

void AristoProtocol::append_impedance_frames(
  std::vector<aristo_actuator::Actuator> & actuators,
  const std::vector<can_hardware_common::ActuatorTarget> & impedance_targets,
  std::vector<TPCANMsg> & direct_frames) const
{
  direct_frames.reserve(impedance_targets.size());
  for (std::size_t i = 0; i < impedance_targets.size() && i < actuators.size(); ++i) {
    if (const auto command = actuators[i].set_joint_impedance(impedance_targets[i])) {
      direct_frames.push_back(command->frame);
    }
  }
}

}  // namespace aristo_hand
