#include "aristo_hardware_interface/aristo_protocol.hpp"

#include <algorithm>

namespace aristo_hand
{

void AristoProtocol::append_enable_frames(
  std::vector<aristo_actuator::Actuator> & actuators,
  std::vector<TPCANMsg> & direct_frames) const
{
  direct_frames.reserve(direct_frames.size() + actuators.size());
  for (auto & actuator : actuators) {
    direct_frames.push_back(actuator.enable_motor().frame);
  }
}

void AristoProtocol::append_disable_frames(
  std::vector<aristo_actuator::Actuator> & actuators,
  std::vector<TPCANMsg> & direct_frames) const
{
  direct_frames.reserve(direct_frames.size() + actuators.size());
  for (auto & actuator : actuators) {
    direct_frames.push_back(actuator.disable_motor().frame);
  }
}

void AristoProtocol::append_zero_frames(
  std::vector<aristo_actuator::Actuator> & actuators,
  std::vector<TPCANMsg> & direct_frames) const
{
  direct_frames.reserve(direct_frames.size() + actuators.size());
  for (auto & actuator : actuators) {
    direct_frames.push_back(actuator.set_current_position_as_zero().frame);
  }
}

void AristoProtocol::initialize_rx_dispatch(
  const std::vector<aristo_actuator::Actuator> & actuators)
{
  rx_dispatch_table_.clear();
  rx_dispatch_table_.reserve(actuators.size());

  for (std::size_t actuator_index = 0; actuator_index < actuators.size(); ++actuator_index) {
    rx_dispatch_table_.push_back({actuators[actuator_index].get_rx_id(), actuator_index});
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
  std::vector<aristo_actuator::Actuator> & actuators) const
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

  actuators[entry_it->actuator_index].process_rx_frame(frame);
  return true;
}

void AristoProtocol::append_impedance_frames(
  std::vector<aristo_actuator::Actuator> & actuators,
  const std::vector<can_hardware_common::ActuatorTarget> & impedance_targets,
  std::vector<TPCANMsg> & direct_frames) const
{
  direct_frames.reserve(direct_frames.size() + std::min(impedance_targets.size(), actuators.size()));
  for (std::size_t i = 0; i < impedance_targets.size() && i < actuators.size(); ++i) {
    if (const auto command = actuators[i].set_joint_impedance(impedance_targets[i])) {
      direct_frames.push_back(command->frame);
    }
  }
}

}  // namespace aristo_hand
