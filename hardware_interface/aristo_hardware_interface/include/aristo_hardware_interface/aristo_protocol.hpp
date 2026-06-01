#ifndef ARISTO_HARDWARE_INTERFACE__ARISTO_PROTOCOL_HPP_
#define ARISTO_HARDWARE_INTERFACE__ARISTO_PROTOCOL_HPP_

#include <cstddef>
#include <vector>

#include <PCANBasic.h>

#include "aristo_hardware_interface/actuator.hpp"
#include "can_hardware_common/actuator_types.hpp"

namespace aristo_hand
{

class AristoProtocol
{
public:
  struct RxDispatchEntry
  {
    uint32_t rx_id = 0;
    std::size_t actuator_index = 0;
  };

  void append_enable_frames(
    std::vector<aristo_actuator::Actuator> & actuators,
    std::vector<TPCANMsg> & direct_frames) const;

  void append_disable_frames(
    std::vector<aristo_actuator::Actuator> & actuators,
    std::vector<TPCANMsg> & direct_frames) const;

  void append_zero_frames(
    std::vector<aristo_actuator::Actuator> & actuators,
    std::vector<TPCANMsg> & direct_frames) const;

  void initialize_rx_dispatch(const std::vector<aristo_actuator::Actuator> & actuators);

  bool process_rx_frame(
    const TPCANMsg & frame,
    std::vector<aristo_actuator::Actuator> & actuators) const;

  void append_impedance_frames(
    std::vector<aristo_actuator::Actuator> & actuators,
    const std::vector<can_hardware_common::ActuatorTarget> & impedance_targets,
    std::vector<TPCANMsg> & direct_frames) const;

private:
  std::vector<RxDispatchEntry> rx_dispatch_table_;
};

}  // namespace aristo_hand

#endif  // ARISTO_HARDWARE_INTERFACE__ARISTO_PROTOCOL_HPP_
