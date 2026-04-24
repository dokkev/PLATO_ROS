#ifndef ARISTO_HARDWARE_INTERFACE__ARISTO_PROTOCOL_HPP_
#define ARISTO_HARDWARE_INTERFACE__ARISTO_PROTOCOL_HPP_

#include <cstddef>
#include <vector>

#include <PCANBasic.h>

#include "aristo_hardware_interface/actuator.hpp"
#include "can_hardware_common/actuator_types.hpp"
#include "can_hardware_common/ft_sensor.hpp"

namespace aristo_hand
{

class AristoProtocol
{
public:
  enum class DispatchTargetKind
  {
    kActuator,
    kForceSensor,
    kTorqueSensor,
  };

  struct RxDispatchEntry
  {
    uint32_t rx_id = 0;
    DispatchTargetKind target_kind = DispatchTargetKind::kActuator;
    std::size_t target_index = 0;
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

  void initialize_rx_dispatch(
    const std::vector<aristo_actuator::Actuator> & actuators,
    const std::vector<sensor::FTSensor> & ft_sensors);

  bool process_rx_frame(
    const TPCANMsg & frame,
    std::vector<aristo_actuator::Actuator> & actuators,
    std::vector<sensor::FTSensor> & ft_sensors) const;

  void append_impedance_frames(
    std::vector<aristo_actuator::Actuator> & actuators,
    const std::vector<can_hardware_common::ActuatorTarget> & impedance_targets,
    std::vector<TPCANMsg> & direct_frames) const;

private:
  std::vector<RxDispatchEntry> rx_dispatch_table_;
};

}  // namespace aristo_hand

#endif  // ARISTO_HARDWARE_INTERFACE__ARISTO_PROTOCOL_HPP_
