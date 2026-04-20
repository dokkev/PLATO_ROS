#ifndef ARISTO_HARDWARE_INTERFACE__ARISTO_PROTOCOL_HPP_
#define ARISTO_HARDWARE_INTERFACE__ARISTO_PROTOCOL_HPP_

#include <cstddef>
#include <vector>

#include <PCANBasic.h>

#include "aristo_hardware_interface/actuator.hpp"
#include "can_hardware_common/actuator.hpp"
#include "can_hardware_common/core/lifecycle_plan.hpp"
#include "can_hardware_common/ft_sensor.hpp"

namespace aristo_hand
{

class AristoProtocol
{
public:
  using LifecyclePlan = can_hardware_common::core::LifecyclePlan;
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

  LifecyclePlan build_lifecycle_plan(
    std::vector<aristo_actuator::Actuator> & actuators,
    can_hardware_common::core::LifecycleOperation operation) const;

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
