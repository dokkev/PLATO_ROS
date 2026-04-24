#ifndef PLATO_HARDWARE_INTERFACE__PLATO_PROTOCOL_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO_PROTOCOL_HPP_

#include <cstddef>
#include <vector>

#include <PCANBasic.h>

#include "can_hardware_common/robot.hpp"
#include "plato_hardware_interface/actuator.hpp"

namespace plato_hand
{

class PlatoProtocol
{
public:
  using ActuatorCommandView = can_hardware_common::RobotIO::ActuatorCommand::ConstView;

  void append_enable_frames(
    std::vector<plato_actuator::Actuator> & actuators,
    std::vector<TPCANMsg> & direct_frames) const;

  void append_disable_frames(
    std::vector<plato_actuator::Actuator> & actuators,
    std::vector<TPCANMsg> & direct_frames) const;

  bool process_rx_frame(
    const TPCANMsg & frame,
    std::vector<plato_actuator::Actuator> & actuators) const;

  void append_write_frames(
    std::vector<plato_actuator::Actuator> & actuators,
    const ActuatorCommandView & actuator_cmd,
    std::size_t write_cycle_count,
    double servo_stiffness_scale,
    std::vector<TPCANMsg> & direct_frames) const;
};

}  // namespace plato_hand

#endif  // PLATO_HARDWARE_INTERFACE__PLATO_PROTOCOL_HPP_
