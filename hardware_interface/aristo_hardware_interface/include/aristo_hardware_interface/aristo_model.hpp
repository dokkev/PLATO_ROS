#ifndef ARISTO_HARDWARE_INTERFACE__ARISTO_MODEL_HPP_
#define ARISTO_HARDWARE_INTERFACE__ARISTO_MODEL_HPP_

#include <vector>

#include "aristo_hardware_interface/actuator.hpp"
#include "can_hardware_common/actuator.hpp"
#include "can_hardware_common/core/state_snapshot.hpp"
#include "can_hardware_common/robot.hpp"

namespace aristo_hand
{

class AristoModel
{
public:
  void update_joint_states(
    const std::vector<aristo_actuator::Actuator> & actuators,
    can_hardware_common::RobotIO::ActuatorState & actuator_states,
    can_hardware_common::RobotIO::JointState & joint_states) const;

  void build_impedance_targets(
    const std::vector<aristo_actuator::Actuator> & actuators,
    const can_hardware_common::RobotIO::JointCommand & joint_commands,
    can_hardware_common::RobotIO::ActuatorCommand & actuator_commands,
    std::vector<can_hardware_common::ActuatorTarget> & impedance_targets) const;

  bool actuators_ready(const std::vector<aristo_actuator::Actuator> & actuators) const;

  void copy_feedback_snapshot(
    const std::vector<aristo_actuator::Actuator> & actuators,
    can_hardware_common::core::StateSnapshot & snapshot) const;
};

}  // namespace aristo_hand

#endif  // ARISTO_HARDWARE_INTERFACE__ARISTO_MODEL_HPP_
