#ifndef ARISTO_CONTROLLER__STATE_MACHINES__MPPI_GRASP_HPP_
#define ARISTO_CONTROLLER__STATE_MACHINES__MPPI_GRASP_HPP_

#include "plato_robot_system/control/state_machine/state_machine.hpp"

namespace aristo_controller::state_machines
{

class MPPIGraspState final : public plato_robot_system::State
{
public:
  static constexpr const char * kName = "mppi_grasp";

  explicit MPPIGraspState(plato_robot_system::StateId id);

  bool PopulateCommand(plato_robot_system::RobotCommand * command) const override;
};

}  // namespace aristo_controller::state_machines

#endif  // ARISTO_CONTROLLER__STATE_MACHINES__MPPI_GRASP_HPP_
