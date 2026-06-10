#include "aristo_controller/state_machines/mppi_grasp.hpp"

namespace aristo_controller::state_machines
{

MPPIGraspState::MPPIGraspState(const plato_robot_system::StateId id)
: plato_robot_system::State(id, kName)
{
}

bool MPPIGraspState::PopulateCommand(plato_robot_system::RobotCommand * command) const
{
  (void)command;
  return false;
}

}  // namespace aristo_controller::state_machines
