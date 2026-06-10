#include "aristo_controller/state_machines/idle.hpp"

namespace aristo_controller::state_machines
{

IdleState::IdleState(
  const plato_robot_system::StateId id,
  const int nq,
  const int nv)
: plato_robot_system::State(id, kName),
  nq_(nq),
  nv_(nv)
{
}

bool IdleState::PopulateCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || nq_ <= 0 || nv_ <= 0) {
    return false;
  }

  command->Resize(nq_, nv_);
  command->q_cmd.setZero();
  command->qdot_cmd.setZero();
  command->tau_cmd.setZero();
  command->kp.setZero();
  command->kd.setZero();
  command->stamp_sec = 0.0;
  command->valid = command->HasValidDimensions() && command->AllFinite();
  return command->valid;
}

}  // namespace aristo_controller::state_machines
