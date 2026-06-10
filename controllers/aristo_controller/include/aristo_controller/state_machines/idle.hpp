#ifndef ARISTO_CONTROLLER__STATE_MACHINES__IDLE_HPP_
#define ARISTO_CONTROLLER__STATE_MACHINES__IDLE_HPP_

#include "plato_robot_system/control/state_machine/state_machine.hpp"

namespace aristo_controller::state_machines
{

class IdleState final : public plato_robot_system::State
{
public:
  static constexpr const char * kName = "idle";

  IdleState(plato_robot_system::StateId id, int nq, int nv);

  bool PopulateCommand(plato_robot_system::RobotCommand * command) const override;

private:
  int nq_{0};
  int nv_{0};
};

}  // namespace aristo_controller::state_machines

#endif  // ARISTO_CONTROLLER__STATE_MACHINES__IDLE_HPP_
