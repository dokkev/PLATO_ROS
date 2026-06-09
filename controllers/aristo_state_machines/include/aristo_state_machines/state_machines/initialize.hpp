#ifndef ARISTO_STATE_MACHINES__STATE_MACHINES__INITIALIZE_HPP_
#define ARISTO_STATE_MACHINES__STATE_MACHINES__INITIALIZE_HPP_

#include <Eigen/Core>

#include <string>

#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/robot/robot_system.hpp"

namespace aristo_state_machines::state_machines
{

class InitializeState final : public plato_robot_system::State
{
public:
  static constexpr const char * kName = "initialize";

  InitializeState(
    plato_robot_system::StateId id,
    std::string name,
    const plato_robot_system::RobotSystem * robot);

  void SetTargetPosition(const Eigen::Ref<const Eigen::VectorXd> & target_jpos);
  void SetFeedbackGains(
    const Eigen::Ref<const Eigen::VectorXd> & kp,
    const Eigen::Ref<const Eigen::VectorXd> & kd);

  bool GetCommand(plato_robot_system::RobotCommand * command) const override;

  const plato_robot_system::RobotCommand & command() const { return command_; }

private:
  bool ResizeCommandFromRobotState() const;

  const plato_robot_system::RobotSystem * robot_{nullptr};
  Eigen::VectorXd target_jpos_;
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  mutable plato_robot_system::RobotCommand command_;
};

}  // namespace aristo_state_machines::state_machines

#endif  // ARISTO_STATE_MACHINES__STATE_MACHINES__INITIALIZE_HPP_
