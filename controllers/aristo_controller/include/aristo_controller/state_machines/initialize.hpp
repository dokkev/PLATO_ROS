#ifndef ARISTO_CONTROLLER__STATE_MACHINES__INITIALIZE_HPP_
#define ARISTO_CONTROLLER__STATE_MACHINES__INITIALIZE_HPP_

#include <Eigen/Core>

#include <string>

#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/robot/robot_system.hpp"
#include "plato_robot_system/task/joint_task.hpp"

namespace aristo_controller::state_machines
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
  void SetDuration(double duration_sec);
  void SetTaskFeedbackGains(
    const Eigen::Ref<const Eigen::VectorXd> & kp_task,
    const Eigen::Ref<const Eigen::VectorXd> & kd_task);

  void OnEnter() override;
  void OnExit() override;
  bool PopulateCommand(plato_robot_system::RobotCommand * command) const override;

  const plato_robot_system::RobotCommand & command() const { return joint_task_.command(); }

private:
  const plato_robot_system::RobotSystem * robot_{nullptr};
  double duration_sec_{2.0};
  Eigen::VectorXd target_jpos_;
  mutable plato_robot_system::task::JointTask joint_task_;
};

}  // namespace aristo_controller::state_machines

#endif  // ARISTO_CONTROLLER__STATE_MACHINES__INITIALIZE_HPP_
