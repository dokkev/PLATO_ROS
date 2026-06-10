#ifndef ARISTO_CONTROLLER__STATE_MACHINES__JOINT_TELEOP_HPP_
#define ARISTO_CONTROLLER__STATE_MACHINES__JOINT_TELEOP_HPP_

#include <Eigen/Core>

#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/task/joint_teleop_task.hpp"

namespace aristo_controller::state_machines
{

struct JointTeleopStateConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  plato_robot_system::task::JointTeleopTaskConfig joint_task;
};

class JointTeleopState final : public plato_robot_system::State
{
public:
  static constexpr const char * kName = "joint_teleop";

  explicit JointTeleopState(plato_robot_system::StateId id);
  JointTeleopState(
    plato_robot_system::StateId id,
    const plato_robot_system::RobotSystem * robot);

  bool ConfigureTask(const JointTeleopStateConfig & config);
  bool SetTargetPosition(const Eigen::Ref<const Eigen::VectorXd> & target_q);

  void OnEnter() override;
  void OnExit() override;
  bool PopulateCommand(plato_robot_system::RobotCommand * command) const override;

  const plato_robot_system::RobotCommand & command() const
  {
    return joint_teleop_task_.command();
  }

  bool task_entered() const { return task_entered_; }

private:
  const plato_robot_system::RobotSystem * robot_{nullptr};
  mutable plato_robot_system::task::JointTeleopTask joint_teleop_task_;
  bool task_configured_{false};
  mutable bool task_entered_{false};
};

}  // namespace aristo_controller::state_machines

#endif  // ARISTO_CONTROLLER__STATE_MACHINES__JOINT_TELEOP_HPP_
