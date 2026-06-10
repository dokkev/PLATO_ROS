#include "aristo_controller/state_machines/joint_teleop.hpp"

namespace aristo_controller::state_machines
{

JointTeleopState::JointTeleopState(const plato_robot_system::StateId id)
: plato_robot_system::State(id, kName)
{
}

JointTeleopState::JointTeleopState(
  const plato_robot_system::StateId id,
  const plato_robot_system::RobotSystem * robot)
: plato_robot_system::State(id, kName),
  robot_(robot)
{
}

bool JointTeleopState::ConfigureTask(const JointTeleopStateConfig & config)
{
  if (robot_ == nullptr || !robot_->hasModel()) {
    task_configured_ = false;
    task_entered_ = false;
    return false;
  }

  task_configured_ =
    joint_teleop_task_.Configure(config.joint_task, robot_->nq(), robot_->nv());
  task_entered_ = false;
  return task_configured_;
}

bool JointTeleopState::SetTargetPosition(
  const Eigen::Ref<const Eigen::VectorXd> & target_q)
{
  return task_configured_ && joint_teleop_task_.SetTargetPosition(target_q);
}

void JointTeleopState::OnEnter()
{
  task_entered_ = false;
  if (!task_configured_ || robot_ == nullptr || !robot_->hasState()) {
    return;
  }
  task_entered_ = joint_teleop_task_.OnEnter(robot_->state());
}

void JointTeleopState::OnExit()
{
  joint_teleop_task_.Reset();
  task_entered_ = false;
}

bool JointTeleopState::PopulateCommand(plato_robot_system::RobotCommand * command) const
{
  if (!task_configured_ || robot_ == nullptr || !robot_->hasState() || command == nullptr) {
    return false;
  }
  if (!task_entered_) {
    task_entered_ = joint_teleop_task_.OnEnter(robot_->state());
    if (!task_entered_) {
      return false;
    }
  }

  return joint_teleop_task_.PopulateCommand(robot_->state(), dt(), command);
}

}  // namespace aristo_controller::state_machines
