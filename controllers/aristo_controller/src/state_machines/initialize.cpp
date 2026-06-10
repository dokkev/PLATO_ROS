#include "aristo_controller/state_machines/initialize.hpp"

#include <algorithm>
#include <utility>

namespace aristo_controller::state_machines
{

InitializeState::InitializeState(
  const plato_robot_system::StateId id,
  std::string name,
  const plato_robot_system::RobotSystem * robot)
: plato_robot_system::State(id, std::move(name)),
  robot_(robot)
{
}

void InitializeState::SetTargetPosition(const Eigen::Ref<const Eigen::VectorXd> & target_jpos)
{
  target_jpos_ = target_jpos;
}

void InitializeState::SetDuration(const double duration_sec)
{
  duration_sec_ = std::max(duration_sec, 1.0e-3);
}

void InitializeState::SetTaskFeedbackGains(
  const Eigen::Ref<const Eigen::VectorXd> & kp_task,
  const Eigen::Ref<const Eigen::VectorXd> & kd_task)
{
  joint_task_.SetTaskFeedbackGains(kp_task, kd_task);
}

void InitializeState::OnEnter()
{
  joint_task_.Reset();
  if (robot_ == nullptr || !robot_->hasState()) {
    return;
  }

  const auto & state = robot_->state();
  if (target_jpos_.size() != state.q.size()) {
    target_jpos_ = state.q;
  }
  joint_task_.StartMinJerk(state, target_jpos_, duration_sec_);
}

void InitializeState::OnExit()
{
  joint_task_.Reset();
}

bool InitializeState::GetCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }

  return joint_task_.BuildCommand(robot_->state(), elapsed_time(), dt(), command);
}

}  // namespace aristo_controller::state_machines
