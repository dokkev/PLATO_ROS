#include "plato_robot_system/task/joint_task.hpp"

namespace plato_robot_system::task
{

void JointTask::SetTaskFeedbackGains(
  const Eigen::Ref<const Eigen::VectorXd> & kp_task,
  const Eigen::Ref<const Eigen::VectorXd> & kd_task)
{
  kp_task_ = kp_task;
  kd_task_ = kd_task;
}

bool JointTask::StartMinJerk(
  const RobotState & state,
  const Eigen::Ref<const Eigen::VectorXd> & target_jpos,
  const double duration_sec)
{
  trajectory_initialized_ = false;
  if (!HasCompatibleState(state) || target_jpos.size() != state.q.size()) {
    return false;
  }

  if (kp_task_.size() != state.qdot.size()) {
    kp_task_ = Eigen::VectorXd::Zero(state.qdot.size());
  }
  if (kd_task_.size() != state.qdot.size()) {
    kd_task_ = Eigen::VectorXd::Zero(state.qdot.size());
  }
  if (kp_task_.size() != state.qdot.size() || kd_task_.size() != state.qdot.size()) {
    return false;
  }

  command_.Resize(static_cast<int>(state.q.size()), static_cast<int>(state.qdot.size()));
  const auto zeros = Eigen::VectorXd::Zero(state.q.size());
  trajectory_.Initialize(
    state.q,
    state.qdot,
    zeros,
    target_jpos,
    zeros,
    zeros,
    duration_sec);
  trajectory_initialized_ = true;
  Reset();
  return true;
}

void JointTask::Reset()
{
}

bool JointTask::PopulateCommand(
  const RobotState & state,
  const double elapsed_time_sec,
  const double dt_sec,
  RobotCommand * command) const
{
  (void)dt_sec;
  if (command == nullptr || !trajectory_initialized_ || !HasCompatibleState(state)) {
    return false;
  }
  if (kp_task_.size() != state.qdot.size() || kd_task_.size() != state.qdot.size()) {
    return false;
  }

  const auto & sample = trajectory_.EvaluateSample(elapsed_time_sec);
  if (sample.value.size() != state.q.size() || sample.derivative.size() != state.qdot.size()) {
    return false;
  }
  command_.q_cmd = sample.value;
  command_.qdot_cmd = sample.derivative;
  command_.tau_cmd =
    kp_task_.cwiseProduct(command_.q_cmd - state.q) +
    kd_task_.cwiseProduct(command_.qdot_cmd - state.qdot);

  command_.stamp_sec = state.time_s;
  command_.valid = command_.HasValidDimensions() && command_.AllFinite();
  if (!command_.valid) {
    return false;
  }

  *command = command_;
  return true;
}

bool JointTask::HasCompatibleState(const RobotState & state) const
{
  return IsValid(state) && state.q.size() == state.qdot.size();
}

}  // namespace plato_robot_system::task
