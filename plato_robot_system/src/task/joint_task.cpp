#include "plato_robot_system/task/joint_task.hpp"

#include <limits>

namespace plato_robot_system::task
{

void JointTask::SetTaskFeedbackGains(
  const Eigen::Ref<const Eigen::VectorXd> & kp_task,
  const Eigen::Ref<const Eigen::VectorXd> & kd_task)
{
  kp_task_ = kp_task;
  kd_task_ = kd_task;
  ConfigurePidControllers();
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
  if (kp_task_.size() != kd_task_.size() ||
    static_cast<Eigen::Index>(pid_controllers_.size()) != state.qdot.size())
  {
    ConfigurePidControllers();
  }

  if (static_cast<Eigen::Index>(pid_controllers_.size()) != state.qdot.size()) {
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
  for (auto & controller : pid_controllers_) {
    controller.Reset();
  }
}

bool JointTask::BuildCommand(
  const RobotState & state,
  const double elapsed_time_sec,
  const double dt_sec,
  RobotCommand * command) const
{
  if (command == nullptr || !trajectory_initialized_ || !HasCompatibleState(state)) {
    return false;
  }
  if (static_cast<Eigen::Index>(pid_controllers_.size()) != state.qdot.size()) {
    return false;
  }

  const auto & sample = trajectory_.EvaluateSample(elapsed_time_sec);
  command_.q_cmd = sample.value;
  command_.qdot_cmd = sample.derivative;
  // RobotCommand.kp/kd are driver-local gains. Host-side task feedback only
  // contributes to tau_cmd here.
  command_.kp.setZero();
  command_.kd.setZero();
  for (Eigen::Index i = 0; i < command_.tau_cmd.size(); ++i) {
    const auto error = command_.q_cmd[i] - state.q[i];
    command_.tau_cmd[i] = pid_controllers_[static_cast<std::size_t>(i)].Compute(error, dt_sec);
  }

  command_.stamp_sec = state.time_s;
  command_.valid = command_.HasValidDimensions() && command_.AllFinite();
  if (!command_.valid) {
    return false;
  }

  *command = command_;
  return true;
}

void JointTask::ConfigurePidControllers()
{
  pid_controllers_.clear();
  if (kp_task_.size() <= 0 || kp_task_.size() != kd_task_.size()) {
    return;
  }

  pid_controllers_.reserve(static_cast<std::size_t>(kp_task_.size()));
  for (Eigen::Index i = 0; i < kp_task_.size(); ++i) {
    pid_controllers_.push_back(
      PIDController(kp_task_[i], 0.0, kd_task_[i], 0.0, std::numeric_limits<double>::max()));
  }
}

bool JointTask::HasCompatibleState(const RobotState & state) const
{
  return IsValid(state) && state.q.size() == state.qdot.size();
}

}  // namespace plato_robot_system::task
