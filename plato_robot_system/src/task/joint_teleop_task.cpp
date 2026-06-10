#include "plato_robot_system/task/joint_teleop_task.hpp"

#include <algorithm>
#include <cmath>

namespace plato_robot_system::task
{
namespace
{

bool IsNonnegativeFinite(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool IsUnitIntervalFinite(const double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

}  // namespace

bool JointTeleopTask::Configure(
  const JointTeleopTaskConfig & config,
  const int nq,
  const int nv)
{
  configured_ = false;
  Reset();

  if (nq <= 0 || nv <= 0 || nq != nv) {
    return false;
  }
  if (
    !IsUnitIntervalFinite(config.lpf_alpha) ||
    config.kp_task.size() != nv ||
    config.kd_task.size() != nv ||
    !config.kp_task.allFinite() ||
    !config.kd_task.allFinite())
  {
    return false;
  }

  config_ = config;
  nq_ = nq;
  nv_ = nv;
  target_q_.resize(0);
  filtered_q_.resize(0);
  has_target_ = false;
  command_.Resize(nq_, nv_);
  configured_ = true;
  return true;
}

void JointTeleopTask::Reset()
{
  filter_initialized_ = false;
  if (command_.HasValidDimensions()) {
    command_.valid = false;
  }
}

bool JointTeleopTask::OnEnter(const RobotState & state)
{
  if (!configured_ || !HasCompatibleState(state)) {
    filter_initialized_ = false;
    return false;
  }

  filtered_q_ = state.q;
  if (!has_target_ || target_q_.size() != nq_) {
    target_q_ = state.q;
    has_target_ = true;
  }
  filter_initialized_ = true;
  command_.Resize(nq_, nv_);
  command_.valid = false;
  return true;
}

bool JointTeleopTask::SetTargetPosition(
  const Eigen::Ref<const Eigen::VectorXd> & target_q)
{
  if (!configured_ || target_q.size() != nq_ || !target_q.allFinite()) {
    return false;
  }

  target_q_ = target_q;
  has_target_ = true;
  return true;
}

bool JointTeleopTask::PopulateCommand(
  const RobotState & state,
  const double dt_sec,
  RobotCommand * command) const
{
  if (
    command == nullptr || !configured_ || !filter_initialized_ ||
    !has_target_ || !HasCompatibleState(state))
  {
    return false;
  }

  const Eigen::VectorXd previous_q_cmd = filtered_q_;
  const double alpha = IsNonnegativeFinite(dt_sec) && dt_sec > 0.0 ? config_.lpf_alpha : 0.0;
  filtered_q_ = previous_q_cmd + alpha * (target_q_ - previous_q_cmd);

  command_.q_cmd = filtered_q_;
  if (std::isfinite(dt_sec) && dt_sec > 0.0) {
    command_.qdot_cmd = (filtered_q_ - previous_q_cmd) / dt_sec;
  } else {
    command_.qdot_cmd.setZero();
  }
  command_.tau_cmd =
    config_.kp_task.cwiseProduct(command_.q_cmd - state.q) +
    config_.kd_task.cwiseProduct(command_.qdot_cmd - state.qdot);

  command_.stamp_sec = state.time_s;
  command_.valid = command_.HasValidDimensions() && command_.AllFinite();
  if (!command_.valid) {
    return false;
  }

  *command = command_;
  return true;
}

bool JointTeleopTask::HasCompatibleState(const RobotState & state) const
{
  return IsValid(state) &&
         state.q.size() == nq_ &&
         state.qdot.size() == nv_ &&
         state.tau.size() == nv_;
}

}  // namespace plato_robot_system::task
