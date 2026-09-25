#include "aristo_controller/state_machines/grasp_ready.hpp"

#include <algorithm>
#include <array>

namespace
{

bool GetThumbJointQIndices(
  const plato_robot_system::RobotSystem & robot,
  const Eigen::Index q_size,
  std::array<Eigen::Index, 2> * indices)
{
  if (indices == nullptr) {
    return false;
  }

  if (robot.hasModel()) {
    const auto & model = robot.model();
    const std::array<const char *, 2> joint_names{"joint3", "joint4"};
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      if (!model.existJointName(joint_names[i])) {
        return false;
      }
      const auto & joint = model.joints[model.getJointId(joint_names[i])];
      if (joint.nq() != 1 || joint.idx_q() >= q_size) {
        return false;
      }
      (*indices)[i] = static_cast<Eigen::Index>(joint.idx_q());
    }
    return true;
  }

  // The config joint-position vectors use [joint1, ..., joint8] when there is
  // no model available to resolve the Pinocchio q indices.
  if (q_size < 4) {
    return false;
  }
  *indices = {2, 3};
  return true;
}

}  // namespace

namespace aristo_controller::state_machines
{

GraspReadyState::GraspReadyState(
  const plato_robot_system::StateId id,
  const plato_robot_system::RobotSystem * robot)
: plato_robot_system::State(id, kName),
  robot_(robot)
{
}

void GraspReadyState::SetTargetPosition(const Eigen::Ref<const Eigen::VectorXd> & target_jpos)
{
  target_jpos_ = target_jpos;
}

void GraspReadyState::SetDuration(const double duration_sec)
{
  duration_sec_ = std::max(duration_sec, 1.0e-3);
}

void GraspReadyState::SetTaskFeedbackGains(
  const Eigen::Ref<const Eigen::VectorXd> & kp_task,
  const Eigen::Ref<const Eigen::VectorXd> & kd_task)
{
  joint_task_.SetTaskFeedbackGains(kp_task, kd_task);
  thumb_first_task_.SetTaskFeedbackGains(kp_task, kd_task);
  remaining_task_.SetTaskFeedbackGains(kp_task, kd_task);
}

void GraspReadyState::OnEnter()
{
  joint_task_.Reset();
  thumb_first_task_.Reset();
  remaining_task_.Reset();
  staged_trajectory_ = false;
  if (robot_ == nullptr || !robot_->hasState()) {
    return;
  }

  const auto & state = robot_->state();
  if (target_jpos_.size() != state.q.size()) {
    target_jpos_ = state.q;
  }
  joint_task_.StartMinJerk(state, target_jpos_, duration_sec_);

  std::array<Eigen::Index, 2> thumb_joint_q_indices{};
  if (!GetThumbJointQIndices(*robot_, state.q.size(), &thumb_joint_q_indices)) {
    return;
  }

  // Move the thumb to its grasp-ready position while the other joints hold
  // their current positions. Then finish the full grasp-ready pose.
  Eigen::VectorXd thumb_first_target = state.q;
  for (const auto q_index : thumb_joint_q_indices) {
    thumb_first_target[q_index] = target_jpos_[q_index];
  }

  auto thumb_first_state = state;
  thumb_first_state.q = thumb_first_target;
  thumb_first_state.qdot.setZero();
  thumb_first_state.tau.setZero();

  const double phase_duration_sec = duration_sec_ * 0.5;
  if (thumb_first_task_.StartMinJerk(state, thumb_first_target, phase_duration_sec) &&
    remaining_task_.StartMinJerk(thumb_first_state, target_jpos_, phase_duration_sec))
  {
    staged_trajectory_ = true;
  }
}

void GraspReadyState::OnExit()
{
  joint_task_.Reset();
  thumb_first_task_.Reset();
  remaining_task_.Reset();
  staged_trajectory_ = false;
}

bool GraspReadyState::IsFinished() const
{
  if (lifecycle_.stay_here || !joint_task_.active()) {
    return false;
  }

  const double trajectory_duration_sec = std::max(duration_sec_, lifecycle_.duration);
  return elapsed_time() >= trajectory_duration_sec + lifecycle_.wait_time;
}

bool GraspReadyState::PopulateCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }

  if (staged_trajectory_) {
    const double phase_duration_sec = duration_sec_ * 0.5;
    const double elapsed_time_sec = elapsed_time();
    if (elapsed_time_sec < phase_duration_sec) {
      return thumb_first_task_.PopulateCommand(
        robot_->state(), elapsed_time_sec, dt(), command);
    }
    return remaining_task_.PopulateCommand(
      robot_->state(), elapsed_time_sec - phase_duration_sec, dt(), command);
  }

  return joint_task_.PopulateCommand(robot_->state(), elapsed_time(), dt(), command);
}

}  // namespace aristo_controller::state_machines
