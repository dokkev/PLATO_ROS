#include "plato_robot_system/task/grasp_task.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace plato_robot_system::task
{
namespace
{

constexpr double kMinDt = 1.0e-9;

bool IsFinite(const double value)
{
  return std::isfinite(value);
}

bool IsNonnegativeFinite(const double value)
{
  return IsFinite(value) && value >= 0.0;
}

bool IsUnitIntervalFinite(const double value)
{
  return IsFinite(value) && value >= 0.0 && value <= 1.0;
}

double Clamp(const double value, const double lower, const double upper)
{
  return std::clamp(value, lower, upper);
}

double Clamp01(const double value)
{
  return Clamp(value, 0.0, 1.0);
}

int NonnegativeTicks(const int ticks)
{
  return std::max(0, ticks);
}

bool ExtractNormalForceN(
  const sensor::TactileState * tactile,
  double * normal_force_n)
{
  if (normal_force_n == nullptr || tactile == nullptr || !tactile->valid) {
    return false;
  }

  double force_n = tactile->ActiveHemisphereNormalForceN();
  if (!IsFinite(force_n) || force_n <= 0.0) {
    if (!tactile->total_force_n.allFinite()) {
      return false;
    }
    force_n = tactile->total_force_n.z();
  }
  if (!IsFinite(force_n)) {
    return false;
  }

  *normal_force_n = std::max(0.0, force_n);
  return true;
}

bool HasValidParallelGeometry(const GraspTaskConfig & config)
{
  return IsFinite(config.parallel_tip_radius_m) &&
         config.parallel_tip_radius_m > 0.0 &&
         IsFinite(config.parallel_lateral_offset_m) &&
         std::abs(config.parallel_lateral_offset_m) <= 2.0 * config.parallel_tip_radius_m &&
         IsFinite(config.parallel_qmin_rad) &&
         IsFinite(config.parallel_qmax_rad) &&
         config.parallel_qmin_rad < config.parallel_qmax_rad &&
         IsFinite(config.parallel_q5_min_rad) &&
         config.parallel_q5_min_rad >= 0.0 &&
         IsFinite(config.parallel_midpoint_u) &&
         config.parallel_midpoint_u > 0.0 &&
         config.parallel_midpoint_u < 1.0 &&
         IsNonnegativeFinite(config.parallel_max_flexion_rad);
}

bool IsValidConfig(const GraspTaskConfig & config)
{
  return (config.q_ready.size() == 0 || config.q_ready.allFinite()) &&
         config.force_enter_debounce_ticks >= 0 &&
         config.force_exit_contact_lost_ticks >= 0 &&
         IsFinite(config.force_exit_u_threshold) &&
         IsNonnegativeFinite(config.min_contact_force_n) &&
         IsUnitIntervalFinite(config.lpf_alpha) &&
         IsFinite(config.kp_tactile_u_fb) &&
         IsFinite(config.kd_tactile_u_fb) &&
         IsFinite(config.kp_tactile_phi_fb) &&
         IsFinite(config.kd_tactile_phi_fb) &&
         HasValidParallelGeometry(config);
}

}  // namespace

bool GraspTask::Configure(
  const pinocchio::Model & model,
  const GraspTaskConfig & config)
{
  configured_ = false;
  has_reference_posture_ = false;
  q_reference_.resize(0);
  q_target_lpf_.resize(0);
  has_q_target_lpf_ = false;
  Reset();

  if (model.nq <= 0 || model.nv <= 0 || !IsValidConfig(config)) {
    return false;
  }
  if (
    config.q_ready.size() > 0 &&
    (config.q_ready.size() != model.nq || !config.q_ready.allFinite()))
  {
    return false;
  }

  active_dof_ = static_cast<int>(kThumbIndexActiveJoints.size());
  for (std::size_t i = 0; i < kThumbIndexActiveJoints.size(); ++i) {
    const auto joint_id = model.getJointId(std::string(kThumbIndexActiveJoints[i]));
    if (joint_id >= static_cast<pinocchio::JointIndex>(model.njoints)) {
      return false;
    }
    if (model.nqs[joint_id] != 1 || model.nvs[joint_id] != 1) {
      return false;
    }
    const int q_index = model.idx_qs[joint_id];
    if (q_index < 0 || q_index >= model.nq) {
      return false;
    }
    active_q_indices_[i] = q_index;
  }

  config_ = config;
  configured_ = true;
  Reset();
  return true;
}

void GraspTask::Reset()
{
  mode_ = GraspTaskMode::kMotionTeleop;
  force_enter_counter_ = 0;
  force_exit_contact_lost_counter_ = 0;
  last_force_error_n_ = 0.0;
  has_last_force_error_ = false;
  status_ = GraspTaskStatus{};
  status_.q_target = Eigen::VectorXd::Zero(0);
  q_target_lpf_.resize(0);
  has_q_target_lpf_ = false;
}

bool GraspTask::OnEnter(RobotSystem & robot, const RobotState & state)
{
  if (!HasCompatibleState(robot, state)) {
    return false;
  }

  Reset();
  return CaptureReferencePosture(state);
}

bool GraspTask::PopulateCommand(
  RobotSystem & robot,
  const RobotState & state,
  const GraspTaskCommand & input,
  const double dt_sec,
  RobotCommand * command)
{
  if (command == nullptr || !HasCompatibleState(robot, state) ||
    !IsFinite(dt_sec) || dt_sec <= kMinDt ||
    !IsFinite(input.u) || !IsFinite(input.phi) ||
    !IsFinite(input.desired_force_n))
  {
    return false;
  }
  if (!has_reference_posture_ && !CaptureReferencePosture(state)) {
    return false;
  }

  const double u = Clamp01(input.u);
  const double phi = Clamp01(input.phi);
  const double desired_force_n = std::max(0.0, input.desired_force_n);

  const GraspTaskGripForceEstimate estimate = EstimateGripForceFromTactile(state);
  UpdateMode(estimate, u);
  if (mode_ == GraspTaskMode::kForceTracking && !estimate.valid_force) {
    mode_ = GraspTaskMode::kMotionTeleop;
    force_enter_counter_ = 0;
    force_exit_contact_lost_counter_ = 0;
    last_force_error_n_ = 0.0;
    has_last_force_error_ = false;
  }

  status_.mode = mode_;
  status_.u = u;
  status_.phi = phi;
  status_.desired_force_n = desired_force_n;
  status_.measured_force_n = estimate.measured_force_n;
  status_.force_enter_counter = force_enter_counter_;
  status_.force_exit_contact_lost_counter = force_exit_contact_lost_counter_;

  GraspTaskCommand clamped_input;
  clamped_input.u = u;
  clamped_input.phi = phi;
  clamped_input.desired_force_n = desired_force_n;

  return BuildParallelJointPositionCommand(
    robot,
    state,
    clamped_input,
    estimate,
    dt_sec,
    command);
}

bool GraspTask::HasCompatibleState(
  const RobotSystem & robot,
  const RobotState & state) const
{
  return configured_ && robot.hasModel() && robot.hasState() && IsValid(state) &&
         state.q.size() == robot.nq() && state.qdot.size() == robot.nv() &&
         state.tau.size() == robot.nv();
}

bool GraspTask::CaptureReferencePosture(const RobotState & state)
{
  if (config_.q_ready.size() == state.q.size()) {
    q_reference_ = config_.q_ready;
  } else {
    q_reference_ = state.q;
  }

  has_reference_posture_ =
    q_reference_.size() == state.q.size() && q_reference_.allFinite();
  return has_reference_posture_;
}

GraspTaskGripForceEstimate GraspTask::EstimateGripForceFromTactile(
  const RobotState & state) const
{
  GraspTaskGripForceEstimate estimate;
  const sensor::TactileState * tactile_a = nullptr;
  const sensor::TactileState * tactile_b = nullptr;
  const std::string frame_a_name(kThumbIndexFrameA);
  const std::string frame_b_name(kThumbIndexFrameB);
  for (const auto & tactile : state.tactile_sensors) {
    if (tactile.frame_name == frame_a_name) {
      tactile_a = &tactile;
    } else if (tactile.frame_name == frame_b_name) {
      tactile_b = &tactile;
    }
  }

  const bool has_force_a = ExtractNormalForceN(tactile_a, &estimate.force_a_n);
  const bool has_force_b = ExtractNormalForceN(tactile_b, &estimate.force_b_n);

  if (config_.use_tactile_presence_for_contact) {
    estimate.enough_contact_a =
      tactile_a != nullptr && tactile_a->valid && tactile_a->HasEnoughContact();
    estimate.enough_contact_b =
      tactile_b != nullptr && tactile_b->valid && tactile_b->HasEnoughContact();
    estimate.lost_contact_a =
      tactile_a == nullptr || !tactile_a->valid ||
      tactile_a->contact_state == sensor::TactileState::kNoContact;
    estimate.lost_contact_b =
      tactile_b == nullptr || !tactile_b->valid ||
      tactile_b->contact_state == sensor::TactileState::kNoContact;
  } else {
    estimate.enough_contact_a =
      has_force_a && estimate.force_a_n >= config_.min_contact_force_n;
    estimate.enough_contact_b =
      has_force_b && estimate.force_b_n >= config_.min_contact_force_n;
    estimate.lost_contact_a =
      !has_force_a || estimate.force_a_n < config_.min_contact_force_n;
    estimate.lost_contact_b =
      !has_force_b || estimate.force_b_n < config_.min_contact_force_n;
  }

  estimate.valid_force = has_force_a && has_force_b;
  if (!estimate.valid_force) {
    return estimate;
  }

  switch (config_.force_aggregation) {
    case GraspTaskConfig::ForceAggregation::kMin:
      estimate.measured_force_n = std::min(estimate.force_a_n, estimate.force_b_n);
      break;
    case GraspTaskConfig::ForceAggregation::kAverage:
      estimate.measured_force_n = 0.5 * (estimate.force_a_n + estimate.force_b_n);
      break;
  }
  estimate.valid_force = IsFinite(estimate.measured_force_n);
  return estimate;
}

void GraspTask::UpdateMode(
  const GraspTaskGripForceEstimate & estimate,
  const double u)
{
  if (mode_ == GraspTaskMode::kMotionTeleop) {
    const bool should_enter_force_tracking =
      estimate.enough_contact_a && estimate.enough_contact_b &&
      u <= config_.force_exit_u_threshold;
    if (should_enter_force_tracking) {
      ++force_enter_counter_;
    } else {
      force_enter_counter_ = 0;
    }

    if (should_enter_force_tracking &&
      force_enter_counter_ >= std::max(1, NonnegativeTicks(config_.force_enter_debounce_ticks)))
    {
      mode_ = GraspTaskMode::kForceTracking;
      last_force_error_n_ = 0.0;
      has_last_force_error_ = false;
      force_exit_contact_lost_counter_ = 0;
    }
    return;
  }

  if (mode_ == GraspTaskMode::kForceTracking) {
    if (u > config_.force_exit_u_threshold) {
      mode_ = GraspTaskMode::kMotionTeleop;
      force_enter_counter_ = 0;
      force_exit_contact_lost_counter_ = 0;
      last_force_error_n_ = 0.0;
      has_last_force_error_ = false;
      return;
    }

    const bool should_exit_for_contact_loss =
      estimate.lost_contact_a && estimate.lost_contact_b;
    if (should_exit_for_contact_loss) {
      ++force_exit_contact_lost_counter_;
    } else {
      force_exit_contact_lost_counter_ = 0;
    }

    if (should_exit_for_contact_loss &&
      force_exit_contact_lost_counter_ >=
      std::max(1, NonnegativeTicks(config_.force_exit_contact_lost_ticks)))
    {
      mode_ = GraspTaskMode::kMotionTeleop;
      force_enter_counter_ = 0;
      force_exit_contact_lost_counter_ = 0;
      last_force_error_n_ = 0.0;
      has_last_force_error_ = false;
    }
  }
}

double GraspTask::ParallelQ5Geometry(const double q3) const
{
  const double cos_q5 = Clamp(
    std::cos(q3) - config_.parallel_lateral_offset_m / config_.parallel_tip_radius_m,
    -1.0,
    1.0);
  return std::max(config_.parallel_q5_min_rad, std::acos(cos_q5));
}

bool GraspTask::BuildParallelJointPositionCommand(
  const RobotSystem & robot,
  const RobotState & state,
  const GraspTaskCommand & input,
  const GraspTaskGripForceEstimate & estimate,
  const double dt_sec,
  RobotCommand * command)
{
  if (command == nullptr || !HasCompatibleState(robot, state) ||
    !IsFinite(dt_sec) || dt_sec <= kMinDt ||
    q_reference_.size() != robot.nq() || !q_reference_.allFinite())
  {
    return false;
  }

  double effective_u = input.u;
  double effective_phi = input.phi;
  status_.force_error_n = 0.0;
  if (mode_ == GraspTaskMode::kForceTracking && estimate.valid_force) {
    const double force_error = input.desired_force_n - estimate.measured_force_n;
    const double force_error_dot =
      has_last_force_error_ ? (force_error - last_force_error_n_) / dt_sec : 0.0;
    last_force_error_n_ = force_error;
    has_last_force_error_ = true;
    if (config_.force_feedback_enabled) {
      effective_u = Clamp01(
        effective_u -
        config_.kp_tactile_u_fb * force_error -
        config_.kd_tactile_u_fb * force_error_dot);
      effective_phi = Clamp01(
        effective_phi +
        config_.kp_tactile_phi_fb * force_error +
        config_.kd_tactile_phi_fb * force_error_dot);
    }
    status_.force_error_n = force_error;
  }

  const double u_parallel = Clamp01(effective_u);

  double q3_target = 0.0;
  double q5_target = 0.0;
  if (u_parallel > config_.parallel_midpoint_u) {
    const double ratio =
      (u_parallel - config_.parallel_midpoint_u) /
      (1.0 - config_.parallel_midpoint_u);
    const double q5_neutral = ParallelQ5Geometry(0.0);
    q3_target = 0.0;
    q5_target = std::max(
      config_.parallel_q5_min_rad,
      q5_neutral - ratio * (q5_neutral - config_.parallel_q5_min_rad));
  } else {
    const double ratio =
      (config_.parallel_midpoint_u - u_parallel) / config_.parallel_midpoint_u;
    q3_target = Clamp(
      config_.parallel_qmax_rad -
      ratio * (config_.parallel_qmax_rad - config_.parallel_qmin_rad),
      config_.parallel_qmin_rad,
      config_.parallel_qmax_rad);
    q5_target = ParallelQ5Geometry(q3_target);
  }

  const double phi = Clamp01(effective_phi);
  const double q4_target = -q3_target - phi * config_.parallel_max_flexion_rad;
  const double q6_target = -q5_target + phi * config_.parallel_max_flexion_rad;

  Eigen::VectorXd q_target = q_reference_;
  q_target[active_q_indices_[2]] = q3_target;
  q_target[active_q_indices_[3]] = q4_target;
  q_target[active_q_indices_[0]] = q5_target;
  q_target[active_q_indices_[1]] = q6_target;
  if (!q_target.allFinite()) {
    return false;
  }

  Eigen::VectorXd q_command = q_target;
  if (config_.lpf_alpha < 1.0) {
    if (
      !has_q_target_lpf_ ||
      q_target_lpf_.size() != q_target.size() ||
      !q_target_lpf_.allFinite())
    {
      q_target_lpf_ = state.q;
      has_q_target_lpf_ =
        q_target_lpf_.size() == q_target.size() && q_target_lpf_.allFinite();
    }
    if (!has_q_target_lpf_) {
      return false;
    }
    q_target_lpf_ =
      (1.0 - config_.lpf_alpha) * q_target_lpf_ +
      config_.lpf_alpha * q_target;
    q_command = q_target_lpf_;
  } else {
    q_target_lpf_ = q_target;
    has_q_target_lpf_ = true;
  }
  if (!q_command.allFinite()) {
    return false;
  }

  command->Resize(robot.nq(), robot.nv());
  command->q_cmd = q_command;
  command->qdot_cmd.setZero();
  command->tau_cmd.setZero();
  command->stamp_sec = state.time_s;
  command->valid = command->HasValidDimensions() && command->AllFinite();
  if (!command->valid) {
    return false;
  }

  status_.effective_u = effective_u;
  status_.effective_phi = effective_phi;
  status_.u_parallel = u_parallel;
  status_.q_target = q_command;
  return true;
}

}  // namespace plato_robot_system::task
