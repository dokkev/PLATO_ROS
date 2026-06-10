#include "plato_robot_system/task/grasp_task.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <proxsuite/proxqp/status.hpp>

namespace plato_robot_system::task
{
namespace
{

constexpr double kMinDt = 1.0e-9;
const Eigen::Vector3d kFingerMotionPlaneNormalBase{0.0, 1.0, 0.0};

bool IsFinite(const double value)
{
  return std::isfinite(value);
}

bool IsNonnegativeFinite(const double value)
{
  return IsFinite(value) && value >= 0.0;
}

bool IsValidWeight(const double value)
{
  return IsNonnegativeFinite(value);
}

bool IsValidPositionLimit(const double lower, const double upper)
{
  return IsFinite(lower) && IsFinite(upper) && lower < upper;
}

bool NormalizeVector(
  const Eigen::Vector3d & value,
  const double min_norm,
  Eigen::Vector3d * normalized)
{
  if (normalized == nullptr || !value.allFinite()) {
    return false;
  }
  const double norm = value.norm();
  if (!IsFinite(norm) || norm < min_norm) {
    return false;
  }
  *normalized = value / norm;
  return true;
}

double Clamp(const double value, const double lower, const double upper)
{
  return std::clamp(value, lower, upper);
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

bool IsValidConfig(const GraspTaskConfig & config)
{
  return IsFinite(config.distance_closed_m) &&
         IsFinite(config.distance_open_m) &&
         config.distance_closed_m >= 0.0 &&
         config.distance_open_m > config.distance_closed_m &&
         config.fallback_close_axis_base.allFinite() &&
         config.force_enter_debounce_ticks >= 0 &&
         config.force_exit_contact_lost_ticks >= 0 &&
         IsFinite(config.force_exit_u_threshold) &&
         IsNonnegativeFinite(config.min_contact_force_n) &&
         IsNonnegativeFinite(config.kp_task) &&
         IsNonnegativeFinite(config.kd_task) &&
         IsNonnegativeFinite(config.align_offset_limit_m) &&
         IsNonnegativeFinite(config.kp_align) &&
         IsNonnegativeFinite(config.kd_align) &&
         IsFinite(config.kp_tactile_fb) &&
         IsFinite(config.kd_tactile_fb) &&
         IsValidWeight(config.w_task_motion) &&
         IsValidWeight(config.w_task_tactile_mode) &&
         IsValidWeight(config.w_align) &&
         IsValidWeight(config.w_tactile) &&
         IsValidWeight(config.w_posture) &&
         IsNonnegativeFinite(config.damping_qp) &&
         IsFinite(config.max_qddot_rad_s2) &&
         config.max_qddot_rad_s2 > 0.0 &&
         IsFinite(config.max_velocity_rad_s) &&
         config.max_velocity_rad_s > 0.0 &&
         IsNonnegativeFinite(config.max_torque_nm) &&
         IsNonnegativeFinite(config.max_torque_rate_nm_per_s) &&
         IsFinite(config.min_axis_distance_m) &&
         config.min_axis_distance_m > 0.0 &&
         IsNonnegativeFinite(config.joint_damping_nm_per_rad_s);
}

}  // namespace

bool GraspTask::Configure(
  const pinocchio::Model & model,
  const GraspTaskConfig & config)
{
  configured_ = false;
  Reset();

  if (model.nq <= 0 || model.nv <= 0 || !IsValidConfig(config)) {
    return false;
  }

  Eigen::Vector3d normalized_fallback_close_axis;
  if (!NormalizeVector(
      config.fallback_close_axis_base,
      config.min_axis_distance_m,
      &normalized_fallback_close_axis))
  {
    return false;
  }

  const auto frame_a_id = model.getFrameId(std::string(kThumbIndexFrameA));
  const auto frame_b_id = model.getFrameId(std::string(kThumbIndexFrameB));
  if (frame_a_id >= static_cast<pinocchio::FrameIndex>(model.nframes) ||
    frame_b_id >= static_cast<pinocchio::FrameIndex>(model.nframes))
  {
    return false;
  }

  GraspTaskConfig normalized_config = config;
  normalized_config.fallback_close_axis_base = normalized_fallback_close_axis;
  active_dof_ = static_cast<int>(kThumbIndexActiveJoints.size());
  if (normalized_config.q_posture.size() == 0) {
    normalized_config.q_posture = Eigen::VectorXd::Zero(active_dof_);
  }
  if (normalized_config.q_posture.size() != active_dof_ ||
    !normalized_config.q_posture.allFinite())
  {
    return false;
  }

  for (std::size_t i = 0; i < kThumbIndexActiveJoints.size(); ++i) {
    const auto joint_id = model.getJointId(std::string(kThumbIndexActiveJoints[i]));
    if (joint_id >= static_cast<pinocchio::JointIndex>(model.njoints)) {
      return false;
    }
    if (model.nqs[joint_id] != 1 || model.nvs[joint_id] != 1) {
      return false;
    }
    const int q_index = model.idx_qs[joint_id];
    const int v_index = model.idx_vs[joint_id];
    if (q_index < 0 || q_index >= model.nq || v_index < 0 || v_index >= model.nv) {
      return false;
    }
    active_q_indices_[i] = q_index;
    active_v_indices_[i] = v_index;
  }

  config_ = normalized_config;
  frame_a_id_ = frame_a_id;
  frame_b_id_ = frame_b_id;
  qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(active_dof_, 0, active_dof_);
  last_tau_cmd_ = Eigen::VectorXd::Zero(model.nv);
  configured_ = true;
  Reset();
  return true;
}

void GraspTask::Reset()
{
  has_entry_geometry_ = false;
  qp_initialized_ = false;
  mode_ = GraspTaskMode::kMotionTeleop;
  force_enter_counter_ = 0;
  force_exit_contact_lost_counter_ = 0;
  last_force_error_n_ = 0.0;
  has_last_force_error_ = false;
  has_last_tau_cmd_ = false;
  if (last_tau_cmd_.size() > 0) {
    last_tau_cmd_.setZero();
  }
  status_ = GraspTaskStatus{};
  status_.qddot_active = Eigen::VectorXd::Zero(active_dof_);
  status_.tau_cmd_active = Eigen::VectorXd::Zero(active_dof_);
  if (qp_) {
    qp_->cleanup();
  }
}

bool GraspTask::OnEnter(RobotSystem & robot, const RobotState & state)
{
  if (!HasCompatibleState(robot, state)) {
    return false;
  }

  Reset();
  if (last_tau_cmd_.size() != robot.nv()) {
    last_tau_cmd_ = Eigen::VectorXd::Zero(robot.nv());
  }
  return CaptureEntryGeometry(robot);
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
    !IsFinite(input.u_close) || !IsFinite(input.u_align) ||
    !IsFinite(input.desired_force_n))
  {
    return false;
  }
  if (!has_entry_geometry_ && !CaptureEntryGeometry(robot)) {
    return false;
  }

  const double u_close = Clamp(input.u_close, 0.0, 1.0);
  const double u_align = Clamp(input.u_align, 0.0, 1.0);
  const double desired_force_n = std::max(0.0, input.desired_force_n);

  const GraspTaskGripForceEstimate estimate = EstimateGripForceFromTactile(state);
  UpdateMode(estimate, u_close);
  if (mode_ == GraspTaskMode::kForceTracking && !estimate.valid_force) {
    mode_ = GraspTaskMode::kMotionTeleop;
    force_enter_counter_ = 0;
    force_exit_contact_lost_counter_ = 0;
    last_force_error_n_ = 0.0;
    has_last_force_error_ = false;
  }

  status_.mode = mode_;
  status_.u_close = u_close;
  status_.u_align = u_align;
  status_.desired_force_n = desired_force_n;
  status_.measured_force_n = estimate.measured_force_n;
  status_.force_enter_counter = force_enter_counter_;
  status_.force_exit_contact_lost_counter = force_exit_contact_lost_counter_;
  status_.qp_solved = false;

  GraspTaskCommand clamped_input;
  clamped_input.u_close = u_close;
  clamped_input.u_align = u_align;
  clamped_input.desired_force_n = desired_force_n;

  Eigen::VectorXd qddot_active;
  if (!SolveActiveAcceleration(robot, state, clamped_input, estimate, dt_sec, &qddot_active)) {
    return false;
  }
  status_.qddot_active = qddot_active;
  status_.qp_solved = true;

  return BuildCommand(robot, state, qddot_active, dt_sec, command);
}

bool GraspTask::HasCompatibleState(
  const RobotSystem & robot,
  const RobotState & state) const
{
  return configured_ && robot.hasModel() && robot.hasState() && IsValid(state) &&
         state.q.size() == robot.nq() && state.qdot.size() == robot.nv() &&
         state.tau.size() == robot.nv();
}

bool GraspTask::CaptureEntryGeometry(RobotSystem & robot)
{
  try {
    const Eigen::Vector3d p_a = robot.FramePoseWorld(frame_a_id_).translation();
    const Eigen::Vector3d p_b = robot.FramePoseWorld(frame_b_id_).translation();
    if (!p_a.allFinite() || !p_b.allFinite()) {
      return false;
    }

    if (!NormalizeVector(
        p_b - p_a,
        config_.min_axis_distance_m,
        &close_axis_base_) &&
      !NormalizeVector(
        config_.fallback_close_axis_base,
        config_.min_axis_distance_m,
        &close_axis_base_))
    {
      return false;
    }
    if (!NormalizeVector(
        kFingerMotionPlaneNormalBase.cross(close_axis_base_),
        config_.min_axis_distance_m,
        &align_axis_base_))
    {
      return false;
    }
    has_entry_geometry_ = true;
    return true;
  } catch (const std::exception &) {
    return false;
  }
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
      u < config_.force_exit_u_threshold;
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
    if (u >= config_.force_exit_u_threshold) {
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

bool GraspTask::SolveActiveAcceleration(
  RobotSystem & robot,
  const RobotState & state,
  const GraspTaskCommand & input,
  const GraspTaskGripForceEstimate & estimate,
  const double dt_sec,
  Eigen::VectorXd * qddot_active)
{
  if (qddot_active == nullptr) {
    return false;
  }

  Eigen::Vector3d p_a;
  Eigen::Vector3d p_b;
  Eigen::MatrixXd j_a_full;
  Eigen::MatrixXd j_b_full;
  try {
    p_a = robot.FramePoseWorld(frame_a_id_).translation();
    p_b = robot.FramePoseWorld(frame_b_id_).translation();
    j_a_full = robot.FrameJacobianWorld(frame_a_id_).topRows(3);
    j_b_full = robot.FrameJacobianWorld(frame_b_id_).topRows(3);
  } catch (const std::exception &) {
    return false;
  }
  if (!p_a.allFinite() || !p_b.allFinite() ||
    !j_a_full.allFinite() || !j_b_full.allFinite())
  {
    return false;
  }

  const Eigen::Vector3d r = p_b - p_a;
  const Eigen::MatrixXd j_r = RestrictToActiveVelocityColumns(j_b_full - j_a_full);
  const Eigen::RowVectorXd j_task = close_axis_base_.transpose() * j_r;
  const Eigen::RowVectorXd j_align = align_axis_base_.transpose() * j_r;

  Eigen::VectorXd q_active(active_dof_);
  Eigen::VectorXd qdot_active(active_dof_);
  for (int i = 0; i < active_dof_; ++i) {
    q_active[i] = state.q[active_q_indices_[static_cast<std::size_t>(i)]];
    qdot_active[i] = state.qdot[active_v_indices_[static_cast<std::size_t>(i)]];
  }
  if (!q_active.allFinite() || !qdot_active.allFinite()) {
    return false;
  }

  const double d_des =
    config_.distance_closed_m +
    input.u_close * (config_.distance_open_m - config_.distance_closed_m);
  const double d = close_axis_base_.dot(r);
  const double d_dot = (j_task * qdot_active)(0);
  const double distance_error = d - d_des;
  const double d_ddot_task_des =
    -config_.kp_task * distance_error - config_.kd_task * d_dot;
  status_.distance_error_m = distance_error;

  const double align_des =
    (2.0 * input.u_align - 1.0) * config_.align_offset_limit_m;
  const double align = align_axis_base_.dot(r);
  const double align_dot = (j_align * qdot_active)(0);
  const double align_error = align - align_des;
  const double align_ddot_des =
    -config_.kp_align * align_error - config_.kd_align * align_dot;
  status_.align_error_m = align_error;

  double d_ddot_tactile_des = 0.0;
  double w_tactile_effective = 0.0;
  status_.force_error_n = 0.0;
  if (mode_ == GraspTaskMode::kForceTracking) {
    const double force_error = input.desired_force_n - estimate.measured_force_n;

    const double force_error_dot =
      has_last_force_error_ ? (force_error - last_force_error_n_) / dt_sec : 0.0;
    last_force_error_n_ = force_error;
    has_last_force_error_ = true;
    d_ddot_tactile_des =
      -config_.kp_tactile_fb * force_error -
      config_.kd_tactile_fb * force_error_dot;
    w_tactile_effective = config_.w_tactile;
    status_.force_error_n = force_error;
  }

  const Eigen::VectorXd q_posture_next_error =
    config_.q_posture - q_active - dt_sec * qdot_active;

  const int max_rows = 1 + 1 + 1 + active_dof_;
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(max_rows, active_dof_);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(max_rows);
  int row = 0;

  const double w_task = mode_ == GraspTaskMode::kForceTracking ?
    config_.w_task_tactile_mode :
    config_.w_task_motion;
  if (w_task > 0.0) {
    const double weight = std::sqrt(w_task);
    a.row(row) = weight * j_task;
    b[row] = weight * d_ddot_task_des;
    ++row;
  }

  if (config_.w_align > 0.0) {
    const double weight = std::sqrt(config_.w_align);
    a.row(row) = weight * j_align;
    b[row] = weight * align_ddot_des;
    ++row;
  }

  if (w_tactile_effective > 0.0) {
    const double weight = std::sqrt(w_tactile_effective);
    a.row(row) = weight * j_task;
    b[row] = weight * d_ddot_tactile_des;
    ++row;
  }

  if (config_.w_posture > 0.0) {
    const double weight = std::sqrt(config_.w_posture);
    const double dt2 = dt_sec * dt_sec;
    a.block(row, 0, active_dof_, active_dof_) =
      weight * dt2 * Eigen::MatrixXd::Identity(active_dof_, active_dof_);
    b.segment(row, active_dof_) = weight * q_posture_next_error;
    row += active_dof_;
  }

  const Eigen::MatrixXd a_used = a.topRows(row);
  const Eigen::VectorXd b_used = b.head(row);
  if (!a_used.allFinite() || !b_used.allFinite()) {
    return false;
  }

  Eigen::MatrixXd h = a_used.transpose() * a_used;
  h.diagonal().array() += std::max(config_.damping_qp, 1.0e-12);
  const Eigen::VectorXd g = -a_used.transpose() * b_used;
  if (!h.allFinite() || !g.allFinite()) {
    return false;
  }

  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  if (!BuildAccelerationBounds(robot.model(), state, dt_sec, &lower, &upper)) {
    return false;
  }

  const Eigen::MatrixXd a_eq(0, active_dof_);
  const Eigen::VectorXd b_eq(0);
  const Eigen::MatrixXd c = Eigen::MatrixXd::Identity(active_dof_, active_dof_);

  try {
    if (!qp_) {
      qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
        active_dof_, 0, active_dof_);
      qp_initialized_ = false;
    }
    if (qp_initialized_) {
      qp_->update(h, g, a_eq, b_eq, c, lower, upper);
    } else {
      qp_->init(h, g, a_eq, b_eq, c, lower, upper);
      qp_initialized_ = true;
    }
    qp_->solve();
  } catch (const std::exception &) {
    qp_initialized_ = false;
    return false;
  }

  if (qp_->results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
    return false;
  }
  if (qp_->results.x.size() != active_dof_ || !qp_->results.x.allFinite()) {
    return false;
  }

  qddot_active->resize(active_dof_);
  for (int i = 0; i < active_dof_; ++i) {
    (*qddot_active)[i] = Clamp(qp_->results.x[i], lower[i], upper[i]);
  }
  return qddot_active->allFinite();
}

Eigen::MatrixXd GraspTask::RestrictToActiveVelocityColumns(
  const Eigen::MatrixXd & jacobian) const
{
  Eigen::MatrixXd active(jacobian.rows(), active_dof_);
  for (int i = 0; i < active_dof_; ++i) {
    active.col(i) = jacobian.col(active_v_indices_[static_cast<std::size_t>(i)]);
  }
  return active;
}

bool GraspTask::BuildAccelerationBounds(
  const pinocchio::Model & model,
  const RobotState & state,
  const double dt_sec,
  Eigen::VectorXd * lower,
  Eigen::VectorXd * upper) const
{
  if (lower == nullptr || upper == nullptr || !IsFinite(dt_sec) || dt_sec <= kMinDt) {
    return false;
  }

  lower->resize(active_dof_);
  upper->resize(active_dof_);
  const double dt2 = dt_sec * dt_sec;

  for (int i = 0; i < active_dof_; ++i) {
    const int q_index = active_q_indices_[static_cast<std::size_t>(i)];
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    const double q_i = state.q[q_index];
    const double qdot_i = state.qdot[v_index];

    double lower_i = -config_.max_qddot_rad_s2;
    double upper_i = config_.max_qddot_rad_s2;

    lower_i = std::max(lower_i, (-config_.max_velocity_rad_s - qdot_i) / dt_sec);
    upper_i = std::min(upper_i, (config_.max_velocity_rad_s - qdot_i) / dt_sec);

    const double model_lower = model.lowerPositionLimit[q_index];
    const double model_upper = model.upperPositionLimit[q_index];
    if (IsValidPositionLimit(model_lower, model_upper)) {
      if (q_i < model_lower) {
        lower_i = std::max(lower_i, 0.0);
      } else if (q_i > model_upper) {
        upper_i = std::min(upper_i, 0.0);
      } else {
        lower_i = std::max(lower_i, (model_lower - q_i - dt_sec * qdot_i) / dt2);
        upper_i = std::min(upper_i, (model_upper - q_i - dt_sec * qdot_i) / dt2);
      }
    }

    if (!IsFinite(lower_i) || !IsFinite(upper_i) || lower_i > upper_i) {
      return false;
    }
    (*lower)[i] = lower_i;
    (*upper)[i] = upper_i;
  }
  return true;
}

bool GraspTask::BuildCommand(
  RobotSystem & robot,
  const RobotState & state,
  const Eigen::VectorXd & qddot_active,
  const double dt_sec,
  RobotCommand * command)
{
  if (command == nullptr || qddot_active.size() != active_dof_ ||
    !qddot_active.allFinite())
  {
    return false;
  }

  Eigen::VectorXd qddot_full = Eigen::VectorXd::Zero(robot.nv());
  for (int i = 0; i < active_dof_; ++i) {
    qddot_full[active_v_indices_[static_cast<std::size_t>(i)]] = qddot_active[i];
  }

  command->Resize(robot.nq(), robot.nv());
  command->q_cmd = state.q;
  command->qdot_cmd.setZero();
  command->tau_cmd.setZero();

  const auto & model = robot.model();
  for (int i = 0; i < active_dof_; ++i) {
    const int q_index = active_q_indices_[static_cast<std::size_t>(i)];
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    const double qdot_next = state.qdot[v_index] + dt_sec * qddot_active[i];
    command->qdot_cmd[v_index] =
      Clamp(qdot_next, -config_.max_velocity_rad_s, config_.max_velocity_rad_s);
    command->q_cmd[q_index] = state.q[q_index] + dt_sec * command->qdot_cmd[v_index];

    const double model_lower = model.lowerPositionLimit[q_index];
    const double model_upper = model.upperPositionLimit[q_index];
    if (IsValidPositionLimit(model_lower, model_upper)) {
      command->q_cmd[q_index] = Clamp(command->q_cmd[q_index], model_lower, model_upper);
    }
  }

  Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(robot.nv());
  if (config_.use_inverse_dynamics) {
    try {
      tau_full = robot.InverseDynamics(state.q, state.qdot, qddot_full);
    } catch (const std::exception &) {
      return false;
    }
    if (tau_full.size() != robot.nv() || !tau_full.allFinite()) {
      return false;
    }
  }

  status_.tau_cmd_active = Eigen::VectorXd::Zero(active_dof_);
  for (int i = 0; i < active_dof_; ++i) {
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    double tau_i = config_.use_inverse_dynamics ? tau_full[v_index] : 0.0;
    tau_i += -config_.joint_damping_nm_per_rad_s * state.qdot[v_index];
    command->tau_cmd[v_index] = tau_i;
  }
  ClampAndRateLimitTorque(dt_sec, command);

  for (int i = 0; i < active_dof_; ++i) {
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    status_.tau_cmd_active[i] = command->tau_cmd[v_index];
  }

  last_tau_cmd_ = command->tau_cmd;
  has_last_tau_cmd_ = true;
  command->stamp_sec = state.time_s;
  command->valid = command->HasValidDimensions() && command->AllFinite();
  return command->valid;
}

void GraspTask::ClampAndRateLimitTorque(
  const double dt_sec,
  RobotCommand * command)
{
  if (command == nullptr) {
    return;
  }

  const double max_delta_tau = config_.max_torque_rate_nm_per_s * dt_sec;
  for (int i = 0; i < active_dof_; ++i) {
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    double tau_i =
      Clamp(command->tau_cmd[v_index], -config_.max_torque_nm, config_.max_torque_nm);
    if (
      has_last_tau_cmd_ && last_tau_cmd_.size() == command->tau_cmd.size() &&
      IsFinite(max_delta_tau))
    {
      tau_i = Clamp(
        tau_i,
        last_tau_cmd_[v_index] - max_delta_tau,
        last_tau_cmd_[v_index] + max_delta_tau);
    }
    command->tau_cmd[v_index] = tau_i;
  }
}

}  // namespace plato_robot_system::task
