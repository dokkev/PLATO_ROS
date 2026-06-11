#include "plato_robot_system/task/grasp_idqp.hpp"

#include "grasp_idqp_parallel_solver.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>

namespace plato_robot_system::task
{
namespace
{

constexpr double kMinDt = 1.0e-9;
constexpr double kMinAxisNorm = 1.0e-12;

bool IsFinite(const double value)
{
  return std::isfinite(value);
}

bool IsNonnegativeFinite(const double value)
{
  return IsFinite(value) && value >= 0.0;
}

bool IsPositiveFinite(const double value)
{
  return IsFinite(value) && value > 0.0;
}

bool IsUnitIntervalFinite(const double value)
{
  return IsFinite(value) && value >= 0.0 && value <= 1.0;
}

bool IsKnownBackend(const GraspIDQPBackend backend)
{
  return backend == GraspIDQPBackend::kLegacyTrigPosition ||
         backend == GraspIDQPBackend::kAccelerationIdQp;
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

bool HasValidParallelGeometry(const GraspIDQPConfig & config)
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

bool HasValidBaseConfig(const GraspIDQPConfig & config)
{
  return IsKnownBackend(config.backend) &&
         (config.q_ready.size() == 0 || config.q_ready.allFinite()) &&
         config.force_enter_debounce_ticks >= 0 &&
         config.force_exit_contact_lost_ticks >= 0 &&
         IsFinite(config.force_exit_u_threshold) &&
         IsNonnegativeFinite(config.min_contact_force_n) &&
         IsUnitIntervalFinite(config.lpf_alpha) &&
         IsFinite(config.kp_tactile_u_fb) &&
         IsFinite(config.kd_tactile_u_fb) &&
         IsFinite(config.kp_tactile_phi_fb) &&
         IsFinite(config.kd_tactile_phi_fb) &&
         IsNonnegativeFinite(config.debug_print_contact_interval_s) &&
         HasValidParallelGeometry(config);
}

bool NeedsPinocchioContactFrames(const GraspIDQPConfig & config)
{
  return config.use_pinocchio_parallel_solver ||
         config.backend == GraspIDQPBackend::kAccelerationIdQp;
}

bool HasValidIdQpConfig(const GraspIDQPConfig & config)
{
  return IsNonnegativeFinite(config.idqp_dt_min) &&
         IsNonnegativeFinite(config.idqp_qddot_limit_rad_s2) &&
         IsNonnegativeFinite(config.idqp_qdot_limit_rad_s) &&
         IsNonnegativeFinite(config.idqp_tau_limit_nm) &&
         IsNonnegativeFinite(config.idqp_w_aperture) &&
         IsNonnegativeFinite(config.idqp_w_parallel) &&
         IsNonnegativeFinite(config.idqp_w_moment) &&
         IsNonnegativeFinite(config.idqp_w_posture) &&
         IsNonnegativeFinite(config.idqp_w_acceleration) &&
         IsNonnegativeFinite(config.idqp_kp_aperture) &&
         IsNonnegativeFinite(config.idqp_kd_aperture) &&
         IsNonnegativeFinite(config.idqp_kp_parallel) &&
         IsNonnegativeFinite(config.idqp_kd_parallel) &&
         IsNonnegativeFinite(config.idqp_kp_moment) &&
         IsNonnegativeFinite(config.idqp_kd_moment) &&
         IsNonnegativeFinite(config.idqp_kp_posture) &&
         IsNonnegativeFinite(config.idqp_kd_posture);
}

bool HasValidSolverConfig(const GraspIDQPConfig & config)
{
  if (!HasValidBaseConfig(config)) {
    return false;
  }
  if (!config.index_contact_normal_axis_frame.allFinite() ||
    !config.thumb_contact_normal_axis_frame.allFinite())
  {
    return false;
  }
  if (NeedsPinocchioContactFrames(config)) {
    if (config.index_contact_point_frame.empty() || config.thumb_contact_point_frame.empty()) {
      return false;
    }
    if (
      config.index_contact_normal_axis_frame.norm() <= kMinAxisNorm ||
      config.thumb_contact_normal_axis_frame.norm() <= kMinAxisNorm)
    {
      return false;
    }
  }

  return IsNonnegativeFinite(config.parallel_solver_w_aperture) &&
         IsNonnegativeFinite(config.parallel_solver_w_parallel) &&
         IsNonnegativeFinite(config.parallel_solver_w_moment) &&
         IsNonnegativeFinite(config.parallel_solver_w_posture) &&
         IsNonnegativeFinite(config.parallel_solver_w_smooth) &&
         IsPositiveFinite(config.parallel_solver_damping) &&
         IsPositiveFinite(config.parallel_solver_fd_eps_rad) &&
         IsPositiveFinite(config.parallel_solver_max_step_rad) &&
         config.parallel_solver_max_iters >= 0 &&
         HasValidIdQpConfig(config);
}

double ClampToFinitePositionLimit(
  const pinocchio::Model & model,
  const int q_index,
  const double value)
{
  if (
    q_index < 0 ||
    q_index >= model.nq ||
    model.lowerPositionLimit.size() != model.nq ||
    model.upperPositionLimit.size() != model.nq)
  {
    return value;
  }

  const double lower = model.lowerPositionLimit[q_index];
  const double upper = model.upperPositionLimit[q_index];
  if (!IsFinite(lower) || !IsFinite(upper) || lower >= upper) {
    return value;
  }
  return Clamp(value, lower, upper);
}

struct IdQpContactGeometry
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_index_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_thumb_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d n_index_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d n_thumb_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d n_pair_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_perp_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d parallel_residual{Eigen::Vector3d::Zero()};
  double aperture_m{0.0};
  double moment_arm_m{0.0};
  bool valid{false};
};

bool EvaluateIdQpContactGeometry(
  const pinocchio::Model & model,
  const pinocchio::FrameIndex index_contact_point_frame_id,
  const pinocchio::FrameIndex thumb_contact_point_frame_id,
  const Eigen::Vector3d & index_contact_normal_axis_frame,
  const Eigen::Vector3d & thumb_contact_normal_axis_frame,
  const Eigen::VectorXd & q_candidate,
  IdQpContactGeometry * geometry)
{
  if (
    geometry == nullptr ||
    model.nq <= 0 ||
    model.nv <= 0 ||
    q_candidate.size() != model.nq ||
    !q_candidate.allFinite() ||
    index_contact_point_frame_id >= static_cast<pinocchio::FrameIndex>(model.nframes) ||
    thumb_contact_point_frame_id >= static_cast<pinocchio::FrameIndex>(model.nframes) ||
    !index_contact_normal_axis_frame.allFinite() ||
    !thumb_contact_normal_axis_frame.allFinite())
  {
    return false;
  }

  const double index_axis_norm = index_contact_normal_axis_frame.norm();
  const double thumb_axis_norm = thumb_contact_normal_axis_frame.norm();
  if (index_axis_norm <= kMinAxisNorm || thumb_axis_norm <= kMinAxisNorm) {
    return false;
  }

  pinocchio::Data data(model);
  const Eigen::VectorXd zero_qdot = Eigen::VectorXd::Zero(model.nv);
  pinocchio::forwardKinematics(model, data, q_candidate, zero_qdot);
  pinocchio::updateFramePlacements(model, data);

  const pinocchio::SE3 & index_pose = data.oMf[index_contact_point_frame_id];
  const pinocchio::SE3 & thumb_pose = data.oMf[thumb_contact_point_frame_id];

  IdQpContactGeometry result;
  result.p_index_world = index_pose.translation();
  result.p_thumb_world = thumb_pose.translation();
  result.n_index_world =
    index_pose.rotation() * (index_contact_normal_axis_frame / index_axis_norm);
  result.n_thumb_world =
    thumb_pose.rotation() * (thumb_contact_normal_axis_frame / thumb_axis_norm);

  const Eigen::Vector3d r_world = result.p_index_world - result.p_thumb_world;
  const Eigen::Vector3d n_pair_raw = result.n_index_world - result.n_thumb_world;
  const double n_pair_norm = n_pair_raw.norm();
  if (n_pair_norm <= kMinAxisNorm) {
    return false;
  }
  result.n_pair_world = n_pair_raw / n_pair_norm;
  result.aperture_m = r_world.dot(result.n_pair_world);
  result.r_perp_world = r_world - result.aperture_m * result.n_pair_world;
  result.parallel_residual = result.n_index_world + result.n_thumb_world;
  result.moment_arm_m = result.r_perp_world.norm();
  result.valid =
    result.p_index_world.allFinite() &&
    result.p_thumb_world.allFinite() &&
    result.n_index_world.allFinite() &&
    result.n_thumb_world.allFinite() &&
    result.n_pair_world.allFinite() &&
    result.r_perp_world.allFinite() &&
    result.parallel_residual.allFinite() &&
    IsFinite(result.aperture_m) &&
    IsFinite(result.moment_arm_m);
  if (!result.valid) {
    return false;
  }

  *geometry = result;
  return true;
}

bool BuildIdQpResidual(
  const pinocchio::Model & model,
  const GraspIDQPConfig & config,
  const std::array<int, kThumbIndexActiveJoints.size()> & active_q_indices,
  const int active_dof,
  const pinocchio::FrameIndex index_contact_point_frame_id,
  const pinocchio::FrameIndex thumb_contact_point_frame_id,
  const Eigen::VectorXd & q_candidate,
  const Eigen::VectorXd & q_seed,
  const double aperture_des_m,
  Eigen::VectorXd * residual,
  IdQpContactGeometry * geometry)
{
  if (
    residual == nullptr ||
    geometry == nullptr ||
    active_dof <= 0 ||
    q_candidate.size() != model.nq ||
    q_seed.size() != model.nq ||
    !q_candidate.allFinite() ||
    !q_seed.allFinite() ||
    !IsFinite(aperture_des_m))
  {
    return false;
  }

  IdQpContactGeometry current_geometry;
  if (
    !EvaluateIdQpContactGeometry(
      model,
      index_contact_point_frame_id,
      thumb_contact_point_frame_id,
      config.index_contact_normal_axis_frame,
      config.thumb_contact_normal_axis_frame,
      q_candidate,
      &current_geometry))
  {
    return false;
  }

  Eigen::VectorXd result(1 + 3 + 3 + active_dof);
  int row = 0;
  result[row++] = current_geometry.aperture_m - aperture_des_m;
  result.segment<3>(row) = current_geometry.parallel_residual;
  row += 3;
  result.segment<3>(row) = current_geometry.r_perp_world;
  row += 3;
  for (int i = 0; i < active_dof; ++i) {
    const int q_index = active_q_indices[static_cast<std::size_t>(i)];
    if (q_index < 0 || q_index >= model.nq) {
      return false;
    }
    result[row++] = q_candidate[q_index] - q_seed[q_index];
  }

  if (!result.allFinite()) {
    return false;
  }

  *residual = result;
  *geometry = current_geometry;
  return true;
}

sensor::TactileGripObservationConfig BuildGripObservationConfig(
  const GraspIDQPConfig & config)
{
  sensor::TactileGripObservationConfig observation_config;
  observation_config.frame_a_name =
    std::string(kThumbIndexFrameA.data(), kThumbIndexFrameA.size());
  observation_config.frame_b_name =
    std::string(kThumbIndexFrameB.data(), kThumbIndexFrameB.size());
  observation_config.min_contact_force_n = config.min_contact_force_n;
  observation_config.use_tactile_presence_for_contact =
    config.use_tactile_presence_for_contact;
  observation_config.force_aggregation = config.force_aggregation;
  return observation_config;
}

}  // namespace

bool GraspIDQP::Configure(
  const pinocchio::Model & model,
  const GraspIDQPConfig & config)
{
  configured_ = false;
  has_reference_posture_ = false;
  q_reference_.resize(0);
  q_target_lpf_.resize(0);
  has_q_target_lpf_ = false;
  has_pinocchio_parallel_frames_ = false;
  Reset();

  if (model.nq <= 0 || model.nv <= 0 || !HasValidSolverConfig(config)) {
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
    const int v_index = model.idx_vs[joint_id];
    if (q_index < 0 || q_index >= model.nq || v_index < 0 || v_index >= model.nv) {
      return false;
    }
    active_q_indices_[i] = q_index;
    active_v_indices_[i] = v_index;
  }

  if (NeedsPinocchioContactFrames(config)) {
    if (
      !ResolveGraspIDQPParallelFrames(
        model,
        config,
        &index_contact_point_frame_id_,
        &thumb_contact_point_frame_id_))
    {
      return false;
    }
    has_pinocchio_parallel_frames_ = true;
  }

  config_ = config;
  configured_ = true;
  Reset();
  return true;
}

void GraspIDQP::Reset()
{
  mode_ = GraspIDQPMode::kMotionTeleop;
  force_enter_counter_ = 0;
  force_exit_contact_lost_counter_ = 0;
  last_force_error_n_ = 0.0;
  has_last_force_error_ = false;
  last_contact_debug_print_time_s_ = -1.0e100;
  status_ = GraspIDQPStatus{};
  status_.q_target = Eigen::VectorXd::Zero(0);
  status_.qddot_sol = Eigen::VectorXd::Zero(0);
  status_.tau_ff_active = Eigen::VectorXd::Zero(0);
  q_target_lpf_.resize(0);
  has_q_target_lpf_ = false;
}

bool GraspIDQP::OnEnter(RobotSystem & robot, const RobotState & state)
{
  if (!HasCompatibleState(robot, state)) {
    return false;
  }

  Reset();
  return CaptureReferencePosture(state);
}

bool GraspIDQP::PopulateCommand(
  RobotSystem & robot,
  const RobotState & state,
  const GraspIDQPCommand & input,
  const double dt_sec,
  RobotCommand * command)
{
  if (command == nullptr) {
    return false;
  }

  if (config_.backend == GraspIDQPBackend::kAccelerationIdQp) {
    return PopulateAccelerationIdQpCommand(robot, state, input, dt_sec, command);
  }

  Eigen::VectorXd q_target;
  if (!BuildMotionTarget(robot, state, input, dt_sec, &q_target)) {
    return false;
  }

  command->Resize(robot.nq(), robot.nv());
  command->q_cmd = q_target;
  command->qdot_cmd.setZero();
  command->tau_cmd.setZero();
  command->stamp_sec = state.time_s;
  command->valid = command->HasValidDimensions() && command->AllFinite();
  return command->valid;
}

bool GraspIDQP::BuildMotionTarget(
  RobotSystem & robot,
  const RobotState & state,
  const GraspIDQPCommand & input,
  const double dt_sec,
  Eigen::VectorXd * q_target,
  GraspIDQPStatus * status)
{
  if (q_target == nullptr) {
    return false;
  }

  sensor::TactileGripObservation estimate;
  GraspIDQPCommand clamped_input;
  if (!PrepareTick(robot, state, input, dt_sec, &estimate, &clamped_input)) {
    return false;
  }

  const bool built = BuildParallelJointPositionTarget(
    robot,
    state,
    clamped_input,
    estimate,
    dt_sec,
    q_target);
  if (status != nullptr) {
    *status = status_;
  }
  return built;
}

bool GraspIDQP::HasCompatibleState(
  const RobotSystem & robot,
  const RobotState & state) const
{
  return configured_ && robot.hasModel() && robot.hasState() && IsValid(state) &&
         state.q.size() == robot.nq() && state.qdot.size() == robot.nv() &&
         state.tau.size() == robot.nv();
}

bool GraspIDQP::CaptureReferencePosture(const RobotState & state)
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

sensor::TactileGripObservation GraspIDQP::EstimateGripForceFromTactile(
  const RobotState & state) const
{
  return sensor::ObserveTactileGrip(
    state.tactile_sensors,
    BuildGripObservationConfig(config_));
}

void GraspIDQP::PrintContactStatesIfNeeded(
  const RobotState & state,
  const sensor::TactileGripObservation & estimate)
{
  if (!config_.debug_print_contact_states || !std::isfinite(state.time_s)) {
    return;
  }

  const double interval_s = config_.debug_print_contact_interval_s;
  if (
    interval_s > 0.0 &&
    state.time_s - last_contact_debug_print_time_s_ < interval_s)
  {
    return;
  }
  last_contact_debug_print_time_s_ = state.time_s;

  std::ostringstream message;
  message << std::fixed << std::setprecision(4)
          << "[grasp_idqp] tactile contact states t=" << state.time_s
          << " sensors=" << state.tactile_sensors.size()
          << " contact_count=" << estimate.ContactCount()
          << " enough_count=" << estimate.EnoughContactCount();

  for (const auto & tactile : state.tactile_sensors) {
    message << " | idx=" << tactile.sensor_index
            << " frame=" << tactile.frame_name
            << " valid=" << (tactile.valid ? "true" : "false")
            << " state=" << sensor::TactileContactStateName(tactile.contact_state)
            << " active_hemi=" << tactile.ActiveHemisphereCount()
            << " force_n=" << tactile.ActiveHemisphereNormalForceN();
  }

  std::cout << message.str() << std::endl;
}

void GraspIDQP::UpdateMode(
  const sensor::TactileGripObservation & estimate,
  const double u)
{
  if (mode_ == GraspIDQPMode::kMotionTeleop) {
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
      mode_ = GraspIDQPMode::kForceTracking;
      last_force_error_n_ = 0.0;
      has_last_force_error_ = false;
      force_exit_contact_lost_counter_ = 0;
    }
    return;
  }

  if (mode_ == GraspIDQPMode::kForceTracking) {
    if (u > config_.force_exit_u_threshold) {
      mode_ = GraspIDQPMode::kMotionTeleop;
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
      mode_ = GraspIDQPMode::kMotionTeleop;
      force_enter_counter_ = 0;
      force_exit_contact_lost_counter_ = 0;
      last_force_error_n_ = 0.0;
      has_last_force_error_ = false;
    }
  }
}

bool GraspIDQP::PrepareTick(
  const RobotSystem & robot,
  const RobotState & state,
  const GraspIDQPCommand & input,
  const double dt_sec,
  sensor::TactileGripObservation * estimate,
  GraspIDQPCommand * clamped_input)
{
  if (
    estimate == nullptr ||
    clamped_input == nullptr ||
    !HasCompatibleState(robot, state) ||
    !IsFinite(dt_sec) ||
    dt_sec <= kMinDt ||
    !IsFinite(input.u) ||
    !IsFinite(input.phi) ||
    !IsFinite(input.desired_force_n))
  {
    return false;
  }
  if (!has_reference_posture_ && !CaptureReferencePosture(state)) {
    return false;
  }

  const double u_open = Clamp01(input.u);
  const double phi = Clamp01(input.phi);
  const double desired_force_n = std::max(0.0, input.desired_force_n);

  *estimate = EstimateGripForceFromTactile(state);
  PrintContactStatesIfNeeded(state, *estimate);
  UpdateMode(*estimate, u_open);
  if (mode_ == GraspIDQPMode::kForceTracking && !estimate->valid_force) {
    mode_ = GraspIDQPMode::kMotionTeleop;
    force_enter_counter_ = 0;
    force_exit_contact_lost_counter_ = 0;
    last_force_error_n_ = 0.0;
    has_last_force_error_ = false;
  }

  status_.mode = mode_;
  status_.u = u_open;
  status_.phi = phi;
  status_.desired_force_n = desired_force_n;
  status_.measured_force_n = estimate->measured_force_n;
  status_.force_a_n = estimate->force_a_n;
  status_.force_b_n = estimate->force_b_n;
  status_.contact_a = estimate->contact_a;
  status_.contact_b = estimate->contact_b;
  status_.enough_contact_a = estimate->enough_contact_a;
  status_.enough_contact_b = estimate->enough_contact_b;
  status_.lost_contact_a = estimate->lost_contact_a;
  status_.lost_contact_b = estimate->lost_contact_b;
  status_.valid_force = estimate->valid_force;
  status_.contact_count = estimate->ContactCount();
  status_.enough_contact_count = estimate->EnoughContactCount();
  status_.force_enter_counter = force_enter_counter_;
  status_.force_exit_contact_lost_counter = force_exit_contact_lost_counter_;

  clamped_input->u = u_open;
  clamped_input->phi = phi;
  clamped_input->desired_force_n = desired_force_n;
  return true;
}

double GraspIDQP::ParallelQ5Geometry(const double q3) const
{
  const double cos_q5 = Clamp(
    std::cos(q3) - config_.parallel_lateral_offset_m / config_.parallel_tip_radius_m,
    -1.0,
    1.0);
  return std::max(config_.parallel_q5_min_rad, std::acos(cos_q5));
}

bool GraspIDQP::BuildLegacyParallelSeedTarget(
  const RobotSystem & robot,
  const RobotState & state,
  const GraspIDQPCommand & input,
  const sensor::TactileGripObservation & estimate,
  const double dt_sec,
  Eigen::VectorXd * q_seed)
{
  if (q_seed == nullptr || !HasCompatibleState(robot, state) ||
    !IsFinite(dt_sec) || dt_sec <= kMinDt ||
    q_reference_.size() != robot.nq() || !q_reference_.allFinite())
  {
    return false;
  }

  double effective_u = input.u;
  double effective_phi = input.phi;
  status_.force_error_n = 0.0;
  if (mode_ == GraspIDQPMode::kForceTracking && estimate.valid_force) {
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

  const double u_open = Clamp01(effective_u);

  double q3_target = 0.0;
  double q5_target = 0.0;
  if (u_open > config_.parallel_midpoint_u) {
    const double ratio =
      (u_open - config_.parallel_midpoint_u) /
      (1.0 - config_.parallel_midpoint_u);
    const double q5_neutral = ParallelQ5Geometry(0.0);
    q3_target = 0.0;
    q5_target = std::max(
      config_.parallel_q5_min_rad,
      q5_neutral - ratio * (q5_neutral - config_.parallel_q5_min_rad));
  } else {
    const double ratio =
      (config_.parallel_midpoint_u - u_open) / config_.parallel_midpoint_u;
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

  status_.effective_u = effective_u;
  status_.effective_phi = effective_phi;
  status_.u_parallel = u_open;
  *q_seed = q_target;
  return true;
}

bool GraspIDQP::BuildParallelJointPositionTarget(
  const RobotSystem & robot,
  const RobotState & state,
  const GraspIDQPCommand & input,
  const sensor::TactileGripObservation & estimate,
  const double dt_sec,
  Eigen::VectorXd * q_target_out)
{
  if (q_target_out == nullptr) {
    return false;
  }

  Eigen::VectorXd q_seed;
  if (!BuildLegacyParallelSeedTarget(robot, state, input, estimate, dt_sec, &q_seed)) {
    return false;
  }

  Eigen::VectorXd q_target = q_seed;
  status_.aperture_des_m = 0.0;
  status_.aperture_m = 0.0;
  status_.parallel_axis_error = 0.0;
  status_.moment_arm_m = 0.0;
  status_.solver_cost = 0.0;
  status_.solver_iters = 0;
  status_.used_pinocchio_parallel_solver = false;
  status_.used_idqp = false;
  status_.idqp_solved = false;
  status_.idqp_cost = 0.0;
  status_.qddot_sol = Eigen::VectorXd::Zero(0);
  status_.tau_ff_active = Eigen::VectorXd::Zero(0);
  status_.fallback_used = false;

  if (config_.use_pinocchio_parallel_solver) {
    const bool has_smooth_reference =
      has_q_target_lpf_ &&
      q_target_lpf_.size() == q_seed.size() &&
      q_target_lpf_.allFinite();
    if (
      !configured_ ||
      !robot.hasModel() ||
      !has_pinocchio_parallel_frames_ ||
      !RefineGraspIDQPParallelTarget(
        robot.model(),
        config_,
        active_q_indices_,
        active_dof_,
        index_contact_point_frame_id_,
        thumb_contact_point_frame_id_,
        q_seed,
        q_target_lpf_,
        has_smooth_reference,
        &q_target,
        &status_))
    {
      return false;
    }
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

  status_.q_target = q_command;
  *q_target_out = q_command;
  return true;
}

bool GraspIDQP::PopulateIdQpFallbackCommand(
  const RobotSystem & robot,
  const RobotState & state,
  const Eigen::VectorXd & q_seed,
  RobotCommand * command)
{
  if (
    command == nullptr ||
    q_seed.size() != robot.nq() ||
    !q_seed.allFinite())
  {
    return false;
  }

  command->Resize(robot.nq(), robot.nv());
  command->q_cmd = q_seed;
  command->qdot_cmd.setZero();
  command->tau_cmd.setZero();
  command->stamp_sec = state.time_s;
  command->valid = command->HasValidDimensions() && command->AllFinite();

  status_.q_target = q_seed;
  status_.used_pinocchio_parallel_solver = false;
  status_.used_idqp = true;
  status_.idqp_solved = false;
  status_.fallback_used = true;
  status_.qddot_sol = Eigen::VectorXd::Zero(active_dof_);
  status_.tau_ff_active = Eigen::VectorXd::Zero(active_dof_);
  return command->valid;
}

bool GraspIDQP::PopulateAccelerationIdQpCommand(
  RobotSystem & robot,
  const RobotState & state,
  const GraspIDQPCommand & input,
  const double dt_sec,
  RobotCommand * command)
{
  if (command == nullptr) {
    return false;
  }

  sensor::TactileGripObservation estimate;
  GraspIDQPCommand clamped_input;
  if (!PrepareTick(robot, state, input, dt_sec, &estimate, &clamped_input)) {
    return false;
  }

  Eigen::VectorXd q_seed;
  if (!BuildLegacyParallelSeedTarget(
      robot,
      state,
      clamped_input,
      estimate,
      dt_sec,
      &q_seed))
  {
    return false;
  }

  status_.aperture_des_m = 0.0;
  status_.aperture_m = 0.0;
  status_.parallel_axis_error = 0.0;
  status_.moment_arm_m = 0.0;
  status_.solver_cost = 0.0;
  status_.solver_iters = 0;
  status_.used_pinocchio_parallel_solver = false;
  status_.used_idqp = true;
  status_.idqp_solved = false;
  status_.idqp_cost = 0.0;
  status_.qddot_sol = Eigen::VectorXd::Zero(active_dof_);
  status_.tau_ff_active = Eigen::VectorXd::Zero(active_dof_);
  status_.fallback_used = false;

  if (
    !has_pinocchio_parallel_frames_ ||
    !IsFinite(dt_sec) ||
    dt_sec <= config_.idqp_dt_min ||
    config_.parallel_solver_fd_eps_rad <= 0.0)
  {
    return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
  }

  const pinocchio::Model & model = robot.model();
  IdQpContactGeometry seed_geometry;
  if (
    !EvaluateIdQpContactGeometry(
      model,
      index_contact_point_frame_id_,
      thumb_contact_point_frame_id_,
      config_.index_contact_normal_axis_frame,
      config_.thumb_contact_normal_axis_frame,
      q_seed,
      &seed_geometry))
  {
    return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
  }
  const double aperture_des_m = seed_geometry.aperture_m;

  Eigen::VectorXd residual;
  IdQpContactGeometry current_geometry;
  if (
    !BuildIdQpResidual(
      model,
      config_,
      active_q_indices_,
      active_dof_,
      index_contact_point_frame_id_,
      thumb_contact_point_frame_id_,
      state.q,
      q_seed,
      aperture_des_m,
      &residual,
      &current_geometry))
  {
    return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
  }

  const int residual_dim = static_cast<int>(residual.size());
  Eigen::MatrixXd residual_jacobian =
    Eigen::MatrixXd::Zero(residual_dim, active_dof_);
  const double fd_eps = config_.parallel_solver_fd_eps_rad;
  for (int i = 0; i < active_dof_; ++i) {
    const int q_index = active_q_indices_[static_cast<std::size_t>(i)];
    Eigen::VectorXd q_plus = state.q;
    Eigen::VectorXd q_minus = state.q;
    q_plus[q_index] += fd_eps;
    q_minus[q_index] -= fd_eps;

    Eigen::VectorXd residual_plus;
    Eigen::VectorXd residual_minus;
    IdQpContactGeometry geometry_plus;
    IdQpContactGeometry geometry_minus;
    if (
      !BuildIdQpResidual(
        model,
        config_,
        active_q_indices_,
        active_dof_,
        index_contact_point_frame_id_,
        thumb_contact_point_frame_id_,
        q_plus,
        q_seed,
        aperture_des_m,
        &residual_plus,
        &geometry_plus) ||
      !BuildIdQpResidual(
        model,
        config_,
        active_q_indices_,
        active_dof_,
        index_contact_point_frame_id_,
        thumb_contact_point_frame_id_,
        q_minus,
        q_seed,
        aperture_des_m,
        &residual_minus,
        &geometry_minus))
    {
      return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
    }

    residual_jacobian.col(i) = (residual_plus - residual_minus) / (2.0 * fd_eps);
  }

  Eigen::VectorXd qdot_active = Eigen::VectorXd::Zero(active_dof_);
  for (int i = 0; i < active_dof_; ++i) {
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    qdot_active[i] = state.qdot[v_index];
  }
  const Eigen::VectorXd residual_dot = residual_jacobian * qdot_active;

  Eigen::MatrixXd task_a = Eigen::MatrixXd::Zero(residual_dim, active_dof_);
  Eigen::VectorXd task_b = Eigen::VectorXd::Zero(residual_dim);
  auto add_rows =
    [&task_a, &task_b, &residual_jacobian, &residual, &residual_dot](
    const int row_begin,
    const int row_count,
    const double weight,
    const double kp,
    const double kd)
    {
      const double scale = std::sqrt(std::max(0.0, weight));
      for (int row = row_begin; row < row_begin + row_count; ++row) {
        task_a.row(row) = scale * residual_jacobian.row(row);
        task_b[row] = scale * (-kp * residual[row] - kd * residual_dot[row]);
      }
    };

  add_rows(0, 1, config_.idqp_w_aperture, config_.idqp_kp_aperture, config_.idqp_kd_aperture);
  add_rows(1, 3, config_.idqp_w_parallel, config_.idqp_kp_parallel, config_.idqp_kd_parallel);
  add_rows(4, 3, config_.idqp_w_moment, config_.idqp_kp_moment, config_.idqp_kd_moment);
  add_rows(
    7, active_dof_, config_.idqp_w_posture, config_.idqp_kp_posture,
    config_.idqp_kd_posture);

  const double numerical_regularization =
    std::max(config_.idqp_w_acceleration, 1.0e-12);
  Eigen::MatrixXd normal_matrix =
    task_a.transpose() * task_a +
    numerical_regularization * Eigen::MatrixXd::Identity(active_dof_, active_dof_);
  const Eigen::VectorXd normal_rhs = task_a.transpose() * task_b;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(normal_matrix);
  if (ldlt.info() != Eigen::Success) {
    return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
  }

  Eigen::VectorXd qddot_active = ldlt.solve(normal_rhs);
  if (!qddot_active.allFinite()) {
    return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
  }

  for (int i = 0; i < active_dof_; ++i) {
    qddot_active[i] = Clamp(
      qddot_active[i],
      -config_.idqp_qddot_limit_rad_s2,
      config_.idqp_qddot_limit_rad_s2);
  }

  Eigen::VectorXd qdot_cmd = state.qdot;
  for (int i = 0; i < active_dof_; ++i) {
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    const double qdot_next = Clamp(
      state.qdot[v_index] + qddot_active[i] * dt_sec,
      -config_.idqp_qdot_limit_rad_s,
      config_.idqp_qdot_limit_rad_s);
    qdot_cmd[v_index] = qdot_next;
    qddot_active[i] = (qdot_next - state.qdot[v_index]) / dt_sec;
  }

  Eigen::VectorXd q_cmd = pinocchio::integrate(model, state.q, qdot_cmd * dt_sec);
  if (!q_cmd.allFinite()) {
    return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
  }

  for (int i = 0; i < active_dof_; ++i) {
    const int q_index = active_q_indices_[static_cast<std::size_t>(i)];
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    const double q_clamped = ClampToFinitePositionLimit(model, q_index, q_cmd[q_index]);
    if (q_clamped != q_cmd[q_index]) {
      q_cmd[q_index] = q_clamped;
      qdot_cmd[v_index] = (q_clamped - state.q[q_index]) / dt_sec;
      qddot_active[i] = (qdot_cmd[v_index] - state.qdot[v_index]) / dt_sec;
    }
  }

  Eigen::VectorXd qddot_full = Eigen::VectorXd::Zero(robot.nv());
  for (int i = 0; i < active_dof_; ++i) {
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    qddot_full[v_index] = qddot_active[i];
  }

  Eigen::VectorXd tau_full;
  try {
    tau_full = robot.InverseDynamics(state.q, state.qdot, qddot_full);
  } catch (...) {
    return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
  }
  if (tau_full.size() != robot.nv() || !tau_full.allFinite()) {
    return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
  }

  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(robot.nv());
  Eigen::VectorXd tau_active = Eigen::VectorXd::Zero(active_dof_);
  for (int i = 0; i < active_dof_; ++i) {
    const int v_index = active_v_indices_[static_cast<std::size_t>(i)];
    const double tau_value = Clamp(
      tau_full[v_index],
      -config_.idqp_tau_limit_nm,
      config_.idqp_tau_limit_nm);
    tau_cmd[v_index] = tau_value;
    tau_active[i] = tau_value;
  }

  const double idqp_cost =
    (task_a * qddot_active - task_b).squaredNorm() +
    config_.idqp_w_acceleration * qddot_active.squaredNorm();

  command->Resize(robot.nq(), robot.nv());
  command->q_cmd = q_cmd;
  command->qdot_cmd = qdot_cmd;
  command->tau_cmd = tau_cmd;
  command->stamp_sec = state.time_s;
  command->valid = command->HasValidDimensions() && command->AllFinite();
  if (!command->valid) {
    return PopulateIdQpFallbackCommand(robot, state, q_seed, command);
  }

  status_.q_target = q_cmd;
  status_.aperture_des_m = aperture_des_m;
  status_.aperture_m = current_geometry.aperture_m;
  status_.parallel_axis_error = current_geometry.parallel_residual.norm();
  status_.moment_arm_m = current_geometry.moment_arm_m;
  status_.solver_cost = idqp_cost;
  status_.solver_iters = 1;
  status_.used_pinocchio_parallel_solver = false;
  status_.used_idqp = true;
  status_.idqp_solved = true;
  status_.idqp_cost = idqp_cost;
  status_.qddot_sol = qddot_active;
  status_.tau_ff_active = tau_active;
  status_.fallback_used = false;
  return true;
}

}  // namespace plato_robot_system::task
