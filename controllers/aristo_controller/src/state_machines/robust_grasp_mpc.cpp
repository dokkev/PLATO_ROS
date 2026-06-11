#include "aristo_controller/state_machines/robust_grasp_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string_view>
#include <utility>

#include "mppi_core/object/object_belief_initializer.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "plato_robot_system/task/thumb_index_grasp_constants.hpp"

namespace aristo_controller::state_machines
{
namespace
{

constexpr std::size_t kThumbContextIndex = 0;
constexpr std::size_t kIndexContextIndex = 1;
constexpr double kBeliefWeightEps = 1.0e-12;

using MppiTactileStateVector =
  std::vector<mppi_core::TactileState, Eigen::aligned_allocator<mppi_core::TactileState>>;

bool SameFrame(const std::string & frame_name, const std::string_view expected)
{
  return frame_name == std::string(expected);
}

mppi_core::HemisphereState ConvertHemisphere(
  const plato_robot_system::sensor::HemisphereState & input)
{
  mppi_core::HemisphereState output;
  output.hemisphere_index = input.hemisphere_index;
  output.contact = input.contact;
  output.cop_sensor_m = input.cop_sensor_m;
  output.normal_force_n = input.normal_force_n;
  output.confidence = input.confidence;
  return output;
}

mppi_core::TactileState ConvertTactileState(
  const plato_robot_system::sensor::TactileState & input)
{
  mppi_core::TactileState output;
  output.valid = input.valid;
  output.stamp_sec = input.stamp_sec;
  output.sensor_index = input.sensor_index;
  output.frame_name = input.frame_name;
  output.contact_state = input.contact_state;
  output.total_force_n = input.total_force_n;
  output.shear_displacement_m = input.shear_displacement_m;
  output.rotational_shear_rad = input.rotational_shear_rad;
  output.shear_velocity_mps = input.shear_velocity_mps;
  output.rotational_shear_velocity_radps = input.rotational_shear_velocity_radps;
  output.slip_score = input.slip_score;
  output.slip_velocity_score = input.slip_velocity_score;
  output.incipient_slip_score = input.incipient_slip_score;
  output.confidence = input.confidence;
  output.hemispheres.reserve(input.hemispheres.size());
  for (const auto & hemisphere : input.hemispheres) {
    output.hemispheres.push_back(ConvertHemisphere(hemisphere));
  }
  return output;
}

MppiTactileStateVector ConvertTactileSensors(
  const plato_robot_system::TactileSensorVector & input)
{
  MppiTactileStateVector output;
  output.reserve(input.size());
  for (const auto & tactile : input) {
    output.push_back(ConvertTactileState(tactile));
  }
  return output;
}

std::vector<mppi_core::HemisphereGeometry> BuildHemisphereGeometry(
  const mppi_core::TactileState & tactile)
{
  std::vector<mppi_core::HemisphereGeometry> geometry;
  geometry.reserve(tactile.hemispheres.size());
  for (std::size_t i = 0; i < tactile.hemispheres.size(); ++i) {
    const auto & hemisphere = tactile.hemispheres[i];
    mppi_core::HemisphereGeometry hemisphere_geometry;
    hemisphere_geometry.hemisphere_index = hemisphere.hemisphere_index;
    hemisphere_geometry.center_sensor_m =
      Eigen::Vector3d{hemisphere.cop_sensor_m.x(), hemisphere.cop_sensor_m.y(), 0.0};
    hemisphere_geometry.normal_sensor = Eigen::Vector3d::UnitZ();
    if (i > 0) {
      hemisphere_geometry.neighbors.push_back(tactile.hemispheres[i - 1].hemisphere_index);
    }
    if (i + 1 < tactile.hemispheres.size()) {
      hemisphere_geometry.neighbors.push_back(tactile.hemispheres[i + 1].hemisphere_index);
    }
    geometry.push_back(std::move(hemisphere_geometry));
  }
  return geometry;
}

bool HasCompatibleRobotState(
  const plato_robot_system::RobotSystem & robot,
  const plato_robot_system::RobotState & state)
{
  return plato_robot_system::IsValid(state) &&
         state.q.size() == robot.nq() &&
         state.qdot.size() == robot.nv() &&
         state.tau.size() == robot.nv();
}

double ClampFiniteRange(const double value, const double lower, const double upper)
{
  if (std::isfinite(lower) && std::isfinite(upper) && lower <= upper) {
    return std::clamp(value, lower, upper);
  }
  return value;
}

double ClampFiniteSymmetric(const double value, const double limit)
{
  if (!std::isfinite(limit)) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

double NormalizedAxisSign(const double value)
{
  return value < 0.0 ? -1.0 : 1.0;
}

int ActiveTactileSensorCount(const MppiTactileStateVector & tactile)
{
  int count = 0;
  for (const auto & sensor : tactile) {
    if (sensor.hasActiveHemisphereContact()) {
      ++count;
    }
  }
  return count;
}

int ActiveHemisphereCountTotal(const MppiTactileStateVector & tactile)
{
  int count = 0;
  for (const auto & sensor : tactile) {
    count += static_cast<int>(sensor.activeHemisphereCount());
  }
  return count;
}

std::string FormatVector(
  const Eigen::Ref<const Eigen::VectorXd> & values,
  const int precision = 4)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << "[";
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}

bool RepresentativeObjectBeliefPose(
  const mppi_core::VirtualObjectBelief & belief,
  Eigen::Isometry3d * pose_world)
{
  if (
    pose_world == nullptr ||
    !mppi_core::HasVirtualObjectBelief(belief) ||
    !mppi_core::IsValidVirtualObjectBelief(belief) ||
    belief.particles.empty())
  {
    return false;
  }

  double weight_sum = 0.0;
  Eigen::Vector3d weighted_translation_m = Eigen::Vector3d::Zero();
  const mppi_core::VirtualObjectState * orientation_source = nullptr;
  double best_weight = -1.0;
  for (const auto & particle : belief.particles) {
    if (!mppi_core::IsValidVirtualObjectState(particle)) {
      continue;
    }
    const double weight =
      std::isfinite(particle.weight) && particle.weight > 0.0 ? particle.weight : 1.0;
    weighted_translation_m.noalias() += weight * particle.pose_world.translation();
    weight_sum += weight;
    if (weight > best_weight) {
      best_weight = weight;
      orientation_source = &particle;
    }
  }

  if (orientation_source == nullptr || weight_sum <= kBeliefWeightEps) {
    return false;
  }
  *pose_world = orientation_source->pose_world;
  pose_world->translation() = weighted_translation_m / weight_sum;
  return pose_world->matrix().allFinite();
}

}  // namespace

RobustGraspMpcState::RobustGraspMpcState(
  const plato_robot_system::StateId id,
  plato_robot_system::RobotSystem * robot)
: plato_robot_system::State(id, kName),
  robot_(robot)
{
}

bool RobustGraspMpcState::ConfigureTask(const RobustGraspMpcStateConfig & config)
{
  configured_ = false;
  if (robot_ == nullptr || !robot_->hasModel() || robot_->nv() <= 0) {
    return false;
  }

  config_ = config;
  config_.policy.rollout.action_dim = static_cast<std::size_t>(robot_->nv());
  config_.policy.action_library.action_dim = static_cast<std::size_t>(robot_->nv());
  if (!ConfigureContactKinematics()) {
    return false;
  }

  try {
    policy_.Initialize(config_.policy);
  } catch (const std::exception &) {
    return false;
  }

  last_command_ = plato_robot_system::RobotCommand{};
  object_belief_ = mppi_core::VirtualObjectBelief{};
  object_belief_contact_count_ = 0;
  object_belief_update_count_ = 0;
  object_belief_best_cost_ = std::numeric_limits<double>::infinity();
  object_belief_updated_this_tick_ = false;
  exit_requested_ = false;
  tick_index_ = 0;
  last_status_print_time_s_ = -1.0e100;
  configured_ = true;
  return true;
}

void RobustGraspMpcState::OnEnter()
{
  tick_index_ = 0;
  last_status_print_time_s_ = -1.0e100;
  last_command_ = plato_robot_system::RobotCommand{};
  object_belief_ = mppi_core::VirtualObjectBelief{};
  object_belief_contact_count_ = 0;
  object_belief_update_count_ = 0;
  object_belief_best_cost_ = std::numeric_limits<double>::infinity();
  object_belief_updated_this_tick_ = false;
  exit_requested_ = false;
  if (robot_ == nullptr || !robot_->hasState()) {
    return;
  }
  const auto & state = robot_->state();
  if (!HasCompatibleRobotState(*robot_, state)) {
    return;
  }
  last_command_ = plato_robot_system::MakeZeroHoldRobotCommand(state.q, state.qdot);
  last_command_.stamp_sec = state.time_s;
}

void RobustGraspMpcState::OnExit()
{
  last_command_ = plato_robot_system::RobotCommand{};
  object_belief_ = mppi_core::VirtualObjectBelief{};
  object_belief_contact_count_ = 0;
  object_belief_update_count_ = 0;
  object_belief_best_cost_ = std::numeric_limits<double>::infinity();
  object_belief_updated_this_tick_ = false;
  exit_requested_ = false;
}

const mppi_core::RobustGraspPolicyStatus & RobustGraspMpcState::policy_status() const
{
  return policy_.status();
}

bool RobustGraspMpcState::IsFinished() const
{
  UpdateExitCondition();
  return (exit_requested_ && lifecycle_.next_state_id >= 0) ||
         plato_robot_system::State::IsFinished();
}

bool RobustGraspMpcState::PopulateCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || !configured_ || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }
  ++tick_index_;
  mppi_core::GraspObservation observation;
  if (!BuildObservation(&observation)) {
    return PopulateHoldCommand(command);
  }

  try {
    auto next_command = policy_.Update(observation);
    if (config_.safety.rollout_only) {
      if (!PopulateHoldCommand(command)) {
        return false;
      }
      PrintStatus(observation.time_s, observation, *command);
      return true;
    }
    if (!next_command.IsUsable() || !ApplyCommandSafety(&next_command)) {
      return PopulateHoldCommand(command);
    }
    *command = next_command;
    last_command_ = next_command;
    PrintStatus(observation.time_s, observation, *command);
    return true;
  } catch (const std::exception & error) {
    std::cout << "[robust_grasp_mpc] fallback: " << error.what() << std::endl;
    return PopulateHoldCommand(command);
  }
}

bool RobustGraspMpcState::ConfigureContactKinematics()
{
  if (robot_ == nullptr || !robot_->hasModel()) {
    return false;
  }
  try {
    contact_kinematics_[kThumbContextIndex].model = &robot_->model();
    contact_kinematics_[kThumbContextIndex].data = &robot_->data();
    contact_kinematics_[kThumbContextIndex].sensor_frame_id =
      robot_->FrameId(std::string(plato_robot_system::task::kThumbIndexFrameB));
    contact_kinematics_[kThumbContextIndex].normal_axis_sign =
      NormalizedAxisSign(config_.tactile.thumb_normal_axis_sign);

    contact_kinematics_[kIndexContextIndex].model = &robot_->model();
    contact_kinematics_[kIndexContextIndex].data = &robot_->data();
    contact_kinematics_[kIndexContextIndex].sensor_frame_id =
      robot_->FrameId(std::string(plato_robot_system::task::kThumbIndexFrameA));
    contact_kinematics_[kIndexContextIndex].normal_axis_sign =
      NormalizedAxisSign(config_.tactile.index_normal_axis_sign);
  } catch (const std::exception &) {
    return false;
  }

  return mppi_core::IsValidContactKinematicsContext(contact_kinematics_[kThumbContextIndex]) &&
         mppi_core::IsValidContactKinematicsContext(contact_kinematics_[kIndexContextIndex]);
}

bool RobustGraspMpcState::BuildObservation(mppi_core::GraspObservation * observation) const
{
  if (observation == nullptr || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }
  const auto & state = robot_->state();
  if (!HasCompatibleRobotState(*robot_, state)) {
    return false;
  }

  TactileStateVector tactile_meas = ConvertTactileSensors(state.tactile_sensors);
  std::vector<mppi_core::TactileSensorContext> tactile_contexts;
  if (!BuildTactileContexts(tactile_meas, &tactile_contexts)) {
    return false;
  }

  observation->q_meas = state.q;
  observation->qdot_meas = state.qdot;
  observation->tau_meas = state.tau;
  observation->q_ref_current = state.q;
  observation->qdot_ref_current = state.qdot;
  if (CanUseLastCommandReference(state)) {
    observation->q_ref_current = last_command_.q_cmd;
    observation->qdot_ref_current = last_command_.qdot_cmd;
  }
  UpdateObjectBelief(state.q, tactile_meas, tactile_contexts);
  observation->tactile_meas = std::move(tactile_meas);
  observation->object_prior = config_.object_prior;
  if (has_object_belief()) {
    observation->object_belief = object_belief_;
  }
  observation->robot_system = robot_;
  observation->tactile_contexts = std::move(tactile_contexts);
  observation->tactile_transition_config =
    &config_.policy.tactile_only_transition.tactile_transition.base;
  observation->time_s = state.time_s;
  return true;
}

bool RobustGraspMpcState::UpdateObjectBelief(
  const Eigen::Ref<const Eigen::VectorXd> & q_meas,
  const TactileStateVector & tactile_meas,
  const std::vector<mppi_core::TactileSensorContext> & tactile_contexts) const
{
  object_belief_updated_this_tick_ = false;
  object_belief_contact_count_ = 0;
  object_belief_best_cost_ = std::numeric_limits<double>::infinity();

  if (!mppi_core::HasObjectPrior(config_.object_prior) ||
    !mppi_core::IsValidObjectPrior(config_.object_prior))
  {
    object_belief_ = mppi_core::VirtualObjectBelief{};
    return false;
  }

  mppi_core::ObjectPrior tracking_prior = config_.object_prior;
  Eigen::Isometry3d previous_pose_world = Eigen::Isometry3d::Identity();
  if (RepresentativeObjectBeliefPose(object_belief_, &previous_pose_world)) {
    tracking_prior.initial_pose_world = previous_pose_world;
  }

  const auto result = mppi_core::InitializeObjectBeliefFromContacts(
    tracking_prior,
    q_meas,
    tactile_meas,
    tactile_contexts,
    config_.policy.object_belief_initialization);
  object_belief_contact_count_ = result.contacts.size();
  object_belief_best_cost_ = result.best_cost;
  if (result.valid) {
    object_belief_ = result.belief;
    object_belief_updated_this_tick_ = true;
    ++object_belief_update_count_;
    return true;
  }

  if (!has_object_belief()) {
    object_belief_ = mppi_core::VirtualObjectBelief{};
  }
  return false;
}

bool RobustGraspMpcState::BuildTactileContexts(
  const TactileStateVector & tactile_meas,
  std::vector<mppi_core::TactileSensorContext> * contexts) const
{
  if (contexts == nullptr) {
    return false;
  }
  contexts->clear();
  contexts->reserve(tactile_meas.size());
  for (std::size_t i = 0; i < tactile_meas.size(); ++i) {
    const auto & tactile = tactile_meas[i];
    const auto * kinematics = KinematicsForTactile(tactile, i);
    if (kinematics == nullptr || !mppi_core::IsValidContactKinematicsContext(*kinematics)) {
      return false;
    }
    mppi_core::TactileSensorContext context;
    context.sensor_index = tactile.sensor_index;
    context.kinematics = kinematics;
    context.hemispheres = BuildHemisphereGeometry(tactile);
    contexts->push_back(std::move(context));
  }
  return true;
}

const mppi_core::PinocchioContactKinematicsContext * RobustGraspMpcState::KinematicsForTactile(
  const mppi_core::TactileState & tactile,
  const std::size_t tactile_index) const
{
  if (SameFrame(tactile.frame_name, plato_robot_system::task::kThumbIndexFrameB)) {
    return &contact_kinematics_[kThumbContextIndex];
  }
  if (SameFrame(tactile.frame_name, plato_robot_system::task::kThumbIndexFrameA)) {
    return &contact_kinematics_[kIndexContextIndex];
  }
  if (tactile.sensor_index >= 0 &&
    static_cast<std::size_t>(tactile.sensor_index) < contact_kinematics_.size())
  {
    return &contact_kinematics_[static_cast<std::size_t>(tactile.sensor_index)];
  }
  if (tactile_index < contact_kinematics_.size()) {
    return &contact_kinematics_[tactile_index];
  }
  return nullptr;
}

bool RobustGraspMpcState::CanUseLastCommandReference(
  const plato_robot_system::RobotState & state) const
{
  if (
    !last_command_.IsUsable() ||
    last_command_.q_cmd.size() != state.q.size() ||
    last_command_.qdot_cmd.size() != state.qdot.size())
  {
    return false;
  }
  const double q_tracking_error = (last_command_.q_cmd - state.q).norm();
  return std::isfinite(q_tracking_error) &&
         q_tracking_error <= config_.safety.max_reference_tracking_error_rad;
}

bool RobustGraspMpcState::ApplyCommandSafety(
  plato_robot_system::RobotCommand * command) const
{
  if (
    command == nullptr || robot_ == nullptr || !robot_->hasModel() ||
    command->q_cmd.size() != robot_->nq() ||
    command->qdot_cmd.size() != robot_->nv() ||
    command->tau_cmd.size() != robot_->nv() ||
    !command->AllFinite())
  {
    return false;
  }

  if (config_.safety.clamp_q_cmd_to_model_limits) {
    const auto & model = robot_->model();
    if (model.lowerPositionLimit.size() == command->q_cmd.size() &&
      model.upperPositionLimit.size() == command->q_cmd.size())
    {
      for (Eigen::Index i = 0; i < command->q_cmd.size(); ++i) {
        command->q_cmd[i] = ClampFiniteRange(
          command->q_cmd[i],
          model.lowerPositionLimit[i],
          model.upperPositionLimit[i]);
      }
    }
  }
  for (Eigen::Index i = 0; i < command->qdot_cmd.size(); ++i) {
    command->qdot_cmd[i] =
      ClampFiniteSymmetric(command->qdot_cmd[i], config_.safety.max_qdot_cmd_rad_s);
  }
  for (Eigen::Index i = 0; i < command->tau_cmd.size(); ++i) {
    command->tau_cmd[i] =
      ClampFiniteSymmetric(command->tau_cmd[i], config_.safety.max_tau_cmd_nm);
  }
  command->valid = command->HasValidDimensions() && command->AllFinite();
  return command->valid;
}

bool RobustGraspMpcState::PopulateHoldCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }
  const auto & state = robot_->state();
  if (!HasCompatibleRobotState(*robot_, state)) {
    return false;
  }
  if (CanUseLastCommandReference(state)) {
    command->Resize(static_cast<int>(state.q.size()), static_cast<int>(state.qdot.size()));
    command->q_cmd = last_command_.q_cmd;
    command->qdot_cmd.setZero();
    command->tau_cmd.setZero();
    command->stamp_sec = state.time_s;
    if (!ApplyCommandSafety(command)) {
      return false;
    }
    last_command_ = *command;
    return command->IsUsable();
  }
  *command = plato_robot_system::MakeZeroHoldRobotCommand(state.q, state.qdot);
  command->stamp_sec = state.time_s;
  if (!ApplyCommandSafety(command)) {
    return false;
  }
  last_command_ = *command;
  return command->IsUsable();
}

bool RobustGraspMpcState::AllContactsLost() const
{
  if (robot_ == nullptr || !robot_->hasState()) {
    return false;
  }

  const auto & tactile_sensors = robot_->state().tactile_sensors;
  for (const auto & tactile : tactile_sensors) {
    if (tactile.valid && tactile.HasActiveHemisphereContact()) {
      return false;
    }
  }
  return true;
}

void RobustGraspMpcState::UpdateExitCondition() const
{
  if (
    exit_requested_ ||
    !config_.safety.exit_on_all_contacts_lost ||
    lifecycle_.next_state_id < 0)
  {
    return;
  }

  if (!AllContactsLost()) {
    return;
  }

  std::cout << "[robust_grasp_mpc] exiting on all contacts lost" << std::endl;
  exit_requested_ = true;
}

void RobustGraspMpcState::PrintStatus(
  const double time_s,
  const mppi_core::GraspObservation & observation,
  const plato_robot_system::RobotCommand & command) const
{
  if (!ShouldPrintStatus(time_s)) {
    return;
  }
  const auto & status = policy_.status();
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4)
         << "[robust_grasp_mpc] tick=" << tick_index_
         << " ready=" << (status.ready ? "true" : "false")
         << " fallback=" << (status.used_hold_fallback ? "true" : "false")
         << " mode="
         << (status.used_continuous_qddot_mppi ? "continuous_qddot_mppi" :
             "discrete_action_selector")
         << " rollout_only=" << (config_.safety.rollout_only ? "true" : "false")
         << " candidate_count=" << status.candidate_count
         << " disturbance_count=" << status.disturbance_count
         << " horizon=" << status.horizon_steps
         << " lambda=" << status.lambda
         << " best_idx=" << status.best_candidate_index
         << " best_action=" << status.best_action_name
         << " raw_best_idx=" << status.raw_best_candidate_index
         << " raw_best_action=" << status.raw_best_action_name
         << " second_best_action=" << status.second_best_action_name
         << " second_best_score=" << status.second_best_score
         << " hold_selected=" << (status.selected_hold_by_margin ? "true" : "false")
         << " score=" << status.best_score
         << " raw_best_score=" << status.raw_best_score
         << " hold_score=" << status.hold_score
         << " hold_improvement=" << status.hold_score_improvement
         << " mean=" << status.best_mean_cost
         << " cvar=" << status.best_cvar_cost
         << " best_sample_cost=" << status.best_sample_cost
         << " weighted_cost=" << status.weighted_cost_estimate
         << " cost_min=" << status.cost_min
         << " cost_mean=" << status.cost_mean
         << " cost_max=" << status.cost_max
         << " ess=" << status.effective_sample_size
         << " qddot_norm=" << status.selected_qddot.norm()
         << " qddot_best_norm=" << status.qddot_best_first.norm()
         << " qddot_nominal_norm=" << status.qddot_nominal_first.norm()
         << " qdot_cmd_norm=" << command.qdot_cmd.norm()
         << " active_sensors=" << ActiveTactileSensorCount(observation.tactile_meas)
         << " active_hemispheres=" << ActiveHemisphereCountTotal(observation.tactile_meas)
         << " belief_valid=" << (has_object_belief() ? "true" : "false")
         << " belief_updated=" << (object_belief_updated_this_tick_ ? "true" : "false")
         << " belief_contacts=" << object_belief_contact_count_
         << " belief_updates=" << object_belief_update_count_
         << " belief_particles=" << object_belief_.particleCount()
         << " belief_best_cost=" << object_belief_best_cost_
         << " exit_requested=" << (exit_requested_ ? "true" : "false")
         << " initial_total_cost=" << status.initial_total_cost
         << " initial_object_support_cost=" << status.initial_object_support_cost
         << " initial_contact_loss_cost=" << status.initial_contact_loss_cost
         << " initial_support_cost=" << status.initial_support_cost
         << " initial_edge_cost=" << status.initial_edge_cost
         << " initial_penetration_cost=" << status.initial_penetration_cost
         << " initial_preload_cost=" << status.initial_preload_cost
         << " initial_force_low_cost=" << status.initial_force_low_cost
         << " initial_force_high_cost=" << status.initial_force_high_cost
         << " initial_balance_cost=" << status.initial_balance_cost
         << " initial_min_gap_m=" << status.initial_object_min_gap_m
         << " initial_edge_margin_m=" << status.initial_object_edge_margin_m
         << " object_samples=" << status.selected_object_sample_count
         << " object_queries=" << status.selected_object_geometry_query_count
         << " object_support_cost=" << status.selected_object_support_cost
         << " contact_loss_cost=" << status.selected_contact_loss_cost
         << " support_cost=" << status.selected_support_cost
         << " edge_cost=" << status.selected_edge_cost
         << " penetration_cost=" << status.selected_penetration_cost
         << " preload_cost=" << status.selected_preload_cost
         << " force_low_cost=" << status.selected_force_low_cost
         << " force_high_cost=" << status.selected_force_high_cost
         << " balance_cost=" << status.selected_balance_cost
         << " control_cost=" << status.selected_control_cost
         << " rate_cost=" << status.selected_rate_cost
         << " pred_hemi=" << status.selected_predicted_active_hemisphere_total
         << " meas_hemi=" << status.selected_measured_active_hemisphere_total
         << " lost_object_contacts=" << status.selected_object_contact_loss_count
         << " selected_min_gap_m=" << status.selected_object_min_gap_m
         << " object_edge_margin_m=" << status.selected_object_edge_margin_m
         << " pred_centroid=[" << status.selected_predicted_centroid_sensor_m.transpose()
         << "] meas_centroid=[" << status.selected_measured_centroid_sensor_m.transpose()
         << "] object_disturbance_speed=" << status.selected_object_linear_disturbance_speed_mps
         << " object_disturbance_omega=" << status.selected_object_angular_disturbance_speed_radps
         << " solve_ms=" << status.solve_time_ms;
  if (config_.debug.print_action_vectors) {
    stream << " qddot_cmd=" << FormatVector(status.qddot_cmd)
           << " qddot_best=" << FormatVector(status.qddot_best_first)
           << " qddot_nominal=" << FormatVector(status.qddot_nominal_first)
           << " qdot_cmd=" << FormatVector(command.qdot_cmd);
    if (command.q_cmd.size() == observation.q_meas.size()) {
      stream << " dq_cmd=" << FormatVector(command.q_cmd - observation.q_meas);
    }
  }
  std::cout << stream.str() << std::endl;
}

bool RobustGraspMpcState::ShouldPrintStatus(const double time_s) const
{
  if (!config_.debug.print_status) {
    return false;
  }
  if (
    !std::isfinite(config_.debug.print_status_interval_s) ||
    config_.debug.print_status_interval_s <= 0.0)
  {
    return true;
  }
  if (
    !std::isfinite(time_s) ||
    time_s - last_status_print_time_s_ >= config_.debug.print_status_interval_s)
  {
    last_status_print_time_s_ = time_s;
    return true;
  }
  return false;
}

}  // namespace aristo_controller::state_machines
