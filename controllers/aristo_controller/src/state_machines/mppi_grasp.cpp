#include "aristo_controller/state_machines/mppi_grasp.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "mppi_core/costs/grasp_stability_cost.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "plato_robot_system/task/thumb_index_grasp_constants.hpp"

namespace aristo_controller::state_machines
{
namespace
{

constexpr std::size_t kThumbContextIndex = 0;
constexpr std::size_t kIndexContextIndex = 1;
constexpr std::size_t kIndexMcpSlot = 0;
constexpr std::size_t kIndexPipSlot = 1;
constexpr std::size_t kThumbMcpSlot = 2;
constexpr std::size_t kThumbIpSlot = 3;

using MppiTactileStateVector =
  std::vector<mppi_core::TactileState, Eigen::aligned_allocator<mppi_core::TactileState>>;

struct MaintenanceContactSummary
{
  bool thumb_active{false};
  bool index_active{false};
  double thumb_force_n{0.0};
  double index_force_n{0.0};

  double totalForceN() const
  {
    return thumb_force_n + index_force_n;
  }
};

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

bool IsNonnegativeFinite(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool IsValidNormalAxisSign(const double value)
{
  return std::isfinite(value) && value != 0.0;
}

double NormalizedAxisSign(const double value)
{
  return value < 0.0 ? -1.0 : 1.0;
}

double ClampFiniteSymmetric(const double value, const double limit)
{
  if (!std::isfinite(limit)) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

double ClampFiniteRange(const double value, const double lower, const double upper)
{
  if (std::isfinite(lower) && std::isfinite(upper) && lower <= upper) {
    return std::clamp(value, lower, upper);
  }
  return value;
}

bool HasValidSafetyConfig(const MPPIGraspSafetyConfig & config)
{
  return IsNonnegativeFinite(config.max_reference_tracking_error_rad) &&
         IsNonnegativeFinite(config.max_qdot_cmd_rad_s) &&
         IsNonnegativeFinite(config.max_tau_cmd_nm) &&
         IsNonnegativeFinite(config.max_tau_rate_nm_s);
}

bool HasValidMaintenanceConfig(const MPPIGraspMaintenanceConfig & config)
{
  return IsNonnegativeFinite(config.target_total_force_n) &&
         IsNonnegativeFinite(config.min_sensor_force_n) &&
         IsNonnegativeFinite(config.closing_qddot_rad_s2) &&
         IsNonnegativeFinite(config.one_sided_closing_qddot_rad_s2);
}

double TactileForceN(const mppi_core::TactileState & tactile)
{
  double sensor_force = tactile.activeHemisphereNormalForceN();
  if ((!std::isfinite(sensor_force) || sensor_force <= 0.0) &&
    tactile.total_force_n.allFinite())
  {
    sensor_force = tactile.total_force_n.norm();
  }
  if (!std::isfinite(sensor_force)) {
    return 0.0;
  }
  return std::max(0.0, sensor_force);
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

std::size_t EnoughContactSensorCount(const mppi_core::GraspState & state)
{
  std::size_t count = 0;
  for (const auto & tactile : state.tactile_sensors) {
    if (tactile.readyForMppiStart()) {
      ++count;
    }
  }
  return count;
}

double TotalTactileForceN(const MppiTactileStateVector & tactile)
{
  double total = 0.0;
  for (const auto & sensor : tactile) {
    total += TactileForceN(sensor);
  }
  return total;
}

std::string VectorSummary(const Eigen::VectorXd & vector, const int max_entries = 8)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << "[";
  const Eigen::Index count =
    std::min<Eigen::Index>(vector.size(), static_cast<Eigen::Index>(max_entries));
  for (Eigen::Index i = 0; i < count; ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << vector[i];
  }
  if (vector.size() > count) {
    stream << ", ...";
  }
  stream << "]";
  return stream.str();
}

}  // namespace

MPPIGraspState::MPPIGraspState(
  const plato_robot_system::StateId id,
  plato_robot_system::RobotSystem * robot)
: plato_robot_system::State(id, kName),
  robot_(robot)
{
}

bool MPPIGraspState::ConfigureTask(const MPPIGraspStateConfig & config)
{
  configured_ = false;
  if (robot_ == nullptr || !robot_->hasModel() || robot_->nv() <= 0) {
    return false;
  }
  if (
    !HasValidSafetyConfig(config.safety) ||
    !HasValidMaintenanceConfig(config.maintenance) ||
    !IsValidNormalAxisSign(config.tactile.thumb_normal_axis_sign) ||
    !IsValidNormalAxisSign(config.tactile.index_normal_axis_sign))
  {
    return false;
  }

  config_ = config;
  if (!ConfigureContactKinematics() || !ConfigureActiveJointIndices()) {
    return false;
  }

  config_.mppi.action_dim = static_cast<std::size_t>(robot_->nv());
  tactile_transition_ =
    mppi_core::ApplyTaskToleranceToTransitionConfig(config_.task, config_.tactile_transition);

  try {
    auto rollout_model = std::make_shared<mppi_core::GraspStateRolloutModel>(
      static_cast<std::size_t>(robot_->nv()),
      config_.rollout);
    auto cost = std::make_shared<mppi_core::GraspStabilityCost>(config_.task.cost);
    optimizer_.Initialize(config_.mppi, rollout_model, cost);
  } catch (const std::exception &) {
    return false;
  }

  last_command_ = plato_robot_system::RobotCommand{};
  logger_.Configure(config_.logging);
  logger_.LogEvent(0, robot_->hasState() ? robot_->state().time_s : 0.0,
    "configure", "MPPIGraspState configured");
  tick_index_ = 0;
  last_phase_ = kName;
  has_entered_mppi_ready_ = false;
  exit_requested_ = false;
  last_action_debug_print_time_s_ = -1.0e100;
  configured_ = true;
  return true;
}

void MPPIGraspState::OnEnter()
{
  tick_index_ = 0;
  last_phase_ = kName;
  has_entered_mppi_ready_ = false;
  exit_requested_ = false;
  last_action_debug_print_time_s_ = -1.0e100;
  logger_.LogEvent(0, robot_ != nullptr && robot_->hasState() ? robot_->state().time_s : 0.0,
    "enter", "MPPIGraspState entered");

  if (configured_) {
    optimizer_.ResetNominalActions();
  }

  last_command_ = plato_robot_system::RobotCommand{};
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

bool MPPIGraspState::IsFinished() const
{
  return (exit_requested_ && lifecycle_.next_state_id >= 0) ||
         plato_robot_system::State::IsFinished();
}

bool MPPIGraspState::PopulateCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || !configured_ || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }

  const uint64_t tick_index = tick_index_++;
  const double time_s = robot_->state().time_s;

  mppi_core::GraspObservation observation;
  if (!BuildObservation(&observation)) {
    const bool populated = PopulateHoldCommand(command);
    const plato_robot_system::RobotCommand log_command =
      populated ? *command : plato_robot_system::RobotCommand{};
    LogTickAndEvent(
      tick_index, time_s, nullptr, log_command, true,
      "fallback", "BuildObservation failed");
    return populated;
  }

  const auto current_grasp_state = mppi_core::MakeGraspState(
    observation.q_meas,
    observation.qdot_meas,
    observation.tau_meas,
    observation.tactile_meas);
  std::string action_debug_detail{"ok"};
  if (!mppi_core::ReadyForMppiStart(current_grasp_state, config_.task.start)) {
    if (!has_entered_mppi_ready_ || !HasContinuationContact(current_grasp_state)) {
      optimizer_.ResetNominalActions();
      RequestExitOnContactLoss(tick_index, observation.time_s, current_grasp_state);
      const bool populated = PopulateHoldCommand(command);
      const plato_robot_system::RobotCommand log_command =
        populated ? *command : plato_robot_system::RobotCommand{};
      LogTickAndEvent(
        tick_index, observation.time_s, &observation, log_command, true,
        "not_ready", "ReadyForMppiStart false");
      return populated;
    }
    action_debug_detail = "partial_contact_continue";
  }
  has_entered_mppi_ready_ = true;

  try {
    auto next_command = optimizer_.Update(observation);
    if (!next_command.IsUsable()) {
      const bool populated = PopulateHoldCommand(command);
      const plato_robot_system::RobotCommand log_command =
        populated ? *command : plato_robot_system::RobotCommand{};
      LogTickAndEvent(
        tick_index, observation.time_s, &observation, log_command, true,
        "fallback", "MPPI update returned unusable command");
      return populated;
    }
    Eigen::VectorXd applied_action = optimizer_.hasLastSelectedAction() ?
      optimizer_.lastSelectedAction() :
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(robot_->nv()));
    ApplyMaintenanceHeuristic(
      observation, current_grasp_state, &next_command, &applied_action, &action_debug_detail);
    if (!ApplyCommandSafety(&next_command)) {
      const bool populated = PopulateHoldCommand(command);
      const plato_robot_system::RobotCommand log_command =
        populated ? *command : plato_robot_system::RobotCommand{};
      LogTickAndEvent(
        tick_index, observation.time_s, &observation, log_command, true,
        "fallback", "ApplyCommandSafety failed");
      return populated;
    }
    *command = next_command;
    last_command_ = next_command;
    double nominal_total_cost = optimizer_.lastNominalTotalCost();
    if (
      logger_.enabled() &&
      optimizer_.hasLastSelectedActionSequence() &&
      ShouldLogRollout(tick_index))
    {
      try {
        const auto trace =
          optimizer_.PredictRollout(observation, optimizer_.lastSelectedActionSequence());
        nominal_total_cost = trace.total_cost;
        logger_.LogRollout(tick_index, observation.time_s, trace);
      } catch (const std::exception & error) {
        logger_.LogEvent(tick_index, observation.time_s, "invalid_rollout", error.what());
      }
    }
    logger_.LogTick(
      BuildTickLogRecord(
        tick_index, &observation, *command, applied_action,
        nominal_total_cost, false, last_phase_));
    PrintActionDebug(
      tick_index, observation.time_s, &observation, *command, applied_action,
      nominal_total_cost, false, action_debug_detail);
    return true;
  } catch (const std::exception & error) {
    const bool populated = PopulateHoldCommand(command);
    const plato_robot_system::RobotCommand log_command =
      populated ? *command : plato_robot_system::RobotCommand{};
    LogTickAndEvent(
      tick_index, observation.time_s, &observation, log_command, true,
      "fallback", error.what());
    return populated;
  }
}

bool MPPIGraspState::ConfigureContactKinematics()
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

bool MPPIGraspState::ConfigureActiveJointIndices()
{
  if (robot_ == nullptr || !robot_->hasModel()) {
    return false;
  }

  for (std::size_t i = 0; i < plato_robot_system::task::kThumbIndexActiveJoints.size(); ++i) {
    const auto joint_id =
      robot_->model().getJointId(std::string(plato_robot_system::task::kThumbIndexActiveJoints[i]));
    if (
      joint_id >= static_cast<pinocchio::JointIndex>(robot_->model().njoints) ||
      robot_->model().nqs[joint_id] != 1 ||
      robot_->model().nvs[joint_id] != 1)
    {
      return false;
    }
    active_q_indices_[i] = robot_->model().idx_qs[joint_id];
    active_v_indices_[i] = robot_->model().idx_vs[joint_id];
    if (active_q_indices_[i] < 0 || active_v_indices_[i] < 0) {
      return false;
    }
  }
  return true;
}

bool MPPIGraspState::BuildObservation(mppi_core::GraspObservation * observation) const
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
  } else {
    optimizer_.ResetNominalActions();
  }
  observation->tactile_meas = std::move(tactile_meas);
  observation->robot_system = robot_;
  observation->tactile_contexts = std::move(tactile_contexts);
  observation->tactile_transition_config = &tactile_transition_;
  observation->time_s = state.time_s;
  return true;
}

bool MPPIGraspState::BuildTactileContexts(
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

const mppi_core::PinocchioContactKinematicsContext * MPPIGraspState::KinematicsForTactile(
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

bool MPPIGraspState::CanUseLastCommandReference(
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

bool MPPIGraspState::ApplyCommandSafety(
  plato_robot_system::RobotCommand * command) const
{
  if (
    command == nullptr || robot_ == nullptr || !robot_->hasModel() ||
    command->q_cmd.size() != robot_->nq() || command->qdot_cmd.size() != robot_->nv() ||
    command->tau_cmd.size() != robot_->nv() || !command->AllFinite())
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

  if (
    last_command_.IsUsable() &&
    last_command_.tau_cmd.size() == command->tau_cmd.size() &&
    std::isfinite(config_.safety.max_tau_rate_nm_s))
  {
    double dt_sec = config_.mppi.dt;
    if (std::isfinite(command->stamp_sec) && std::isfinite(last_command_.stamp_sec)) {
      const double measured_dt = command->stamp_sec - last_command_.stamp_sec;
      if (measured_dt > 0.0) {
        dt_sec = measured_dt;
      }
    }
    const double max_tau_delta = config_.safety.max_tau_rate_nm_s * dt_sec;
    if (std::isfinite(max_tau_delta) && max_tau_delta >= 0.0) {
      for (Eigen::Index i = 0; i < command->tau_cmd.size(); ++i) {
        command->tau_cmd[i] = std::clamp(
          command->tau_cmd[i],
          last_command_.tau_cmd[i] - max_tau_delta,
          last_command_.tau_cmd[i] + max_tau_delta);
      }
    }
  }

  command->valid = command->HasValidDimensions() && command->AllFinite();
  return command->valid;
}

bool MPPIGraspState::PopulateHoldCommand(plato_robot_system::RobotCommand * command) const
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
    command->valid = command->HasValidDimensions() && command->AllFinite();
    last_command_ = *command;
    return command->valid;
  }

  *command = plato_robot_system::MakeZeroHoldRobotCommand(state.q, state.qdot);
  command->stamp_sec = state.time_s;
  if (!ApplyCommandSafety(command)) {
    return false;
  }
  last_command_ = *command;
  return command->IsUsable();
}

bool MPPIGraspState::HasContinuationContact(const mppi_core::GraspState & state) const
{
  if (!state.valid) {
    return false;
  }
  return state.activeTactileSensorCount() >=
           config_.continuation.min_active_tactile_sensors &&
         state.activeHemisphereCountTotal() >=
           config_.continuation.min_active_hemisphere_total;
}

void MPPIGraspState::RequestExitOnContactLoss(
  const uint64_t tick_index,
  const double time_s,
  const mppi_core::GraspState & state) const
{
  if (
    !config_.exit_on_contact_lost ||
    !has_entered_mppi_ready_ ||
    lifecycle_.next_state_id < 0)
  {
    return;
  }

  const std::size_t enough_contact_sensors = EnoughContactSensorCount(state);
  const std::size_t active_tactile_sensors = state.activeTactileSensorCount();
  const std::size_t active_hemispheres = state.activeHemisphereCountTotal();
  const bool contact_lost =
    active_tactile_sensors < config_.continuation.min_active_tactile_sensors ||
    active_hemispheres < config_.continuation.min_active_hemisphere_total;
  if (!contact_lost) {
    return;
  }

  if (!exit_requested_) {
    const std::string detail =
      "continuation contact lost after MPPI ready: enough_contact_sensors=" +
      std::to_string(enough_contact_sensors) +
      "/" + std::to_string(config_.task.start.min_enough_contact_sensors) +
      " active_tactile_sensors=" + std::to_string(active_tactile_sensors) +
      "/" + std::to_string(config_.continuation.min_active_tactile_sensors) +
      " active_hemispheres=" + std::to_string(active_hemispheres) +
      "/" + std::to_string(config_.continuation.min_active_hemisphere_total);
    std::cout << "[mppi_grasp] exiting on contact loss "
              << detail
              << std::endl;
    logger_.LogEvent(tick_index, time_s, "contact_lost_exit", detail);
  }
  exit_requested_ = true;
}

mppi_core::logging::MppiTickLogRecord MPPIGraspState::BuildTickLogRecord(
  const uint64_t tick_index,
  const mppi_core::GraspObservation * observation,
  const plato_robot_system::RobotCommand & command,
  const Eigen::VectorXd & selected_action,
  const double nominal_total_cost,
  const bool used_fallback,
  const std::string & phase) const
{
  mppi_core::logging::MppiTickLogRecord record;
  record.tick_index = tick_index;
  record.controller_state = kName;
  record.phase = phase;
  record.selected_action_qddot = selected_action;
  record.nominal_total_cost = nominal_total_cost;
  record.command_valid = command.valid;
  record.used_fallback = used_fallback;

  if (observation != nullptr) {
    record.time_s = observation->time_s;
    record.q_meas = observation->q_meas;
    record.qdot_meas = observation->qdot_meas;
    record.tau_meas = observation->tau_meas;
    record.q_ref_current = observation->q_ref_current;
    record.qdot_ref_current = observation->qdot_ref_current;
    record.tactile_sensor_count = static_cast<int>(observation->tactile_meas.size());
    record.active_tactile_sensor_count = ActiveTactileSensorCount(observation->tactile_meas);
    record.active_hemisphere_count_total =
      ActiveHemisphereCountTotal(observation->tactile_meas);
    record.tactile_total_force_n = TotalTactileForceN(observation->tactile_meas);
  } else if (robot_ != nullptr && robot_->hasState()) {
    const auto & state = robot_->state();
    record.time_s = state.time_s;
    record.q_meas = state.q;
    record.qdot_meas = state.qdot;
    record.tau_meas = state.tau;
    const auto tactile = ConvertTactileSensors(state.tactile_sensors);
    record.tactile_sensor_count = static_cast<int>(tactile.size());
    record.active_tactile_sensor_count = ActiveTactileSensorCount(tactile);
    record.active_hemisphere_count_total = ActiveHemisphereCountTotal(tactile);
    record.tactile_total_force_n = TotalTactileForceN(tactile);
  }

  record.q_cmd = command.q_cmd;
  record.qdot_cmd = command.qdot_cmd;
  record.tau_cmd = command.tau_cmd;
  record.kp = command.kp;
  record.kd = command.kd;
  return record;
}

void MPPIGraspState::LogTickAndEvent(
  const uint64_t tick_index,
  const double time_s,
  const mppi_core::GraspObservation * observation,
  const plato_robot_system::RobotCommand & command,
  const bool used_fallback,
  const std::string & event,
  const std::string & detail) const
{
  const Eigen::VectorXd selected_action = optimizer_.hasLastSelectedAction() ?
    optimizer_.lastSelectedAction() :
    Eigen::VectorXd{};
  logger_.LogTick(
    BuildTickLogRecord(
      tick_index, observation, command, selected_action,
      optimizer_.lastNominalTotalCost(), used_fallback, last_phase_));
  logger_.LogEvent(tick_index, time_s, event, detail);
  PrintActionDebug(
    tick_index, time_s, observation, command, selected_action,
    optimizer_.lastNominalTotalCost(), used_fallback, event + ": " + detail);
}

bool MPPIGraspState::ShouldLogRollout(const uint64_t tick_index) const
{
  const int stride = std::max(1, config_.logging.rollout_log_stride);
  return tick_index % static_cast<uint64_t>(stride) == 0;
}

void MPPIGraspState::PrintActionDebug(
  const uint64_t tick_index,
  const double time_s,
  const mppi_core::GraspObservation * observation,
  const plato_robot_system::RobotCommand & command,
  const Eigen::VectorXd & selected_action,
  const double nominal_total_cost,
  const bool used_fallback,
  const std::string & detail) const
{
  if (!ShouldPrintActionDebug(time_s)) {
    return;
  }

  double q_delta_norm = 0.0;
  if (
    observation != nullptr &&
    command.q_cmd.size() == observation->q_meas.size() &&
    command.q_cmd.size() > 0)
  {
    q_delta_norm = (command.q_cmd - observation->q_meas).norm();
  }

  int active_sensors = 0;
  int active_hemispheres = 0;
  double tactile_force_n = 0.0;
  if (observation != nullptr) {
    active_sensors = ActiveTactileSensorCount(observation->tactile_meas);
    active_hemispheres = ActiveHemisphereCountTotal(observation->tactile_meas);
    tactile_force_n = TotalTactileForceN(observation->tactile_meas);
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4)
         << "[mppi_grasp] action"
         << " tick=" << tick_index
         << " fallback=" << (used_fallback ? "true" : "false")
         << " reason=" << detail
         << " qddot_norm=" << selected_action.norm()
         << " qddot=" << VectorSummary(selected_action)
         << " q_delta_norm=" << q_delta_norm
         << " qdot_cmd_norm=" << command.qdot_cmd.norm()
         << " tau_cmd_norm=" << command.tau_cmd.norm()
         << " cost=" << nominal_total_cost
         << " active_sensors=" << active_sensors
         << " active_hemispheres=" << active_hemispheres
         << " tactile_force_n=" << tactile_force_n;
  std::cout << stream.str() << std::endl;
}

bool MPPIGraspState::ShouldPrintActionDebug(const double time_s) const
{
  if (!config_.debug.print_action) {
    return false;
  }
  if (
    !std::isfinite(config_.debug.print_action_interval_s) ||
    config_.debug.print_action_interval_s <= 0.0)
  {
    return true;
  }
  if (
    !std::isfinite(time_s) ||
    time_s - last_action_debug_print_time_s_ >= config_.debug.print_action_interval_s)
  {
    last_action_debug_print_time_s_ = time_s;
    return true;
  }
  return false;
}

}  // namespace aristo_controller::state_machines
