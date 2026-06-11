#include "aristo_controller/state_machines/mppi_grasp.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <memory>
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
  if (!ConfigureContactKinematics()) {
    return false;
  }

  config_ = config;
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
  configured_ = true;
  return true;
}

void MPPIGraspState::OnEnter()
{
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

bool MPPIGraspState::PopulateCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || !configured_ || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }

  mppi_core::GraspObservation observation;
  if (!BuildObservation(&observation)) {
    return PopulateHoldCommand(command);
  }

  const auto current_grasp_state = mppi_core::MakeGraspState(
    observation.q_meas,
    observation.qdot_meas,
    observation.tau_meas,
    observation.tactile_meas);
  if (!mppi_core::ReadyForMppiStart(current_grasp_state, config_.task.start)) {
    optimizer_.ResetNominalActions();
    return PopulateHoldCommand(command);
  }

  try {
    auto next_command = optimizer_.Update(observation);
    if (!next_command.IsUsable()) {
      return PopulateHoldCommand(command);
    }
    *command = next_command;
    last_command_ = next_command;
    return true;
  } catch (const std::exception &) {
    return PopulateHoldCommand(command);
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
    contact_kinematics_[kThumbContextIndex].normal_axis_sign = 1.0;

    contact_kinematics_[kIndexContextIndex].model = &robot_->model();
    contact_kinematics_[kIndexContextIndex].data = &robot_->data();
    contact_kinematics_[kIndexContextIndex].sensor_frame_id =
      robot_->FrameId(std::string(plato_robot_system::task::kThumbIndexFrameA));
    contact_kinematics_[kIndexContextIndex].normal_axis_sign = 1.0;
  } catch (const std::exception &) {
    return false;
  }

  return mppi_core::IsValidContactKinematicsContext(contact_kinematics_[kThumbContextIndex]) &&
         mppi_core::IsValidContactKinematicsContext(contact_kinematics_[kIndexContextIndex]);
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
  observation->qdot_ref_current = Eigen::VectorXd::Zero(robot_->nv());
  if (
    last_command_.IsUsable() &&
    last_command_.q_cmd.size() == state.q.size() &&
    last_command_.qdot_cmd.size() == state.qdot.size())
  {
    observation->q_ref_current = last_command_.q_cmd;
    observation->qdot_ref_current = last_command_.qdot_cmd;
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

bool MPPIGraspState::PopulateHoldCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }

  const auto & state = robot_->state();
  if (!HasCompatibleRobotState(*robot_, state)) {
    return false;
  }

  if (
    last_command_.IsUsable() &&
    last_command_.q_cmd.size() == state.q.size() &&
    last_command_.qdot_cmd.size() == state.qdot.size())
  {
    command->Resize(static_cast<int>(state.q.size()), static_cast<int>(state.qdot.size()));
    command->q_cmd = last_command_.q_cmd;
    command->qdot_cmd.setZero();
    command->tau_cmd.setZero();
    command->stamp_sec = state.time_s;
    command->valid = command->HasValidDimensions() && command->AllFinite();
    last_command_ = *command;
    return command->valid;
  }

  *command = plato_robot_system::MakeZeroHoldRobotCommand(state.q, state.qdot);
  command->stamp_sec = state.time_s;
  last_command_ = *command;
  return command->IsUsable();
}

}  // namespace aristo_controller::state_machines
