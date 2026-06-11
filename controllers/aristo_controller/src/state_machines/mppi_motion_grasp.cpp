#include "aristo_controller/state_machines/mppi_motion_grasp.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

#include "mppi_core/rollout/motion_rollout_model.hpp"
#include "plato_robot_system/task/thumb_index_grasp_constants.hpp"

namespace aristo_controller::state_machines
{
namespace
{

constexpr std::size_t kIndexMcpSlot = 0;
constexpr std::size_t kIndexPipSlot = 1;
constexpr std::size_t kThumbMcpSlot = 2;
constexpr std::size_t kThumbIpSlot = 3;

bool IsFinite(const double value)
{
  return std::isfinite(value);
}

bool IsNonnegativeFinite(const double value)
{
  return IsFinite(value) && value >= 0.0;
}

double Clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

double ClampSymmetric(const double value, const double limit)
{
  return std::clamp(value, -limit, limit);
}

int DebounceTicks(const int ticks)
{
  return std::max(1, ticks);
}

bool HasValidInitiationConfig(const MPPIMotionGraspInitiationConfig & config)
{
  return config.contact_enter_debounce_ticks >= 0 &&
         config.contact_exit_debounce_ticks >= 0 &&
         IsNonnegativeFinite(config.min_contact_force_n) &&
         IsNonnegativeFinite(config.contacted_finger_hold_weight) &&
         IsNonnegativeFinite(config.moving_finger_target_weight) &&
         IsNonnegativeFinite(config.posture_weight) &&
         IsNonnegativeFinite(config.max_reference_tracking_error_rad);
}

bool HasValidSafetyConfig(const MPPIMotionGraspSafetyConfig & config)
{
  return IsNonnegativeFinite(config.max_velocity_rad_s) &&
         IsNonnegativeFinite(config.max_torque_nm) &&
         IsNonnegativeFinite(config.max_torque_rate_nm_per_s);
}

bool ExtractNormalForceN(
  const plato_robot_system::sensor::TactileState & tactile,
  double * normal_force_n)
{
  if (normal_force_n == nullptr || !tactile.valid) {
    return false;
  }
  double force_n = tactile.ActiveHemisphereNormalForceN();
  if (!IsFinite(force_n) || force_n <= 0.0) {
    if (!tactile.total_force_n.allFinite()) {
      return false;
    }
    force_n = tactile.total_force_n.z();
  }
  if (!IsFinite(force_n)) {
    return false;
  }
  *normal_force_n = std::max(0.0, force_n);
  return true;
}

int ActiveTactileSensorCount(const plato_robot_system::TactileSensorVector & tactile)
{
  int count = 0;
  for (const auto & sensor : tactile) {
    if (sensor.HasActiveHemisphereContact()) {
      ++count;
    }
  }
  return count;
}

int ActiveHemisphereCountTotal(const plato_robot_system::TactileSensorVector & tactile)
{
  int count = 0;
  for (const auto & sensor : tactile) {
    count += static_cast<int>(sensor.ActiveHemisphereCount());
  }
  return count;
}

double TotalTactileForceN(const plato_robot_system::TactileSensorVector & tactile)
{
  double total = 0.0;
  for (const auto & sensor : tactile) {
    double sensor_force = sensor.ActiveHemisphereNormalForceN();
    if ((!IsFinite(sensor_force) || sensor_force <= 0.0) &&
      sensor.total_force_n.allFinite())
    {
      sensor_force = sensor.total_force_n.norm();
    }
    if (IsFinite(sensor_force)) {
      total += std::max(0.0, sensor_force);
    }
  }
  return total;
}

}  // namespace

const char * ToString(const GraspInitiationPhase phase)
{
  switch (phase) {
    case GraspInitiationPhase::kBothClosing:
      return "both_closing";
    case GraspInitiationPhase::kThumbContactWaitIndex:
      return "thumb_contact_wait_index";
    case GraspInitiationPhase::kIndexContactWaitThumb:
      return "index_contact_wait_thumb";
    case GraspInitiationPhase::kBothContactAlignment:
      return "both_contact_alignment";
    case GraspInitiationPhase::kForceRamp:
      return "force_ramp";
    case GraspInitiationPhase::kForceTracking:
      return "force_tracking";
  }
  return "unknown";
}

MPPIMotionGraspState::MPPIMotionGraspState(
  const plato_robot_system::StateId id,
  plato_robot_system::RobotSystem * robot)
: plato_robot_system::State(id, kName),
  robot_(robot)
{
}

bool MPPIMotionGraspState::ConfigureTask(const MPPIMotionGraspStateConfig & config)
{
  configured_ = false;
  if (
    robot_ == nullptr || !robot_->hasModel() || robot_->nv() <= 0 ||
    !IsFinite(config.default_u) || !IsFinite(config.default_phi) ||
    !IsFinite(config.default_desired_force_n) ||
    !HasValidInitiationConfig(config.initiation) ||
    !HasValidSafetyConfig(config.safety))
  {
    return false;
  }

  config_ = config;
  config_.default_u = Clamp01(config_.default_u);
  config_.default_phi = Clamp01(config_.default_phi);
  config_.default_desired_force_n = std::max(0.0, config_.default_desired_force_n);
  config_.mppi.action_dim = static_cast<std::size_t>(robot_->nv());
  config_.grasp_task.force_feedback_enabled = false;
  if (config_.cost.frame_a_name.empty()) {
    config_.cost.frame_a_name = std::string(plato_robot_system::task::kThumbIndexFrameA);
  }
  if (config_.cost.frame_b_name.empty()) {
    config_.cost.frame_b_name = std::string(plato_robot_system::task::kThumbIndexFrameB);
  }

  if (!ConfigureActiveJointIndices() ||
    !grasp_task_.Configure(robot_->model(), config_.grasp_task))
  {
    return false;
  }

  try {
    auto rollout_model =
      std::make_shared<mppi_core::MotionRolloutModel>(
        static_cast<std::size_t>(robot_->nv()));
    auto cost =
      std::make_shared<mppi_core::MotionTrackingCost>(config_.cost);
    optimizer_.Initialize(config_.mppi, rollout_model, cost);
  } catch (const std::exception &) {
    return false;
  }

  input_.u = config_.default_u;
  input_.phi = config_.default_phi;
  input_.desired_force_n = config_.default_desired_force_n;
  last_command_ = plato_robot_system::RobotCommand{};
  ResetPhaseState();
  logger_.Configure(config_.logging);
  logger_.LogEvent(0, robot_->hasState() ? robot_->state().time_s : 0.0,
    "configure", "MPPIMotionGraspState configured");
  tick_index_ = 0;
  configured_ = true;
  return true;
}

void MPPIMotionGraspState::SetInput(const MPPIMotionGraspInput & input)
{
  if (IsFinite(input.u)) {
    input_.u = Clamp01(input.u);
  }
  if (IsFinite(input.phi)) {
    input_.phi = Clamp01(input.phi);
  }
  if (IsFinite(input.desired_force_n)) {
    input_.desired_force_n = std::max(0.0, input.desired_force_n);
  }
}

void MPPIMotionGraspState::OnEnter()
{
  input_.u = config_.default_u;
  input_.phi = config_.default_phi;
  input_.desired_force_n = config_.default_desired_force_n;
  optimizer_.ResetNominalActions();
  last_command_ = plato_robot_system::RobotCommand{};
  ResetPhaseState();
  tick_index_ = 0;
  logger_.LogEvent(0, robot_ != nullptr && robot_->hasState() ? robot_->state().time_s : 0.0,
    "enter", "MPPIMotionGraspState entered");
  if (configured_ && robot_ != nullptr && robot_->hasState()) {
    (void)grasp_task_.OnEnter(*robot_, robot_->state());
    last_command_ =
      plato_robot_system::MakeZeroHoldRobotCommand(robot_->state().q, robot_->state().qdot);
    last_command_.stamp_sec = robot_->state().time_s;
  }
}

void MPPIMotionGraspState::OnExit()
{
  logger_.LogEvent(
    tick_index_, robot_ != nullptr && robot_->hasState() ? robot_->state().time_s : 0.0,
    "exit", "MPPIMotionGraspState exited");
  optimizer_.ResetNominalActions();
  grasp_task_.Reset();
  last_command_ = plato_robot_system::RobotCommand{};
  ResetPhaseState();
}

bool MPPIMotionGraspState::PopulateCommand(plato_robot_system::RobotCommand * command) const
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

  try {
    auto next_command = optimizer_.Update(observation);
    if (!next_command.IsUsable() || !ApplyCommandSafety(&next_command)) {
      const bool populated = PopulateHoldCommand(command);
      const plato_robot_system::RobotCommand log_command =
        populated ? *command : plato_robot_system::RobotCommand{};
      LogTickAndEvent(
        tick_index, observation.time_s, &observation, log_command, true,
        "fallback", "MPPI update or safety failed");
      return populated;
    }
    *command = next_command;
    last_command_ = next_command;
    used_hold_fallback_ = false;
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
    const Eigen::VectorXd selected_action = optimizer_.hasLastSelectedAction() ?
      optimizer_.lastSelectedAction() :
      Eigen::VectorXd{};
    logger_.LogTick(
      BuildTickLogRecord(
        tick_index, &observation, *command, selected_action,
        nominal_total_cost, false, ToString(phase_)));
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

bool MPPIMotionGraspState::ConfigureActiveJointIndices()
{
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

bool MPPIMotionGraspState::BuildObservation(mppi_core::GraspObservation * observation) const
{
  if (observation == nullptr || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }
  const auto & state = robot_->state();
  if (
    !plato_robot_system::IsValid(state) ||
    state.q.size() != robot_->nq() ||
    state.qdot.size() != robot_->nv() ||
    state.tau.size() != robot_->nv())
  {
    return false;
  }

  Eigen::VectorXd q_target;
  Eigen::VectorXd q_weight;
  if (!BuildPhaseMotionTarget(state, &q_target, &q_weight)) {
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

  observation->q_motion_target = q_target;
  observation->qdot_motion_target = Eigen::VectorXd::Zero(robot_->nv());
  observation->q_motion_weight = q_weight;
  observation->qdot_motion_weight = Eigen::VectorXd::Ones(robot_->nv());
  observation->motion_target_valid =
    q_target.size() == state.q.size() &&
    q_target.allFinite() &&
    q_weight.size() == state.qdot.size() &&
    q_weight.allFinite();
  observation->robot_system = robot_;
  observation->time_s = state.time_s;
  return observation->motion_target_valid;
}

bool MPPIMotionGraspState::BuildPhaseMotionTarget(
  const plato_robot_system::RobotState & state,
  Eigen::VectorXd * q_target,
  Eigen::VectorXd * q_weight) const
{
  if (q_target == nullptr || q_weight == nullptr) {
    return false;
  }

  bool thumb_contact = false;
  bool index_contact = false;
  if (!UpdateContactDebounce(state, &thumb_contact, &index_contact)) {
    return false;
  }

  const MPPIMotionGraspInput input = input_;
  plato_robot_system::task::GraspTaskCommand task_command;
  task_command.u = Clamp01(input.u);
  task_command.phi = Clamp01(input.phi);
  task_command.desired_force_n = std::max(0.0, input.desired_force_n);

  Eigen::VectorXd nominal_target;
  plato_robot_system::task::GraspTaskStatus unused_status;
  if (!grasp_task_.BuildMotionTarget(
      *robot_, state, task_command, dt(), &nominal_target, &unused_status))
  {
    return false;
  }

  if (thumb_contact && index_contact) {
    TransitionToPhase(GraspInitiationPhase::kBothContactAlignment, state, nominal_target);
  } else if (phase_ == GraspInitiationPhase::kBothClosing) {
    if (thumb_contact && !index_contact) {
      TransitionToPhase(GraspInitiationPhase::kThumbContactWaitIndex, state, nominal_target);
    } else if (index_contact && !thumb_contact) {
      TransitionToPhase(GraspInitiationPhase::kIndexContactWaitThumb, state, nominal_target);
    }
  } else if (phase_ == GraspInitiationPhase::kThumbContactWaitIndex) {
    if (index_contact) {
      TransitionToPhase(GraspInitiationPhase::kBothContactAlignment, state, nominal_target);
    } else if (
      thumb_lost_counter_ >= DebounceTicks(config_.initiation.contact_exit_debounce_ticks))
    {
      TransitionToPhase(GraspInitiationPhase::kBothClosing, state, nominal_target);
    }
  } else if (phase_ == GraspInitiationPhase::kIndexContactWaitThumb) {
    if (thumb_contact) {
      TransitionToPhase(GraspInitiationPhase::kBothContactAlignment, state, nominal_target);
    } else if (
      index_lost_counter_ >= DebounceTicks(config_.initiation.contact_exit_debounce_ticks))
    {
      TransitionToPhase(GraspInitiationPhase::kBothClosing, state, nominal_target);
    }
  }

  *q_target = nominal_target;
  *q_weight = Eigen::VectorXd::Constant(robot_->nv(), config_.initiation.posture_weight);
  const auto set_index_weight = [&](const double weight) {
    (*q_weight)[active_v_indices_[kIndexMcpSlot]] = weight;
    (*q_weight)[active_v_indices_[kIndexPipSlot]] = weight;
  };
  const auto set_thumb_weight = [&](const double weight) {
    (*q_weight)[active_v_indices_[kThumbMcpSlot]] = weight;
    (*q_weight)[active_v_indices_[kThumbIpSlot]] = weight;
  };

  switch (phase_) {
    case GraspInitiationPhase::kBothClosing:
      set_index_weight(config_.initiation.moving_finger_target_weight);
      set_thumb_weight(config_.initiation.moving_finger_target_weight);
      break;
    case GraspInitiationPhase::kThumbContactWaitIndex:
      if (!thumb_latch_active_) {
        LatchThumb(state);
      }
      (*q_target)[active_q_indices_[kThumbMcpSlot]] = thumb_latched_q_[0];
      (*q_target)[active_q_indices_[kThumbIpSlot]] = thumb_latched_q_[1];
      set_thumb_weight(config_.initiation.contacted_finger_hold_weight);
      set_index_weight(config_.initiation.moving_finger_target_weight);
      break;
    case GraspInitiationPhase::kIndexContactWaitThumb:
      if (!index_latch_active_) {
        LatchIndex(state);
      }
      (*q_target)[active_q_indices_[kIndexMcpSlot]] = index_latched_q_[0];
      (*q_target)[active_q_indices_[kIndexPipSlot]] = index_latched_q_[1];
      set_index_weight(config_.initiation.contacted_finger_hold_weight);
      set_thumb_weight(config_.initiation.moving_finger_target_weight);
      break;
    case GraspInitiationPhase::kBothContactAlignment:
      if (!full_target_latch_active_) {
        LatchFullTarget(state, nominal_target);
      }
      *q_target = full_latched_q_;
      set_index_weight(config_.initiation.contacted_finger_hold_weight);
      set_thumb_weight(config_.initiation.contacted_finger_hold_weight);
      break;
    case GraspInitiationPhase::kForceRamp:
    case GraspInitiationPhase::kForceTracking:
      return false;
  }

  return q_target->size() == state.q.size() && q_target->allFinite() &&
         q_weight->size() == state.qdot.size() && q_weight->allFinite();
}

bool MPPIMotionGraspState::UpdateContactDebounce(
  const plato_robot_system::RobotState & state,
  bool * thumb_contact,
  bool * index_contact) const
{
  if (thumb_contact == nullptr || index_contact == nullptr) {
    return false;
  }

  const bool raw_thumb_contact = HasEnoughThumbContact(state);
  const bool raw_index_contact = HasEnoughIndexContact(state);
  if (raw_thumb_contact) {
    ++thumb_contact_counter_;
    thumb_lost_counter_ = 0;
  } else {
    thumb_contact_counter_ = 0;
    ++thumb_lost_counter_;
  }
  if (raw_index_contact) {
    ++index_contact_counter_;
    index_lost_counter_ = 0;
  } else {
    index_contact_counter_ = 0;
    ++index_lost_counter_;
  }

  *thumb_contact =
    thumb_contact_counter_ >= DebounceTicks(config_.initiation.contact_enter_debounce_ticks);
  *index_contact =
    index_contact_counter_ >= DebounceTicks(config_.initiation.contact_enter_debounce_ticks);
  return true;
}

bool MPPIMotionGraspState::HasEnoughThumbContact(
  const plato_robot_system::RobotState & state) const
{
  return HasEnoughContactForFrame(
    state,
    std::string(plato_robot_system::task::kThumbIndexFrameB));
}

bool MPPIMotionGraspState::HasEnoughIndexContact(
  const plato_robot_system::RobotState & state) const
{
  return HasEnoughContactForFrame(
    state,
    std::string(plato_robot_system::task::kThumbIndexFrameA));
}

bool MPPIMotionGraspState::HasEnoughContactForFrame(
  const plato_robot_system::RobotState & state,
  const std::string & frame_name) const
{
  for (const auto & tactile : state.tactile_sensors) {
    if (tactile.frame_name != frame_name) {
      continue;
    }
    if (config_.initiation.use_tactile_presence_for_contact) {
      return tactile.valid && tactile.HasEnoughContact();
    }
    double normal_force_n = 0.0;
    return ExtractNormalForceN(tactile, &normal_force_n) &&
           normal_force_n >= config_.initiation.min_contact_force_n;
  }
  return false;
}

void MPPIMotionGraspState::TransitionToPhase(
  const GraspInitiationPhase phase,
  const plato_robot_system::RobotState & state,
  const Eigen::VectorXd & current_target) const
{
  if (phase_ == phase) {
    return;
  }

  std::cout << "[mppi_motion_grasp] phase " << ToString(phase_)
            << " -> " << ToString(phase)
            << " t=" << state.time_s << std::endl;
  const std::string detail =
    std::string(ToString(phase_)) + " -> " + std::string(ToString(phase));
  logger_.LogEvent(
    tick_index_ == 0 ? 0 : tick_index_ - 1,
    state.time_s,
    "phase_transition",
    detail);
  phase_ = phase;
  optimizer_.ResetNominalActions();
  if (phase == GraspInitiationPhase::kBothClosing) {
    thumb_latch_active_ = false;
    index_latch_active_ = false;
    full_target_latch_active_ = false;
    full_latched_q_.resize(0);
  } else if (phase == GraspInitiationPhase::kThumbContactWaitIndex) {
    LatchThumb(state);
  } else if (phase == GraspInitiationPhase::kIndexContactWaitThumb) {
    LatchIndex(state);
  } else if (phase == GraspInitiationPhase::kBothContactAlignment) {
    LatchFullTarget(state, current_target);
  }
}

void MPPIMotionGraspState::LatchThumb(
  const plato_robot_system::RobotState & state) const
{
  const Eigen::VectorXd reference = LatchReference(state);
  if (reference.size() != state.q.size()) {
    return;
  }
  thumb_latched_q_[0] = reference[active_q_indices_[kThumbMcpSlot]];
  thumb_latched_q_[1] = reference[active_q_indices_[kThumbIpSlot]];
  thumb_latch_active_ = true;
}

void MPPIMotionGraspState::LatchIndex(
  const plato_robot_system::RobotState & state) const
{
  const Eigen::VectorXd reference = LatchReference(state);
  if (reference.size() != state.q.size()) {
    return;
  }
  index_latched_q_[0] = reference[active_q_indices_[kIndexMcpSlot]];
  index_latched_q_[1] = reference[active_q_indices_[kIndexPipSlot]];
  index_latch_active_ = true;
}

void MPPIMotionGraspState::LatchFullTarget(
  const plato_robot_system::RobotState & state,
  const Eigen::VectorXd & current_target) const
{
  if (current_target.size() == state.q.size() && current_target.allFinite()) {
    full_latched_q_ = current_target;
  } else {
    full_latched_q_ = LatchReference(state);
  }
  full_target_latch_active_ =
    full_latched_q_.size() == state.q.size() && full_latched_q_.allFinite();
}

Eigen::VectorXd MPPIMotionGraspState::LatchReference(
  const plato_robot_system::RobotState & state) const
{
  if (CanUseLastCommandReference(state)) {
    return last_command_.q_cmd;
  }
  return state.q;
}

bool MPPIMotionGraspState::CanUseLastCommandReference(
  const plato_robot_system::RobotState & state) const
{
  reference_tracking_error_rad_ = 0.0;
  if (
    !last_command_.IsUsable() ||
    last_command_.q_cmd.size() != state.q.size() ||
    last_command_.qdot_cmd.size() != state.qdot.size())
  {
    return false;
  }
  reference_tracking_error_rad_ = (last_command_.q_cmd - state.q).norm();
  return IsFinite(reference_tracking_error_rad_) &&
         reference_tracking_error_rad_ <=
         config_.initiation.max_reference_tracking_error_rad;
}

bool MPPIMotionGraspState::ApplyCommandSafety(
  plato_robot_system::RobotCommand * command) const
{
  if (
    command == nullptr || command->q_cmd.size() != robot_->nq() ||
    command->qdot_cmd.size() != robot_->nv() ||
    command->tau_cmd.size() != robot_->nv() || !command->AllFinite())
  {
    return false;
  }

  for (Eigen::Index i = 0; i < command->qdot_cmd.size(); ++i) {
    command->qdot_cmd[i] =
      ClampSymmetric(command->qdot_cmd[i], config_.safety.max_velocity_rad_s);
  }
  for (Eigen::Index i = 0; i < command->tau_cmd.size(); ++i) {
    command->tau_cmd[i] =
      ClampSymmetric(command->tau_cmd[i], config_.safety.max_torque_nm);
  }

  if (
    last_command_.IsUsable() &&
    last_command_.tau_cmd.size() == command->tau_cmd.size())
  {
    double dt_sec = config_.mppi.dt;
    if (IsFinite(command->stamp_sec) && IsFinite(last_command_.stamp_sec)) {
      const double measured_dt = command->stamp_sec - last_command_.stamp_sec;
      if (measured_dt > 0.0) {
        dt_sec = measured_dt;
      }
    }
    const double max_tau_delta = config_.safety.max_torque_rate_nm_per_s * dt_sec;
    if (IsNonnegativeFinite(max_tau_delta)) {
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

bool MPPIMotionGraspState::PopulateHoldCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || robot_ == nullptr || !robot_->hasState()) {
    return false;
  }
  const auto & state = robot_->state();
  if (!plato_robot_system::IsValid(state)) {
    return false;
  }

  if (CanUseLastCommandReference(state)) {
    command->Resize(static_cast<int>(state.q.size()), static_cast<int>(state.qdot.size()));
    command->q_cmd = last_command_.q_cmd;
    command->qdot_cmd.setZero();
    command->tau_cmd.setZero();
    command->stamp_sec = state.time_s;
  } else {
    *command = plato_robot_system::MakeZeroHoldRobotCommand(state.q, state.qdot);
    command->stamp_sec = state.time_s;
  }
  if (!ApplyCommandSafety(command)) {
    return false;
  }
  last_command_ = *command;
  used_hold_fallback_ = true;
  return command->IsUsable();
}

void MPPIMotionGraspState::ResetPhaseState() const
{
  phase_ = GraspInitiationPhase::kBothClosing;
  thumb_contact_counter_ = 0;
  index_contact_counter_ = 0;
  thumb_lost_counter_ = 0;
  index_lost_counter_ = 0;
  thumb_latch_active_ = false;
  index_latch_active_ = false;
  thumb_latched_q_.setZero();
  index_latched_q_.setZero();
  full_target_latch_active_ = false;
  full_latched_q_.resize(0);
  used_hold_fallback_ = false;
  reference_tracking_error_rad_ = 0.0;
}

mppi_core::logging::MppiTickLogRecord MPPIMotionGraspState::BuildTickLogRecord(
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
  } else if (robot_ != nullptr && robot_->hasState()) {
    const auto & state = robot_->state();
    record.time_s = state.time_s;
    record.q_meas = state.q;
    record.qdot_meas = state.qdot;
    record.tau_meas = state.tau;
  }

  if (robot_ != nullptr && robot_->hasState()) {
    const auto & tactile = robot_->state().tactile_sensors;
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

void MPPIMotionGraspState::LogTickAndEvent(
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
      optimizer_.lastNominalTotalCost(), used_fallback, ToString(phase_)));
  logger_.LogEvent(tick_index, time_s, event, detail);
}

bool MPPIMotionGraspState::ShouldLogRollout(const uint64_t tick_index) const
{
  const int stride = std::max(1, config_.logging.rollout_log_stride);
  return tick_index % static_cast<uint64_t>(stride) == 0;
}

}  // namespace aristo_controller::state_machines
