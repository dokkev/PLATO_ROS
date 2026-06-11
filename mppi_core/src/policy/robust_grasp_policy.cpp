// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/policy/robust_grasp_policy.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "mppi_core/robot/robot_command_builder.hpp"

namespace mppi_core {
namespace {

constexpr double kLargeCost = 1.0e30;

bool HasValue(const Eigen::VectorXd& value, const Eigen::Index size) {
  return value.size() == size && value.allFinite();
}

bool ValidObservation(const GraspObservation& observation,
                      const std::size_t action_dim) {
  return action_dim > 0 &&
         observation.q_ref_current.size() > 0 &&
         observation.q_ref_current.allFinite() &&
         observation.tau_meas.size() ==
             static_cast<Eigen::Index>(action_dim) &&
         observation.tau_meas.allFinite() &&
         observation.tactile_meas.size() ==
             observation.tactile_contexts.size();
}

RolloutContext MakeRolloutContext(const GraspObservation& observation,
                                  const GraspState& initial_state) {
  RolloutContext context;
  context.observation = &observation;
  context.initial_reference_state = &initial_state;
  context.robot_system = observation.robot_system;
  context.tactile_contexts = observation.tactile_contexts;
  context.tactile_transition_config =
      observation.tactile_transition_config;
  context.contact_force_projection_config =
      observation.contact_force_projection_config;
  context.contact_force_rollout_config =
      observation.contact_force_rollout_config;
  context.contact_force_correction_state =
      observation.contact_force_correction_state;
  return context;
}

std::size_t EnoughContactSensorCount(const GraspState& state) {
  std::size_t count = 0;
  for (const auto& tactile : state.tactile_sensors) {
    if (tactile.readyForMppiStart()) {
      ++count;
    }
  }
  return count;
}

double MeanOfWorstTail(std::vector<double> costs, const double tail_fraction) {
  if (costs.empty()) {
    return kLargeCost;
  }
  std::sort(costs.begin(), costs.end(), std::greater<double>());
  const double fraction =
      std::isfinite(tail_fraction) ? std::clamp(tail_fraction, 0.0, 1.0) : 0.25;
  const std::size_t tail_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
              std::ceil(static_cast<double>(costs.size()) * fraction)));
  const double total =
      std::accumulate(costs.begin(), costs.begin() + tail_count, 0.0);
  return total / static_cast<double>(tail_count);
}

}  // namespace

void RobustGraspPolicy::Initialize(RobustGraspPolicyConfig config) {
  if (config.rollout.action_dim == 0 ||
      config.rollout.horizon_steps == 0 ||
      !std::isfinite(config.rollout.dt) || config.rollout.dt <= 0.0 ||
      !std::isfinite(config.risk_weight) || config.risk_weight < 0.0 ||
      !std::isfinite(config.cvar_tail_fraction) ||
      config.cvar_tail_fraction < 0.0 ||
      config.cvar_tail_fraction > 1.0 ||
      !std::isfinite(config.safe_hold_score_threshold) ||
      !std::isfinite(config.min_required_score_improvement) ||
      config.min_required_score_improvement < 0.0 ||
      !std::isfinite(config.action_rate_weight) ||
      config.action_rate_weight < 0.0) {
    throw std::invalid_argument("RobustGraspPolicyConfig: invalid rollout or scoring field");
  }

  config_ = std::move(config);
  config_.disturbance_sampler.horizon_steps = config_.rollout.horizon_steps;
  config_.action_library.horizon_steps = config_.rollout.horizon_steps;
  config_.action_library.action_dim = config_.rollout.action_dim;
  config_.action_library.dt = config_.rollout.dt;
  if (config_.action_library.qddot_lower_bound.size() == 0) {
    config_.action_library.qddot_lower_bound =
        config_.rollout.action_lower_bound;
  }
  if (config_.action_library.qddot_upper_bound.size() == 0) {
    config_.action_library.qddot_upper_bound =
        config_.rollout.action_upper_bound;
  }

  disturbance_sampler_ =
      std::make_unique<GraspDisturbanceSampler>(
          config_.disturbance_sampler);
  action_library_ =
      std::make_unique<GraspActionLibrary>(config_.action_library);
  cost_ = std::make_unique<RobustGraspStateCost>(config_.cost);
  status_ = RobustGraspPolicyStatus{};
  last_selected_qddot_ =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.rollout.action_dim));
  initialized_ = true;
}

RobotCommand RobustGraspPolicy::Update(const GraspObservation& observation) {
  if (!initialized_) {
    throw std::logic_error("RobustGraspPolicy::Update: policy is not initialized");
  }
  status_ = RobustGraspPolicyStatus{};

  if (!ValidObservation(observation, config_.rollout.action_dim)) {
    if (config_.return_hold_when_not_ready) {
      status_.used_hold_fallback = true;
      return MakeHoldCommand(observation);
    }
    throw std::invalid_argument("RobustGraspPolicy::Update: invalid observation");
  }

  GraspState initial_state = MakeInitialState(observation);
  status_.ready = IsReady(initial_state);
  if (!status_.ready) {
    if (config_.return_hold_when_not_ready) {
      status_.used_hold_fallback = true;
      return MakeHoldCommand(observation);
    }
    throw std::invalid_argument("RobustGraspPolicy::Update: grasp state is not ready");
  }

  RolloutContext context = MakeRolloutContext(observation, initial_state);
  const auto candidates =
      action_library_->BuildCandidates(initial_state, context);
  const auto disturbances = disturbance_sampler_->SampleBatch();
  status_.candidate_count = candidates.size();
  status_.disturbance_count = disturbances.size();

  if (candidates.empty() || disturbances.empty()) {
    status_.used_hold_fallback = true;
    return MakeHoldCommand(observation);
  }

  double best_score = std::numeric_limits<double>::infinity();
  std::size_t best_index = 0;
  double best_mean = kLargeCost;
  double best_cvar = kLargeCost;
  double hold_score = kLargeCost;
  double hold_mean = kLargeCost;
  double hold_cvar = kLargeCost;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    double mean_cost = kLargeCost;
    double cvar_cost = kLargeCost;
    const double score = EvaluateCandidate(
        initial_state, candidates[i], disturbances, context, &mean_cost,
        &cvar_cost);
    if (i == 0U) {
      hold_score = score;
      hold_mean = mean_cost;
      hold_cvar = cvar_cost;
    }
    if (score < best_score) {
      best_score = score;
      best_index = i;
      best_mean = mean_cost;
      best_cvar = cvar_cost;
    }
  }

  bool select_hold = false;
  if (std::isfinite(hold_score)) {
    const bool hold_is_safe =
        config_.safe_hold_score_threshold >= 0.0 &&
        hold_score <= config_.safe_hold_score_threshold;
    const bool correction_not_meaningfully_better =
        hold_score - best_score < config_.min_required_score_improvement;
    select_hold = hold_is_safe || correction_not_meaningfully_better;
  }
  if (select_hold) {
    best_index = 0U;
    best_score = hold_score;
    best_mean = hold_mean;
    best_cvar = hold_cvar;
  }

  status_.best_score = best_score;
  status_.best_mean_cost = best_mean;
  status_.best_cvar_cost = best_cvar;
  status_.best_candidate_index = best_index;
  status_.selected_hold_by_margin = select_hold;
  status_.hold_score = hold_score;
  status_.hold_mean_cost = hold_mean;
  status_.hold_cvar_cost = hold_cvar;
  status_.selected_qddot = candidates[best_index].firstAction();
  last_selected_qddot_ = status_.selected_qddot;
  return MakeRobotCommandFromQddot(
      observation, status_.selected_qddot, config_.rollout.dt);
}

GraspState RobustGraspPolicy::MakeInitialState(
    const GraspObservation& observation) const {
  const Eigen::Index action_dim =
      static_cast<Eigen::Index>(config_.rollout.action_dim);
  Eigen::VectorXd qdot =
      Eigen::VectorXd::Zero(action_dim);
  if (HasValue(observation.qdot_ref_current, action_dim)) {
    qdot = observation.qdot_ref_current;
  }
  Eigen::VectorXd tau = observation.tau_meas;
  if (!HasValue(tau, action_dim)) {
    tau = Eigen::VectorXd::Zero(action_dim);
  }
  return MakeGraspState(
      MakeRobotState(observation.q_ref_current, qdot, tau,
                     observation.time_s),
      observation.tactile_meas);
}

double RobustGraspPolicy::EvaluateCandidate(
    const GraspState& initial_state,
    const ActionSequence& candidate,
    const std::vector<GraspDisturbanceSequence,
                      Eigen::aligned_allocator<GraspDisturbanceSequence>>&
        disturbances,
    const RolloutContext& context,
    double* mean_cost,
    double* cvar_cost) const {
  std::vector<double> rollout_costs;
  rollout_costs.reserve(disturbances.size());

  for (const auto& disturbance_sequence : disturbances) {
    GraspState state = initial_state;
    double total_cost = 0.0;
    bool valid = disturbance_sequence.horizonSteps() >= candidate.horizonSteps();
    for (std::size_t step = 0; valid && step < candidate.horizonSteps(); ++step) {
      const Eigen::VectorXd action = candidate.action(step);
      const auto result = StepGraspStateWithDisturbance(
          state, action, disturbance_sequence.steps[step], context,
          config_.disturbed_rollout, config_.rollout.dt);
      if (!result.valid) {
        valid = false;
        break;
      }
      const double step_cost = cost_->Evaluate(
          result.next_state, action, context);
      if (!std::isfinite(step_cost)) {
        valid = false;
        break;
      }
      total_cost = std::min(kLargeCost, total_cost + step_cost);
      state = result.next_state;
    }
    rollout_costs.push_back(valid ? total_cost : kLargeCost);
  }

  const double mean =
      rollout_costs.empty()
          ? kLargeCost
          : std::accumulate(rollout_costs.begin(), rollout_costs.end(), 0.0) /
                static_cast<double>(rollout_costs.size());
  const double cvar = MeanOfWorstTail(
      rollout_costs, config_.cvar_tail_fraction);
  if (mean_cost != nullptr) {
    *mean_cost = mean;
  }
  if (cvar_cost != nullptr) {
    *cvar_cost = cvar;
  }
  double score = mean + config_.risk_weight * cvar;
  if (config_.action_rate_weight > 0.0 &&
      last_selected_qddot_.size() == candidate.firstAction().size() &&
      last_selected_qddot_.allFinite()) {
    score += config_.action_rate_weight *
             (candidate.firstAction() - last_selected_qddot_).squaredNorm();
  }
  return score;
}

RobotCommand RobustGraspPolicy::MakeHoldCommand(
    const GraspObservation& observation) const {
  const Eigen::VectorXd q =
      observation.q_meas.size() == observation.q_ref_current.size() &&
              observation.q_meas.allFinite()
          ? observation.q_meas
          : observation.q_ref_current;
  Eigen::VectorXd qdot =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.rollout.action_dim));
  if (observation.qdot_meas.size() == qdot.size() &&
      observation.qdot_meas.allFinite()) {
    qdot = observation.qdot_meas;
  } else if (observation.qdot_ref_current.size() == qdot.size() &&
             observation.qdot_ref_current.allFinite()) {
    qdot = observation.qdot_ref_current;
  }
  RobotCommand command = MakeZeroHoldRobotCommand(q, qdot);
  command.stamp_sec = observation.time_s;
  command.valid = command.HasValidDimensions() && command.AllFinite();
  return command;
}

bool RobustGraspPolicy::IsReady(const GraspState& state) const {
  if (!state.valid) {
    return false;
  }
  if (config_.require_both_contact_for_update &&
      EnoughContactSensorCount(state) < 2U) {
    return false;
  }
  return ReadyForMppiStart(state, config_.start);
}

}  // namespace mppi_core
