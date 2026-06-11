// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/policy/robust_grasp_policy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

#include "mppi_core/object/object_belief_initializer.hpp"
#include "mppi_core/robot/robot_command_builder.hpp"

namespace mppi_core {

struct RobustGraspPolicy::CandidateEvaluationStats {
  double object_support_cost{0.0};
  double contact_loss_cost{0.0};
  double support_cost{0.0};
  double edge_cost{0.0};
  double penetration_cost{0.0};
  double preload_cost{0.0};
  double balance_cost{0.0};
  double action_cost{0.0};
  std::size_t object_sample_count{0};
  std::size_t object_geometry_query_count{0};
  double predicted_active_hemisphere_total_sum{0.0};
  double measured_active_hemisphere_total_sum{0.0};
  double object_contact_loss_count_sum{0.0};
  double object_edge_margin_min{std::numeric_limits<double>::infinity()};
  Eigen::Vector2d predicted_centroid_sum{Eigen::Vector2d::Zero()};
  Eigen::Vector2d measured_centroid_sum{Eigen::Vector2d::Zero()};
  std::size_t predicted_centroid_count{0};
  std::size_t measured_centroid_count{0};
  double object_linear_disturbance_speed_sum{0.0};
  double object_angular_disturbance_speed_sum{0.0};
  std::size_t object_support_step_count{0};
  std::size_t disturbance_step_count{0};

  double predictedActiveHemisphereAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : predicted_active_hemisphere_total_sum /
                     static_cast<double>(object_support_step_count);
  }

  double objectSupportCostAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : object_support_cost /
                     static_cast<double>(object_support_step_count);
  }

  double contactLossCostAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : contact_loss_cost /
                     static_cast<double>(object_support_step_count);
  }

  double supportCostAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : support_cost / static_cast<double>(object_support_step_count);
  }

  double edgeCostAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : edge_cost / static_cast<double>(object_support_step_count);
  }

  double penetrationCostAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : penetration_cost /
                     static_cast<double>(object_support_step_count);
  }

  double preloadCostAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : preload_cost / static_cast<double>(object_support_step_count);
  }

  double balanceCostAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : balance_cost / static_cast<double>(object_support_step_count);
  }

  double actionCostAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : action_cost / static_cast<double>(object_support_step_count);
  }

  double measuredActiveHemisphereAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : measured_active_hemisphere_total_sum /
                     static_cast<double>(object_support_step_count);
  }

  double objectContactLossAverage() const {
    return object_support_step_count == 0U
               ? 0.0
               : object_contact_loss_count_sum /
                     static_cast<double>(object_support_step_count);
  }

  double objectEdgeMarginMin() const {
    return std::isfinite(object_edge_margin_min) ? object_edge_margin_min
                                                 : 0.0;
  }

  Eigen::Vector2d predictedCentroidAverage() const {
    return predicted_centroid_count == 0U
               ? Eigen::Vector2d::Constant(
                     std::numeric_limits<double>::quiet_NaN())
               : predicted_centroid_sum /
                     static_cast<double>(predicted_centroid_count);
  }

  Eigen::Vector2d measuredCentroidAverage() const {
    return measured_centroid_count == 0U
               ? Eigen::Vector2d::Constant(
                     std::numeric_limits<double>::quiet_NaN())
               : measured_centroid_sum /
                     static_cast<double>(measured_centroid_count);
  }

  double objectLinearDisturbanceSpeedAverage() const {
    return disturbance_step_count == 0U
               ? 0.0
               : object_linear_disturbance_speed_sum /
                     static_cast<double>(disturbance_step_count);
  }

  double objectAngularDisturbanceSpeedAverage() const {
    return disturbance_step_count == 0U
               ? 0.0
               : object_angular_disturbance_speed_sum /
                     static_cast<double>(disturbance_step_count);
  }
};

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
             observation.tactile_contexts.size() &&
         IsValidObjectPrior(observation.object_prior) &&
         IsValidVirtualObjectBelief(observation.object_belief);
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

std::string CandidateName(const ActionSequence& candidate,
                          const std::size_t index) {
  if (!candidate.name().empty()) {
    return candidate.name();
  }
  return std::string("candidate_") + std::to_string(index);
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
      config.action_rate_weight < 0.0 ||
      !std::isfinite(config.continuous_control_rate_cost_weight) ||
      config.continuous_control_rate_cost_weight < 0.0 ||
      !std::isfinite(config.continuous_smoothing_alpha) ||
      config.continuous_smoothing_alpha < 0.0 ||
      config.continuous_smoothing_alpha > 1.0) {
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
  ContinuousQddotMppiConfig continuous_config;
  continuous_config.rollout = config_.rollout;
  continuous_config.disturbance_sampler = config_.disturbance_sampler;
  continuous_config.cost = config_.cost;
  continuous_config.control_rate_cost_weight =
      config_.continuous_control_rate_cost_weight;
  continuous_config.smoothing_alpha = config_.continuous_smoothing_alpha;
  continuous_config.limits.qdot_lower_bound =
      config_.rollout.qdot_lower_bound;
  continuous_config.limits.qdot_upper_bound =
      config_.rollout.qdot_upper_bound;
  continuous_mppi_ = std::make_unique<ContinuousQddotMppiController>();
  if (config_.control_mode == RobustGraspControlMode::kContinuousQddotMppi) {
    continuous_mppi_->Initialize(std::move(continuous_config));
  }
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
  const auto solve_start = std::chrono::steady_clock::now();
  const auto stamp_solve_time = [this, solve_start]() {
    const auto elapsed = std::chrono::steady_clock::now() - solve_start;
    status_.solve_time_ms =
        std::chrono::duration<double, std::milli>(elapsed).count();
  };

  if (!ValidObservation(observation, config_.rollout.action_dim)) {
    if (config_.return_hold_when_not_ready) {
      status_.used_hold_fallback = true;
      RobotCommand command = MakeHoldCommand(observation);
      stamp_solve_time();
      return command;
    }
    throw std::invalid_argument("RobustGraspPolicy::Update: invalid observation");
  }

  GraspState initial_state = MakeInitialState(observation);
  status_.ready = IsReady(initial_state);
  if (!status_.ready) {
    if (config_.return_hold_when_not_ready) {
      status_.used_hold_fallback = true;
      RobotCommand command = MakeHoldCommand(observation);
      stamp_solve_time();
      return command;
    }
    throw std::invalid_argument("RobustGraspPolicy::Update: grasp state is not ready");
  }

  RolloutContext context = MakeRolloutContext(observation, initial_state);
  status_.control_mode = config_.control_mode;
  status_.horizon_steps = config_.rollout.horizon_steps;
  status_.lambda = config_.rollout.temperature;
  RobotCommand command =
      config_.control_mode == RobustGraspControlMode::kContinuousQddotMppi
          ? UpdateContinuousQddotMppi(observation, initial_state, context)
          : UpdateDiscreteActionSelector(observation, initial_state, context);
  stamp_solve_time();
  return command;
}

RobotCommand RobustGraspPolicy::UpdateDiscreteActionSelector(
    const GraspObservation& observation,
    const GraspState& initial_state,
    const RolloutContext& context) {
  const auto candidates =
      action_library_->BuildCandidates(initial_state, context);
  const auto disturbances = disturbance_sampler_->SampleBatch();
  status_.candidate_count = candidates.size();
  status_.disturbance_count = disturbances.size();

  if (candidates.empty() || disturbances.empty()) {
    status_.used_hold_fallback = true;
    RobotCommand command = MakeHoldCommand(observation);
    return command;
  }

  double best_score = std::numeric_limits<double>::infinity();
  std::size_t best_index = 0;
  double best_mean = kLargeCost;
  double best_cvar = kLargeCost;
  double hold_score = kLargeCost;
  double hold_mean = kLargeCost;
  double hold_cvar = kLargeCost;
  double second_best_score = std::numeric_limits<double>::infinity();
  std::size_t second_best_index = 0U;
  CandidateEvaluationStats best_stats;
  CandidateEvaluationStats hold_stats;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    double mean_cost = kLargeCost;
    double cvar_cost = kLargeCost;
    CandidateEvaluationStats candidate_stats;
    const double score = EvaluateCandidate(
        initial_state, candidates[i], disturbances, context, &mean_cost,
        &cvar_cost, &candidate_stats);
    if (i == 0U) {
      hold_score = score;
      hold_mean = mean_cost;
      hold_cvar = cvar_cost;
      hold_stats = candidate_stats;
    }
    if (score < best_score) {
      second_best_score = best_score;
      second_best_index = best_index;
      best_score = score;
      best_index = i;
      best_mean = mean_cost;
      best_cvar = cvar_cost;
      best_stats = candidate_stats;
    } else if (score < second_best_score) {
      second_best_score = score;
      second_best_index = i;
    }
  }

  const double raw_best_score = best_score;
  const double raw_best_mean = best_mean;
  const double raw_best_cvar = best_cvar;
  const std::size_t raw_best_index = best_index;
  const double hold_improvement =
      std::isfinite(hold_score) && std::isfinite(raw_best_score)
          ? hold_score - raw_best_score
          : 0.0;

  bool select_hold = false;
  if (std::isfinite(hold_score)) {
    const bool hold_is_safe =
        config_.safe_hold_score_threshold >= 0.0 &&
        hold_score <= config_.safe_hold_score_threshold;
    const bool correction_not_meaningfully_better =
        hold_improvement < config_.min_required_score_improvement;
    select_hold = hold_is_safe || correction_not_meaningfully_better;
  }
  if (select_hold) {
    best_index = 0U;
    best_score = hold_score;
    best_mean = hold_mean;
    best_cvar = hold_cvar;
    best_stats = hold_stats;
    if (raw_best_index != 0U) {
      second_best_score = raw_best_score;
      second_best_index = raw_best_index;
    }
  }

  status_.best_score = best_score;
  status_.best_mean_cost = best_mean;
  status_.best_cvar_cost = best_cvar;
  status_.best_candidate_index = best_index;
  status_.best_action_name = CandidateName(candidates[best_index], best_index);
  status_.selected_hold_by_margin = select_hold;
  status_.raw_best_score = raw_best_score;
  status_.raw_best_mean_cost = raw_best_mean;
  status_.raw_best_cvar_cost = raw_best_cvar;
  status_.raw_best_candidate_index = raw_best_index;
  status_.raw_best_action_name =
      CandidateName(candidates[raw_best_index], raw_best_index);
  status_.hold_score_improvement = hold_improvement;
  status_.hold_score = hold_score;
  status_.hold_mean_cost = hold_mean;
  status_.hold_cvar_cost = hold_cvar;
  status_.hold_action_name = CandidateName(candidates[0], 0U);
  status_.second_best_score =
      std::isfinite(second_best_score) ? second_best_score : best_score;
  status_.second_best_candidate_index = second_best_index;
  status_.second_best_action_name =
      CandidateName(candidates[second_best_index], second_best_index);
  status_.selected_object_support_cost =
      best_stats.objectSupportCostAverage();
  status_.selected_contact_loss_cost =
      best_stats.contactLossCostAverage();
  status_.selected_support_cost = best_stats.supportCostAverage();
  status_.selected_edge_cost = best_stats.edgeCostAverage();
  status_.selected_penetration_cost =
      best_stats.penetrationCostAverage();
  status_.selected_preload_cost = best_stats.preloadCostAverage();
  status_.selected_balance_cost = best_stats.balanceCostAverage();
  status_.selected_action_cost = best_stats.actionCostAverage();
  status_.selected_object_sample_count = best_stats.object_sample_count;
  status_.selected_object_geometry_query_count =
      best_stats.object_geometry_query_count;
  status_.selected_predicted_active_hemisphere_total =
      best_stats.predictedActiveHemisphereAverage();
  status_.selected_measured_active_hemisphere_total =
      best_stats.measuredActiveHemisphereAverage();
  status_.selected_object_contact_loss_count =
      best_stats.objectContactLossAverage();
  status_.selected_object_edge_margin_m =
      best_stats.objectEdgeMarginMin();
  status_.selected_predicted_centroid_sensor_m =
      best_stats.predictedCentroidAverage();
  status_.selected_measured_centroid_sensor_m =
      best_stats.measuredCentroidAverage();
  status_.selected_object_linear_disturbance_speed_mps =
      best_stats.objectLinearDisturbanceSpeedAverage();
  status_.selected_object_angular_disturbance_speed_radps =
      best_stats.objectAngularDisturbanceSpeedAverage();
  status_.selected_qddot = candidates[best_index].firstAction();
  status_.qddot_cmd = status_.selected_qddot;
  status_.qddot_nominal_first =
      candidates.empty() ? Eigen::VectorXd() : candidates[0].firstAction();
  status_.qddot_best_first = status_.selected_qddot;
  status_.selected_control_cost = status_.selected_action_cost;
  last_selected_qddot_ = status_.selected_qddot;
  RobotCommand command = MakeRobotCommandFromQddot(
      observation, status_.selected_qddot, config_.rollout.dt);
  return command;
}

RobotCommand RobustGraspPolicy::UpdateContinuousQddotMppi(
    const GraspObservation& observation,
    const GraspState& initial_state,
    const RolloutContext& context) {
  if (!continuous_mppi_) {
    throw std::logic_error(
        "RobustGraspPolicy::UpdateContinuousQddotMppi: controller is null");
  }

  RobotCommand command =
      continuous_mppi_->Update(observation, initial_state, context);
  CopyContinuousStatus(continuous_mppi_->status());
  last_selected_qddot_ = status_.selected_qddot;
  return command;
}

void RobustGraspPolicy::CopyContinuousStatus(
    const ContinuousQddotMppiStatus& continuous_status) {
  status_.used_continuous_qddot_mppi = true;
  status_.candidate_count = continuous_status.num_samples;
  status_.disturbance_count = continuous_status.num_samples;
  status_.horizon_steps = continuous_status.horizon_steps;
  status_.lambda = continuous_status.lambda;

  status_.best_score = continuous_status.weighted_cost_estimate;
  status_.best_mean_cost = continuous_status.cost_mean;
  status_.best_cvar_cost = 0.0;
  status_.best_candidate_index = continuous_status.best_sample_index;
  status_.best_action_name = "continuous_qddot_mppi_weighted";
  status_.raw_best_score = continuous_status.best_sample_cost;
  status_.raw_best_mean_cost = continuous_status.best_sample_cost;
  status_.raw_best_cvar_cost = 0.0;
  status_.raw_best_candidate_index = continuous_status.best_sample_index;
  status_.raw_best_action_name =
      std::string("mppi_sample_") +
      std::to_string(continuous_status.best_sample_index);
  status_.second_best_score = continuous_status.cost_mean;
  status_.second_best_candidate_index = continuous_status.best_sample_index;
  status_.second_best_action_name = "weighted_average";

  status_.best_sample_cost = continuous_status.best_sample_cost;
  status_.weighted_cost_estimate = continuous_status.weighted_cost_estimate;
  status_.cost_min = continuous_status.cost_min;
  status_.cost_mean = continuous_status.cost_mean;
  status_.cost_max = continuous_status.cost_max;
  status_.effective_sample_size = continuous_status.effective_sample_size;

  status_.selected_object_support_cost =
      continuous_status.selected_contact_loss_cost +
      continuous_status.selected_support_cost +
      continuous_status.selected_edge_cost +
      continuous_status.selected_penetration_cost;
  status_.selected_contact_loss_cost =
      continuous_status.selected_contact_loss_cost;
  status_.selected_support_cost = continuous_status.selected_support_cost;
  status_.selected_edge_cost = continuous_status.selected_edge_cost;
  status_.selected_penetration_cost =
      continuous_status.selected_penetration_cost;
  status_.selected_preload_cost = continuous_status.selected_preload_cost;
  status_.selected_balance_cost = continuous_status.selected_balance_cost;
  status_.selected_action_cost = continuous_status.selected_control_cost;
  status_.selected_control_cost = continuous_status.selected_control_cost;
  status_.selected_rate_cost = continuous_status.selected_rate_cost;
  status_.selected_object_sample_count =
      continuous_status.object_sample_count;
  status_.selected_object_geometry_query_count =
      continuous_status.geometry_query_count;
  status_.selected_predicted_active_hemisphere_total =
      continuous_status.predicted_active_hemisphere_total;
  status_.selected_measured_active_hemisphere_total =
      continuous_status.measured_active_hemisphere_total;
  status_.selected_object_contact_loss_count =
      continuous_status.object_contact_loss_count;
  status_.selected_object_edge_margin_m =
      continuous_status.object_edge_margin_m;
  status_.selected_predicted_centroid_sensor_m =
      continuous_status.predicted_centroid_sensor_m;
  status_.selected_measured_centroid_sensor_m =
      continuous_status.measured_centroid_sensor_m;
  status_.selected_object_linear_disturbance_speed_mps =
      continuous_status.object_linear_disturbance_speed_mps;
  status_.selected_object_angular_disturbance_speed_radps =
      continuous_status.object_angular_disturbance_speed_radps;

  status_.qddot_cmd = continuous_status.qddot_cmd;
  status_.qddot_nominal_first = continuous_status.qddot_nominal_first;
  status_.qddot_best_first = continuous_status.qddot_best_first;
  status_.selected_qddot = continuous_status.qddot_cmd;
}

GraspState RobustGraspPolicy::MakeInitialState(
    const GraspObservation& observation) const {
  const Eigen::Index action_dim =
      static_cast<Eigen::Index>(config_.rollout.action_dim);
  Eigen::VectorXd q = observation.q_ref_current;
  if (HasValue(observation.q_meas, observation.q_ref_current.size())) {
    q = observation.q_meas;
  }
  Eigen::VectorXd qdot =
      Eigen::VectorXd::Zero(action_dim);
  if (HasValue(observation.qdot_meas, action_dim)) {
    qdot = observation.qdot_meas;
  } else if (HasValue(observation.qdot_ref_current, action_dim)) {
    qdot = observation.qdot_ref_current;
  }
  Eigen::VectorXd tau = observation.tau_meas;
  if (!HasValue(tau, action_dim)) {
    tau = Eigen::VectorXd::Zero(action_dim);
  }
  return MakeGraspState(
      MakeRobotState(q, qdot, tau, observation.time_s),
      observation.tactile_meas,
      ResolveObjectBeliefForObservation(observation, q));
}

double RobustGraspPolicy::EvaluateCandidate(
    const GraspState& initial_state,
    const ActionSequence& candidate,
    const std::vector<GraspDisturbanceSequence,
                      Eigen::aligned_allocator<GraspDisturbanceSequence>>&
        disturbances,
    const RolloutContext& context,
    double* mean_cost,
    double* cvar_cost,
    CandidateEvaluationStats* stats) const {
  std::vector<double> rollout_costs;
  rollout_costs.reserve(disturbances.size());
  CandidateEvaluationStats local_stats;

  for (const auto& disturbance_sequence : disturbances) {
    GraspState state = initial_state;
    double total_cost = 0.0;
    bool valid = disturbance_sequence.horizonSteps() >= candidate.horizonSteps();
    for (std::size_t step = 0; valid && step < candidate.horizonSteps(); ++step) {
      const Eigen::VectorXd action = candidate.action(step);
      const auto& disturbance_step = disturbance_sequence.steps[step];
      local_stats.object_linear_disturbance_speed_sum +=
          disturbance_step.object_disturbance.linear_velocity_world_mps.norm();
      local_stats.object_angular_disturbance_speed_sum +=
          disturbance_step.object_disturbance.angular_velocity_world_radps.norm();
      ++local_stats.disturbance_step_count;
      const auto result = StepObjectPriorGraspState(
          state, action, disturbance_step, context, config_.rollout.dt);
      if (!result.valid) {
        valid = false;
        break;
      }
      RobustGraspStateCostBreakdown breakdown;
      const double step_cost = cost_->Evaluate(
          result.next_state, action, context, &breakdown);
      if (!std::isfinite(step_cost)) {
        valid = false;
        break;
      }
      if (breakdown.object_support.valid) {
        local_stats.object_support_cost += breakdown.object_support_cost;
        local_stats.contact_loss_cost +=
            breakdown.object_support.contact_loss_cost;
        local_stats.support_cost += breakdown.object_support.support_cost;
        local_stats.edge_cost += breakdown.object_support.edge_cost;
        local_stats.penetration_cost +=
            breakdown.object_support.penetration_cost;
        local_stats.preload_cost += breakdown.preload_cost;
        local_stats.balance_cost += breakdown.force_balance_cost;
        local_stats.action_cost += breakdown.action_cost;
        local_stats.object_sample_count +=
            breakdown.object_support.object_sample_count;
        local_stats.object_geometry_query_count +=
            breakdown.object_support.geometry_query_count;
        local_stats.predicted_active_hemisphere_total_sum +=
            breakdown.object_support.predicted_active_hemisphere_total;
        local_stats.measured_active_hemisphere_total_sum +=
            breakdown.object_support.measured_active_hemisphere_total;
        local_stats.object_contact_loss_count_sum +=
            breakdown.object_support.lost_measured_contact_count;
        local_stats.object_edge_margin_min = std::min(
            local_stats.object_edge_margin_min,
            breakdown.object_support.min_edge_margin_m);
        if (breakdown.object_support.support_summary
                .predicted_centroid_sensor_m.allFinite()) {
          local_stats.predicted_centroid_sum +=
              breakdown.object_support.support_summary
                  .predicted_centroid_sensor_m;
          ++local_stats.predicted_centroid_count;
        }
        if (breakdown.object_support.support_summary
                .measured_centroid_sensor_m.allFinite()) {
          local_stats.measured_centroid_sum +=
              breakdown.object_support.support_summary
                  .measured_centroid_sensor_m;
          ++local_stats.measured_centroid_count;
        }
        ++local_stats.object_support_step_count;
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
  if (stats != nullptr) {
    *stats = local_stats;
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
