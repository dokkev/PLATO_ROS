// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/core/mppi_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <stdexcept>
#include <vector>

namespace mppi_core {
namespace {

constexpr double kLargeCost = 1.0e30;
constexpr double kDefaultCommandKp = 20.0;
constexpr double kDefaultCommandKd = 1.0;

Eigen::VectorXd DefaultVector(std::size_t dim, double value) {
  return Eigen::VectorXd::Constant(static_cast<Eigen::Index>(dim), value);
}

void PrepareConfigVectors(MPPIConfig* config) {
  const auto dim = config->action_dim;
  if (config->action_lower_bound.size() == 0) {
    config->action_lower_bound =
        DefaultVector(dim, -std::numeric_limits<double>::infinity());
  }
  if (config->action_upper_bound.size() == 0) {
    config->action_upper_bound =
        DefaultVector(dim, std::numeric_limits<double>::infinity());
  }
  if (config->action_noise_std.size() == 0) {
    config->action_noise_std = DefaultVector(dim, 1.0);
  }
  if (config->command_kp.size() == 0) {
    config->command_kp = DefaultVector(dim, kDefaultCommandKp);
  }
  if (config->command_kd.size() == 0) {
    config->command_kd = DefaultVector(dim, kDefaultCommandKd);
  }
}

void CheckVectorDim(const Eigen::VectorXd& value, std::size_t dim,
                    const char* name) {
  if (value.size() != static_cast<Eigen::Index>(dim)) {
    throw std::invalid_argument(std::string("MPPIConfig: ") + name +
                                " dimension mismatch");
  }
}

bool IsFiniteOrInfinite(const Eigen::VectorXd& value) {
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (std::isnan(value[i])) {
      return false;
    }
  }
  return true;
}

bool IsFiniteAndNonnegative(const Eigen::VectorXd& value) {
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (!std::isfinite(value[i]) || value[i] < 0.0) {
      return false;
    }
  }
  return true;
}

bool HasPinocchioModelData(const RobotSystem* robot_system) {
  return robot_system != nullptr && robot_system->hasModel() &&
         robot_system->data().oMi.size() == robot_system->model().joints.size();
}

bool HasPinocchioConfigurationSpace(const GraspObservation& observation,
                                    std::size_t action_dim) {
  const RobotSystem* robot_system = observation.robot_system;
  return HasPinocchioModelData(robot_system) &&
         observation.q_ref_current.size() ==
             static_cast<Eigen::Index>(robot_system->nq()) &&
         static_cast<Eigen::Index>(action_dim) ==
             static_cast<Eigen::Index>(robot_system->nv());
}

struct RobotStateView {
  const Eigen::VectorXd* q{nullptr};
  const Eigen::VectorXd* qdot{nullptr};
};

RobotStateView SelectCurrentStateForRnea(const GraspObservation& observation,
                                         std::size_t action_dim) {
  RobotSystem* robot_system = observation.robot_system;
  if (!HasPinocchioModelData(robot_system) ||
      static_cast<Eigen::Index>(action_dim) != robot_system->nv()) {
    return {};
  }

  if (robot_system->hasState()) {
    const RobotState& state = robot_system->state();
    if (IsValid(state) &&
        state.q.size() == static_cast<Eigen::Index>(robot_system->nq()) &&
        state.qdot.size() == static_cast<Eigen::Index>(robot_system->nv())) {
      return RobotStateView{&state.q, &state.qdot};
    }
  }

  if (observation.q_meas.size() ==
          static_cast<Eigen::Index>(robot_system->nq()) &&
      observation.qdot_meas.size() ==
          static_cast<Eigen::Index>(robot_system->nv()) &&
      observation.q_meas.allFinite() && observation.qdot_meas.allFinite()) {
    return RobotStateView{&observation.q_meas, &observation.qdot_meas};
  }

  return {};
}

bool HasCompatibleReferenceConfiguration(const GraspObservation& observation,
                                         std::size_t action_dim) {
  return observation.q_ref_current.size() ==
             static_cast<Eigen::Index>(action_dim) ||
         HasPinocchioConfigurationSpace(observation, action_dim);
}

bool AllRolloutsInvalid(const std::vector<double>& rollout_costs) {
  return !rollout_costs.empty() &&
         std::all_of(rollout_costs.begin(), rollout_costs.end(),
                     [](double cost) { return cost >= kLargeCost; });
}

Eigen::VectorXd ReferenceVelocityOrZero(const GraspObservation& observation,
                                        std::size_t action_dim) {
  if (observation.qdot_ref_current.size() ==
      static_cast<Eigen::Index>(action_dim)) {
    if (!observation.qdot_ref_current.allFinite()) {
      throw std::invalid_argument(
          "MPPIOptimizer: qdot_ref_current must be finite when provided");
    }
    return observation.qdot_ref_current;
  }
  return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(action_dim));
}

Eigen::VectorXd IntegrateReferenceStep(
    const GraspObservation& observation,
    const Eigen::Ref<const Eigen::VectorXd>& tangent_step) {
  if (HasPinocchioConfigurationSpace(
          observation, static_cast<std::size_t>(tangent_step.size()))) {
    const RobotSystem* robot_system = observation.robot_system;
    return pinocchio::integrate(robot_system->model(),
                                observation.q_ref_current, tangent_step);
  }
  if (observation.q_ref_current.size() == tangent_step.size()) {
    return observation.q_ref_current + tangent_step;
  }
  throw std::invalid_argument(
      "MPPIOptimizer::MakeCommand: q_ref_current dimension mismatch");
}

Eigen::VectorXd CommandFeedForwardTorqueOrZero(
    const GraspObservation& observation, const Eigen::VectorXd& qddot_sol) {
  Eigen::VectorXd tau_ff_cmd = Eigen::VectorXd::Zero(qddot_sol.size());
  RobotSystem* robot_system = observation.robot_system;
  const RobotStateView current_state = SelectCurrentStateForRnea(
      observation, static_cast<std::size_t>(qddot_sol.size()));
  if (robot_system == nullptr || current_state.q == nullptr ||
      current_state.qdot == nullptr) {
    return tau_ff_cmd;
  }

  const Eigen::VectorXd rnea =
      pinocchio::rnea(robot_system->model(), robot_system->data(),
                      *current_state.q, *current_state.qdot, qddot_sol);
  if (rnea.size() == tau_ff_cmd.size() && rnea.allFinite()) {
    tau_ff_cmd = rnea;
  }
  return tau_ff_cmd;
}

RolloutContext MakeRolloutContext(const GraspObservation& observation,
                                  const GraspState& initial_reference_state) {
  RolloutContext rollout_context;
  rollout_context.observation = &observation;
  rollout_context.initial_reference_state = &initial_reference_state;
  rollout_context.robot_system = observation.robot_system;
  rollout_context.tactile_contexts = observation.tactile_contexts;
  rollout_context.tactile_transition_config =
      observation.tactile_transition_config;
  rollout_context.grasp_rollout_config = observation.grasp_rollout_config;
  rollout_context.contact_force_projection_config =
      observation.contact_force_projection_config;
  rollout_context.contact_force_rollout_config =
      observation.contact_force_rollout_config;
  rollout_context.contact_force_correction_state =
      observation.contact_force_correction_state;
  return rollout_context;
}

void CheckConfig(const MPPIConfig& config) {
  if (config.horizon_steps == 0) {
    throw std::invalid_argument("MPPIConfig: horizon_steps must be nonzero");
  }
  if (config.num_rollouts == 0) {
    throw std::invalid_argument("MPPIConfig: num_rollouts must be nonzero");
  }
  if (config.action_dim == 0) {
    throw std::invalid_argument("MPPIConfig: action_dim must be nonzero");
  }
  if (!std::isfinite(config.dt) || config.dt <= 0.0) {
    throw std::invalid_argument("MPPIConfig: dt must be finite and positive");
  }
  if (!std::isfinite(config.temperature) || config.temperature <= 0.0) {
    throw std::invalid_argument(
        "MPPIConfig: temperature must be finite and positive");
  }
  CheckVectorDim(config.action_lower_bound, config.action_dim,
                 "action_lower_bound");
  CheckVectorDim(config.action_upper_bound, config.action_dim,
                 "action_upper_bound");
  CheckVectorDim(config.action_noise_std, config.action_dim,
                 "action_noise_std");
  CheckVectorDim(config.command_kp, config.action_dim, "command_kp");
  CheckVectorDim(config.command_kd, config.action_dim, "command_kd");

  if (!IsFiniteOrInfinite(config.action_lower_bound) ||
      !IsFiniteOrInfinite(config.action_upper_bound)) {
    throw std::invalid_argument("MPPIConfig: action bounds cannot contain NaN");
  }
  if (!IsFiniteAndNonnegative(config.action_noise_std)) {
    throw std::invalid_argument(
        "MPPIConfig: action_noise_std must be finite and nonnegative");
  }
  if (!IsFiniteAndNonnegative(config.command_kp) ||
      !IsFiniteAndNonnegative(config.command_kd)) {
    throw std::invalid_argument(
        "MPPIConfig: command gains must be finite and nonnegative");
  }
  for (Eigen::Index i = 0; i < config.action_lower_bound.size(); ++i) {
    if (config.action_lower_bound[i] > config.action_upper_bound[i]) {
      throw std::invalid_argument(
          "MPPIConfig: action_lower_bound must be <= action_upper_bound");
    }
  }
}

double SanitizeCost(double cost) {
  if (!std::isfinite(cost)) {
    return kLargeCost;
  }
  return std::min(cost, kLargeCost);
}

}  // namespace

void MPPIOptimizer::Initialize(MPPIConfig config,
                               std::shared_ptr<const RolloutModelBase> model,
                               std::shared_ptr<const CostTermBase> cost_term) {
  if (!model) {
    throw std::invalid_argument("MPPIOptimizer::Initialize: model is null");
  }

  PrepareConfigVectors(&config);
  CheckConfig(config);
  if (model->actionDim() != config.action_dim) {
    throw std::invalid_argument(
        "MPPIOptimizer::Initialize: model action dimension mismatch");
  }

  config_ = std::move(config);
  model_ = std::move(model);
  cost_term_ = std::move(cost_term);
  rng_.seed(config_.random_seed);

  nominal_actions_.Resize(config_.action_dim, config_.horizon_steps);
  sampled_actions_.assign(
      config_.num_rollouts,
      ActionSequence(config_.action_dim, config_.horizon_steps));
  rollout_costs_.assign(config_.num_rollouts, 0.0);
  initialized_ = true;
}

RobotCommand MPPIOptimizer::Update(const GraspObservation& observation) {
  if (!initialized_) {
    throw std::logic_error(
        "MPPIOptimizer::Update: optimizer is not initialized");
  }

  if (!cost_term_) {
    RobotCommand command =
        MakeCommand(observation, nominal_actions_.firstAction());
    ShiftNominalTrajectory();
    return command;
  }

  // MPPIOptimizer stays rollout-model agnostic. Contact-gated grasp-state
  // controllers must reject no-contact observations before calling Update().
  // If a model still marks every sampled rollout invalid, return a hold
  // command.
  SampleActionSequences();
  for (std::size_t i = 0; i < sampled_actions_.size(); ++i) {
    rollout_costs_[i] =
        SanitizeCost(EvaluateRollout(observation, sampled_actions_[i]));
  }
  if (AllRolloutsInvalid(rollout_costs_)) {
    ResetNominalActions();
    return MakeCommand(
        observation,
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim)));
  }
  UpdateNominalActionSequence();

  RobotCommand command =
      MakeCommand(observation, nominal_actions_.firstAction());
  ShiftNominalTrajectory();
  return command;
}

RolloutTrace MPPIOptimizer::PredictRollout(
    const GraspObservation& observation, const ActionSequence& actions) const {
  if (!initialized_) {
    throw std::logic_error(
        "MPPIOptimizer::PredictRollout: optimizer is not initialized");
  }
  if (actions.actionDim() != config_.action_dim) {
    throw std::invalid_argument(
        "MPPIOptimizer::PredictRollout: action dimension mismatch");
  }
  if (actions.horizonSteps() == 0) {
    throw std::invalid_argument(
        "MPPIOptimizer::PredictRollout: action horizon is zero");
  }
  if (!HasCompatibleReferenceConfiguration(observation, config_.action_dim) ||
      !observation.q_ref_current.allFinite()) {
    throw std::invalid_argument(
        "MPPIOptimizer::PredictRollout: q_ref_current dimension mismatch or "
        "nonfinite");
  }
  if (observation.tau_meas.size() !=
          static_cast<Eigen::Index>(config_.action_dim) ||
      !observation.tau_meas.allFinite()) {
    throw std::invalid_argument(
        "MPPIOptimizer::PredictRollout: tau_meas dimension mismatch or "
        "nonfinite");
  }

  Eigen::VectorXd dq_ref_current =
      ReferenceVelocityOrZero(observation, config_.action_dim);
  GraspState state = MakeGraspState(
      observation.q_ref_current, dq_ref_current,
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim)),
      observation.tactile_meas);

  const GraspState initial_reference_state = state;
  RolloutContext rollout_context =
      MakeRolloutContext(observation, initial_reference_state);

  RolloutTrace trace;
  trace.states.reserve(actions.horizonSteps() + 1);
  trace.actions.reserve(actions.horizonSteps());
  trace.step_costs.reserve(actions.horizonSteps());
  trace.states.push_back(state);

  GraspState next_state = state;
  for (std::size_t step = 0; step < actions.horizonSteps(); ++step) {
    const auto action = actions.values().col(static_cast<Eigen::Index>(step));
    model_->Step(state, action, rollout_context, config_.dt, &next_state);

    CostContext cost_context;
    cost_context.rollout = &rollout_context;
    cost_context.step_index = step;
    cost_context.time_s = observation.time_s + config_.dt * step;

    double step_cost = 0.0;
    if (!next_state.valid) {
      step_cost = kLargeCost;
    } else if (cost_term_) {
      step_cost =
          SanitizeCost(cost_term_->Evaluate(next_state, action, cost_context));
    }
    trace.total_cost = SanitizeCost(trace.total_cost + step_cost);
    trace.actions.emplace_back(action);
    trace.step_costs.push_back(step_cost);
    trace.states.push_back(next_state);
    if (!next_state.valid) {
      break;
    }
    state = next_state;
  }

  return trace;
}

RolloutTrace MPPIOptimizer::PredictNominalRollout(
    const GraspObservation& observation) const {
  return PredictRollout(observation, nominal_actions_);
}

void MPPIOptimizer::ResetNominalActions() { nominal_actions_.SetZero(); }

void MPPIOptimizer::ShiftNominalTrajectory() {
  nominal_actions_.ShiftAndRepeatLast();
}

void MPPIOptimizer::SampleActionSequences() {
  std::normal_distribution<double> normal(0.0, 1.0);

  for (std::size_t rollout = 0; rollout < sampled_actions_.size(); ++rollout) {
    auto& sampled = sampled_actions_[rollout].values();
    sampled = nominal_actions_.values();

    if (rollout == 0) {
      continue;
    }

    for (Eigen::Index step = 0; step < sampled.cols(); ++step) {
      auto action = sampled.col(step);
      for (Eigen::Index dim = 0; dim < action.size(); ++dim) {
        action[dim] +=
            normal(rng_) * config_.action_noise_std[static_cast<int>(dim)];
      }
      for (Eigen::Index dim = 0; dim < action.size(); ++dim) {
        action[dim] = std::min(
            action[dim], config_.action_upper_bound[static_cast<int>(dim)]);
        action[dim] = std::max(
            action[dim], config_.action_lower_bound[static_cast<int>(dim)]);
      }
    }
  }
}

double MPPIOptimizer::EvaluateRollout(const GraspObservation& observation,
                                      const ActionSequence& actions) const {
  if (!HasCompatibleReferenceConfiguration(observation, config_.action_dim) ||
      !observation.q_ref_current.allFinite()) {
    throw std::invalid_argument(
        "MPPIOptimizer::EvaluateRollout: q_ref_current dimension mismatch or "
        "nonfinite");
  }
  if (observation.tau_meas.size() !=
          static_cast<Eigen::Index>(config_.action_dim) ||
      !observation.tau_meas.allFinite()) {
    throw std::invalid_argument(
        "MPPIOptimizer::EvaluateRollout: tau_meas dimension mismatch or "
        "nonfinite");
  }

  Eigen::VectorXd dq_ref_current =
      ReferenceVelocityOrZero(observation, config_.action_dim);
  GraspState state = MakeGraspState(
      observation.q_ref_current, dq_ref_current,
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim)),
      observation.tactile_meas);

  const GraspState initial_reference_state = state;
  GraspState next_state = state;
  RolloutContext rollout_context =
      MakeRolloutContext(observation, initial_reference_state);

  double cost = 0.0;
  for (std::size_t step = 0; step < actions.horizonSteps(); ++step) {
    const auto action = actions.values().col(static_cast<Eigen::Index>(step));
    model_->Step(state, action, rollout_context, config_.dt, &next_state);

    CostContext cost_context;
    cost_context.rollout = &rollout_context;
    cost_context.step_index = step;
    cost_context.time_s = observation.time_s + config_.dt * step;

    if (!next_state.valid) {
      return kLargeCost;
    }
    cost += cost_term_->Evaluate(next_state, action, cost_context);
    if (!std::isfinite(cost)) {
      return kLargeCost;
    }
    state = next_state;
  }
  return cost;
}

RobotCommand MPPIOptimizer::MakeCommand(
    const GraspObservation& observation,
    const Eigen::VectorXd& qddot_sol) const {
  RobotCommand command;
  if (qddot_sol.size() != static_cast<Eigen::Index>(config_.action_dim) ||
      !qddot_sol.allFinite()) {
    throw std::invalid_argument(
        "MPPIOptimizer::MakeCommand: qddot_sol dimension mismatch or "
        "nonfinite");
  }
  if (!HasCompatibleReferenceConfiguration(observation, config_.action_dim) ||
      !observation.q_ref_current.allFinite()) {
    throw std::invalid_argument(
        "MPPIOptimizer::MakeCommand: q_ref_current dimension mismatch or "
        "nonfinite");
  }
  if (observation.tau_meas.size() !=
          static_cast<Eigen::Index>(config_.action_dim) ||
      !observation.tau_meas.allFinite()) {
    throw std::invalid_argument(
        "MPPIOptimizer::MakeCommand: tau_meas dimension mismatch or nonfinite");
  }

  command.Resize(static_cast<int>(observation.q_ref_current.size()),
                 static_cast<int>(config_.action_dim));
  command.stamp_sec = observation.time_s;
  const Eigen::VectorXd qdot_ref_current =
      ReferenceVelocityOrZero(observation, config_.action_dim);
  command.qdot_cmd = qdot_ref_current + qddot_sol * config_.dt;
  command.q_cmd =
      IntegrateReferenceStep(observation, command.qdot_cmd * config_.dt);
  if (!command.q_cmd.allFinite()) {
    throw std::invalid_argument("MPPIOptimizer::MakeCommand: q_cmd nonfinite");
  }
  const Eigen::VectorXd tau_ff_cmd =
      CommandFeedForwardTorqueOrZero(observation, qddot_sol);
  command.tau_cmd = tau_ff_cmd;
  command.kp = config_.command_kp;
  command.kd = config_.command_kd;
  command.valid = command.HasValidDimensions() && command.AllFinite();
  return command;
}

void MPPIOptimizer::UpdateNominalActionSequence() {
  const auto min_it =
      std::min_element(rollout_costs_.begin(), rollout_costs_.end());
  const double min_cost = *min_it;

  Eigen::MatrixXd weighted_actions = Eigen::MatrixXd::Zero(
      nominal_actions_.values().rows(), nominal_actions_.values().cols());
  double weight_sum = 0.0;

  for (std::size_t i = 0; i < sampled_actions_.size(); ++i) {
    const double weight =
        std::exp(-(rollout_costs_[i] - min_cost) / config_.temperature);
    weighted_actions += weight * sampled_actions_[i].values();
    weight_sum += weight;
  }

  if (weight_sum > 0.0) {
    nominal_actions_.values() = weighted_actions / weight_sum;
  }
}

}  // namespace mppi_core
