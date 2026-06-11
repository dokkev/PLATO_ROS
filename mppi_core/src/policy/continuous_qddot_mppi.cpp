// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/policy/continuous_qddot_mppi.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

#include "mppi_core/rollout/object_prior_grasp_rollout.hpp"

namespace mppi_core {
namespace {

constexpr double kLargeCost = 1.0e30;
constexpr double kTiny = 1.0e-12;

Eigen::VectorXd DefaultVector(const std::size_t dim, const double value) {
  return Eigen::VectorXd::Constant(static_cast<Eigen::Index>(dim), value);
}

bool IsFiniteAndNonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
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

void CheckVectorDim(const Eigen::VectorXd& value,
                    const std::size_t expected_size,
                    const char* name) {
  if (value.size() != static_cast<Eigen::Index>(expected_size)) {
    throw std::invalid_argument(std::string("ContinuousQddotMppiConfig: ") +
                                name + " dimension mismatch");
  }
}

void PrepareConfig(ContinuousQddotMppiConfig* config) {
  if (config == nullptr) {
    throw std::invalid_argument("ContinuousQddotMppiConfig: config is null");
  }
  const std::size_t dim = config->rollout.action_dim;
  if (dim == 0U) {
    throw std::invalid_argument(
        "ContinuousQddotMppiConfig: action_dim must be nonzero");
  }

  if (config->rollout.action_lower_bound.size() == 0) {
    config->rollout.action_lower_bound = DefaultVector(dim, -20.0);
  }
  if (config->rollout.action_upper_bound.size() == 0) {
    config->rollout.action_upper_bound = DefaultVector(dim, 20.0);
  }
  if (config->rollout.action_noise_std.size() == 0) {
    config->rollout.action_noise_std = DefaultVector(dim, 1.0);
  }
  if (config->rollout.action_noise_clip.size() == 0) {
    config->rollout.action_noise_clip = DefaultVector(dim, 10.0);
  }
  if (config->limits.qdot_lower_bound.size() == 0) {
    config->limits.qdot_lower_bound = config->rollout.qdot_lower_bound;
  }
  if (config->limits.qdot_upper_bound.size() == 0) {
    config->limits.qdot_upper_bound = config->rollout.qdot_upper_bound;
  }
  if (config->limits.qdot_lower_bound.size() == 0) {
    config->limits.qdot_lower_bound = DefaultVector(dim, -10.0);
  }
  if (config->limits.qdot_upper_bound.size() == 0) {
    config->limits.qdot_upper_bound = DefaultVector(dim, 10.0);
  }

  config->disturbance_sampler.horizon_steps = config->rollout.horizon_steps;
  config->disturbance_sampler.num_disturbance_rollouts =
      config->rollout.num_rollouts;
}

void ValidateConfig(const ContinuousQddotMppiConfig& config) {
  const std::size_t dim = config.rollout.action_dim;
  if (config.rollout.horizon_steps < 2U ||
      config.rollout.num_rollouts == 0U ||
      !std::isfinite(config.rollout.dt) || config.rollout.dt <= 0.0 ||
      !std::isfinite(config.rollout.temperature) ||
      config.rollout.temperature <= 0.0 ||
      !IsFiniteAndNonnegative(config.control_rate_cost_weight) ||
      !std::isfinite(config.smoothing_alpha) ||
      config.smoothing_alpha < 0.0 || config.smoothing_alpha > 1.0) {
    throw std::invalid_argument(
        "ContinuousQddotMppiConfig: invalid scalar field");
  }

  CheckVectorDim(config.rollout.action_lower_bound, dim,
                 "action_lower_bound");
  CheckVectorDim(config.rollout.action_upper_bound, dim,
                 "action_upper_bound");
  CheckVectorDim(config.rollout.action_noise_std, dim,
                 "action_noise_std");
  CheckVectorDim(config.rollout.action_noise_clip, dim,
                 "action_noise_clip");
  CheckVectorDim(config.limits.qdot_lower_bound, dim, "qdot_lower_bound");
  CheckVectorDim(config.limits.qdot_upper_bound, dim, "qdot_upper_bound");

  if (!IsFiniteOrInfinite(config.rollout.action_lower_bound) ||
      !IsFiniteOrInfinite(config.rollout.action_upper_bound) ||
      !IsFiniteOrInfinite(config.limits.qdot_lower_bound) ||
      !IsFiniteOrInfinite(config.limits.qdot_upper_bound)) {
    throw std::invalid_argument(
        "ContinuousQddotMppiConfig: bounds cannot contain NaN");
  }
  if (!IsFiniteAndNonnegative(config.rollout.action_noise_std) ||
      !IsFiniteAndNonnegative(config.rollout.action_noise_clip)) {
    throw std::invalid_argument(
        "ContinuousQddotMppiConfig: noise fields must be finite and nonnegative");
  }
  for (Eigen::Index i = 0; i < config.rollout.action_lower_bound.size(); ++i) {
    if (config.rollout.action_lower_bound[i] >
        config.rollout.action_upper_bound[i]) {
      throw std::invalid_argument(
          "ContinuousQddotMppiConfig: action_lower_bound must be <= action_upper_bound");
    }
    if (config.limits.qdot_lower_bound[i] >
        config.limits.qdot_upper_bound[i]) {
      throw std::invalid_argument(
          "ContinuousQddotMppiConfig: qdot_lower_bound must be <= qdot_upper_bound");
    }
  }
}

double SanitizeCost(const double cost) {
  if (!std::isfinite(cost)) {
    return kLargeCost;
  }
  return std::clamp(cost, 0.0, kLargeCost);
}

Eigen::VectorXd ClampVector(const Eigen::Ref<const Eigen::VectorXd>& value,
                            const Eigen::VectorXd& lower,
                            const Eigen::VectorXd& upper) {
  Eigen::VectorXd clamped = value;
  if (lower.size() != value.size() || upper.size() != value.size()) {
    return clamped;
  }
  for (Eigen::Index i = 0; i < clamped.size(); ++i) {
    clamped[i] = std::clamp(clamped[i], lower[i], upper[i]);
  }
  return clamped;
}

bool HasPinocchioConfiguration(const Eigen::Ref<const Eigen::VectorXd>& q,
                               const Eigen::Ref<const Eigen::VectorXd>& v,
                               const RobotSystem* robot_system) {
  return robot_system != nullptr && robot_system->hasModel() &&
         q.size() == static_cast<Eigen::Index>(robot_system->nq()) &&
         v.size() == static_cast<Eigen::Index>(robot_system->nv());
}

Eigen::VectorXd IntegrateConfiguration(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& tangent_step,
    const RobotSystem* robot_system) {
  if (HasPinocchioConfiguration(q, tangent_step, robot_system)) {
    return pinocchio::integrate(robot_system->model(), q, tangent_step);
  }
  if (q.size() == tangent_step.size()) {
    return q + tangent_step;
  }
  return {};
}

void ClampPositionToModelLimits(RobotSystem* robot_system,
                                Eigen::VectorXd* q) {
  if (robot_system == nullptr || q == nullptr || !robot_system->hasModel()) {
    return;
  }
  const auto& model = robot_system->model();
  if (q->size() != static_cast<Eigen::Index>(model.nq) ||
      model.lowerPositionLimit.size() != model.nq ||
      model.upperPositionLimit.size() != model.nq) {
    return;
  }
  for (Eigen::Index i = 0; i < q->size(); ++i) {
    const double lower = model.lowerPositionLimit[i];
    const double upper = model.upperPositionLimit[i];
    if (std::isfinite(lower) && (*q)[i] < lower) {
      (*q)[i] = lower;
    }
    if (std::isfinite(upper) && (*q)[i] > upper) {
      (*q)[i] = upper;
    }
  }
}

Eigen::VectorXd RneaOrZero(const Eigen::Ref<const Eigen::VectorXd>& q,
                           const Eigen::Ref<const Eigen::VectorXd>& qdot,
                           const Eigen::Ref<const Eigen::VectorXd>& qddot,
                           RobotSystem* robot_system) {
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(qddot.size());
  if (!HasPinocchioConfiguration(q, qdot, robot_system) ||
      qddot.size() != static_cast<Eigen::Index>(robot_system->nv())) {
    return tau;
  }
  const Eigen::VectorXd rnea =
      pinocchio::rnea(robot_system->model(), robot_system->data(),
                      q, qdot, qddot);
  if (rnea.size() == tau.size() && rnea.allFinite()) {
    tau = rnea;
  }
  return tau;
}

struct CostAccumulation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double contact_loss_cost{0.0};
  double support_cost{0.0};
  double edge_cost{0.0};
  double penetration_cost{0.0};
  double preload_cost{0.0};
  double force_low_cost{0.0};
  double force_high_cost{0.0};
  double balance_cost{0.0};
  double control_cost{0.0};
  double rate_cost{0.0};

  std::size_t geometry_query_count{0};
  std::size_t object_sample_count{0};
  double predicted_active_hemisphere_total{0.0};
  double measured_active_hemisphere_total{0.0};
  double object_contact_loss_count{0.0};
  double object_edge_margin_m{std::numeric_limits<double>::infinity()};
  double object_min_signed_distance_m{std::numeric_limits<double>::infinity()};
  Eigen::Vector2d predicted_centroid_sum{Eigen::Vector2d::Zero()};
  Eigen::Vector2d measured_centroid_sum{Eigen::Vector2d::Zero()};
  double predicted_centroid_weight{0.0};
  double measured_centroid_weight{0.0};
  double object_linear_disturbance_speed_mps{0.0};
  double object_angular_disturbance_speed_radps{0.0};
  double normalization_count{0.0};

  Eigen::Vector2d predictedCentroid() const {
    if (predicted_centroid_weight <= 0.0) {
      return Eigen::Vector2d::Constant(
          std::numeric_limits<double>::quiet_NaN());
    }
    return predicted_centroid_sum / predicted_centroid_weight;
  }

  Eigen::Vector2d measuredCentroid() const {
    if (measured_centroid_weight <= 0.0) {
      return Eigen::Vector2d::Constant(
          std::numeric_limits<double>::quiet_NaN());
    }
    return measured_centroid_sum / measured_centroid_weight;
  }
};

using ObjectPoseRollout =
    std::vector<Eigen::Isometry3d,
                Eigen::aligned_allocator<Eigen::Isometry3d>>;

bool RepresentativeObjectPose(const VirtualObjectBelief& belief,
                              Eigen::Isometry3d* pose_world) {
  if (pose_world == nullptr || !HasVirtualObjectBelief(belief) ||
      !IsValidVirtualObjectBelief(belief) || belief.particles.empty()) {
    return false;
  }

  double weight_sum = 0.0;
  Eigen::Vector3d weighted_translation = Eigen::Vector3d::Zero();
  const VirtualObjectState* orientation_source = nullptr;
  double orientation_weight = -1.0;
  for (const auto& particle : belief.particles) {
    if (!IsValidVirtualObjectState(particle) ||
        !particle.pose_world.matrix().allFinite()) {
      continue;
    }
    const double weight =
        std::isfinite(particle.weight) && particle.weight > 0.0
            ? particle.weight
            : 1.0;
    weighted_translation += weight * particle.pose_world.translation();
    weight_sum += weight;
    if (weight > orientation_weight) {
      orientation_weight = weight;
      orientation_source = &particle;
    }
  }
  if (orientation_source == nullptr || weight_sum <= kTiny) {
    return false;
  }

  *pose_world = orientation_source->pose_world;
  pose_world->translation() = weighted_translation / weight_sum;
  return true;
}

void AppendRepresentativeObjectPose(const VirtualObjectBelief& belief,
                                    ObjectPoseRollout* rollout) {
  if (rollout == nullptr) {
    return;
  }
  Eigen::Isometry3d pose_world = Eigen::Isometry3d::Identity();
  if (RepresentativeObjectPose(belief, &pose_world)) {
    rollout->push_back(pose_world);
  }
}

template <typename EvaluationVector>
ObjectPoseRollout WeightedObjectPoseRollout(
    const EvaluationVector& evaluations, const std::vector<double>& weights,
    const std::size_t best_index) {
  ObjectPoseRollout output;
  if (evaluations.empty() || weights.size() != evaluations.size() ||
      best_index >= evaluations.size()) {
    return output;
  }

  const auto& best_rollout = evaluations[best_index].stats.object_pose_rollout;
  std::size_t max_steps = best_rollout.size();
  for (const auto& evaluation : evaluations) {
    max_steps = std::max(max_steps, evaluation.stats.object_pose_rollout.size());
  }
  output.reserve(max_steps);

  for (std::size_t step = 0; step < max_steps; ++step) {
    Eigen::Vector3d weighted_translation = Eigen::Vector3d::Zero();
    double weight_sum = 0.0;
    for (std::size_t sample = 0; sample < evaluations.size(); ++sample) {
      const auto& rollout = evaluations[sample].stats.object_pose_rollout;
      if (step >= rollout.size() || weights[sample] <= 0.0 ||
          !std::isfinite(weights[sample])) {
        continue;
      }
      weighted_translation += weights[sample] * rollout[step].translation();
      weight_sum += weights[sample];
    }

    Eigen::Isometry3d pose_world =
        step < best_rollout.size() ? best_rollout[step]
                                   : Eigen::Isometry3d::Identity();
    if (weight_sum > kTiny) {
      pose_world.translation() = weighted_translation / weight_sum;
    }
    output.push_back(pose_world);
  }
  return output;
}

void AccumulateStageBreakdown(
    const RobustGraspStateCostBreakdown& breakdown,
    const GraspDisturbanceStep& disturbance,
    const double rate_cost,
    CostAccumulation* total) {
  if (total == nullptr) {
    return;
  }
  total->preload_cost += breakdown.preload_cost;
  total->force_low_cost += breakdown.force_low_cost;
  total->force_high_cost += breakdown.force_high_cost;
  total->balance_cost += breakdown.force_balance_cost;
  total->control_cost += breakdown.action_cost;
  total->rate_cost += rate_cost;
  total->object_linear_disturbance_speed_mps +=
      disturbance.object_disturbance.linear_velocity_world_mps.norm();
  total->object_angular_disturbance_speed_radps +=
      disturbance.object_disturbance.angular_velocity_world_radps.norm();
  total->normalization_count += 1.0;

  if (!breakdown.object_support.valid) {
    return;
  }
  const auto& object = breakdown.object_support;
  total->contact_loss_cost += object.contact_loss_cost;
  total->support_cost += object.support_cost;
  total->edge_cost += object.edge_cost;
  total->penetration_cost += object.penetration_cost;
  total->geometry_query_count += object.geometry_query_count;
  total->object_sample_count += object.object_sample_count;
  total->predicted_active_hemisphere_total +=
      object.predicted_active_hemisphere_total;
  total->measured_active_hemisphere_total +=
      object.measured_active_hemisphere_total;
  total->object_contact_loss_count += object.lost_measured_contact_count;
  total->object_edge_margin_m =
      std::min(total->object_edge_margin_m, object.min_edge_margin_m);
  total->object_min_signed_distance_m =
      std::min(total->object_min_signed_distance_m,
               object.min_signed_distance_m);
  if (object.support_summary.predicted_centroid_sensor_m.allFinite()) {
    total->predicted_centroid_sum +=
        object.support_summary.predicted_centroid_sensor_m;
    total->predicted_centroid_weight += 1.0;
  }
  if (object.support_summary.measured_centroid_sensor_m.allFinite()) {
    total->measured_centroid_sum +=
        object.support_summary.measured_centroid_sensor_m;
    total->measured_centroid_weight += 1.0;
  }
}

void AddWeightedStats(const CostAccumulation& sample,
                      const double weight,
                      ContinuousQddotMppiStatus* status) {
  if (status == nullptr || weight <= 0.0 || !std::isfinite(weight)) {
    return;
  }
  status->selected_contact_loss_cost += weight * sample.contact_loss_cost;
  status->selected_support_cost += weight * sample.support_cost;
  status->selected_edge_cost += weight * sample.edge_cost;
  status->selected_penetration_cost += weight * sample.penetration_cost;
  status->selected_preload_cost += weight * sample.preload_cost;
  status->selected_force_low_cost += weight * sample.force_low_cost;
  status->selected_force_high_cost += weight * sample.force_high_cost;
  status->selected_balance_cost += weight * sample.balance_cost;
  status->selected_control_cost += weight * sample.control_cost;
  status->selected_rate_cost += weight * sample.rate_cost;
  const double average_scale =
      sample.normalization_count > kTiny ? 1.0 / sample.normalization_count
                                         : 1.0;
  status->predicted_active_hemisphere_total +=
      weight * sample.predicted_active_hemisphere_total * average_scale;
  status->measured_active_hemisphere_total +=
      weight * sample.measured_active_hemisphere_total * average_scale;
  status->object_contact_loss_count +=
      weight * sample.object_contact_loss_count * average_scale;
  status->object_edge_margin_m =
      std::min(status->object_edge_margin_m, sample.object_edge_margin_m);
  status->object_min_signed_distance_m =
      std::min(
          status->object_min_signed_distance_m,
          sample.object_min_signed_distance_m);
  status->object_linear_disturbance_speed_mps +=
      weight * sample.object_linear_disturbance_speed_mps * average_scale;
  status->object_angular_disturbance_speed_radps +=
      weight * sample.object_angular_disturbance_speed_radps * average_scale;

  const Eigen::Vector2d predicted_centroid = sample.predictedCentroid();
  if (predicted_centroid.allFinite()) {
    if (!status->predicted_centroid_sensor_m.allFinite()) {
      status->predicted_centroid_sensor_m.setZero();
    }
    status->predicted_centroid_sensor_m += weight * predicted_centroid;
  }
  const Eigen::Vector2d measured_centroid = sample.measuredCentroid();
  if (measured_centroid.allFinite()) {
    if (!status->measured_centroid_sensor_m.allFinite()) {
      status->measured_centroid_sensor_m.setZero();
    }
    status->measured_centroid_sensor_m += weight * measured_centroid;
  }
}

}  // namespace

struct ContinuousQddotMppiController::SampleStats {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CostAccumulation costs;
  ObjectPoseRollout object_pose_rollout;
};

struct ContinuousQddotMppiController::SampleEvaluation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double total_cost{kLargeCost};
  SampleStats stats;
  bool valid{false};
};

std::vector<double> ComputeSoftMppiWeights(
    const std::vector<double>& costs, const double temperature_lambda) {
  std::vector<double> weights(costs.size(), 0.0);
  if (costs.empty() || !std::isfinite(temperature_lambda) ||
      temperature_lambda <= 0.0) {
    return weights;
  }

  double beta = std::numeric_limits<double>::infinity();
  for (const double cost : costs) {
    if (std::isfinite(cost)) {
      beta = std::min(beta, cost);
    }
  }
  if (!std::isfinite(beta)) {
    return weights;
  }

  double weight_sum = 0.0;
  for (std::size_t i = 0; i < costs.size(); ++i) {
    if (!std::isfinite(costs[i])) {
      continue;
    }
    weights[i] = std::exp(-(costs[i] - beta) / temperature_lambda);
    weight_sum += weights[i];
  }
  if (weight_sum <= kTiny || !std::isfinite(weight_sum)) {
    const auto best_it = std::min_element(costs.begin(), costs.end());
    if (best_it != costs.end() && std::isfinite(*best_it)) {
      weights[static_cast<std::size_t>(best_it - costs.begin())] = 1.0;
    }
    return weights;
  }
  for (double& weight : weights) {
    weight /= weight_sum;
  }
  return weights;
}

RobotState StepRobotStateWithQddotLimits(
    const RobotState& robot,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
    RobotSystem* robot_system,
    const double dt,
    const QddotRolloutLimits& limits) {
  RobotState next;
  if (!IsValid(robot) || !std::isfinite(dt) || dt <= 0.0 ||
      qddot_sol.size() != robot.qdot.size() || !qddot_sol.allFinite()) {
    return next;
  }

  next.qdot = robot.qdot + qddot_sol * dt;
  next.qdot =
      ClampVector(next.qdot, limits.qdot_lower_bound,
                  limits.qdot_upper_bound);
  next.q = IntegrateConfiguration(robot.q, next.qdot * dt, robot_system);
  if (limits.clamp_q_to_model_position_limits) {
    ClampPositionToModelLimits(robot_system, &next.q);
  }
  if (next.q.size() == 0 || !next.q.allFinite() || !next.qdot.allFinite()) {
    return RobotState{};
  }
  next.tau = RneaOrZero(robot.q, robot.qdot, qddot_sol, robot_system);
  next.time_s = robot.time_s + dt;
  next.valid = next.q.allFinite() && next.qdot.allFinite() &&
               next.tau.allFinite() && next.qdot.size() == next.tau.size() &&
               std::isfinite(next.time_s);
  return next;
}

void ContinuousQddotMppiController::Initialize(
    ContinuousQddotMppiConfig config) {
  PrepareConfig(&config);
  ValidateConfig(config);

  config_ = std::move(config);
  cost_ = RobustGraspStateCost(config_.cost);
  disturbance_sampler_ =
      GraspDisturbanceSampler(config_.disturbance_sampler);
  rng_.seed(config_.rollout.random_seed);
  nominal_sequence_.Resize(config_.rollout.action_dim,
                           config_.rollout.horizon_steps);
  nominal_sequence_.SetZero();
  previous_qddot_cmd_.resize(0);
  has_previous_qddot_cmd_ = false;
  status_ = ContinuousQddotMppiStatus{};
  initialized_ = true;
}

void ContinuousQddotMppiController::ResetNominalSequence() {
  if (nominal_sequence_.actionDim() == config_.rollout.action_dim &&
      nominal_sequence_.horizonSteps() == config_.rollout.horizon_steps) {
    nominal_sequence_.SetZero();
  }
  previous_qddot_cmd_.resize(0);
  has_previous_qddot_cmd_ = false;
}

ActionSequence ContinuousQddotMppiController::SampleSequence(
    const std::size_t sample_index) {
  ActionSequence sequence(config_.rollout.action_dim,
                          config_.rollout.horizon_steps);
  sequence.values() = nominal_sequence_.values();
  sequence.setName(std::string("mppi_sample_") +
                   std::to_string(sample_index));
  if (sample_index == 0U) {
    sequence.setName("nominal");
    return sequence;
  }

  std::normal_distribution<double> normal(0.0, 1.0);
  for (Eigen::Index step = 0; step < sequence.values().cols(); ++step) {
    Eigen::VectorXd action = sequence.values().col(step);
    for (Eigen::Index dim = 0; dim < action.size(); ++dim) {
      const double noise = std::clamp(
          normal(rng_) * config_.rollout.action_noise_std[dim],
          -config_.rollout.action_noise_clip[dim],
          config_.rollout.action_noise_clip[dim]);
      action[dim] += noise;
    }
    sequence.values().col(step) =
        ClampVector(action, config_.rollout.action_lower_bound,
                    config_.rollout.action_upper_bound);
  }
  return sequence;
}

ContinuousQddotMppiController::SampleEvaluation
ContinuousQddotMppiController::EvaluateSequence(
    const GraspState& initial_state,
    const ActionSequence& sequence,
    const GraspDisturbanceSequence& disturbance_sequence,
    const RolloutContext& context) const {
  SampleEvaluation evaluation;
  if (!initial_state.valid ||
      sequence.actionDim() != config_.rollout.action_dim ||
      sequence.horizonSteps() != config_.rollout.horizon_steps ||
      disturbance_sequence.horizonSteps() < sequence.horizonSteps()) {
    return evaluation;
  }

  GraspState state = initial_state;
  AppendRepresentativeObjectPose(
      state.object_belief, &evaluation.stats.object_pose_rollout);
  Eigen::VectorXd previous_action =
      has_previous_qddot_cmd_ &&
              previous_qddot_cmd_.size() ==
                  static_cast<Eigen::Index>(config_.rollout.action_dim)
          ? previous_qddot_cmd_
          : Eigen::VectorXd::Zero(
                static_cast<Eigen::Index>(config_.rollout.action_dim));
  double total_cost = 0.0;

  for (std::size_t step = 0; step < sequence.horizonSteps(); ++step) {
    const Eigen::VectorXd action = sequence.action(step);
    const auto& disturbance_step = disturbance_sequence.steps[step];

    ObjectPriorRolloutStepResult result;
    result.next_state.robot = StepRobotStateWithQddotLimits(
        state.robot, action, context.robot_system, config_.rollout.dt,
        config_.limits);
    result.next_state.tactile_sensors = state.tactile_sensors;
    result.next_state.object_belief = StepVirtualObjectBelief(
        state.object_belief, disturbance_step.object_disturbance,
        config_.rollout.dt);
    result.next_state.valid =
        IsValid(result.next_state.robot) &&
        HasValidTactileSensors(result.next_state) &&
        IsValidVirtualObjectBelief(result.next_state.object_belief);
    result.valid = result.next_state.valid;
    if (!result.valid) {
      return evaluation;
    }

    RobustGraspStateCostBreakdown breakdown;
    const double stage_cost = cost_.Evaluate(
        result.next_state, action, context, &breakdown);
    const double rate_cost =
        config_.control_rate_cost_weight *
        (action - previous_action).squaredNorm();
    const double step_cost = SanitizeCost(stage_cost + rate_cost);
    total_cost = SanitizeCost(total_cost + step_cost);
    AccumulateStageBreakdown(
        breakdown, disturbance_step, rate_cost, &evaluation.stats.costs);

    state = result.next_state;
    AppendRepresentativeObjectPose(
        state.object_belief, &evaluation.stats.object_pose_rollout);
    previous_action = action;
  }

  evaluation.total_cost = total_cost;
  evaluation.valid = std::isfinite(total_cost) && total_cost < kLargeCost;
  return evaluation;
}

RobotCommand ContinuousQddotMppiController::MakeCommand(
    const GraspObservation& observation,
    const GraspState& initial_state,
    const RolloutContext& context,
    const Eigen::VectorXd& qddot_cmd) const {
  RobotState next = StepRobotStateWithQddotLimits(
      initial_state.robot, qddot_cmd, context.robot_system,
      config_.rollout.dt, config_.limits);
  if (!IsValid(next)) {
    RobotCommand hold =
        MakeZeroHoldRobotCommand(initial_state.robot.q,
                                 initial_state.robot.qdot);
    hold.stamp_sec = observation.time_s;
    return hold;
  }

  RobotCommand command;
  command.Resize(static_cast<int>(next.q.size()),
                 static_cast<int>(next.qdot.size()));
  command.q_cmd = next.q;
  command.qdot_cmd = next.qdot;
  command.tau_cmd = next.tau;
  command.stamp_sec = observation.time_s;
  command.valid = command.HasValidDimensions() && command.AllFinite();
  return command;
}

void ContinuousQddotMppiController::ShiftUpdatedSequence(
    const Eigen::MatrixXd& updated_values) {
  nominal_sequence_.values() = updated_values;
  if (nominal_sequence_.horizonSteps() <= 1U) {
    nominal_sequence_.SetZero();
    return;
  }
  nominal_sequence_.values().leftCols(nominal_sequence_.values().cols() - 1) =
      updated_values.rightCols(updated_values.cols() - 1).eval();
  nominal_sequence_.values().rightCols(1).setZero();
}

RobotCommand ContinuousQddotMppiController::Update(
    const GraspObservation& observation,
    const GraspState& initial_state,
    const RolloutContext& context) {
  if (!initialized_) {
    throw std::logic_error(
        "ContinuousQddotMppiController::Update: controller is not initialized");
  }

  status_ = ContinuousQddotMppiStatus{};
  status_.num_samples = config_.rollout.num_rollouts;
  status_.horizon_steps = config_.rollout.horizon_steps;
  status_.lambda = config_.rollout.temperature;
  status_.object_edge_margin_m = std::numeric_limits<double>::infinity();
  status_.object_min_signed_distance_m =
      std::numeric_limits<double>::infinity();
  status_.qddot_nominal_first = nominal_sequence_.firstAction();

  std::vector<ActionSequence> samples;
  samples.reserve(config_.rollout.num_rollouts);
  for (std::size_t i = 0; i < config_.rollout.num_rollouts; ++i) {
    samples.push_back(SampleSequence(i));
  }

  const auto disturbances = disturbance_sampler_.SampleBatch();
  std::vector<double> costs(samples.size(), kLargeCost);
  std::vector<SampleEvaluation> evaluations(samples.size());
  double cost_sum = 0.0;
  double cost_max = 0.0;
  std::size_t finite_count = 0U;
  std::size_t total_geometry_queries = 0U;
  std::size_t total_object_samples = 0U;

  for (std::size_t i = 0; i < samples.size(); ++i) {
    evaluations[i] = EvaluateSequence(
        initial_state, samples[i], disturbances[i], context);
    costs[i] = SanitizeCost(evaluations[i].total_cost);
    total_geometry_queries +=
        evaluations[i].stats.costs.geometry_query_count;
    total_object_samples += evaluations[i].stats.costs.object_sample_count;
    if (std::isfinite(costs[i]) && costs[i] < kLargeCost) {
      cost_sum += costs[i];
      cost_max = std::max(cost_max, costs[i]);
      ++finite_count;
    }
  }
  if (finite_count == 0U) {
    ResetNominalSequence();
    RobotCommand hold =
        MakeZeroHoldRobotCommand(initial_state.robot.q,
                                 initial_state.robot.qdot);
    hold.stamp_sec = observation.time_s;
    return hold;
  }

  const auto best_it = std::min_element(costs.begin(), costs.end());
  const std::size_t best_index =
      static_cast<std::size_t>(best_it - costs.begin());
  const std::vector<double> weights =
      ComputeSoftMppiWeights(costs, config_.rollout.temperature);

  Eigen::MatrixXd updated = Eigen::MatrixXd::Zero(
      nominal_sequence_.values().rows(), nominal_sequence_.values().cols());
  double weighted_cost = 0.0;
  double weight_square_sum = 0.0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const double weight = weights[i];
    if (weight <= 0.0) {
      continue;
    }
    updated += weight * samples[i].values();
    weighted_cost += weight * costs[i];
    weight_square_sum += weight * weight;
    AddWeightedStats(evaluations[i].stats.costs, weight, &status_);
  }

  Eigen::VectorXd qddot_cmd = updated.col(0);
  if (has_previous_qddot_cmd_ &&
      previous_qddot_cmd_.size() == qddot_cmd.size()) {
    qddot_cmd = config_.smoothing_alpha * qddot_cmd +
                (1.0 - config_.smoothing_alpha) * previous_qddot_cmd_;
  }
  qddot_cmd = ClampVector(qddot_cmd, config_.rollout.action_lower_bound,
                          config_.rollout.action_upper_bound);

  status_.valid = true;
  status_.best_sample_index = best_index;
  status_.best_sample_cost = *best_it;
  status_.weighted_cost_estimate = weighted_cost;
  status_.cost_min = *best_it;
  status_.cost_mean = cost_sum / static_cast<double>(finite_count);
  status_.cost_max = cost_max;
  status_.effective_sample_size =
      weight_square_sum > kTiny ? 1.0 / weight_square_sum : 0.0;
  status_.qddot_cmd = qddot_cmd;
  status_.qddot_best_first = samples[best_index].firstAction();
  status_.object_pose_rollout =
      WeightedObjectPoseRollout(evaluations, weights, best_index);
  status_.geometry_query_count = total_geometry_queries;
  status_.object_sample_count = total_object_samples;
  if (!std::isfinite(status_.object_edge_margin_m)) {
    status_.object_edge_margin_m = 0.0;
  }
  if (!std::isfinite(status_.object_min_signed_distance_m)) {
    status_.object_min_signed_distance_m = 0.0;
  }

  RobotCommand command =
      MakeCommand(observation, initial_state, context, qddot_cmd);
  previous_qddot_cmd_ = qddot_cmd;
  has_previous_qddot_cmd_ = true;
  ShiftUpdatedSequence(updated);
  return command;
}

}  // namespace mppi_core
