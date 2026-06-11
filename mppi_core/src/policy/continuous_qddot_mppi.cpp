// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/policy/continuous_qddot_mppi.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mppi_core/contact/contact_kinematics.hpp"
#include "mppi_core/rollout/object_prior_grasp_rollout.hpp"

namespace mppi_core {
namespace {

constexpr double kLargeCost = 1.0e30;
constexpr double kTiny = 1.0e-12;

struct ThreadLocalRolloutWorkspace {
  RobotSystem robot_system;
  std::vector<PinocchioContactKinematicsContext> kinematics;
  RolloutContext context;
};

Eigen::VectorXd DefaultVector(const std::size_t dim, const double value) {
  return Eigen::VectorXd::Constant(static_cast<Eigen::Index>(dim), value);
}

std::size_t ResolveEvaluationThreadCount(const MPPIConfig& config) {
  if (config.num_rollouts <= 1U || config.num_threads == 1U) {
    return 1U;
  }
  std::size_t requested = config.num_threads;
  if (requested == 0U) {
    requested = static_cast<std::size_t>(std::thread::hardware_concurrency());
  }
  if (requested == 0U) {
    return 1U;
  }
  return std::max<std::size_t>(
      1U, std::min<std::size_t>(config.num_rollouts, requested));
}

void ConfigureThreadLocalRolloutWorkspace(
    const RolloutContext& source,
    ThreadLocalRolloutWorkspace* workspace) {
  if (workspace == nullptr) {
    return;
  }
  workspace->context = source;
  workspace->context.tactile_contexts = source.tactile_contexts;
  workspace->context.robot_system = source.robot_system;
  workspace->kinematics.clear();

  if (source.robot_system == nullptr || !source.robot_system->hasModel()) {
    return;
  }

  workspace->robot_system.LoadModel(source.robot_system->model());
  workspace->context.robot_system = &workspace->robot_system;
  workspace->kinematics.resize(workspace->context.tactile_contexts.size());
  for (std::size_t i = 0; i < workspace->context.tactile_contexts.size(); ++i) {
    const auto* source_kinematics = source.tactile_contexts[i].kinematics;
    if (source_kinematics == nullptr) {
      workspace->context.tactile_contexts[i].kinematics = nullptr;
      continue;
    }
    auto& local_kinematics = workspace->kinematics[i];
    local_kinematics.model = &workspace->robot_system.model();
    local_kinematics.data = &workspace->robot_system.data();
    local_kinematics.sensor_frame_id = source_kinematics->sensor_frame_id;
    local_kinematics.normal_axis_sign =
        source_kinematics->normal_axis_sign;
    workspace->context.tactile_contexts[i].kinematics =
        &local_kinematics;
  }
}

bool IsFiniteAndNonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

double MaxAbsCoeff(const Eigen::VectorXd& value) {
  double max_abs = 0.0;
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    max_abs = std::max(max_abs, std::abs(value[i]));
  }
  return max_abs;
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
  if (config->rnea_feedforward.max_tau_ff_nm < 0.0) {
    config->rnea_feedforward.max_tau_ff_nm = 0.0;
  }
  if (config->rnea_feedforward.max_tau_ff_rate_nm_s < 0.0) {
    config->rnea_feedforward.max_tau_ff_rate_nm_s = 0.0;
  }
  if (config->base_grasp_controller.max_qddot_base < 0.0) {
    config->base_grasp_controller.max_qddot_base = 0.0;
  }
  if (config->base_grasp_controller.max_qddot_residual < 0.0) {
    config->base_grasp_controller.max_qddot_residual = 0.0;
  }
  if (config->base_grasp_controller.base_deviation_weight < 0.0) {
    config->base_grasp_controller.base_deviation_weight = 0.0;
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
  const RneaFeedforwardConfig& rnea = config.rnea_feedforward;
  if (!std::isfinite(rnea.tau_ff_scale) ||
      !IsFiniteAndNonnegative(rnea.max_tau_ff_nm) ||
      !IsFiniteAndNonnegative(rnea.max_tau_ff_rate_nm_s)) {
    throw std::invalid_argument(
        "ContinuousQddotMppiConfig: invalid RNEA feedforward scalar field");
  }
  if (rnea.subtract_contact_torque) {
    throw std::invalid_argument(
        "ContinuousQddotMppiConfig: subtract_contact_torque is not supported");
  }
  const BaseGraspControllerConfig& base = config.base_grasp_controller;
  if (!IsFiniteAndNonnegative(base.target_normal_force_n) ||
      !IsFiniteAndNonnegative(base.min_normal_force_per_sensor_n) ||
      !IsFiniteAndNonnegative(base.max_normal_force_per_sensor_n) ||
      base.min_normal_force_per_sensor_n > base.max_normal_force_per_sensor_n ||
      !IsFiniteAndNonnegative(base.force_gain) ||
      !IsFiniteAndNonnegative(base.force_balance_gain) ||
      !IsFiniteAndNonnegative(base.contact_loss_gain) ||
      !IsFiniteAndNonnegative(base.high_force_release_gain) ||
      !IsFiniteAndNonnegative(base.max_qddot_base) ||
      !IsFiniteAndNonnegative(base.max_qddot_residual) ||
      !IsFiniteAndNonnegative(base.base_deviation_weight)) {
    throw std::invalid_argument(
        "ContinuousQddotMppiConfig: invalid base grasp controller field");
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

Eigen::VectorXd ClampVectorNorm(const Eigen::Ref<const Eigen::VectorXd>& value,
                                const double max_norm) {
  Eigen::VectorXd clamped = value;
  if (!clamped.allFinite()) {
    clamped.setZero();
    return clamped;
  }
  if (!std::isfinite(max_norm)) {
    return clamped;
  }
  if (max_norm <= 0.0) {
    clamped.setZero();
    return clamped;
  }
  const double norm = clamped.norm();
  if (std::isfinite(norm) && norm > max_norm) {
    clamped *= max_norm / norm;
  }
  return clamped;
}

Eigen::VectorXd NormalizedOrZero(const Eigen::Ref<const Eigen::VectorXd>& value,
                                 const std::size_t dim) {
  Eigen::VectorXd normalized = value;
  if (normalized.size() != static_cast<Eigen::Index>(dim) ||
      !normalized.allFinite()) {
    return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim));
  }
  const double norm = normalized.norm();
  if (!std::isfinite(norm) || norm <= kTiny) {
    return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim));
  }
  normalized /= norm;
  return normalized;
}

bool LooksLikeRole(const TactileState& tactile, const char* role) {
  return role != nullptr &&
         tactile.frame_name.find(role) != std::string::npos;
}

struct BaseSensorBasis {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};
  bool is_thumb{false};
  bool is_index{false};
  double force_n{0.0};
  Eigen::VectorXd close_direction;
};

BaseSensorBasis ComputeBaseSensorBasis(
    const GraspState& state,
    const TactileState& tactile,
    const TactileSensorContext& tactile_context,
    const std::size_t sensor_order,
    const std::size_t action_dim) {
  BaseSensorBasis basis;
  basis.close_direction =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(action_dim));
  if (!state.valid || !IsValid(state.robot) || !tactile.valid ||
      !tactile.hasActiveHemisphereContact() ||
      tactile_context.kinematics == nullptr ||
      !IsValidContactKinematicsContext(*tactile_context.kinematics) ||
      action_dim == 0U ||
      state.robot.q.size() !=
          static_cast<Eigen::Index>(tactile_context.kinematics->model->nq) ||
      static_cast<Eigen::Index>(action_dim) !=
          tactile_context.kinematics->model->nv) {
    return basis;
  }

  const Eigen::VectorXd zero_tangent =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(action_dim));
  const auto motions = ComputeHemisphereMotions(
      state.robot, tactile, zero_tangent, *tactile_context.kinematics);
  if (motions.empty()) {
    return basis;
  }

  Eigen::VectorXd close_sum =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(action_dim));
  std::size_t count = 0U;
  for (const auto& motion : motions) {
    if (motion.J_normal.size() != static_cast<Eigen::Index>(action_dim) ||
        !motion.J_normal.allFinite()) {
      continue;
    }
    close_sum += motion.J_normal.transpose();
    ++count;
  }
  if (count == 0U) {
    return basis;
  }

  basis.close_direction = NormalizedOrZero(close_sum, action_dim);
  basis.valid = basis.close_direction.norm() > 0.0;
  basis.force_n = tactile.activeHemisphereNormalForceN();
  basis.is_thumb = LooksLikeRole(tactile, "thumb");
  basis.is_index = LooksLikeRole(tactile, "index");
  if (!basis.is_thumb && !basis.is_index) {
    basis.is_thumb = sensor_order == 0U;
    basis.is_index = sensor_order == 1U;
  }
  return basis;
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

}  // namespace

RneaFeedforwardCommandResult ComputeRneaFeedforwardCommand(
    const RneaFeedforwardConfig& config,
    const GraspObservation& observation,
    const GraspState& initial_state,
    const RolloutContext& context,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_cmd,
    const Eigen::Ref<const Eigen::VectorXd>& previous_tau_ff_cmd,
    const bool has_previous_tau_ff_cmd,
    const double dt) {
  RneaFeedforwardCommandResult result;
  result.raw = Eigen::VectorXd::Zero(qddot_cmd.size());
  result.scaled = Eigen::VectorXd::Zero(qddot_cmd.size());
  result.command = Eigen::VectorXd::Zero(qddot_cmd.size());

  if (!config.enabled) {
    return result;
  }
  if (config.zero_tau_when_not_ready && !initial_state.valid) {
    result.zeroed_not_ready = true;
    return result;
  }
  if (config.zero_tau_on_contact_loss && !initial_state.hasAnyTactileContact()) {
    result.zeroed_contact_loss = true;
    return result;
  }

  RobotSystem* robot_system =
      context.robot_system != nullptr ? context.robot_system
                                      : observation.robot_system;
  const Eigen::VectorXd& q =
      config.use_measured_state ? observation.q_meas : observation.q_ref_current;
  const Eigen::VectorXd& qdot =
      config.use_measured_state ? observation.qdot_meas
                                : observation.qdot_ref_current;
  if (!HasPinocchioConfiguration(q, qdot, robot_system) ||
      qddot_cmd.size() != static_cast<Eigen::Index>(robot_system->nv())) {
    return result;
  }

  // RNEA feedforward maps desired joint acceleration to model torque.
  // It does not estimate contact force; contact stability is handled by
  // tactile feedback and object-support cost.
  Eigen::VectorXd qddot_rnea = qddot_cmd;
  if (config.gravity_only_when_qddot_zero && qddot_cmd.isZero(kTiny)) {
    qddot_rnea.setZero();
  }
  result.raw = RneaOrZero(q, qdot, qddot_rnea, robot_system);
  result.scaled = config.tau_ff_scale * result.raw;
  result.command = result.scaled;

  if (std::isfinite(config.max_tau_ff_nm)) {
    for (Eigen::Index i = 0; i < result.command.size(); ++i) {
      const double before = result.command[i];
      result.command[i] =
          std::clamp(before, -config.max_tau_ff_nm, config.max_tau_ff_nm);
      result.clamped = result.clamped || result.command[i] != before;
    }
  }

  if (has_previous_tau_ff_cmd &&
      previous_tau_ff_cmd.size() == result.command.size() &&
      previous_tau_ff_cmd.allFinite() &&
      std::isfinite(config.max_tau_ff_rate_nm_s) &&
      std::isfinite(dt) && dt > 0.0) {
    const double max_delta = config.max_tau_ff_rate_nm_s * dt;
    for (Eigen::Index i = 0; i < result.command.size(); ++i) {
      const double before = result.command[i];
      result.command[i] =
          std::clamp(before,
                     previous_tau_ff_cmd[i] - max_delta,
                     previous_tau_ff_cmd[i] + max_delta);
      result.rate_limited = result.rate_limited || result.command[i] != before;
    }
  }

  return result;
}

namespace {

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
  double base_deviation_cost{0.0};

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
  Eigen::Vector3d object_linear_disturbance_world_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d object_angular_disturbance_world_radps{Eigen::Vector3d::Zero()};
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

  Eigen::Vector3d objectLinearDisturbanceAverage() const {
    if (normalization_count <= 0.0) {
      return Eigen::Vector3d::Zero();
    }
    return object_linear_disturbance_world_mps / normalization_count;
  }

  Eigen::Vector3d objectAngularDisturbanceAverage() const {
    if (normalization_count <= 0.0) {
      return Eigen::Vector3d::Zero();
    }
    return object_angular_disturbance_world_radps / normalization_count;
  }

  double objectLinearDisturbanceSpeedAverage() const {
    return normalization_count <= 0.0
               ? 0.0
               : object_linear_disturbance_speed_mps / normalization_count;
  }

  double objectAngularDisturbanceSpeedAverage() const {
    return normalization_count <= 0.0
               ? 0.0
               : object_angular_disturbance_speed_radps / normalization_count;
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

Eigen::Vector3d AverageObjectLinearDisturbance(
    const GraspDisturbanceSequence& sequence) {
  if (sequence.steps.empty()) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  double count = 0.0;
  for (const auto& step : sequence.steps) {
    if (step.object_disturbance.linear_velocity_world_mps.allFinite()) {
      sum += step.object_disturbance.linear_velocity_world_mps;
      count += 1.0;
    }
  }
  if (count <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  return sum / count;
}

Eigen::Vector3d AverageObjectAngularDisturbance(
    const GraspDisturbanceSequence& sequence) {
  if (sequence.steps.empty()) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  double count = 0.0;
  for (const auto& step : sequence.steps) {
    if (step.object_disturbance.angular_velocity_world_radps.allFinite()) {
      sum += step.object_disturbance.angular_velocity_world_radps;
      count += 1.0;
    }
  }
  if (count <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  return sum / count;
}

void AccumulateStageBreakdown(
    const RobustGraspStateCostBreakdown& breakdown,
    const GraspDisturbanceStep& disturbance,
    const double rate_cost,
    const double base_deviation_cost,
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
  total->base_deviation_cost += base_deviation_cost;
  total->object_linear_disturbance_world_mps +=
      disturbance.object_disturbance.linear_velocity_world_mps;
  total->object_angular_disturbance_world_radps +=
      disturbance.object_disturbance.angular_velocity_world_radps;
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
  status->selected_base_deviation_cost +=
      weight * sample.base_deviation_cost;
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
  status->object_linear_disturbance_world_mps +=
      weight * sample.object_linear_disturbance_world_mps * average_scale;
  status->object_angular_disturbance_world_radps +=
      weight * sample.object_angular_disturbance_world_radps * average_scale;
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
  previous_qddot_residual_cmd_.resize(0);
  previous_tau_ff_cmd_.resize(0);
  has_previous_qddot_cmd_ = false;
  has_previous_qddot_residual_cmd_ = false;
  has_previous_tau_ff_cmd_ = false;
  status_ = ContinuousQddotMppiStatus{};
  initialized_ = true;
}

void ContinuousQddotMppiController::ResetNominalSequence() {
  if (nominal_sequence_.actionDim() == config_.rollout.action_dim &&
      nominal_sequence_.horizonSteps() == config_.rollout.horizon_steps) {
    nominal_sequence_.SetZero();
  }
  previous_qddot_cmd_.resize(0);
  previous_qddot_residual_cmd_.resize(0);
  previous_tau_ff_cmd_.resize(0);
  has_previous_qddot_cmd_ = false;
  has_previous_qddot_residual_cmd_ = false;
  has_previous_tau_ff_cmd_ = false;
}

BaseGraspControllerStatus
ContinuousQddotMppiController::ComputeBaseGraspCommand(
    const GraspState& initial_state,
    const RolloutContext& context) const {
  BaseGraspControllerStatus status;
  status.enabled = config_.base_grasp_controller.enabled;
  const std::size_t dim = config_.rollout.action_dim;
  status.qddot_base =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim));
  status.qddot_residual =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim));
  if (!status.enabled || !initial_state.valid ||
      initial_state.tactile_sensors.size() != context.tactile_contexts.size()) {
    return status;
  }

  std::vector<BaseSensorBasis> sensor_bases;
  sensor_bases.reserve(initial_state.tactile_sensors.size());
  for (std::size_t i = 0; i < initial_state.tactile_sensors.size(); ++i) {
    BaseSensorBasis basis = ComputeBaseSensorBasis(
        initial_state, initial_state.tactile_sensors[i],
        context.tactile_contexts[i], i, dim);
    if (basis.valid) {
      sensor_bases.push_back(std::move(basis));
    }
  }
  status.active_sensor_count = sensor_bases.size();
  if (sensor_bases.empty()) {
    return status;
  }

  Eigen::VectorXd squeeze_sum =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim));
  double force_sum = 0.0;
  status.min_force_n = std::numeric_limits<double>::infinity();
  status.max_force_n = 0.0;
  const BaseSensorBasis* thumb = nullptr;
  const BaseSensorBasis* index = nullptr;
  for (const auto& basis : sensor_bases) {
    squeeze_sum += basis.close_direction;
    force_sum += std::max(0.0, basis.force_n);
    status.min_force_n =
        std::min(status.min_force_n, std::max(0.0, basis.force_n));
    status.max_force_n =
        std::max(status.max_force_n, std::max(0.0, basis.force_n));
    if (basis.is_thumb && thumb == nullptr) {
      thumb = &basis;
    }
    if (basis.is_index && index == nullptr) {
      index = &basis;
    }
  }
  if (!std::isfinite(status.min_force_n)) {
    status.min_force_n = 0.0;
  }
  status.average_force_n =
      force_sum / static_cast<double>(sensor_bases.size());
  status.has_thumb = thumb != nullptr;
  status.has_index = index != nullptr;
  status.thumb_force_n =
      thumb != nullptr ? std::max(0.0, thumb->force_n) : 0.0;
  status.index_force_n =
      index != nullptr ? std::max(0.0, index->force_n) : 0.0;
  if (thumb != nullptr && index != nullptr) {
    status.average_force_n =
        0.5 * (status.thumb_force_n + status.index_force_n);
    status.min_force_n =
        std::min(status.thumb_force_n, status.index_force_n);
    status.max_force_n =
        std::max(status.thumb_force_n, status.index_force_n);
  }

  const Eigen::VectorXd squeeze_dir = NormalizedOrZero(squeeze_sum, dim);
  if (squeeze_dir.norm() <= 0.0) {
    return status;
  }

  Eigen::VectorXd qddot_base =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim));
  const auto& config = config_.base_grasp_controller;
  status.force_error_n =
      config.target_normal_force_n - status.average_force_n;
  const double weakest_force_deficit_n =
      std::max(0.0, config.min_normal_force_per_sensor_n -
                        status.min_force_n);
  qddot_base +=
      config.force_gain *
      (status.force_error_n + weakest_force_deficit_n) *
      squeeze_dir;

  if (config.use_force_balance && thumb != nullptr && index != nullptr) {
    status.force_balance_error_n =
        status.index_force_n - status.thumb_force_n;
    const Eigen::VectorXd balance_dir =
        NormalizedOrZero(thumb->close_direction - index->close_direction, dim);
    qddot_base +=
        config.force_balance_gain *
        status.force_balance_error_n *
        balance_dir;
  }

  if (
      config.use_high_force_guard &&
      status.max_force_n > config.max_normal_force_per_sensor_n) {
    status.high_force_guard_active = true;
    status.high_force_excess_n =
        status.max_force_n - config.max_normal_force_per_sensor_n;
    qddot_base -=
        config.high_force_release_gain *
        status.high_force_excess_n *
        squeeze_dir;
  }

  if (
      config.use_contact_loss_reflex &&
      status.active_sensor_count < 2U) {
    status.contact_loss_reflex_active = true;
    qddot_base += config.contact_loss_gain * squeeze_dir;
  }

  qddot_base =
      ClampVectorNorm(qddot_base, config.max_qddot_base);
  qddot_base =
      ClampVector(qddot_base, config_.rollout.action_lower_bound,
                  config_.rollout.action_upper_bound);
  status.qddot_base = qddot_base;
  status.qddot_base_norm = qddot_base.norm();
  status.active = status.qddot_base_norm > kTiny;
  return status;
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
    if (config_.base_grasp_controller.enabled) {
      sequence.values().col(step) =
          ClampVectorNorm(
              action, config_.base_grasp_controller.max_qddot_residual);
    } else {
      sequence.values().col(step) =
          ClampVector(action, config_.rollout.action_lower_bound,
                      config_.rollout.action_upper_bound);
    }
  }
  return sequence;
}

ContinuousQddotMppiController::SampleEvaluation
ContinuousQddotMppiController::EvaluateSequence(
    const GraspState& initial_state,
    const ActionSequence& sequence,
    const Eigen::VectorXd& qddot_base,
    const GraspDisturbanceSequence& disturbance_sequence,
    const RolloutContext& context) const {
  SampleEvaluation evaluation;
  if (!initial_state.valid ||
      sequence.actionDim() != config_.rollout.action_dim ||
      sequence.horizonSteps() != config_.rollout.horizon_steps ||
      qddot_base.size() !=
          static_cast<Eigen::Index>(config_.rollout.action_dim) ||
      !qddot_base.allFinite() ||
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
    const Eigen::VectorXd residual = sequence.action(step);
    const Eigen::VectorXd action = ClampVector(
        qddot_base + residual, config_.rollout.action_lower_bound,
        config_.rollout.action_upper_bound);
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
    const double base_deviation_cost =
        config_.base_grasp_controller.enabled
            ? config_.base_grasp_controller.base_deviation_weight *
                  (action - qddot_base).squaredNorm()
            : 0.0;
    const double step_cost =
        SanitizeCost(stage_cost + rate_cost + base_deviation_cost);
    total_cost = SanitizeCost(total_cost + step_cost);
    AccumulateStageBreakdown(
        breakdown, disturbance_step, rate_cost, base_deviation_cost,
        &evaluation.stats.costs);

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
    const Eigen::VectorXd& qddot_cmd) {
  RobotState next = StepRobotStateWithQddotLimits(
      initial_state.robot, qddot_cmd, context.robot_system,
      config_.rollout.dt, config_.limits);
  if (!IsValid(next)) {
    RobotCommand hold =
        MakeZeroHoldRobotCommand(initial_state.robot.q,
                                 initial_state.robot.qdot);
    hold.stamp_sec = observation.time_s;
    previous_tau_ff_cmd_ = Eigen::VectorXd::Zero(qddot_cmd.size());
    has_previous_tau_ff_cmd_ = previous_tau_ff_cmd_.size() > 0;
    return hold;
  }

  const RneaFeedforwardCommandResult tau_ff = ComputeRneaFeedforwardCommand(
      config_.rnea_feedforward, observation, initial_state, context,
      qddot_cmd, previous_tau_ff_cmd_, has_previous_tau_ff_cmd_,
      config_.rollout.dt);

  RobotCommand command;
  command.Resize(static_cast<int>(next.q.size()),
                 static_cast<int>(next.qdot.size()));
  command.q_cmd = next.q;
  command.qdot_cmd = next.qdot;
  command.tau_cmd = tau_ff.command;
  command.stamp_sec = observation.time_s;
  command.valid = command.HasValidDimensions() && command.AllFinite();
  previous_tau_ff_cmd_ = command.tau_cmd;
  has_previous_tau_ff_cmd_ =
      previous_tau_ff_cmd_.size() == static_cast<Eigen::Index>(
          config_.rollout.action_dim) &&
      previous_tau_ff_cmd_.allFinite();

  status_.use_rnea_feedforward = config_.rnea_feedforward.enabled;
  status_.tau_ff_scale = config_.rnea_feedforward.tau_ff_scale;
  status_.tau_ff_raw = tau_ff.raw;
  status_.tau_ff_scaled = tau_ff.scaled;
  status_.tau_ff_cmd = tau_ff.command;
  status_.tau_ff_raw_norm = tau_ff.raw.norm();
  status_.tau_ff_cmd_norm = tau_ff.command.norm();
  status_.tau_ff_max_abs = MaxAbsCoeff(tau_ff.command);
  status_.tau_ff_clamped = tau_ff.clamped;
  status_.tau_ff_rate_limited = tau_ff.rate_limited;
  status_.tau_ff_zeroed_not_ready = tau_ff.zeroed_not_ready;
  status_.tau_ff_zeroed_contact_loss = tau_ff.zeroed_contact_loss;
  return command;
}

void ContinuousQddotMppiController::ShiftUpdatedSequence(
    const Eigen::MatrixXd& updated_values) {
  Eigen::MatrixXd shifted_values = updated_values;
  if (config_.base_grasp_controller.enabled) {
    for (Eigen::Index step = 0; step < shifted_values.cols(); ++step) {
      shifted_values.col(step) =
          ClampVectorNorm(
              shifted_values.col(step),
              config_.base_grasp_controller.max_qddot_residual);
    }
  }
  nominal_sequence_.values() = shifted_values;
  if (nominal_sequence_.horizonSteps() <= 1U) {
    nominal_sequence_.SetZero();
    return;
  }
  nominal_sequence_.values().leftCols(nominal_sequence_.values().cols() - 1) =
      shifted_values.rightCols(shifted_values.cols() - 1).eval();
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
  status_.num_threads = ResolveEvaluationThreadCount(config_.rollout);
  status_.horizon_steps = config_.rollout.horizon_steps;
  status_.lambda = config_.rollout.temperature;
  status_.use_rnea_feedforward = config_.rnea_feedforward.enabled;
  status_.tau_ff_scale = config_.rnea_feedforward.tau_ff_scale;
  status_.object_edge_margin_m = std::numeric_limits<double>::infinity();
  status_.object_min_signed_distance_m =
      std::numeric_limits<double>::infinity();
  BaseGraspControllerStatus base_status =
      ComputeBaseGraspCommand(initial_state, context);
  Eigen::VectorXd qddot_base = base_status.qddot_base;
  if (qddot_base.size() !=
          static_cast<Eigen::Index>(config_.rollout.action_dim) ||
      !qddot_base.allFinite()) {
    qddot_base =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(
            config_.rollout.action_dim));
    base_status.qddot_base = qddot_base;
    base_status.qddot_base_norm = 0.0;
    base_status.active = false;
  }
  status_.base_grasp = base_status;
  status_.qddot_base = qddot_base;
  status_.qddot_residual_cmd =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(
          config_.rollout.action_dim));
  status_.qddot_nominal_first =
      ClampVector(qddot_base + nominal_sequence_.firstAction(),
                  config_.rollout.action_lower_bound,
                  config_.rollout.action_upper_bound);

  std::vector<ActionSequence> samples;
  samples.reserve(config_.rollout.num_rollouts);
  for (std::size_t i = 0; i < config_.rollout.num_rollouts; ++i) {
    samples.push_back(SampleSequence(i));
  }

  const auto disturbances = disturbance_sampler_.SampleBatch();
  status_.sampled_object_linear_disturbances_world_mps.clear();
  status_.sampled_object_angular_disturbances_world_radps.clear();
  status_.sampled_object_linear_disturbances_world_mps.reserve(
      disturbances.size());
  status_.sampled_object_angular_disturbances_world_radps.reserve(
      disturbances.size());
  for (const auto& disturbance_sequence : disturbances) {
    status_.sampled_object_linear_disturbances_world_mps.push_back(
        AverageObjectLinearDisturbance(disturbance_sequence));
    status_.sampled_object_angular_disturbances_world_radps.push_back(
        AverageObjectAngularDisturbance(disturbance_sequence));
  }
  std::vector<double> costs(samples.size(), kLargeCost);
  std::vector<SampleEvaluation> evaluations(samples.size());
  double cost_sum = 0.0;
  double cost_max = 0.0;
  std::size_t finite_count = 0U;
  std::size_t total_geometry_queries = 0U;
  std::size_t total_object_samples = 0U;

  if (status_.num_threads <= 1U || samples.size() <= 1U) {
    for (std::size_t i = 0; i < samples.size(); ++i) {
      evaluations[i] = EvaluateSequence(
          initial_state, samples[i], qddot_base, disturbances[i], context);
      costs[i] = SanitizeCost(evaluations[i].total_cost);
    }
  } else {
    const std::size_t worker_count =
        std::min<std::size_t>(status_.num_threads, samples.size());
    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    std::vector<std::exception_ptr> exceptions(worker_count);
    const std::size_t chunk_size =
        (samples.size() + worker_count - 1U) / worker_count;
    for (std::size_t worker_index = 0; worker_index < worker_count;
         ++worker_index) {
      const std::size_t begin = worker_index * chunk_size;
      const std::size_t end =
          std::min<std::size_t>(samples.size(), begin + chunk_size);
      if (begin >= end) {
        continue;
      }
      workers.emplace_back(
          [this, &initial_state, &samples, &qddot_base, &disturbances,
           &context, &evaluations, &costs, &exceptions, worker_index,
           begin, end]() {
            try {
              ThreadLocalRolloutWorkspace workspace;
              ConfigureThreadLocalRolloutWorkspace(context, &workspace);
              for (std::size_t i = begin; i < end; ++i) {
                evaluations[i] = EvaluateSequence(
                    initial_state, samples[i], qddot_base, disturbances[i],
                    workspace.context);
                costs[i] = SanitizeCost(evaluations[i].total_cost);
              }
            } catch (...) {
              exceptions[worker_index] = std::current_exception();
            }
          });
    }
    for (auto& worker : workers) {
      worker.join();
    }
    for (const auto& exception : exceptions) {
      if (exception) {
        std::rethrow_exception(exception);
      }
    }
  }

  for (std::size_t i = 0; i < samples.size(); ++i) {
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
    previous_tau_ff_cmd_ =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(
            config_.rollout.action_dim));
    previous_qddot_residual_cmd_ =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(
            config_.rollout.action_dim));
    has_previous_tau_ff_cmd_ = true;
    has_previous_qddot_residual_cmd_ = true;
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

  Eigen::VectorXd qddot_residual_cmd = updated.col(0);
  if (config_.base_grasp_controller.enabled) {
    qddot_residual_cmd =
        ClampVectorNorm(
            qddot_residual_cmd,
            config_.base_grasp_controller.max_qddot_residual);
  }
  if (has_previous_qddot_residual_cmd_ &&
      previous_qddot_residual_cmd_.size() == qddot_residual_cmd.size()) {
    qddot_residual_cmd =
        config_.smoothing_alpha * qddot_residual_cmd +
        (1.0 - config_.smoothing_alpha) * previous_qddot_residual_cmd_;
  } else if (!config_.base_grasp_controller.enabled &&
             has_previous_qddot_cmd_ &&
             previous_qddot_cmd_.size() == qddot_residual_cmd.size()) {
    qddot_residual_cmd =
        config_.smoothing_alpha * qddot_residual_cmd +
        (1.0 - config_.smoothing_alpha) * previous_qddot_cmd_;
  }
  if (config_.base_grasp_controller.enabled) {
    qddot_residual_cmd =
        ClampVectorNorm(
            qddot_residual_cmd,
            config_.base_grasp_controller.max_qddot_residual);
  } else {
    qddot_residual_cmd =
        ClampVector(qddot_residual_cmd, config_.rollout.action_lower_bound,
                    config_.rollout.action_upper_bound);
  }
  Eigen::VectorXd qddot_cmd = qddot_base + qddot_residual_cmd;
  qddot_cmd = ClampVector(qddot_cmd, config_.rollout.action_lower_bound,
                          config_.rollout.action_upper_bound);
  qddot_residual_cmd = qddot_cmd - qddot_base;

  status_.valid = true;
  status_.best_sample_index = best_index;
  status_.best_sample_cost = *best_it;
  status_.weighted_cost_estimate = weighted_cost;
  status_.nominal_sample_cost = costs.empty() ? kLargeCost : costs[0];
  status_.cost_min = *best_it;
  status_.cost_mean = cost_sum / static_cast<double>(finite_count);
  status_.cost_max = cost_max;
  status_.effective_sample_size =
      weight_square_sum > kTiny ? 1.0 / weight_square_sum : 0.0;
  status_.qddot_cmd = qddot_cmd;
  status_.qddot_residual_cmd = qddot_residual_cmd;
  status_.base_grasp.qddot_residual = qddot_residual_cmd;
  status_.base_grasp.qddot_residual_norm = qddot_residual_cmd.norm();
  status_.base_grasp.base_deviation_cost =
      config_.base_grasp_controller.enabled
          ? config_.base_grasp_controller.base_deviation_weight *
                qddot_residual_cmd.squaredNorm()
          : 0.0;
  status_.qddot_best_first =
      ClampVector(qddot_base + samples[best_index].firstAction(),
                  config_.rollout.action_lower_bound,
                  config_.rollout.action_upper_bound);
  status_.object_pose_rollout =
      WeightedObjectPoseRollout(evaluations, weights, best_index);
  if (best_index < evaluations.size()) {
    const auto& best_costs = evaluations[best_index].stats.costs;
    status_.object_linear_disturbance_world_mps =
        best_costs.objectLinearDisturbanceAverage();
    status_.object_angular_disturbance_world_radps =
        best_costs.objectAngularDisturbanceAverage();
    status_.object_linear_disturbance_speed_mps =
        best_costs.objectLinearDisturbanceSpeedAverage();
    status_.object_angular_disturbance_speed_radps =
        best_costs.objectAngularDisturbanceSpeedAverage();
  }
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
  previous_qddot_residual_cmd_ = qddot_residual_cmd;
  has_previous_qddot_cmd_ = true;
  has_previous_qddot_residual_cmd_ =
      previous_qddot_residual_cmd_.size() ==
          static_cast<Eigen::Index>(config_.rollout.action_dim) &&
      previous_qddot_residual_cmd_.allFinite();
  ShiftUpdatedSequence(updated);
  return command;
}

}  // namespace mppi_core
