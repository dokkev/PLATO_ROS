// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/rollout/grasp_state_rollout_model.hpp"

#include <cmath>
#include <cstddef>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

#include "mppi_core/contact/contact_force_correction.hpp"
#include "mppi_core/contact/contact_force_projection.hpp"
#include "mppi_core/contact/contact_kinematics.hpp"
#include "mppi_core/rollout/contact_force_rollout.hpp"

namespace mppi_core {
namespace {

TactileState InitialTactilePrediction(const RolloutContext& context,
                                      std::size_t tactile_sensor) {
  if (context.observation == nullptr ||
      tactile_sensor >= context.observation->tactile_meas.size()) {
    return TactileState{};
  }
  return context.observation->tactile_meas[tactile_sensor];
}

bool HasActiveTactileHemisphere(const TactileState& tactile) {
  return tactile.hasActiveHemisphereContact();
}

std::size_t ActiveTactileSensorCount(
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors) {
  std::size_t count = 0;
  for (const auto& tactile : tactile_sensors) {
    if (HasActiveTactileHemisphere(tactile)) {
      ++count;
    }
  }
  return count;
}

bool HasMatchingSensorIndex(const TactileState& tactile,
                            const TactileSensorContext& context) {
  if (tactile.sensor_index < 0 || context.sensor_index < 0) {
    return true;
  }
  return tactile.sensor_index == context.sensor_index;
}

bool HasMatchingTactileContextShape(
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const RolloutContext& context) {
  if (tactile_sensors.size() != context.tactile_contexts.size()) {
    return false;
  }
  for (std::size_t i = 0; i < tactile_sensors.size(); ++i) {
    if (!HasMatchingSensorIndex(tactile_sensors[i],
                                context.tactile_contexts[i])) {
      return false;
    }
  }
  return true;
}

bool ShouldTryKinematicPatchFallback(TactileRolloutPolicy policy) {
  return policy == TactileRolloutPolicy::kResidualThenKinematicFallback;
}

TactileRolloutPolicy EffectiveTactileRolloutPolicy(
    TactileRolloutPolicy configured_policy,
    bool residual_projection_supported) {
  if (residual_projection_supported) {
    return configured_policy;
  }
  return TactileRolloutPolicy::kResidualThenKinematicFallback;
}

bool CanUsePinocchioState(const Eigen::Ref<const Eigen::VectorXd>& q,
                          const Eigen::Ref<const Eigen::VectorXd>& v,
                          const Eigen::Ref<const Eigen::VectorXd>& a,
                          const RobotDynamicsContext* context) {
  return context != nullptr && IsValidRobotDynamicsContext(*context) &&
         q.size() == static_cast<Eigen::Index>(context->model->nq) &&
         v.size() == static_cast<Eigen::Index>(context->model->nv) &&
         a.size() == static_cast<Eigen::Index>(context->model->nv);
}

Eigen::VectorXd IntegrateReference(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& tangent_step,
    const RobotDynamicsContext* context) {
  if (CanUsePinocchioState(q, tangent_step, tangent_step, context)) {
    return pinocchio::integrate(*context->model, q, tangent_step);
  }
  if (q.size() != tangent_step.size()) {
    throw std::invalid_argument(
        "GraspStateRolloutModel::Step: q and action dimension mismatch");
  }
  return q + tangent_step;
}

Eigen::VectorXd RneaOrZero(const Eigen::Ref<const Eigen::VectorXd>& q,
                           const Eigen::Ref<const Eigen::VectorXd>& v,
                           const Eigen::Ref<const Eigen::VectorXd>& a,
                           const RobotDynamicsContext* context) {
  Eigen::VectorXd tau_ff = Eigen::VectorXd::Zero(a.size());
  if (!CanUsePinocchioState(q, v, a, context)) {
    return tau_ff;
  }

  const Eigen::VectorXd rnea =
      pinocchio::rnea(*context->model, *context->data, q, v, a);
  if (rnea.size() == tau_ff.size() && rnea.allFinite()) {
    tau_ff = rnea;
  }
  return tau_ff;
}

bool TryMeasuredTorqueResidual(const GraspObservation& observation,
                               const RobotDynamicsContext& dynamics,
                               Eigen::VectorXd* tau_residual) {
  if (tau_residual == nullptr ||
      !CanUsePinocchioState(observation.q_meas, observation.qdot_meas,
                            observation.tau_meas, &dynamics)) {
    return false;
  }

  const Eigen::VectorXd zero_acceleration =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dynamics.model->nv));
  const Eigen::VectorXd tau_model =
      pinocchio::rnea(*dynamics.model, *dynamics.data, observation.q_meas,
                      observation.qdot_meas, zero_acceleration);
  if (tau_model.size() != observation.tau_meas.size() ||
      !tau_model.allFinite()) {
    return false;
  }

  *tau_residual = observation.tau_meas - tau_model;
  return tau_residual->allFinite();
}

bool TryResidualAwareTactileTransition(
    const GraspState& state, const TactileState& tactile,
    const TactileSensorContext& tactile_context, bool allow_residual_projection,
    const RolloutContext& context, double dt, TactileState* tactile_out) {
  const PinocchioContactKinematicsContext* kinematics =
      tactile_context.kinematics;
  if (!allow_residual_projection || tactile_out == nullptr ||
      kinematics == nullptr || !IsValidRobotState(state.robot) ||
      !tactile.valid || !HasActiveTactileHemisphere(tactile) ||
      !IsValidContactKinematicsContext(*kinematics) ||
      context.robot_dynamics == nullptr ||
      !IsValidRobotDynamicsContext(*context.robot_dynamics)) {
    return false;
  }

  const auto& selected_kinematics = *kinematics;
  if (state.robot.q_des.size() !=
          static_cast<Eigen::Index>(selected_kinematics.model->nq) ||
      !state.robot.q_des.allFinite()) {
    return false;
  }

  const ContactForceProjectionConfig projection_config =
      context.contact_force_projection_config != nullptr
          ? *context.contact_force_projection_config
          : ContactForceProjectionConfig{};
  const ContactForceRolloutConfig transition_config =
      context.contact_force_rollout_config != nullptr
          ? *context.contact_force_rollout_config
          : ContactForceRolloutConfig{};
  if (!projection_config.enabled ||
      !transition_config.enable_force_projection_update) {
    return false;
  }

  Eigen::VectorXd tau_residual;
  if (context.observation == nullptr ||
      !TryMeasuredTorqueResidual(*context.observation, *context.robot_dynamics,
                                 &tau_residual)) {
    return false;
  }

  RobotState projection_robot = state.robot;
  if (context.observation->q_meas.size() ==
          static_cast<Eigen::Index>(selected_kinematics.model->nq) &&
      context.observation->q_meas.allFinite()) {
    projection_robot.q_des = context.observation->q_meas;
  }

  ContactForceProjectionResult projection =
      ProjectContactForcesFromTorqueResidual(projection_robot, tactile,
                                             tau_residual, selected_kinematics,
                                             projection_config);
  if (!projection.valid) {
    return false;
  }

  if (context.contact_force_correction_state != nullptr) {
    projection.total_normal_force_n =
        ApplyContactForceCorrection(projection.total_normal_force_n,
                                    *context.contact_force_correction_state);
  }

  *tactile_out = tactile;
  StepTactileStateFromProjectedForce(projection, dt, transition_config,
                                     tactile_out);
  return tactile_out->valid;
}

bool HasValidStateVectors(const GraspState& state) {
  return HasValidTactileSensors(state) &&
         state.robot.qdot_des.size() == state.robot.qddot_des.size() &&
         state.robot.qdot_des.size() == state.robot.tau_ff.size() &&
         state.robot.q_des.allFinite() && state.robot.qdot_des.allFinite() &&
         state.robot.qddot_des.allFinite() && state.robot.tau_ff.allFinite();
}

bool StepOneTactileState(const GraspState& robot_rollout_state,
                         const TactileState& tactile_seed,
                         const Eigen::Ref<const Eigen::VectorXd>& tangent_step,
                         const TactileSensorContext& tactile_context,
                         bool allow_residual_projection,
                         const RolloutContext& context,
                         TactileRolloutPolicy rollout_policy, double dt,
                         TactileState* tactile_out) {
  if (tactile_out == nullptr) {
    return false;
  }
  if (!tactile_seed.valid) {
    *tactile_out = tactile_seed;
    return false;
  }
  if (!HasActiveTactileHemisphere(tactile_seed)) {
    *tactile_out = tactile_seed;
    return true;
  }

  if (TryResidualAwareTactileTransition(
          robot_rollout_state, tactile_seed, tactile_context,
          allow_residual_projection, context, dt, tactile_out)) {
    return true;
  }

  if (rollout_policy == TactileRolloutPolicy::kResidualRequired) {
    *tactile_out = tactile_seed;
    return false;
  }

  if (!ShouldTryKinematicPatchFallback(rollout_policy) ||
      tactile_context.kinematics == nullptr ||
      !IsValidContactKinematicsInput(robot_rollout_state.robot, tactile_seed,
                                     tangent_step,
                                     *tactile_context.kinematics)) {
    *tactile_out = tactile_seed;
    return false;
  }

  const GraspRolloutConfig rollout_config =
      context.grasp_rollout_config != nullptr ? *context.grasp_rollout_config
                                              : GraspRolloutConfig{};
  const auto motions =
      ComputeHemisphereMotions(robot_rollout_state.robot, tactile_seed,
                               tangent_step, *tactile_context.kinematics);
  *tactile_out =
      StepTactileTransition(tactile_seed, motions, dt, rollout_config);
  return tactile_out->valid;
}

}  // namespace

GraspStateRolloutModel::GraspStateRolloutModel(std::size_t joint_dim)
    : GraspStateRolloutModel(joint_dim, GraspStateRolloutConfig{}) {}

GraspStateRolloutModel::GraspStateRolloutModel(std::size_t joint_dim,
                                               GraspStateRolloutConfig config)
    : joint_dim_(joint_dim), config_(std::move(config)) {
  if (joint_dim_ == 0) {
    throw std::invalid_argument(
        "GraspStateRolloutModel: joint_dim must be nonzero");
  }
}

void GraspStateRolloutModel::Step(
    const GraspState& state, const Eigen::Ref<const Eigen::VectorXd>& action,
    const RolloutContext& context, double dt, GraspState* next_state) const {
  if (next_state == nullptr) {
    throw std::invalid_argument(
        "GraspStateRolloutModel::Step: next_state is null");
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument(
        "GraspStateRolloutModel::Step: dt must be positive");
  }
  if (state.robot.qdot_des.size() != static_cast<Eigen::Index>(joint_dim_) ||
      state.robot.qddot_des.size() != state.robot.qdot_des.size() ||
      state.robot.tau_ff.size() != state.robot.qdot_des.size() ||
      action.size() != static_cast<Eigen::Index>(joint_dim_)) {
    throw std::invalid_argument(
        "GraspStateRolloutModel::Step: dimension mismatch");
  }
  if (!state.robot.q_des.allFinite() || !state.robot.qdot_des.allFinite() ||
      !state.robot.qddot_des.allFinite() || !state.robot.tau_ff.allFinite() ||
      !action.allFinite()) {
    throw std::invalid_argument(
        "GraspStateRolloutModel::Step: state and action must be finite");
  }

  const Eigen::VectorXd qddot_des = action;
  const Eigen::VectorXd qdot_des = state.robot.qdot_des + qddot_des * dt;
  const Eigen::VectorXd tangent_step = qdot_des * dt;
  const Eigen::VectorXd q_des = IntegrateReference(
      state.robot.q_des, tangent_step, context.robot_dynamics);
  const Eigen::VectorXd tau_ff =
      RneaOrZero(q_des, qdot_des, qddot_des, context.robot_dynamics);

  std::vector<TactileState, Eigen::aligned_allocator<TactileState>>
      tactile_seeds = state.tactile_sensors;
  for (std::size_t i = 0; i < tactile_seeds.size(); ++i) {
    if (!tactile_seeds[i].valid) {
      tactile_seeds[i] = InitialTactilePrediction(context, i);
    }
  }

  *next_state =
      MakeGraspState(q_des, qdot_des, qddot_des, tau_ff, tactile_seeds);
  next_state->tactile_sensors.resize(tactile_seeds.size());
  if (!HasMatchingTactileContextShape(tactile_seeds, context)) {
    next_state->valid = false;
    return;
  }

  const TactileRolloutPolicy rollout_policy = config_.tactile_rollout_policy;
  const bool residual_projection_supported =
      ActiveTactileSensorCount(tactile_seeds) == 1;
  const TactileRolloutPolicy effective_rollout_policy =
      EffectiveTactileRolloutPolicy(rollout_policy,
                                    residual_projection_supported);
  bool tactile_valid = true;
  for (std::size_t i = 0; i < tactile_seeds.size(); ++i) {
    tactile_valid =
        StepOneTactileState(
            *next_state, tactile_seeds[i], tangent_step,
            context.tactile_contexts[i], residual_projection_supported, context,
            effective_rollout_policy, dt, &next_state->tactile_sensors[i]) &&
        tactile_valid;
  }

  next_state->valid = tactile_valid && HasValidStateVectors(*next_state) &&
                      next_state->activeTactileSensorCount() > 0;
}

}  // namespace mppi_core
