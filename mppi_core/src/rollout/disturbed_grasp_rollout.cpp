// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/rollout/disturbed_grasp_rollout.hpp"

#include <cmath>

#include "mppi_core/contact/contact_kinematics.hpp"

namespace mppi_core {
namespace {

bool HasMatchingShape(const GraspState& state,
                      const GraspDisturbanceStep& disturbance,
                      const RolloutContext& context) {
  if (!state.valid || !disturbance.valid ||
      state.tactile_sensors.size() != context.tactile_contexts.size() ||
      state.tactile_sensors.size() !=
          disturbance.tactile_sensor_disturbances.size()) {
    return false;
  }
  for (std::size_t i = 0; i < state.tactile_sensors.size(); ++i) {
    const auto& tactile = state.tactile_sensors[i];
    const auto& tactile_context = context.tactile_contexts[i];
    if (tactile.sensor_index >= 0 && tactile_context.sensor_index >= 0 &&
        tactile.sensor_index != tactile_context.sensor_index) {
      return false;
    }
  }
  return true;
}

}  // namespace

DisturbedRolloutStepResult StepGraspStateWithDisturbance(
    const GraspState& state,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
    const GraspDisturbanceStep& disturbance,
    const RolloutContext& context,
    const DisturbedGraspRolloutConfig& config, const double dt) {
  DisturbedRolloutStepResult result;
  if (!std::isfinite(dt) || dt <= 0.0 || !qddot_sol.allFinite() ||
      !HasMatchingShape(state, disturbance, context)) {
    return result;
  }

  result.next_state.robot =
      StepRobotState(state.robot, qddot_sol, context.robot_system, dt);
  result.next_state.tactile_sensors = state.tactile_sensors;
  if (!IsValid(result.next_state.robot)) {
    return result;
  }

  DisturbedTactileTransitionConfig transition_config =
      config.tactile_transition;
  if (context.tactile_transition_config != nullptr) {
    transition_config.base = *context.tactile_transition_config;
  }

  for (std::size_t i = 0; i < state.tactile_sensors.size(); ++i) {
    const auto& tactile = state.tactile_sensors[i];
    const auto& tactile_context = context.tactile_contexts[i];
    const auto& tactile_disturbance =
        disturbance.tactile_sensor_disturbances[i];
    if (!tactile.valid ||
        tactile_context.kinematics == nullptr ||
        !IsValidContactKinematicsContext(*tactile_context.kinematics)) {
      return result;
    }

    const auto motions = ComputeHemisphereMotions(
        state.robot, result.next_state.robot, tactile, tactile_context, dt);
    if (motions.size() != tactile.hemispheres.size()) {
      return result;
    }

    result.next_state.tactile_sensors[i] =
        StepTactileStateWithDisturbance(
            tactile, state.robot, result.next_state.robot, motions,
            tactile_context, transition_config, tactile_disturbance, dt);
    if (!result.next_state.tactile_sensors[i].valid) {
      return result;
    }
  }

  result.next_state.valid =
      IsValid(result.next_state.robot) &&
      HasValidTactileSensors(result.next_state);
  result.valid = result.next_state.valid;
  return result;
}

}  // namespace mppi_core
