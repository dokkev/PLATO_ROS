// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/rollout/object_prior_grasp_rollout.hpp"

#include <Eigen/Geometry>

#include <cmath>

namespace mppi_core {
namespace {

Eigen::Matrix3d AngularVelocityToRotation(
    const Eigen::Vector3d& angular_velocity_world_radps, const double dt) {
  if (!angular_velocity_world_radps.allFinite() || !std::isfinite(dt) ||
      dt <= 0.0) {
    return Eigen::Matrix3d::Identity();
  }
  const Eigen::Vector3d rotation_vector = angular_velocity_world_radps * dt;
  const double angle = rotation_vector.norm();
  if (angle <= 1.0e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

bool HasFiniteDisturbance(const VirtualObjectDisturbance& disturbance) {
  return disturbance.valid &&
         disturbance.linear_velocity_world_mps.allFinite() &&
         disturbance.angular_velocity_world_radps.allFinite();
}

}  // namespace

VirtualObjectBelief StepVirtualObjectBelief(
    const VirtualObjectBelief& belief,
    const VirtualObjectDisturbance& disturbance,
    const double dt) {
  if (!IsValidVirtualObjectBelief(belief) ||
      !HasFiniteDisturbance(disturbance) ||
      !std::isfinite(dt) || dt <= 0.0) {
    return {};
  }
  if (!HasVirtualObjectBelief(belief)) {
    return belief;
  }

  VirtualObjectBelief next = belief;
  const Eigen::Vector3d translation_step_m =
      disturbance.linear_velocity_world_mps * dt;
  const Eigen::Matrix3d rotation_step =
      AngularVelocityToRotation(disturbance.angular_velocity_world_radps, dt);

  for (auto& particle : next.particles) {
    if (!IsValidVirtualObjectState(particle)) {
      next.valid = false;
      return {};
    }
    particle.pose_world.translation() += translation_step_m;
    particle.pose_world.linear() = rotation_step * particle.pose_world.linear();
    particle.velocity_world.head<3>() = disturbance.linear_velocity_world_mps;
    particle.velocity_world.tail<3>() = disturbance.angular_velocity_world_radps;
  }
  next.valid = IsValidVirtualObjectBelief(next);
  return next;
}

ObjectPriorRolloutStepResult StepObjectPriorGraspState(
    const GraspState& state,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
    const GraspDisturbanceStep& disturbance,
    const RolloutContext& context,
    const double dt) {
  ObjectPriorRolloutStepResult result;
  if (!state.valid || !disturbance.valid || !qddot_sol.allFinite() ||
      !std::isfinite(dt) || dt <= 0.0) {
    return result;
  }

  result.next_state.robot =
      StepRobotState(state.robot, qddot_sol, context.robot_system, dt);
  result.next_state.tactile_sensors = state.tactile_sensors;
  result.next_state.object_belief =
      StepVirtualObjectBelief(
          state.object_belief, disturbance.object_disturbance, dt);

  result.next_state.valid =
      IsValid(result.next_state.robot) &&
      HasValidTactileSensors(result.next_state) &&
      IsValidVirtualObjectBelief(result.next_state.object_belief);
  result.valid = result.next_state.valid;
  return result;
}

}  // namespace mppi_core
