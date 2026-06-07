// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/rollout/grasp_state_rollout_model.hpp"

#include <cmath>
#include <cstddef>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <stdexcept>
#include <vector>

#include "mppi_core/contact/contact_kinematics.hpp"

namespace mppi_core {
namespace {

bool CanUsePinocchioConfiguration(const Eigen::Ref<const Eigen::VectorXd>& q,
                                  const Eigen::Ref<const Eigen::VectorXd>& v,
                                  const RobotSystem* robot_system) {
  return robot_system != nullptr && robot_system->hasModel() &&
         robot_system->data().oMi.size() ==
             robot_system->model().joints.size() &&
         q.size() == static_cast<Eigen::Index>(robot_system->nq()) &&
         v.size() == static_cast<Eigen::Index>(robot_system->nv());
}

bool CanUsePinocchioState(const Eigen::Ref<const Eigen::VectorXd>& q,
                          const Eigen::Ref<const Eigen::VectorXd>& v,
                          const Eigen::Ref<const Eigen::VectorXd>& a,
                          const RobotSystem* robot_system) {
  return CanUsePinocchioConfiguration(q, v, robot_system) &&
         a.size() == static_cast<Eigen::Index>(robot_system->nv());
}

Eigen::VectorXd IntegrateReference(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& tangent_step,
    const RobotSystem* robot_system) {
  if (CanUsePinocchioConfiguration(q, tangent_step, robot_system)) {
    return pinocchio::integrate(robot_system->model(), q, tangent_step);
  }
  if (q.size() != tangent_step.size()) {
    return {};
  }
  return q + tangent_step;
}

Eigen::VectorXd RneaOrZero(const Eigen::Ref<const Eigen::VectorXd>& q,
                           const Eigen::Ref<const Eigen::VectorXd>& v,
                           const Eigen::Ref<const Eigen::VectorXd>& a,
                           RobotSystem* robot_system) {
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(a.size());
  if (!CanUsePinocchioState(q, v, a, robot_system)) {
    return tau;
  }

  const Eigen::VectorXd rnea =
      pinocchio::rnea(robot_system->model(), robot_system->data(), q, v, a);
  if (rnea.size() == tau.size() && rnea.allFinite()) {
    tau = rnea;
  }
  return tau;
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

bool HasValidStateVectors(const GraspState& state) {
  return IsValid(state.robot) && HasValidTactileSensors(state) &&
         state.robot.qdot.size() == state.robot.tau.size() &&
         state.robot.q.allFinite() && state.robot.qdot.allFinite() &&
         state.robot.tau.allFinite();
}

}  // namespace

RobotState StepRobotState(const RobotState& robot,
                          const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
                          RobotSystem* robot_system, double dt) {
  RobotState next;
  if (!IsValid(robot) || !std::isfinite(dt) || dt <= 0.0 ||
      qddot_sol.size() != robot.qdot.size() || !qddot_sol.allFinite()) {
    return next;
  }

  next.qdot = robot.qdot + qddot_sol * dt;
  const Eigen::VectorXd tangent_step = next.qdot * dt;
  next.q = IntegrateReference(robot.q, tangent_step, robot_system);
  if (next.q.size() == 0 || !next.q.allFinite() || !next.qdot.allFinite()) {
    return RobotState{};
  }

  // In rollout, RobotState::tau is the model torque for the current state and
  // solver acceleration. It is not a command-layer tau_cmd or tau_ff_cmd.
  next.tau = RneaOrZero(robot.q, robot.qdot, qddot_sol, robot_system);
  next.time_s = robot.time_s + dt;
  next.valid = next.q.allFinite() && next.qdot.allFinite() &&
               next.tau.allFinite() && next.qdot.size() == next.tau.size() &&
               std::isfinite(next.time_s);
  return next;
}

GraspStateRolloutModel::GraspStateRolloutModel(std::size_t joint_dim)
    : GraspStateRolloutModel(joint_dim, GraspStateRolloutConfig{}) {}

GraspStateRolloutModel::GraspStateRolloutModel(std::size_t joint_dim,
                                               GraspStateRolloutConfig)
    : joint_dim_(joint_dim) {
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
  next_state->valid = false;
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument(
        "GraspStateRolloutModel::Step: dt must be positive");
  }
  if (action.size() != static_cast<Eigen::Index>(joint_dim_)) {
    throw std::invalid_argument(
        "GraspStateRolloutModel::Step: action dimension mismatch");
  }
  if (!action.allFinite()) {
    throw std::invalid_argument(
        "GraspStateRolloutModel::Step: action must be finite");
  }
  if (!state.valid || !IsValid(state.robot) ||
      state.robot.qdot.size() != static_cast<Eigen::Index>(joint_dim_) ||
      !HasMatchingTactileContextShape(state.tactile_sensors, context)) {
    *next_state = state;
    next_state->valid = false;
    return;
  }

  // Residual-based tactile correction is intentionally disabled in the current
  // rollout. Residual will later be handled as observation-time GraspState
  // correction, not as horizon dynamics.
  const TactileTransitionConfig transition_config =
      context.tactile_transition_config != nullptr
          ? *context.tactile_transition_config
          : TactileTransitionConfig{};

  next_state->robot =
      StepRobotState(state.robot, action, context.robot_system, dt);
  next_state->tactile_sensors = state.tactile_sensors;
  if (!IsValid(next_state->robot)) {
    next_state->valid = false;
    return;
  }

  for (std::size_t i = 0; i < state.tactile_sensors.size(); ++i) {
    const auto& tactile = state.tactile_sensors[i];
    const auto& tactile_context = context.tactile_contexts[i];
    if (!tactile.valid ||
        !HasMatchingSensorIndex(tactile, tactile_context)) {
      next_state->valid = false;
      return;
    }

    const auto motions = ComputeHemisphereMotions(
        state.robot, next_state->robot, tactile, tactile_context, dt);
    if (motions.size() != tactile.hemispheres.size()) {
      next_state->valid = false;
      return;
    }

    next_state->tactile_sensors[i] = StepTactileState(
        tactile, state.robot, next_state->robot, motions, tactile_context,
        transition_config, dt);
    if (!next_state->tactile_sensors[i].valid) {
      next_state->valid = false;
      return;
    }
  }

  next_state->valid = HasValidStateVectors(*next_state);
}

}  // namespace mppi_core
