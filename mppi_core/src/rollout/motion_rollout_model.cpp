// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/rollout/motion_rollout_model.hpp"

#include <cmath>
#include <stdexcept>

#include "mppi_core/rollout/grasp_state_rollout_model.hpp"

namespace mppi_core {

MotionRolloutModel::MotionRolloutModel(const std::size_t action_dim)
    : action_dim_(action_dim) {
  if (action_dim_ == 0) {
    throw std::invalid_argument("MotionRolloutModel: action_dim must be nonzero");
  }
}

void MotionRolloutModel::Step(
    const GraspState& state, const Eigen::Ref<const Eigen::VectorXd>& action,
    const RolloutContext& context, const double dt,
    GraspState* next_state) const {
  if (next_state == nullptr) {
    return;
  }

  *next_state = GraspState{};
  next_state->valid = false;

  if (!state.valid || !IsValid(state.robot) ||
      action.size() != static_cast<Eigen::Index>(action_dim_) ||
      !action.allFinite() || context.robot_system == nullptr ||
      !std::isfinite(dt) || dt <= 0.0) {
    return;
  }

  next_state->robot =
      StepRobotState(state.robot, action, context.robot_system, dt);
  next_state->tactile_sensors = state.tactile_sensors;
  next_state->valid = IsValid(next_state->robot);
}

}  // namespace mppi_core
