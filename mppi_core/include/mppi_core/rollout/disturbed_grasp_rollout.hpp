// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>

#include "mppi_core/disturbance/grasp_disturbance.hpp"
#include "mppi_core/rollout/grasp_state_rollout_model.hpp"
#include "mppi_core/rollout/rollout_model.hpp"
#include "mppi_core/tactile/disturbed_tactile_transition.hpp"

namespace mppi_core {

struct TactileOnlyContactTransitionConfig {
  DisturbedTactileTransitionConfig tactile_transition;
};

using DisturbedGraspRolloutConfig = TactileOnlyContactTransitionConfig;

struct DisturbedRolloutStepResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraspState next_state;
  bool valid{false};
};

DisturbedRolloutStepResult StepTactileOnlyGraspStateWithDisturbance(
    const GraspState& state,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
    const GraspDisturbanceStep& disturbance,
    const RolloutContext& context,
    const TactileOnlyContactTransitionConfig& config, double dt);

DisturbedRolloutStepResult StepGraspStateWithDisturbance(
    const GraspState& state,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
    const GraspDisturbanceStep& disturbance,
    const RolloutContext& context,
    const TactileOnlyContactTransitionConfig& config, double dt);

}  // namespace mppi_core
