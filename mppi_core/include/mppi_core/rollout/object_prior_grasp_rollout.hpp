// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>

#include "mppi_core/disturbance/grasp_disturbance.hpp"
#include "mppi_core/object/object_contact_belief.hpp"
#include "mppi_core/rollout/grasp_state_rollout_model.hpp"
#include "mppi_core/rollout/rollout_model.hpp"
#include "mppi_core/state/grasp_state.hpp"

namespace mppi_core {

struct ObjectPriorRolloutStepResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraspState next_state;
  bool valid{false};
};

VirtualObjectBelief StepVirtualObjectBelief(
    const VirtualObjectBelief& belief,
    const VirtualObjectDisturbance& disturbance,
    double dt);

ObjectPriorRolloutStepResult StepObjectPriorGraspState(
    const GraspState& state,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
    const GraspDisturbanceStep& disturbance,
    const RolloutContext& context,
    double dt);

}  // namespace mppi_core
