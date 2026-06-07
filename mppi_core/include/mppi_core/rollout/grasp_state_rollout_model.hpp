// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cstddef>

#include "mppi_core/rollout/rollout_model.hpp"

namespace mppi_core {

struct GraspStateRolloutConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Reserved for rollout-model parameters. Residual force projection is not a
  // GraspState rollout policy.
};

RobotState StepRobotState(const RobotState& robot,
                          const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
                          RobotSystem* robot_system, double dt);

class GraspStateRolloutModel final : public RolloutModelBase {
 public:
  explicit GraspStateRolloutModel(std::size_t joint_dim);
  GraspStateRolloutModel(std::size_t joint_dim, GraspStateRolloutConfig config);

  std::size_t actionDim() const override { return joint_dim_; }

  void Step(const GraspState& state,
            const Eigen::Ref<const Eigen::VectorXd>& action,
            const RolloutContext& context, double dt,
            GraspState* next_state) const override;

 private:
  std::size_t joint_dim_{0};
};

}  // namespace mppi_core
