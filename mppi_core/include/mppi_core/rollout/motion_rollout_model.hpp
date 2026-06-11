// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cstddef>

#include "mppi_core/rollout/rollout_model.hpp"

namespace mppi_core {

class MotionRolloutModel final : public RolloutModelBase {
 public:
  explicit MotionRolloutModel(std::size_t action_dim);

  std::size_t actionDim() const override { return action_dim_; }

  void Step(const GraspState& state,
            const Eigen::Ref<const Eigen::VectorXd>& action,
            const RolloutContext& context, double dt,
            GraspState* next_state) const override;

 private:
  std::size_t action_dim_{0};
};

}  // namespace mppi_core
