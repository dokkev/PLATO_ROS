// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <vector>

#include "mppi_core/core/action_sequence.hpp"
#include "mppi_core/rollout/rollout_model.hpp"
#include "mppi_core/state/grasp_state.hpp"

namespace mppi_core {

struct GraspActionLibraryConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t horizon_steps{20};
  std::size_t action_dim{0};

  double dt{0.01};

  double squeeze_light_accel_scale{2.0};
  double squeeze_medium_accel_scale{5.0};
  double squeeze_strong_accel_scale{10.0};
  double release_accel_scale{2.0};

  double align_accel_scale{5.0};
  double sequence_decay{0.95};

  Eigen::VectorXd qddot_lower_bound;
  Eigen::VectorXd qddot_upper_bound;
};

class GraspActionLibrary {
 public:
  explicit GraspActionLibrary(GraspActionLibraryConfig config);

  std::vector<ActionSequence> BuildCandidates(
      const GraspState& state, const RolloutContext& context) const;

 private:
  ActionSequence BuildDecayedSequence(
      const Eigen::Ref<const Eigen::VectorXd>& first_action) const;
  Eigen::VectorXd ClampAction(const Eigen::VectorXd& action) const;
  bool BuildSqueezeDirection(const GraspState& state,
                             const RolloutContext& context,
                             Eigen::VectorXd* direction) const;

  GraspActionLibraryConfig config_;
};

}  // namespace mppi_core
