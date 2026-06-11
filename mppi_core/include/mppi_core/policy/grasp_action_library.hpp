// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "mppi_core/core/action_sequence.hpp"
#include "mppi_core/rollout/rollout_model.hpp"
#include "mppi_core/state/grasp_state.hpp"

namespace mppi_core {

struct GraspCorrectiveAction {
  double squeeze{0.0};
  double release{0.0};
  double align_lateral{0.0};
  double force_balance{0.0};
  double thumb_bias{0.0};
  double index_bias{0.0};
};

struct GraspActionLibraryConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t horizon_steps{20};
  std::size_t action_dim{0};
  std::size_t num_action_samples{32};
  std::uint32_t random_seed{11};

  double dt{0.01};

  double target_min_normal_force_n{1.0};
  double force_to_squeeze_gain{4.0};
  double force_balance_gain{2.0};
  double contact_line_align_gain{20.0};

  double squeeze_std{2.0};
  double align_std{2.0};
  double force_balance_std{2.0};
  double thumb_bias_std{1.0};
  double index_bias_std{1.0};

  double max_squeeze{10.0};
  double max_align{10.0};
  double max_force_balance{10.0};
  double max_thumb_bias{10.0};
  double max_index_bias{10.0};

  // Debug/smoke-test basis probes. The sampled coefficient policy is the main
  // path; these deterministic probes make sign mistakes visible in logs/tests.
  bool include_basis_probe_actions{true};
  double squeeze_light_accel_scale{2.0};
  double squeeze_medium_accel_scale{5.0};
  double squeeze_strong_accel_scale{10.0};
  double release_accel_scale{2.0};

  double align_accel_scale{5.0};
  double sequence_decay{0.95};
  Eigen::Vector3d close_axis_base{0.0, 0.0, 1.0};

  Eigen::VectorXd qddot_lower_bound;
  Eigen::VectorXd qddot_upper_bound;
};

class GraspActionLibrary {
 public:
  explicit GraspActionLibrary(GraspActionLibraryConfig config);

  std::vector<ActionSequence> BuildCandidates(
      const GraspState& state, const RolloutContext& context);

 private:
  struct GraspActionBasis;

  ActionSequence BuildDecayedSequence(
      const Eigen::Ref<const Eigen::VectorXd>& first_action) const;
  Eigen::VectorXd ClampAction(const Eigen::VectorXd& action) const;
  bool BuildActionBasis(const GraspState& state,
                        const RolloutContext& context,
                        GraspActionBasis* basis) const;
  GraspCorrectiveAction BuildNominalAction(
      const GraspActionBasis& basis) const;
  Eigen::VectorXd MapActionToQddot(
      const GraspCorrectiveAction& action,
      const GraspActionBasis& basis) const;
  GraspCorrectiveAction SampleActionAround(
      const GraspCorrectiveAction& nominal);
  void AddBasisProbeActions(
      const GraspActionBasis& basis,
      std::vector<ActionSequence>* candidates) const;

  GraspActionLibraryConfig config_;
  std::mt19937 rng_;
};

}  // namespace mppi_core
