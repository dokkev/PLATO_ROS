// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <memory>
#include <vector>

#include "mppi_core/core/mppi_config.hpp"
#include "mppi_core/costs/robust_grasp_state_cost.hpp"
#include "mppi_core/disturbance/grasp_disturbance_sampler.hpp"
#include "mppi_core/policy/grasp_action_library.hpp"
#include "mppi_core/rollout/disturbed_grasp_rollout.hpp"
#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/state/grasp_observation.hpp"
#include "mppi_core/state/grasp_state.hpp"

namespace mppi_core {

struct RobustGraspPolicyConfig {
  MPPIConfig rollout;
  GraspStartConfig start;

  DisturbedGraspRolloutConfig disturbed_rollout;
  GraspDisturbanceSamplerConfig disturbance_sampler;
  RobustGraspStateCostConfig cost;
  GraspActionLibraryConfig action_library;

  double risk_weight{1.0};
  double cvar_tail_fraction{0.25};

  bool require_both_contact_for_update{true};
  bool return_hold_when_not_ready{true};
};

struct RobustGraspPolicyStatus {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool ready{false};
  bool used_hold_fallback{false};

  std::size_t candidate_count{0};
  std::size_t disturbance_count{0};

  double best_score{0.0};
  double best_mean_cost{0.0};
  double best_cvar_cost{0.0};
  std::size_t best_candidate_index{0};

  Eigen::VectorXd selected_qddot;
};

class RobustGraspPolicy {
 public:
  RobustGraspPolicy() = default;

  void Initialize(RobustGraspPolicyConfig config);

  RobotCommand Update(const GraspObservation& observation);

  const RobustGraspPolicyStatus& status() const { return status_; }

 private:
  GraspState MakeInitialState(const GraspObservation& observation) const;

  double EvaluateCandidate(
      const GraspState& initial_state,
      const ActionSequence& candidate,
      const std::vector<GraspDisturbanceSequence,
                        Eigen::aligned_allocator<GraspDisturbanceSequence>>&
          disturbances,
      const RolloutContext& context,
      double* mean_cost,
      double* cvar_cost) const;

  RobotCommand MakeHoldCommand(const GraspObservation& observation) const;
  bool IsReady(const GraspState& state) const;

  RobustGraspPolicyConfig config_;
  RobustGraspPolicyStatus status_;
  std::unique_ptr<GraspDisturbanceSampler> disturbance_sampler_;
  std::unique_ptr<GraspActionLibrary> action_library_;
  std::unique_ptr<RobustGraspStateCost> cost_;
  bool initialized_{false};
};

}  // namespace mppi_core
