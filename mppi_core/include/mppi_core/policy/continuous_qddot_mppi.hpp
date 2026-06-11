// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "mppi_core/core/action_sequence.hpp"
#include "mppi_core/core/mppi_config.hpp"
#include "mppi_core/costs/robust_grasp_state_cost.hpp"
#include "mppi_core/disturbance/grasp_disturbance_sampler.hpp"
#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/rollout/rollout_model.hpp"
#include "mppi_core/state/grasp_observation.hpp"
#include "mppi_core/state/grasp_state.hpp"

namespace mppi_core {

struct QddotRolloutLimits {
  Eigen::VectorXd qdot_lower_bound;
  Eigen::VectorXd qdot_upper_bound;
  bool clamp_q_to_model_position_limits{true};
};

struct ContinuousQddotMppiConfig {
  MPPIConfig rollout;
  GraspDisturbanceSamplerConfig disturbance_sampler;
  RobustGraspStateCostConfig cost;

  double control_rate_cost_weight{1.0e-3};
  double smoothing_alpha{0.5};

  QddotRolloutLimits limits;
};

struct ContinuousQddotMppiStatus {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};

  std::size_t num_samples{0};
  std::size_t horizon_steps{0};
  double lambda{0.0};

  std::size_t best_sample_index{0};
  double best_sample_cost{0.0};
  double weighted_cost_estimate{0.0};
  double cost_min{0.0};
  double cost_mean{0.0};
  double cost_max{0.0};
  double effective_sample_size{0.0};

  Eigen::VectorXd qddot_cmd;
  Eigen::VectorXd qddot_nominal_first;
  Eigen::VectorXd qddot_best_first;

  double selected_contact_loss_cost{0.0};
  double selected_support_cost{0.0};
  double selected_edge_cost{0.0};
  double selected_penetration_cost{0.0};
  double selected_preload_cost{0.0};
  double selected_force_low_cost{0.0};
  double selected_force_high_cost{0.0};
  double selected_balance_cost{0.0};
  double selected_control_cost{0.0};
  double selected_rate_cost{0.0};

  std::size_t geometry_query_count{0};
  std::size_t object_sample_count{0};
  double predicted_active_hemisphere_total{0.0};
  double measured_active_hemisphere_total{0.0};
  double object_contact_loss_count{0.0};
  double object_edge_margin_m{0.0};
  double object_min_signed_distance_m{0.0};
  Eigen::Vector2d predicted_centroid_sensor_m{
      Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector2d measured_centroid_sensor_m{
      Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())};
  double object_linear_disturbance_speed_mps{0.0};
  double object_angular_disturbance_speed_radps{0.0};

  std::vector<Eigen::Isometry3d,
              Eigen::aligned_allocator<Eigen::Isometry3d>>
      object_pose_rollout;
};

std::vector<double> ComputeSoftMppiWeights(
    const std::vector<double>& costs, double temperature_lambda);

RobotState StepRobotStateWithQddotLimits(
    const RobotState& robot,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
    RobotSystem* robot_system,
    double dt,
    const QddotRolloutLimits& limits);

class ContinuousQddotMppiController {
 public:
  ContinuousQddotMppiController() = default;

  void Initialize(ContinuousQddotMppiConfig config);

  RobotCommand Update(const GraspObservation& observation,
                      const GraspState& initial_state,
                      const RolloutContext& context);

  void ResetNominalSequence();

  const ContinuousQddotMppiStatus& status() const { return status_; }
  const ActionSequence& nominalSequence() const { return nominal_sequence_; }

 private:
  struct SampleStats;
  struct SampleEvaluation;

  ActionSequence SampleSequence(std::size_t sample_index);
  SampleEvaluation EvaluateSequence(
      const GraspState& initial_state,
      const ActionSequence& sequence,
      const GraspDisturbanceSequence& disturbance_sequence,
      const RolloutContext& context) const;
  RobotCommand MakeCommand(const GraspObservation& observation,
                           const GraspState& initial_state,
                           const RolloutContext& context,
                           const Eigen::VectorXd& qddot_cmd) const;
  void ShiftUpdatedSequence(const Eigen::MatrixXd& updated_values);

  ContinuousQddotMppiConfig config_;
  RobustGraspStateCost cost_{RobustGraspStateCostConfig{}};
  GraspDisturbanceSampler disturbance_sampler_{
      GraspDisturbanceSamplerConfig{}};
  ActionSequence nominal_sequence_;
  Eigen::VectorXd previous_qddot_cmd_;
  bool has_previous_qddot_cmd_{false};
  std::mt19937 rng_;
  ContinuousQddotMppiStatus status_;
  bool initialized_{false};
};

}  // namespace mppi_core
