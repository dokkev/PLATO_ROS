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

struct RneaFeedforwardConfig {
  bool enabled{false};
  bool use_measured_state{true};
  bool subtract_contact_torque{false};
  double tau_ff_scale{1.0};
  double max_tau_ff_nm{0.05};
  double max_tau_ff_rate_nm_s{1.0};
  bool zero_tau_when_not_ready{true};
  bool zero_tau_on_contact_loss{true};
  bool gravity_only_when_qddot_zero{false};
};

struct RneaFeedforwardCommandResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::VectorXd raw;
  Eigen::VectorXd scaled;
  Eigen::VectorXd command;
  bool clamped{false};
  bool rate_limited{false};
  bool zeroed_not_ready{false};
  bool zeroed_contact_loss{false};
};

struct BaseGraspControllerConfig {
  bool enabled{false};

  double target_normal_force_n{1.0};
  double min_normal_force_per_sensor_n{0.1};
  double max_normal_force_per_sensor_n{5.0};

  double force_gain{0.2};
  double force_balance_gain{0.1};
  double contact_loss_gain{0.2};
  double high_force_release_gain{0.2};

  double max_qddot_base{2.0};
  double max_qddot_residual{1.0};
  double base_deviation_weight{0.01};

  bool use_force_balance{true};
  bool use_contact_loss_reflex{true};
  bool use_high_force_guard{true};
};

struct BaseGraspControllerStatus {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool enabled{false};
  bool active{false};
  bool has_thumb{false};
  bool has_index{false};
  bool contact_loss_reflex_active{false};
  bool high_force_guard_active{false};

  double thumb_force_n{0.0};
  double index_force_n{0.0};
  double average_force_n{0.0};
  double min_force_n{0.0};
  double max_force_n{0.0};
  double force_error_n{0.0};
  double force_balance_error_n{0.0};
  double high_force_excess_n{0.0};
  std::size_t active_sensor_count{0};

  Eigen::VectorXd qddot_base;
  Eigen::VectorXd qddot_residual;
  double qddot_base_norm{0.0};
  double qddot_residual_norm{0.0};
  double base_deviation_cost{0.0};
};

struct ContinuousQddotMppiConfig {
  MPPIConfig rollout;
  GraspDisturbanceSamplerConfig disturbance_sampler;
  RobustGraspStateCostConfig cost;

  double control_rate_cost_weight{1.0e-3};
  double smoothing_alpha{0.5};

  QddotRolloutLimits limits;
  BaseGraspControllerConfig base_grasp_controller;
  RneaFeedforwardConfig rnea_feedforward;
};

struct ContinuousQddotMppiStatus {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};

  std::size_t num_samples{0};
  std::size_t num_threads{1};
  std::size_t horizon_steps{0};
  double lambda{0.0};

  std::size_t best_sample_index{0};
  double best_sample_cost{0.0};
  double weighted_cost_estimate{0.0};
  double nominal_sample_cost{0.0};
  double cost_min{0.0};
  double cost_mean{0.0};
  double cost_max{0.0};
  double effective_sample_size{0.0};

  Eigen::VectorXd qddot_cmd;
  Eigen::VectorXd qddot_nominal_first;
  Eigen::VectorXd qddot_best_first;
  Eigen::VectorXd qddot_base;
  Eigen::VectorXd qddot_residual_cmd;
  BaseGraspControllerStatus base_grasp;

  bool use_rnea_feedforward{false};
  double tau_ff_scale{1.0};
  Eigen::VectorXd tau_ff_raw;
  Eigen::VectorXd tau_ff_scaled;
  Eigen::VectorXd tau_ff_cmd;
  double tau_ff_raw_norm{0.0};
  double tau_ff_cmd_norm{0.0};
  double tau_ff_max_abs{0.0};
  bool tau_ff_clamped{false};
  bool tau_ff_rate_limited{false};
  bool tau_ff_zeroed_not_ready{false};
  bool tau_ff_zeroed_contact_loss{false};

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
  double selected_base_deviation_cost{0.0};

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
  Eigen::Vector3d object_linear_disturbance_world_mps{
      Eigen::Vector3d::Zero()};
  Eigen::Vector3d object_angular_disturbance_world_radps{
      Eigen::Vector3d::Zero()};
  double object_linear_disturbance_speed_mps{0.0};
  double object_angular_disturbance_speed_radps{0.0};
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      sampled_object_linear_disturbances_world_mps;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      sampled_object_angular_disturbances_world_radps;

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

RneaFeedforwardCommandResult ComputeRneaFeedforwardCommand(
    const RneaFeedforwardConfig& config,
    const GraspObservation& observation,
    const GraspState& initial_state,
    const RolloutContext& context,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_cmd,
    const Eigen::Ref<const Eigen::VectorXd>& previous_tau_ff_cmd,
    bool has_previous_tau_ff_cmd,
    double dt);

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

  BaseGraspControllerStatus ComputeBaseGraspCommand(
      const GraspState& initial_state,
      const RolloutContext& context) const;
  ActionSequence SampleSequence(std::size_t sample_index);
  SampleEvaluation EvaluateSequence(
      const GraspState& initial_state,
      const ActionSequence& sequence,
      const Eigen::VectorXd& qddot_base,
      const GraspDisturbanceSequence& disturbance_sequence,
      const RolloutContext& context) const;
  RobotCommand MakeCommand(const GraspObservation& observation,
                           const GraspState& initial_state,
                           const RolloutContext& context,
                           const Eigen::VectorXd& qddot_cmd);
  void ShiftUpdatedSequence(const Eigen::MatrixXd& updated_values);

  ContinuousQddotMppiConfig config_;
  RobustGraspStateCost cost_{RobustGraspStateCostConfig{}};
  GraspDisturbanceSampler disturbance_sampler_{
      GraspDisturbanceSamplerConfig{}};
  ActionSequence nominal_sequence_;
  Eigen::VectorXd previous_qddot_cmd_;
  Eigen::VectorXd previous_qddot_residual_cmd_;
  Eigen::VectorXd previous_tau_ff_cmd_;
  bool has_previous_qddot_cmd_{false};
  bool has_previous_qddot_residual_cmd_{false};
  bool has_previous_tau_ff_cmd_{false};
  std::mt19937 rng_;
  ContinuousQddotMppiStatus status_;
  bool initialized_{false};
};

}  // namespace mppi_core
