// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "mppi_core/core/mppi_config.hpp"
#include "mppi_core/costs/robust_grasp_state_cost.hpp"
#include "mppi_core/disturbance/grasp_disturbance_sampler.hpp"
#include "mppi_core/object/object_belief_initializer.hpp"
#include "mppi_core/policy/continuous_qddot_mppi.hpp"
#include "mppi_core/policy/grasp_action_library.hpp"
#include "mppi_core/rollout/disturbed_grasp_rollout.hpp"
#include "mppi_core/rollout/object_prior_grasp_rollout.hpp"
#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/state/grasp_observation.hpp"
#include "mppi_core/state/grasp_state.hpp"

namespace mppi_core {

enum class RobustGraspControlMode {
  kContinuousQddotMppi,
  kDiscreteActionSelector,
};

struct RobustGraspPolicyConfig {
  MPPIConfig rollout;
  GraspStartConfig start;

  TactileOnlyContactTransitionConfig tactile_only_transition;
  GraspDisturbanceSamplerConfig disturbance_sampler;
  ObjectBeliefInitializationConfig object_belief_initialization;
  RobustGraspStateCostConfig cost;
  GraspActionLibraryConfig action_library;

  RobustGraspControlMode control_mode{
      RobustGraspControlMode::kContinuousQddotMppi};
  double continuous_control_rate_cost_weight{1.0e-3};
  double continuous_smoothing_alpha{0.5};
  bool skip_mppi_when_not_enough_contacts{false};
  BaseGraspControllerConfig base_grasp_controller;
  RneaFeedforwardConfig rnea_feedforward;

  double risk_weight{1.0};
  double cvar_tail_fraction{0.25};
  double safe_hold_score_threshold{-1.0};
  double min_required_score_improvement{1.0};
  double action_rate_weight{0.01};

  bool require_both_contact_for_update{true};
  bool return_hold_when_not_ready{true};
};

struct RobustGraspPolicyStatus {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool ready{false};
  bool used_hold_fallback{false};
  RobustGraspControlMode control_mode{
      RobustGraspControlMode::kContinuousQddotMppi};
  bool used_continuous_qddot_mppi{false};
  bool mppi_skipped_for_contact_recovery{false};

  std::size_t candidate_count{0};
  std::size_t disturbance_count{0};
  std::size_t evaluation_thread_count{1};
  std::size_t horizon_steps{0};
  double lambda{0.0};

  double best_score{0.0};
  double best_mean_cost{0.0};
  double best_cvar_cost{0.0};
  std::size_t best_candidate_index{0};
  std::string best_action_name;
  bool selected_hold_by_margin{false};

  double raw_best_score{0.0};
  double raw_best_mean_cost{0.0};
  double raw_best_cvar_cost{0.0};
  std::size_t raw_best_candidate_index{0};
  std::string raw_best_action_name;
  double hold_score_improvement{0.0};

  double hold_score{0.0};
  double hold_mean_cost{0.0};
  double hold_cvar_cost{0.0};
  std::string hold_action_name{"hold"};

  double second_best_score{0.0};
  std::size_t second_best_candidate_index{0};
  std::string second_best_action_name;

  double selected_object_support_cost{0.0};
  double selected_contact_loss_cost{0.0};
  double selected_support_cost{0.0};
  double selected_edge_cost{0.0};
  double selected_penetration_cost{0.0};
  double selected_preload_cost{0.0};
  double selected_force_low_cost{0.0};
  double selected_force_high_cost{0.0};
  double selected_balance_cost{0.0};
  double selected_action_cost{0.0};
  double selected_control_cost{0.0};
  double selected_rate_cost{0.0};
  double selected_base_deviation_cost{0.0};
  double best_sample_cost{0.0};
  double weighted_cost_estimate{0.0};
  double nominal_cost{0.0};
  double cost_min{0.0};
  double cost_mean{0.0};
  double cost_max{0.0};
  double effective_sample_size{0.0};
  std::size_t selected_object_sample_count{0};
  std::size_t selected_object_geometry_query_count{0};
  double selected_predicted_active_hemisphere_total{0.0};
  double selected_measured_active_hemisphere_total{0.0};
  double selected_object_contact_loss_count{0.0};
  double selected_object_edge_margin_m{0.0};
  double selected_object_min_gap_m{0.0};
  double measured_thumb_force_n{0.0};
  double measured_index_force_n{0.0};
  double initial_total_cost{0.0};
  double initial_object_support_cost{0.0};
  double initial_contact_loss_cost{0.0};
  double initial_support_cost{0.0};
  double initial_edge_cost{0.0};
  double initial_penetration_cost{0.0};
  double initial_preload_cost{0.0};
  double initial_force_low_cost{0.0};
  double initial_force_high_cost{0.0};
  double initial_balance_cost{0.0};
  double initial_object_min_gap_m{0.0};
  double initial_object_edge_margin_m{0.0};
  Eigen::Vector2d selected_predicted_centroid_sensor_m{
      Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector2d selected_measured_centroid_sensor_m{
      Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d selected_object_linear_disturbance_world_mps{
      Eigen::Vector3d::Zero()};
  Eigen::Vector3d selected_object_angular_disturbance_world_radps{
      Eigen::Vector3d::Zero()};
  double selected_object_linear_disturbance_speed_mps{0.0};
  double selected_object_angular_disturbance_speed_radps{0.0};
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      sampled_object_linear_disturbances_world_mps;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      sampled_object_angular_disturbances_world_radps;
  double solve_time_ms{0.0};
  double belief_update_ms{0.0};
  double sample_generation_ms{0.0};
  double workspace_setup_ms{0.0};
  double rollout_eval_ms{0.0};
  double mppi_weighting_ms{0.0};
  double command_build_ms{0.0};
  double logging_ms{0.0};

  std::vector<Eigen::Isometry3d,
              Eigen::aligned_allocator<Eigen::Isometry3d>>
      selected_object_pose_rollout;

  Eigen::VectorXd qddot_cmd;
  Eigen::VectorXd qddot_nominal_first;
  Eigen::VectorXd qddot_best_first;
  Eigen::VectorXd selected_qddot;
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
};

class RobustGraspPolicy {
 public:
  RobustGraspPolicy() = default;

  void Initialize(RobustGraspPolicyConfig config);

  RobotCommand Update(const GraspObservation& observation);

  const RobustGraspPolicyStatus& status() const { return status_; }

 private:
  struct CandidateEvaluationStats;

  GraspState MakeInitialState(const GraspObservation& observation) const;

  double EvaluateCandidate(
      const GraspState& initial_state,
      const ActionSequence& candidate,
      const std::vector<GraspDisturbanceSequence,
                        Eigen::aligned_allocator<GraspDisturbanceSequence>>&
          disturbances,
      const RolloutContext& context,
      double* mean_cost,
      double* cvar_cost,
      CandidateEvaluationStats* stats) const;

  RobotCommand UpdateContinuousQddotMppi(
      const GraspObservation& observation,
      const GraspState& initial_state,
      const RolloutContext& context);
  RobotCommand UpdateDiscreteActionSelector(
      const GraspObservation& observation,
      const GraspState& initial_state,
      const RolloutContext& context);
  void CopyContinuousStatus(
      const ContinuousQddotMppiStatus& continuous_status);

  RobotCommand MakeHoldCommand(const GraspObservation& observation) const;
  bool IsReady(const GraspState& state) const;

  RobustGraspPolicyConfig config_;
  RobustGraspPolicyStatus status_;
  std::unique_ptr<GraspDisturbanceSampler> disturbance_sampler_;
  std::unique_ptr<GraspActionLibrary> action_library_;
  std::unique_ptr<RobustGraspStateCost> cost_;
  std::unique_ptr<ContinuousQddotMppiController> continuous_mppi_;
  Eigen::VectorXd last_selected_qddot_;
  bool initialized_{false};
};

}  // namespace mppi_core
