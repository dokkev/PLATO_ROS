// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "mppi_core/object/object_contact_belief.hpp"
#include "mppi_core/object/object_prior.hpp"
#include "mppi_core/state/grasp_observation.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"
#include "mppi_core/tactile/tactile_state.hpp"

namespace mppi_core {

struct ObjectBeliefInitializationConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t particle_count{32};
  std::uint32_t random_seed{17};
  std::size_t min_contact_count{1};

  Eigen::Vector3d fallback_position_sample_std_m{
      Eigen::Vector3d{0.01, 0.01, 0.01}};
  Eigen::Vector3d fallback_rpy_sample_std_rad{
      Eigen::Vector3d{0.05, 0.05, 0.05}};

  double surface_distance_sigma_m{0.005};
  double normal_alignment_sigma{0.25};
  double prior_position_sigma_m{0.02};
  double prior_rotation_sigma_rad{0.35};
  double contact_force_threshold_n{0.0};
  double contact_force_weight_scale_n{1.0};

  double w_surface{1.0};
  double w_normal{0.5};
  double w_prior{0.1};
};

struct ObjectContactObservation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t sensor_index{0};
  std::size_t hemisphere_index{0};

  Eigen::Vector3d point_world_m{Eigen::Vector3d::Zero()};

  // Direction of positive tactile compression/contact in world coordinates.
  Eigen::Vector3d normal_world{Eigen::Vector3d::UnitZ()};

  double normal_force_n{0.0};
  double confidence{1.0};
};

struct ObjectParticleScore {
  double surface_cost{std::numeric_limits<double>::infinity()};
  double normal_cost{std::numeric_limits<double>::infinity()};
  double prior_cost{std::numeric_limits<double>::infinity()};
  double total_cost{std::numeric_limits<double>::infinity()};
  double mean_surface_distance_m{std::numeric_limits<double>::infinity()};
  double mean_normal_alignment_error{std::numeric_limits<double>::infinity()};
};

struct ObjectBeliefInitializationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};
  VirtualObjectBelief belief;

  std::vector<ObjectContactObservation,
              Eigen::aligned_allocator<ObjectContactObservation>>
      contacts;
  std::vector<ObjectParticleScore> particle_scores;

  std::size_t best_particle_index{0};
  double best_cost{std::numeric_limits<double>::infinity()};
  double best_surface_distance_m{std::numeric_limits<double>::infinity()};
  double best_normal_alignment_error{std::numeric_limits<double>::infinity()};
};

std::vector<ObjectContactObservation,
            Eigen::aligned_allocator<ObjectContactObservation>>
ExtractObjectContactObservations(
    const Eigen::Ref<const Eigen::VectorXd>& q_meas,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const std::vector<TactileSensorContext>& tactile_contexts);

ObjectBeliefInitializationResult InitializeObjectBeliefFromContacts(
    const ObjectPrior& prior,
    const Eigen::Ref<const Eigen::VectorXd>& q_meas,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const std::vector<TactileSensorContext>& tactile_contexts,
    const ObjectBeliefInitializationConfig& config = {});

ObjectBeliefInitializationResult InitializeObjectBeliefFromObservation(
    const GraspObservation& observation,
    const ObjectBeliefInitializationConfig& config = {});

VirtualObjectBelief ResolveObjectBeliefForObservation(
    const GraspObservation& observation,
    const Eigen::Ref<const Eigen::VectorXd>& q_rollout_root,
    const ObjectBeliefInitializationConfig& config = {});

}  // namespace mppi_core
