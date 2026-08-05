// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstddef>
#include <limits>
#include <vector>

#include "mppi_core/object/object_belief_initializer.hpp"
#include "mppi_core/object/object_contact_belief.hpp"
#include "mppi_core/object/object_prior.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"
#include "mppi_core/tactile/tactile_state.hpp"

namespace mppi_core {

struct ObjectPriorEstimatorConfig {
  ObjectBeliefInitializationConfig belief;

  // Reject tactile corrections when the two strongest tactile sensors are
  // closer than the minimum plausible object extent. This catches the common
  // false positive where fingertips touch each other instead of grasping the
  // object.
  bool reject_close_sensor_gap_as_fingertip_touch{false};
  double sensor_gap_min_object_extent_scale{0.8};
  double sensor_gap_min_margin_m{0.002};
};

struct ObjectPriorEstimatorStatus {
  bool configured{false};
  bool updated{false};
  bool belief_valid{false};
  bool close_sensor_gap_rejected{false};

  std::size_t update_count{0};
  std::size_t contact_count{0};
  std::size_t particle_count{0};
  std::size_t best_particle_index{0};

  double best_cost{std::numeric_limits<double>::infinity()};
  double best_surface_distance_m{std::numeric_limits<double>::infinity()};
  double best_normal_alignment_error{
      std::numeric_limits<double>::infinity()};
  double best_quasi_static_rbd_cost{
      std::numeric_limits<double>::quiet_NaN()};
  double best_static_force_residual_n{
      std::numeric_limits<double>::quiet_NaN()};
  double best_static_torque_residual_nm{
      std::numeric_limits<double>::quiet_NaN()};
  double sensor_gap_m{std::numeric_limits<double>::infinity()};
  double min_sensor_gap_m{0.0};
  double object_min_extent_m{std::numeric_limits<double>::infinity()};
};

// ROS-free object pose prior estimator backed by tactile-contact particle
// correction. The estimator owns the previous belief so controller and ROS
// wrappers can share the same stateful update behavior.
class ObjectPriorEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ObjectPriorEstimator() = default;
  ObjectPriorEstimator(
      const ObjectPrior& prior,
      const ObjectBeliefInitializationConfig& config = {});
  ObjectPriorEstimator(const ObjectPrior& prior,
                       const ObjectPriorEstimatorConfig& config);

  bool Configure(const ObjectPrior& prior,
                 const ObjectBeliefInitializationConfig& config = {});
  bool Configure(const ObjectPrior& prior,
                 const ObjectPriorEstimatorConfig& config);
  void Reset();

  ObjectBeliefInitializationResult Update(
      const Eigen::Ref<const Eigen::VectorXd>& q_meas,
      const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
          tactile_sensors,
      const std::vector<TactileSensorContext>& tactile_contexts);

  bool configured() const { return configured_; }
  bool has_belief() const;

  const ObjectPrior& prior() const { return prior_; }
  const ObjectBeliefInitializationConfig& config() const {
    return config_.belief;
  }
  const ObjectPriorEstimatorConfig& estimator_config() const {
    return config_;
  }
  const VirtualObjectBelief& belief() const { return belief_; }
  const ObjectPriorEstimatorStatus& status() const { return status_; }

  bool representativePose(Eigen::Isometry3d* pose_world) const;
  static bool RepresentativePose(const VirtualObjectBelief& belief,
                                 Eigen::Isometry3d* pose_world);

 private:
  ObjectPrior prior_;
  ObjectPriorEstimatorConfig config_;
  VirtualObjectBelief belief_;
  ObjectPriorEstimatorStatus status_;
  bool configured_{false};
};

}  // namespace mppi_core
