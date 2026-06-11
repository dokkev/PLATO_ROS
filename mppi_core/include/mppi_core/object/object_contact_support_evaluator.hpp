// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <limits>

#include "mppi_core/rollout/rollout_model.hpp"
#include "mppi_core/state/grasp_state.hpp"

namespace mppi_core {

struct ContactSupportSummary {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int predicted_active_count{0};
  int measured_active_count{0};

  double predicted_support_area{0.0};
  double predicted_edge_margin_m{0.0};

  Eigen::Vector2d predicted_centroid_sensor_m{
      Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())};

  Eigen::Vector2d measured_centroid_sensor_m{
      Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())};

  // Deprecated compatibility output; geometric object support leaves this zero.
  double predicted_total_force_n{0.0};
};

struct ObjectContactSupportEvaluatorConfig {
  bool enabled{true};

  std::size_t max_object_samples{16};
  std::size_t min_active_tactile_sensors{2};
  std::size_t min_active_hemisphere_total{2};

  double contact_birth_margin_m{0.001};
  double contact_loss_margin_m{0.003};
  // Deprecated compatibility field. Geometry support does not synthesize a
  // contact force.
  double contact_stiffness_n_per_m{0.0};
  double hemisphere_radius_m{0.0};
  double support_distance_scale_m{0.001};

  // Good-contact band around the object surface. Slightly inside and slightly
  // outside contacts are both treated as useful support.
  double good_contact_gap_min_m{-0.001};
  double good_contact_gap_max_m{0.001};
  double deep_contact_scale_m{0.002};
  double deep_contact_weight{5.0};

  // Deprecated compatibility field. Hard penetration penalties are not used by
  // the current evaluator because measured tactile contact is treated as a real
  // contact anchor.
  double max_allowed_penetration_m{0.0};

  // Deprecated compatibility fields. Object support evaluation is geometric and
  // does not estimate true contact force; force/preload costs use measured
  // tactile force only.
  double target_predicted_normal_force_n{0.0};
  double max_predicted_normal_force_n{0.0};
  double min_predicted_contact_force_n{0.0};
  double max_predicted_force_per_sensor_n{0.0};

  double contact_loss_weight{200.0};
  double support_weight{20.0};
  double edge_weight{10.0};
  // Deprecated compatibility field. Use deep_contact_weight instead.
  double penetration_weight{0.0};
  double predicted_force_low_weight{0.0};
  double predicted_force_high_weight{0.0};
  double target_edge_margin_m{0.001};

  bool use_particle_weights{true};
};

struct ObjectContactSupportEvaluation {
  bool valid{false};

  std::size_t object_sample_count{0};
  std::size_t geometry_query_count{0};

  double measured_active_hemisphere_total{0.0};
  double measured_active_tactile_sensors{0.0};
  double predicted_active_hemisphere_total{0.0};
  double predicted_active_tactile_sensors{0.0};
  double lost_measured_contact_count{0.0};

  double min_signed_distance_m{0.0};

  // Deprecated compatibility output. Object support does not estimate true
  // contact force; this remains zero in the geometric evaluator.
  double predicted_normal_force_total_n{0.0};
  double min_edge_margin_m{0.0};

  double contact_loss_cost{0.0};
  double support_cost{0.0};
  double edge_cost{0.0};
  double predicted_force_low_cost{0.0};
  double predicted_force_high_cost{0.0};
  double penetration_cost{0.0};

  ContactSupportSummary support_summary;

  double totalCost() const {
    return contact_loss_cost + support_cost + edge_cost +
           penetration_cost;
  }
};

ObjectContactSupportEvaluation EvaluateObjectContactSupport(
    const GraspState& state,
    const RolloutContext& context,
    const ObjectContactSupportEvaluatorConfig& config = {});

}  // namespace mppi_core
