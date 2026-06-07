// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "mppi_core/contact/hemisphere_motion.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"

namespace mppi_core {

struct TactileTransitionConfig {
  bool enable_birth{true};
  bool enable_loss{true};

  double birth_score_threshold{0.5};
  double loss_score_threshold{0.5};

  double birth_neighbor_weight{0.25};
  double birth_tangent_approach_weight{1.0};
  double birth_normal_approach_weight{0.5};
  double birth_shear_penalty_weight{0.5};

  double loss_unloading_weight{1.0};
  double loss_shear_weight{0.5};
  double loss_low_force_weight{0.5};

  double born_normal_force_n{0.05};
  double born_confidence{0.5};

  double inactive_confidence{0.0};
  double contact_confidence_decay{1.0};

  double aggregate_shear_decay{1.0};
  double aggregate_rotation_decay{1.0};

  std::size_t enough_contact_hemisphere_count{2};
};

struct GraspRolloutConfig {
  // Reduced tactile support geometry limits.
  double sensor_half_width_x_m{0.003};
  double sensor_half_width_y_m{0.009};
  double edge_risk_limit{1.0};

  // Contact classification. These are rollout model parameters, not costs.
  std::size_t min_active_hemisphere_count{4};
  double min_contact_confidence{1.0e-3};

  // Geometry update gains.
  double centroid_motion_gain{1.0};
  double tangential_shear_gain{1.0};
  double rotational_shear_gain{1.0};

  // Normal preload proxy. This is not exact force estimation.
  double normal_force_gain_n_per_m{100.0};
  double max_normal_force_n{10.0};

  // Confidence update from motion tendency.
  double closing_confidence_gain_per_m{20.0};
  double separating_confidence_loss_per_m{200.0};
  double tangential_confidence_loss_per_m{5.0};
  double edge_confidence_loss_gain{0.25};

  // State bounds for one-step rollout robustness.
  double max_centroid_step_m{1.0e-2};
  double max_shear_m{2.0e-2};
  double max_rotational_shear_rad{1.0};

  // Normalization references for predicted tactile slip scores.
  double shear_ref_m{1.0e-3};
  double rotational_shear_ref_rad{2.0e-2};
};

inline double Clamp01(double x) {
  if (!std::isfinite(x)) {
    return 0.0;
  }
  return std::clamp(x, 0.0, 1.0);
}

inline bool HasNeighborInActiveSet(const HemisphereGeometry& geometry,
                                   const std::vector<bool>& active_by_index) {
  for (const std::size_t neighbor : geometry.neighbors) {
    if (neighbor < active_by_index.size() && active_by_index[neighbor]) {
      return true;
    }
  }
  return false;
}

inline void RefreshTactileAggregate(TactileState* tactile,
                                    const TactileTransitionConfig& config) {
  if (tactile == nullptr) {
    return;
  }

  tactile->valid = true;
  const std::size_t active_count = tactile->activeHemisphereCount();
  if (active_count == 0) {
    tactile->contact_state = TactileState::kNoContact;
  } else if (active_count < config.enough_contact_hemisphere_count) {
    tactile->contact_state = TactileState::kFewContacts;
  } else {
    tactile->contact_state = TactileState::kEnoughContacts;
  }

  tactile->total_force_n =
      Eigen::Vector3d{0.0, 0.0, tactile->activeHemisphereNormalForceN()};

  if (active_count == 0) {
    tactile->confidence = Clamp01(config.inactive_confidence);
  } else {
    tactile->confidence = Clamp01(tactile->confidence);
  }

  if (!tactile->shear_displacement_m.allFinite()) {
    tactile->shear_displacement_m = Eigen::Vector2d::Zero();
  }
  tactile->shear_displacement_m *=
      std::isfinite(config.aggregate_shear_decay)
          ? config.aggregate_shear_decay
          : 0.0;

  if (!std::isfinite(tactile->rotational_shear_rad)) {
    tactile->rotational_shear_rad = 0.0;
  }
  tactile->rotational_shear_rad *=
      std::isfinite(config.aggregate_rotation_decay)
          ? config.aggregate_rotation_decay
          : 0.0;

  tactile->slip_score = tactile->shear_displacement_m.norm() +
                        std::abs(tactile->rotational_shear_rad);
  tactile->slip_velocity_score = tactile->shear_velocity_mps.allFinite()
                                     ? tactile->shear_velocity_mps.norm()
                                     : 0.0;
  tactile->incipient_slip_score = tactile->slip_score;
}

inline Eigen::Vector2d ActiveHemisphereCentroidOrZero(
    const TactileState& tactile) {
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  std::size_t count = 0;
  for (const auto& hemi : tactile.hemispheres) {
    if (hemi.contact && hemi.cop_sensor_m.allFinite()) {
      sum += hemi.cop_sensor_m;
      ++count;
    }
  }
  if (count == 0) {
    return Eigen::Vector2d::Zero();
  }
  return sum / static_cast<double>(count);
}

inline TactileState StepTactileState(
    const TactileState& tactile, const RobotState& /*robot*/,
    const RobotState& /*next_robot*/,
    const std::vector<HemisphereMotion>& motions,
    const TactileSensorContext& sensor_context,
    const TactileTransitionConfig& config, double dt) {
  TactileState out = tactile;
  if (!tactile.valid ||
      tactile.hemispheres.size() != sensor_context.hemispheres.size() ||
      tactile.hemispheres.size() != motions.size() || !std::isfinite(dt) ||
      dt <= 0.0) {
    out.valid = false;
    return out;
  }

  for (std::size_t i = 0; i < tactile.hemispheres.size(); ++i) {
    const auto& hemi = tactile.hemispheres[i];
    const auto& motion = motions[i];
    const auto& geometry = sensor_context.hemispheres[i];
    if (motion.hemisphere_index != hemi.hemisphere_index ||
        geometry.hemisphere_index != hemi.hemisphere_index ||
        !IsFiniteHemisphereMotion(motion)) {
      out.valid = false;
      return out;
    }
  }

  std::size_t max_index = 0;
  for (const auto& hemi : tactile.hemispheres) {
    max_index = std::max(max_index, hemi.hemisphere_index);
  }
  std::vector<bool> active_by_index(max_index + 1, false);
  for (const auto& hemi : tactile.hemispheres) {
    if (hemi.hemisphere_index < active_by_index.size()) {
      active_by_index[hemi.hemisphere_index] = hemi.contact;
    }
  }

  Eigen::Vector2d tangent_delta_sum_m = Eigen::Vector2d::Zero();
  Eigen::Vector2d tangent_velocity_sum_mps = Eigen::Vector2d::Zero();
  std::size_t active_motion_count = 0;
  for (std::size_t i = 0; i < tactile.hemispheres.size(); ++i) {
    if (!tactile.hemispheres[i].contact) {
      continue;
    }
    const Eigen::Vector2d tangent_delta_m =
        motions[i].velocity_sensor_xy_mps * dt;
    tangent_delta_sum_m += tangent_delta_m;
    tangent_velocity_sum_mps += motions[i].velocity_sensor_xy_mps;
    ++active_motion_count;
  }

  Eigen::Vector2d mean_tangent_delta_m = Eigen::Vector2d::Zero();
  Eigen::Vector2d mean_tangent_velocity_mps = Eigen::Vector2d::Zero();
  if (active_motion_count > 0) {
    const double count = static_cast<double>(active_motion_count);
    mean_tangent_delta_m = tangent_delta_sum_m / count;
    mean_tangent_velocity_mps = tangent_velocity_sum_mps / count;
  }

  const Eigen::Vector2d active_centroid =
      ActiveHemisphereCentroidOrZero(tactile);
  double rotation_delta_rad = 0.0;
  if (active_motion_count > 0) {
    double numerator = 0.0;
    double denominator = 0.0;
    for (std::size_t i = 0; i < tactile.hemispheres.size(); ++i) {
      if (!tactile.hemispheres[i].contact) {
        continue;
      }
      const Eigen::Vector2d radius_m =
          motions[i].position_sensor_m.head<2>() - active_centroid;
      const Eigen::Vector2d tangent_delta_m =
          motions[i].velocity_sensor_xy_mps * dt - mean_tangent_delta_m;
      numerator += radius_m.x() * tangent_delta_m.y() -
                   radius_m.y() * tangent_delta_m.x();
      denominator += radius_m.squaredNorm();
    }
    if (denominator > 1.0e-12 && std::isfinite(numerator)) {
      rotation_delta_rad = numerator / denominator;
    }
  }

  const Eigen::Vector2d current_shear_m =
      tactile.shear_displacement_m.allFinite() ? tactile.shear_displacement_m
                                               : Eigen::Vector2d::Zero();
  out.shear_displacement_m = current_shear_m + mean_tangent_delta_m;
  out.shear_velocity_mps = mean_tangent_velocity_mps;
  const double current_rotation_rad =
      std::isfinite(tactile.rotational_shear_rad)
          ? tactile.rotational_shear_rad
          : 0.0;
  out.rotational_shear_rad = current_rotation_rad + rotation_delta_rad;
  out.rotational_shear_velocity_radps = rotation_delta_rad / dt;

  const double shear_norm = out.shear_displacement_m.allFinite()
                                ? out.shear_displacement_m.norm()
                                : 0.0;
  constexpr double kForceEpsilon = 1.0e-6;

  for (std::size_t i = 0; i < out.hemispheres.size(); ++i) {
    auto& hemi = out.hemispheres[i];
    const auto& motion = motions[i];
    const auto& geometry = sensor_context.hemispheres[i];
    hemi.hemisphere_index = tactile.hemispheres[i].hemisphere_index;

    if (hemi.contact) {
      if (hemi.cop_sensor_m.allFinite()) {
        hemi.cop_sensor_m += motion.velocity_sensor_xy_mps * dt;
      }
      const double unloading = std::max(0.0, -motion.normal_velocity_mps);
      const double normal_force =
          std::max(0.0, std::isfinite(hemi.normal_force_n)
                            ? hemi.normal_force_n
                            : 0.0);
      const double low_force =
          std::clamp(1.0 - normal_force, 0.0, 1.0) + kForceEpsilon;
      const double loss_score = config.loss_unloading_weight * unloading +
                                config.loss_shear_weight * shear_norm +
                                config.loss_low_force_weight * low_force;

      if (config.enable_loss && loss_score > config.loss_score_threshold) {
        hemi.contact = false;
        hemi.normal_force_n = 0.0;
        hemi.confidence = config.inactive_confidence;
      } else {
        hemi.normal_force_n = normal_force;
        hemi.confidence =
            Clamp01(hemi.confidence * config.contact_confidence_decay);
      }
      continue;
    }

    const bool adjacent =
        HasNeighborInActiveSet(geometry, active_by_index);
    if (!config.enable_birth || !adjacent) {
      hemi.contact = false;
      hemi.normal_force_n = 0.0;
      hemi.confidence = config.inactive_confidence;
      continue;
    }

    Eigen::Vector2d toward_candidate = Eigen::Vector2d::Zero();
    if (hemi.cop_sensor_m.allFinite()) {
      toward_candidate = hemi.cop_sensor_m - active_centroid;
    }
    if (toward_candidate.norm() > 1.0e-12) {
      toward_candidate.normalize();
    } else {
      toward_candidate.setZero();
    }

    const double neighbor_term = adjacent ? 1.0 : 0.0;
    const double tangent_approach =
        std::max(0.0, motion.velocity_sensor_xy_mps.dot(toward_candidate));
    const double normal_approach =
        std::max(0.0, motion.normal_velocity_mps);
    const double birth_score =
        config.birth_neighbor_weight * neighbor_term +
        config.birth_tangent_approach_weight * tangent_approach +
        config.birth_normal_approach_weight * normal_approach -
        config.birth_shear_penalty_weight * shear_norm;

    if (birth_score >= config.birth_score_threshold) {
      hemi.contact = true;
      hemi.normal_force_n = config.born_normal_force_n;
      hemi.confidence = config.born_confidence;
    } else {
      hemi.contact = false;
      hemi.normal_force_n = 0.0;
      hemi.confidence = config.inactive_confidence;
    }
  }

  RefreshTactileAggregate(&out, config);
  return out;
}

inline Eigen::Vector2d ClampPatchVectorNorm(const Eigen::Vector2d& value,
                                            double max_norm) {
  if (!value.allFinite()) {
    return Eigen::Vector2d::Zero();
  }

  const double safe_max_norm =
      std::max(0.0, std::isfinite(max_norm) ? max_norm : 0.0);
  const double norm = value.norm();
  if (norm <= 1.0e-12) {
    return value;
  }
  if (safe_max_norm <= 0.0) {
    return Eigen::Vector2d::Zero();
  }
  if (norm <= safe_max_norm) {
    return value;
  }
  return value * (safe_max_norm / norm);
}

inline double ComputeTactileContactEdgeRisk(const Eigen::Vector2d& centroid_m,
                                            const GraspRolloutConfig& config) {
  if (!centroid_m.allFinite()) {
    return 0.0;
  }

  const double x_norm =
      std::abs(centroid_m.x()) / std::max(1.0e-9, config.sensor_half_width_x_m);
  const double y_norm =
      std::abs(centroid_m.y()) / std::max(1.0e-9, config.sensor_half_width_y_m);

  return std::clamp(std::max(x_norm, y_norm), 0.0, 2.0);
}

inline double ComputeActiveHemisphereScore(std::size_t active_hemisphere_count,
                                           const GraspRolloutConfig& config) {
  if (config.min_active_hemisphere_count == 0) {
    return 0.0;
  }
  return Clamp01(static_cast<double>(active_hemisphere_count) /
                 static_cast<double>(config.min_active_hemisphere_count));
}

inline bool ComputeActiveHemisphereCentroidM(const TactileState& tactile,
                                             Eigen::Vector2d* centroid_m) {
  if (centroid_m == nullptr) {
    return false;
  }

  Eigen::Vector2d weighted_sum = Eigen::Vector2d::Zero();
  double weight_sum = 0.0;
  for (const auto& hemisphere : tactile.hemispheres) {
    if (!hemisphere.contact || !hemisphere.cop_sensor_m.allFinite()) {
      continue;
    }
    const double weight = std::isfinite(hemisphere.normal_force_n) &&
                                  hemisphere.normal_force_n > 0.0
                              ? hemisphere.normal_force_n
                              : 1.0;
    weighted_sum += weight * hemisphere.cop_sensor_m;
    weight_sum += weight;
  }

  if (weight_sum <= 0.0) {
    return false;
  }
  *centroid_m = weighted_sum / weight_sum;
  return centroid_m->allFinite();
}

inline int PredictContactState(const TactileState& tactile,
                               const GraspRolloutConfig& config) {
  if (!tactile.valid || !tactile.total_force_n.allFinite() ||
      tactile.total_force_n.z() <= 0.0 ||
      Clamp01(tactile.confidence) <= config.min_contact_confidence ||
      tactile.activeHemisphereCount() == 0) {
    return TactileState::kNoContact;
  }

  if (ComputeActiveHemisphereScore(tactile.activeHemisphereCount(), config) >=
      1.0) {
    return TactileState::kEnoughContacts;
  }
  return TactileState::kFewContacts;
}

inline double ComputeTactileRolloutSlipScore(
    const TactileState& tactile, const GraspRolloutConfig& config = {}) {
  const double shear_ref_m = std::max(
      1.0e-9, std::isfinite(config.shear_ref_m) ? config.shear_ref_m : 1.0e-3);
  const double rotational_ref_rad =
      std::max(1.0e-9, std::isfinite(config.rotational_shear_ref_rad)
                           ? config.rotational_shear_ref_rad
                           : 2.0e-2);

  const double shear_score =
      tactile.shear_displacement_m.allFinite()
          ? tactile.shear_displacement_m.norm() / shear_ref_m
          : 0.0;
  const double rotational_score =
      std::isfinite(tactile.rotational_shear_rad)
          ? std::abs(tactile.rotational_shear_rad) / rotational_ref_rad
          : 0.0;

  return shear_score + rotational_score;
}

inline void RefreshPredictedTactileFields(TactileState* tactile,
                                          const GraspRolloutConfig& config) {
  if (tactile == nullptr) {
    return;
  }

  tactile->valid = true;

  if (!tactile->total_force_n.allFinite()) {
    tactile->total_force_n = Eigen::Vector3d::Zero();
  }
  tactile->total_force_n.z() =
      std::clamp(tactile->total_force_n.z(), 0.0,
                 std::max(0.0, config.max_normal_force_n));

  if (!tactile->shear_displacement_m.allFinite()) {
    tactile->shear_displacement_m = Eigen::Vector2d::Zero();
  } else {
    tactile->shear_displacement_m =
        ClampPatchVectorNorm(tactile->shear_displacement_m, config.max_shear_m);
  }

  if (!std::isfinite(tactile->rotational_shear_rad)) {
    tactile->rotational_shear_rad = 0.0;
  } else {
    const double max_rot =
        std::max(0.0, std::isfinite(config.max_rotational_shear_rad)
                          ? config.max_rotational_shear_rad
                          : 0.0);
    tactile->rotational_shear_rad =
        std::clamp(tactile->rotational_shear_rad, -max_rot, max_rot);
  }

  tactile->confidence = Clamp01(tactile->confidence);

  if (tactile->total_force_n.z() <= 0.0 ||
      tactile->confidence <= config.min_contact_confidence) {
    for (auto& hemisphere : tactile->hemispheres) {
      hemisphere.contact = false;
      hemisphere.normal_force_n = 0.0;
    }
  }

  tactile->contact_state = PredictContactState(*tactile, config);
  tactile->slip_score = ComputeTactileRolloutSlipScore(*tactile, config);
  tactile->slip_velocity_score = tactile->shear_velocity_mps.allFinite()
                                     ? tactile->shear_velocity_mps.norm()
                                     : 0.0;
  tactile->incipient_slip_score = tactile->slip_score;
}

inline double EstimateRotationalPatchMotionRad(
    const std::vector<HemisphereMotion>& motions,
    const Eigen::Vector2d& center_m,
    const Eigen::Vector2d& mean_tangent_delta_m) {
  double numerator = 0.0;
  double denominator = 0.0;

  for (const auto& motion : motions) {
    if (!IsFiniteHemisphereMotion(motion)) {
      continue;
    }

    const Eigen::Vector2d radius_m =
        motion.position_sensor_m.head<2>() - center_m;
    const Eigen::Vector2d tangent_delta_m =
        motion.delta_position_sensor_m.head<2>() - mean_tangent_delta_m;

    numerator +=
        radius_m.x() * tangent_delta_m.y() - radius_m.y() * tangent_delta_m.x();
    denominator += radius_m.squaredNorm();
  }

  if (denominator <= 1.0e-12 || !std::isfinite(numerator)) {
    return 0.0;
  }
  return numerator / denominator;
}

inline TactileState StepTactileTransition(
    const TactileState& tactile, const std::vector<HemisphereMotion>& motions,
    double dt, const GraspRolloutConfig& config = {}) {
  TactileState out = tactile;

  if (!std::isfinite(dt) || dt <= 0.0 || !tactile.valid ||
      !tactile.hasContact()) {
    RefreshPredictedTactileFields(&out, config);
    return out;
  }

  Eigen::Vector2d position_sum_m = Eigen::Vector2d::Zero();
  Eigen::Vector2d tangent_delta_sum_m = Eigen::Vector2d::Zero();
  double normal_delta_sum_m = 0.0;
  std::size_t valid_motion_count = 0;

  for (const auto& motion : motions) {
    if (!IsFiniteHemisphereMotion(motion)) {
      continue;
    }

    position_sum_m += motion.position_sensor_m.head<2>();
    tangent_delta_sum_m += motion.delta_position_sensor_m.head<2>();
    normal_delta_sum_m += motion.delta_position_sensor_m.z();
    ++valid_motion_count;
  }

  if (valid_motion_count == 0) {
    RefreshPredictedTactileFields(&out, config);
    return out;
  }

  const double count = static_cast<double>(valid_motion_count);
  const Eigen::Vector2d mean_position_m = position_sum_m / count;
  const Eigen::Vector2d mean_tangent_delta_m = tangent_delta_sum_m / count;
  const double mean_normal_delta_m = normal_delta_sum_m / count;
  Eigen::Vector2d base_centroid_m = mean_position_m;
  Eigen::Vector2d active_centroid_m = Eigen::Vector2d::Zero();
  if (ComputeActiveHemisphereCentroidM(tactile, &active_centroid_m)) {
    base_centroid_m = active_centroid_m;
  }

  const Eigen::Vector2d centroid_delta_m =
      ClampPatchVectorNorm(config.centroid_motion_gain * mean_tangent_delta_m,
                           config.max_centroid_step_m);
  for (auto& hemisphere : out.hemispheres) {
    if (hemisphere.contact && hemisphere.cop_sensor_m.allFinite()) {
      hemisphere.cop_sensor_m += centroid_delta_m;
    }
  }

  const Eigen::Vector2d current_shear_m =
      tactile.shear_displacement_m.allFinite() ? tactile.shear_displacement_m
                                               : Eigen::Vector2d::Zero();
  const Eigen::Vector2d shear_delta_m =
      config.tangential_shear_gain * mean_tangent_delta_m;
  out.shear_displacement_m =
      ClampPatchVectorNorm(current_shear_m + shear_delta_m, config.max_shear_m);
  out.shear_velocity_mps = Eigen::Vector2d::Zero();
  if (dt > 0.0) {
    out.shear_velocity_mps = shear_delta_m / dt;
  }

  const double rotation_delta_rad =
      config.rotational_shear_gain *
      EstimateRotationalPatchMotionRad(motions, base_centroid_m,
                                       mean_tangent_delta_m);
  const double current_rotation_rad =
      std::isfinite(tactile.rotational_shear_rad) ? tactile.rotational_shear_rad
                                                  : 0.0;
  out.rotational_shear_rad = current_rotation_rad + rotation_delta_rad;
  out.rotational_shear_velocity_radps =
      dt > 0.0 ? rotation_delta_rad / dt : 0.0;

  const double current_normal_force_n =
      tactile.total_force_n.allFinite()
          ? std::max(0.0, tactile.total_force_n.z())
          : 0.0;
  out.total_force_n.z() =
      current_normal_force_n +
      config.normal_force_gain_n_per_m * mean_normal_delta_m;

  double confidence = Clamp01(tactile.confidence);
  if (mean_normal_delta_m >= 0.0) {
    confidence += config.closing_confidence_gain_per_m * mean_normal_delta_m;
  } else {
    confidence -=
        config.separating_confidence_loss_per_m * std::abs(mean_normal_delta_m);
  }
  confidence -=
      config.tangential_confidence_loss_per_m * mean_tangent_delta_m.norm();

  const Eigen::Vector2d predicted_centroid_m =
      base_centroid_m + centroid_delta_m;
  const double edge_risk =
      ComputeTactileContactEdgeRisk(predicted_centroid_m, config);
  confidence -= config.edge_confidence_loss_gain *
                std::max(0.0, edge_risk - config.edge_risk_limit);
  out.confidence = Clamp01(confidence);

  RefreshPredictedTactileFields(&out, config);
  return out;
}

}  // namespace mppi_core
