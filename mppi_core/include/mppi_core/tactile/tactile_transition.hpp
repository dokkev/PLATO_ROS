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
  double birth_approach_velocity_mps{0.002};
  double loss_unloading_velocity_mps{0.002};

  double max_shear_m{0.003};
  double max_rotation_rad{0.05};
  double born_normal_force_n{0.05};
  double born_confidence{0.5};
  double contact_confidence_decay{1.0};
};

inline double Clamp01(double x) {
  if (!std::isfinite(x)) {
    return 0.0;
  }
  return std::clamp(x, 0.0, 1.0);
}

inline constexpr double kInactiveConfidence = 0.0;
inline constexpr std::size_t kEnoughContactHemisphereCount = 2;

inline bool HasNeighborInActiveSet(const HemisphereGeometry& geometry,
                                   const std::vector<bool>& active_by_index) {
  for (const std::size_t neighbor : geometry.neighbors) {
    if (neighbor < active_by_index.size() && active_by_index[neighbor]) {
      return true;
    }
  }
  return false;
}

inline void RefreshTactileAggregate(
    TactileState* tactile, const TactileTransitionConfig& /*config*/) {
  if (tactile == nullptr) {
    return;
  }

  tactile->valid = true;
  const std::size_t active_count = tactile->activeHemisphereCount();
  if (active_count == 0) {
    tactile->contact_state = TactileState::kNoContact;
  } else if (active_count < kEnoughContactHemisphereCount) {
    tactile->contact_state = TactileState::kFewContacts;
  } else {
    tactile->contact_state = TactileState::kEnoughContacts;
  }

  tactile->total_force_n =
      Eigen::Vector3d{0.0, 0.0, tactile->activeHemisphereNormalForceN()};

  if (active_count == 0) {
    tactile->confidence = kInactiveConfidence;
  } else {
    tactile->confidence = Clamp01(tactile->confidence);
  }

  if (!tactile->shear_displacement_m.allFinite()) {
    tactile->shear_displacement_m = Eigen::Vector2d::Zero();
  }
  if (!std::isfinite(tactile->rotational_shear_rad)) {
    tactile->rotational_shear_rad = 0.0;
  }

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
  const bool shear_bad =
      shear_norm > config.max_shear_m ||
      std::abs(out.rotational_shear_rad) > config.max_rotation_rad;
  const bool shear_ok = !shear_bad;

  for (std::size_t i = 0; i < out.hemispheres.size(); ++i) {
    auto& hemi = out.hemispheres[i];
    const auto& motion = motions[i];
    const auto& geometry = sensor_context.hemispheres[i];
    hemi.hemisphere_index = tactile.hemispheres[i].hemisphere_index;

    if (hemi.contact) {
      if (hemi.cop_sensor_m.allFinite()) {
        hemi.cop_sensor_m += motion.velocity_sensor_xy_mps * dt;
      }
      const double normal_force =
          std::max(0.0, std::isfinite(hemi.normal_force_n)
                            ? hemi.normal_force_n
                            : 0.0);
      const bool unloading =
          -motion.normal_velocity_mps > config.loss_unloading_velocity_mps;

      if (unloading || shear_bad) {
        hemi.contact = false;
        hemi.normal_force_n = 0.0;
        hemi.confidence = kInactiveConfidence;
      } else {
        hemi.normal_force_n = normal_force;
        hemi.confidence =
            Clamp01(hemi.confidence * config.contact_confidence_decay);
      }
      continue;
    }

    const bool adjacent =
        HasNeighborInActiveSet(geometry, active_by_index);
    if (!adjacent) {
      hemi.contact = false;
      hemi.normal_force_n = 0.0;
      hemi.confidence = kInactiveConfidence;
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
    const bool approaching =
        motion.normal_velocity_mps > config.birth_approach_velocity_mps ||
        tangent_approach > config.birth_approach_velocity_mps;

    if (neighbor_term > 0.0 && approaching && shear_ok) {
      hemi.contact = true;
      hemi.normal_force_n = config.born_normal_force_n;
      hemi.confidence = Clamp01(config.born_confidence);
    } else {
      hemi.contact = false;
      hemi.normal_force_n = 0.0;
      hemi.confidence = kInactiveConfidence;
    }
  }

  RefreshTactileAggregate(&out, config);
  return out;
}

}  // namespace mppi_core
