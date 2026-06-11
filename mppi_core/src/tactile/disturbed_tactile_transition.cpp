// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/tactile/disturbed_tactile_transition.hpp"

#include <algorithm>
#include <cmath>

namespace mppi_core {
namespace {

double ClampFinite(const double value, const double lower, const double upper) {
  if (!std::isfinite(value)) {
    return lower;
  }
  return std::clamp(value, lower, upper);
}

Eigen::Vector2d ClampNorm(const Eigen::Vector2d& value, const double max_norm) {
  if (!value.allFinite()) {
    return Eigen::Vector2d::Zero();
  }
  if (!std::isfinite(max_norm) || max_norm <= 0.0) {
    return value;
  }
  const double norm = value.norm();
  if (norm <= max_norm || norm <= 1.0e-12) {
    return value;
  }
  return value * (max_norm / norm);
}

bool ValidDisturbance(const TactileSensorDisturbance& disturbance) {
  return disturbance.valid &&
         disturbance.tangent_velocity_sensor_mps.allFinite() &&
         disturbance.cop_drift_velocity_sensor_mps.allFinite() &&
         std::isfinite(disturbance.rotational_velocity_radps) &&
         std::isfinite(disturbance.normal_force_rate_nps) &&
         std::isfinite(disturbance.friction_scale) &&
         disturbance.friction_scale > 0.0;
}

bool ValidInputs(const TactileState& tactile,
                 const std::vector<HemisphereMotion>& motions,
                 const TactileSensorContext& sensor_context,
                 const TactileSensorDisturbance& disturbance,
                 const double dt) {
  if (!tactile.valid ||
      tactile.hemispheres.size() != sensor_context.hemispheres.size() ||
      tactile.hemispheres.size() != motions.size() ||
      !ValidDisturbance(disturbance) || !std::isfinite(dt) || dt <= 0.0) {
    return false;
  }

  for (std::size_t i = 0; i < tactile.hemispheres.size(); ++i) {
    const auto& hemi = tactile.hemispheres[i];
    const auto& motion = motions[i];
    const auto& geometry = sensor_context.hemispheres[i];
    if (motion.hemisphere_index != hemi.hemisphere_index ||
        geometry.hemisphere_index != hemi.hemisphere_index ||
        !IsFiniteHemisphereMotion(motion)) {
      return false;
    }
  }
  return true;
}

std::vector<bool> ActiveByHemisphereIndex(const TactileState& tactile) {
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
  return active_by_index;
}

Eigen::Vector2d RelativeTangentVelocity(
    const HemisphereMotion& motion,
    const TactileSensorDisturbance& disturbance) {
  return motion.velocity_sensor_xy_mps -
         disturbance.tangent_velocity_sensor_mps +
         disturbance.cop_drift_velocity_sensor_mps;
}

}  // namespace

TactileState StepTactileStateWithDisturbance(
    const TactileState& tactile, const RobotState& /*robot*/,
    const RobotState& /*next_robot*/, const std::vector<HemisphereMotion>& motions,
    const TactileSensorContext& sensor_context,
    const DisturbedTactileTransitionConfig& config,
    const TactileSensorDisturbance& disturbance, const double dt) {
  TactileState out = tactile;
  if (!ValidInputs(tactile, motions, sensor_context, disturbance, dt)) {
    out.valid = false;
    return out;
  }

  const std::vector<bool> active_by_index =
      ActiveByHemisphereIndex(tactile);

  Eigen::Vector2d tangent_delta_sum_m = Eigen::Vector2d::Zero();
  Eigen::Vector2d tangent_velocity_sum_mps = Eigen::Vector2d::Zero();
  std::size_t active_motion_count = 0;
  for (std::size_t i = 0; i < tactile.hemispheres.size(); ++i) {
    if (!tactile.hemispheres[i].contact) {
      continue;
    }
    const Eigen::Vector2d relative_velocity =
        RelativeTangentVelocity(motions[i], disturbance);
    tangent_delta_sum_m += relative_velocity * dt;
    tangent_velocity_sum_mps += relative_velocity;
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
          RelativeTangentVelocity(motions[i], disturbance) * dt -
          mean_tangent_delta_m;
      numerator += radius_m.x() * tangent_delta_m.y() -
                   radius_m.y() * tangent_delta_m.x();
      denominator += radius_m.squaredNorm();
    }
    if (denominator > 1.0e-12 && std::isfinite(numerator)) {
      rotation_delta_rad = numerator / denominator;
    }
  }
  rotation_delta_rad += disturbance.rotational_velocity_radps * dt;

  const Eigen::Vector2d current_shear_m =
      tactile.shear_displacement_m.allFinite() ? tactile.shear_displacement_m
                                               : Eigen::Vector2d::Zero();
  const Eigen::Vector2d unclamped_shear_m =
      current_shear_m + mean_tangent_delta_m;
  const double current_rotation_rad =
      std::isfinite(tactile.rotational_shear_rad)
          ? tactile.rotational_shear_rad
          : 0.0;
  const double unclamped_rotation_rad =
      current_rotation_rad + rotation_delta_rad;

  out.shear_displacement_m =
      ClampNorm(unclamped_shear_m, config.max_abs_shear_m);
  out.shear_velocity_mps = mean_tangent_velocity_mps;
  out.rotational_shear_rad =
      ClampFinite(unclamped_rotation_rad, -config.max_abs_rotation_rad,
                  config.max_abs_rotation_rad);
  out.rotational_shear_velocity_radps = rotation_delta_rad / dt;

  const double friction_scale = std::max(0.1, disturbance.friction_scale);
  const double effective_max_shear = config.base.max_shear_m * friction_scale;
  const double effective_max_rotation =
      config.base.max_rotation_rad * friction_scale;
  const double shear_norm =
      unclamped_shear_m.allFinite() ? unclamped_shear_m.norm() : 0.0;
  const bool shear_bad =
      shear_norm > effective_max_shear ||
      std::abs(unclamped_rotation_rad) > effective_max_rotation;
  const bool shear_ok = !shear_bad;

  for (std::size_t i = 0; i < out.hemispheres.size(); ++i) {
    auto& hemi = out.hemispheres[i];
    const auto& motion = motions[i];
    const auto& geometry = sensor_context.hemispheres[i];
    hemi.hemisphere_index = tactile.hemispheres[i].hemisphere_index;

    if (hemi.contact) {
      const Eigen::Vector2d relative_velocity =
          RelativeTangentVelocity(motion, disturbance);
      if (hemi.cop_sensor_m.allFinite()) {
        hemi.cop_sensor_m += relative_velocity * dt;
      }

      const double normal_force =
          std::max(0.0, std::isfinite(hemi.normal_force_n)
                            ? hemi.normal_force_n
                            : 0.0);
      const double dF_robot =
          config.normal_stiffness_n_per_m *
          motion.normal_velocity_mps * dt;
      const double dF_disturbance =
          disturbance.normal_force_rate_nps * dt;
      hemi.normal_force_n =
          ClampFinite(normal_force + dF_robot + dF_disturbance,
                      0.0, config.max_normal_force_n);

      const bool unloading =
          -motion.normal_velocity_mps > config.base.loss_unloading_velocity_mps;
      const bool force_lost =
          config.lose_contact_below_min_force &&
          hemi.normal_force_n < config.min_contact_force_n;
      if (disturbance.dropout || unloading || shear_bad || force_lost) {
        hemi.contact = false;
        hemi.normal_force_n = 0.0;
        hemi.confidence = kInactiveConfidence;
      } else {
        hemi.confidence =
            Clamp01(hemi.confidence * config.base.contact_confidence_decay);
      }
      continue;
    }

    const bool adjacent =
        HasNeighborInActiveSet(geometry, active_by_index);
    if (!adjacent || disturbance.dropout) {
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

    const Eigen::Vector2d relative_velocity =
        RelativeTangentVelocity(motion, disturbance);
    const double tangent_approach =
        std::max(0.0, relative_velocity.dot(toward_candidate));
    const bool approaching =
        motion.normal_velocity_mps > config.base.birth_approach_velocity_mps ||
        tangent_approach > config.base.birth_approach_velocity_mps;

    if (approaching && shear_ok) {
      hemi.contact = true;
      hemi.normal_force_n = config.base.born_normal_force_n;
      hemi.confidence = Clamp01(config.base.born_confidence);
    } else {
      hemi.contact = false;
      hemi.normal_force_n = 0.0;
      hemi.confidence = kInactiveConfidence;
    }
  }

  RefreshTactileAggregate(&out, config.base);
  return out;
}

}  // namespace mppi_core
