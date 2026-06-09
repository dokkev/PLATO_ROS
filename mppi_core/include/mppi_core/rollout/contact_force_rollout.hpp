// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <Eigen/Core>

#include "mppi_core/contact/contact_force_projection.hpp"
#include "mppi_core/tactile/tactile_state.hpp"

namespace mppi_core {

struct ContactForceRolloutConfig {
  bool enable_force_projection_update{true};

  // Normal force update.
  double force_lowpass_alpha{0.5};
  double max_predicted_normal_force_n{20.0};

  // Shear update from tangential force.
  double shear_force_gain_m_per_n_s{1.0e-4};

  // Rotational shear update from torsional moment.
  double rotational_shear_gain_rad_per_nm_s{1.0e-2};

  // Confidence update.
  double friction_violation_confidence_decay{0.2};
  double negative_normal_confidence_decay{0.5};

  // Contact classification. These are rollout model parameters, not costs.
  std::size_t min_active_hemisphere_count{4};
  double min_contact_confidence{1.0e-3};

  // Slip score normalization.
  double shear_ref_m{1.0e-3};
  double rotational_shear_ref_rad{2.0e-2};
};

inline double ForceRolloutClamp01(double x) {
  if (!std::isfinite(x)) {
    return 0.0;
  }
  return std::clamp(x, 0.0, 1.0);
}

inline double SafePositiveReference(double x, double fallback) {
  if (!std::isfinite(x) || x <= 0.0) {
    return fallback;
  }
  return x;
}

inline void MarkAllContactHemispheresInactive(TactileState* tactile) {
  if (tactile == nullptr) {
    return;
  }
  for (auto& hemisphere : tactile->hemispheres) {
    hemisphere.contact = false;
    hemisphere.normal_force_n = 0.0;
  }
}

inline void UpdatePredictedHemisphereForces(
    const ContactForceProjectionResult& projection, TactileState* tactile) {
  if (tactile == nullptr) {
    return;
  }

  for (auto& hemisphere : tactile->hemispheres) {
    for (const auto& force : projection.hemisphere_forces) {
      if (force.hemisphere_index != hemisphere.hemisphere_index) {
        continue;
      }
      hemisphere.contact = force.normal_force_n > 0.0;
      hemisphere.normal_force_n = force.normal_force_n;
      hemisphere.confidence = ForceRolloutClamp01(tactile->confidence);
      break;
    }
  }
}

inline std::size_t CountPositiveNormalForceContacts(
    const ContactForceProjectionResult& projection) {
  std::size_t count = 0;
  for (const auto& force : projection.hemisphere_forces) {
    if (force.normal_force_n > 0.0) {
      ++count;
    }
  }
  return count;
}

inline int PredictForceRolloutContactState(
    const TactileState& tactile, const ContactForceRolloutConfig& config) {
  if (!tactile.valid || !tactile.total_force_n.allFinite() ||
      tactile.total_force_n.z() <= 0.0 ||
      ForceRolloutClamp01(tactile.confidence) <=
          config.min_contact_confidence ||
      tactile.activeHemisphereCount() == 0) {
    return TactileState::kNoContact;
  }

  if (config.min_active_hemisphere_count > 0 &&
      tactile.activeHemisphereCount() >= config.min_active_hemisphere_count) {
    return TactileState::kEnoughContacts;
  }
  return TactileState::kFewContacts;
}

inline double ComputeForceRolloutSlipScore(
    const TactileState& tactile, const ContactForceRolloutConfig& config) {
  const double shear_ref_m = SafePositiveReference(config.shear_ref_m, 1.0e-3);
  const double rotational_ref_rad =
      SafePositiveReference(config.rotational_shear_ref_rad, 2.0e-2);

  const double shear_score = tactile.shear_displacement_m.allFinite()
                                 ? tactile.shear_displacement_m.norm() /
                                       shear_ref_m
                                 : 0.0;
  const double rotational_score = std::isfinite(tactile.rotational_shear_rad)
                                      ? std::abs(tactile.rotational_shear_rad) /
                                            rotational_ref_rad
                                      : 0.0;

  return shear_score + rotational_score;
}

inline void StepTactileStateFromProjectedForce(
    const ContactForceProjectionResult& projection, double dt,
    const ContactForceRolloutConfig& config, TactileState* tactile) {
  if (tactile == nullptr || !config.enable_force_projection_update ||
      !projection.valid || !std::isfinite(dt) || dt < 0.0) {
    return;
  }

  tactile->valid = true;

  const double alpha = ForceRolloutClamp01(config.force_lowpass_alpha);
  const double old_normal_force_n =
      tactile->total_force_n.allFinite()
          ? std::max(0.0, tactile->total_force_n.z())
          : 0.0;
  const double projected_normal_force_n =
      std::clamp(projection.total_normal_force_n, 0.0,
                 std::max(0.0, SafePositiveReference(
                                   config.max_predicted_normal_force_n, 20.0)));

  tactile->total_force_n =
      Eigen::Vector3d{projection.net_tangential_force_n.x(),
                      projection.net_tangential_force_n.y(),
                      (1.0 - alpha) * old_normal_force_n +
                          alpha * projected_normal_force_n};

  const Eigen::Vector2d tangential_force_n =
      projection.net_tangential_force_n.allFinite()
          ? projection.net_tangential_force_n
          : Eigen::Vector2d::Zero();
  const Eigen::Vector2d shear_delta_m =
      config.shear_force_gain_m_per_n_s * tangential_force_n * dt;
  const Eigen::Vector2d current_shear_m =
      tactile->shear_displacement_m.allFinite()
          ? tactile->shear_displacement_m
          : Eigen::Vector2d::Zero();
  tactile->shear_displacement_m = current_shear_m + shear_delta_m;
  tactile->shear_velocity_mps =
      config.shear_force_gain_m_per_n_s * tangential_force_n;

  const double torsional_moment_nm =
      std::isfinite(projection.net_torsional_moment_nm)
          ? projection.net_torsional_moment_nm
          : 0.0;
  const double rotational_delta_rad =
      config.rotational_shear_gain_rad_per_nm_s * torsional_moment_nm * dt;
  const double current_rotational_shear_rad =
      std::isfinite(tactile->rotational_shear_rad)
          ? tactile->rotational_shear_rad
          : 0.0;
  tactile->rotational_shear_rad =
      current_rotational_shear_rad + rotational_delta_rad;
  tactile->rotational_shear_velocity_radps =
      config.rotational_shear_gain_rad_per_nm_s * torsional_moment_nm;

  double confidence = ForceRolloutClamp01(tactile->confidence);
  if (projection.total_normal_force_n <= 0.0) {
    confidence -= config.negative_normal_confidence_decay;
  }
  confidence -= config.friction_violation_confidence_decay *
                std::max(0.0, projection.total_friction_violation);
  tactile->confidence = ForceRolloutClamp01(confidence);

  UpdatePredictedHemisphereForces(projection, tactile);

  tactile->slip_score = ComputeForceRolloutSlipScore(*tactile, config);
  tactile->slip_velocity_score =
      tactile->shear_velocity_mps.allFinite()
          ? tactile->shear_velocity_mps.norm()
          : 0.0;
  tactile->incipient_slip_score = tactile->slip_score;

  tactile->contact_state = PredictForceRolloutContactState(*tactile, config);
  if (tactile->contact_state == TactileState::kNoContact) {
    MarkAllContactHemispheresInactive(tactile);
  }
}

}  // namespace mppi_core
