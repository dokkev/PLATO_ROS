// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cstddef>
#include <string>
#include <vector>

namespace mppi_core {

// MPPI-level tactile reasoning unit.
//
// A hemisphere may internally aggregate multiple low-level sensing nodes,
// but MPPI does not reason over raw nodes directly.
struct HemisphereState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t hemisphere_index{0};

  // Whether this hemisphere/contact unit is currently active.
  bool contact{false};

  // Contact/candidate point in the tactile sensor frame.
  //
  // Convention:
  //   x/y lie on the tactile surface,
  //   z is the local tactile normal axis.
  //
  // Despite the historical "cop" name, this field is the planning point used by
  // MPPI. For active hemispheres, it is the CoP-derived contact point. For
  // inactive hemispheres, it stores the fixed hemisphere center and can be used
  // as a candidate contact point by future contact-birth models.
  Eigen::Vector2d cop_sensor_m{Eigen::Vector2d::Zero()};

  // Hemisphere-level normal force.
  //
  // Meaningful when contact == true.
  double normal_force_n{0.0};

  // Best-effort confidence of this hemisphere measurement or prediction.
  // Expected range: [0, 1].
  double confidence{1.0};
};

// Tactile state for one tactile sensor.
//
// TactileState is used both for measured tactile observations and predicted
// tactile state inside MPPI rollout.
//
// It contains:
//   1. hemisphere-level contact data,
//   2. sensor-level aggregate tactile data.
//
// It intentionally does not contain Pinocchio Jacobians. Per-hemisphere contact
// Jacobians are derived from the robot state and the runtime contact kinematics
// context during rollout.
struct TactileState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // NARI-Touch contact_state values.
  //
  // Keep these as sensor-specific integer constants instead of introducing
  // another enum/type. The raw NARI state can be assigned directly.
  static constexpr int kNoContact = 0;
  static constexpr int kFewContacts = 1;
  static constexpr int kEnoughContacts = 2;

  bool valid{false};
  double stamp_sec{0.0};

  // Sensor identity and frame.
  int sensor_index{-1};
  std::string frame_name{};

  // Sensor-level contact gate from NARI-Touch tactile_state.contact_state.
  //
  // Used mainly to decide whether MPPI should start/stop.
  int contact_state{kNoContact};

  // Sensor-level aggregate force.
  Eigen::Vector3d total_force_n{Eigen::Vector3d::Zero()};

  // Sensor-level aggregate translational shear displacement.
  Eigen::Vector2d shear_displacement_m{Eigen::Vector2d::Zero()};

  // Sensor-level aggregate rotational shear displacement.
  double rotational_shear_rad{0.0};

  // Optional derived velocities, computed by adapter/filter if available.
  Eigen::Vector2d shear_velocity_mps{Eigen::Vector2d::Zero()};
  double rotational_shear_velocity_radps{0.0};

  // Dimensionless planning/control features derived from tactile signals.
  double slip_score{0.0};
  double slip_velocity_score{0.0};
  double incipient_slip_score{0.0};

  // Best-effort sensor-level confidence.
  // Expected range: [0, 1].
  double confidence{1.0};

  // Hemisphere-level tactile measurements / predictions.
  std::vector<HemisphereState, Eigen::aligned_allocator<HemisphereState>>
      hemispheres{};

  // NARI top-level contact_state gate. Use this for MPPI start/stop decisions,
  // not for detailed hemisphere topology reasoning.
  bool hasContact() const {
    return contact_state == kFewContacts || contact_state == kEnoughContacts;
  }

  bool hasEnoughContact() const { return contact_state == kEnoughContacts; }

  bool readyForMppiStart() const { return valid && hasEnoughContact(); }

  // Hemisphere-level contact topology.
  //
  // These helpers are used by rollout/cost to reason about contact survival,
  // loss, and birth.
  std::size_t activeHemisphereCount() const {
    std::size_t count = 0;
    for (const auto& hemi : hemispheres) {
      if (hemi.contact) {
        ++count;
      }
    }
    return count;
  }

  std::size_t hemisphereCount() const { return hemispheres.size(); }

  // Hemisphere topology check. Use this for rollout, cost, contact survival,
  // and future contact-birth reasoning.
  bool hasActiveHemisphereContact() const {
    return activeHemisphereCount() > 0;
  }

  double activeHemisphereNormalForceN() const {
    double total = 0.0;
    for (const auto& hemi : hemispheres) {
      if (hemi.contact) {
        total += hemi.normal_force_n;
      }
    }
    return total;
  }

  bool hasSufficientActiveHemispheres(
      const std::size_t min_active_hemispheres) const {
    return activeHemisphereCount() >= min_active_hemispheres;
  }
};

}  // namespace mppi_core
