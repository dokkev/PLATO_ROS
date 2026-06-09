// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace mppi_core {

// Sensor-specific contact state from NARI-Touch tactile_state.contact_state.
enum class NariTouchContactState : int {
  kNoContact = 0,
  kFewContacts = 1,
  kEnoughContacts = 2,
};

inline constexpr std::size_t kNariTouchUnitCount = 8;

struct NariTouchUnitState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Sensor-local unit center in meters, using tactile frame X/Y axes.
  Eigen::Vector2d position_m{Eigen::Vector2d::Zero()};

  bool contact{false};

  // Local center-of-pressure offset from position_m.
  Eigen::Vector2d cop{Eigen::Vector2d::Zero()};

  // Optional per-unit normal force if the upstream NARI source provides it.
  double normal_force_n{0.0};
};

inline std::array<Eigen::Vector2d, kNariTouchUnitCount>
NariTouchUnitPositionsM() {
  return {
      Eigen::Vector2d{-0.003, -0.009}, Eigen::Vector2d{-0.003, -0.003},
      Eigen::Vector2d{-0.003, 0.003},  Eigen::Vector2d{-0.003, 0.009},
      Eigen::Vector2d{0.003, -0.009},  Eigen::Vector2d{0.003, -0.003},
      Eigen::Vector2d{0.003, 0.003},   Eigen::Vector2d{0.003, 0.009},
  };
}

struct NariTouchState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NariTouchState() {
    const auto unit_positions = NariTouchUnitPositionsM();
    for (std::size_t i = 0; i < units.size(); ++i) {
      units[i].position_m = unit_positions[i];
    }
  }

  bool valid{false};
  double stamp_sec{0.0};

  int sensor_index{-1};
  std::string frame_name{};

  NariTouchContactState contact_state{NariTouchContactState::kNoContact};

  // Sensor-level aggregate force from the NARI source.
  Eigen::Vector3d force_n{Eigen::Vector3d::Zero()};

  // Sensor-level aggregate shear/slip features from the NARI source.
  Eigen::Vector2d shear_displacement_m{Eigen::Vector2d::Zero()};
  double rotational_shear_rad{0.0};

  Eigen::Vector2d shear_velocity_mps{Eigen::Vector2d::Zero()};
  double rotational_shear_velocity_radps{0.0};

  double slip_score{0.0};
  double slip_velocity_score{0.0};
  double incipient_slip_score{0.0};

  std::array<NariTouchUnitState, kNariTouchUnitCount> units{};

  bool hasContact() const {
    if (contact_state == NariTouchContactState::kFewContacts ||
        contact_state == NariTouchContactState::kEnoughContacts) {
      return true;
    }
    for (const auto& unit : units) {
      if (unit.contact) {
        return true;
      }
    }
    return false;
  }

  bool hasEnoughContact() const {
    return contact_state == NariTouchContactState::kEnoughContacts;
  }

  bool readyForMppiStart() const { return valid && hasEnoughContact(); }

  std::size_t contactUnitCount() const {
    std::size_t count = 0;
    for (const auto& unit : units) {
      if (unit.contact) {
        ++count;
      }
    }
    return count;
  }
};

constexpr int ToContactStateValue(NariTouchContactState state) {
  return static_cast<int>(state);
}

inline bool ComputeNariTouchContactCentroidM(const NariTouchState& sensor,
                                             Eigen::Vector2d* centroid_m) {
  if (centroid_m == nullptr) {
    return false;
  }

  Eigen::Vector2d weighted_sum = Eigen::Vector2d::Zero();
  double weight_sum = 0.0;

  for (const auto& unit : sensor.units) {
    if (!unit.contact) {
      continue;
    }

    double weight = 1.0;
    if (std::isfinite(unit.normal_force_n) && unit.normal_force_n > 0.0) {
      weight = unit.normal_force_n;
    }

    Eigen::Vector2d contact_position_m = unit.position_m;
    if (unit.cop.allFinite()) {
      contact_position_m += unit.cop;
    }

    weighted_sum += weight * contact_position_m;
    weight_sum += weight;
  }

  if (weight_sum <= 0.0) {
    return false;
  }

  *centroid_m = weighted_sum / weight_sum;
  return true;
}

}  // namespace mppi_core
