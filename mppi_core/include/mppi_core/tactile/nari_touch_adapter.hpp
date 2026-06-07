// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "mppi_core/tactile/nari_touch_state.hpp"
#include "mppi_core/tactile/tactile_state.hpp"

namespace mppi_core {

inline int ToTactileContactState(NariTouchContactState state) {
  switch (state) {
    case NariTouchContactState::kNoContact:
      return TactileState::kNoContact;
    case NariTouchContactState::kFewContacts:
      return TactileState::kFewContacts;
    case NariTouchContactState::kEnoughContacts:
      return TactileState::kEnoughContacts;
  }
  return TactileState::kNoContact;
}

inline double ComputeNariTouchConfidence(const NariTouchState& nari,
                                         const TactileState& tactile) {
  double confidence = 0.0;
  if (nari.hasContact()) {
    confidence += 0.25;
  }
  if (nari.contact_state == NariTouchContactState::kEnoughContacts) {
    confidence += 0.45;
  } else if (nari.contact_state == NariTouchContactState::kFewContacts) {
    confidence += 0.25;
  }
  if (tactile.activeHemisphereCount() > 0) {
    confidence += 0.15;
  }
  if (tactile.total_force_n.allFinite() && tactile.total_force_n.z() > 0.0) {
    confidence += 0.15;
  }
  return std::clamp(confidence, 0.0, 1.0);
}

inline TactileState ConvertNariTouchToTactileState(
    const NariTouchState& nari) {
  TactileState out;

  out.valid = nari.valid;
  out.stamp_sec = nari.stamp_sec;
  out.sensor_index = nari.sensor_index;
  out.frame_name = nari.frame_name;

  out.contact_state = static_cast<int>(nari.contact_state);
  out.total_force_n = nari.force_n;

  out.shear_displacement_m = nari.shear_displacement_m;
  out.rotational_shear_rad = nari.rotational_shear_rad;
  out.shear_velocity_mps = nari.shear_velocity_mps;
  out.rotational_shear_velocity_radps = nari.rotational_shear_velocity_radps;

  out.slip_score = nari.slip_score;
  out.slip_velocity_score = nari.slip_velocity_score;
  out.incipient_slip_score = nari.incipient_slip_score;

  out.hemispheres.clear();
  out.hemispheres.reserve(nari.units.size());
  for (std::size_t i = 0; i < nari.units.size(); ++i) {
    const auto& unit = nari.units[i];

    HemisphereState hemi;
    hemi.hemisphere_index = i;
    hemi.contact = unit.contact;
    hemi.cop_sensor_m = unit.position_m;
    if (unit.contact && unit.cop.allFinite()) {
      hemi.cop_sensor_m += unit.cop;
    }
    if (unit.contact && std::isfinite(unit.normal_force_n) &&
        unit.normal_force_n > 0.0) {
      hemi.normal_force_n = unit.normal_force_n;
    } else {
      hemi.normal_force_n = 0.0;
    }
    hemi.confidence = unit.contact ? 1.0 : 0.0;
    out.hemispheres.push_back(hemi);
  }

  out.confidence = ComputeNariTouchConfidence(nari, out);
  return out;
}

}  // namespace mppi_core
