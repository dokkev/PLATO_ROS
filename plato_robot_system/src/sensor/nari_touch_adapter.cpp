// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "plato_robot_system/sensor/nari_touch_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace plato_robot_system::sensor
{

int ToTactileContactState(const NARITouchContactState state)
{
  switch (state) {
    case NARITouchContactState::kNoContact:
      return TactileState::kNoContact;
    case NARITouchContactState::kFewContacts:
      return TactileState::kFewContacts;
    case NARITouchContactState::kEnoughContacts:
      return TactileState::kEnoughContacts;
  }
  return TactileState::kNoContact;
}

double ComputeNARITouchConfidence(
  const NARITouchSample & sample,
  const TactileState & tactile_state)
{
  double confidence = 0.0;
  if (sample.HasContact()) {
    confidence += 0.25;
  }
  if (sample.contact_state == NARITouchContactState::kEnoughContacts) {
    confidence += 0.45;
  } else if (sample.contact_state == NARITouchContactState::kFewContacts) {
    confidence += 0.25;
  }
  if (tactile_state.ActiveHemisphereCount() > 0) {
    confidence += 0.15;
  }
  if (tactile_state.total_force_n.allFinite() && tactile_state.total_force_n.z() > 0.0) {
    confidence += 0.15;
  }
  return std::clamp(confidence, 0.0, 1.0);
}

TactileState ConvertNARITouchToTactileState(const NARITouchSample & sample)
{
  TactileState out;

  out.valid = sample.valid;
  out.stamp_sec = sample.stamp_sec;
  out.sensor_index = sample.sensor_index;
  out.frame_name = sample.frame_name;

  out.contact_state = ToTactileContactState(sample.contact_state);
  out.total_force_n = sample.force_n;

  out.shear_displacement_m = sample.shear_displacement_m;
  out.rotational_shear_rad = sample.rotational_shear_rad;
  out.shear_velocity_mps = sample.shear_velocity_mps;
  out.rotational_shear_velocity_radps = sample.rotational_shear_velocity_radps;

  out.slip_score = sample.slip_score;
  out.slip_velocity_score = sample.slip_velocity_score;
  out.incipient_slip_score = sample.incipient_slip_score;

  out.hemispheres.clear();
  out.hemispheres.reserve(sample.units.size());
  for (std::size_t i = 0; i < sample.units.size(); ++i) {
    const auto & unit = sample.units[i];
    const bool force_contact =
      std::isfinite(unit.normal_force_n) &&
      unit.normal_force_n >= kNARITouchHemisphereContactThresholdN;
    const bool hemisphere_contact = unit.contact || force_contact;

    HemisphereState hemisphere;
    hemisphere.hemisphere_index = i;
    hemisphere.contact = hemisphere_contact;
    hemisphere.cop_sensor_m = unit.position_m;
    if (hemisphere_contact && unit.cop.allFinite()) {
      hemisphere.cop_sensor_m += unit.cop;
    }
    if (std::isfinite(unit.normal_force_n) && unit.normal_force_n > 0.0) {
      hemisphere.normal_force_n = unit.normal_force_n;
    } else {
      hemisphere.normal_force_n = 0.0;
    }
    hemisphere.confidence = hemisphere_contact ? 1.0 : 0.0;
    out.hemispheres.push_back(hemisphere);
  }

  out.confidence = ComputeNARITouchConfidence(sample, out);
  return out;
}

}  // namespace plato_robot_system::sensor
