// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "plato_robot_system/sensor/tactile_state.hpp"

namespace plato_robot_system::sensor
{

bool TactileState::HasContact() const
{
  return contact_state == kFewContacts || contact_state == kEnoughContacts;
}

bool TactileState::HasEnoughContact() const
{
  return contact_state == kEnoughContacts;
}

bool TactileState::ReadyForContactControl() const
{
  return valid && HasContact();
}

std::size_t TactileState::ActiveHemisphereCount() const
{
  std::size_t count = 0;
  for (const auto & hemisphere : hemispheres) {
    if (hemisphere.contact) {
      ++count;
    }
  }
  return count;
}

std::size_t TactileState::HemisphereCount() const
{
  return hemispheres.size();
}

bool TactileState::HasActiveHemisphereContact() const
{
  return ActiveHemisphereCount() > 0;
}

double TactileState::ActiveHemisphereNormalForceN() const
{
  double total_force_n = 0.0;
  for (const auto & hemisphere : hemispheres) {
    if (hemisphere.contact) {
      total_force_n += hemisphere.normal_force_n;
    }
  }
  return total_force_n;
}

bool TactileState::HasSufficientActiveHemispheres(
  const std::size_t min_active_hemispheres) const
{
  return ActiveHemisphereCount() >= min_active_hemispheres;
}

}  // namespace plato_robot_system::sensor
