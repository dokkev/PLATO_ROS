#include "plato_robot_system/sensor/tactile_grip_observation.hpp"

#include <algorithm>
#include <cmath>

namespace plato_robot_system::sensor
{
namespace
{

bool IsFinite(const double value)
{
  return std::isfinite(value);
}

bool SameFrameName(
  const std::string & lhs,
  const std::string_view rhs)
{
  return lhs.size() == rhs.size() &&
         std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

}  // namespace

int TactileGripObservation::ContactCount() const
{
  return (contact_a ? 1 : 0) + (contact_b ? 1 : 0);
}

int TactileGripObservation::EnoughContactCount() const
{
  return (enough_contact_a ? 1 : 0) + (enough_contact_b ? 1 : 0);
}

const char * TactileContactStateName(const int contact_state)
{
  switch (contact_state) {
    case TactileState::kNoContact:
      return "no_contact";
    case TactileState::kFewContacts:
      return "few_contacts";
    case TactileState::kEnoughContacts:
      return "enough_contacts";
    default:
      return "unknown";
  }
}

const TactileState * FindTactileStateByFrame(
  const TactileStateVector & tactile_sensors,
  const std::string_view frame_name)
{
  for (const auto & tactile : tactile_sensors) {
    if (SameFrameName(tactile.frame_name, frame_name)) {
      return &tactile;
    }
  }
  return nullptr;
}

bool ExtractTactileNormalForceN(
  const TactileState * tactile,
  double * normal_force_n)
{
  if (normal_force_n == nullptr || tactile == nullptr || !tactile->valid) {
    return false;
  }

  double force_n = tactile->ActiveHemisphereNormalForceN();
  if (!IsFinite(force_n) || force_n <= 0.0) {
    if (!tactile->total_force_n.allFinite()) {
      return false;
    }
    force_n = tactile->total_force_n.z();
  }
  if (!IsFinite(force_n)) {
    return false;
  }

  *normal_force_n = std::max(0.0, force_n);
  return true;
}

TactileGripObservation ObserveTactileGrip(
  const TactileStateVector & tactile_sensors,
  const TactileGripObservationConfig & config)
{
  TactileGripObservation observation;
  const TactileState * tactile_a =
    FindTactileStateByFrame(tactile_sensors, config.frame_a_name);
  const TactileState * tactile_b =
    FindTactileStateByFrame(tactile_sensors, config.frame_b_name);

  const bool has_force_a = ExtractTactileNormalForceN(tactile_a, &observation.force_a_n);
  const bool has_force_b = ExtractTactileNormalForceN(tactile_b, &observation.force_b_n);

  if (config.use_tactile_presence_for_contact) {
    observation.contact_a =
      tactile_a != nullptr && tactile_a->valid && tactile_a->HasContact();
    observation.contact_b =
      tactile_b != nullptr && tactile_b->valid && tactile_b->HasContact();
    observation.enough_contact_a =
      tactile_a != nullptr && tactile_a->valid && tactile_a->HasEnoughContact();
    observation.enough_contact_b =
      tactile_b != nullptr && tactile_b->valid && tactile_b->HasEnoughContact();
    observation.lost_contact_a =
      tactile_a == nullptr || !tactile_a->valid ||
      tactile_a->contact_state == TactileState::kNoContact;
    observation.lost_contact_b =
      tactile_b == nullptr || !tactile_b->valid ||
      tactile_b->contact_state == TactileState::kNoContact;
  } else {
    observation.enough_contact_a =
      has_force_a && observation.force_a_n >= config.min_contact_force_n;
    observation.enough_contact_b =
      has_force_b && observation.force_b_n >= config.min_contact_force_n;
    observation.contact_a = observation.enough_contact_a;
    observation.contact_b = observation.enough_contact_b;
    observation.lost_contact_a =
      !has_force_a || observation.force_a_n < config.min_contact_force_n;
    observation.lost_contact_b =
      !has_force_b || observation.force_b_n < config.min_contact_force_n;
  }

  observation.valid_force = has_force_a && has_force_b;
  if (!observation.valid_force) {
    return observation;
  }

  switch (config.force_aggregation) {
    case TactileForceAggregation::kMin:
      observation.measured_force_n =
        std::min(observation.force_a_n, observation.force_b_n);
      break;
    case TactileForceAggregation::kAverage:
      observation.measured_force_n =
        0.5 * (observation.force_a_n + observation.force_b_n);
      break;
  }
  observation.valid_force = IsFinite(observation.measured_force_n);
  return observation;
}

}  // namespace plato_robot_system::sensor
