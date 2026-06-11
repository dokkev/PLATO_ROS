#ifndef PLATO_ROBOT_SYSTEM__SENSOR__TACTILE_GRIP_OBSERVATION_HPP_
#define PLATO_ROBOT_SYSTEM__SENSOR__TACTILE_GRIP_OBSERVATION_HPP_

#include <string>
#include <string_view>

#include "plato_robot_system/sensor/tactile_state.hpp"

namespace plato_robot_system::sensor
{

enum class TactileForceAggregation
{
  kMin,
  kAverage,
};

struct TactileGripObservationConfig
{
  std::string frame_a_name;
  std::string frame_b_name;
  double min_contact_force_n{0.05};
  bool use_tactile_presence_for_contact{true};
  TactileForceAggregation force_aggregation{TactileForceAggregation::kMin};
};

struct TactileGripObservation
{
  double measured_force_n{0.0};
  double force_a_n{0.0};
  double force_b_n{0.0};
  bool contact_a{false};
  bool contact_b{false};
  bool enough_contact_a{false};
  bool enough_contact_b{false};
  bool lost_contact_a{true};
  bool lost_contact_b{true};
  bool valid_force{false};

  int ContactCount() const;
  int EnoughContactCount() const;
};

const char * TactileContactStateName(int contact_state);

const TactileState * FindTactileStateByFrame(
  const TactileStateVector & tactile_sensors,
  std::string_view frame_name);

bool ExtractTactileNormalForceN(
  const TactileState * tactile,
  double * normal_force_n);

TactileGripObservation ObserveTactileGrip(
  const TactileStateVector & tactile_sensors,
  const TactileGripObservationConfig & config);

}  // namespace plato_robot_system::sensor

#endif  // PLATO_ROBOT_SYSTEM__SENSOR__TACTILE_GRIP_OBSERVATION_HPP_
