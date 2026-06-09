// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "plato_robot_system/sensor/nari_touch.hpp"

#include <cmath>
#include <utility>

namespace plato_robot_system::sensor
{

std::array<Eigen::Vector2d, kNARITouchUnitCount> NARITouchUnitPositionsM()
{
  return {
    Eigen::Vector2d{-0.003, -0.009}, Eigen::Vector2d{-0.003, -0.003},
    Eigen::Vector2d{-0.003, 0.003},  Eigen::Vector2d{-0.003, 0.009},
    Eigen::Vector2d{0.003, -0.009},  Eigen::Vector2d{0.003, -0.003},
    Eigen::Vector2d{0.003, 0.003},   Eigen::Vector2d{0.003, 0.009},
  };
}

NARITouchSample::NARITouchSample()
{
  const auto unit_positions = NARITouchUnitPositionsM();
  for (std::size_t i = 0; i < units.size(); ++i) {
    units[i].position_m = unit_positions[i];
  }
}

bool NARITouchSample::HasContact() const
{
  if (
    contact_state == NARITouchContactState::kFewContacts ||
    contact_state == NARITouchContactState::kEnoughContacts)
  {
    return true;
  }
  for (const auto & unit : units) {
    if (unit.contact) {
      return true;
    }
  }
  return false;
}

bool NARITouchSample::HasEnoughContact() const
{
  return contact_state == NARITouchContactState::kEnoughContacts;
}

bool NARITouchSample::ReadyForContactControl() const
{
  return valid && HasContact();
}

std::size_t NARITouchSample::ContactUnitCount() const
{
  std::size_t count = 0;
  for (const auto & unit : units) {
    if (unit.contact) {
      ++count;
    }
  }
  return count;
}

int ToContactStateValue(const NARITouchContactState state)
{
  return static_cast<int>(state);
}

bool ComputeNARITouchContactCentroidM(
  const NARITouchSample & sample, Eigen::Vector2d * centroid_m)
{
  if (centroid_m == nullptr) {
    return false;
  }

  Eigen::Vector2d weighted_sum = Eigen::Vector2d::Zero();
  double weight_sum = 0.0;

  for (const auto & unit : sample.units) {
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

NARITouch::NARITouch() = default;

NARITouch::NARITouch(const int sensor_index, std::string frame_name)
{
  SetSensorInfo(sensor_index, std::move(frame_name));
}

void NARITouch::SetSensorInfo(const int sensor_index, std::string frame_name)
{
  sensor_index_ = sensor_index;
  frame_name_ = std::move(frame_name);
  sample_.sensor_index = sensor_index_;
  sample_.frame_name = frame_name_;
}

void NARITouch::Update(const NARITouchSample & sample)
{
  sample_ = sample;
  if (sample_.sensor_index < 0) {
    sample_.sensor_index = sensor_index_;
  }
  if (sample_.frame_name.empty()) {
    sample_.frame_name = frame_name_;
  }
}

void NARITouch::Reset()
{
  sample_ = NARITouchSample{};
  sample_.sensor_index = sensor_index_;
  sample_.frame_name = frame_name_;
}

}  // namespace plato_robot_system::sensor
