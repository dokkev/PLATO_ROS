// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <string>

namespace plato_robot_system::sensor
{

enum class NARITouchContactState : int
{
  kNoContact = 0,
  kFewContacts = 1,
  kEnoughContacts = 2,
};

inline constexpr std::size_t kNARITouchUnitCount = 8;
inline constexpr double kNARITouchCopToMScale = 1.0e-3;
inline constexpr double kNARITouchHemisphereContactThresholdN = 0.1;

struct NARITouchUnitState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2d position_m{Eigen::Vector2d::Zero()};
  bool contact{false};
  Eigen::Vector2d cop{Eigen::Vector2d::Zero()};
  double normal_force_n{0.0};
};

std::array<Eigen::Vector2d, kNARITouchUnitCount> NARITouchUnitPositionsM();

struct NARITouchSample
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NARITouchSample();

  bool valid{false};
  double stamp_sec{0.0};

  int sensor_index{-1};
  std::string frame_name{};

  NARITouchContactState contact_state{NARITouchContactState::kNoContact};

  Eigen::Vector3d force_n{Eigen::Vector3d::Zero()};
  Eigen::Vector2d shear_displacement_m{Eigen::Vector2d::Zero()};
  double rotational_shear_rad{0.0};

  Eigen::Vector2d shear_velocity_mps{Eigen::Vector2d::Zero()};
  double rotational_shear_velocity_radps{0.0};

  double slip_score{0.0};
  double slip_velocity_score{0.0};
  double incipient_slip_score{0.0};

  std::array<NARITouchUnitState, kNARITouchUnitCount> units{};

  bool HasContact() const;
  bool HasEnoughContact() const;
  bool ReadyForContactControl() const;
  std::size_t ContactUnitCount() const;
};

int ToContactStateValue(NARITouchContactState state);
bool ComputeNARITouchContactCentroidM(const NARITouchSample & sample, Eigen::Vector2d * centroid_m);

class NARITouch
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NARITouch();
  NARITouch(int sensor_index, std::string frame_name);

  void SetSensorInfo(int sensor_index, std::string frame_name);
  void Update(const NARITouchSample & sample);
  void Reset();

  const NARITouchSample & sample() const { return sample_; }

  bool valid() const { return sample_.valid; }
  bool HasContact() const { return sample_.HasContact(); }
  bool ReadyForContactControl() const { return sample_.ReadyForContactControl(); }

private:
  int sensor_index_{-1};
  std::string frame_name_{};

  NARITouchSample sample_{};
};

}  // namespace plato_robot_system::sensor
