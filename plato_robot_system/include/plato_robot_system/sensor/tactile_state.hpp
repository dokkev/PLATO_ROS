// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cstddef>
#include <string>
#include <vector>

namespace plato_robot_system::sensor
{

struct HemisphereState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t hemisphere_index{0};
  bool contact{false};
  Eigen::Vector2d cop_sensor_m{Eigen::Vector2d::Zero()};
  double normal_force_n{0.0};
  double confidence{1.0};
};

struct TactileState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kNoContact = 0;
  static constexpr int kFewContacts = 1;
  static constexpr int kEnoughContacts = 2;

  bool valid{false};
  double stamp_sec{0.0};

  int sensor_index{-1};
  std::string frame_name{};

  int contact_state{kNoContact};
  Eigen::Vector3d total_force_n{Eigen::Vector3d::Zero()};

  Eigen::Vector2d shear_displacement_m{Eigen::Vector2d::Zero()};
  double rotational_shear_rad{0.0};

  Eigen::Vector2d shear_velocity_mps{Eigen::Vector2d::Zero()};
  double rotational_shear_velocity_radps{0.0};

  double slip_score{0.0};
  double slip_velocity_score{0.0};
  double incipient_slip_score{0.0};

  double confidence{1.0};

  std::vector<HemisphereState, Eigen::aligned_allocator<HemisphereState>> hemispheres{};

  bool HasContact() const;
  bool HasEnoughContact() const;
  bool ReadyForContactControl() const;
  std::size_t ActiveHemisphereCount() const;
  std::size_t HemisphereCount() const;
  bool HasActiveHemisphereContact() const;
  double ActiveHemisphereNormalForceN() const;
  bool HasSufficientActiveHemispheres(std::size_t min_active_hemispheres) const;
};

using TactileStateVector =
  std::vector<TactileState, Eigen::aligned_allocator<TactileState>>;

}  // namespace plato_robot_system::sensor
