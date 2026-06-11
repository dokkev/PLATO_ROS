// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace mppi_core {

struct VirtualObjectState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Isometry3d pose_world{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 1> velocity_world{
      Eigen::Matrix<double, 6, 1>::Zero()};
  double weight{1.0};
  bool valid{false};
};

inline bool IsValidVirtualObjectState(const VirtualObjectState& state) {
  return state.valid && state.pose_world.matrix().allFinite() &&
         state.velocity_world.allFinite() && std::isfinite(state.weight) &&
         state.weight >= 0.0;
}

}  // namespace mppi_core
