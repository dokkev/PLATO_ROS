// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace mppi_core {

struct PinocchioContactKinematicsContext;

struct HemisphereGeometry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t hemisphere_index{0};

  Eigen::Vector3d center_sensor_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal_sensor{Eigen::Vector3d::UnitZ()};
  double radius_m{0.0};

  std::vector<std::size_t> neighbors;
};

struct TactileSensorContext {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int sensor_index{-1};

  const PinocchioContactKinematicsContext* kinematics{nullptr};

  std::vector<HemisphereGeometry> hemispheres;
};

}  // namespace mppi_core
