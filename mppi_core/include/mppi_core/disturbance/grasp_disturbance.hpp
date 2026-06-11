// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <vector>

namespace mppi_core {

struct CommonGraspDisturbance {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2d tangent_velocity_grasp_mps{Eigen::Vector2d::Zero()};
  double rotational_velocity_radps{0.0};
  double normal_force_rate_nps{0.0};
  Eigen::Vector2d cop_drift_velocity_grasp_mps{Eigen::Vector2d::Zero()};
  bool dropout{false};
  bool valid{true};
};

struct TactileSensorDisturbance {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2d tangent_velocity_sensor_mps{Eigen::Vector2d::Zero()};
  double rotational_velocity_radps{0.0};
  double normal_force_rate_nps{0.0};
  double friction_scale{1.0};
  Eigen::Vector2d cop_drift_velocity_sensor_mps{Eigen::Vector2d::Zero()};
  bool dropout{false};
  bool valid{true};
};

struct GraspDisturbanceStep {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CommonGraspDisturbance common_grasp_disturbance;

  std::vector<TactileSensorDisturbance,
              Eigen::aligned_allocator<TactileSensorDisturbance>>
      tactile_sensor_disturbances;

  bool valid{true};
};

struct GraspDisturbanceSequence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<GraspDisturbanceStep,
              Eigen::aligned_allocator<GraspDisturbanceStep>>
      steps;

  std::size_t horizonSteps() const { return steps.size(); }
};

}  // namespace mppi_core
