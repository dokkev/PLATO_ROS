// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <vector>

namespace mppi_core {

struct VirtualObjectDisturbance {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d linear_velocity_world_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_world_radps{Eigen::Vector3d::Zero()};
  // Deprecated compatibility fields. Initial object pose uncertainty belongs
  // to ObjectPrior particles; rollout updates intentionally ignore these.
  Eigen::Vector3d position_offset_world_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_offset_world_rad{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_force_world_n{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_torque_world_nm{Eigen::Vector3d::Zero()};
  bool dropout{false};
  bool valid{true};
};

struct ContactMeasurementNoise {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double force_rate_noise_nps{0.0};
  Eigen::Vector2d cop_noise_sensor_m{Eigen::Vector2d::Zero()};
  double friction_scale{1.0};
  bool valid{true};
};

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

  VirtualObjectDisturbance object_disturbance;
  CommonGraspDisturbance common_grasp_disturbance;

  std::vector<TactileSensorDisturbance,
              Eigen::aligned_allocator<TactileSensorDisturbance>>
      tactile_sensor_disturbances;
  std::vector<ContactMeasurementNoise,
              Eigen::aligned_allocator<ContactMeasurementNoise>>
      contact_measurement_noises;

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
