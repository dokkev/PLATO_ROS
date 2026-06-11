// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <vector>

#include "mppi_core/object/object_geometry_query.hpp"
#include "mppi_core/object/virtual_object_state.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"
#include "mppi_core/tactile/tactile_state.hpp"

namespace mppi_core {

struct ObjectContactPredictionConfig {
  double contact_distance_threshold_m{0.001};
  double contact_stiffness_n_per_m{500.0};
  double max_predicted_normal_force_n{10.0};
  double min_predicted_contact_force_n{0.01};
};

struct ObjectHemisphereContactPrediction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};
  bool measured_contact{false};
  bool predicted_contact{false};

  std::size_t sensor_index{0};
  std::size_t hemisphere_index{0};

  Eigen::Vector3d point_world_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tactile_normal_world{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d object_normal_world{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d closest_point_world{Eigen::Vector3d::Zero()};

  double signed_distance_m{0.0};
  double predicted_normal_force_n{0.0};
};

std::vector<ObjectHemisphereContactPrediction,
            Eigen::aligned_allocator<ObjectHemisphereContactPrediction>>
PredictObjectHemisphereContacts(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const std::vector<TactileSensorContext>& tactile_contexts,
    const VirtualObjectState& object,
    const ObjectGeometryHandle& geometry,
    const ObjectContactPredictionConfig& config = {});

}  // namespace mppi_core
