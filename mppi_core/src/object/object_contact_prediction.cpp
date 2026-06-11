// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/object/object_contact_prediction.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

#include "mppi_core/contact/contact_kinematics.hpp"

namespace mppi_core {
namespace {

constexpr double kTiny = 1.0e-12;

double SafeNonnegative(double value) {
  return std::isfinite(value) ? std::max(0.0, value) : 0.0;
}

double PredictNormalForce(const ObjectSurfaceQueryResult& surface,
                          const ObjectContactPredictionConfig& config) {
  const double threshold_m =
      SafeNonnegative(config.contact_distance_threshold_m);
  const double stiffness = SafeNonnegative(config.contact_stiffness_n_per_m);
  const double max_force = SafeNonnegative(config.max_predicted_normal_force_n);
  const double compression_m = threshold_m - surface.signed_distance_m;
  if (!surface.valid || compression_m <= 0.0 || stiffness <= 0.0) {
    return 0.0;
  }
  return std::min(max_force, stiffness * compression_m);
}

bool ReadHemispherePointAndNormal(
    const TactileState& tactile,
    const TactileSensorContext& sensor_context,
    std::size_t hemisphere_index_in_vector,
    Eigen::Vector3d* point_sensor_m,
    Eigen::Vector3d* normal_sensor) {
  if (point_sensor_m == nullptr || normal_sensor == nullptr ||
      hemisphere_index_in_vector >= tactile.hemispheres.size()) {
    return false;
  }

  const auto& hemisphere = tactile.hemispheres[hemisphere_index_in_vector];
  if (sensor_context.hemispheres.size() == tactile.hemispheres.size()) {
    const auto& geometry =
        sensor_context.hemispheres[hemisphere_index_in_vector];
    if (geometry.hemisphere_index != hemisphere.hemisphere_index ||
        !geometry.center_sensor_m.allFinite() ||
        !geometry.normal_sensor.allFinite() ||
        geometry.normal_sensor.norm() <= kTiny) {
      return false;
    }
    *point_sensor_m = HemisphereLocalPointSensorM(hemisphere, geometry);
    *normal_sensor = geometry.normal_sensor.normalized();
    return point_sensor_m->allFinite();
  }

  if (!hemisphere.cop_sensor_m.allFinite()) {
    return false;
  }
  *point_sensor_m =
      Eigen::Vector3d{hemisphere.cop_sensor_m.x(),
                      hemisphere.cop_sensor_m.y(), 0.0};
  *normal_sensor = Eigen::Vector3d::UnitZ();
  return true;
}

}  // namespace

std::vector<ObjectHemisphereContactPrediction,
            Eigen::aligned_allocator<ObjectHemisphereContactPrediction>>
PredictObjectHemisphereContacts(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const std::vector<TactileSensorContext>& tactile_contexts,
    const VirtualObjectState& object,
    const ObjectGeometryHandle& geometry,
    const ObjectContactPredictionConfig& config) {
  std::vector<ObjectHemisphereContactPrediction,
              Eigen::aligned_allocator<ObjectHemisphereContactPrediction>>
      predictions;

  if (q.size() == 0 || !q.allFinite() ||
      tactile_sensors.size() != tactile_contexts.size() ||
      !IsValidVirtualObjectState(object) || !IsValidObjectGeometry(geometry)) {
    return predictions;
  }

  for (std::size_t sensor_i = 0; sensor_i < tactile_sensors.size(); ++sensor_i) {
    const auto& tactile = tactile_sensors[sensor_i];
    const auto& sensor_context = tactile_contexts[sensor_i];
    const auto* kinematics = sensor_context.kinematics;
    if (!tactile.valid || kinematics == nullptr ||
        !IsValidContactKinematicsContext(*kinematics) ||
        q.size() != static_cast<Eigen::Index>(kinematics->model->nq)) {
      continue;
    }

    pinocchio::Data data(*kinematics->model);
    pinocchio::forwardKinematics(*kinematics->model, data, q);
    pinocchio::updateFramePlacements(*kinematics->model, data);
    const auto& sensor_pose_world = data.oMf[kinematics->sensor_frame_id];
    const Eigen::Matrix3d rotation_world_sensor =
        sensor_pose_world.rotation();

    for (std::size_t hemi_i = 0; hemi_i < tactile.hemispheres.size(); ++hemi_i) {
      const auto& hemisphere = tactile.hemispheres[hemi_i];
      Eigen::Vector3d point_sensor_m = Eigen::Vector3d::Zero();
      Eigen::Vector3d normal_sensor = Eigen::Vector3d::UnitZ();
      if (!ReadHemispherePointAndNormal(tactile, sensor_context, hemi_i,
                                        &point_sensor_m, &normal_sensor)) {
        continue;
      }

      const ObjectSurfaceQueryResult surface = QueryObjectSurface(
          geometry, object.pose_world, sensor_pose_world.act(point_sensor_m));
      if (!surface.valid) {
        continue;
      }

      ObjectHemisphereContactPrediction prediction;
      prediction.valid = true;
      prediction.measured_contact = hemisphere.contact;
      prediction.sensor_index = tactile.sensor_index >= 0
                                    ? static_cast<std::size_t>(
                                          tactile.sensor_index)
                                    : sensor_i;
      prediction.hemisphere_index = hemisphere.hemisphere_index;
      prediction.point_world_m = sensor_pose_world.act(point_sensor_m);
      prediction.tactile_normal_world =
          rotation_world_sensor * normal_sensor.normalized();
      if (std::isfinite(kinematics->normal_axis_sign) &&
          kinematics->normal_axis_sign < 0.0) {
        prediction.tactile_normal_world = -prediction.tactile_normal_world;
      }
      prediction.object_normal_world = surface.normal_world;
      prediction.closest_point_world = surface.closest_point_world;
      prediction.signed_distance_m = surface.signed_distance_m;
      prediction.predicted_normal_force_n = PredictNormalForce(surface, config);
      prediction.predicted_contact =
          prediction.signed_distance_m <=
              SafeNonnegative(config.contact_distance_threshold_m) &&
          prediction.predicted_normal_force_n >=
              SafeNonnegative(config.min_predicted_contact_force_n);
      predictions.push_back(std::move(prediction));
    }
  }

  return predictions;
}

}  // namespace mppi_core
