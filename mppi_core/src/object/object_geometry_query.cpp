// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/object/object_geometry_query.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mppi_core {
namespace {

constexpr double kTiny = 1.0e-12;

bool IsFinitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool HasPositiveBoxSize(const Eigen::Vector3d& size_m) {
  return size_m.allFinite() && size_m.x() > 0.0 && size_m.y() > 0.0 &&
         size_m.z() > 0.0;
}

bool HasPositiveSphereSize(const Eigen::Vector3d& size_m) {
  return size_m.allFinite() && size_m.x() > 0.0;
}

bool HasPositiveCylinderSize(const Eigen::Vector3d& size_m) {
  return size_m.allFinite() && size_m.x() > 0.0 && size_m.z() > 0.0;
}

ObjectGeometryType EffectivePrimitiveType(const ObjectGeometryHandle& geometry) {
  if (geometry.type == ObjectGeometryType::kBox &&
      HasPositiveBoxSize(geometry.primitive_size_m)) {
    return ObjectGeometryType::kBox;
  }
  if (geometry.type == ObjectGeometryType::kSphere &&
      HasPositiveSphereSize(geometry.primitive_size_m)) {
    return ObjectGeometryType::kSphere;
  }
  if (geometry.type == ObjectGeometryType::kCylinder &&
      HasPositiveCylinderSize(geometry.primitive_size_m)) {
    return ObjectGeometryType::kCylinder;
  }
  if ((geometry.type == ObjectGeometryType::kUrdf ||
       geometry.type == ObjectGeometryType::kMesh) &&
      HasPositiveBoxSize(geometry.primitive_size_m)) {
    return ObjectGeometryType::kBox;
  }
  return ObjectGeometryType::kUnknown;
}

ObjectSurfaceQueryResult MakeWorldResult(
    const Eigen::Isometry3d& pose_world,
    const Eigen::Vector3d& closest_point_object_m,
    const Eigen::Vector3d& normal_object,
    double signed_distance_m) {
  ObjectSurfaceQueryResult result;
  if (!closest_point_object_m.allFinite() || !normal_object.allFinite() ||
      normal_object.norm() <= kTiny || !std::isfinite(signed_distance_m)) {
    return result;
  }
  result.closest_point_world = pose_world * closest_point_object_m;
  result.normal_world = pose_world.linear() * normal_object.normalized();
  result.signed_distance_m = signed_distance_m;
  result.valid = result.closest_point_world.allFinite() &&
                 result.normal_world.allFinite();
  return result;
}

ObjectSurfaceQueryResult QueryBox(
    const Eigen::Isometry3d& pose_world,
    const Eigen::Vector3d& p_object_m,
    const Eigen::Vector3d& size_m) {
  if (!p_object_m.allFinite() || !HasPositiveBoxSize(size_m)) {
    return {};
  }

  const Eigen::Vector3d half = 0.5 * size_m;
  Eigen::Vector3d clamped = p_object_m;
  for (Eigen::Index i = 0; i < 3; ++i) {
    clamped[i] = std::clamp(clamped[i], -half[i], half[i]);
  }

  const Eigen::Vector3d outside = p_object_m - clamped;
  const double outside_distance = outside.norm();
  if (outside_distance > kTiny) {
    return MakeWorldResult(pose_world, clamped, outside / outside_distance,
                           outside_distance);
  }

  Eigen::Index closest_axis = 0;
  double closest_margin = half.x() - std::abs(p_object_m.x());
  for (Eigen::Index axis = 1; axis < 3; ++axis) {
    const double margin = half[axis] - std::abs(p_object_m[axis]);
    if (margin < closest_margin) {
      closest_margin = margin;
      closest_axis = axis;
    }
  }

  Eigen::Vector3d closest = p_object_m;
  const double sign = p_object_m[closest_axis] >= 0.0 ? 1.0 : -1.0;
  closest[closest_axis] = sign * half[closest_axis];
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  normal[closest_axis] = sign;
  return MakeWorldResult(pose_world, closest, normal, -closest_margin);
}

ObjectSurfaceQueryResult QuerySphere(
    const Eigen::Isometry3d& pose_world,
    const Eigen::Vector3d& p_object_m,
    double radius_m) {
  if (!p_object_m.allFinite() || !IsFinitePositive(radius_m)) {
    return {};
  }

  const double norm = p_object_m.norm();
  const Eigen::Vector3d normal =
      norm > kTiny ? p_object_m / norm : Eigen::Vector3d::UnitX();
  return MakeWorldResult(pose_world, radius_m * normal, normal,
                         norm - radius_m);
}

ObjectSurfaceQueryResult QueryCylinder(
    const Eigen::Isometry3d& pose_world,
    const Eigen::Vector3d& p_object_m,
    double radius_m,
    double height_m) {
  if (!p_object_m.allFinite() || !IsFinitePositive(radius_m) ||
      !IsFinitePositive(height_m)) {
    return {};
  }

  const double half_height = 0.5 * height_m;
  const Eigen::Vector2d xy = p_object_m.head<2>();
  const double xy_norm = xy.norm();
  const Eigen::Vector2d radial =
      xy_norm > kTiny ? xy / xy_norm : Eigen::Vector2d::UnitX();

  const bool inside_radial = xy_norm <= radius_m;
  const bool inside_height = std::abs(p_object_m.z()) <= half_height;
  if (inside_radial && inside_height) {
    const double radial_margin = radius_m - xy_norm;
    const double cap_margin = half_height - std::abs(p_object_m.z());
    if (radial_margin <= cap_margin) {
      const Eigen::Vector3d normal{radial.x(), radial.y(), 0.0};
      const Eigen::Vector3d closest{
          radius_m * radial.x(), radius_m * radial.y(), p_object_m.z()};
      return MakeWorldResult(pose_world, closest, normal, -radial_margin);
    }
    const double cap_sign = p_object_m.z() >= 0.0 ? 1.0 : -1.0;
    const Eigen::Vector3d closest{p_object_m.x(), p_object_m.y(),
                                  cap_sign * half_height};
    return MakeWorldResult(pose_world, closest,
                           cap_sign * Eigen::Vector3d::UnitZ(), -cap_margin);
  }

  ObjectSurfaceQueryResult best;
  best.signed_distance_m = std::numeric_limits<double>::infinity();

  const Eigen::Vector3d side_closest{
      radius_m * radial.x(), radius_m * radial.y(),
      std::clamp(p_object_m.z(), -half_height, half_height)};
  const double side_distance = (p_object_m - side_closest).norm();
  best = MakeWorldResult(pose_world, side_closest,
                         Eigen::Vector3d{radial.x(), radial.y(), 0.0},
                         side_distance);

  const double cap_signs[2] = {-1.0, 1.0};
  for (double cap_sign : cap_signs) {
    const double clamped_radius = std::min(xy_norm, radius_m);
    const Eigen::Vector3d cap_closest{
        clamped_radius * radial.x(), clamped_radius * radial.y(),
        cap_sign * half_height};
    const double cap_distance = (p_object_m - cap_closest).norm();
    if (cap_distance < best.signed_distance_m) {
      best = MakeWorldResult(pose_world, cap_closest,
                             cap_sign * Eigen::Vector3d::UnitZ(),
                             cap_distance);
    }
  }
  return best;
}

}  // namespace

ObjectGeometryQuery::ObjectGeometryQuery(ObjectGeometryHandle geometry)
    : geometry_(std::move(geometry)) {}

ObjectSurfaceQueryResult ObjectGeometryQuery::Query(
    const Eigen::Isometry3d& object_pose_world,
    const Eigen::Vector3d& point_world_m) const {
  return QueryObjectSurface(geometry_, object_pose_world, point_world_m);
}

ObjectSurfaceQueryResult QueryObjectSurface(
    const ObjectGeometryHandle& geometry,
    const Eigen::Isometry3d& object_pose_world,
    const Eigen::Vector3d& point_world_m) {
  if (!IsValidObjectGeometry(geometry) ||
      !object_pose_world.matrix().allFinite() ||
      !point_world_m.allFinite()) {
    return {};
  }

  const Eigen::Vector3d p_object_m = object_pose_world.inverse() * point_world_m;
  switch (EffectivePrimitiveType(geometry)) {
    case ObjectGeometryType::kBox:
      return QueryBox(object_pose_world, p_object_m,
                      geometry.primitive_size_m);
    case ObjectGeometryType::kSphere:
      return QuerySphere(object_pose_world, p_object_m,
                         geometry.primitive_size_m.x());
    case ObjectGeometryType::kCylinder:
      return QueryCylinder(object_pose_world, p_object_m,
                           geometry.primitive_size_m.x(),
                           geometry.primitive_size_m.z());
    default:
      return {};
  }
}

}  // namespace mppi_core
