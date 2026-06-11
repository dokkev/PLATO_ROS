// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace mppi_core {

enum class ObjectGeometryType {
  kUnknown,
  kMesh,
  kUrdf,
  kBox,
  kSphere,
  kCylinder,
};

struct ObjectGeometryHandle {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string name;
  ObjectGeometryType type{ObjectGeometryType::kUnknown};
  std::string uri;

  // Primitive dimensions. Meaning depends on `type`:
  //   kBox: x/y/z side lengths in meters.
  //   kSphere: x is radius in meters.
  //   kCylinder: x is radius, z is height in meters.
  Eigen::Vector3d primitive_size_m{Eigen::Vector3d::Zero()};

  bool valid{false};
};

struct ObjectPrior {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string name;
  ObjectGeometryHandle geometry;
  Eigen::Isometry3d initial_pose_world{Eigen::Isometry3d::Identity()};

  Eigen::Vector3d position_std_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_std_rad{Eigen::Vector3d::Zero()};

  bool valid{false};
};

inline bool HasObjectGeometry(const ObjectGeometryHandle& geometry) {
  return geometry.valid || !geometry.name.empty() || !geometry.uri.empty() ||
         geometry.type != ObjectGeometryType::kUnknown;
}

inline bool IsValidObjectGeometry(const ObjectGeometryHandle& geometry) {
  if (!HasObjectGeometry(geometry)) {
    return true;
  }
  if (!geometry.valid || !geometry.primitive_size_m.allFinite()) {
    return false;
  }
  if (geometry.type == ObjectGeometryType::kUnknown) {
    return false;
  }
  if ((geometry.type == ObjectGeometryType::kMesh ||
       geometry.type == ObjectGeometryType::kUrdf) &&
      geometry.uri.empty()) {
    return false;
  }
  if (geometry.type == ObjectGeometryType::kBox) {
    return geometry.primitive_size_m.x() > 0.0 &&
           geometry.primitive_size_m.y() > 0.0 &&
           geometry.primitive_size_m.z() > 0.0;
  }
  if (geometry.type == ObjectGeometryType::kSphere) {
    return geometry.primitive_size_m.x() > 0.0;
  }
  if (geometry.type == ObjectGeometryType::kCylinder) {
    return geometry.primitive_size_m.x() > 0.0 &&
           geometry.primitive_size_m.z() > 0.0;
  }
  return true;
}

inline bool HasObjectPrior(const ObjectPrior& prior) {
  return prior.valid || !prior.name.empty() || HasObjectGeometry(prior.geometry);
}

inline bool IsValidObjectPrior(const ObjectPrior& prior) {
  if (!HasObjectPrior(prior)) {
    return true;
  }
  return prior.valid && IsValidObjectGeometry(prior.geometry) &&
         prior.initial_pose_world.matrix().allFinite() &&
         prior.position_std_m.allFinite() &&
         prior.rpy_std_rad.allFinite();
}

}  // namespace mppi_core
