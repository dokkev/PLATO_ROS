// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mppi_core/object/object_prior.hpp"

namespace mppi_core {

struct ObjectSurfaceQueryResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Signed distance convention:
  //   > 0: point is outside the object with a gap.
  //   = 0: point lies on the object surface.
  //   < 0: point is inside/penetrating the object.
  double signed_distance_m{0.0};
  Eigen::Vector3d closest_point_world{Eigen::Vector3d::Zero()};

  // Outward object surface normal in world coordinates.
  Eigen::Vector3d normal_world{Eigen::Vector3d::UnitZ()};
  bool valid{false};
};

class ObjectGeometryQuery {
 public:
  ObjectGeometryQuery() = default;
  explicit ObjectGeometryQuery(ObjectGeometryHandle geometry);

  const ObjectGeometryHandle& geometry() const { return geometry_; }

  ObjectSurfaceQueryResult Query(
      const Eigen::Isometry3d& object_pose_world,
      const Eigen::Vector3d& point_world_m) const;

 private:
  ObjectGeometryHandle geometry_;
};

ObjectSurfaceQueryResult QueryObjectSurface(
    const ObjectGeometryHandle& geometry,
    const Eigen::Isometry3d& object_pose_world,
    const Eigen::Vector3d& point_world_m);

}  // namespace mppi_core
