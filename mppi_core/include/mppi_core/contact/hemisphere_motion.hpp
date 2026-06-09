// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstddef>

namespace mppi_core {

// Per-hemisphere kinematic signal used by grasp-state rollout.
//
// HemisphereMotion is produced by contact kinematics and consumed by tactile
// transition. It describes point motion and Jacobians; it is not an exact
// contact-force prediction.
struct HemisphereMotion {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t hemisphere_index{0};

  Eigen::Vector3d point_world_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal_world{Eigen::Vector3d::UnitZ()};

  Eigen::Vector3d velocity_world_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector2d velocity_sensor_xy_mps{Eigen::Vector2d::Zero()};

  // Convention: positive means closing/contact approach, negative means
  // separating/unloading along the tactile normal.
  double normal_velocity_mps{0.0};

  Eigen::MatrixXd J_contact_world;
  Eigen::RowVectorXd J_normal;

  // Compatibility fields for the older kinematic patch helper. New
  // GraspState rollout code uses the world velocity/Jacobian fields above.
  Eigen::Vector3d position_sensor_m{Eigen::Vector3d::Zero()};

  // Displacement of this contact/support point over the current rollout step,
  // expressed in the tactile sensor frame.
  // Convention: positive z means closing/compression along the tactile normal.
  Eigen::Vector3d delta_position_sensor_m{Eigen::Vector3d::Zero()};
};

inline bool IsFiniteHemisphereMotion(const HemisphereMotion& motion) {
  return motion.point_world_m.allFinite() && motion.normal_world.allFinite() &&
         motion.velocity_world_mps.allFinite() &&
         motion.velocity_sensor_xy_mps.allFinite() &&
         std::isfinite(motion.normal_velocity_mps) &&
         motion.position_sensor_m.allFinite() &&
         motion.delta_position_sensor_m.allFinite();
}

}  // namespace mppi_core
