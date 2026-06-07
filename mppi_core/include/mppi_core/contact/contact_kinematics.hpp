// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/skew.hpp>
#include <vector>

#include "mppi_core/robot/robot_state.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "mppi_core/tactile/tactile_transition.hpp"

namespace mppi_core {

struct PinocchioContactKinematicsContext {
  const pinocchio::Model* model{nullptr};
  pinocchio::Data* data{nullptr};
  pinocchio::FrameIndex sensor_frame_id{
      std::numeric_limits<pinocchio::FrameIndex>::max()};

  // Maps the Pinocchio sensor-frame z displacement to the tactile rollout
  // convention. +1 means sensor +z is closing/compression, -1 flips z so
  // HemisphereMotion::delta_position_sensor_m.z() remains positive closing.
  double normal_axis_sign{1.0};
};

inline bool IsValidContactKinematicsContext(
    const PinocchioContactKinematicsContext& context) {
  return context.model != nullptr && context.data != nullptr &&
         context.sensor_frame_id < context.model->frames.size() &&
         context.data->oMi.size() == context.model->joints.size();
}

inline bool IsValidContactKinematicsInput(
    const RobotState& robot, const TactileState& tactile,
    const Eigen::Ref<const Eigen::VectorXd>& tangent_step,
    const PinocchioContactKinematicsContext& context) {
  if (!IsValidRobotState(robot) || !tactile.valid ||
      !IsValidContactKinematicsContext(context)) {
    return false;
  }
  if (robot.q_des.size() != static_cast<Eigen::Index>(context.model->nq) ||
      tangent_step.size() != static_cast<Eigen::Index>(context.model->nv)) {
    return false;
  }
  return robot.q_des.allFinite() && tangent_step.allFinite();
}

inline Eigen::Vector3d ApplyTactileNormalAxisConvention(
    const Eigen::Vector3d& delta_sensor_m,
    const PinocchioContactKinematicsContext& context) {
  Eigen::Vector3d out = delta_sensor_m;
  if (std::isfinite(context.normal_axis_sign) &&
      context.normal_axis_sign < 0.0) {
    out.z() = -out.z();
  }
  return out;
}

inline bool ComputeContactPointJacobianSensor(
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& frame_jacobian_sensor,
    const Eigen::Vector3d& position_sensor_m,
    Eigen::Matrix<double, 3, Eigen::Dynamic>* point_jacobian_sensor) {
  if (point_jacobian_sensor == nullptr || !position_sensor_m.allFinite() ||
      frame_jacobian_sensor.rows() != 6) {
    return false;
  }

  // Pinocchio LOCAL frame Jacobian is expressed in the tactile sensor frame.
  // Rows 0:3 are frame-origin linear motion, rows 3:6 are angular motion.
  // For a point fixed at p in that frame:
  //   v_point = v_origin + omega x p.
  // Since omega x p = -skew(p) * omega, the point Jacobian is:
  //   J_point = J_linear - skew(p) * J_angular.
  // Tactile convention: x/y are tangent axes, z is the local normal axis.
  *point_jacobian_sensor = frame_jacobian_sensor.topRows<3>() -
                           pinocchio::skew(position_sensor_m) *
                               frame_jacobian_sensor.bottomRows<3>();
  return point_jacobian_sensor->allFinite();
}

inline std::vector<HemisphereMotion> ComputeHemisphereMotions(
    const RobotState& robot, const TactileState& tactile,
    const Eigen::Ref<const Eigen::VectorXd>& tangent_step,
    const PinocchioContactKinematicsContext& context) {
  std::vector<HemisphereMotion> motions;

  // This kinematic approximation assumes tangent_step is the integrated
  // Pinocchio tangent-space step over the current rollout interval.
  if (!IsValidContactKinematicsInput(robot, tactile, tangent_step, context) ||
      tactile.hemispheres.empty()) {
    return motions;
  }

  std::size_t active_count = 0;
  for (const auto& hemisphere : tactile.hemispheres) {
    if (hemisphere.contact && hemisphere.cop_sensor_m.allFinite()) {
      ++active_count;
    }
  }
  if (active_count == 0) {
    return motions;
  }
  motions.reserve(active_count);

  auto& data = *context.data;
  const auto& model = *context.model;

  pinocchio::computeJointJacobians(model, data, robot.q_des);
  pinocchio::updateFramePlacements(model, data);

  Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian(6, model.nv);
  frame_jacobian.setZero();
  pinocchio::getFrameJacobian(model, data, context.sensor_frame_id,
                              pinocchio::LOCAL, frame_jacobian);

  Eigen::Matrix<double, 3, Eigen::Dynamic> point_jacobian(3, model.nv);

  for (const auto& hemisphere : tactile.hemispheres) {
    if (!hemisphere.contact || !hemisphere.cop_sensor_m.allFinite()) {
      continue;
    }
    const Eigen::Vector3d position_sensor_m{hemisphere.cop_sensor_m.x(),
                                            hemisphere.cop_sensor_m.y(), 0.0};

    if (!ComputeContactPointJacobianSensor(frame_jacobian, position_sensor_m,
                                           &point_jacobian)) {
      continue;
    }

    HemisphereMotion motion;
    motion.hemisphere_index = hemisphere.hemisphere_index;
    motion.position_sensor_m = position_sensor_m;
    motion.delta_position_sensor_m = ApplyTactileNormalAxisConvention(
        point_jacobian * tangent_step, context);

    if (motion.delta_position_sensor_m.allFinite()) {
      motions.push_back(motion);
    }
  }

  return motions;
}

}  // namespace mppi_core
