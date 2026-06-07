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
#include <utility>
#include <vector>

#include "mppi_core/contact/hemisphere_motion.hpp"
#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"

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
  if (!IsValid(robot) || !tactile.valid ||
      !IsValidContactKinematicsContext(context)) {
    return false;
  }
  if (robot.q.size() != static_cast<Eigen::Index>(context.model->nq) ||
      tangent_step.size() != static_cast<Eigen::Index>(context.model->nv)) {
    return false;
  }
  return robot.q.allFinite() && tangent_step.allFinite();
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

  pinocchio::computeJointJacobians(model, data, robot.q);
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

inline bool HasMatchingTactileSensorContext(
    const TactileState& tactile, const TactileSensorContext& sensor_context) {
  if (tactile.sensor_index >= 0 && sensor_context.sensor_index >= 0 &&
      tactile.sensor_index != sensor_context.sensor_index) {
    return false;
  }
  return tactile.hemispheres.size() == sensor_context.hemispheres.size();
}

inline Eigen::Vector3d HemisphereLocalPointSensorM(
    const HemisphereState& hemisphere, const HemisphereGeometry& geometry) {
  if (hemisphere.cop_sensor_m.allFinite()) {
    return Eigen::Vector3d{hemisphere.cop_sensor_m.x(),
                           hemisphere.cop_sensor_m.y(), 0.0};
  }
  return geometry.center_sensor_m;
}

inline std::vector<HemisphereMotion> ComputeHemisphereMotions(
    const RobotState& robot, const RobotState& next_robot,
    const TactileState& tactile, const TactileSensorContext& sensor_context,
    double dt) {
  std::vector<HemisphereMotion> motions;
  const PinocchioContactKinematicsContext* context =
      sensor_context.kinematics;
  if (!IsValid(robot) || !IsValid(next_robot) || !tactile.valid ||
      context == nullptr || !IsValidContactKinematicsContext(*context) ||
      !HasMatchingTactileSensorContext(tactile, sensor_context) ||
      !std::isfinite(dt) || dt <= 0.0 ||
      robot.q.size() != static_cast<Eigen::Index>(context->model->nq) ||
      next_robot.q.size() != static_cast<Eigen::Index>(context->model->nq) ||
      robot.qdot.size() != static_cast<Eigen::Index>(context->model->nv)) {
    return motions;
  }

  const auto& model = *context->model;
  pinocchio::Data data_now(model);
  pinocchio::Data data_next(model);

  pinocchio::forwardKinematics(model, data_now, robot.q);
  pinocchio::updateFramePlacements(model, data_now);
  pinocchio::forwardKinematics(model, data_next, next_robot.q);
  pinocchio::updateFramePlacements(model, data_next);

  auto& jacobian_data = *context->data;
  pinocchio::computeJointJacobians(model, jacobian_data, robot.q);
  pinocchio::updateFramePlacements(model, jacobian_data);

  Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian_sensor(6, model.nv);
  frame_jacobian_sensor.setZero();
  pinocchio::getFrameJacobian(model, jacobian_data, context->sensor_frame_id,
                              pinocchio::LOCAL, frame_jacobian_sensor);

  const Eigen::Matrix3d rotation_world_sensor =
      data_now.oMf[context->sensor_frame_id].rotation();
  Eigen::Matrix<double, 3, Eigen::Dynamic> point_jacobian_sensor(3, model.nv);

  motions.reserve(tactile.hemispheres.size());
  for (std::size_t i = 0; i < tactile.hemispheres.size(); ++i) {
    const auto& hemisphere = tactile.hemispheres[i];
    const auto& geometry = sensor_context.hemispheres[i];
    if (hemisphere.hemisphere_index != geometry.hemisphere_index) {
      return {};
    }

    const Eigen::Vector3d point_sensor_m =
        HemisphereLocalPointSensorM(hemisphere, geometry);
    if (!point_sensor_m.allFinite() || !geometry.normal_sensor.allFinite() ||
        geometry.normal_sensor.norm() <= 1.0e-12) {
      return {};
    }

    const Eigen::Vector3d point_world_now =
        data_now.oMf[context->sensor_frame_id].act(point_sensor_m);
    const Eigen::Vector3d point_world_next =
        data_next.oMf[context->sensor_frame_id].act(point_sensor_m);
    Eigen::Vector3d normal_world =
        rotation_world_sensor * geometry.normal_sensor.normalized();
    if (std::isfinite(context->normal_axis_sign) &&
        context->normal_axis_sign < 0.0) {
      normal_world = -normal_world;
    }

    HemisphereMotion motion;
    motion.hemisphere_index = hemisphere.hemisphere_index;
    motion.point_world_m = point_world_now;
    motion.normal_world = normal_world;
    motion.velocity_world_mps = (point_world_next - point_world_now) / dt;

    const Eigen::Vector3d velocity_sensor_mps =
        rotation_world_sensor.transpose() * motion.velocity_world_mps;
    const Eigen::Vector3d tactile_velocity_sensor_mps =
        ApplyTactileNormalAxisConvention(velocity_sensor_mps, *context);
    motion.velocity_sensor_xy_mps = tactile_velocity_sensor_mps.head<2>();
    motion.normal_velocity_mps =
        motion.normal_world.dot(motion.velocity_world_mps);

    motion.position_sensor_m = point_sensor_m;
    motion.delta_position_sensor_m = tactile_velocity_sensor_mps * dt;

    if (ComputeContactPointJacobianSensor(frame_jacobian_sensor,
                                          point_sensor_m,
                                          &point_jacobian_sensor)) {
      motion.J_contact_world = rotation_world_sensor * point_jacobian_sensor;
      motion.J_normal = motion.normal_world.transpose() * motion.J_contact_world;
    } else {
      motion.J_contact_world.resize(0, 0);
      motion.J_normal.resize(0);
    }

    if (!IsFiniteHemisphereMotion(motion)) {
      return {};
    }
    motions.push_back(std::move(motion));
  }

  return motions;
}

}  // namespace mppi_core
