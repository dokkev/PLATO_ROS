// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/costs/robust_grasp_state_cost.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "mppi_core/contact/contact_kinematics.hpp"

namespace mppi_core {
namespace {

double Square(const double value) { return value * value; }

bool IsFiniteAndNonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

void ValidateConfig(const RobustGraspStateCostConfig& config) {
  if (!IsFiniteAndNonnegative(config.contact_loss_weight) ||
      !IsFiniteAndNonnegative(config.support_weight) ||
      !IsFiniteAndNonnegative(config.target_normal_force_n) ||
      !IsFiniteAndNonnegative(config.min_normal_force_per_sensor_n) ||
      !IsFiniteAndNonnegative(config.max_normal_force_per_sensor_n) ||
      !IsFiniteAndNonnegative(config.force_low_weight) ||
      !IsFiniteAndNonnegative(config.force_high_weight) ||
      !IsFiniteAndNonnegative(config.shear_weight) ||
      !IsFiniteAndNonnegative(config.rotation_weight) ||
      !IsFiniteAndNonnegative(config.slip_score_weight) ||
      !IsFiniteAndNonnegative(config.contact_line_alignment_weight) ||
      !IsFiniteAndNonnegative(config.qddot_weight) ||
      !IsFiniteAndNonnegative(config.tau_weight) ||
      !config.close_axis_base.allFinite() ||
      config.close_axis_base.norm() <= 1.0e-12 ||
      config.min_normal_force_per_sensor_n >
          config.max_normal_force_per_sensor_n) {
    throw std::invalid_argument(
        "RobustGraspStateCostConfig: invalid numeric field");
  }
}

double ContactSupportCost(const GraspState& state,
                          const RobustGraspStateCostConfig& config) {
  double cost = 0.0;
  const std::size_t active_sensor_count = state.activeTactileSensorCount();
  if (active_sensor_count < config.min_active_tactile_sensors) {
    cost += config.contact_loss_weight *
            Square(static_cast<double>(
                config.min_active_tactile_sensors - active_sensor_count));
  }

  const std::size_t active_hemisphere_total =
      state.activeHemisphereCountTotal();
  if (active_hemisphere_total < config.min_active_hemisphere_total) {
    cost += config.support_weight *
            Square(static_cast<double>(
                config.min_active_hemisphere_total - active_hemisphere_total));
  }
  return cost;
}

double ForceCost(const GraspState& state,
                 const RobustGraspStateCostConfig& config) {
  double total_force_n = 0.0;
  double cost = 0.0;
  for (const auto& tactile : state.tactile_sensors) {
    if (!tactile.hasActiveHemisphereContact()) {
      continue;
    }
    const double force_n =
        std::max(0.0, tactile.activeHemisphereNormalForceN());
    total_force_n += force_n;
    if (force_n < config.min_normal_force_per_sensor_n) {
      cost += config.force_low_weight *
              Square(config.min_normal_force_per_sensor_n - force_n);
    }
    if (force_n > config.max_normal_force_per_sensor_n) {
      cost += config.force_high_weight *
              Square(force_n - config.max_normal_force_per_sensor_n);
    }
  }
  if (std::isfinite(total_force_n)) {
    cost += config.force_low_weight *
            Square(total_force_n - config.target_normal_force_n);
  }
  return cost;
}

double ShearCost(const GraspState& state,
                 const RobustGraspStateCostConfig& config) {
  double cost = 0.0;
  for (const auto& tactile : state.tactile_sensors) {
    if (tactile.shear_displacement_m.allFinite()) {
      cost += config.shear_weight *
              tactile.shear_displacement_m.squaredNorm();
    }
    if (std::isfinite(tactile.rotational_shear_rad)) {
      cost += config.rotation_weight *
              Square(tactile.rotational_shear_rad);
    }
    if (std::isfinite(tactile.slip_score)) {
      cost += config.slip_score_weight * Square(tactile.slip_score);
    }
  }
  return cost;
}

double ContactLineAlignmentCost(
    const GraspState& state, const RolloutContext& context,
    const RobustGraspStateCostConfig& config) {
  if (!config.enable_contact_line_alignment ||
      state.tactile_sensors.size() != context.tactile_contexts.size()) {
    return 0.0;
  }

  std::vector<Eigen::Vector3d> centroids;
  centroids.reserve(state.tactile_sensors.size());
  for (std::size_t i = 0; i < state.tactile_sensors.size(); ++i) {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    if (ComputeActiveTactileContactCentroidWorld(
            state, state.tactile_sensors[i], context.tactile_contexts[i],
            &centroid)) {
      centroids.push_back(centroid);
    }
  }
  if (centroids.size() != 2U) {
    return 0.0;
  }

  const Eigen::Vector3d axis = config.close_axis_base.normalized();
  const Eigen::Vector3d d = centroids[1] - centroids[0];
  const Eigen::Vector3d tangent_error = d - d.dot(axis) * axis;
  return config.contact_line_alignment_weight *
         tangent_error.squaredNorm();
}

}  // namespace

bool ComputeActiveTactileContactCentroidWorld(
    const GraspState& state, const TactileState& tactile,
    const TactileSensorContext& sensor_context,
    Eigen::Vector3d* centroid_world) {
  if (centroid_world == nullptr || !state.valid || !IsValid(state.robot) ||
      sensor_context.kinematics == nullptr ||
      !IsValidContactKinematicsContext(*sensor_context.kinematics)) {
    return false;
  }

  Eigen::Vector2d centroid_sensor_xy = Eigen::Vector2d::Zero();
  std::size_t active_count = 0;
  for (const auto& hemisphere : tactile.hemispheres) {
    if (hemisphere.contact && hemisphere.cop_sensor_m.allFinite()) {
      centroid_sensor_xy += hemisphere.cop_sensor_m;
      ++active_count;
    }
  }
  if (active_count == 0) {
    return false;
  }
  centroid_sensor_xy /= static_cast<double>(active_count);

  const auto& context = *sensor_context.kinematics;
  const auto& model = *context.model;
  if (state.robot.q.size() != static_cast<Eigen::Index>(model.nq)) {
    return false;
  }

  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, state.robot.q);
  pinocchio::updateFramePlacements(model, data);
  const Eigen::Vector3d centroid_sensor{
      centroid_sensor_xy.x(), centroid_sensor_xy.y(), 0.0};
  *centroid_world =
      data.oMf[context.sensor_frame_id].act(centroid_sensor);
  return centroid_world->allFinite();
}

RobustGraspStateCost::RobustGraspStateCost(
    RobustGraspStateCostConfig config)
    : config_(std::move(config)) {
  ValidateConfig(config_);
}

double RobustGraspStateCost::Evaluate(
    const GraspState& state,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
    const RolloutContext& context) const {
  if (!state.valid || !qddot_sol.allFinite()) {
    return 1.0e30;
  }

  double cost = ContactSupportCost(state, config_);
  cost += ForceCost(state, config_);
  cost += ShearCost(state, config_);
  cost += ContactLineAlignmentCost(state, context, config_);
  cost += config_.qddot_weight * qddot_sol.squaredNorm();
  if (state.robot.tau.size() > 0 && state.robot.tau.allFinite()) {
    cost += config_.tau_weight * state.robot.tau.squaredNorm();
  }
  return std::isfinite(cost) ? cost : 1.0e30;
}

}  // namespace mppi_core
