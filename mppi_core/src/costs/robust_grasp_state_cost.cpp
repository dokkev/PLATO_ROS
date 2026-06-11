// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/costs/robust_grasp_state_cost.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "mppi_core/contact/contact_kinematics.hpp"

namespace mppi_core {
namespace {

double Square(const double value) { return value * value; }

double HingeExcess(const double value, const double deadband) {
  return std::max(0.0, value - deadband);
}

bool IsFiniteAndNonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool IsFinite(const double value) { return std::isfinite(value); }

bool IsValidObjectSupportCostConfig(
    const ObjectContactSupportEvaluatorConfig& config) {
  return config.max_object_samples > 0 &&
         config.min_active_tactile_sensors > 0 &&
         config.min_active_hemisphere_total > 0 &&
         IsFiniteAndNonnegative(config.contact_birth_margin_m) &&
         IsFiniteAndNonnegative(config.contact_loss_margin_m) &&
         config.contact_loss_margin_m >= config.contact_birth_margin_m &&
         IsFiniteAndNonnegative(config.hemisphere_radius_m) &&
         IsFiniteAndNonnegative(config.support_distance_scale_m) &&
         config.support_distance_scale_m > 0.0 &&
         IsFinite(config.good_contact_gap_min_m) &&
         IsFinite(config.good_contact_gap_max_m) &&
         config.good_contact_gap_max_m >= config.good_contact_gap_min_m &&
         IsFiniteAndNonnegative(config.deep_contact_scale_m) &&
         config.deep_contact_scale_m > 0.0 &&
         IsFiniteAndNonnegative(config.deep_contact_weight) &&
         IsFiniteAndNonnegative(config.max_allowed_penetration_m) &&
         IsFiniteAndNonnegative(config.contact_loss_weight) &&
         IsFiniteAndNonnegative(config.support_weight) &&
         IsFiniteAndNonnegative(config.edge_weight) &&
         IsFiniteAndNonnegative(config.penetration_weight) &&
         IsFiniteAndNonnegative(config.target_edge_margin_m);
}

void ValidateConfig(const RobustGraspStateCostConfig& config) {
  if (!IsFiniteAndNonnegative(config.contact_loss_weight) ||
      !IsFiniteAndNonnegative(config.support_weight) ||
      !IsFiniteAndNonnegative(config.target_normal_force_n) ||
      !IsFiniteAndNonnegative(config.min_normal_force_per_sensor_n) ||
      !IsFiniteAndNonnegative(config.max_normal_force_per_sensor_n) ||
      !IsFiniteAndNonnegative(config.force_low_weight) ||
      !IsFiniteAndNonnegative(config.force_high_weight) ||
      !IsFiniteAndNonnegative(config.force_balance_weight) ||
      !IsFiniteAndNonnegative(config.force_balance_deadband_n) ||
      !IsFiniteAndNonnegative(config.shear_weight) ||
      !IsFiniteAndNonnegative(config.rotation_weight) ||
      !IsFiniteAndNonnegative(config.slip_score_weight) ||
      !IsFiniteAndNonnegative(config.shear_safe_limit_m) ||
      !IsFiniteAndNonnegative(config.rotation_safe_limit_rad) ||
      !IsFiniteAndNonnegative(config.slip_score_safe_limit) ||
      !IsFiniteAndNonnegative(config.contact_line_alignment_weight) ||
      !IsFiniteAndNonnegative(config.contact_line_alignment_deadband_m) ||
      !IsFiniteAndNonnegative(config.qddot_weight) ||
      !IsFiniteAndNonnegative(config.tau_weight) ||
      !IsValidObjectSupportCostConfig(config.object_support) ||
      !config.close_axis_base.allFinite() ||
      config.close_axis_base.norm() <= 1.0e-12 ||
      config.min_normal_force_per_sensor_n >
          config.max_normal_force_per_sensor_n) {
    throw std::invalid_argument(
        "RobustGraspStateCostConfig: invalid numeric field");
  }
}

struct ForceCostTerms {
  double preload_cost{0.0};
  double force_low_cost{0.0};
  double force_high_cost{0.0};
  double balance_cost{0.0};

  double totalCost() const { return preload_cost + balance_cost; }
};

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

ForceCostTerms ForceCost(const GraspState& state,
                         const RobustGraspStateCostConfig& config) {
  ForceCostTerms terms;
  std::vector<double> active_forces_n;
  active_forces_n.reserve(state.tactile_sensors.size());
  for (const auto& tactile : state.tactile_sensors) {
    if (!tactile.hasActiveHemisphereContact()) {
      continue;
    }
    const double force_n =
        std::max(0.0, tactile.activeHemisphereNormalForceN());
    active_forces_n.push_back(force_n);
    if (force_n < config.min_normal_force_per_sensor_n) {
      const double cost =
          config.force_low_weight *
          Square(config.min_normal_force_per_sensor_n - force_n);
      terms.preload_cost += cost;
      terms.force_low_cost += cost;
    }
    if (force_n > config.max_normal_force_per_sensor_n) {
      const double cost =
          config.force_high_weight *
          Square(force_n - config.max_normal_force_per_sensor_n);
      terms.preload_cost += cost;
      terms.force_high_cost += cost;
    }
  }

  if (!active_forces_n.empty()) {
    const double weakest_force_n =
        *std::min_element(active_forces_n.begin(), active_forces_n.end());
    const double force_deficit_n =
        std::max(0.0, config.target_normal_force_n - weakest_force_n);
    const double cost = config.force_low_weight * Square(force_deficit_n);
    terms.preload_cost += cost;
    terms.force_low_cost += cost;
  }

  if (active_forces_n.size() >= 2U) {
    const auto [min_force_it, max_force_it] =
        std::minmax_element(active_forces_n.begin(), active_forces_n.end());
    const double imbalance_n = *max_force_it - *min_force_it;
    const double excess_n =
        HingeExcess(imbalance_n, config.force_balance_deadband_n);
    terms.balance_cost += config.force_balance_weight * Square(excess_n);
  }
  return terms;
}

double ShearCost(const GraspState& state,
                 const RobustGraspStateCostConfig& config) {
  double cost = 0.0;
  for (const auto& tactile : state.tactile_sensors) {
    if (tactile.shear_displacement_m.allFinite()) {
      const double excess_shear_m =
          HingeExcess(tactile.shear_displacement_m.norm(),
                      config.shear_safe_limit_m);
      cost += config.shear_weight *
              Square(excess_shear_m);
    }
    if (std::isfinite(tactile.rotational_shear_rad)) {
      const double excess_rotation_rad =
          HingeExcess(std::abs(tactile.rotational_shear_rad),
                      config.rotation_safe_limit_rad);
      cost += config.rotation_weight *
              Square(excess_rotation_rad);
    }
    if (std::isfinite(tactile.slip_score)) {
      const double excess_slip =
          HingeExcess(tactile.slip_score, config.slip_score_safe_limit);
      cost += config.slip_score_weight * Square(excess_slip);
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
  const double excess_error_m =
      HingeExcess(tangent_error.norm(),
                  config.contact_line_alignment_deadband_m);
  return config.contact_line_alignment_weight *
         Square(excess_error_m);
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
  return Evaluate(state, qddot_sol, context, nullptr);
}

double RobustGraspStateCost::Evaluate(
    const GraspState& state,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
    const RolloutContext& context,
    RobustGraspStateCostBreakdown* breakdown) const {
  if (!state.valid || !qddot_sol.allFinite()) {
    return 1.0e30;
  }

  RobustGraspStateCostBreakdown local;
  local.tactile_contact_support_cost = ContactSupportCost(state, config_);
  const ForceCostTerms force_terms = ForceCost(state, config_);
  local.preload_cost = force_terms.preload_cost;
  local.force_low_cost = force_terms.force_low_cost;
  local.force_high_cost = force_terms.force_high_cost;
  local.force_balance_cost = force_terms.balance_cost;
  local.force_cost = force_terms.totalCost();
  local.shear_cost = ShearCost(state, config_);
  local.contact_line_alignment_cost =
      ContactLineAlignmentCost(state, context, config_);
  local.action_cost = config_.qddot_weight * qddot_sol.squaredNorm();
  if (state.robot.tau.size() > 0 && state.robot.tau.allFinite()) {
    local.torque_cost = config_.tau_weight * state.robot.tau.squaredNorm();
  }
  local.object_support =
      EvaluateObjectContactSupport(state, context, config_.object_support);
  if (local.object_support.valid) {
    local.object_support_cost = local.object_support.totalCost();
  }

  const double cost = local.totalCost();
  if (breakdown != nullptr) {
    *breakdown = local;
  }
  return std::isfinite(cost) ? cost : 1.0e30;
}

}  // namespace mppi_core
