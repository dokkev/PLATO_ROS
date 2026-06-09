// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/costs/grasp_stability_cost.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace mppi_core {
namespace {

double Square(double value) { return value * value; }

bool IsFiniteAndNonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

GraspStabilityCost::GraspStabilityCost(GraspStabilityCostConfig config)
    : config_(std::move(config)) {
  if (!IsFiniteAndNonnegative(config_.contact_loss_weight) ||
      !IsFiniteAndNonnegative(config_.support_weight) ||
      !IsFiniteAndNonnegative(config_.shear_weight) ||
      !IsFiniteAndNonnegative(config_.rotation_weight) ||
      !IsFiniteAndNonnegative(config_.qddot_weight) ||
      !IsFiniteAndNonnegative(config_.tau_weight)) {
    throw std::invalid_argument(
        "GraspStabilityCost: weights must be finite and nonnegative");
  }
}

double GraspStabilityCost::Evaluate(
    const GraspState& state, const Eigen::Ref<const Eigen::VectorXd>& action,
    const CostContext& /*context*/) const {
  return RobotEffortCost(state, action) + TactileSensorsCost(state);
}

double GraspStabilityCost::RobotEffortCost(
    const GraspState& state,
    const Eigen::Ref<const Eigen::VectorXd>& action) const {
  double cost = config_.qddot_weight * action.squaredNorm();
  if (state.robot.tau.size() > 0 && state.robot.tau.allFinite()) {
    cost += config_.tau_weight * state.robot.tau.squaredNorm();
  }
  return cost;
}

double GraspStabilityCost::TactileSensorsCost(
    const GraspState& state) const {
  double cost = 0.0;

  const std::size_t active_sensor_count = state.activeTactileSensorCount();
  if (active_sensor_count < config_.min_active_tactile_sensors) {
    const double error =
        static_cast<double>(config_.min_active_tactile_sensors -
                            active_sensor_count);
    cost += config_.contact_loss_weight * Square(error);
  }

  const std::size_t active_hemisphere_total =
      state.activeHemisphereCountTotal();
  if (active_hemisphere_total < config_.target_active_hemisphere_total) {
    const double error =
        static_cast<double>(config_.target_active_hemisphere_total -
                            active_hemisphere_total);
    cost += config_.support_weight * Square(error);
  }

  for (const auto& tactile : state.tactile_sensors) {
    if (tactile.shear_displacement_m.allFinite()) {
      cost +=
          config_.shear_weight * tactile.shear_displacement_m.squaredNorm();
    }
    if (std::isfinite(tactile.rotational_shear_rad)) {
      cost +=
          config_.rotation_weight * Square(tactile.rotational_shear_rad);
    }
  }

  return cost;
}

}  // namespace mppi_core
