// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/policy/grasp_action_library.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "mppi_core/contact/contact_kinematics.hpp"

namespace mppi_core {
namespace {

bool IsFiniteAndNonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

Eigen::VectorXd DefaultBound(const std::size_t dim, const double value) {
  return Eigen::VectorXd::Constant(static_cast<Eigen::Index>(dim), value);
}

void ValidateVectorDim(const Eigen::VectorXd& value, const std::size_t dim,
                       const char* name) {
  if (value.size() != static_cast<Eigen::Index>(dim)) {
    throw std::invalid_argument(std::string("GraspActionLibraryConfig: ") +
                                name + " dimension mismatch");
  }
}

void ValidateConfig(GraspActionLibraryConfig* config) {
  if (config == nullptr || config->horizon_steps == 0 ||
      config->action_dim == 0 || !std::isfinite(config->dt) ||
      config->dt <= 0.0 ||
      !IsFiniteAndNonnegative(config->squeeze_light_accel_scale) ||
      !IsFiniteAndNonnegative(config->squeeze_medium_accel_scale) ||
      !IsFiniteAndNonnegative(config->squeeze_strong_accel_scale) ||
      !IsFiniteAndNonnegative(config->release_accel_scale) ||
      !IsFiniteAndNonnegative(config->align_accel_scale) ||
      !std::isfinite(config->sequence_decay)) {
    throw std::invalid_argument(
        "GraspActionLibraryConfig: invalid numeric field");
  }
  if (config->qddot_lower_bound.size() == 0) {
    config->qddot_lower_bound = DefaultBound(
        config->action_dim, -std::numeric_limits<double>::infinity());
  }
  if (config->qddot_upper_bound.size() == 0) {
    config->qddot_upper_bound = DefaultBound(
        config->action_dim, std::numeric_limits<double>::infinity());
  }
  ValidateVectorDim(config->qddot_lower_bound, config->action_dim,
                    "qddot_lower_bound");
  ValidateVectorDim(config->qddot_upper_bound, config->action_dim,
                    "qddot_upper_bound");
  for (Eigen::Index i = 0; i < config->qddot_lower_bound.size(); ++i) {
    if (std::isnan(config->qddot_lower_bound[i]) ||
        std::isnan(config->qddot_upper_bound[i]) ||
        config->qddot_lower_bound[i] > config->qddot_upper_bound[i]) {
      throw std::invalid_argument(
          "GraspActionLibraryConfig: invalid qddot bounds");
    }
  }
}

}  // namespace

GraspActionLibrary::GraspActionLibrary(GraspActionLibraryConfig config)
    : config_(std::move(config)) {
  ValidateConfig(&config_);
}

std::vector<ActionSequence> GraspActionLibrary::BuildCandidates(
    const GraspState& state, const RolloutContext& context) const {
  std::vector<ActionSequence> candidates;
  const Eigen::VectorXd zero =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));
  candidates.push_back(BuildDecayedSequence(zero));

  Eigen::VectorXd squeeze_dir;
  if (!BuildSqueezeDirection(state, context, &squeeze_dir)) {
    return candidates;
  }

  const auto add_scaled = [&](const double scale) {
      candidates.push_back(BuildDecayedSequence(squeeze_dir * scale));
    };
  add_scaled(config_.squeeze_light_accel_scale);
  add_scaled(config_.squeeze_medium_accel_scale);
  add_scaled(config_.squeeze_strong_accel_scale);
  add_scaled(-config_.release_accel_scale);
  return candidates;
}

ActionSequence GraspActionLibrary::BuildDecayedSequence(
    const Eigen::Ref<const Eigen::VectorXd>& first_action) const {
  if (first_action.size() != static_cast<Eigen::Index>(config_.action_dim)) {
    throw std::invalid_argument(
        "GraspActionLibrary::BuildDecayedSequence: action dimension mismatch");
  }

  ActionSequence sequence(config_.action_dim, config_.horizon_steps);
  for (std::size_t step = 0; step < config_.horizon_steps; ++step) {
    const double decay =
        std::pow(config_.sequence_decay, static_cast<double>(step));
    sequence.setAction(step, ClampAction(first_action * decay));
  }
  return sequence;
}

Eigen::VectorXd GraspActionLibrary::ClampAction(
    const Eigen::VectorXd& action) const {
  Eigen::VectorXd clamped = action;
  for (Eigen::Index i = 0; i < clamped.size(); ++i) {
    clamped[i] = std::clamp(
        clamped[i], config_.qddot_lower_bound[i],
        config_.qddot_upper_bound[i]);
  }
  return clamped;
}

bool GraspActionLibrary::BuildSqueezeDirection(
    const GraspState& state, const RolloutContext& context,
    Eigen::VectorXd* direction) const {
  if (direction == nullptr || !state.valid ||
      state.tactile_sensors.size() != context.tactile_contexts.size()) {
    return false;
  }

  Eigen::VectorXd normal_jacobian_sum =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));
  std::size_t row_count = 0;
  const Eigen::VectorXd zero_tangent =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));

  for (std::size_t i = 0; i < state.tactile_sensors.size(); ++i) {
    const auto& tactile = state.tactile_sensors[i];
    const auto& tactile_context = context.tactile_contexts[i];
    if (tactile_context.kinematics == nullptr ||
        !IsValidContactKinematicsContext(*tactile_context.kinematics)) {
      continue;
    }
    const auto motions = ComputeHemisphereMotions(
        state.robot, tactile, zero_tangent, *tactile_context.kinematics);
    for (const auto& motion : motions) {
      if (motion.J_normal.size() ==
          static_cast<Eigen::Index>(config_.action_dim)) {
        normal_jacobian_sum += motion.J_normal.transpose();
        ++row_count;
      }
    }
  }

  if (row_count == 0) {
    return false;
  }
  *direction = normal_jacobian_sum / static_cast<double>(row_count);
  const double norm = direction->norm();
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    return false;
  }
  *direction /= norm;
  return direction->allFinite();
}

}  // namespace mppi_core
