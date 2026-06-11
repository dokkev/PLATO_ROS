// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/disturbance/grasp_disturbance_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mppi_core {
namespace {

bool IsNonnegativeFinite(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

double ClampSymmetric(const double value, const double limit) {
  return std::clamp(value, -limit, limit);
}

double SampleClampedNormal(std::mt19937* rng, const double stddev,
                           const double max_abs) {
  if (rng == nullptr || stddev <= 0.0 || max_abs <= 0.0) {
    return 0.0;
  }
  std::normal_distribution<double> normal(0.0, stddev);
  return ClampSymmetric(normal(*rng), max_abs);
}

void ValidateConfig(const GraspDisturbanceSamplerConfig& config) {
  if (config.num_disturbance_rollouts == 0) {
    throw std::invalid_argument(
        "GraspDisturbanceSamplerConfig: num_disturbance_rollouts must be positive");
  }
  if (config.horizon_steps == 0) {
    throw std::invalid_argument(
        "GraspDisturbanceSamplerConfig: horizon_steps must be positive");
  }
  if (config.tactile_sensor_count == 0) {
    throw std::invalid_argument(
        "GraspDisturbanceSamplerConfig: tactile_sensor_count must be positive");
  }
  if (!IsNonnegativeFinite(config.tangent_velocity_std_mps) ||
      !IsNonnegativeFinite(config.tangent_velocity_max_mps) ||
      !IsNonnegativeFinite(config.rotational_velocity_std_radps) ||
      !IsNonnegativeFinite(config.rotational_velocity_max_radps) ||
      !IsNonnegativeFinite(config.normal_force_rate_std_nps) ||
      !IsNonnegativeFinite(config.normal_force_rate_max_nps) ||
      !IsNonnegativeFinite(config.cop_drift_velocity_std_mps) ||
      !IsNonnegativeFinite(config.cop_drift_velocity_max_mps) ||
      !IsNonnegativeFinite(config.friction_scale_std) ||
      !IsNonnegativeFinite(config.dropout_probability_per_step) ||
      !std::isfinite(config.friction_scale_mean) ||
      !std::isfinite(config.friction_scale_min) ||
      !std::isfinite(config.friction_scale_max) ||
      config.friction_scale_min <= 0.0 ||
      config.friction_scale_min > config.friction_scale_max ||
      config.dropout_probability_per_step > 1.0) {
    throw std::invalid_argument(
        "GraspDisturbanceSamplerConfig: invalid numeric field");
  }
}

TactileSensorDisturbance Antithetic(
    const TactileSensorDisturbance& disturbance,
    const GraspDisturbanceSamplerConfig& config) {
  TactileSensorDisturbance out = disturbance;
  out.tangent_velocity_sensor_mps = -out.tangent_velocity_sensor_mps;
  out.rotational_velocity_radps = -out.rotational_velocity_radps;
  out.normal_force_rate_nps = -out.normal_force_rate_nps;
  out.cop_drift_velocity_sensor_mps = -out.cop_drift_velocity_sensor_mps;
  out.friction_scale = std::clamp(
      2.0 * config.friction_scale_mean - out.friction_scale,
      config.friction_scale_min, config.friction_scale_max);
  return out;
}

}  // namespace

GraspDisturbanceSampler::GraspDisturbanceSampler(
    GraspDisturbanceSamplerConfig config)
    : config_(config), rng_(config.random_seed) {
  ValidateConfig(config_);
}

std::vector<GraspDisturbanceSequence,
            Eigen::aligned_allocator<GraspDisturbanceSequence>>
GraspDisturbanceSampler::SampleBatch() {
  std::vector<GraspDisturbanceSequence,
              Eigen::aligned_allocator<GraspDisturbanceSequence>>
      batch;
  batch.resize(config_.num_disturbance_rollouts);

  std::bernoulli_distribution dropout_dist(
      config_.dropout_probability_per_step);

  for (std::size_t rollout = 0; rollout < batch.size(); ++rollout) {
    auto& sequence = batch[rollout];
    sequence.steps.resize(config_.horizon_steps);
    const bool use_pair =
        config_.use_antithetic_samples && rollout > 0 && (rollout % 2U == 1U);
    const auto* pair_source = use_pair ? &batch[rollout - 1] : nullptr;

    for (std::size_t step = 0; step < config_.horizon_steps; ++step) {
      auto& disturbance_step = sequence.steps[step];
      disturbance_step.tactile_sensor_disturbances.resize(
          config_.tactile_sensor_count);
      disturbance_step.valid = true;

      for (std::size_t sensor = 0; sensor < config_.tactile_sensor_count;
           ++sensor) {
        if (pair_source != nullptr &&
            step < pair_source->steps.size() &&
            sensor <
                pair_source->steps[step].tactile_sensor_disturbances.size()) {
          disturbance_step.tactile_sensor_disturbances[sensor] = Antithetic(
              pair_source->steps[step].tactile_sensor_disturbances[sensor],
              config_);
          continue;
        }

        auto& tactile_disturbance =
            disturbance_step.tactile_sensor_disturbances[sensor];
        tactile_disturbance.tangent_velocity_sensor_mps =
            Eigen::Vector2d{
                SampleClampedNormal(
                    &rng_, config_.tangent_velocity_std_mps,
                    config_.tangent_velocity_max_mps),
                SampleClampedNormal(
                    &rng_, config_.tangent_velocity_std_mps,
                    config_.tangent_velocity_max_mps)};
        tactile_disturbance.rotational_velocity_radps =
            SampleClampedNormal(
                &rng_, config_.rotational_velocity_std_radps,
                config_.rotational_velocity_max_radps);
        tactile_disturbance.normal_force_rate_nps =
            SampleClampedNormal(
                &rng_, config_.normal_force_rate_std_nps,
                config_.normal_force_rate_max_nps);
        tactile_disturbance.cop_drift_velocity_sensor_mps =
            Eigen::Vector2d{
                SampleClampedNormal(
                    &rng_, config_.cop_drift_velocity_std_mps,
                    config_.cop_drift_velocity_max_mps),
                SampleClampedNormal(
                    &rng_, config_.cop_drift_velocity_std_mps,
                    config_.cop_drift_velocity_max_mps)};

        std::normal_distribution<double> friction_dist(
            config_.friction_scale_mean, config_.friction_scale_std);
        tactile_disturbance.friction_scale = std::clamp(
            friction_dist(rng_), config_.friction_scale_min,
            config_.friction_scale_max);
        tactile_disturbance.dropout = dropout_dist(rng_);
        tactile_disturbance.valid = true;
      }
    }
  }
  return batch;
}

}  // namespace mppi_core
