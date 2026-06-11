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
      !IsNonnegativeFinite(config.object.linear_velocity_std_mps) ||
      !IsNonnegativeFinite(config.object.linear_velocity_max_mps) ||
      !IsNonnegativeFinite(config.object.angular_velocity_std_radps) ||
      !IsNonnegativeFinite(config.object.angular_velocity_max_radps) ||
      !IsNonnegativeFinite(config.object.pose_xyz_std_m) ||
      !IsNonnegativeFinite(config.object.pose_xyz_max_m) ||
      !IsNonnegativeFinite(config.object.pose_rpy_std_rad) ||
      !IsNonnegativeFinite(config.object.pose_rpy_max_rad) ||
      !IsNonnegativeFinite(config.tangent_velocity_max_mps) ||
      !IsNonnegativeFinite(config.rotational_velocity_std_radps) ||
      !IsNonnegativeFinite(config.rotational_velocity_max_radps) ||
      !IsNonnegativeFinite(config.normal_force_rate_std_nps) ||
      !IsNonnegativeFinite(config.normal_force_rate_max_nps) ||
      !IsNonnegativeFinite(config.cop_drift_velocity_std_mps) ||
      !IsNonnegativeFinite(config.cop_drift_velocity_max_mps) ||
      !IsNonnegativeFinite(config.sensor_local_noise_scale) ||
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

CommonGraspDisturbance Antithetic(
    const CommonGraspDisturbance& disturbance) {
  CommonGraspDisturbance out = disturbance;
  out.tangent_velocity_grasp_mps = -out.tangent_velocity_grasp_mps;
  out.rotational_velocity_radps = -out.rotational_velocity_radps;
  out.normal_force_rate_nps = -out.normal_force_rate_nps;
  out.cop_drift_velocity_grasp_mps = -out.cop_drift_velocity_grasp_mps;
  return out;
}

VirtualObjectDisturbance Antithetic(
    const VirtualObjectDisturbance& disturbance) {
  VirtualObjectDisturbance out = disturbance;
  out.linear_velocity_world_mps = -out.linear_velocity_world_mps;
  out.angular_velocity_world_radps = -out.angular_velocity_world_radps;
  out.position_offset_world_m = -out.position_offset_world_m;
  out.rpy_offset_world_rad = -out.rpy_offset_world_rad;
  out.external_force_world_n = -out.external_force_world_n;
  out.external_torque_world_nm = -out.external_torque_world_nm;
  return out;
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

CommonGraspDisturbance SampleCommonDisturbance(
    std::mt19937* rng, const GraspDisturbanceSamplerConfig& config,
    std::bernoulli_distribution* dropout_dist) {
  CommonGraspDisturbance disturbance;
  disturbance.tangent_velocity_grasp_mps =
      Eigen::Vector2d{
          SampleClampedNormal(
              rng, config.tangent_velocity_std_mps,
              config.tangent_velocity_max_mps),
          SampleClampedNormal(
              rng, config.tangent_velocity_std_mps,
              config.tangent_velocity_max_mps)};
  disturbance.rotational_velocity_radps =
      SampleClampedNormal(
          rng, config.rotational_velocity_std_radps,
          config.rotational_velocity_max_radps);
  disturbance.normal_force_rate_nps =
      SampleClampedNormal(
          rng, config.normal_force_rate_std_nps,
          config.normal_force_rate_max_nps);
  disturbance.cop_drift_velocity_grasp_mps =
      Eigen::Vector2d{
          SampleClampedNormal(
              rng, config.cop_drift_velocity_std_mps,
              config.cop_drift_velocity_max_mps),
          SampleClampedNormal(
              rng, config.cop_drift_velocity_std_mps,
              config.cop_drift_velocity_max_mps)};
  disturbance.dropout = dropout_dist != nullptr && (*dropout_dist)(*rng);
  disturbance.valid = true;
  return disturbance;
}

VirtualObjectDisturbance SampleObjectDisturbance(
    std::mt19937* rng, const ObjectDisturbanceSamplerConfig& config,
    std::bernoulli_distribution* dropout_dist) {
  VirtualObjectDisturbance disturbance;
  disturbance.linear_velocity_world_mps =
      Eigen::Vector3d{
          SampleClampedNormal(
              rng, config.linear_velocity_std_mps,
              config.linear_velocity_max_mps),
          SampleClampedNormal(
              rng, config.linear_velocity_std_mps,
              config.linear_velocity_max_mps),
          SampleClampedNormal(
              rng, config.linear_velocity_std_mps,
              config.linear_velocity_max_mps)};
  disturbance.angular_velocity_world_radps =
      Eigen::Vector3d{
          SampleClampedNormal(
              rng, config.angular_velocity_std_radps,
              config.angular_velocity_max_radps),
          SampleClampedNormal(
              rng, config.angular_velocity_std_radps,
              config.angular_velocity_max_radps),
          SampleClampedNormal(
              rng, config.angular_velocity_std_radps,
              config.angular_velocity_max_radps)};
  disturbance.position_offset_world_m =
      Eigen::Vector3d{
          SampleClampedNormal(rng, config.pose_xyz_std_m,
                              config.pose_xyz_max_m),
          SampleClampedNormal(rng, config.pose_xyz_std_m,
                              config.pose_xyz_max_m),
          SampleClampedNormal(rng, config.pose_xyz_std_m,
                              config.pose_xyz_max_m)};
  disturbance.rpy_offset_world_rad =
      Eigen::Vector3d{
          SampleClampedNormal(rng, config.pose_rpy_std_rad,
                              config.pose_rpy_max_rad),
          SampleClampedNormal(rng, config.pose_rpy_std_rad,
                              config.pose_rpy_max_rad),
          SampleClampedNormal(rng, config.pose_rpy_std_rad,
                              config.pose_rpy_max_rad)};
  disturbance.dropout = dropout_dist != nullptr && (*dropout_dist)(*rng);
  disturbance.valid = true;
  return disturbance;
}

TactileSensorDisturbance SampleSensorDisturbance(
    std::mt19937* rng, const GraspDisturbanceSamplerConfig& config,
    const CommonGraspDisturbance& common,
    std::bernoulli_distribution* dropout_dist) {
  const double local_scale = config.sensor_local_noise_scale;
  TactileSensorDisturbance disturbance;
  disturbance.tangent_velocity_sensor_mps =
      common.tangent_velocity_grasp_mps +
      Eigen::Vector2d{
          SampleClampedNormal(
              rng, config.tangent_velocity_std_mps * local_scale,
              config.tangent_velocity_max_mps * local_scale),
          SampleClampedNormal(
              rng, config.tangent_velocity_std_mps * local_scale,
              config.tangent_velocity_max_mps * local_scale)};
  disturbance.rotational_velocity_radps =
      ClampSymmetric(
          common.rotational_velocity_radps +
              SampleClampedNormal(
                  rng, config.rotational_velocity_std_radps * local_scale,
                  config.rotational_velocity_max_radps * local_scale),
          config.rotational_velocity_max_radps);
  disturbance.normal_force_rate_nps =
      ClampSymmetric(
          common.normal_force_rate_nps +
              SampleClampedNormal(
                  rng, config.normal_force_rate_std_nps * local_scale,
                  config.normal_force_rate_max_nps * local_scale),
          config.normal_force_rate_max_nps);
  disturbance.cop_drift_velocity_sensor_mps =
      common.cop_drift_velocity_grasp_mps +
      Eigen::Vector2d{
          SampleClampedNormal(
              rng, config.cop_drift_velocity_std_mps * local_scale,
              config.cop_drift_velocity_max_mps * local_scale),
          SampleClampedNormal(
              rng, config.cop_drift_velocity_std_mps * local_scale,
              config.cop_drift_velocity_max_mps * local_scale)};

  std::normal_distribution<double> friction_dist(
      config.friction_scale_mean, config.friction_scale_std);
  disturbance.friction_scale = std::clamp(
      friction_dist(*rng), config.friction_scale_min,
      config.friction_scale_max);
  disturbance.dropout =
      common.dropout || (dropout_dist != nullptr && (*dropout_dist)(*rng));
  disturbance.valid = true;
  return disturbance;
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
      if (pair_source != nullptr && step < pair_source->steps.size()) {
        disturbance_step.object_disturbance =
            Antithetic(pair_source->steps[step].object_disturbance);
        disturbance_step.common_grasp_disturbance = Antithetic(
            pair_source->steps[step].common_grasp_disturbance);
      } else {
        disturbance_step.object_disturbance =
            SampleObjectDisturbance(&rng_, config_.object, &dropout_dist);
        disturbance_step.common_grasp_disturbance =
            SampleCommonDisturbance(&rng_, config_, &dropout_dist);
      }

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

        disturbance_step.tactile_sensor_disturbances[sensor] =
            SampleSensorDisturbance(
                &rng_, config_, disturbance_step.common_grasp_disturbance,
                &dropout_dist);
      }
    }
  }
  return batch;
}

}  // namespace mppi_core
