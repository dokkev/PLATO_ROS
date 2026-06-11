// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "mppi_core/disturbance/grasp_disturbance.hpp"

namespace mppi_core {

struct ObjectDisturbanceSamplerConfig {
  double linear_velocity_std_mps{0.0};
  double linear_velocity_max_mps{0.0};

  double angular_velocity_std_radps{0.0};
  double angular_velocity_max_radps{0.0};

  double pose_xyz_std_m{0.0};
  double pose_xyz_max_m{0.0};

  double pose_rpy_std_rad{0.0};
  double pose_rpy_max_rad{0.0};
};

struct GraspDisturbanceSamplerConfig {
  std::size_t num_disturbance_rollouts{32};
  std::size_t horizon_steps{20};
  std::size_t tactile_sensor_count{2};

  std::uint32_t random_seed{7};

  ObjectDisturbanceSamplerConfig object;

  double tangent_velocity_std_mps{0.004};
  double tangent_velocity_max_mps{0.020};

  double rotational_velocity_std_radps{0.05};
  double rotational_velocity_max_radps{0.30};

  double normal_force_rate_std_nps{1.0};
  double normal_force_rate_max_nps{5.0};

  double cop_drift_velocity_std_mps{0.001};
  double cop_drift_velocity_max_mps{0.005};

  double sensor_local_noise_scale{0.25};

  double friction_scale_mean{1.0};
  double friction_scale_std{0.15};
  double friction_scale_min{0.4};
  double friction_scale_max{1.5};

  double dropout_probability_per_step{0.0};
  bool use_antithetic_samples{true};
};

class GraspDisturbanceSampler {
 public:
  explicit GraspDisturbanceSampler(GraspDisturbanceSamplerConfig config);

  const GraspDisturbanceSamplerConfig& config() const { return config_; }

  std::vector<GraspDisturbanceSequence,
              Eigen::aligned_allocator<GraspDisturbanceSequence>>
  SampleBatch();

 private:
  GraspDisturbanceSamplerConfig config_;
  std::mt19937 rng_;
};

}  // namespace mppi_core
