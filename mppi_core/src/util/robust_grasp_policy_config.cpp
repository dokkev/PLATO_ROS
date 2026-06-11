// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/config/robust_grasp_policy_config.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "mppi_core/config/mppi_config.hpp"
#include "mppi_core/util/yaml_utils.hpp"

namespace mppi_core {
namespace {

void CheckMap(const YAML::Node& params, const char* function_name) {
  if (yaml_utils::HasValue(params) && !params.IsMap()) {
    throw std::invalid_argument(std::string(function_name) +
                                ": params must be a map");
  }
}

YAML::Node RobustConfigNode(const YAML::Node& root) {
  return yaml_utils::RootSectionOrRoot(root, "robust_grasp",
                                       "Robust grasp YAML");
}

Eigen::VectorXd ReadVectorXdOrScalar(const YAML::Node& params,
                                     const char* key,
                                     const std::size_t expected_size,
                                     const Eigen::VectorXd& default_value) {
  if (!yaml_utils::HasValue(params)) {
    return default_value;
  }
  const YAML::Node value = params[key];
  if (!yaml_utils::HasValue(value)) {
    return default_value;
  }
  if (value.IsScalar()) {
    return Eigen::VectorXd::Constant(
        static_cast<Eigen::Index>(expected_size), value.as<double>());
  }
  if (!value.IsSequence() || value.size() != expected_size) {
    throw std::invalid_argument(std::string("Field '") + key +
                                "' dimension mismatch");
  }
  Eigen::VectorXd out(static_cast<Eigen::Index>(expected_size));
  for (std::size_t i = 0; i < expected_size; ++i) {
    out[static_cast<Eigen::Index>(i)] = value[i].as<double>();
  }
  return out;
}

Eigen::Vector3d ReadVector3OrScalar(const YAML::Node& params,
                                    const char* key,
                                    const Eigen::Vector3d& default_value) {
  const Eigen::VectorXd parsed = ReadVectorXdOrScalar(
      params, key, 3, Eigen::VectorXd());
  if (parsed.size() == 0) {
    return default_value;
  }
  return Eigen::Vector3d{parsed[0], parsed[1], parsed[2]};
}

std::size_t ReadActionDim(const YAML::Node& rollout,
                          const std::size_t requested_action_dim,
                          const RobustGraspPolicyConfig& defaults) {
  if (requested_action_dim > 0) {
    return requested_action_dim;
  }
  const std::size_t yaml_action_dim =
      yaml_utils::ReadSize(rollout, "action_dim",
                           defaults.rollout.action_dim);
  if (yaml_action_dim == 0) {
    throw std::invalid_argument(
        "ParseRobustGraspPolicyConfig: action_dim must be nonzero");
  }
  return yaml_action_dim;
}

}  // namespace

GraspDisturbanceSamplerConfig ParseGraspDisturbanceSamplerConfig(
    const YAML::Node& params, GraspDisturbanceSamplerConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseGraspDisturbanceSamplerConfig");

  defaults.num_disturbance_rollouts =
      yaml_utils::ReadSize(safe_params, "num_disturbance_rollouts",
                           defaults.num_disturbance_rollouts);
  defaults.horizon_steps =
      yaml_utils::ReadSize(safe_params, "horizon_steps",
                           defaults.horizon_steps);
  defaults.tactile_sensor_count =
      yaml_utils::ReadSize(safe_params, "tactile_sensor_count",
                           defaults.tactile_sensor_count);
  defaults.random_seed = static_cast<std::uint32_t>(
      yaml_utils::ReadSize(safe_params, "random_seed",
                           defaults.random_seed));
  defaults.tangent_velocity_std_mps =
      yaml_utils::ReadDouble(safe_params, "tangent_velocity_std_mps",
                             defaults.tangent_velocity_std_mps);
  defaults.tangent_velocity_max_mps =
      yaml_utils::ReadDouble(safe_params, "tangent_velocity_max_mps",
                             defaults.tangent_velocity_max_mps);
  defaults.rotational_velocity_std_radps =
      yaml_utils::ReadDouble(safe_params, "rotational_velocity_std_radps",
                             defaults.rotational_velocity_std_radps);
  defaults.rotational_velocity_max_radps =
      yaml_utils::ReadDouble(safe_params, "rotational_velocity_max_radps",
                             defaults.rotational_velocity_max_radps);
  defaults.normal_force_rate_std_nps =
      yaml_utils::ReadDouble(safe_params, "normal_force_rate_std_nps",
                             defaults.normal_force_rate_std_nps);
  defaults.normal_force_rate_max_nps =
      yaml_utils::ReadDouble(safe_params, "normal_force_rate_max_nps",
                             defaults.normal_force_rate_max_nps);
  defaults.cop_drift_velocity_std_mps =
      yaml_utils::ReadDouble(safe_params, "cop_drift_velocity_std_mps",
                             defaults.cop_drift_velocity_std_mps);
  defaults.cop_drift_velocity_max_mps =
      yaml_utils::ReadDouble(safe_params, "cop_drift_velocity_max_mps",
                             defaults.cop_drift_velocity_max_mps);
  defaults.sensor_local_noise_scale =
      yaml_utils::ReadDouble(safe_params, "sensor_local_noise_scale",
                             defaults.sensor_local_noise_scale);
  defaults.friction_scale_mean =
      yaml_utils::ReadDouble(safe_params, "friction_scale_mean",
                             defaults.friction_scale_mean);
  defaults.friction_scale_std =
      yaml_utils::ReadDouble(safe_params, "friction_scale_std",
                             defaults.friction_scale_std);
  defaults.friction_scale_min =
      yaml_utils::ReadDouble(safe_params, "friction_scale_min",
                             defaults.friction_scale_min);
  defaults.friction_scale_max =
      yaml_utils::ReadDouble(safe_params, "friction_scale_max",
                             defaults.friction_scale_max);
  defaults.dropout_probability_per_step =
      yaml_utils::ReadDouble(safe_params, "dropout_probability_per_step",
                             defaults.dropout_probability_per_step);
  defaults.use_antithetic_samples =
      yaml_utils::ReadBool(safe_params, "use_antithetic_samples",
                           defaults.use_antithetic_samples);
  return defaults;
}

DisturbedTactileTransitionConfig ParseDisturbedTactileTransitionConfig(
    const YAML::Node& params, DisturbedTactileTransitionConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseDisturbedTactileTransitionConfig");

  defaults.base.birth_approach_velocity_mps =
      yaml_utils::ReadDouble(safe_params, "birth_approach_velocity_mps",
                             defaults.base.birth_approach_velocity_mps);
  defaults.base.loss_unloading_velocity_mps =
      yaml_utils::ReadDouble(safe_params, "loss_unloading_velocity_mps",
                             defaults.base.loss_unloading_velocity_mps);
  defaults.base.max_shear_m =
      yaml_utils::ReadDouble(safe_params, "max_shear_m",
                             defaults.base.max_shear_m);
  defaults.base.max_rotation_rad =
      yaml_utils::ReadDouble(safe_params, "max_rotation_rad",
                             defaults.base.max_rotation_rad);
  defaults.base.born_normal_force_n =
      yaml_utils::ReadDouble(safe_params, "born_normal_force_n",
                             defaults.base.born_normal_force_n);
  defaults.base.born_confidence =
      yaml_utils::ReadDouble(safe_params, "born_confidence",
                             defaults.base.born_confidence);
  defaults.base.contact_confidence_decay =
      yaml_utils::ReadDouble(safe_params, "contact_confidence_decay",
                             defaults.base.contact_confidence_decay);
  defaults.normal_stiffness_n_per_m =
      yaml_utils::ReadDouble(safe_params, "normal_stiffness_n_per_m",
                             defaults.normal_stiffness_n_per_m);
  defaults.max_normal_force_n =
      yaml_utils::ReadDouble(safe_params, "max_normal_force_n",
                             defaults.max_normal_force_n);
  defaults.min_contact_force_n =
      yaml_utils::ReadDouble(safe_params, "min_contact_force_n",
                             defaults.min_contact_force_n);
  defaults.max_abs_shear_m =
      yaml_utils::ReadDouble(safe_params, "max_abs_shear_m",
                             defaults.max_abs_shear_m);
  defaults.max_abs_rotation_rad =
      yaml_utils::ReadDouble(safe_params, "max_abs_rotation_rad",
                             defaults.max_abs_rotation_rad);
  defaults.lose_contact_below_min_force =
      yaml_utils::ReadBool(safe_params, "lose_contact_below_min_force",
                           defaults.lose_contact_below_min_force);
  return defaults;
}

RobustGraspStateCostConfig ParseRobustGraspStateCostConfig(
    const YAML::Node& params, RobustGraspStateCostConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseRobustGraspStateCostConfig");

  defaults.min_active_tactile_sensors =
      yaml_utils::ReadSize(safe_params, "min_active_tactile_sensors",
                           defaults.min_active_tactile_sensors);
  defaults.min_active_hemisphere_total =
      yaml_utils::ReadSize(safe_params, "min_active_hemisphere_total",
                           defaults.min_active_hemisphere_total);
  defaults.contact_loss_weight =
      yaml_utils::ReadDouble(safe_params, "contact_loss_weight",
                             defaults.contact_loss_weight);
  defaults.support_weight =
      yaml_utils::ReadDouble(safe_params, "support_weight",
                             defaults.support_weight);
  defaults.target_normal_force_n =
      yaml_utils::ReadDouble(safe_params, "target_normal_force_n",
                             defaults.target_normal_force_n);
  defaults.min_normal_force_per_sensor_n =
      yaml_utils::ReadDouble(safe_params, "min_normal_force_per_sensor_n",
                             defaults.min_normal_force_per_sensor_n);
  defaults.max_normal_force_per_sensor_n =
      yaml_utils::ReadDouble(safe_params, "max_normal_force_per_sensor_n",
                             defaults.max_normal_force_per_sensor_n);
  defaults.force_low_weight =
      yaml_utils::ReadDouble(safe_params, "force_low_weight",
                             defaults.force_low_weight);
  defaults.force_high_weight =
      yaml_utils::ReadDouble(safe_params, "force_high_weight",
                             defaults.force_high_weight);
  defaults.force_balance_weight =
      yaml_utils::ReadDouble(safe_params, "force_balance_weight",
                             defaults.force_balance_weight);
  defaults.shear_weight =
      yaml_utils::ReadDouble(safe_params, "shear_weight",
                             defaults.shear_weight);
  defaults.rotation_weight =
      yaml_utils::ReadDouble(safe_params, "rotation_weight",
                             defaults.rotation_weight);
  defaults.slip_score_weight =
      yaml_utils::ReadDouble(safe_params, "slip_score_weight",
                             defaults.slip_score_weight);
  defaults.enable_contact_line_alignment =
      yaml_utils::ReadBool(safe_params, "enable_contact_line_alignment",
                           defaults.enable_contact_line_alignment);
  defaults.contact_line_alignment_weight =
      yaml_utils::ReadDouble(safe_params, "contact_line_alignment_weight",
                             defaults.contact_line_alignment_weight);
  defaults.close_axis_base =
      ReadVector3OrScalar(safe_params, "close_axis_base",
                          defaults.close_axis_base);
  defaults.qddot_weight =
      yaml_utils::ReadDouble(safe_params, "qddot_weight",
                             defaults.qddot_weight);
  defaults.tau_weight =
      yaml_utils::ReadDouble(safe_params, "tau_weight",
                             defaults.tau_weight);
  return defaults;
}

GraspActionLibraryConfig ParseGraspActionLibraryConfig(
    const YAML::Node& params, const std::size_t action_dim,
    GraspActionLibraryConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseGraspActionLibraryConfig");
  if (action_dim == 0) {
    throw std::invalid_argument(
        "ParseGraspActionLibraryConfig: action_dim must be nonzero");
  }

  defaults.action_dim = action_dim;
  defaults.horizon_steps =
      yaml_utils::ReadSize(safe_params, "horizon_steps",
                           defaults.horizon_steps);
  defaults.num_action_samples =
      yaml_utils::ReadSize(safe_params, "num_action_samples",
                           defaults.num_action_samples);
  defaults.random_seed = static_cast<std::uint32_t>(
      yaml_utils::ReadSize(safe_params, "random_seed",
                           defaults.random_seed));
  defaults.dt =
      yaml_utils::ReadDouble(safe_params, "dt", defaults.dt);
  defaults.target_min_normal_force_n =
      yaml_utils::ReadDouble(safe_params, "target_min_normal_force_n",
                             defaults.target_min_normal_force_n);
  defaults.force_to_squeeze_gain =
      yaml_utils::ReadDouble(safe_params, "force_to_squeeze_gain",
                             defaults.force_to_squeeze_gain);
  defaults.force_balance_gain =
      yaml_utils::ReadDouble(safe_params, "force_balance_gain",
                             defaults.force_balance_gain);
  defaults.contact_line_align_gain =
      yaml_utils::ReadDouble(safe_params, "contact_line_align_gain",
                             defaults.contact_line_align_gain);
  defaults.squeeze_std =
      yaml_utils::ReadDouble(safe_params, "squeeze_std",
                             defaults.squeeze_std);
  defaults.align_std =
      yaml_utils::ReadDouble(safe_params, "align_std",
                             defaults.align_std);
  defaults.force_balance_std =
      yaml_utils::ReadDouble(safe_params, "force_balance_std",
                             defaults.force_balance_std);
  defaults.thumb_bias_std =
      yaml_utils::ReadDouble(safe_params, "thumb_bias_std",
                             defaults.thumb_bias_std);
  defaults.index_bias_std =
      yaml_utils::ReadDouble(safe_params, "index_bias_std",
                             defaults.index_bias_std);
  defaults.max_squeeze =
      yaml_utils::ReadDouble(safe_params, "max_squeeze",
                             defaults.max_squeeze);
  defaults.max_align =
      yaml_utils::ReadDouble(safe_params, "max_align",
                             defaults.max_align);
  defaults.max_force_balance =
      yaml_utils::ReadDouble(safe_params, "max_force_balance",
                             defaults.max_force_balance);
  defaults.max_thumb_bias =
      yaml_utils::ReadDouble(safe_params, "max_thumb_bias",
                             defaults.max_thumb_bias);
  defaults.max_index_bias =
      yaml_utils::ReadDouble(safe_params, "max_index_bias",
                             defaults.max_index_bias);
  defaults.include_basis_probe_actions =
      yaml_utils::ReadBool(safe_params, "include_basis_probe_actions",
                           defaults.include_basis_probe_actions);
  defaults.squeeze_light_accel_scale =
      yaml_utils::ReadDouble(safe_params, "squeeze_light_accel_scale",
                             defaults.squeeze_light_accel_scale);
  defaults.squeeze_medium_accel_scale =
      yaml_utils::ReadDouble(safe_params, "squeeze_medium_accel_scale",
                             defaults.squeeze_medium_accel_scale);
  defaults.squeeze_strong_accel_scale =
      yaml_utils::ReadDouble(safe_params, "squeeze_strong_accel_scale",
                             defaults.squeeze_strong_accel_scale);
  defaults.release_accel_scale =
      yaml_utils::ReadDouble(safe_params, "release_accel_scale",
                             defaults.release_accel_scale);
  defaults.align_accel_scale =
      yaml_utils::ReadDouble(safe_params, "align_accel_scale",
                             defaults.align_accel_scale);
  defaults.sequence_decay =
      yaml_utils::ReadDouble(safe_params, "sequence_decay",
                             defaults.sequence_decay);
  defaults.close_axis_base =
      ReadVector3OrScalar(safe_params, "close_axis_base",
                          defaults.close_axis_base);
  defaults.qddot_lower_bound =
      ReadVectorXdOrScalar(safe_params, "qddot_lower_bound", action_dim,
                           defaults.qddot_lower_bound);
  defaults.qddot_upper_bound =
      ReadVectorXdOrScalar(safe_params, "qddot_upper_bound", action_dim,
                           defaults.qddot_upper_bound);
  return defaults;
}

RobustGraspPolicyConfig ParseRobustGraspPolicyConfig(
    const YAML::Node& params, const std::size_t action_dim,
    RobustGraspPolicyConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseRobustGraspPolicyConfig");

  const YAML::Node rollout = yaml_utils::ReadSection(safe_params, "rollout");
  const std::size_t resolved_action_dim =
      ReadActionDim(rollout, action_dim, defaults);
  defaults.rollout =
      ParseMPPIConfig(rollout, resolved_action_dim, defaults.rollout);

  const YAML::Node start = yaml_utils::ReadSection(safe_params, "start");
  defaults.start.min_enough_contact_sensors =
      yaml_utils::ReadSize(start, "min_enough_contact_sensors",
                           defaults.start.min_enough_contact_sensors);
  defaults.start.min_active_hemispheres_total =
      yaml_utils::ReadSize(start, "min_active_hemispheres_total",
                           defaults.start.min_active_hemispheres_total);
  defaults.start.min_active_hemispheres_total =
      yaml_utils::ReadSize(start, "min_active_hemisphere_total",
                           defaults.start.min_active_hemispheres_total);

  defaults.disturbance_sampler = ParseGraspDisturbanceSamplerConfig(
      yaml_utils::ReadSection(safe_params, "disturbance"),
      defaults.disturbance_sampler);
  defaults.disturbance_sampler.horizon_steps = defaults.rollout.horizon_steps;

  defaults.disturbed_rollout.tactile_transition =
      ParseDisturbedTactileTransitionConfig(
          yaml_utils::ReadSection(safe_params, "tactile_transition"),
          defaults.disturbed_rollout.tactile_transition);
  defaults.cost = ParseRobustGraspStateCostConfig(
      yaml_utils::ReadSection(safe_params, "cost"), defaults.cost);
  defaults.action_library = ParseGraspActionLibraryConfig(
      yaml_utils::ReadSection(safe_params, "action_library"),
      resolved_action_dim, defaults.action_library);
  defaults.action_library.horizon_steps = defaults.rollout.horizon_steps;
  defaults.action_library.dt = defaults.rollout.dt;
  if (defaults.action_library.qddot_lower_bound.size() == 0) {
    defaults.action_library.qddot_lower_bound =
        defaults.rollout.action_lower_bound;
  }
  if (defaults.action_library.qddot_upper_bound.size() == 0) {
    defaults.action_library.qddot_upper_bound =
        defaults.rollout.action_upper_bound;
  }

  const YAML::Node robust_score =
      yaml_utils::ReadSection(safe_params, "robust_score");
  defaults.risk_weight =
      yaml_utils::ReadDouble(robust_score, "risk_weight",
                             defaults.risk_weight);
  defaults.cvar_tail_fraction =
      yaml_utils::ReadDouble(robust_score, "cvar_tail_fraction",
                             defaults.cvar_tail_fraction);

  const YAML::Node behavior =
      yaml_utils::ReadSection(safe_params, "behavior");
  defaults.require_both_contact_for_update =
      yaml_utils::ReadBool(behavior, "require_both_contact_for_update",
                           defaults.require_both_contact_for_update);
  defaults.return_hold_when_not_ready =
      yaml_utils::ReadBool(behavior, "return_hold_when_not_ready",
                           defaults.return_hold_when_not_ready);
  return defaults;
}

RobustGraspPolicyConfig LoadRobustGraspPolicyConfigFromYamlFile(
    const std::string& yaml_path, const std::size_t action_dim,
    RobustGraspPolicyConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseRobustGraspPolicyConfig(
        RobustConfigNode(root), action_dim, std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error(
        "LoadRobustGraspPolicyConfigFromYamlFile: failed to load '" +
        yaml_path + "': " + ex.what());
  }
}

}  // namespace mppi_core
