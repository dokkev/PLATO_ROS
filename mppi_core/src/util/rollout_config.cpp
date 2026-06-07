// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/config/rollout_config.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "mppi_core/util/yaml_utils.hpp"

namespace mppi_core {
namespace {

YAML::Node RolloutConfigNode(const YAML::Node& root) {
  return yaml_utils::RootSectionOrRoot(root, "rollout", "Rollout YAML");
}

void CheckMap(const YAML::Node& params, const char* function_name) {
  if (yaml_utils::HasValue(params) && !params.IsMap()) {
    throw std::invalid_argument(std::string(function_name) +
                                ": params must be a map");
  }
}

}  // namespace

GraspStateRolloutConfig ParseGraspStateRolloutConfig(
    const YAML::Node& params, GraspStateRolloutConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseGraspStateRolloutConfig");

  const YAML::Node disturbance =
      yaml_utils::ReadSection(safe_params, "disturbance");
  defaults.disturbance_enabled =
      yaml_utils::ReadBool(disturbance, "enabled",
                           defaults.disturbance_enabled);
  return defaults;
}

GraspStateRolloutConfig LoadGraspStateRolloutConfigFromYamlFile(
    const std::string& yaml_path, GraspStateRolloutConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseGraspStateRolloutConfig(RolloutConfigNode(root),
                                        std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error(
        "LoadGraspStateRolloutConfigFromYamlFile: failed to load '" +
        yaml_path + "': " + ex.what());
  }
}

TactileTransitionConfig ParseTactileTransitionConfig(
    const YAML::Node& params, TactileTransitionConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseTactileTransitionConfig");

  const YAML::Node tactile_model =
      yaml_utils::ReadSection(safe_params, "tactile_model");
  defaults.birth_approach_velocity_mps =
      yaml_utils::ReadDouble(tactile_model, "birth_approach_velocity_mps",
                             defaults.birth_approach_velocity_mps);
  defaults.loss_unloading_velocity_mps =
      yaml_utils::ReadDouble(tactile_model, "loss_unloading_velocity_mps",
                             defaults.loss_unloading_velocity_mps);
  defaults.born_normal_force_n =
      yaml_utils::ReadDouble(tactile_model, "born_normal_force_n",
                             defaults.born_normal_force_n);
  defaults.born_confidence =
      yaml_utils::ReadDouble(tactile_model, "born_confidence",
                             defaults.born_confidence);
  defaults.contact_confidence_decay =
      yaml_utils::ReadDouble(tactile_model, "contact_confidence_decay",
                             defaults.contact_confidence_decay);
  return defaults;
}

TactileTransitionConfig LoadTactileTransitionConfigFromYamlFile(
    const std::string& yaml_path, TactileTransitionConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseTactileTransitionConfig(RolloutConfigNode(root),
                                        std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error(
        "LoadTactileTransitionConfigFromYamlFile: failed to load '" +
        yaml_path + "': " + ex.what());
  }
}

ContactForceRolloutConfig ParseContactForceRolloutConfig(
    const YAML::Node& params, ContactForceRolloutConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseContactForceRolloutConfig");

  const YAML::Node rollout =
      yaml_utils::ReadSection(safe_params, "contact_force_rollout");
  defaults.enable_force_projection_update = yaml_utils::ReadBool(
      rollout, "enable_force_projection_update",
      defaults.enable_force_projection_update);
  defaults.force_lowpass_alpha =
      yaml_utils::ReadDouble(rollout, "force_lowpass_alpha",
                             defaults.force_lowpass_alpha);
  defaults.max_predicted_normal_force_n =
      yaml_utils::ReadDouble(rollout, "max_predicted_normal_force_n",
                             defaults.max_predicted_normal_force_n);
  defaults.shear_force_gain_m_per_n_s =
      yaml_utils::ReadDouble(rollout, "shear_force_gain_m_per_n_s",
                             defaults.shear_force_gain_m_per_n_s);
  defaults.rotational_shear_gain_rad_per_nm_s =
      yaml_utils::ReadDouble(rollout, "rotational_shear_gain_rad_per_nm_s",
                             defaults.rotational_shear_gain_rad_per_nm_s);
  defaults.friction_violation_confidence_decay = yaml_utils::ReadDouble(
      rollout, "friction_violation_confidence_decay",
      defaults.friction_violation_confidence_decay);
  defaults.negative_normal_confidence_decay = yaml_utils::ReadDouble(
      rollout, "negative_normal_confidence_decay",
      defaults.negative_normal_confidence_decay);
  defaults.min_active_hemisphere_count =
      yaml_utils::ReadSize(rollout, "min_active_hemisphere_count",
                           defaults.min_active_hemisphere_count);
  defaults.min_contact_confidence =
      yaml_utils::ReadDouble(rollout, "min_contact_confidence",
                             defaults.min_contact_confidence);
  defaults.shear_ref_m =
      yaml_utils::ReadDouble(rollout, "shear_ref_m", defaults.shear_ref_m);
  defaults.rotational_shear_ref_rad =
      yaml_utils::ReadDouble(rollout, "rotational_shear_ref_rad",
                             defaults.rotational_shear_ref_rad);
  return defaults;
}

ContactForceRolloutConfig LoadContactForceRolloutConfigFromYamlFile(
    const std::string& yaml_path, ContactForceRolloutConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseContactForceRolloutConfig(RolloutConfigNode(root),
                                          std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error(
        "LoadContactForceRolloutConfigFromYamlFile: failed to load '" +
        yaml_path + "': " + ex.what());
  }
}

}  // namespace mppi_core
