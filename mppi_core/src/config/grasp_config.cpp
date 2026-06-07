// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/config/grasp_config.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace mppi_core {
namespace {

bool HasValue(const YAML::Node& node) {
  return node && node.Type() != YAML::NodeType::Undefined &&
         node.Type() != YAML::NodeType::Null;
}

double ReadDouble(const YAML::Node& params, const char* key,
                  double default_value) {
  if (!HasValue(params)) {
    return default_value;
  }
  const YAML::Node value = params[key];
  if (!HasValue(value)) {
    return default_value;
  }
  return value.as<double>();
}

bool ReadBool(const YAML::Node& node, const char* key, bool default_value) {
  if (!HasValue(node)) {
    return default_value;
  }
  const YAML::Node value = node[key];
  if (!HasValue(value)) {
    return default_value;
  }
  return value.as<bool>();
}

std::size_t ReadSize(const YAML::Node& node, const char* key,
                     std::size_t default_value) {
  if (!HasValue(node)) {
    return default_value;
  }
  const YAML::Node value = node[key];
  if (!HasValue(value)) {
    return default_value;
  }
  const int parsed = value.as<int>();
  if (parsed < 0) {
    throw std::invalid_argument(std::string("Field '") + key +
                                "' must be nonnegative");
  }
  return static_cast<std::size_t>(parsed);
}

YAML::Node ReadSection(const YAML::Node& node, const char* key) {
  if (!HasValue(node)) {
    return YAML::Node();
  }
  const YAML::Node value = node[key];
  if (!HasValue(value)) {
    return YAML::Node();
  }
  if (!value.IsMap()) {
    throw std::invalid_argument(std::string("Section '") + key +
                                "' must be a map");
  }
  return value;
}

Eigen::Vector2d ReadVector2(const YAML::Node& params, const char* key,
                            const Eigen::Vector2d& default_value) {
  if (!HasValue(params)) {
    return default_value;
  }
  const YAML::Node value = params[key];
  if (!HasValue(value)) {
    return default_value;
  }
  if (!value.IsSequence() || value.size() != 2) {
    throw std::invalid_argument(std::string("Field '") + key +
                                "' must be a length-2 sequence");
  }
  return Eigen::Vector2d{value[0].as<double>(), value[1].as<double>()};
}

Eigen::VectorXd ReadVectorXd(const YAML::Node& params, const char* key,
                             std::size_t expected_size,
                             const Eigen::VectorXd& default_value) {
  if (!HasValue(params)) {
    return default_value;
  }
  const YAML::Node value = params[key];
  if (!HasValue(value)) {
    return default_value;
  }
  if (!value.IsSequence()) {
    throw std::invalid_argument(std::string("Field '") + key +
                                "' must be a sequence");
  }
  if (value.size() != expected_size) {
    throw std::invalid_argument(std::string("Field '") + key +
                                "' dimension mismatch");
  }

  Eigen::VectorXd out(static_cast<Eigen::Index>(expected_size));
  for (std::size_t i = 0; i < expected_size; ++i) {
    out[static_cast<Eigen::Index>(i)] = value[i].as<double>();
  }
  return out;
}

YAML::Node GraspConfigNode(const YAML::Node& root) {
  if (!HasValue(root)) {
    throw std::invalid_argument("Grasp YAML root is empty");
  }
  if (!root.IsMap()) {
    throw std::invalid_argument("Grasp YAML root must be a map");
  }

  const YAML::Node grasp = root["grasp"];
  if (HasValue(grasp)) {
    return grasp;
  }

  return root;
}

}  // namespace

GraspStabilityCostConfig ParseGraspConfig(const YAML::Node& params,
                                          std::size_t action_dim,
                                          GraspStabilityCostConfig defaults) {
  if (action_dim == 0) {
    throw std::invalid_argument("ParseGraspConfig: action_dim is zero");
  }

  const YAML::Node safe_params = HasValue(params) ? params : YAML::Node();
  if (HasValue(safe_params) && !safe_params.IsMap()) {
    throw std::invalid_argument("ParseGraspConfig: params must be a map");
  }

  const YAML::Node force_window =
      ReadSection(safe_params, "normal_force_window");
  defaults.force_min_n =
      ReadDouble(force_window, "min_n", defaults.force_min_n);
  defaults.force_max_n =
      ReadDouble(force_window, "max_n", defaults.force_max_n);
  defaults.force_under_weight =
      ReadDouble(force_window, "under_weight", defaults.force_under_weight);
  defaults.force_over_weight =
      ReadDouble(force_window, "over_weight", defaults.force_over_weight);
  const YAML::Node slip_risk = ReadSection(safe_params, "slip_risk");
  defaults.slip_threshold =
      ReadDouble(slip_risk, "threshold", defaults.slip_threshold);
  defaults.slip_risk_weight =
      ReadDouble(slip_risk, "weight", defaults.slip_risk_weight);
  defaults.slip_velocity_weight =
      ReadDouble(slip_risk, "velocity_weight", defaults.slip_velocity_weight);

  const YAML::Node contact_centroid =
      ReadSection(safe_params, "contact_centroid");
  defaults.contact_centroid_enabled =
      ReadBool(contact_centroid, "enabled", defaults.contact_centroid_enabled);
  defaults.centroid_boundary_weight = ReadDouble(
      contact_centroid, "boundary_weight", defaults.centroid_boundary_weight);
  defaults.centroid_x_min =
      ReadDouble(contact_centroid, "x_min", defaults.centroid_x_min);
  defaults.centroid_x_max =
      ReadDouble(contact_centroid, "x_max", defaults.centroid_x_max);
  defaults.centroid_y_min =
      ReadDouble(contact_centroid, "y_min", defaults.centroid_y_min);
  defaults.centroid_y_max =
      ReadDouble(contact_centroid, "y_max", defaults.centroid_y_max);

  YAML::Node hemisphere_contact =
      ReadSection(safe_params, "hemisphere_contact");
  if (!HasValue(hemisphere_contact)) {
    hemisphere_contact = ReadSection(safe_params, "contact_patch");
  }
  defaults.hemisphere_contact_enabled = ReadBool(
      hemisphere_contact, "enabled", defaults.hemisphere_contact_enabled);
  defaults.target_active_hemisphere_count =
      ReadDouble(hemisphere_contact, "target_active_hemisphere_count",
                 defaults.target_active_hemisphere_count);
  defaults.target_active_hemisphere_count =
      ReadDouble(hemisphere_contact, "target_node_count",
                 defaults.target_active_hemisphere_count);
  defaults.hemisphere_contact_weight = ReadDouble(
      hemisphere_contact, "weight", defaults.hemisphere_contact_weight);

  const YAML::Node tracking_guard = ReadSection(safe_params, "tracking_guard");
  defaults.tracking_weight =
      ReadDouble(tracking_guard, "weight", defaults.tracking_weight);
  defaults.tracking_action_scale_weight =
      ReadDouble(tracking_guard, "action_scale_weight",
                 defaults.tracking_action_scale_weight);

  const YAML::Node action_smoothness =
      ReadSection(safe_params, "action_smoothness");
  defaults.action_smoothness_weight = ReadDouble(
      action_smoothness, "weight", defaults.action_smoothness_weight);

  const YAML::Node joint_limit = ReadSection(safe_params, "joint_limit");
  defaults.joint_limit_weight =
      ReadDouble(joint_limit, "weight", defaults.joint_limit_weight);
  defaults.joint_lower_bound = ReadVectorXd(
      joint_limit, "lower_bound", action_dim, defaults.joint_lower_bound);
  defaults.joint_upper_bound = ReadVectorXd(
      joint_limit, "upper_bound", action_dim, defaults.joint_upper_bound);

  // Backward-compatible flat fields for older experimental configs.
  defaults.force_min_n =
      ReadDouble(safe_params, "force_min_n", defaults.force_min_n);
  defaults.force_max_n =
      ReadDouble(safe_params, "force_max_n", defaults.force_max_n);
  const double force_window_weight =
      ReadDouble(safe_params, "force_window_weight", -1.0);
  if (force_window_weight >= 0.0) {
    defaults.force_under_weight = force_window_weight;
    defaults.force_over_weight = force_window_weight;
  }
  defaults.force_under_weight = ReadDouble(safe_params, "force_under_weight",
                                           defaults.force_under_weight);
  defaults.force_over_weight =
      ReadDouble(safe_params, "force_over_weight", defaults.force_over_weight);
  defaults.slip_threshold =
      ReadDouble(safe_params, "slip_threshold", defaults.slip_threshold);
  defaults.slip_risk_weight =
      ReadDouble(safe_params, "slip_weight", defaults.slip_risk_weight);
  const Eigen::Vector2d centroid_abs_limit_m = ReadVector2(
      safe_params, "centroid_abs_limit_m", Eigen::Vector2d{-1.0, -1.0});
  if ((centroid_abs_limit_m.array() > 0.0).all()) {
    defaults.centroid_x_min = -centroid_abs_limit_m.x();
    defaults.centroid_x_max = centroid_abs_limit_m.x();
    defaults.centroid_y_min = -centroid_abs_limit_m.y();
    defaults.centroid_y_max = centroid_abs_limit_m.y();
  }
  defaults.centroid_boundary_weight = ReadDouble(
      safe_params, "centroid_weight", defaults.centroid_boundary_weight);
  defaults.contact_loss_weight = ReadDouble(safe_params, "contact_loss_weight",
                                            defaults.contact_loss_weight);
  defaults.action_smoothness_weight = ReadDouble(
      safe_params, "action_weight", defaults.action_smoothness_weight);
  return defaults;
}

GraspStabilityCostConfig LoadGraspConfigFromYamlFile(
    const std::string& yaml_path, std::size_t action_dim,
    GraspStabilityCostConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseGraspConfig(GraspConfigNode(root), action_dim,
                            std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error("LoadGraspConfigFromYamlFile: failed to load '" +
                             yaml_path + "': " + ex.what());
  }
}

GraspStateRolloutConfig ParseGraspStateRolloutConfig(
    const YAML::Node& params, GraspStateRolloutConfig defaults) {
  const YAML::Node safe_params = HasValue(params) ? params : YAML::Node();
  if (HasValue(safe_params) && !safe_params.IsMap()) {
    throw std::invalid_argument(
        "ParseGraspStateRolloutConfig: params must be a map");
  }

  return defaults;
}

GraspStateRolloutConfig LoadGraspStateRolloutConfigFromYamlFile(
    const std::string& yaml_path, GraspStateRolloutConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseGraspStateRolloutConfig(GraspConfigNode(root),
                                        std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error(
        "LoadGraspStateRolloutConfigFromYamlFile: failed to load '" +
        yaml_path + "': " + ex.what());
  }
}

TactileTransitionConfig ParseTactileTransitionConfig(
    const YAML::Node& params, TactileTransitionConfig defaults) {
  const YAML::Node safe_params = HasValue(params) ? params : YAML::Node();
  if (HasValue(safe_params) && !safe_params.IsMap()) {
    throw std::invalid_argument(
        "ParseTactileTransitionConfig: params must be a map");
  }

  const YAML::Node transition = ReadSection(safe_params, "tactile_transition");
  defaults.enable_birth =
      ReadBool(transition, "enable_birth", defaults.enable_birth);
  defaults.enable_loss =
      ReadBool(transition, "enable_loss", defaults.enable_loss);

  defaults.birth_score_threshold =
      ReadDouble(transition, "birth_score_threshold",
                 defaults.birth_score_threshold);
  defaults.loss_score_threshold =
      ReadDouble(transition, "loss_score_threshold",
                 defaults.loss_score_threshold);

  defaults.birth_neighbor_weight =
      ReadDouble(transition, "birth_neighbor_weight",
                 defaults.birth_neighbor_weight);
  defaults.birth_tangent_approach_weight =
      ReadDouble(transition, "birth_tangent_approach_weight",
                 defaults.birth_tangent_approach_weight);
  defaults.birth_normal_approach_weight =
      ReadDouble(transition, "birth_normal_approach_weight",
                 defaults.birth_normal_approach_weight);
  defaults.birth_shear_penalty_weight =
      ReadDouble(transition, "birth_shear_penalty_weight",
                 defaults.birth_shear_penalty_weight);

  defaults.loss_unloading_weight =
      ReadDouble(transition, "loss_unloading_weight",
                 defaults.loss_unloading_weight);
  defaults.loss_shear_weight =
      ReadDouble(transition, "loss_shear_weight", defaults.loss_shear_weight);
  defaults.loss_low_force_weight =
      ReadDouble(transition, "loss_low_force_weight",
                 defaults.loss_low_force_weight);

  defaults.born_normal_force_n =
      ReadDouble(transition, "born_normal_force_n",
                 defaults.born_normal_force_n);
  defaults.born_confidence =
      ReadDouble(transition, "born_confidence", defaults.born_confidence);

  defaults.inactive_confidence =
      ReadDouble(transition, "inactive_confidence",
                 defaults.inactive_confidence);
  defaults.contact_confidence_decay =
      ReadDouble(transition, "contact_confidence_decay",
                 defaults.contact_confidence_decay);

  defaults.aggregate_shear_decay =
      ReadDouble(transition, "aggregate_shear_decay",
                 defaults.aggregate_shear_decay);
  defaults.aggregate_rotation_decay =
      ReadDouble(transition, "aggregate_rotation_decay",
                 defaults.aggregate_rotation_decay);

  defaults.enough_contact_hemisphere_count =
      ReadSize(transition, "enough_contact_hemisphere_count",
               defaults.enough_contact_hemisphere_count);
  return defaults;
}

TactileTransitionConfig LoadTactileTransitionConfigFromYamlFile(
    const std::string& yaml_path, TactileTransitionConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseTactileTransitionConfig(GraspConfigNode(root),
                                        std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error(
        "LoadTactileTransitionConfigFromYamlFile: failed to load '" +
        yaml_path + "': " + ex.what());
  }
}

ContactForceRolloutConfig ParseContactForceRolloutConfig(
    const YAML::Node& params, ContactForceRolloutConfig defaults) {
  const YAML::Node safe_params = HasValue(params) ? params : YAML::Node();
  if (HasValue(safe_params) && !safe_params.IsMap()) {
    throw std::invalid_argument(
        "ParseContactForceRolloutConfig: params must be a map");
  }

  const YAML::Node rollout = ReadSection(safe_params, "contact_force_rollout");
  defaults.enable_force_projection_update =
      ReadBool(rollout, "enable_force_projection_update",
               defaults.enable_force_projection_update);
  defaults.force_lowpass_alpha =
      ReadDouble(rollout, "force_lowpass_alpha", defaults.force_lowpass_alpha);
  defaults.max_predicted_normal_force_n =
      ReadDouble(rollout, "max_predicted_normal_force_n",
                 defaults.max_predicted_normal_force_n);
  defaults.shear_force_gain_m_per_n_s =
      ReadDouble(rollout, "shear_force_gain_m_per_n_s",
                 defaults.shear_force_gain_m_per_n_s);
  defaults.rotational_shear_gain_rad_per_nm_s =
      ReadDouble(rollout, "rotational_shear_gain_rad_per_nm_s",
                 defaults.rotational_shear_gain_rad_per_nm_s);
  defaults.friction_violation_confidence_decay =
      ReadDouble(rollout, "friction_violation_confidence_decay",
                 defaults.friction_violation_confidence_decay);
  defaults.negative_normal_confidence_decay =
      ReadDouble(rollout, "negative_normal_confidence_decay",
                 defaults.negative_normal_confidence_decay);
  defaults.min_active_hemisphere_count =
      ReadSize(rollout, "min_active_hemisphere_count",
               defaults.min_active_hemisphere_count);
  defaults.min_active_hemisphere_count =
      ReadSize(rollout, "min_stable_support_count",
               defaults.min_active_hemisphere_count);
  defaults.min_contact_confidence = ReadDouble(
      rollout, "min_contact_confidence", defaults.min_contact_confidence);
  defaults.shear_ref_m =
      ReadDouble(rollout, "shear_ref_m", defaults.shear_ref_m);
  defaults.rotational_shear_ref_rad = ReadDouble(
      rollout, "rotational_shear_ref_rad", defaults.rotational_shear_ref_rad);
  return defaults;
}

ContactForceRolloutConfig LoadContactForceRolloutConfigFromYamlFile(
    const std::string& yaml_path, ContactForceRolloutConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseContactForceRolloutConfig(GraspConfigNode(root),
                                          std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error(
        "LoadContactForceRolloutConfigFromYamlFile: failed to load '" +
        yaml_path + "': " + ex.what());
  }
}

}  // namespace mppi_core
