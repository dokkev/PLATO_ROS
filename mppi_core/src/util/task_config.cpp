// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/task/task_config.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "mppi_core/util/yaml_utils.hpp"

namespace mppi_core {
namespace {

YAML::Node TaskConfigNode(const YAML::Node& root) {
  return yaml_utils::RootSectionOrRoot(root, "task", "Task YAML");
}

void CheckMap(const YAML::Node& params, const char* function_name) {
  if (yaml_utils::HasValue(params) && !params.IsMap()) {
    throw std::invalid_argument(std::string(function_name) +
                                ": params must be a map");
  }
}

}  // namespace

TactileTransitionConfig ApplyTaskToleranceToTransitionConfig(
    const TaskConfig& task, TactileTransitionConfig transition) {
  transition.max_shear_m = task.tolerance.max_shear_m;
  transition.max_rotation_rad = task.tolerance.max_rotation_rad;
  return transition;
}

TaskConfig ParseTaskConfig(const YAML::Node& params, TaskConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseTaskConfig");

  defaults.name =
      yaml_utils::ReadString(safe_params, "name", defaults.name);

  const YAML::Node start = yaml_utils::ReadSection(safe_params, "start");
  defaults.start.min_enough_contact_sensors = yaml_utils::ReadSize(
      start, "min_enough_contact_sensors",
      defaults.start.min_enough_contact_sensors);
  defaults.start.min_active_hemispheres_total = yaml_utils::ReadSize(
      start, "min_active_hemisphere_total",
      defaults.start.min_active_hemispheres_total);

  const YAML::Node objective =
      yaml_utils::ReadSection(safe_params, "objective");
  defaults.cost.min_active_tactile_sensors = yaml_utils::ReadSize(
      objective, "min_active_tactile_sensors",
      defaults.cost.min_active_tactile_sensors);
  defaults.cost.target_active_hemisphere_total = yaml_utils::ReadSize(
      objective, "target_active_hemisphere_total",
      defaults.cost.target_active_hemisphere_total);

  const YAML::Node tolerance =
      yaml_utils::ReadSection(safe_params, "tolerance");
  defaults.tolerance.max_shear_m = yaml_utils::ReadDouble(
      tolerance, "max_shear_m", defaults.tolerance.max_shear_m);
  defaults.tolerance.max_rotation_rad = yaml_utils::ReadDouble(
      tolerance, "max_rotation_rad", defaults.tolerance.max_rotation_rad);

  const YAML::Node cost = yaml_utils::ReadSection(safe_params, "cost");
  defaults.cost.contact_loss_weight = yaml_utils::ReadDouble(
      cost, "contact_loss_weight", defaults.cost.contact_loss_weight);
  defaults.cost.support_weight =
      yaml_utils::ReadDouble(cost, "support_weight",
                             defaults.cost.support_weight);
  defaults.cost.shear_weight =
      yaml_utils::ReadDouble(cost, "shear_weight",
                             defaults.cost.shear_weight);
  defaults.cost.rotation_weight =
      yaml_utils::ReadDouble(cost, "rotation_weight",
                             defaults.cost.rotation_weight);
  defaults.cost.qddot_weight =
      yaml_utils::ReadDouble(cost, "qddot_weight",
                             defaults.cost.qddot_weight);
  defaults.cost.tau_weight =
      yaml_utils::ReadDouble(cost, "tau_weight", defaults.cost.tau_weight);
  return defaults;
}

TaskConfig LoadTaskConfigFromYamlFile(const std::string& yaml_path,
                                      TaskConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseTaskConfig(TaskConfigNode(root), std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error("LoadTaskConfigFromYamlFile: failed to load '" +
                             yaml_path + "': " + ex.what());
  }
}

}  // namespace mppi_core
