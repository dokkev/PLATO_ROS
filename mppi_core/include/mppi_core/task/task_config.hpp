// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

#include "mppi_core/costs/grasp_stability_cost.hpp"
#include "mppi_core/object/object_prior.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "mppi_core/tactile/tactile_transition.hpp"

namespace mppi_core {

struct TaskToleranceConfig {
  double max_shear_m{0.003};
  double max_rotation_rad{0.05};
};

struct TaskConfig {
  std::string name{"grasp_hold"};
  GraspStartConfig start{};
  GraspStabilityCostConfig cost{};
  TaskToleranceConfig tolerance{};
  ObjectPrior object_prior;
};

TactileTransitionConfig ApplyTaskToleranceToTransitionConfig(
    const TaskConfig& task, TactileTransitionConfig transition);

TaskConfig ParseTaskConfig(const YAML::Node& params,
                           TaskConfig defaults = {});

TaskConfig LoadTaskConfigFromYamlFile(const std::string& yaml_path,
                                      TaskConfig defaults = {});

}  // namespace mppi_core
