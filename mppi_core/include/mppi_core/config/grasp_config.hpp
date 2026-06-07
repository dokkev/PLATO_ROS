// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cstddef>
#include <string>

#include <yaml-cpp/yaml.h>

#include "mppi_core/costs/grasp_stability_cost.hpp"
#include "mppi_core/rollout/grasp_state_rollout_model.hpp"
#include "mppi_core/rollout/contact_force_rollout.hpp"
#include "mppi_core/tactile/tactile_transition.hpp"

namespace mppi_core {

GraspStabilityCostConfig ParseGraspConfig(
    const YAML::Node& params, std::size_t action_dim,
    GraspStabilityCostConfig defaults = {});

GraspStabilityCostConfig LoadGraspConfigFromYamlFile(
    const std::string& yaml_path, std::size_t action_dim,
    GraspStabilityCostConfig defaults = {});

GraspStateRolloutConfig ParseGraspStateRolloutConfig(
    const YAML::Node& params, GraspStateRolloutConfig defaults = {});

GraspStateRolloutConfig LoadGraspStateRolloutConfigFromYamlFile(
    const std::string& yaml_path,
    GraspStateRolloutConfig defaults = {});

TactileTransitionConfig ParseTactileTransitionConfig(
    const YAML::Node& params, TactileTransitionConfig defaults = {});

TactileTransitionConfig LoadTactileTransitionConfigFromYamlFile(
    const std::string& yaml_path, TactileTransitionConfig defaults = {});

ContactForceRolloutConfig ParseContactForceRolloutConfig(
    const YAML::Node& params, ContactForceRolloutConfig defaults = {});

ContactForceRolloutConfig LoadContactForceRolloutConfigFromYamlFile(
    const std::string& yaml_path, ContactForceRolloutConfig defaults = {});

}  // namespace mppi_core
