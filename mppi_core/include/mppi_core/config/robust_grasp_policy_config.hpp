// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cstddef>
#include <string>

#include <yaml-cpp/yaml.h>

#include "mppi_core/policy/robust_grasp_policy.hpp"

namespace mppi_core {

GraspDisturbanceSamplerConfig ParseGraspDisturbanceSamplerConfig(
    const YAML::Node& params,
    GraspDisturbanceSamplerConfig defaults = {});

DisturbedTactileTransitionConfig ParseDisturbedTactileTransitionConfig(
    const YAML::Node& params,
    DisturbedTactileTransitionConfig defaults = {});

RobustGraspStateCostConfig ParseRobustGraspStateCostConfig(
    const YAML::Node& params,
    RobustGraspStateCostConfig defaults = {});

ObjectContactSupportEvaluatorConfig ParseObjectContactSupportEvaluatorConfig(
    const YAML::Node& params,
    ObjectContactSupportEvaluatorConfig defaults = {});

GraspActionLibraryConfig ParseGraspActionLibraryConfig(
    const YAML::Node& params, std::size_t action_dim,
    GraspActionLibraryConfig defaults = {});

RobustGraspPolicyConfig ParseRobustGraspPolicyConfig(
    const YAML::Node& params, std::size_t action_dim,
    RobustGraspPolicyConfig defaults = {});

RobustGraspPolicyConfig LoadRobustGraspPolicyConfigFromYamlFile(
    const std::string& yaml_path, std::size_t action_dim,
    RobustGraspPolicyConfig defaults = {});

}  // namespace mppi_core
