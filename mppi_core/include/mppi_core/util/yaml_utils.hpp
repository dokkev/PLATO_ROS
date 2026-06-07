// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cstddef>
#include <string>

#include <yaml-cpp/yaml.h>

namespace mppi_core {
namespace yaml_utils {

bool HasValue(const YAML::Node& node);

double ReadDouble(const YAML::Node& node, const char* key,
                  double default_value);

bool ReadBool(const YAML::Node& node, const char* key, bool default_value);

std::size_t ReadSize(const YAML::Node& node, const char* key,
                     std::size_t default_value);

std::string ReadString(const YAML::Node& node, const char* key,
                       const std::string& default_value);

YAML::Node ReadSection(const YAML::Node& node, const char* key);

YAML::Node RootSectionOrRoot(const YAML::Node& root,
                             const char* section_name,
                             const char* yaml_name);

}  // namespace yaml_utils
}  // namespace mppi_core
