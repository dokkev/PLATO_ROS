// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/util/yaml_utils.hpp"

#include <stdexcept>
#include <string>

namespace mppi_core {
namespace yaml_utils {

bool HasValue(const YAML::Node& node) {
  return node && node.Type() != YAML::NodeType::Undefined &&
         node.Type() != YAML::NodeType::Null;
}

double ReadDouble(const YAML::Node& node, const char* key,
                  double default_value) {
  if (!HasValue(node)) {
    return default_value;
  }
  const YAML::Node value = node[key];
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

std::string ReadString(const YAML::Node& node, const char* key,
                       const std::string& default_value) {
  if (!HasValue(node)) {
    return default_value;
  }
  const YAML::Node value = node[key];
  if (!HasValue(value)) {
    return default_value;
  }
  return value.as<std::string>();
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

YAML::Node RootSectionOrRoot(const YAML::Node& root,
                             const char* section_name,
                             const char* yaml_name) {
  if (!HasValue(root)) {
    throw std::invalid_argument(std::string(yaml_name) + " root is empty");
  }
  if (!root.IsMap()) {
    throw std::invalid_argument(std::string(yaml_name) +
                                " root must be a map");
  }

  const YAML::Node section = root[section_name];
  if (HasValue(section)) {
    if (!section.IsMap()) {
      throw std::invalid_argument(std::string("Section '") + section_name +
                                  "' must be a map");
    }
    return section;
  }

  return root;
}

}  // namespace yaml_utils
}  // namespace mppi_core
