#include "plato_hardware_interface/utils/actuator_config_loader.hpp"

#include <array>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace plato_actuator
{

namespace
{

constexpr std::array<const char *, 8> kExpectedActuatorNames = {
  "thumb_roll",
  "thumb_yaw",
  "thumb_mcp",
  "thumb_pip",
  "index_mcp",
  "index_pip",
  "middle_mcp",
  "middle_pip",
};

std::string require_scalar(const YAML::Node & node, const char * key)
{
  const auto value = node[key];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error(std::string("Missing scalar field: ") + key);
  }
  return value.as<std::string>();
}

uint8_t parse_u8(const YAML::Node & node, const char * key)
{
  const auto raw = require_scalar(node, key);
  const auto parsed = std::stoul(raw, nullptr, 0);
  if (parsed > UINT8_MAX) {
    throw std::runtime_error(std::string("Value out of range for ") + key + ": " + raw);
  }
  return static_cast<uint8_t>(parsed);
}

uint32_t parse_u32_or_default(const YAML::Node & node, const char * key, uint32_t default_value)
{
  const auto value = node[key];
  if (!value) {
    return default_value;
  }
  if (!value.IsScalar()) {
    throw std::runtime_error(std::string("Missing scalar field: ") + key);
  }
  return static_cast<uint32_t>(std::stoul(value.as<std::string>(), nullptr, 0));
}

int8_t parse_direction(const YAML::Node & node)
{
  const auto raw = require_scalar(node, "direction");
  const auto parsed = std::stoi(raw, nullptr, 0);
  if (parsed != -1 && parsed != 1) {
    throw std::runtime_error("direction must be either -1 or 1");
  }
  return static_cast<int8_t>(parsed);
}

float parse_float(const YAML::Node & node, const char * key)
{
  const auto value = node[key];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error(std::string("Missing scalar field: ") + key);
  }
  return value.as<float>();
}

actuator::Limits parse_limits(const YAML::Node & node)
{
  const auto limits_node = node["limits"];
  if (!limits_node || !limits_node.IsMap()) {
    throw std::runtime_error("Missing limits map");
  }

  actuator::Limits limits;
  limits.position_limit_min = parse_float(limits_node, "min");
  limits.position_limit_max = parse_float(limits_node, "max");
  return limits;
}

Config parse_config(const YAML::Node & node)
{
  Config config;
  config.core.can_tx_id = parse_u8(node, "tx_id");
  config.core.can_rx_id = parse_u8(node, "rx_id");
  config.core.direction = parse_direction(node);
  config.core.torque_constant = parse_float(node, "torque_constant");
  config.core.gear_ratio = parse_float(node, "gear_ratio");
  config.limits = parse_limits(node);
  config.servo_current_milliamps = parse_u32_or_default(node, "servo_current_milliamps", 0);
  return config;
}

void validate_can_ids(const std::vector<Config> & configs)
{
  std::set<uint8_t> tx_ids;
  std::set<uint8_t> rx_ids;

  for (const auto & config : configs) {
    if (!tx_ids.insert(config.core.can_tx_id).second) {
      throw std::runtime_error("Duplicate tx_id found in actuator config");
    }
    if (!rx_ids.insert(config.core.can_rx_id).second) {
      throw std::runtime_error("Duplicate rx_id found in actuator config");
    }
  }
}

}  // namespace

const std::array<const char *, 8> & expected_plato_actuator_names()
{
  return kExpectedActuatorNames;
}

std::vector<Config> load_plato_actuator_configs()
{
  const auto yaml_path =
    ament_index_cpp::get_package_share_directory("plato_hardware_interface") +
    "/config/plato_actuators.yaml";
  return load_plato_actuator_configs(yaml_path);
}

std::vector<Config> load_plato_actuator_configs(const std::string & yaml_path)
{
  const auto root = YAML::LoadFile(yaml_path);
  const auto actuators_node = root["actuators"];
  if (!actuators_node || !actuators_node.IsSequence()) {
    throw std::runtime_error("Expected 'actuators' sequence in " + yaml_path);
  }

  std::unordered_map<std::string, Config> configs_by_name;
  configs_by_name.reserve(actuators_node.size());

  for (const auto & actuator_node : actuators_node) {
    const auto name = require_scalar(actuator_node, "name");
    const auto inserted = configs_by_name.emplace(name, parse_config(actuator_node));
    if (!inserted.second) {
      throw std::runtime_error("Duplicate actuator name in YAML: " + name);
    }
  }

  if (configs_by_name.size() != kExpectedActuatorNames.size()) {
    throw std::runtime_error(
      "Expected exactly " + std::to_string(kExpectedActuatorNames.size()) +
      " actuators in YAML");
  }

  std::vector<Config> configs;
  configs.reserve(kExpectedActuatorNames.size());

  for (const auto * expected_name : kExpectedActuatorNames) {
    const auto it = configs_by_name.find(expected_name);
    if (it == configs_by_name.end()) {
      throw std::runtime_error(std::string("Missing actuator entry in YAML: ") + expected_name);
    }
    configs.push_back(it->second);
  }

  validate_can_ids(configs);
  return configs;
}

}  // namespace plato_actuator
