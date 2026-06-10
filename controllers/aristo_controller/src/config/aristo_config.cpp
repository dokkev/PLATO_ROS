#include "aristo_controller/config/aristo_config.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace aristo_controller::config
{
namespace
{

template<typename T>
T optional_scalar(const YAML::Node & node, const std::string & key, const T & fallback)
{
  if (!node || !node[key]) {
    return fallback;
  }
  return node[key].as<T>();
}

YAML::Node required_node(const YAML::Node & node, const std::string & key)
{
  if (!node || !node[key]) {
    throw std::runtime_error("Missing required YAML key '" + key + "'");
  }
  return node[key];
}

template<typename T>
T required_scalar(const YAML::Node & node, const std::string & key)
{
  if (!node || !node[key]) {
    throw std::runtime_error("Missing required YAML key '" + key + "'");
  }
  if (!node[key].IsScalar()) {
    throw std::runtime_error("YAML key '" + key + "' must be a scalar");
  }
  return node[key].as<T>();
}

YAML::Node state_by_name(const YAML::Node & states, const std::string & name)
{
  if (!states || !states.IsSequence()) {
    throw std::runtime_error("state_machine.states must be a YAML sequence");
  }

  for (const auto & state : states) {
    if (state["name"] && state["name"].as<std::string>() == name) {
      return state;
    }
  }
  throw std::runtime_error("state_machine.states does not define state '" + name + "'");
}

Eigen::VectorXd scalar_vector(double value, int size)
{
  return Eigen::VectorXd::Constant(size, value);
}

Eigen::VectorXd parse_vector_or_scalar(
  const YAML::Node & node,
  const std::string & key,
  int size,
  double fallback)
{
  if (!node || !node[key]) {
    return scalar_vector(fallback, size);
  }

  const auto value = node[key];
  if (value.IsScalar()) {
    return scalar_vector(value.as<double>(), size);
  }
  if (!value.IsSequence()) {
    throw std::runtime_error("YAML key '" + key + "' must be a scalar or sequence");
  }
  if (static_cast<int>(value.size()) != size) {
    throw std::runtime_error(
      "YAML key '" + key + "' must have " + std::to_string(size) +
      " entries when specified as a sequence");
  }

  Eigen::VectorXd vector(size);
  for (int i = 0; i < size; ++i) {
    vector[i] = value[static_cast<std::size_t>(i)].as<double>();
  }
  return vector;
}

Eigen::VectorXd parse_required_vector_or_scalar(
  const YAML::Node & node,
  const std::string & key,
  int size)
{
  if (!node || !node[key]) {
    throw std::runtime_error("Missing required YAML key '" + key + "'");
  }
  return parse_vector_or_scalar(node, key, size, 0.0);
}

}  // namespace

std::string default_aristo_config_path()
{
  return ament_index_cpp::get_package_share_directory("aristo_controller") +
         "/config/aristo.yaml";
}

AristoConfig load_aristo_config(const std::string & yaml_path)
{
  const auto root = YAML::LoadFile(yaml_path);
  AristoConfig config;

  const auto robot_model = required_node(root, "robot_model");
  config.robot_model.urdf_path = required_scalar<std::string>(robot_model, "urdf_path");
  if (config.robot_model.urdf_path.empty()) {
    throw std::runtime_error("robot_model.urdf_path must not be empty");
  }
  config.robot_model.is_floating_base =
    optional_scalar<bool>(robot_model, "is_floating_base", false);
  config.robot_model.base_frame = optional_scalar<std::string>(robot_model, "base_frame", "");
  config.robot_model.fixed_thumb = optional_scalar<bool>(robot_model, "fixed_thumb", false);

  const auto controller = required_node(root, "controller");
  config.num_joints = optional_scalar<int>(controller, "num_joints", 8);
  if (config.num_joints <= 0) {
    throw std::runtime_error("controller.num_joints must be positive");
  }

  const auto debug = root["debug"];
  config.debug_enabled = optional_scalar<bool>(debug, "enabled", false);

  const auto driver_gains = required_node(root, "driver_gains");
  config.driver_gains.kp = parse_vector_or_scalar(driver_gains, "kp", config.num_joints, 0.0);
  config.driver_gains.kd = parse_vector_or_scalar(driver_gains, "kd", config.num_joints, 0.0);
  if (
    !config.driver_gains.IsValid() ||
    !config.driver_gains.HasValidDimensions(config.num_joints))
  {
    throw std::runtime_error("driver_gains produced invalid driver PD gains");
  }

  const auto state_machine = required_node(root, "state_machine");
  const auto states = required_node(state_machine, "states");
  const auto initialize = state_by_name(states, "initialize");
  const auto initialize_params = required_node(initialize, "params");

  config.initialize.id = optional_scalar<plato_robot_system::StateId>(initialize, "id", 0);
  config.initialize.duration_sec = optional_scalar<double>(initialize, "duration", 2.0);
  if (config.initialize.duration_sec <= 0.0) {
    throw std::runtime_error("initialize.duration must be positive");
  }
  config.initialize.target_jpos =
    parse_vector_or_scalar(initialize_params, "target_jpos", config.num_joints, 0.0);
  config.initialize.kp_task =
    parse_required_vector_or_scalar(initialize_params, "kp_task", config.num_joints);
  config.initialize.kd_task =
    parse_required_vector_or_scalar(initialize_params, "kd_task", config.num_joints);

  return config;
}

}  // namespace aristo_controller::config
