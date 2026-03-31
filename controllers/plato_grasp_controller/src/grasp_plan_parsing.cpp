#include "plato_grasp_controller/grasp_plan_parsing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "plato_utils/joint_state_ordering.hpp"
#include "plato_utils/yaml_helpers.hpp"

namespace plato_grasp_controller
{
namespace parsing
{

namespace
{

std::string trim_copy(const std::string & input)
{
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::vector<std::string> parse_joint_names(
  const YAML::Node & node,
  int joint_count)
{
  if (!node) {
    return {};
  }
  if (!node.IsSequence()) {
    throw std::runtime_error("grasp_force.joint_names must be a sequence.");
  }

  std::vector<std::string> joint_names;
  joint_names.reserve(node.size());
  for (const auto & item : node) {
    const auto joint_name = trim_copy(item.as<std::string>());
    if (!plato::joint_state::is_known_joint_name(joint_name, joint_count)) {
      throw std::runtime_error("Unknown grasp-force joint name: " + joint_name);
    }
    if (std::find(joint_names.begin(), joint_names.end(), joint_name) != joint_names.end()) {
      throw std::runtime_error("Duplicate grasp-force joint name: " + joint_name);
    }
    joint_names.push_back(joint_name);
  }
  return joint_names;
}

std::vector<double> parse_effort_directions(
  const YAML::Node & node,
  size_t expected_size)
{
  if (!node) {
    return std::vector<double>(expected_size, 1.0);
  }
  if (!node.IsSequence()) {
    throw std::runtime_error("grasp_force.effort_direction must be a sequence.");
  }
  if (node.size() != expected_size) {
    throw std::runtime_error(
            "grasp_force.effort_direction must contain exactly " +
            std::to_string(expected_size) + " values.");
  }

  std::vector<double> directions;
  directions.reserve(expected_size);
  for (const auto & item : node) {
    const auto direction = item.as<double>();
    if (!std::isfinite(direction) || std::abs(direction) < 1e-9) {
      throw std::runtime_error(
              "grasp_force.effort_direction values must be finite and non-zero.");
    }
    directions.push_back(direction);
  }

  return directions;
}

std::vector<double> parse_effort_vector_exact(
  const YAML::Node & node,
  int joint_count)
{
  if (!node || !node.IsSequence()) {
    throw std::runtime_error("grasp_force.effort_ff must be a sequence.");
  }

  std::vector<double> effort_ff;
  effort_ff.reserve(node.size());
  for (const auto & item : node) {
    effort_ff.push_back(item.as<double>());
  }

  if (static_cast<int>(effort_ff.size()) != joint_count) {
    throw std::runtime_error(
            "grasp_force.effort_ff must contain exactly " +
            std::to_string(joint_count) + " values.");
  }

  return effort_ff;
}

std::vector<double> parse_legacy_effort_vector(
  const YAML::Node & grasp_force_node,
  int joint_count)
{
  const auto joint_names = parse_joint_names(grasp_force_node["joint_names"], joint_count);
  const auto effort_directions = parse_effort_directions(
    grasp_force_node["effort_direction"], joint_names.size());
  const auto effort_scalar_node = grasp_force_node["effort_ff"];
  if (!effort_scalar_node || effort_scalar_node.IsSequence()) {
    throw std::runtime_error(
            "Legacy grasp_force requires scalar effort_ff with joint_names.");
  }

  const auto effort_scalar = effort_scalar_node.as<double>();
  std::vector<double> effort_ff(static_cast<size_t>(joint_count), 0.0);
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const int joint_index = plato::joint_state::joint_index_for_name(joint_names[i], joint_count);
    if (joint_index >= 0) {
      effort_ff[static_cast<size_t>(joint_index)] = effort_scalar * effort_directions[i];
    }
  }
  return effort_ff;
}

double parse_impedance_level(const YAML::Node & node)
{
  const auto impedance_level_node = node["impedance_level"] ?
    node["impedance_level"] : node["impedance_preset_val"];
  if (!impedance_level_node) {
    throw std::runtime_error("impedance_level is missing.");
  }

  const auto impedance_level = impedance_level_node.as<double>();
  if (!std::isfinite(impedance_level) || impedance_level < 0.0 || impedance_level > 10.0) {
    throw std::runtime_error("impedance_level must be finite and within [0.0, 10.0].");
  }

  return impedance_level;
}

std::optional<GraspPlanConfig> parse_optional_task_grasp_plan(
  const YAML::Node & task_node,
  int joint_count,
  double * grasp_duration_sec_out)
{
  if (grasp_duration_sec_out == nullptr) {
    throw std::runtime_error("grasp_duration_sec output pointer is null.");
  }

  const auto grasp_node = task_node["grasp_plan"] ? task_node["grasp_plan"] : task_node["grasp"];
  if (!grasp_node) {
    return std::nullopt;
  }
  if (!grasp_node.IsMap()) {
    throw std::runtime_error("grasp_plan must be a map.");
  }

  GraspPlanConfig grasp_plan;
  const auto effort_node = grasp_node["effort_ff"];
  if (effort_node && effort_node.IsSequence()) {
    grasp_plan.grasp_force_effort_ff = parse_effort_vector_exact(effort_node, joint_count);
  } else {
    grasp_plan.grasp_force_effort_ff = parse_legacy_effort_vector(grasp_node, joint_count);
  }

  const auto grasp_duration_node = grasp_node["grasp_duration_sec"];
  if (grasp_duration_node) {
    *grasp_duration_sec_out = std::max(0.0, grasp_duration_node.as<double>());
  }

  return grasp_plan;
}

}  // namespace

bool load_task_configs(
  const std::string & yaml_path,
  int joint_count,
  std::unordered_map<std::string, GraspTaskConfig> * task_configs_out,
  std::string * error_out)
{
  if (task_configs_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Task-config output pointer is null.";
    }
    return false;
  }

  YAML::Node root(YAML::NodeType::Map);
  std::string error;
  if (!plato::yaml::load_yaml_map_file(yaml_path, &root, false, &error)) {
    if (error_out != nullptr) {
      *error_out = "Failed to load grasp config from " + yaml_path + ": " + error;
    }
    return false;
  }

  const auto tasks_node = root["tasks"] ? root["tasks"] : root["grasp_tasks"];
  if (!tasks_node) {
    *task_configs_out = {};
    return true;
  }
  if (!tasks_node.IsMap()) {
    if (error_out != nullptr) {
      *error_out = "Task-config YAML 'tasks' entry must be a map: " + yaml_path;
    }
    return false;
  }

  const auto valid_joint_count = plato::joint_state::clamp_joint_count(joint_count);
  std::unordered_map<std::string, GraspTaskConfig> loaded_tasks;
  for (const auto & entry : tasks_node) {
    const auto task_name = trim_copy(entry.first.as<std::string>());
    const auto task_node = entry.second;
    if (!task_node.IsMap()) {
      if (error_out != nullptr) {
        *error_out = "Task '" + task_name + "' must be a map.";
      }
      return false;
    }

    try {
      GraspTaskConfig task;
      const auto use_current_position_node = task_node["use_current_position"];
      if (use_current_position_node) {
        task.use_current_position = use_current_position_node.as<bool>();
      }

      const auto pos_preset_name_node = task_node["pos_preset_name"] ?
        task_node["pos_preset_name"] : task_node["motion_plan_name"];
      if (pos_preset_name_node) {
        task.pos_preset_name = trim_copy(pos_preset_name_node.as<std::string>());
      }
      if (!task.use_current_position && task.pos_preset_name.empty()) {
        throw std::runtime_error("pos_preset_name is empty.");
      }
      task.impedance_level = parse_impedance_level(task_node);

      const auto wait_node = task_node["wait_sec"];
      if (wait_node) {
        task.wait_sec = std::max(0.0, wait_node.as<double>());
      }

      const auto grasp_plan = parse_optional_task_grasp_plan(
        task_node, valid_joint_count, &task.grasp_duration_sec);
      if (!grasp_plan.has_value()) {
        throw std::runtime_error("Task is missing inline grasp_plan.");
      }
      task.grasp_plan = *grasp_plan;

      const auto grasp_duration_node = task_node["grasp_duration_sec"];
      if (grasp_duration_node) {
        task.grasp_duration_sec = std::max(0.0, grasp_duration_node.as<double>());
      }

      loaded_tasks.emplace(task_name, std::move(task));
    } catch (const std::exception & ex) {
      if (error_out != nullptr) {
        *error_out = "Failed to parse task '" + task_name + "': " + ex.what();
      }
      return false;
    }
  }

  *task_configs_out = std::move(loaded_tasks);
  return true;
}

std::vector<double> make_effort_ff_vector(
  const GraspPlanConfig & grasp_plan,
  int joint_count)
{
  if (static_cast<int>(grasp_plan.grasp_force_effort_ff.size()) == joint_count) {
    return grasp_plan.grasp_force_effort_ff;
  }

  std::vector<double> effort_ff(static_cast<size_t>(joint_count), 0.0);
  const auto copy_count = std::min(effort_ff.size(), grasp_plan.grasp_force_effort_ff.size());
  std::copy_n(grasp_plan.grasp_force_effort_ff.begin(), copy_count, effort_ff.begin());
  return effort_ff;
}

}  // namespace parsing
}  // namespace plato_grasp_controller
