#include "plato_utils/joint_position_storage.hpp"
#include "plato_utils/yaml_helpers.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace plato
{
namespace storage
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

std::string make_timestamp_name()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf {};
  localtime_r(&now_time, &tm_buf);

  std::ostringstream oss;
  oss << "joint_position_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
  return oss.str();
}

}  // namespace

std::string default_joint_position_yaml_path(
  const std::string & package_name,
  const std::string & relative_path)
{
  return plato::yaml::package_share_file_path(package_name, relative_path);
}

std::string normalize_save_name(const std::string & requested_name)
{
  const auto trimmed = trim_copy(requested_name);
  if (!trimmed.empty()) {
    return trimmed;
  }
  return make_timestamp_name();
}

bool save_joint_position_yaml(
  const std::string & yaml_path,
  const std::string & requested_name,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  std::string * saved_name_out,
  std::string * error_out)
{
  if (yaml_path.empty()) {
    if (error_out != nullptr) {
      *error_out = "Output YAML path is empty.";
    }
    return false;
  }

  if (joint_positions.empty()) {
    if (error_out != nullptr) {
      *error_out = "No joint positions available to save.";
    }
    return false;
  }

  if (joint_names.size() != joint_positions.size()) {
    if (error_out != nullptr) {
      *error_out = "Joint name count does not match joint position count.";
    }
    return false;
  }

  const auto saved_name = normalize_save_name(requested_name);
  if (saved_name_out != nullptr) {
    *saved_name_out = saved_name;
  }

  YAML::Node root(YAML::NodeType::Map);
  if (!plato::yaml::load_yaml_map_file(
      yaml_path,
      &root,
      true,
      error_out))
  {
    return false;
  }

  YAML::Node pose(YAML::NodeType::Map);
  YAML::Node names_node(YAML::NodeType::Sequence);
  YAML::Node positions_node(YAML::NodeType::Sequence);

  for (const auto & joint_name : joint_names) {
    names_node.push_back(joint_name);
  }
  for (const auto joint_position : joint_positions) {
    positions_node.push_back(joint_position);
  }

  names_node.SetStyle(YAML::EmitterStyle::Flow);
  positions_node.SetStyle(YAML::EmitterStyle::Flow);
  pose["joint_names"] = names_node;
  pose["position"] = positions_node;
  root[saved_name] = pose;

  return plato::yaml::write_yaml_file(yaml_path, root, error_out);
}

bool load_joint_position_yaml(
  const std::string & yaml_path,
  const std::string & saved_name,
  std::vector<std::string> * joint_names_out,
  std::vector<double> * joint_positions_out,
  std::string * error_out)
{
  if (joint_names_out == nullptr || joint_positions_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Output vector pointer is null.";
    }
    return false;
  }

  const auto normalized_name = trim_copy(saved_name);
  if (normalized_name.empty()) {
    if (error_out != nullptr) {
      *error_out = "Saved joint-position name is empty.";
    }
    return false;
  }

  YAML::Node root(YAML::NodeType::Map);
  if (!plato::yaml::load_yaml_map_file(yaml_path, &root, false, error_out)) {
    return false;
  }

  const auto pose = root[normalized_name];
  if (!pose || !pose.IsMap()) {
    if (error_out != nullptr) {
      *error_out = "Saved joint-position entry not found: " + normalized_name;
    }
    return false;
  }

  const auto joint_names_node = pose["joint_names"];
  const auto positions_node = pose["position"];
  if (!joint_names_node || !joint_names_node.IsSequence()) {
    if (error_out != nullptr) {
      *error_out = "Saved joint-position entry is missing 'joint_names' sequence.";
    }
    return false;
  }
  if (!positions_node || !positions_node.IsSequence()) {
    if (error_out != nullptr) {
      *error_out = "Saved joint-position entry is missing 'position' sequence.";
    }
    return false;
  }

  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  joint_names.reserve(joint_names_node.size());
  joint_positions.reserve(positions_node.size());

  try {
    for (const auto & joint_name_node : joint_names_node) {
      joint_names.push_back(joint_name_node.as<std::string>());
    }
    for (const auto & position_node : positions_node) {
      joint_positions.push_back(position_node.as<double>());
    }
  } catch (const std::exception & ex) {
    if (error_out != nullptr) {
      *error_out = std::string("Failed to parse saved joint-position entry: ") + ex.what();
    }
    return false;
  }

  if (joint_names.size() != joint_positions.size()) {
    if (error_out != nullptr) {
      *error_out = "Saved joint-position entry has mismatched joint_names and position sizes.";
    }
    return false;
  }

  *joint_names_out = std::move(joint_names);
  *joint_positions_out = std::move(joint_positions);
  return true;
}

}  // namespace storage
}  // namespace plato
