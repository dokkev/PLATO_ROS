#include "plato_utils/joint_position_storage.hpp"
#include "plato_utils/yaml_helpers.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>
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

double round_position_for_yaml(double value)
{
  constexpr double kScale = 1000.0;
  return std::round(value * kScale) / kScale;
}

std::filesystem::path find_workspace_root_from_share_dir(
  const std::filesystem::path & share_dir)
{
  auto current = share_dir;
  while (!current.empty()) {
    if (current.filename() == "install") {
      return current.parent_path();
    }
    const auto parent = current.parent_path();
    if (parent == current) {
      break;
    }
    current = parent;
  }
  return {};
}

std::string package_source_file_path(
  const std::string & package_name,
  const std::string & relative_path)
{
  if (package_name.empty()) {
    return "";
  }

  std::filesystem::path share_dir;
  try {
    share_dir = ament_index_cpp::get_package_share_directory(package_name);
  } catch (const std::exception &) {
    return "";
  }

  const auto workspace_root = find_workspace_root_from_share_dir(share_dir);
  if (workspace_root.empty()) {
    return "";
  }

  const auto src_root = workspace_root / "src";
  std::error_code ec;
  if (!std::filesystem::exists(src_root, ec) || !std::filesystem::is_directory(src_root, ec)) {
    return "";
  }

  const auto direct_candidate = src_root / package_name;
  if (std::filesystem::is_directory(direct_candidate, ec)) {
    return (direct_candidate / relative_path).string();
  }

  for (std::filesystem::recursive_directory_iterator it(
      src_root,
      std::filesystem::directory_options::skip_permission_denied,
      ec);
    !ec && it != std::filesystem::recursive_directory_iterator();
    it.increment(ec))
  {
    if (!it->is_directory()) {
      continue;
    }
    if (it->path().filename() == package_name) {
      return (it->path() / relative_path).string();
    }
  }

  return "";
}

YAML::Node clone_sequence_with_style(
  const YAML::Node & sequence_in,
  YAML::EmitterStyle::value style)
{
  YAML::Node sequence_out(YAML::NodeType::Sequence);
  for (const auto & item : sequence_in) {
    sequence_out.push_back(item);
  }
  sequence_out.SetStyle(style);
  return sequence_out;
}

YAML::Node format_saved_joint_pose_for_output(const YAML::Node & pose_in)
{
  if (!pose_in || !pose_in.IsMap()) {
    return pose_in;
  }

  YAML::Node pose_out(YAML::NodeType::Map);
  pose_out.SetStyle(YAML::EmitterStyle::Block);

  const auto joint_names = pose_in["joint_names"];
  if (joint_names && joint_names.IsSequence()) {
    pose_out["joint_names"] = clone_sequence_with_style(
      joint_names, YAML::EmitterStyle::Flow);
  }

  const auto positions = pose_in["position"];
  if (positions && positions.IsSequence()) {
    pose_out["position"] = clone_sequence_with_style(
      positions, YAML::EmitterStyle::Flow);
  }

  for (const auto & field : pose_in) {
    const auto field_name = field.first.as<std::string>();
    if (field_name == "joint_names" || field_name == "position") {
      continue;
    }
    pose_out[field_name] = field.second;
  }

  return pose_out;
}

YAML::Node format_joint_position_root_for_output(const YAML::Node & root_in)
{
  if (!root_in || !root_in.IsMap()) {
    return root_in;
  }

  YAML::Node root_out(YAML::NodeType::Map);
  root_out.SetStyle(YAML::EmitterStyle::Block);
  for (const auto & entry : root_in) {
    root_out[entry.first.as<std::string>()] =
      format_saved_joint_pose_for_output(entry.second);
  }

  return root_out;
}

}  // namespace

std::string default_joint_position_yaml_path(
  const std::string & package_name,
  const std::string & relative_path)
{
  const auto source_path = package_source_file_path(package_name, relative_path);
  if (!source_path.empty()) {
    return source_path;
  }
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
    positions_node.push_back(round_position_for_yaml(joint_position));
  }

  names_node.SetStyle(YAML::EmitterStyle::Flow);
  positions_node.SetStyle(YAML::EmitterStyle::Flow);
  pose["joint_names"] = names_node;
  pose["position"] = positions_node;
  pose.SetStyle(YAML::EmitterStyle::Block);
  root[saved_name] = pose;

  return plato::yaml::write_yaml_file(
    yaml_path,
    format_joint_position_root_for_output(root),
    error_out);
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
