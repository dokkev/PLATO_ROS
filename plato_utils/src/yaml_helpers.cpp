#include "plato_utils/yaml_helpers.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <exception>
#include <filesystem>
#include <fstream>
#include <string>

namespace plato
{
namespace yaml
{

namespace
{

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

}  // namespace

std::string package_share_file_path(
  const std::string & package_name,
  const std::string & relative_path)
{
  try {
    return (
      std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name)) /
      relative_path).string();
  } catch (const std::exception &) {
    return relative_path;
  }
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

std::string package_source_or_share_file_path(
  const std::string & package_name,
  const std::string & relative_path)
{
  const auto source_path = package_source_file_path(package_name, relative_path);
  if (!source_path.empty()) {
    return source_path;
  }
  return package_share_file_path(package_name, relative_path);
}

bool create_parent_directories(
  const std::string & file_path,
  std::string * error_out)
{
  if (file_path.empty()) {
    if (error_out != nullptr) {
      *error_out = "Output file path is empty.";
    }
    return false;
  }

  try {
    const auto path = std::filesystem::path(file_path);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }
  } catch (const std::exception & ex) {
    if (error_out != nullptr) {
      *error_out = std::string("Failed to create output directory: ") + ex.what();
    }
    return false;
  }

  return true;
}

bool load_yaml_file(
  const std::string & yaml_path,
  YAML::Node * root_out,
  std::string * error_out)
{
  if (root_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Output YAML node pointer is null.";
    }
    return false;
  }

  if (yaml_path.empty()) {
    if (error_out != nullptr) {
      *error_out = "YAML file path is empty.";
    }
    return false;
  }

  if (!std::filesystem::exists(yaml_path)) {
    if (error_out != nullptr) {
      *error_out = "YAML file does not exist: " + yaml_path;
    }
    return false;
  }

  try {
    *root_out = YAML::LoadFile(yaml_path);
  } catch (const std::exception & ex) {
    if (error_out != nullptr) {
      *error_out = std::string("Failed to parse YAML file: ") + ex.what();
    }
    return false;
  }

  return true;
}

bool load_yaml_map_file(
  const std::string & yaml_path,
  YAML::Node * root_out,
  bool allow_missing,
  std::string * error_out)
{
  if (root_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Output YAML node pointer is null.";
    }
    return false;
  }

  if (yaml_path.empty()) {
    if (error_out != nullptr) {
      *error_out = "YAML file path is empty.";
    }
    return false;
  }

  if (allow_missing && !std::filesystem::exists(yaml_path)) {
    *root_out = YAML::Node(YAML::NodeType::Map);
    return true;
  }

  if (!load_yaml_file(yaml_path, root_out, error_out)) {
    return false;
  }

  if (!(*root_out) || root_out->IsNull()) {
    *root_out = YAML::Node(YAML::NodeType::Map);
    return true;
  }

  if (!root_out->IsMap()) {
    if (error_out != nullptr) {
      *error_out = "YAML root is not a map: " + yaml_path;
    }
    return false;
  }

  return true;
}

bool write_yaml_file(
  const std::string & yaml_path,
  const YAML::Node & root,
  std::string * error_out)
{
  if (!create_parent_directories(yaml_path, error_out)) {
    return false;
  }

  YAML::Emitter emitter;
  emitter << root;
  if (!emitter.good()) {
    if (error_out != nullptr) {
      *error_out = std::string("Failed to emit YAML: ") + emitter.GetLastError();
    }
    return false;
  }

  std::ofstream out(yaml_path, std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    if (error_out != nullptr) {
      *error_out = "Failed to open YAML file for writing.";
    }
    return false;
  }

  out << emitter.c_str() << '\n';
  if (!out.good()) {
    if (error_out != nullptr) {
      *error_out = "Failed while writing YAML file.";
    }
    return false;
  }

  return true;
}

}  // namespace yaml
}  // namespace plato
