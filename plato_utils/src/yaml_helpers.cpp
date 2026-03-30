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
