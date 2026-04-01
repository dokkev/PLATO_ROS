#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

namespace plato
{
namespace yaml
{

std::string package_share_file_path(
  const std::string & package_name,
  const std::string & relative_path);

std::string package_source_file_path(
  const std::string & package_name,
  const std::string & relative_path);

std::string package_source_or_share_file_path(
  const std::string & package_name,
  const std::string & relative_path);

bool create_parent_directories(
  const std::string & file_path,
  std::string * error_out = nullptr);

bool load_yaml_file(
  const std::string & yaml_path,
  YAML::Node * root_out,
  std::string * error_out = nullptr);

bool load_yaml_map_file(
  const std::string & yaml_path,
  YAML::Node * root_out,
  bool allow_missing = false,
  std::string * error_out = nullptr);

bool write_yaml_file(
  const std::string & yaml_path,
  const YAML::Node & root,
  std::string * error_out = nullptr);

}  // namespace yaml
}  // namespace plato
