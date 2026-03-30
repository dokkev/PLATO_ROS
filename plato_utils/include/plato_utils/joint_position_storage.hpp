#pragma once

#include <string>
#include <vector>

namespace plato
{
namespace storage
{

std::string default_joint_position_yaml_path(
  const std::string & package_name,
  const std::string & relative_path = "config/joint_positions.yaml");

std::string normalize_save_name(const std::string & requested_name);

bool load_joint_position_yaml(
  const std::string & yaml_path,
  const std::string & saved_name,
  std::vector<std::string> * joint_names_out,
  std::vector<double> * joint_positions_out,
  std::string * error_out = nullptr);

bool save_joint_position_yaml(
  const std::string & yaml_path,
  const std::string & requested_name,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  std::string * saved_name_out = nullptr,
  std::string * error_out = nullptr);

}  // namespace storage
}  // namespace plato
