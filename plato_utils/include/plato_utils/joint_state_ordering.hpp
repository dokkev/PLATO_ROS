#pragma once

#include <string>
#include <vector>

namespace plato
{
namespace joint_state
{

int clamp_joint_count(int joint_count);

const std::vector<std::string> & canonical_joint_names();

std::vector<std::string> ordered_joint_names(int joint_count);

bool is_known_joint_name(const std::string & joint_name, int joint_count);

int joint_index_for_name(const std::string & joint_name, int joint_count);

bool reorder_named_joint_positions(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  int joint_count,
  bool require_complete,
  std::vector<double> * ordered_positions_out,
  std::string * error_out = nullptr);

}  // namespace joint_state
}  // namespace plato
