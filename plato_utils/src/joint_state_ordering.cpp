#include "plato_utils/joint_state_ordering.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace plato
{
namespace joint_state
{

namespace
{

const std::vector<std::string> kCanonicalJointNames{
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};

}  // namespace

int clamp_joint_count(int joint_count)
{
  return std::clamp(joint_count, 0, static_cast<int>(kCanonicalJointNames.size()));
}

const std::vector<std::string> & canonical_joint_names()
{
  return kCanonicalJointNames;
}

std::vector<std::string> ordered_joint_names(int joint_count)
{
  const auto valid_joint_count = clamp_joint_count(joint_count);
  return std::vector<std::string>(
    kCanonicalJointNames.begin(),
    kCanonicalJointNames.begin() + valid_joint_count);
}

bool is_known_joint_name(const std::string & joint_name, int joint_count)
{
  return joint_index_for_name(joint_name, joint_count) >= 0;
}

int joint_index_for_name(const std::string & joint_name, int joint_count)
{
  const auto valid_joint_count = clamp_joint_count(joint_count);
  for (int i = 0; i < valid_joint_count; ++i) {
    if (joint_name == kCanonicalJointNames[static_cast<size_t>(i)]) {
      return i;
    }
  }
  return -1;
}

bool reorder_named_joint_positions(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  int joint_count,
  bool require_complete,
  std::vector<double> * ordered_positions_out,
  std::string * error_out)
{
  if (ordered_positions_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Ordered-position output pointer is null.";
    }
    return false;
  }

  if (joint_names.size() != joint_positions.size()) {
    if (error_out != nullptr) {
      *error_out = "Named joint-position input has mismatched joint_names and position sizes.";
    }
    return false;
  }

  const auto valid_joint_count = clamp_joint_count(joint_count);
  std::vector<double> ordered_positions(static_cast<size_t>(valid_joint_count), 0.0);
  std::vector<bool> joint_seen(static_cast<size_t>(valid_joint_count), false);
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const int joint_index = joint_index_for_name(joint_names[i], valid_joint_count);
    if (joint_index < 0) {
      if (error_out != nullptr) {
        *error_out = "Unknown joint name: " + joint_names[i];
      }
      return false;
    }
    if (joint_seen[static_cast<size_t>(joint_index)]) {
      if (error_out != nullptr) {
        *error_out = "Duplicate joint name: " + joint_names[i];
      }
      return false;
    }
    ordered_positions[static_cast<size_t>(joint_index)] = joint_positions[i];
    joint_seen[static_cast<size_t>(joint_index)] = true;
  }

  if (require_complete) {
    for (int joint_index = 0; joint_index < valid_joint_count; ++joint_index) {
      if (!joint_seen[static_cast<size_t>(joint_index)]) {
        if (error_out != nullptr) {
          *error_out =
            "Missing joint name: " + kCanonicalJointNames[static_cast<size_t>(joint_index)];
        }
        return false;
      }
    }
  }

  *ordered_positions_out = std::move(ordered_positions);
  return true;
}

}  // namespace joint_state
}  // namespace plato
