#include "plato_grasp_controller/plato_grasp_planner.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "plato_grasp_controller/grasp_plan_parsing.hpp"
#include "plato_utils/joint_position_storage.hpp"
#include "plato_utils/joint_state_ordering.hpp"

namespace plato_grasp_controller
{

PlatoGraspPlanner::PlatoGraspPlanner(
  int joint_count,
  std::string saved_joint_positions_yaml_path)
: joint_count_(plato::joint_state::clamp_joint_count(joint_count)),
  saved_joint_positions_yaml_path_(std::move(saved_joint_positions_yaml_path))
{
  last_positions_.assign(static_cast<size_t>(joint_count_), 0.0);
  active_target_positions_.assign(static_cast<size_t>(joint_count_), 0.0);
}

void PlatoGraspPlanner::update_joint_state(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions)
{
  std::vector<double> ordered_positions(static_cast<size_t>(joint_count_), 0.0);
  const auto valid_size = std::min(joint_names.size(), joint_positions.size());
  for (size_t i = 0; i < valid_size; ++i) {
    const int joint_index = plato::joint_state::joint_index_for_name(joint_names[i], joint_count_);
    if (joint_index >= 0) {
      ordered_positions[static_cast<size_t>(joint_index)] = joint_positions[i];
    }
  }

  last_positions_ = std::move(ordered_positions);
  has_joint_state_ = true;
}

bool PlatoGraspPlanner::save_current_joint_position(
  const std::string & requested_name,
  std::string * saved_name_out,
  std::string * error_out)
{
  if (!has_joint_state_) {
    if (error_out != nullptr) {
      *error_out = "Current joint positions have not been received yet.";
    }
    return false;
  }

  const auto joint_names = plato::joint_state::ordered_joint_names(joint_count_);
  return plato::storage::save_joint_position_yaml(
    saved_joint_positions_yaml_path_,
    requested_name,
    joint_names,
    last_positions_,
    saved_name_out,
    error_out);
}

bool PlatoGraspPlanner::make_motion_command(
  const std::string & pos_preset_name,
  PlatoGraspPlannedCommand * command_out,
  std::string * error_out)
{
  if (command_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Motion command output pointer is null.";
    }
    return false;
  }

  if (pos_preset_name.empty()) {
    if (error_out != nullptr) {
      *error_out = "Position preset name is empty.";
    }
    return false;
  }

  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  if (!plato::storage::load_joint_position_yaml(
      saved_joint_positions_yaml_path_,
      pos_preset_name,
      &joint_names,
      &joint_positions,
      error_out))
  {
    return false;
  }

  std::vector<double> ordered_positions;
  if (!plato::joint_state::reorder_named_joint_positions(
      joint_names,
      joint_positions,
      joint_count_,
      true,
      &ordered_positions,
      error_out))
  {
    return false;
  }

  command_out->position = ordered_positions;
  command_out->velocity.assign(command_out->position.size(), 0.0);
  command_out->effort_ff.assign(command_out->position.size(), 0.0);

  active_target_positions_ = std::move(ordered_positions);
  active_pos_preset_name_ = pos_preset_name;
  phase_ = Phase::Motion;
  return true;
}

bool PlatoGraspPlanner::make_grasp_command(
  const GraspPlanConfig & grasp_plan,
  PlatoGraspPlannedCommand * command_out,
  std::string * error_out)
{
  if (command_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Grasp-plan command output pointer is null.";
    }
    return false;
  }
  if (active_pos_preset_name_.empty()) {
    if (error_out != nullptr) {
      *error_out = "Cannot apply grasp plan before a motion target has been selected.";
    }
    return false;
  }

  command_out->position = active_target_positions_;
  command_out->velocity.assign(command_out->position.size(), 0.0);
  command_out->effort_ff = parsing::make_effort_ff_vector(grasp_plan, joint_count_);

  phase_ = Phase::Grasp;
  return true;
}

bool PlatoGraspPlanner::has_joint_state() const
{
  return has_joint_state_;
}

PlatoGraspPlanner::Phase PlatoGraspPlanner::phase() const
{
  return phase_;
}

const std::string & PlatoGraspPlanner::saved_joint_positions_yaml_path() const
{
  return saved_joint_positions_yaml_path_;
}

const std::string & PlatoGraspPlanner::active_pos_preset_name() const
{
  return active_pos_preset_name_;
}

}  // namespace plato_grasp_controller
