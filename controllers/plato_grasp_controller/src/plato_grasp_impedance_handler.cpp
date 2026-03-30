#include "plato_grasp_controller/plato_grasp_impedance_handler.hpp"

#include <string>
#include <utility>

#include "plato_utils/joint_state_ordering.hpp"

namespace plato_grasp_controller
{

PlatoGraspImpedanceHandler::PlatoGraspImpedanceHandler(
  int joint_count,
  std::string impedance_preset_yaml_path)
: joint_count_(plato::joint_state::clamp_joint_count(joint_count)),
  impedance_handler_(joint_count_, std::move(impedance_preset_yaml_path))
{
}

bool PlatoGraspImpedanceHandler::activate(
  double impedance_level,
  std::string * error_out)
{
  if (!impedance_handler_.set_level(impedance_level, error_out)) {
    return false;
  }
  has_active_impedance_ = true;
  return true;
}

bool PlatoGraspImpedanceHandler::apply_current_impedance(
  const PlatoGraspPlannedCommand & planned_command,
  plato_interfaces::msg::ImpedanceCommands * command_out,
  std::string * error_out) const
{
  if (command_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Impedance command output pointer is null.";
    }
    return false;
  }

  if (!has_active_impedance_) {
    if (error_out != nullptr) {
      *error_out = "No active motion plan is set in the impedance handler.";
    }
    return false;
  }

  const auto gains = impedance_handler_.gains();

  command_out->position = planned_command.position;
  command_out->velocity = planned_command.velocity;
  command_out->effort_ff = planned_command.effort_ff;
  command_out->stiffness = gains.stiffness;
  command_out->damping = gains.damping;
  return true;
}

}  // namespace plato_grasp_controller
