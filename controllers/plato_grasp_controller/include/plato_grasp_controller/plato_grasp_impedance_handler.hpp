#ifndef PLATO_GRASP_CONTROLLER__PLATO_GRASP_IMPEDANCE_HANDLER_HPP_
#define PLATO_GRASP_CONTROLLER__PLATO_GRASP_IMPEDANCE_HANDLER_HPP_

#include <string>

#include "joint_impedance_controller/impedance_handler.hpp"
#include "plato_grasp_controller/plato_grasp_planner.hpp"
#include "plato_interfaces/msg/impedance_commands.hpp"

namespace plato_grasp_controller
{

class PlatoGraspImpedanceHandler
{
public:
  PlatoGraspImpedanceHandler(
    int joint_count,
    std::string impedance_preset_yaml_path);

  bool activate(
    double impedance_level,
    std::string * error_out);

  bool apply_current_impedance(
    const PlatoGraspPlannedCommand & planned_command,
    plato_interfaces::msg::ImpedanceCommands * command_out,
    std::string * error_out) const;

  int joint_count_;
  joint_impedance_controller::ImpedanceHandler impedance_handler_;
  bool has_active_impedance_ = false;
};

}  // namespace plato_grasp_controller

#endif  // PLATO_GRASP_CONTROLLER__PLATO_GRASP_IMPEDANCE_HANDLER_HPP_
