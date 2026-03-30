#ifndef PLATO_GRASP_CONTROLLER__GRASP_PLAN_PARSING_HPP_
#define PLATO_GRASP_CONTROLLER__GRASP_PLAN_PARSING_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "plato_grasp_controller/plato_grasp_planner.hpp"

namespace plato_grasp_controller
{
namespace parsing
{

bool load_task_configs(
  const std::string & yaml_path,
  int joint_count,
  std::unordered_map<std::string, GraspTaskConfig> * task_configs_out,
  std::string * error_out = nullptr);

std::vector<double> make_effort_ff_vector(
  const GraspPlanConfig & grasp_plan,
  int joint_count);

}  // namespace parsing
}  // namespace plato_grasp_controller

#endif  // PLATO_GRASP_CONTROLLER__GRASP_PLAN_PARSING_HPP_
