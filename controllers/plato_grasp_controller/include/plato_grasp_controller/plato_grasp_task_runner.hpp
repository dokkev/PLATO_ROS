#ifndef PLATO_GRASP_CONTROLLER__PLATO_GRASP_TASK_RUNNER_HPP_
#define PLATO_GRASP_CONTROLLER__PLATO_GRASP_TASK_RUNNER_HPP_

#include <optional>
#include <string>
#include <unordered_map>

#include "plato_grasp_controller/plato_grasp_planner.hpp"

namespace plato_grasp_controller
{

class PlatoGraspTaskRunner
{
public:
  enum class State
  {
    Idle,
    Waiting,
    Grasping
  };

  enum class ActionType
  {
    None,
    PublishMotionPlan,
    PublishGraspPlan,
    PublishMotionHold
  };

  struct Action
  {
    ActionType type = ActionType::None;
    std::string pos_preset_name;
    bool use_current_position = false;
    double impedance_level = 0.0;
    std::optional<GraspPlanConfig> grasp_plan;
  };

  explicit PlatoGraspTaskRunner(
    std::unordered_map<std::string, GraspTaskConfig> task_configs);

  bool start_task(
    const std::string & task_name,
    Action * action_out,
    std::string * error_out = nullptr);

  void cancel_task();

  bool update(
    double dt_sec,
    Action * action_out,
    std::string * error_out = nullptr);

  bool is_active() const;
  State state() const;
  const std::string & active_task_name() const;

private:
  std::unordered_map<std::string, GraspTaskConfig> task_configs_;
  State state_ = State::Idle;
  std::string active_task_name_;
  GraspTaskConfig active_task_config_;
  double state_elapsed_sec_ = 0.0;
};

}  // namespace plato_grasp_controller

#endif  // PLATO_GRASP_CONTROLLER__PLATO_GRASP_TASK_RUNNER_HPP_
