#include "plato_grasp_controller/plato_grasp_task_runner.hpp"

#include <algorithm>
#include <string>
#include <utility>

namespace plato_grasp_controller
{

PlatoGraspTaskRunner::PlatoGraspTaskRunner(
  std::unordered_map<std::string, GraspTaskConfig> task_configs)
: task_configs_(std::move(task_configs))
{
}

bool PlatoGraspTaskRunner::start_task(
  const std::string & task_name,
  Action * action_out,
  std::string * error_out)
{
  if (action_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Task action output pointer is null.";
    }
    return false;
  }

  const auto task_it = task_configs_.find(task_name);
  if (task_it == task_configs_.end()) {
    if (error_out != nullptr) {
      *error_out = "Unknown task: " + task_name;
    }
    return false;
  }

  active_task_name_ = task_name;
  active_task_config_ = task_it->second;
  state_ = State::Waiting;
  state_elapsed_sec_ = 0.0;

  action_out->type = ActionType::PublishMotionPlan;
  action_out->pos_preset_name = active_task_config_.pos_preset_name;
  action_out->use_current_position = active_task_config_.use_current_position;
  action_out->impedance_level = active_task_config_.impedance_level;
  action_out->grasp_plan.reset();
  return true;
}

void PlatoGraspTaskRunner::cancel_task()
{
  state_ = State::Idle;
  active_task_name_.clear();
  active_task_config_ = GraspTaskConfig{};
  state_elapsed_sec_ = 0.0;
}

bool PlatoGraspTaskRunner::update(
  double dt_sec,
  Action * action_out,
  std::string * error_out)
{
  if (action_out == nullptr) {
    if (error_out != nullptr) {
      *error_out = "Task action output pointer is null.";
    }
    return false;
  }

  *action_out = Action{};
  if (state_ == State::Idle) {
    return true;
  }

  state_elapsed_sec_ += std::max(0.0, dt_sec);

  switch (state_) {
    case State::Idle:
      return true;
    case State::Waiting:
      if (state_elapsed_sec_ < active_task_config_.wait_sec) {
        return true;
      }
      state_ = State::Grasping;
      state_elapsed_sec_ = 0.0;
      action_out->type = ActionType::PublishGraspPlan;
      action_out->pos_preset_name = active_task_config_.pos_preset_name;
      action_out->use_current_position = active_task_config_.use_current_position;
      action_out->impedance_level = active_task_config_.impedance_level;
      action_out->grasp_plan = active_task_config_.grasp_plan;
      return true;
    case State::Grasping:
      if (active_task_config_.grasp_duration_sec <= 0.0) {
        return true;
      }
      if (state_elapsed_sec_ < active_task_config_.grasp_duration_sec) {
        return true;
      }
      action_out->type = ActionType::PublishMotionHold;
      action_out->pos_preset_name = active_task_config_.pos_preset_name;
      action_out->use_current_position = active_task_config_.use_current_position;
      action_out->impedance_level = active_task_config_.impedance_level;
      action_out->grasp_plan.reset();
      cancel_task();
      return true;
  }

  return true;
}

bool PlatoGraspTaskRunner::is_active() const
{
  return state_ != State::Idle;
}

PlatoGraspTaskRunner::State PlatoGraspTaskRunner::state() const
{
  return state_;
}

const std::string & PlatoGraspTaskRunner::active_task_name() const
{
  return active_task_name_;
}

}  // namespace plato_grasp_controller
