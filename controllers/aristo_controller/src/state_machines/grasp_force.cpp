#include "aristo_controller/state_machines/grasp_force.hpp"

#include <algorithm>
#include <cmath>

namespace aristo_controller::state_machines
{
namespace
{

bool IsFinite(const double value)
{
  return std::isfinite(value);
}

}  // namespace

GraspForceState::GraspForceState(
  const plato_robot_system::StateId id,
  plato_robot_system::RobotSystem * robot)
: plato_robot_system::State(id, kName),
  robot_(robot)
{
}

bool GraspForceState::ConfigureTask(const GraspForceStateConfig & config)
{
  if (robot_ == nullptr || !robot_->hasModel()) {
    task_configured_ = false;
    return false;
  }
  if (
    !IsFinite(config.default_u) || !IsFinite(config.default_phi) ||
    !IsFinite(config.default_desired_force_n))
  {
    task_configured_ = false;
    return false;
  }

  if (!grasp_task_.Configure(robot_->model(), config.grasp_task)) {
    task_configured_ = false;
    task_entered_ = false;
    return false;
  }

  config_ = config;
  input_.u = std::clamp(config_.default_u, 0.0, 1.0);
  input_.phi = std::clamp(config_.default_phi, 0.0, 1.0);
  input_.desired_force_n = std::max(0.0, config_.default_desired_force_n);
  task_configured_ = true;
  task_entered_ = false;
  return task_configured_;
}

void GraspForceState::SetInput(const GraspForceInput & input)
{
  if (IsFinite(input.u)) {
    input_.u = std::clamp(input.u, 0.0, 1.0);
  }
  if (IsFinite(input.phi)) {
    input_.phi = std::clamp(input.phi, 0.0, 1.0);
  }
  if (IsFinite(input.desired_force_n)) {
    input_.desired_force_n = std::max(0.0, input.desired_force_n);
  }
}

void GraspForceState::OnEnter()
{
  task_entered_ = false;
  if (!task_configured_ || robot_ == nullptr || !robot_->hasState()) {
    return;
  }
  input_.u = std::clamp(config_.default_u, 0.0, 1.0);
  input_.phi = std::clamp(config_.default_phi, 0.0, 1.0);
  input_.desired_force_n = std::max(0.0, config_.default_desired_force_n);
  task_entered_ = grasp_task_.OnEnter(*robot_, robot_->state());
}

void GraspForceState::OnExit()
{
  grasp_task_.Reset();
  task_entered_ = false;
}

bool GraspForceState::PopulateCommand(plato_robot_system::RobotCommand * command) const
{
  if (!task_configured_ || robot_ == nullptr || !robot_->hasState() || command == nullptr) {
    return false;
  }
  if (!task_entered_) {
    task_entered_ = grasp_task_.OnEnter(*robot_, robot_->state());
    if (!task_entered_) {
      return false;
    }
  }

  const auto & state = robot_->state();
  const GraspForceInput input = input_;

  plato_robot_system::task::GraspTaskCommand task_command;
  task_command.u = input.u;
  task_command.phi = input.phi;
  task_command.desired_force_n = input.desired_force_n;
  return grasp_task_.PopulateCommand(
    *robot_,
    state,
    task_command,
    dt(),
    command);
}

}  // namespace aristo_controller::state_machines
