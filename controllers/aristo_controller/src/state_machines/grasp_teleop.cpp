#include "aristo_controller/state_machines/grasp_teleop.hpp"

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

GraspTeleopState::GraspTeleopState(const plato_robot_system::StateId id)
: plato_robot_system::State(id, kName)
{
}

GraspTeleopState::GraspTeleopState(
  const plato_robot_system::StateId id,
  plato_robot_system::RobotSystem * robot)
: plato_robot_system::State(id, kName),
  robot_(robot)
{
}

bool GraspTeleopState::ConfigureTask(
  const GraspTeleopStateConfig & config)
{
  if (robot_ == nullptr || !robot_->hasModel()) {
    task_configured_ = false;
    return false;
  }
  if (
    !IsFinite(config.default_u) || !IsFinite(config.default_phi) ||
    !IsFinite(config.default_desired_force_n) ||
    config.shared_control_min_contact_sensors < 1 ||
    config.shared_control_min_contact_sensors > 2 ||
    config.shared_control_enter_debounce_ticks < 0)
  {
    task_configured_ = false;
    return false;
  }

  auto task_config = config.grasp_task;
  task_config.force_feedback_enabled = false;

  if (!grasp_task_.Configure(robot_->model(), task_config)) {
    task_configured_ = false;
    task_entered_ = false;
    return false;
  }

  config_ = config;
  config_.grasp_task = task_config;
  input_.u = std::clamp(config_.default_u, 0.0, 1.0);
  input_.phi = std::clamp(config_.default_phi, 0.0, 1.0);
  input_.desired_force_n = std::max(0.0, config_.default_desired_force_n);
  task_configured_ = true;
  task_entered_ = false;
  force_handoff_requested_ = false;
  shared_control_enter_counter_ = 0;
  return task_configured_;
}

void GraspTeleopState::SetInput(const GraspTeleopInput & input)
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

void GraspTeleopState::OnEnter()
{
  task_entered_ = false;
  force_handoff_requested_ = false;
  shared_control_enter_counter_ = 0;
  if (!task_configured_ || robot_ == nullptr || !robot_->hasState()) {
    return;
  }
  input_.u = std::clamp(config_.default_u, 0.0, 1.0);
  input_.phi = std::clamp(config_.default_phi, 0.0, 1.0);
  input_.desired_force_n = std::max(0.0, config_.default_desired_force_n);
  task_entered_ = grasp_task_.OnEnter(*robot_, robot_->state());
}

void GraspTeleopState::OnExit()
{
  force_handoff_requested_ = false;
  shared_control_enter_counter_ = 0;
  task_entered_ = false;
}

bool GraspTeleopState::IsFinished() const
{
  return (
    force_handoff_requested_ && lifecycle_.next_state_id >= 0) ||
    plato_robot_system::State::IsFinished();
}

bool GraspTeleopState::PopulateCommand(plato_robot_system::RobotCommand * command) const
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
  const GraspTeleopInput input = input_;

  plato_robot_system::task::GraspTaskCommand task_command;
  task_command.u = input.u;
  task_command.phi = input.phi;
  task_command.desired_force_n = input.desired_force_n;
  const bool populated = grasp_task_.PopulateCommand(
    *robot_,
    state,
    task_command,
    dt(),
    command);
  if (populated) {
    UpdateSharedControlHandoff(input);
  }
  return populated;
}

void GraspTeleopState::UpdateSharedControlHandoff(const GraspTeleopInput & input) const
{
  if (!config_.shared_grasp_control || lifecycle_.next_state_id < 0) {
    shared_control_enter_counter_ = 0;
    return;
  }

  const auto & status = grasp_task_.status();
  const bool command_allows_handoff =
    !config_.shared_control_requires_u_below_threshold ||
    input.u <= config_.grasp_task.force_exit_u_threshold;
  const int contact_count = config_.shared_control_requires_enough_contact ?
    status.enough_contact_count :
    status.contact_count;
  const bool contact_ready =
    contact_count >= config_.shared_control_min_contact_sensors;

  if (command_allows_handoff && contact_ready) {
    ++shared_control_enter_counter_;
  } else {
    shared_control_enter_counter_ = 0;
  }

  const int required_ticks = std::max(1, config_.shared_control_enter_debounce_ticks);
  if (shared_control_enter_counter_ >= required_ticks) {
    force_handoff_requested_ = true;
  }
}

}  // namespace aristo_controller::state_machines
