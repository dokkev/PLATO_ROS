#ifndef ARISTO_CONTROLLER__STATE_MACHINES__GRASP_TELEOP_HPP_
#define ARISTO_CONTROLLER__STATE_MACHINES__GRASP_TELEOP_HPP_

#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/task/grasp_task.hpp"

namespace aristo_controller::state_machines
{

struct GraspTeleopInput
{
  double u{0.0};
  double phi{0.0};
  double desired_force_n{1.0};
};

struct GraspTeleopStateConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double default_u{0.5};
  double default_phi{0.0};
  double default_desired_force_n{1.0};
  bool shared_grasp_control{false};
  int shared_control_min_contact_sensors{2};
  int shared_control_enter_debounce_ticks{3};
  bool shared_control_requires_u_below_threshold{true};
  bool shared_control_requires_enough_contact{true};

  plato_robot_system::task::GraspTaskConfig grasp_task;
};

class GraspTeleopState final : public plato_robot_system::State
{
public:
  static constexpr const char * kName = "grasp_teleop";

  explicit GraspTeleopState(plato_robot_system::StateId id);
  GraspTeleopState(
    plato_robot_system::StateId id,
    plato_robot_system::RobotSystem * robot);

  bool ConfigureTask(
    const GraspTeleopStateConfig & config);
  void SetInput(const GraspTeleopInput & input);

  void OnEnter() override;
  void OnExit() override;
  bool IsFinished() const override;

  bool PopulateCommand(plato_robot_system::RobotCommand * command) const override;

  plato_robot_system::task::GraspTaskMode mode() const { return grasp_task_.mode(); }
  const plato_robot_system::task::GraspTaskStatus & status() const
  {
    return grasp_task_.status();
  }
  const GraspTeleopInput & input() const { return input_; }
  double default_desired_force_n() const { return config_.default_desired_force_n; }
  bool task_entered() const { return task_entered_; }
  bool force_handoff_requested() const { return force_handoff_requested_; }
  bool shared_control_handoff_requested() const { return force_handoff_requested_; }

private:
  void UpdateSharedControlHandoff(const GraspTeleopInput & input) const;

  plato_robot_system::RobotSystem * robot_{nullptr};
  mutable plato_robot_system::task::GraspTask grasp_task_;
  GraspTeleopStateConfig config_;
  GraspTeleopInput input_;
  bool task_configured_{false};
  mutable bool task_entered_{false};
  mutable bool force_handoff_requested_{false};
  mutable int shared_control_enter_counter_{0};
};

}  // namespace aristo_controller::state_machines

#endif  // ARISTO_CONTROLLER__STATE_MACHINES__GRASP_TELEOP_HPP_
