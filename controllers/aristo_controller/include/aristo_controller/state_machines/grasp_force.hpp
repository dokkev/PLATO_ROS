#ifndef ARISTO_CONTROLLER__STATE_MACHINES__GRASP_FORCE_HPP_
#define ARISTO_CONTROLLER__STATE_MACHINES__GRASP_FORCE_HPP_

#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/task/grasp_task.hpp"

namespace aristo_controller::state_machines
{

struct GraspForceInput
{
  double u{0.0};
  double phi{0.0};
  double desired_force_n{1.0};
};

struct GraspForceStateConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double default_u{0.0};
  double default_phi{0.0};
  double default_desired_force_n{1.0};

  plato_robot_system::task::GraspTaskConfig grasp_task;
};

class GraspForceState final : public plato_robot_system::State
{
public:
  static constexpr const char * kName = "grasp_force";

  GraspForceState(
    plato_robot_system::StateId id,
    plato_robot_system::RobotSystem * robot);

  bool ConfigureTask(const GraspForceStateConfig & config);
  void SetInput(const GraspForceInput & input);

  void OnEnter() override;
  void OnExit() override;
  bool PopulateCommand(plato_robot_system::RobotCommand * command) const override;

  plato_robot_system::task::GraspTaskMode mode() const { return grasp_task_.mode(); }
  const plato_robot_system::task::GraspTaskStatus & status() const
  {
    return grasp_task_.status();
  }
  const GraspForceInput & input() const { return input_; }
  double default_desired_force_n() const { return config_.default_desired_force_n; }
  bool task_entered() const { return task_entered_; }

private:
  plato_robot_system::RobotSystem * robot_{nullptr};
  mutable plato_robot_system::task::GraspTask grasp_task_;
  GraspForceStateConfig config_;
  GraspForceInput input_;
  bool task_configured_{false};
  mutable bool task_entered_{false};
};

}  // namespace aristo_controller::state_machines

#endif  // ARISTO_CONTROLLER__STATE_MACHINES__GRASP_FORCE_HPP_
