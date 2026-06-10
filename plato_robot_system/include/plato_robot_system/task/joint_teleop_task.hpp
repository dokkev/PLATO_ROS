#ifndef PLATO_ROBOT_SYSTEM__TASK__JOINT_TELEOP_TASK_HPP_
#define PLATO_ROBOT_SYSTEM__TASK__JOINT_TELEOP_TASK_HPP_

#include <Eigen/Core>

#include "plato_robot_system/robot/robot_system.hpp"

namespace plato_robot_system::task
{

struct JointTeleopTaskConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Host-side task feedback gains. Driver-local gains are attached later by
  // ControlArchitecture::FinalizeCommand().
  Eigen::VectorXd kp_task;
  Eigen::VectorXd kd_task;

  // First-order low-pass filter coefficient for teleop q_cmd smoothing.
  // 0 freezes the command, and 1 applies the target immediately.
  double lpf_alpha{0.1};
};

class JointTeleopTask
{
public:
  bool Configure(const JointTeleopTaskConfig & config, int nq, int nv);

  void Reset();
  bool OnEnter(const RobotState & state);
  bool SetTargetPosition(const Eigen::Ref<const Eigen::VectorXd> & target_q);

  bool PopulateCommand(
    const RobotState & state,
    double dt_sec,
    RobotCommand * command) const;

  const RobotCommand & command() const { return command_; }
  bool configured() const { return configured_; }
  bool entered() const { return filter_initialized_; }

private:
  bool HasCompatibleState(const RobotState & state) const;

  JointTeleopTaskConfig config_;
  bool configured_{false};
  int nq_{0};
  int nv_{0};

  Eigen::VectorXd target_q_;
  bool has_target_{false};

  mutable bool filter_initialized_{false};
  mutable Eigen::VectorXd filtered_q_;
  mutable RobotCommand command_;
};

}  // namespace plato_robot_system::task

#endif  // PLATO_ROBOT_SYSTEM__TASK__JOINT_TELEOP_TASK_HPP_
