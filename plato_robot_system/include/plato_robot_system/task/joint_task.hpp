#ifndef PLATO_ROBOT_SYSTEM__TASK__JOINT_TASK_HPP_
#define PLATO_ROBOT_SYSTEM__TASK__JOINT_TASK_HPP_

#include <Eigen/Core>

#include <vector>

#include "plato_robot_system/control/pid_controller.hpp"
#include "plato_robot_system/robot/robot_system.hpp"
#include "plato_robot_system/trajectory/interpolation.hpp"

namespace plato_robot_system::task
{

class JointTask
{
public:
  void SetFeedbackGains(
    const Eigen::Ref<const Eigen::VectorXd> & kp,
    const Eigen::Ref<const Eigen::VectorXd> & kd);

  bool StartMinJerk(
    const RobotState & state,
    const Eigen::Ref<const Eigen::VectorXd> & target_jpos,
    double duration_sec);

  void Reset();
  bool BuildCommand(
    const RobotState & state,
    double elapsed_time_sec,
    double dt_sec,
    RobotCommand * command) const;

  const RobotCommand & command() const { return command_; }
  bool active() const { return trajectory_initialized_; }

private:
  void ConfigurePidControllers();
  bool HasCompatibleState(const RobotState & state) const;

  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  mutable bool trajectory_initialized_{false};
  mutable trajectory::MinJerkCurveVec trajectory_;
  mutable std::vector<PIDController> pid_controllers_;
  mutable RobotCommand command_;
};

}  // namespace plato_robot_system::task

#endif  // PLATO_ROBOT_SYSTEM__TASK__JOINT_TASK_HPP_
