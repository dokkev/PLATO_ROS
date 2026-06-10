#include <gtest/gtest.h>

#include <limits>

#include "plato_robot_system/task/joint_teleop_task.hpp"

namespace
{

plato_robot_system::RobotState MakeState(
  const Eigen::Vector2d & q,
  const Eigen::Vector2d & qdot)
{
  Eigen::VectorXd q_dyn = q;
  Eigen::VectorXd qdot_dyn = qdot;
  return plato_robot_system::MakeRobotState(
    q_dyn,
    qdot_dyn,
    Eigen::VectorXd::Zero(2),
    1.0);
}

}  // namespace

TEST(JointTeleopTaskTest, LowPassFiltersTargetAndComputesTaskFeedbackTorque)
{
  plato_robot_system::task::JointTeleopTaskConfig config;
  config.kp_task.resize(2);
  config.kp_task << 10.0, 20.0;
  config.kd_task.resize(2);
  config.kd_task << 1.0, 2.0;
  config.lpf_alpha = 0.5;

  plato_robot_system::task::JointTeleopTask task;
  ASSERT_TRUE(task.Configure(config, 2, 2));

  const auto state = MakeState(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());
  ASSERT_TRUE(task.OnEnter(state));
  Eigen::Vector2d target;
  target << 1.0, -1.0;
  ASSERT_TRUE(task.SetTargetPosition(target));

  plato_robot_system::RobotCommand command;
  ASSERT_TRUE(task.PopulateCommand(state, 0.1, &command));

  EXPECT_TRUE(command.valid);
  Eigen::Vector2d expected_q;
  expected_q << 0.5, -0.5;
  Eigen::Vector2d expected_qdot;
  expected_qdot << 5.0, -5.0;
  Eigen::Vector2d expected_tau;
  expected_tau << 10.0, -20.0;
  EXPECT_TRUE(command.q_cmd.isApprox(expected_q));
  EXPECT_TRUE(command.qdot_cmd.isApprox(expected_qdot));
  EXPECT_TRUE(command.tau_cmd.isApprox(expected_tau));
  EXPECT_TRUE(command.kp.isZero());
  EXPECT_TRUE(command.kd.isZero());
}

TEST(JointTeleopTaskTest, RejectsInvalidTarget)
{
  plato_robot_system::task::JointTeleopTaskConfig config;
  config.kp_task = Eigen::Vector2d::Zero();
  config.kd_task = Eigen::Vector2d::Zero();

  plato_robot_system::task::JointTeleopTask task;
  ASSERT_TRUE(task.Configure(config, 2, 2));
  ASSERT_TRUE(task.OnEnter(MakeState(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero())));

  Eigen::VectorXd wrong_size = Eigen::VectorXd::Zero(3);
  EXPECT_FALSE(task.SetTargetPosition(wrong_size));

  Eigen::VectorXd invalid_target(2);
  invalid_target << 0.0, std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(task.SetTargetPosition(invalid_target));
}
