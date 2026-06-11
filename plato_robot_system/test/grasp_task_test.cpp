#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <string_view>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include "plato_robot_system/robot/robot_system.hpp"
#include "plato_robot_system/task/grasp_task.hpp"
#include "plato_robot_system/task/thumb_index_grasp_constants.hpp"

namespace
{

constexpr double kDtSec = 0.02;
constexpr double kTolerance = 1.0e-9;

std::filesystem::path AristoUrdfPath()
{
  return std::filesystem::path(PLATO_ROBOT_SYSTEM_ARISTO_URDF_PATH);
}

plato_robot_system::RobotSystem MakeNeutralAristoRobot()
{
  plato_robot_system::RobotSystem robot(AristoUrdfPath().string());
  const auto & model = robot.model();
  const Eigen::VectorXd q = pinocchio::neutral(model);
  const Eigen::VectorXd qdot = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  robot.UpdateState(q, qdot, tau, 0.0);
  robot.UpdateKinematics();
  return robot;
}

int JointPositionIndex(
  const pinocchio::Model & model,
  const std::string & joint_name)
{
  const auto joint_id = model.getJointId(joint_name);
  EXPECT_LT(joint_id, static_cast<pinocchio::JointIndex>(model.njoints));
  EXPECT_EQ(model.nqs[joint_id], 1);
  return model.idx_qs[joint_id];
}

void SetJointPosition(
  const pinocchio::Model & model,
  const std::string & joint_name,
  const double value,
  Eigen::VectorXd * q)
{
  ASSERT_NE(q, nullptr);
  (*q)[JointPositionIndex(model, joint_name)] = value;
}

Eigen::VectorXd GraspReadyQ(const pinocchio::Model & model)
{
  Eigen::VectorXd q = pinocchio::neutral(model);
  SetJointPosition(model, "joint1", 0.0, &q);
  SetJointPosition(model, "joint2", 0.0, &q);
  SetJointPosition(model, "joint3", 0.0, &q);
  SetJointPosition(model, "joint4", 0.0, &q);
  SetJointPosition(model, "joint5", 0.8849433621761859, &q);
  SetJointPosition(model, "joint6", -0.8849433621761859, &q);
  SetJointPosition(model, "joint7", 0.785, &q);
  SetJointPosition(model, "joint8", 1.5708, &q);
  return q;
}

plato_robot_system::sensor::TactileState MakeTactileState(
  const std::string_view frame_name,
  const double normal_force_n,
  const int contact_state = plato_robot_system::sensor::TactileState::kEnoughContacts)
{
  plato_robot_system::sensor::TactileState tactile;
  tactile.valid = true;
  tactile.frame_name = std::string(frame_name);
  tactile.contact_state = contact_state;
  tactile.total_force_n = Eigen::Vector3d{0.0, 0.0, normal_force_n};
  return tactile;
}

plato_robot_system::RobotState MakeStateWithTactile(
  plato_robot_system::RobotSystem * robot,
  const double index_force_n,
  const double thumb_force_n,
  const int contact_state = plato_robot_system::sensor::TactileState::kEnoughContacts)
{
  auto state = robot->state();
  state.tactile_sensors.clear();
  state.tactile_sensors.push_back(
    MakeTactileState(
      plato_robot_system::task::kThumbIndexFrameA, index_force_n, contact_state));
  state.tactile_sensors.push_back(
    MakeTactileState(
      plato_robot_system::task::kThumbIndexFrameB, thumb_force_n, contact_state));
  robot->UpdateState(state);
  robot->UpdateKinematics();
  return robot->state();
}

plato_robot_system::task::GraspTaskConfig MakeTaskConfig(
  const pinocchio::Model & model)
{
  plato_robot_system::task::GraspTaskConfig config;
  config.q_ready = GraspReadyQ(model);
  config.force_enter_debounce_ticks = 1;
  config.force_exit_contact_lost_ticks = 1;
  config.force_exit_u_threshold = 0.75;
  config.min_contact_force_n = 0.05;
  config.use_tactile_presence_for_contact = true;
  config.kp_tactile_u_fb = 0.1;
  config.kd_tactile_u_fb = 0.0;
  config.kp_tactile_phi_fb = 0.0;
  config.kd_tactile_phi_fb = 0.0;
  return config;
}

double ParallelQ5Geometry(
  const plato_robot_system::task::GraspTaskConfig & config,
  const double q3)
{
  const double cos_q5 = std::clamp(
    std::cos(q3) - config.parallel_lateral_offset_m / config.parallel_tip_radius_m,
    -1.0,
    1.0);
  return std::max(config.parallel_q5_min_rad, std::acos(cos_q5));
}

void ExpectValidPositionCommand(
  const plato_robot_system::RobotSystem & robot,
  const plato_robot_system::RobotCommand & command)
{
  ASSERT_TRUE(command.IsUsable());
  EXPECT_EQ(command.q_cmd.size(), robot.nq());
  EXPECT_EQ(command.qdot_cmd.size(), robot.nv());
  EXPECT_EQ(command.tau_cmd.size(), robot.nv());
  EXPECT_TRUE(command.q_cmd.allFinite());
  EXPECT_TRUE(command.qdot_cmd.isZero(kTolerance));
  EXPECT_TRUE(command.tau_cmd.isZero(kTolerance));
  EXPECT_TRUE(command.kp.isZero(kTolerance));
  EXPECT_TRUE(command.kd.isZero(kTolerance));
}

}  // namespace

TEST(GraspTaskTest, ReadyCommandUsesParallelControllerGeometry)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  auto config = MakeTaskConfig(robot.model());
  robot.UpdateState(
    config.q_ready,
    Eigen::VectorXd::Zero(robot.nv()),
    Eigen::VectorXd::Zero(robot.nv()),
    0.0);
  robot.UpdateKinematics();

  plato_robot_system::task::GraspTask task;
  ASSERT_TRUE(task.Configure(robot.model(), config));
  ASSERT_TRUE(task.OnEnter(robot, robot.state()));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 0.5;
  input.phi = 0.0;
  input.desired_force_n = 1.0;

  plato_robot_system::RobotCommand command;
  ASSERT_TRUE(task.PopulateCommand(robot, robot.state(), input, kDtSec, &command));
  ExpectValidPositionCommand(robot, command);

  const auto & model = robot.model();
  const int joint3_q = JointPositionIndex(model, "joint3");
  const int joint4_q = JointPositionIndex(model, "joint4");
  const int joint5_q = JointPositionIndex(model, "joint5");
  const int joint6_q = JointPositionIndex(model, "joint6");

  const double q5_ready = ParallelQ5Geometry(config, 0.0);
  EXPECT_NEAR(command.q_cmd[joint3_q], 0.0, 1.0e-12);
  EXPECT_NEAR(command.q_cmd[joint4_q], 0.0, 1.0e-12);
  EXPECT_NEAR(command.q_cmd[joint5_q], q5_ready, 1.0e-12);
  EXPECT_NEAR(command.q_cmd[joint6_q], -q5_ready, 1.0e-12);
  EXPECT_NEAR(task.status().effective_u, 0.5, 1.0e-12);
  EXPECT_NEAR(task.status().u_parallel, config.parallel_midpoint_u, 1.0e-12);
}

TEST(GraspTaskTest, ClosedFlexedCommandUsesParallelControllerGeometry)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  auto config = MakeTaskConfig(robot.model());

  plato_robot_system::task::GraspTask task;
  ASSERT_TRUE(task.Configure(robot.model(), config));
  ASSERT_TRUE(task.OnEnter(robot, robot.state()));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 0.0;
  input.phi = 1.0;
  input.desired_force_n = 1.0;

  plato_robot_system::RobotCommand command;
  ASSERT_TRUE(task.PopulateCommand(robot, robot.state(), input, kDtSec, &command));
  ExpectValidPositionCommand(robot, command);

  const auto & model = robot.model();
  const int joint3_q = JointPositionIndex(model, "joint3");
  const int joint4_q = JointPositionIndex(model, "joint4");
  const int joint5_q = JointPositionIndex(model, "joint5");
  const int joint6_q = JointPositionIndex(model, "joint6");
  const double q3_closed = config.parallel_qmin_rad;
  const double q5_closed = ParallelQ5Geometry(config, q3_closed);

  EXPECT_NEAR(command.q_cmd[joint3_q], q3_closed, 1.0e-12);
  EXPECT_NEAR(
    command.q_cmd[joint4_q],
    -q3_closed - config.parallel_max_flexion_rad,
    1.0e-12);
  EXPECT_NEAR(command.q_cmd[joint5_q], q5_closed, 1.0e-12);
  EXPECT_NEAR(
    command.q_cmd[joint6_q],
    -q5_closed + config.parallel_max_flexion_rad,
    1.0e-12);
}

TEST(GraspTaskTest, LpfAlphaSmoothsPositionCommand)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  auto raw_config = MakeTaskConfig(robot.model());
  auto smoothed_config = raw_config;
  smoothed_config.lpf_alpha = 0.5;

  plato_robot_system::task::GraspTask raw_task;
  ASSERT_TRUE(raw_task.Configure(robot.model(), raw_config));
  ASSERT_TRUE(raw_task.OnEnter(robot, robot.state()));

  plato_robot_system::task::GraspTask smoothed_task;
  ASSERT_TRUE(smoothed_task.Configure(robot.model(), smoothed_config));
  ASSERT_TRUE(smoothed_task.OnEnter(robot, robot.state()));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 0.0;
  input.phi = 1.0;
  input.desired_force_n = 1.0;

  plato_robot_system::RobotCommand raw_command;
  ASSERT_TRUE(raw_task.PopulateCommand(robot, robot.state(), input, kDtSec, &raw_command));
  ExpectValidPositionCommand(robot, raw_command);

  plato_robot_system::RobotCommand smoothed_command;
  ASSERT_TRUE(
    smoothed_task.PopulateCommand(robot, robot.state(), input, kDtSec, &smoothed_command));
  ExpectValidPositionCommand(robot, smoothed_command);

  const Eigen::VectorXd expected =
    robot.state().q + smoothed_config.lpf_alpha * (raw_command.q_cmd - robot.state().q);
  EXPECT_TRUE(smoothed_command.q_cmd.isApprox(expected, 1.0e-12));
  EXPECT_TRUE(smoothed_task.status().q_target.isApprox(expected, 1.0e-12));
}

TEST(GraspTaskTest, RejectsWrongSizedQReady)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  auto config = MakeTaskConfig(robot.model());
  config.q_ready = Eigen::VectorXd::Zero(robot.nq() - 1);

  plato_robot_system::task::GraspTask task;
  EXPECT_FALSE(task.Configure(robot.model(), config));
}

TEST(GraspTaskTest, RejectsInvalidParallelGeometry)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  auto config = MakeTaskConfig(robot.model());
  config.parallel_lateral_offset_m = 3.0 * config.parallel_tip_radius_m;

  plato_robot_system::task::GraspTask task;
  EXPECT_FALSE(task.Configure(robot.model(), config));
}

TEST(GraspTaskTest, EnoughContactEnablesForceTrackingAndOffsetsCloseCommand)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  const auto state = MakeStateWithTactile(&robot, 0.2, 0.25);
  auto config = MakeTaskConfig(robot.model());
  config.kp_tactile_phi_fb = 0.1;

  plato_robot_system::task::GraspTask task;
  ASSERT_TRUE(task.Configure(robot.model(), config));
  ASSERT_TRUE(task.OnEnter(robot, state));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 0.2;
  input.phi = 0.2;
  input.desired_force_n = 1.0;

  plato_robot_system::RobotCommand command;
  ASSERT_TRUE(task.PopulateCommand(robot, state, input, kDtSec, &command));
  ExpectValidPositionCommand(robot, command);
  EXPECT_EQ(task.mode(), plato_robot_system::task::GraspTaskMode::kForceTracking);
  EXPECT_GT(task.status().force_error_n, 0.0);
  EXPECT_LT(task.status().effective_u, input.u);
  EXPECT_GT(task.status().effective_phi, input.phi);
}

TEST(GraspTaskTest, InvalidTactileDuringForceTrackingFallsBackToMotionTeleop)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  const auto contact_state = MakeStateWithTactile(&robot, 0.2, 0.25);
  auto config = MakeTaskConfig(robot.model());

  plato_robot_system::task::GraspTask task;
  ASSERT_TRUE(task.Configure(robot.model(), config));
  ASSERT_TRUE(task.OnEnter(robot, contact_state));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 0.2;
  input.phi = 0.0;
  input.desired_force_n = 1.0;

  plato_robot_system::RobotCommand command;
  ASSERT_TRUE(task.PopulateCommand(robot, contact_state, input, kDtSec, &command));
  ASSERT_EQ(task.mode(), plato_robot_system::task::GraspTaskMode::kForceTracking);

  auto missing_tactile_state = robot.state();
  missing_tactile_state.tactile_sensors.clear();
  missing_tactile_state.time_s += kDtSec;
  robot.UpdateState(missing_tactile_state);
  robot.UpdateKinematics();

  ASSERT_TRUE(task.PopulateCommand(robot, robot.state(), input, kDtSec, &command));
  EXPECT_EQ(task.mode(), plato_robot_system::task::GraspTaskMode::kMotionTeleop);
  EXPECT_DOUBLE_EQ(task.status().force_error_n, 0.0);
}

TEST(GraspTaskTest, OpeningCommandExitsForceTracking)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  const auto state = MakeStateWithTactile(&robot, 0.2, 0.25);
  auto config = MakeTaskConfig(robot.model());

  plato_robot_system::task::GraspTask task;
  ASSERT_TRUE(task.Configure(robot.model(), config));
  ASSERT_TRUE(task.OnEnter(robot, state));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 0.2;
  input.phi = 0.0;
  input.desired_force_n = 1.0;

  plato_robot_system::RobotCommand command;
  ASSERT_TRUE(task.PopulateCommand(robot, state, input, kDtSec, &command));
  ASSERT_EQ(task.mode(), plato_robot_system::task::GraspTaskMode::kForceTracking);

  input.u = 1.0;
  ASSERT_TRUE(task.PopulateCommand(robot, state, input, kDtSec, &command));
  EXPECT_EQ(task.mode(), plato_robot_system::task::GraspTaskMode::kMotionTeleop);
}
