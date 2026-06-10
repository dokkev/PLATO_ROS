#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include "plato_robot_system/robot/robot_system.hpp"
#include "plato_robot_system/task/grasp_task.hpp"
#include "plato_robot_system/task/thumb_index_grasp_constants.hpp"

namespace
{

constexpr double kDtSec = 0.02;
constexpr double kMaxVelocityRadS = 2.0;
constexpr double kMaxTorqueNm = 10.0;
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

Eigen::Vector3d ContactVector(plato_robot_system::RobotSystem * robot)
{
  const Eigen::Vector3d p_a =
    robot->FramePoseWorld(std::string(plato_robot_system::task::kThumbIndexFrameA))
    .translation();
  const Eigen::Vector3d p_b =
    robot->FramePoseWorld(std::string(plato_robot_system::task::kThumbIndexFrameB))
    .translation();
  return p_b - p_a;
}

double ContactAxisDistance(
  plato_robot_system::RobotSystem * robot,
  const Eigen::Vector3d & axis_world)
{
  return axis_world.dot(ContactVector(robot));
}

std::vector<int> ActiveVelocityIndices(const pinocchio::Model & model)
{
  std::vector<int> indices;
  indices.reserve(plato_robot_system::task::kThumbIndexActiveJoints.size());
  for (const auto joint_name : plato_robot_system::task::kThumbIndexActiveJoints) {
    const auto joint_id = model.getJointId(std::string(joint_name));
    indices.push_back(model.idx_vs[joint_id]);
  }
  return indices;
}

std::vector<int> ActivePositionIndices(const pinocchio::Model & model)
{
  std::vector<int> indices;
  indices.reserve(plato_robot_system::task::kThumbIndexActiveJoints.size());
  for (const auto joint_name : plato_robot_system::task::kThumbIndexActiveJoints) {
    const auto joint_id = model.getJointId(std::string(joint_name));
    indices.push_back(model.idx_qs[joint_id]);
  }
  return indices;
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
  const double current_distance_m)
{
  plato_robot_system::task::GraspTaskConfig config;
  config.distance_closed_m = std::max(0.001, current_distance_m - 0.02);
  config.distance_open_m = current_distance_m + 0.02;
  config.fallback_close_axis_base = Eigen::Vector3d{0.0, 0.0, -1.0};
  config.force_enter_debounce_ticks = 1;
  config.force_exit_contact_lost_ticks = 1;
  config.force_exit_u_threshold = 0.75;
  config.min_contact_force_n = 0.05;
  config.use_tactile_presence_for_contact = true;
  config.kp_task = 80.0;
  config.kd_task = 2.0;
  config.q_posture_phi0 = Eigen::VectorXd::Zero(4);
  config.q_posture_phi1 = Eigen::VectorXd::Zero(4);
  config.kp_tactile_fb = 0.1;
  config.kd_tactile_fb = 0.0;
  config.w_task_motion = 100.0;
  config.w_task_tactile_mode = 1.0;
  config.w_tactile = 100.0;
  config.w_posture = 0.01;
  config.damping_qp = 1.0e-6;
  config.max_qddot_rad_s2 = 80.0;
  config.max_velocity_rad_s = kMaxVelocityRadS;
  config.max_torque_nm = kMaxTorqueNm;
  config.max_torque_rate_nm_per_s = 1000.0;
  config.use_inverse_dynamics = true;
  config.joint_damping_nm_per_rad_s = 0.0;
  return config;
}

void ExpectSaneActiveOnlyCommand(
  const plato_robot_system::RobotSystem & robot,
  const plato_robot_system::RobotState & state,
  const plato_robot_system::RobotCommand & command)
{
  ASSERT_TRUE(command.IsUsable());
  EXPECT_EQ(command.q_cmd.size(), robot.nq());
  EXPECT_EQ(command.qdot_cmd.size(), robot.nv());
  EXPECT_EQ(command.tau_cmd.size(), robot.nv());
  EXPECT_EQ(command.kp.size(), robot.nv());
  EXPECT_EQ(command.kd.size(), robot.nv());
  EXPECT_TRUE(command.q_cmd.allFinite());
  EXPECT_TRUE(command.qdot_cmd.allFinite());
  EXPECT_TRUE(command.tau_cmd.allFinite());
  EXPECT_TRUE(command.kp.isZero(kTolerance));
  EXPECT_TRUE(command.kd.isZero(kTolerance));

  const auto active_q_indices = ActivePositionIndices(robot.model());
  const auto active_v_indices = ActiveVelocityIndices(robot.model());
  Eigen::VectorXd active_q_mask = Eigen::VectorXd::Zero(robot.nq());
  Eigen::VectorXd active_v_mask = Eigen::VectorXd::Zero(robot.nv());

  bool any_active_joint_moved = false;
  const auto & model = robot.model();
  for (std::size_t i = 0; i < active_q_indices.size(); ++i) {
    const int q_index = active_q_indices[i];
    const int v_index = active_v_indices[i];
    active_q_mask[q_index] = 1.0;
    active_v_mask[v_index] = 1.0;

    const double delta_q = command.q_cmd[q_index] - state.q[q_index];
    EXPECT_NEAR(command.qdot_cmd[v_index], delta_q / kDtSec, 1.0e-8);
    EXPECT_LE(std::abs(command.qdot_cmd[v_index]), kMaxVelocityRadS + 1.0e-8);
    EXPECT_LE(std::abs(command.tau_cmd[v_index]), kMaxTorqueNm + 1.0e-8);
    EXPECT_GE(command.q_cmd[q_index], model.lowerPositionLimit[q_index] - 1.0e-8);
    EXPECT_LE(command.q_cmd[q_index], model.upperPositionLimit[q_index] + 1.0e-8);
    any_active_joint_moved = any_active_joint_moved || std::abs(delta_q) > 1.0e-8;
  }
  EXPECT_TRUE(any_active_joint_moved);

  for (Eigen::Index i = 0; i < robot.nq(); ++i) {
    if (active_q_mask[i] == 0.0) {
      EXPECT_NEAR(command.q_cmd[i], state.q[i], kTolerance);
    }
  }
  for (Eigen::Index i = 0; i < robot.nv(); ++i) {
    if (active_v_mask[i] == 0.0) {
      EXPECT_NEAR(command.qdot_cmd[i], 0.0, kTolerance);
      EXPECT_NEAR(command.tau_cmd[i], 0.0, kTolerance);
    }
  }
}

}  // namespace

TEST(GraspTaskTest, OpeningInputProducesSaneCommandAndIncreasesContactDistance)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  const auto state = robot.state();
  const Eigen::Vector3d entry_contact_vector = ContactVector(&robot);
  ASSERT_GT(entry_contact_vector.norm(), 1.0e-6);
  const Eigen::Vector3d entry_axis = entry_contact_vector.normalized();
  const double entry_distance = ContactAxisDistance(&robot, entry_axis);

  plato_robot_system::task::GraspTask task;
  ASSERT_TRUE(task.Configure(robot.model(), MakeTaskConfig(entry_distance)));
  ASSERT_TRUE(task.OnEnter(robot, state));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 1.0;
  input.phi = 0.25;
  input.desired_force_n = 1.0;

  plato_robot_system::RobotCommand command;
  ASSERT_TRUE(task.PopulateCommand(robot, state, input, kDtSec, &command));
  EXPECT_EQ(task.mode(), plato_robot_system::task::GraspTaskMode::kMotionTeleop);
  ExpectSaneActiveOnlyCommand(robot, state, command);

  robot.UpdateState(
    command.q_cmd,
    Eigen::VectorXd::Zero(robot.nv()),
    Eigen::VectorXd::Zero(robot.nv()),
    state.time_s + kDtSec);
  robot.UpdateKinematics();
  EXPECT_GT(ContactAxisDistance(&robot, entry_axis), entry_distance + 1.0e-8);
}

TEST(GraspTaskTest, RejectsDegenerateFallbackCloseAxis)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  const double entry_distance = ContactVector(&robot).norm();
  auto config = MakeTaskConfig(entry_distance);
  config.fallback_close_axis_base.setZero();

  plato_robot_system::task::GraspTask task;
  EXPECT_FALSE(task.Configure(robot.model(), config));
}

TEST(GraspTaskTest, EnoughContactEnablesForceTrackingAndKeepsCommandSane)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  const Eigen::Vector3d entry_contact_vector = ContactVector(&robot);
  ASSERT_GT(entry_contact_vector.norm(), 1.0e-6);
  const double entry_distance = entry_contact_vector.norm();
  const auto state = MakeStateWithTactile(&robot, 0.2, 0.25);

  plato_robot_system::task::GraspTask task;
  ASSERT_TRUE(task.Configure(robot.model(), MakeTaskConfig(entry_distance)));
  ASSERT_TRUE(task.OnEnter(robot, state));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 0.2;
  input.phi = 0.5;
  input.desired_force_n = 1.0;

  plato_robot_system::RobotCommand command;
  ASSERT_TRUE(task.PopulateCommand(robot, state, input, kDtSec, &command));
  EXPECT_EQ(task.mode(), plato_robot_system::task::GraspTaskMode::kForceTracking);
  EXPECT_TRUE(task.status().qp_solved);
  EXPECT_GT(task.status().force_error_n, 0.0);
  ExpectSaneActiveOnlyCommand(robot, state, command);
}

TEST(GraspTaskTest, InvalidTactileDuringForceTrackingFallsBackToMotionTeleop)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  const double entry_distance = ContactVector(&robot).norm();
  const auto contact_state = MakeStateWithTactile(&robot, 0.2, 0.25);

  plato_robot_system::task::GraspTaskConfig config = MakeTaskConfig(entry_distance);
  config.kd_tactile_fb = 1.0;

  plato_robot_system::task::GraspTask task;
  ASSERT_TRUE(task.Configure(robot.model(), config));
  ASSERT_TRUE(task.OnEnter(robot, contact_state));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 0.2;
  input.phi = 0.5;
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
  EXPECT_TRUE(task.status().qp_solved);
  EXPECT_DOUBLE_EQ(task.status().force_error_n, 0.0);
  ExpectSaneActiveOnlyCommand(robot, robot.state(), command);
}

TEST(GraspTaskTest, OpeningCommandExitsForceTracking)
{
  ASSERT_TRUE(std::filesystem::exists(AristoUrdfPath())) << AristoUrdfPath();

  auto robot = MakeNeutralAristoRobot();
  const double entry_distance = ContactVector(&robot).norm();
  const auto state = MakeStateWithTactile(&robot, 0.2, 0.25);

  plato_robot_system::task::GraspTask task;
  ASSERT_TRUE(task.Configure(robot.model(), MakeTaskConfig(entry_distance)));
  ASSERT_TRUE(task.OnEnter(robot, state));

  plato_robot_system::task::GraspTaskCommand input;
  input.u = 0.2;
  input.phi = 0.5;
  input.desired_force_n = 1.0;

  plato_robot_system::RobotCommand command;
  ASSERT_TRUE(task.PopulateCommand(robot, state, input, kDtSec, &command));
  ASSERT_EQ(task.mode(), plato_robot_system::task::GraspTaskMode::kForceTracking);

  input.u = 1.0;
  ASSERT_TRUE(task.PopulateCommand(robot, state, input, kDtSec, &command));
  EXPECT_EQ(task.mode(), plato_robot_system::task::GraspTaskMode::kMotionTeleop);
  ExpectSaneActiveOnlyCommand(robot, state, command);
}
