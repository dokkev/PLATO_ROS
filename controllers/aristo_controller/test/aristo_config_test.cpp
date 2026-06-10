#include <gtest/gtest.h>

#include <filesystem>

#include "aristo_controller/config/aristo_config.hpp"

namespace
{

std::filesystem::path AristoConfigPath()
{
  return std::filesystem::path(ARISTO_CONTROLLER_CONFIG_PATH);
}

}  // namespace

TEST(AristoConfigTest, LoadsDefaultYamlWithUnifiedGraspTaskConfig)
{
  ASSERT_TRUE(std::filesystem::exists(AristoConfigPath())) << AristoConfigPath();

  const auto config =
    aristo_controller::config::load_aristo_config(AristoConfigPath().string());

  EXPECT_EQ(config.num_joints, 8);
  EXPECT_EQ(config.joint_teleop.id, 2);
  EXPECT_EQ(config.grasp_teleop.id, 3);
  EXPECT_EQ(config.mppi_grasp.id, 4);

  EXPECT_EQ(config.driver_gains.kp.size(), config.num_joints);
  EXPECT_EQ(config.driver_gains.kd.size(), config.num_joints);

  const auto & joint_teleop_task = config.joint_teleop.state.joint_task;
  EXPECT_EQ(joint_teleop_task.kp_task.size(), config.num_joints);
  EXPECT_EQ(joint_teleop_task.kd_task.size(), config.num_joints);
  EXPECT_DOUBLE_EQ(joint_teleop_task.lpf_alpha, 0.1);

  const auto & grasp_task = config.grasp_teleop.state.grasp_task;
  const plato_robot_system::task::GraspTaskConfig header_defaults;
  EXPECT_DOUBLE_EQ(config.grasp_teleop.state.default_u_close, 0.8);
  EXPECT_DOUBLE_EQ(config.grasp_teleop.state.default_u_lateral, 0.5);
  ASSERT_EQ(grasp_task.q_posture.size(), 4);
  EXPECT_TRUE(grasp_task.q_posture.isZero());
  EXPECT_GT(grasp_task.distance_open_m, grasp_task.distance_closed_m);
  EXPECT_DOUBLE_EQ(grasp_task.lateral_offset_limit_m, 0.02);
  EXPECT_DOUBLE_EQ(grasp_task.max_torque_nm, 0.05);
  EXPECT_DOUBLE_EQ(grasp_task.max_qddot_rad_s2, 5.0);
  EXPECT_DOUBLE_EQ(grasp_task.max_velocity_rad_s, 0.5);
  EXPECT_DOUBLE_EQ(grasp_task.force_exit_u_threshold, 0.75);
  EXPECT_DOUBLE_EQ(grasp_task.kp_lateral, 20.0);
  EXPECT_DOUBLE_EQ(grasp_task.kd_lateral, 2.0);
  EXPECT_DOUBLE_EQ(grasp_task.kp_tactile_fb, 0.02);
  EXPECT_DOUBLE_EQ(grasp_task.w_lateral, 5.0);
  EXPECT_TRUE(grasp_task.fallback_close_axis_base.isApprox(
    Eigen::Vector3d{0.0, 0.0, -1.0}));

  EXPECT_EQ(grasp_task.force_enter_debounce_ticks, header_defaults.force_enter_debounce_ticks);
  EXPECT_DOUBLE_EQ(grasp_task.w_tactile, header_defaults.w_tactile);
  EXPECT_EQ(
    grasp_task.force_aggregation,
    plato_robot_system::task::GraspTaskConfig::ForceAggregation::kMin);
  EXPECT_TRUE(grasp_task.use_inverse_dynamics);
}
