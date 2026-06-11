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
  EXPECT_EQ(config.idle.id, 100);
  EXPECT_EQ(config.joint_teleop.id, 2);
  EXPECT_EQ(config.grasp_ready.id, 3);
  EXPECT_EQ(config.grasp_teleop.id, 4);
  EXPECT_EQ(config.mppi_grasp.id, 5);
  EXPECT_EQ(config.grasp_force.id, 6);
  EXPECT_EQ(config.mppi_motion_grasp.id, 7);
  EXPECT_EQ(config.poke.id, 1);
  EXPECT_TRUE(config.initialize.lifecycle.stay_here);
  EXPECT_FALSE(config.grasp_ready.lifecycle.stay_here);
  EXPECT_DOUBLE_EQ(config.grasp_ready.lifecycle.duration, 2.0);
  EXPECT_EQ(config.grasp_ready.lifecycle.next_state_id, config.grasp_teleop.id);
  EXPECT_EQ(config.grasp_teleop.lifecycle.next_state_id, config.mppi_grasp.id);

  EXPECT_EQ(config.driver_gains.kp.size(), config.num_joints);
  EXPECT_EQ(config.driver_gains.kd.size(), config.num_joints);

  ASSERT_EQ(config.initialize.target_jpos.size(), config.num_joints);
  Eigen::VectorXd expected_initialize_target(config.num_joints);
  expected_initialize_target <<
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0;
  EXPECT_TRUE(config.initialize.target_jpos.isApprox(expected_initialize_target));

  ASSERT_EQ(config.poke.target_jpos.size(), config.num_joints);
  Eigen::VectorXd expected_poke_target(config.num_joints);
  expected_poke_target <<
    0.0,
    0.0,
    -0.25024986267089844,
    -1.4372527771779176,
    0.0,
    0.0,
    1.100553035736084,
    1.5071229671464579;
  EXPECT_TRUE(config.poke.target_jpos.isApprox(expected_poke_target));

  ASSERT_EQ(config.grasp_ready.target_jpos.size(), config.num_joints);
  Eigen::VectorXd expected_grasp_ready_target(config.num_joints);
  expected_grasp_ready_target <<
    0.0,
    0.0,
    0.0,
    0.0,
    0.8849433621761859,
    -0.8849433621761859,
    0.785,
    1.5708;
  EXPECT_TRUE(config.grasp_ready.target_jpos.isApprox(expected_grasp_ready_target));

  const auto & joint_teleop_task = config.joint_teleop.state.joint_task;
  EXPECT_EQ(joint_teleop_task.kp_task.size(), config.num_joints);
  EXPECT_EQ(joint_teleop_task.kd_task.size(), config.num_joints);
  EXPECT_DOUBLE_EQ(joint_teleop_task.lpf_alpha, 0.1);

  const auto & grasp_task = config.grasp_teleop.state.grasp_task;
  const plato_robot_system::task::GraspTaskConfig header_defaults;
  EXPECT_DOUBLE_EQ(config.grasp_teleop.state.default_u, 0.7);
  EXPECT_DOUBLE_EQ(config.grasp_teleop.state.default_phi, 0.0);
  EXPECT_TRUE(config.grasp_teleop.state.shared_grasp_control);
  EXPECT_EQ(config.grasp_teleop.state.shared_control_min_contact_sensors, 2);
  EXPECT_EQ(config.grasp_teleop.state.shared_control_enter_debounce_ticks, 3);
  EXPECT_TRUE(config.grasp_teleop.state.shared_control_requires_u_below_threshold);
  EXPECT_TRUE(config.grasp_teleop.state.shared_control_requires_enough_contact);
  ASSERT_EQ(grasp_task.q_ready.size(), config.num_joints);
  EXPECT_TRUE(grasp_task.q_ready.isApprox(expected_grasp_ready_target));
  EXPECT_FALSE(grasp_task.force_feedback_enabled);
  EXPECT_DOUBLE_EQ(grasp_task.lpf_alpha, 0.2);
  EXPECT_DOUBLE_EQ(grasp_task.force_exit_u_threshold, 0.75);
  EXPECT_DOUBLE_EQ(grasp_task.parallel_midpoint_u, header_defaults.parallel_midpoint_u);
  EXPECT_DOUBLE_EQ(
    grasp_task.parallel_lateral_offset_m,
    header_defaults.parallel_lateral_offset_m);
  EXPECT_DOUBLE_EQ(
    grasp_task.parallel_max_flexion_rad,
    header_defaults.parallel_max_flexion_rad);

  EXPECT_EQ(grasp_task.force_enter_debounce_ticks, header_defaults.force_enter_debounce_ticks);
  EXPECT_EQ(
    grasp_task.force_aggregation,
    plato_robot_system::task::GraspTaskConfig::ForceAggregation::kMin);

  const auto & grasp_force_task = config.grasp_force.state.grasp_task;
  EXPECT_DOUBLE_EQ(config.grasp_force.state.default_u, 0.7);
  EXPECT_DOUBLE_EQ(config.grasp_force.state.default_phi, 0.0);
  ASSERT_EQ(grasp_force_task.q_ready.size(), config.num_joints);
  EXPECT_TRUE(grasp_force_task.q_ready.isApprox(expected_grasp_ready_target));
  EXPECT_TRUE(grasp_force_task.force_feedback_enabled);
  EXPECT_DOUBLE_EQ(grasp_force_task.lpf_alpha, 0.2);
  EXPECT_EQ(grasp_force_task.force_enter_debounce_ticks, 0);
  EXPECT_DOUBLE_EQ(grasp_force_task.force_exit_u_threshold, 0.75);
  EXPECT_DOUBLE_EQ(grasp_force_task.kp_tactile_u_fb, 0.02);

  const auto & mppi_grasp = config.mppi_grasp.state;
  EXPECT_EQ(mppi_grasp.mppi.horizon_steps, 10U);
  EXPECT_EQ(mppi_grasp.mppi.num_rollouts, 32U);
  EXPECT_EQ(mppi_grasp.mppi.action_dim, static_cast<std::size_t>(config.num_joints));
  EXPECT_DOUBLE_EQ(mppi_grasp.mppi.dt, 0.01);
  ASSERT_EQ(mppi_grasp.mppi.action_lower_bound.size(), config.num_joints);
  ASSERT_EQ(mppi_grasp.mppi.action_upper_bound.size(), config.num_joints);
  ASSERT_EQ(mppi_grasp.mppi.action_noise_std.size(), config.num_joints);
  EXPECT_TRUE(
    mppi_grasp.mppi.action_lower_bound.isApprox(
      Eigen::VectorXd::Constant(config.num_joints, -1.0)));
  EXPECT_TRUE(
    mppi_grasp.mppi.action_upper_bound.isApprox(
      Eigen::VectorXd::Constant(config.num_joints, 1.0)));
  EXPECT_TRUE(
    mppi_grasp.mppi.action_noise_std.isApprox(
      Eigen::VectorXd::Constant(config.num_joints, 0.5)));
  EXPECT_DOUBLE_EQ(mppi_grasp.safety.max_reference_tracking_error_rad, 0.25);
  EXPECT_DOUBLE_EQ(mppi_grasp.safety.max_qdot_cmd_rad_s, 0.5);
  EXPECT_DOUBLE_EQ(mppi_grasp.safety.max_tau_cmd_nm, 0.05);
  EXPECT_DOUBLE_EQ(mppi_grasp.safety.max_tau_rate_nm_s, 1.0);
  EXPECT_TRUE(mppi_grasp.safety.clamp_q_cmd_to_model_limits);
  EXPECT_DOUBLE_EQ(mppi_grasp.tactile.thumb_normal_axis_sign, 1.0);
  EXPECT_DOUBLE_EQ(mppi_grasp.tactile.index_normal_axis_sign, 1.0);
  EXPECT_EQ(mppi_grasp.task.start.min_enough_contact_sensors, 2U);
  EXPECT_EQ(mppi_grasp.task.start.min_active_hemispheres_total, 2U);
  EXPECT_EQ(mppi_grasp.task.cost.target_active_hemisphere_total, 4U);
  EXPECT_DOUBLE_EQ(mppi_grasp.task.tolerance.max_shear_m, 0.003);
  EXPECT_DOUBLE_EQ(mppi_grasp.tactile_transition.max_shear_m, 0.003);

  const auto & mppi_motion_grasp = config.mppi_motion_grasp.state;
  EXPECT_TRUE(mppi_motion_grasp.enabled);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.default_u, 1.0);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.default_phi, 0.5);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.default_desired_force_n, 1.0);
  EXPECT_EQ(mppi_motion_grasp.initiation.contact_enter_debounce_ticks, 3);
  EXPECT_EQ(mppi_motion_grasp.initiation.contact_exit_debounce_ticks, 3);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.initiation.contacted_finger_hold_weight, 50.0);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.initiation.moving_finger_target_weight, 10.0);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.initiation.posture_weight, 1.0);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.initiation.max_reference_tracking_error_rad, 0.5);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.safety.max_velocity_rad_s, 1.0);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.safety.max_torque_nm, 0.2);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.safety.max_torque_rate_nm_per_s, 2.0);
  EXPECT_EQ(mppi_motion_grasp.mppi.horizon_steps, 20U);
  EXPECT_EQ(mppi_motion_grasp.mppi.num_rollouts, 256U);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.mppi.action_lower_bound[0], -4.0);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.mppi.action_upper_bound[0], 4.0);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.mppi.action_noise_std[0], 1.0);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.cost.q_target_weight, 20.0);
  EXPECT_DOUBLE_EQ(mppi_motion_grasp.cost.line_of_action_weight, 5.0);
  EXPECT_TRUE(mppi_motion_grasp.cost.enable_line_of_action_cost);
  ASSERT_EQ(mppi_motion_grasp.grasp_task.q_ready.size(), config.num_joints);
  EXPECT_TRUE(mppi_motion_grasp.grasp_task.q_ready.isApprox(expected_grasp_ready_target));
  EXPECT_FALSE(mppi_motion_grasp.grasp_task.force_feedback_enabled);
}
