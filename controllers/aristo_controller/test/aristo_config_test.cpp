#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include <yaml-cpp/yaml.h>

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
  EXPECT_EQ(config.grasp_force.id, 6);
  EXPECT_EQ(config.poke.id, 1);
  EXPECT_TRUE(config.initialize.lifecycle.stay_here);
  EXPECT_FALSE(config.grasp_ready.lifecycle.stay_here);
  EXPECT_DOUBLE_EQ(config.grasp_ready.lifecycle.duration, 2.0);
  EXPECT_EQ(config.grasp_ready.lifecycle.next_state_id, config.grasp_teleop.id);
  EXPECT_EQ(config.grasp_teleop.lifecycle.next_state_id, config.grasp_force.id);
  EXPECT_EQ(config.grasp_force.lifecycle.next_state_id, config.grasp_teleop.id);

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
  EXPECT_DOUBLE_EQ(config.grasp_teleop.state.default_u, 1.0);
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
  EXPECT_FALSE(grasp_task.debug_print_contact_states);
  EXPECT_DOUBLE_EQ(grasp_task.debug_print_contact_interval_s, 0.25);
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
  EXPECT_DOUBLE_EQ(config.grasp_force.state.default_u, 0.2);
  EXPECT_DOUBLE_EQ(config.grasp_force.state.default_phi, 0.4);
  EXPECT_TRUE(config.grasp_force.state.exit_on_contact_lost);
  EXPECT_TRUE(config.grasp_force.state.exit_on_u_above_threshold);
  ASSERT_EQ(grasp_force_task.q_ready.size(), config.num_joints);
  EXPECT_TRUE(grasp_force_task.q_ready.isApprox(expected_grasp_ready_target));
  EXPECT_TRUE(grasp_force_task.force_feedback_enabled);
  EXPECT_FALSE(grasp_force_task.debug_print_contact_states);
  EXPECT_DOUBLE_EQ(grasp_force_task.lpf_alpha, 0.2);
  EXPECT_EQ(grasp_force_task.force_enter_debounce_ticks, 0);
  EXPECT_DOUBLE_EQ(grasp_force_task.force_exit_u_threshold, 0.6);
  EXPECT_DOUBLE_EQ(grasp_force_task.kp_tactile_u_fb, 3.0);

}

TEST(AristoConfigTest, GraspTeleopDefaultUCanBeConfigured)
{
  auto root = YAML::LoadFile(AristoConfigPath().string());
  bool found_grasp_teleop = false;
  for (auto state : root["state_machine"]["states"]) {
    if (state["name"] && state["name"].as<std::string>() == "grasp_teleop") {
      auto params = state["params"];
      params["default_u"] = 0.99;
      params["default_phi"] = 0.88;
      params["shared_control_min_contact_sensors"] = 9;
      params["shared_control_enter_debounce_ticks"] = 11;
      params["shared_control_requires_enough_contact"] = false;
      found_grasp_teleop = true;
    }
  }
  ASSERT_TRUE(found_grasp_teleop);

  const auto temp_path =
    std::filesystem::temp_directory_path() / "aristo_hardcoded_config_test.yaml";
  {
    std::ofstream out(temp_path);
    ASSERT_TRUE(out.good()) << temp_path;
    out << root;
  }

  const auto config =
    aristo_controller::config::load_aristo_config(temp_path.string());
  std::filesystem::remove(temp_path);

  EXPECT_DOUBLE_EQ(config.grasp_teleop.state.default_u, 0.99);
  EXPECT_DOUBLE_EQ(config.grasp_teleop.state.default_phi, 0.0);
  EXPECT_EQ(config.grasp_teleop.state.shared_control_min_contact_sensors, 2);
  EXPECT_EQ(config.grasp_teleop.state.shared_control_enter_debounce_ticks, 3);
  EXPECT_TRUE(config.grasp_teleop.state.shared_control_requires_enough_contact);
}
