#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "plato_grasp_controller/grasp_plan_parsing.hpp"
#include "plato_grasp_controller/plato_grasp_impedance_handler.hpp"
#include "plato_grasp_controller/plato_grasp_planner.hpp"
#include "plato_grasp_controller/plato_grasp_task_runner.hpp"
#include "plato_utils/joint_position_storage.hpp"

namespace
{

class TempDir
{
public:
  TempDir()
  {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
      ("plato_grasp_controller_test_" + std::to_string(stamp));
    std::filesystem::create_directories(path_);
  }

  ~TempDir()
  {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  std::string file_path(const std::string & name) const
  {
    return (path_ / name).string();
  }

  void write_file(const std::string & name, const std::string & content) const
  {
    const auto full_path = path_ / name;
    std::filesystem::create_directories(full_path.parent_path());
    std::ofstream out(full_path, std::ios::out | std::ios::trunc);
    ASSERT_TRUE(out.is_open());
    out << content;
    ASSERT_TRUE(out.good());
  }

private:
  std::filesystem::path path_;
};

std::vector<double> make_expected_ordered_positions()
{
  return {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
}

std::vector<std::string> make_scrambled_joint_names()
{
  return {"joint4", "joint2", "joint8", "joint1", "joint6", "joint3", "joint5", "joint7"};
}

std::vector<double> make_scrambled_joint_positions()
{
  return {0.4, 0.2, 0.8, 0.1, 0.6, 0.3, 0.5, 0.7};
}

void expect_vectors_near(
  const std::vector<double> & actual,
  const std::vector<double> & expected,
  double tolerance = 1e-12)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], tolerance) << "at index " << i;
  }
}

std::string make_plan_yaml()
{
  return
    R"(tasks:
  idle:
    use_current_position: true
    impedance_level: 0.0
    wait_sec: 0.0
    grasp_plan:
      effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  pick:
    pos_preset_name: captured_pose
    impedance_level: 5.0
    wait_sec: 0.15
    grasp_plan:
      effort_ff: [0.0, 0.0, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
      grasp_duration_sec: 0.25
)";
}

std::string make_impedance_preset_yaml()
{
  return
    R"(impedance_preset:
  min:
    stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    damping: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max:
    stiffness: [2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4]
    damping: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
)";
}

}  // namespace

TEST(PlatoGraspControllerCoreTest, ParsesPlansTasksAndImpedanceLevels)
{
  TempDir temp_dir;
  const auto yaml_path = temp_dir.file_path("grasp_plans.yaml");
  temp_dir.write_file("grasp_plans.yaml", make_plan_yaml());

  std::string error;

  std::unordered_map<std::string, plato_grasp_controller::GraspTaskConfig> tasks;
  ASSERT_TRUE(
    plato_grasp_controller::parsing::load_task_configs(
      yaml_path, 8, &tasks, &error)) << error;
  ASSERT_EQ(tasks.size(), 2u);
  EXPECT_TRUE(tasks.at("idle").use_current_position);
  EXPECT_EQ(tasks.at("idle").pos_preset_name, "");
  EXPECT_DOUBLE_EQ(tasks.at("idle").impedance_level, 0.0);
  EXPECT_EQ(tasks.at("pick").pos_preset_name, "captured_pose");
  EXPECT_FALSE(tasks.at("pick").use_current_position);
  EXPECT_DOUBLE_EQ(tasks.at("pick").impedance_level, 5.0);
  EXPECT_DOUBLE_EQ(tasks.at("pick").wait_sec, 0.15);
  EXPECT_DOUBLE_EQ(tasks.at("pick").grasp_duration_sec, 0.25);
  EXPECT_EQ(
    tasks.at("pick").grasp_plan.grasp_force_effort_ff,
    std::vector<double>({0.0, 0.0, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0}));
}

TEST(PlatoGraspControllerCoreTest, PlannerSavesJointStateAndBuildsMotionAndGraspCommands)
{
  TempDir temp_dir;
  const auto yaml_path = temp_dir.file_path("grasp_plans.yaml");
  const auto saved_positions_path = temp_dir.file_path("joint_positions.yaml");
  temp_dir.write_file("grasp_plans.yaml", make_plan_yaml());

  plato_grasp_controller::PlatoGraspPlanner planner(8, saved_positions_path);

  std::string saved_name;
  std::string error;
  EXPECT_FALSE(planner.save_current_joint_position("captured_pose", &saved_name, &error));
  EXPECT_NE(error.find("have not been received"), std::string::npos);

  planner.update_joint_state(make_scrambled_joint_names(), make_scrambled_joint_positions());
  ASSERT_TRUE(planner.has_joint_state());
  ASSERT_TRUE(planner.save_current_joint_position("captured_pose", &saved_name, &error)) << error;
  EXPECT_EQ(saved_name, "captured_pose");

  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  ASSERT_TRUE(
    plato::storage::load_joint_position_yaml(
      saved_positions_path, "captured_pose", &joint_names, &joint_positions, &error)) << error;
  EXPECT_EQ(
    joint_names,
    std::vector<std::string>(
      {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
        "joint7", "joint8"}));
  EXPECT_EQ(joint_positions, make_expected_ordered_positions());

  plato_grasp_controller::PlatoGraspPlannedCommand motion_command;
  ASSERT_TRUE(planner.make_motion_command("captured_pose", &motion_command, &error)) << error;
  EXPECT_EQ(motion_command.position, make_expected_ordered_positions());
  EXPECT_EQ(motion_command.velocity, std::vector<double>(8, 0.0));
  EXPECT_EQ(motion_command.effort_ff, std::vector<double>(8, 0.0));
  EXPECT_EQ(planner.phase(), plato_grasp_controller::PlatoGraspPlanner::Phase::Motion);
  EXPECT_EQ(planner.active_pos_preset_name(), "captured_pose");

  plato_grasp_controller::PlatoGraspPlannedCommand current_motion_command;
  ASSERT_TRUE(planner.make_current_motion_command(&current_motion_command, &error)) << error;
  EXPECT_EQ(current_motion_command.position, make_expected_ordered_positions());
  EXPECT_EQ(current_motion_command.velocity, std::vector<double>(8, 0.0));
  EXPECT_EQ(current_motion_command.effort_ff, std::vector<double>(8, 0.0));
  EXPECT_EQ(planner.phase(), plato_grasp_controller::PlatoGraspPlanner::Phase::Motion);
  EXPECT_EQ(planner.active_pos_preset_name(), "");

  plato_grasp_controller::PlatoGraspPlannedCommand grasp_command;
  const plato_grasp_controller::GraspPlanConfig grasp_plan{
    std::vector<double>({0.0, 0.0, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0})};
  ASSERT_TRUE(planner.make_grasp_command(grasp_plan, &grasp_command, &error)) << error;
  EXPECT_EQ(grasp_command.position, make_expected_ordered_positions());
  EXPECT_EQ(
    grasp_command.effort_ff,
    std::vector<double>({0.0, 0.0, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0}));
  EXPECT_EQ(planner.phase(), plato_grasp_controller::PlatoGraspPlanner::Phase::Grasp);
}

TEST(PlatoGraspControllerCoreTest, GraspCommandRequiresActiveMotionPlan)
{
  TempDir temp_dir;
  const auto yaml_path = temp_dir.file_path("grasp_plans.yaml");
  const auto saved_positions_path = temp_dir.file_path("joint_positions.yaml");
  temp_dir.write_file("grasp_plans.yaml", make_plan_yaml());

  plato_grasp_controller::PlatoGraspPlanner planner(8, saved_positions_path);
  plato_grasp_controller::PlatoGraspPlannedCommand grasp_command;
  const plato_grasp_controller::GraspPlanConfig grasp_plan{
    std::vector<double>({0.0, 0.0, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0})};
  std::string error;
  EXPECT_FALSE(planner.make_grasp_command(grasp_plan, &grasp_command, &error));
  EXPECT_NE(error.find("before a motion target"), std::string::npos);
}

TEST(PlatoGraspControllerCoreTest, ImpedanceHandlerAppliesActiveMotionPlanImpedance)
{
  TempDir temp_dir;
  const auto yaml_path = temp_dir.file_path("grasp_plans.yaml");
  const auto impedance_preset_yaml_path = temp_dir.file_path("impedance_preset.yaml");
  temp_dir.write_file("grasp_plans.yaml", make_plan_yaml());
  temp_dir.write_file("impedance_preset.yaml", make_impedance_preset_yaml());

  plato_grasp_controller::PlatoGraspImpedanceHandler handler(
    8, impedance_preset_yaml_path);
  plato_interfaces::msg::ImpedanceCommands command;
  plato_grasp_controller::PlatoGraspPlannedCommand planned_command;
  planned_command.position = std::vector<double>(8, 1.0);
  planned_command.velocity = std::vector<double>(8, 0.0);
  planned_command.effort_ff = std::vector<double>(8, 0.2);

  std::string error;
  EXPECT_FALSE(handler.apply_current_impedance(planned_command, &command, &error));
  EXPECT_NE(error.find("No active motion plan"), std::string::npos);

  ASSERT_TRUE(handler.activate(3.5, &error)) << error;
  ASSERT_TRUE(handler.apply_current_impedance(planned_command, &command, &error)) << error;
  EXPECT_EQ(command.position, planned_command.position);
  EXPECT_EQ(command.velocity, planned_command.velocity);
  EXPECT_EQ(command.effort_ff, planned_command.effort_ff);
  expect_vectors_near(
    command.stiffness,
    std::vector<double>({0.7, 0.77, 0.84, 0.91, 0.98, 1.05, 1.12, 1.19}));
  expect_vectors_near(command.damping, std::vector<double>(8, 0.07));
}

TEST(PlatoGraspControllerCoreTest, TaskRunnerPublishesMotionThenGraspThenHold)
{
  std::unordered_map<std::string, plato_grasp_controller::GraspTaskConfig> task_configs;
  task_configs.emplace(
    "pick",
    plato_grasp_controller::GraspTaskConfig{
    "captured_pose",
    false,
    5.0,
    0.10,
    plato_grasp_controller::GraspPlanConfig{
      std::vector<double>({0.0, 0.0, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0})},
    0.20});

  plato_grasp_controller::PlatoGraspTaskRunner runner(std::move(task_configs));
  plato_grasp_controller::PlatoGraspTaskRunner::Action action;
  std::string error;

  ASSERT_TRUE(runner.start_task("pick", &action, &error)) << error;
  EXPECT_EQ(
    action.type,
    plato_grasp_controller::PlatoGraspTaskRunner::ActionType::PublishMotionPlan);
  EXPECT_EQ(action.pos_preset_name, "captured_pose");
  EXPECT_FALSE(action.use_current_position);
  EXPECT_DOUBLE_EQ(action.impedance_level, 5.0);
  EXPECT_TRUE(runner.is_active());
  EXPECT_EQ(runner.state(), plato_grasp_controller::PlatoGraspTaskRunner::State::Waiting);

  ASSERT_TRUE(runner.update(0.05, &action, &error)) << error;
  EXPECT_EQ(action.type, plato_grasp_controller::PlatoGraspTaskRunner::ActionType::None);
  EXPECT_EQ(runner.state(), plato_grasp_controller::PlatoGraspTaskRunner::State::Waiting);

  ASSERT_TRUE(runner.update(0.05, &action, &error)) << error;
  EXPECT_EQ(
    action.type,
    plato_grasp_controller::PlatoGraspTaskRunner::ActionType::PublishGraspPlan);
  ASSERT_TRUE(action.grasp_plan.has_value());
  EXPECT_EQ(
    action.grasp_plan->grasp_force_effort_ff,
    std::vector<double>({0.0, 0.0, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0}));
  EXPECT_EQ(runner.state(), plato_grasp_controller::PlatoGraspTaskRunner::State::Grasping);

  ASSERT_TRUE(runner.update(0.19, &action, &error)) << error;
  EXPECT_EQ(action.type, plato_grasp_controller::PlatoGraspTaskRunner::ActionType::None);
  EXPECT_TRUE(runner.is_active());

  ASSERT_TRUE(runner.update(0.01, &action, &error)) << error;
  EXPECT_EQ(
    action.type,
    plato_grasp_controller::PlatoGraspTaskRunner::ActionType::PublishMotionHold);
  EXPECT_EQ(action.pos_preset_name, "captured_pose");
  EXPECT_DOUBLE_EQ(action.impedance_level, 5.0);
  EXPECT_FALSE(runner.is_active());
  EXPECT_EQ(runner.state(), plato_grasp_controller::PlatoGraspTaskRunner::State::Idle);
}

TEST(PlatoGraspControllerCoreTest, TaskRunnerKeepsGraspActiveWhenDurationIsZero)
{
  std::unordered_map<std::string, plato_grasp_controller::GraspTaskConfig> task_configs;
  plato_grasp_controller::GraspTaskConfig task;
  task.pos_preset_name = "captured_pose";
  task.use_current_position = false;
  task.impedance_level = 4.0;
  task.wait_sec = 0.0;
  task.grasp_plan = plato_grasp_controller::GraspPlanConfig{
    std::vector<double>({0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0})};
  task.grasp_duration_sec = 0.0;
  task_configs.emplace("hold_pick", task);

  plato_grasp_controller::PlatoGraspTaskRunner runner(std::move(task_configs));
  plato_grasp_controller::PlatoGraspTaskRunner::Action action;
  std::string error;

  ASSERT_TRUE(runner.start_task("hold_pick", &action, &error)) << error;
  ASSERT_TRUE(runner.update(0.0, &action, &error)) << error;
  EXPECT_EQ(
    action.type,
    plato_grasp_controller::PlatoGraspTaskRunner::ActionType::PublishGraspPlan);
  EXPECT_TRUE(action.grasp_plan.has_value());

  ASSERT_TRUE(runner.update(10.0, &action, &error)) << error;
  EXPECT_EQ(
    action.type,
    plato_grasp_controller::PlatoGraspTaskRunner::ActionType::PublishGraspPlan);
  EXPECT_TRUE(action.grasp_plan.has_value());
  EXPECT_EQ(
    action.grasp_plan->grasp_force_effort_ff,
    std::vector<double>({0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_TRUE(runner.is_active());
  EXPECT_EQ(runner.state(), plato_grasp_controller::PlatoGraspTaskRunner::State::Grasping);
}

TEST(PlatoGraspControllerCoreTest, TaskRunnerPropagatesUseCurrentPositionTasks)
{
  std::unordered_map<std::string, plato_grasp_controller::GraspTaskConfig> task_configs;
  plato_grasp_controller::GraspTaskConfig task;
  task.use_current_position = true;
  task.impedance_level = 0.0;
  task.wait_sec = 0.0;
  task.grasp_plan = plato_grasp_controller::GraspPlanConfig{
    std::vector<double>(8, 0.0)};
  task.grasp_duration_sec = 0.0;
  task_configs.emplace("idle", task);

  plato_grasp_controller::PlatoGraspTaskRunner runner(std::move(task_configs));
  plato_grasp_controller::PlatoGraspTaskRunner::Action action;
  std::string error;

  ASSERT_TRUE(runner.start_task("idle", &action, &error)) << error;
  EXPECT_EQ(
    action.type,
    plato_grasp_controller::PlatoGraspTaskRunner::ActionType::PublishMotionPlan);
  EXPECT_TRUE(action.use_current_position);
  EXPECT_EQ(action.pos_preset_name, "");
  EXPECT_DOUBLE_EQ(action.impedance_level, 0.0);

  ASSERT_TRUE(runner.update(0.0, &action, &error)) << error;
  EXPECT_EQ(
    action.type,
    plato_grasp_controller::PlatoGraspTaskRunner::ActionType::PublishGraspPlan);
  EXPECT_TRUE(action.use_current_position);
  ASSERT_TRUE(action.grasp_plan.has_value());
  EXPECT_EQ(action.grasp_plan->grasp_force_effort_ff, std::vector<double>(8, 0.0));
}
