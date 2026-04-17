#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "plato_grasp_controller/plato_grasp_controller_node.hpp"
#include "plato_interfaces/msg/impedance_commands.hpp"
#include "plato_interfaces/srv/save_joint_position.hpp"
#include "plato_utils/joint_position_storage.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

using namespace std::chrono_literals;

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class TempDir
{
public:
  TempDir()
  {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
      ("plato_grasp_controller_pipeline_test_" + std::to_string(stamp));
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

class PlatoGraspControllerPipelineTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    temp_dir_ = std::make_unique<TempDir>();
    saved_joint_positions_path_ = temp_dir_->file_path("joint_positions.yaml");
    plan_config_path_ = temp_dir_->file_path("grasp_plans.yaml");
    impedance_preset_yaml_path_ = temp_dir_->file_path("impedance_preset.yaml");
    temp_dir_->write_file("grasp_plans.yaml", make_plan_yaml());
    temp_dir_->write_file("impedance_preset.yaml", make_impedance_preset_yaml());

    const auto stamp = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    joint_state_topic_ = "/plato_grasp_controller_test_" + stamp + "/joint_states";
    motion_state_topic_ = "/plato_grasp_controller_test_" + stamp + "/motion_state";
    task_topic_ = "/plato_grasp_controller_test_" + stamp + "/task";
    trajectory_goal_topic_ = "/plato_grasp_controller_test_" + stamp + "/goal_command";

    rclcpp::NodeOptions options;
    options.append_parameter_override("joint_count", 8);
    options.append_parameter_override("joint_state_topic", joint_state_topic_);
    options.append_parameter_override("motion_state_topic", motion_state_topic_);
    options.append_parameter_override("task_topic", task_topic_);
    options.append_parameter_override("trajectory_goal_topic", trajectory_goal_topic_);
    options.append_parameter_override(
      "saved_joint_positions_yaml_path", saved_joint_positions_path_);
    options.append_parameter_override("plan_config_yaml_path", plan_config_path_);
    options.append_parameter_override("impedance_preset_yaml_path", impedance_preset_yaml_path_);
    options.append_parameter_override("manual_motion_impedance_level", 4.0);
    options.append_parameter_override("task_update_rate_hz", 100.0);

    controller_node_ = std::make_shared<plato_grasp_controller::PlatoGraspControllerNode>(options);
    test_node_ = std::make_shared<rclcpp::Node>("plato_grasp_controller_pipeline_test_node");

    executor_.add_node(controller_node_);
    executor_.add_node(test_node_);

    joint_state_pub_ = test_node_->create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS());
    motion_state_pub_ = test_node_->create_publisher<std_msgs::msg::String>(
      motion_state_topic_, 10);
    task_pub_ = test_node_->create_publisher<std_msgs::msg::String>(task_topic_, 10);
    goal_sub_ = test_node_->create_subscription<plato_interfaces::msg::ImpedanceCommands>(
      trajectory_goal_topic_,
      50,
      [this](const plato_interfaces::msg::ImpedanceCommands::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(messages_mutex_);
        received_commands_.push_back(*msg);
      });
    save_client_ = test_node_->create_client<plato_interfaces::srv::SaveJointPosition>(
      "/plato_grasp_controller_node/save_joint_position");
  }

  void TearDown() override
  {
    executor_.remove_node(test_node_);
    executor_.remove_node(controller_node_);
    test_node_.reset();
    controller_node_.reset();
    temp_dir_.reset();
  }

  static std::string make_plan_yaml()
  {
    return
      R"(tasks:
  idle:
    use_current_position: true
    impedance_level: 0.0
    wait_sec: 0.0
    grasp_plan:
      grasp_duration_sec: 0.0
  pick:
    pos_preset_name: saved_contact
    impedance_level: 8.0
    wait_sec: 0.05
    grasp_plan:
      grasp_closure_scale: [0.0, 0.0, -0.1, -0.1, 0.1, 0.1, 0.0, 0.0]
      grasp_duration_sec: 0.05
  fractional_pick:
    pos_preset_name: saved_contact
    impedance_level: 7.5
    wait_sec: 0.05
    grasp_plan:
      grasp_closure_scale: [0.0, 0.0, -0.1, -0.1, 0.1, 0.1, 0.0, 0.0]
      grasp_duration_sec: 0.05
  hold_pick:
    pos_preset_name: saved_contact
    impedance_level: 8.0
    wait_sec: 0.02
    grasp_plan:
      grasp_closure_scale: [0.0, 0.0, -0.1, -0.1, 0.1, 0.1, 0.0, 0.0]
      grasp_duration_sec: 0.0
)";
  }

  static std::string make_impedance_preset_yaml()
  {
    return
      R"(impedance_preset:
  min:
    stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    damping: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max:
    stiffness: [2.5, 2.5, 3.75, 3.75, 3.75, 3.75, 1.25, 1.25]
    damping: [0.25, 0.25, 0.375, 0.375, 0.375, 0.375, 0.125, 0.125]
)";
  }

  void spin_for(const std::chrono::milliseconds duration)
  {
    const auto end_time = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end_time) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  bool spin_until(
    const std::function<bool()> & predicate,
    const std::chrono::milliseconds timeout)
  {
    const auto end_time = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end_time) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
    return predicate();
  }

  size_t command_count() const
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    return received_commands_.size();
  }

  plato_interfaces::msg::ImpedanceCommands command_at(size_t index) const
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    return received_commands_.at(index);
  }

  void publish_joint_state_sample()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = test_node_->now();
    msg.name = {"joint4", "joint2", "joint8", "joint1", "joint6", "joint3", "joint5", "joint7"};
    msg.position = {0.4, 0.2, 0.8, 0.1, 0.6, 0.3, 0.5, 0.7};
    joint_state_pub_->publish(msg);
  }

  std::vector<double> expected_positions() const
  {
    return {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
  }

  std::vector<double> expected_grasp_positions() const
  {
    return {0.1, 0.2, 0.2, 0.3, 0.6, 0.7, 0.7, 0.8};
  }

  void publish_string(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & publisher,
    const std::string & value)
  {
    std_msgs::msg::String msg;
    msg.data = value;
    publisher->publish(msg);
  }

  std::unique_ptr<TempDir> temp_dir_;
  std::string saved_joint_positions_path_;
  std::string plan_config_path_;
  std::string impedance_preset_yaml_path_;
  std::string joint_state_topic_;
  std::string motion_state_topic_;
  std::string task_topic_;
  std::string trajectory_goal_topic_;

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<plato_grasp_controller::PlatoGraspControllerNode> controller_node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;
  rclcpp::Subscription<plato_interfaces::msg::ImpedanceCommands>::SharedPtr goal_sub_;
  rclcpp::Client<plato_interfaces::srv::SaveJointPosition>::SharedPtr save_client_;
  mutable std::mutex messages_mutex_;
  std::vector<plato_interfaces::msg::ImpedanceCommands> received_commands_;
};

TEST_F(PlatoGraspControllerPipelineTest, RunsSaveMotionAndTaskPipeline)
{
  ASSERT_TRUE(
    spin_until(
      [this]() {return save_client_->wait_for_service(0s);}, 2s));

  for (int i = 0; i < 5; ++i) {
    publish_joint_state_sample();
    spin_for(20ms);
  }

  auto request = std::make_shared<plato_interfaces::srv::SaveJointPosition::Request>();
  request->name = "saved_contact";
  auto future = save_client_->async_send_request(request);
  ASSERT_TRUE(
    spin_until(
      [&future]() {
        return future.wait_for(0s) == std::future_status::ready;
      }, 2s));

  const auto response = future.get();
  ASSERT_TRUE(response->success) << response->message;
  EXPECT_EQ(response->saved_name, "saved_contact");
  EXPECT_EQ(response->saved_path, saved_joint_positions_path_);

  std::vector<std::string> saved_joint_names;
  std::vector<double> saved_positions;
  std::string error;
  ASSERT_TRUE(
    plato::storage::load_joint_position_yaml(
      saved_joint_positions_path_,
      "saved_contact",
      &saved_joint_names,
      &saved_positions,
      &error)) << error;
  EXPECT_EQ(saved_positions, expected_positions());

  publish_string(motion_state_pub_, "saved_contact");
  ASSERT_TRUE(spin_until([this]() {return command_count() >= 1;}, 2s));
  const auto motion_command = command_at(0);
  EXPECT_EQ(motion_command.position, expected_positions());
  EXPECT_EQ(motion_command.effort_ff, std::vector<double>(8, 0.0));
  expect_vectors_near(
    motion_command.stiffness,
    std::vector<double>({1.0, 1.0, 1.5, 1.5, 1.5, 1.5, 0.5, 0.5}));
  expect_vectors_near(
    motion_command.damping,
    std::vector<double>({0.1, 0.1, 0.15, 0.15, 0.15, 0.15, 0.05, 0.05}));

  const auto baseline_count = command_count();
  publish_string(task_pub_, "pick");
  ASSERT_TRUE(
    spin_until(
      [this, baseline_count]() {
        return command_count() >= baseline_count + 3;
      }, 3s));

  const auto task_motion_command = command_at(baseline_count);
  const auto task_grasp_command = command_at(baseline_count + 1);
  const auto task_hold_command = command_at(baseline_count + 2);

  EXPECT_EQ(task_motion_command.position, expected_positions());
  EXPECT_EQ(task_motion_command.effort_ff, std::vector<double>(8, 0.0));
  expect_vectors_near(
    task_motion_command.stiffness,
    std::vector<double>({2.0, 2.0, 3.0, 3.0, 3.0, 3.0, 1.0, 1.0}));
  expect_vectors_near(
    task_motion_command.damping,
    std::vector<double>({0.2, 0.2, 0.3, 0.3, 0.3, 0.3, 0.1, 0.1}));

  EXPECT_EQ(task_grasp_command.effort_ff, std::vector<double>(8, 0.0));
  expect_vectors_near(task_grasp_command.position, expected_grasp_positions());
  expect_vectors_near(
    task_grasp_command.stiffness,
    std::vector<double>({2.0, 2.0, 3.0, 3.0, 3.0, 3.0, 1.0, 1.0}));
  expect_vectors_near(
    task_grasp_command.damping,
    std::vector<double>({0.2, 0.2, 0.3, 0.3, 0.3, 0.3, 0.1, 0.1}));

  expect_vectors_near(task_hold_command.position, expected_grasp_positions());
  EXPECT_EQ(task_hold_command.effort_ff, std::vector<double>(8, 0.0));
  expect_vectors_near(task_hold_command.stiffness, task_grasp_command.stiffness);
  expect_vectors_near(task_hold_command.damping, task_grasp_command.damping);
}

TEST_F(PlatoGraspControllerPipelineTest, IdleTaskHoldsCurrentPositionWithZeroImpedance)
{
  for (int i = 0; i < 5; ++i) {
    publish_joint_state_sample();
    spin_for(20ms);
  }

  publish_string(task_pub_, "idle");
  ASSERT_TRUE(spin_until([this]() {return command_count() >= 2;}, 2s));

  const auto idle_motion_command = command_at(0);
  const auto idle_grasp_command = command_at(1);

  EXPECT_EQ(idle_motion_command.position, expected_positions());
  EXPECT_EQ(idle_motion_command.effort_ff, std::vector<double>(8, 0.0));
  expect_vectors_near(idle_motion_command.stiffness, std::vector<double>(8, 0.0));
  expect_vectors_near(idle_motion_command.damping, std::vector<double>(8, 0.0));

  EXPECT_EQ(idle_grasp_command.position, expected_positions());
  EXPECT_EQ(idle_grasp_command.effort_ff, std::vector<double>(8, 0.0));
  expect_vectors_near(idle_grasp_command.stiffness, std::vector<double>(8, 0.0));
  expect_vectors_near(idle_grasp_command.damping, std::vector<double>(8, 0.0));
}

TEST_F(PlatoGraspControllerPipelineTest, FractionalImpedanceLevelInterpolatesPublishedGains)
{
  ASSERT_TRUE(
    spin_until(
      [this]() {return save_client_->wait_for_service(0s);}, 2s));

  for (int i = 0; i < 5; ++i) {
    publish_joint_state_sample();
    spin_for(20ms);
  }

  auto request = std::make_shared<plato_interfaces::srv::SaveJointPosition::Request>();
  request->name = "saved_contact";
  auto future = save_client_->async_send_request(request);
  ASSERT_TRUE(
    spin_until(
      [&future]() {
        return future.wait_for(0s) == std::future_status::ready;
      }, 2s));
  ASSERT_TRUE(future.get()->success);

  const auto baseline_count = command_count();
  publish_string(task_pub_, "fractional_pick");
  ASSERT_TRUE(
    spin_until(
      [this, baseline_count]() {
        return command_count() >= baseline_count + 1;
      }, 2s));

  const auto motion_command = command_at(baseline_count);
  EXPECT_EQ(motion_command.position, expected_positions());
  EXPECT_EQ(motion_command.effort_ff, std::vector<double>(8, 0.0));
  expect_vectors_near(
    motion_command.stiffness,
    std::vector<double>({1.875, 1.875, 2.8125, 2.8125, 2.8125, 2.8125, 0.9375, 0.9375}),
    1e-9);
  expect_vectors_near(
    motion_command.damping,
    std::vector<double>({0.1875, 0.1875, 0.28125, 0.28125, 0.28125, 0.28125, 0.09375, 0.09375}),
    1e-9);
}

TEST_F(PlatoGraspControllerPipelineTest, ZeroDurationGraspKeepsRepublishingHighImpedanceCommand)
{
  ASSERT_TRUE(
    spin_until(
      [this]() {return save_client_->wait_for_service(0s);}, 2s));

  for (int i = 0; i < 5; ++i) {
    publish_joint_state_sample();
    spin_for(20ms);
  }

  auto request = std::make_shared<plato_interfaces::srv::SaveJointPosition::Request>();
  request->name = "saved_contact";
  auto future = save_client_->async_send_request(request);
  ASSERT_TRUE(
    spin_until(
      [&future]() {
        return future.wait_for(0s) == std::future_status::ready;
      }, 2s));
  ASSERT_TRUE(future.get()->success);

  const auto baseline_count = command_count();
  publish_string(task_pub_, "hold_pick");
  ASSERT_TRUE(
    spin_until(
      [this, baseline_count]() {
        return command_count() >= baseline_count + 4;
      }, 3s));

  const auto motion_command = command_at(baseline_count);
  const auto first_grasp_command = command_at(baseline_count + 1);
  const auto repeated_grasp_command = command_at(baseline_count + 2);

  EXPECT_EQ(motion_command.effort_ff, std::vector<double>(8, 0.0));
  EXPECT_EQ(first_grasp_command.effort_ff, std::vector<double>(8, 0.0));
  expect_vectors_near(
    first_grasp_command.stiffness,
    std::vector<double>({2.0, 2.0, 3.0, 3.0, 3.0, 3.0, 1.0, 1.0}));
  expect_vectors_near(
    first_grasp_command.damping,
    std::vector<double>({0.2, 0.2, 0.3, 0.3, 0.3, 0.3, 0.1, 0.1}));
  expect_vectors_near(repeated_grasp_command.position, expected_grasp_positions());
  EXPECT_EQ(repeated_grasp_command.stiffness, first_grasp_command.stiffness);
  EXPECT_EQ(repeated_grasp_command.damping, first_grasp_command.damping);
  EXPECT_EQ(repeated_grasp_command.effort_ff, first_grasp_command.effort_ff);
}

::testing::Environment * const kRclcppEnvironment =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

}  // namespace
