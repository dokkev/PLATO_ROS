#include "plato_grasp_controller/plato_grasp_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "plato_grasp_controller/grasp_plan_parsing.hpp"
#include "plato_utils/joint_position_storage.hpp"
#include "plato_utils/yaml_helpers.hpp"

namespace plato_grasp_controller
{

namespace
{

std::string trim_copy(const std::string & input)
{
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

const char * phase_to_string(PlatoGraspPlanner::Phase phase)
{
  switch (phase) {
    case PlatoGraspPlanner::Phase::Idle:
      return "idle";
    case PlatoGraspPlanner::Phase::Motion:
      return "motion";
    case PlatoGraspPlanner::Phase::Grasp:
      return "grasp";
  }
  return "unknown";
}

const char * task_state_to_string(PlatoGraspTaskRunner::State state)
{
  switch (state) {
    case PlatoGraspTaskRunner::State::Idle:
      return "idle";
    case PlatoGraspTaskRunner::State::Waiting:
      return "waiting";
    case PlatoGraspTaskRunner::State::Grasping:
      return "grasping";
  }
  return "unknown";
}

}  // namespace

PlatoGraspControllerNode::PlatoGraspControllerNode(const rclcpp::NodeOptions & options)
: Node("plato_grasp_controller_node", options),
  joint_count_(this->declare_parameter<int>("joint_count", 8)),
  joint_state_topic_(
    this->declare_parameter<std::string>("joint_state_topic", "/plato2/joint_states")),
  motion_state_topic_(this->declare_parameter<std::string>(
      "motion_state_topic", "/plato2/plato_grasp_controller/motion_state")),
  task_topic_(this->declare_parameter<std::string>(
      "task_topic", "/plato2/plato_grasp_controller/task")),
  trajectory_goal_topic_(this->declare_parameter<std::string>(
      "trajectory_goal_topic", "/plato2/joint_impedance_trajectory_controller/goal_command")),
  saved_joint_positions_yaml_path_(this->declare_parameter<std::string>(
      "saved_joint_positions_yaml_path",
      plato::storage::default_joint_position_yaml_path("plato_grasp_controller"))),
  plan_config_yaml_path_(this->declare_parameter<std::string>(
      "plan_config_yaml_path",
      plato::yaml::package_source_or_share_file_path(
        "plato_grasp_controller", "config/grasp_plans.yaml"))),
  impedance_preset_yaml_path_(this->declare_parameter<std::string>(
      "impedance_preset_yaml_path",
      plato::yaml::package_source_or_share_file_path(
        "joint_impedance_controller",
        "config/impedance_preset.yaml"))),
  manual_motion_impedance_level_(
    this->declare_parameter<double>("manual_motion_impedance_level", 5.0)),
  task_update_rate_hz_(this->declare_parameter<double>("task_update_rate_hz", 50.0)),
  planner_(std::make_unique<PlatoGraspPlanner>(
      joint_count_, saved_joint_positions_yaml_path_)),
  impedance_handler_(std::make_unique<PlatoGraspImpedanceHandler>(
      joint_count_, impedance_preset_yaml_path_))
{
  std::unordered_map<std::string, GraspTaskConfig> task_configs;
  std::string task_load_error;
  if (!parsing::load_task_configs(
      plan_config_yaml_path_, joint_count_, &task_configs, &task_load_error))
  {
    throw std::runtime_error(task_load_error);
  }
  task_runner_ = std::make_unique<PlatoGraspTaskRunner>(std::move(task_configs));

  trajectory_goal_pub_ =
    this->create_publisher<plato_interfaces::msg::ImpedanceCommands>(
    trajectory_goal_topic_, 10);

  joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PlatoGraspControllerNode::handle_joint_state, this, std::placeholders::_1));

  motion_state_sub_ =
    this->create_subscription<std_msgs::msg::String>(
    motion_state_topic_,
    10,
    std::bind(&PlatoGraspControllerNode::handle_motion_state_command, this, std::placeholders::_1));

  task_sub_ =
    this->create_subscription<std_msgs::msg::String>(
    task_topic_,
    10,
    std::bind(&PlatoGraspControllerNode::handle_task_command, this, std::placeholders::_1));

  save_joint_position_srv_ =
    this->create_service<plato_interfaces::srv::SaveJointPosition>(
    "~/save_joint_position",
    std::bind(
      &PlatoGraspControllerNode::handle_save_joint_position,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  const auto safe_task_update_rate_hz = std::max(task_update_rate_hz_, 1.0);
  const auto task_update_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / safe_task_update_rate_hz));
  task_update_timer_ = this->create_wall_timer(
    task_update_period,
    std::bind(&PlatoGraspControllerNode::handle_task_update, this));

  RCLCPP_INFO(
    this->get_logger(),
    "plato_grasp_controller listening on joint-state '%s', motion-state '%s', task '%s', publishing '%s'",
    joint_state_topic_.c_str(),
    motion_state_topic_.c_str(),
    task_topic_.c_str(),
    trajectory_goal_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Joint position save service available at %s",
    save_joint_position_srv_->get_service_name());
}

void PlatoGraspControllerNode::handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  planner_->update_joint_state(msg->name, msg->position);
}

void PlatoGraspControllerNode::handle_motion_state_command(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto pos_preset_name = trim_copy(msg->data);
  if (pos_preset_name.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty motion-state command.");
    return;
  }

  task_runner_->cancel_task();
  std::string error;
  if (!publish_motion_plan(pos_preset_name, false, manual_motion_impedance_level_, &error)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to apply motion target '%s': %s",
      pos_preset_name.c_str(),
      error.c_str());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Published motion target '%s'. phase=%s",
    pos_preset_name.c_str(),
    phase_to_string(planner_->phase()));
}

void PlatoGraspControllerNode::handle_task_command(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto task_name = trim_copy(msg->data);
  if (task_name.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty task command.");
    return;
  }

  PlatoGraspTaskRunner::Action action;
  std::string error;
  if (!task_runner_->start_task(task_name, &action, &error)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to start task '%s': %s",
      task_name.c_str(),
      error.c_str());
    return;
  }

  if (action.type == PlatoGraspTaskRunner::ActionType::PublishMotionPlan) {
    if (!publish_motion_plan(
        action.pos_preset_name,
        action.use_current_position,
        action.impedance_level,
        &error))
    {
      task_runner_->cancel_task();
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to publish initial motion target '%s' for task '%s': %s",
        action.pos_preset_name.c_str(),
        task_name.c_str(),
        error.c_str());
      return;
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Started task '%s'. state=%s",
    task_name.c_str(),
    task_state_to_string(task_runner_->state()));
}

void PlatoGraspControllerNode::handle_task_update()
{
  const auto now = steady_clock_.now();
  double dt_sec = 0.0;
  if (has_last_task_update_time_) {
    dt_sec = (now - last_task_update_time_).seconds();
  }
  last_task_update_time_ = now;
  has_last_task_update_time_ = true;

  PlatoGraspTaskRunner::Action action;
  std::string error;
  if (!task_runner_->update(dt_sec, &action, &error)) {
    RCLCPP_ERROR(this->get_logger(), "Task update failed: %s", error.c_str());
    task_runner_->cancel_task();
    return;
  }

  switch (action.type) {
    case PlatoGraspTaskRunner::ActionType::None:
      return;
    case PlatoGraspTaskRunner::ActionType::PublishMotionPlan:
    case PlatoGraspTaskRunner::ActionType::PublishMotionHold:
      if (!publish_motion_plan(
          action.pos_preset_name,
          action.use_current_position,
          action.impedance_level,
          &error))
      {
        task_runner_->cancel_task();
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to publish motion target '%s' during task update: %s",
          action.pos_preset_name.c_str(),
          error.c_str());
        return;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Task action -> motion target '%s'. task_state=%s",
        action.pos_preset_name.c_str(),
        task_state_to_string(task_runner_->state()));
      return;
    case PlatoGraspTaskRunner::ActionType::PublishGraspPlan:
      {
        if (!action.grasp_plan.has_value()) {
          task_runner_->cancel_task();
          RCLCPP_ERROR(this->get_logger(), "Task update produced an empty grasp command.");
          return;
        }
        if (!publish_grasp(*action.grasp_plan, action.impedance_level, &error)) {
          task_runner_->cancel_task();
          RCLCPP_ERROR(
            this->get_logger(),
            "Failed to publish grasp command during task update: %s",
            error.c_str());
          return;
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Task action -> grasp. task_state=%s",
          task_state_to_string(task_runner_->state()));
        return;
      }
  }
}

void PlatoGraspControllerNode::handle_save_joint_position(
  const std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Request> request,
  std::shared_ptr<plato_interfaces::srv::SaveJointPosition::Response> response)
{
  response->saved_path = planner_->saved_joint_positions_yaml_path();

  std::string error;
  std::string saved_name;
  const bool success = planner_->save_current_joint_position(
    request->name, &saved_name, &error);

  response->success = success;
  response->saved_name = saved_name;
  response->message = success ? "Saved current joint positions to YAML." : error;

  if (success) {
    RCLCPP_INFO(
      this->get_logger(),
      "Saved joint positions as '%s' to %s",
      response->saved_name.c_str(),
      response->saved_path.c_str());
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to save joint positions to %s: %s",
      response->saved_path.c_str(),
      response->message.c_str());
  }
}

bool PlatoGraspControllerNode::publish_motion_plan(
  const std::string & pos_preset_name,
  bool use_current_position,
  double impedance_level,
  std::string * error_out)
{
  PlatoGraspPlannedCommand planned_command;
  if (use_current_position) {
    if (!planner_->make_current_motion_command(&planned_command, error_out)) {
      return false;
    }
  } else if (!planner_->make_motion_command(pos_preset_name, &planned_command, error_out)) {
    return false;
  }

  if (!impedance_handler_->activate(impedance_level, error_out)) {
    return false;
  }

  plato_interfaces::msg::ImpedanceCommands command;
  if (!impedance_handler_->apply_current_impedance(planned_command, &command, error_out)) {
    return false;
  }

  trajectory_goal_pub_->publish(command);
  return true;
}

bool PlatoGraspControllerNode::publish_grasp(
  const GraspPlanConfig & grasp_plan,
  double impedance_level,
  std::string * error_out)
{
  PlatoGraspPlannedCommand planned_command;
  if (!planner_->make_grasp_command(grasp_plan, &planned_command, error_out)) {
    return false;
  }

  if (!impedance_handler_->activate(impedance_level, error_out)) {
    return false;
  }

  plato_interfaces::msg::ImpedanceCommands command;
  if (!impedance_handler_->apply_current_impedance(planned_command, &command, error_out)) {
    return false;
  }

  trajectory_goal_pub_->publish(command);
  return true;
}

}  // namespace plato_grasp_controller
