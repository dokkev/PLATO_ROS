#include "aristo_controller/config/aristo_config.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace aristo_controller::config
{
namespace
{

template<typename T>
T optional_scalar(const YAML::Node & node, const std::string & key, const T & fallback)
{
  if (!node || !node[key]) {
    return fallback;
  }
  return node[key].as<T>();
}

YAML::Node required_node(const YAML::Node & node, const std::string & key)
{
  if (!node || !node[key]) {
    throw std::runtime_error("Missing required YAML key '" + key + "'");
  }
  return node[key];
}

template<typename T>
T required_scalar(const YAML::Node & node, const std::string & key)
{
  if (!node || !node[key]) {
    throw std::runtime_error("Missing required YAML key '" + key + "'");
  }
  if (!node[key].IsScalar()) {
    throw std::runtime_error("YAML key '" + key + "' must be a scalar");
  }
  return node[key].as<T>();
}

YAML::Node state_by_name(const YAML::Node & states, const std::string & name)
{
  if (!states || !states.IsSequence()) {
    throw std::runtime_error("state_machine.states must be a YAML sequence");
  }

  for (const auto & state : states) {
    if (state["name"] && state["name"].as<std::string>() == name) {
      return state;
    }
  }
  throw std::runtime_error("state_machine.states does not define state '" + name + "'");
}

Eigen::VectorXd scalar_vector(double value, int size)
{
  return Eigen::VectorXd::Constant(size, value);
}

Eigen::VectorXd parse_vector_or_scalar(
  const YAML::Node & node,
  const std::string & key,
  int size,
  double fallback)
{
  if (!node || !node[key]) {
    return scalar_vector(fallback, size);
  }

  const auto value = node[key];
  if (value.IsScalar()) {
    return scalar_vector(value.as<double>(), size);
  }
  if (!value.IsSequence()) {
    throw std::runtime_error("YAML key '" + key + "' must be a scalar or sequence");
  }
  if (static_cast<int>(value.size()) != size) {
    throw std::runtime_error(
      "YAML key '" + key + "' must have " + std::to_string(size) +
      " entries when specified as a sequence");
  }

  Eigen::VectorXd vector(size);
  for (int i = 0; i < size; ++i) {
    vector[i] = value[static_cast<std::size_t>(i)].as<double>();
  }
  return vector;
}

Eigen::Vector3d parse_vector3_or_scalar(
  const YAML::Node & node,
  const std::string & key,
  const Eigen::Vector3d & fallback)
{
  if (!node || !node[key]) {
    return fallback;
  }

  const Eigen::VectorXd vector = parse_vector_or_scalar(node, key, 3, 0.0);
  return Eigen::Vector3d{vector[0], vector[1], vector[2]};
}

Eigen::VectorXd parse_required_vector_or_scalar(
  const YAML::Node & node,
  const std::string & key,
  int size)
{
  if (!node || !node[key]) {
    throw std::runtime_error("Missing required YAML key '" + key + "'");
  }
  return parse_vector_or_scalar(node, key, size, 0.0);
}

StateConfig parse_state_config(const YAML::Node & states, const std::string & name)
{
  const auto state = state_by_name(states, name);
  StateConfig config;
  config.id = required_scalar<plato_robot_system::StateId>(state, "id");
  return config;
}

plato_robot_system::task::GraspTaskConfig parse_grasp_task_config(
  const YAML::Node & params)
{
  plato_robot_system::task::GraspTaskConfig config;
  const int active_dof =
    static_cast<int>(plato_robot_system::task::kThumbIndexActiveJoints.size());

  config.distance_closed_m =
    optional_scalar<double>(params, "distance_closed_m", config.distance_closed_m);
  config.distance_open_m =
    optional_scalar<double>(params, "distance_open_m", config.distance_open_m);
  config.force_exit_u_threshold =
    optional_scalar<double>(
      params, "force_exit_u_threshold", config.force_exit_u_threshold);
  config.q_posture =
    parse_vector_or_scalar(params, "q_posture", active_dof, 0.0);
  config.fallback_close_axis_base =
    parse_vector3_or_scalar(
      params, "fallback_close_axis_base", config.fallback_close_axis_base);
  config.kp_task =
    optional_scalar<double>(params, "kp_task", config.kp_task);
  config.kd_task =
    optional_scalar<double>(params, "kd_task", config.kd_task);
  config.lateral_offset_limit_m =
    optional_scalar<double>(
      params, "lateral_offset_limit_m", config.lateral_offset_limit_m);
  config.kp_lateral =
    optional_scalar<double>(params, "kp_lateral", config.kp_lateral);
  config.kd_lateral =
    optional_scalar<double>(params, "kd_lateral", config.kd_lateral);
  config.kp_tactile_fb =
    optional_scalar<double>(params, "kp_tactile_fb", config.kp_tactile_fb);
  config.kd_tactile_fb =
    optional_scalar<double>(params, "kd_tactile_fb", config.kd_tactile_fb);
  config.w_task_motion =
    optional_scalar<double>(params, "w_task_motion", config.w_task_motion);
  config.w_task_tactile_mode =
    optional_scalar<double>(params, "w_task_tactile_mode", config.w_task_tactile_mode);
  config.w_lateral =
    optional_scalar<double>(params, "w_lateral", config.w_lateral);
  config.w_tactile =
    optional_scalar<double>(params, "w_tactile", config.w_tactile);
  config.w_posture =
    optional_scalar<double>(params, "w_posture", config.w_posture);
  config.damping_qp =
    optional_scalar<double>(params, "damping_qp", config.damping_qp);
  config.max_qddot_rad_s2 =
    optional_scalar<double>(params, "max_qddot_rad_s2", config.max_qddot_rad_s2);
  config.max_velocity_rad_s =
    optional_scalar<double>(params, "max_velocity_rad_s", config.max_velocity_rad_s);
  config.max_torque_nm =
    optional_scalar<double>(params, "max_torque_nm", config.max_torque_nm);

  return config;
}

aristo_controller::state_machines::GraspTeleopStateConfig parse_grasp_teleop_state_config(
  const YAML::Node & params)
{
  aristo_controller::state_machines::GraspTeleopStateConfig config;

  config.default_u_close =
    optional_scalar<double>(params, "default_u_close", config.default_u_close);
  config.default_u_lateral =
    optional_scalar<double>(params, "default_u_lateral", config.default_u_lateral);
  config.default_desired_force_n =
    optional_scalar<double>(
      params, "default_desired_force_n", config.default_desired_force_n);
  const auto grasp_task = required_node(params, "grasp_task");
  config.grasp_task = parse_grasp_task_config(grasp_task);

  return config;
}

aristo_controller::state_machines::JointTeleopStateConfig parse_joint_teleop_state_config(
  const YAML::Node & params,
  const int num_joints)
{
  aristo_controller::state_machines::JointTeleopStateConfig config;

  config.joint_task.kp_task =
    parse_required_vector_or_scalar(params, "kp_task", num_joints);
  config.joint_task.kd_task =
    parse_required_vector_or_scalar(params, "kd_task", num_joints);
  config.joint_task.lpf_alpha =
    optional_scalar<double>(
      params,
      "lpf_alpha",
      config.joint_task.lpf_alpha);
  if (
    !std::isfinite(config.joint_task.lpf_alpha) ||
    config.joint_task.lpf_alpha < 0.0 ||
    config.joint_task.lpf_alpha > 1.0)
  {
    throw std::runtime_error(
      "joint_teleop lpf_alpha must be finite and in [0, 1]");
  }

  return config;
}

void validate_distinct_state_ids(std::vector<plato_robot_system::StateId> ids)
{
  std::sort(ids.begin(), ids.end());
  if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
    throw std::runtime_error("state_machine state ids must be distinct");
  }
}

}  // namespace

std::string default_aristo_config_path()
{
  return ament_index_cpp::get_package_share_directory("aristo_controller") +
         "/config/aristo.yaml";
}

AristoConfig load_aristo_config(const std::string & yaml_path)
{
  const auto root = YAML::LoadFile(yaml_path);
  AristoConfig config;

  const auto robot_model = required_node(root, "robot_model");
  config.robot_model.urdf_path = required_scalar<std::string>(robot_model, "urdf_path");
  if (config.robot_model.urdf_path.empty()) {
    throw std::runtime_error("robot_model.urdf_path must not be empty");
  }
  config.robot_model.is_floating_base =
    optional_scalar<bool>(robot_model, "is_floating_base", false);
  config.robot_model.base_frame = optional_scalar<std::string>(robot_model, "base_frame", "");
  config.robot_model.fixed_thumb = optional_scalar<bool>(robot_model, "fixed_thumb", false);

  const auto controller = required_node(root, "controller");
  config.num_joints = optional_scalar<int>(controller, "num_joints", 8);
  if (config.num_joints <= 0) {
    throw std::runtime_error("controller.num_joints must be positive");
  }

  const auto debug = root["debug"];
  config.debug_enabled = optional_scalar<bool>(debug, "enabled", false);

  const auto driver_gains = required_node(root, "driver_gains");
  config.driver_gains.kp = parse_vector_or_scalar(driver_gains, "kp", config.num_joints, 0.0);
  config.driver_gains.kd = parse_vector_or_scalar(driver_gains, "kd", config.num_joints, 0.0);
  if (
    !config.driver_gains.IsValid() ||
    !config.driver_gains.HasValidDimensions(config.num_joints))
  {
    throw std::runtime_error("driver_gains produced invalid driver PD gains");
  }

  const auto state_machine = required_node(root, "state_machine");
  const auto states = required_node(state_machine, "states");
  config.idle = parse_state_config(states, "idle");
  config.mppi_grasp = parse_state_config(states, "mppi_grasp");

  const auto joint_teleop = state_by_name(states, "joint_teleop");
  config.joint_teleop.id =
    required_scalar<plato_robot_system::StateId>(joint_teleop, "id");
  config.joint_teleop.state =
    parse_joint_teleop_state_config(required_node(joint_teleop, "params"), config.num_joints);

  const auto grasp_teleop = state_by_name(states, "grasp_teleop");
  config.grasp_teleop.id =
    required_scalar<plato_robot_system::StateId>(grasp_teleop, "id");
  config.grasp_teleop.state =
    parse_grasp_teleop_state_config(grasp_teleop["params"]);

  const auto initialize = state_by_name(states, "initialize");
  const auto initialize_params = required_node(initialize, "params");

  config.initialize.id = required_scalar<plato_robot_system::StateId>(initialize, "id");
  validate_distinct_state_ids(
    {config.idle.id, config.initialize.id, config.joint_teleop.id,
      config.grasp_teleop.id, config.mppi_grasp.id});
  config.initialize.duration_sec = optional_scalar<double>(initialize, "duration", 2.0);
  if (config.initialize.duration_sec <= 0.0) {
    throw std::runtime_error("initialize.duration must be positive");
  }
  config.initialize.target_jpos =
    parse_vector_or_scalar(initialize_params, "target_jpos", config.num_joints, 0.0);
  config.initialize.kp_task =
    parse_required_vector_or_scalar(initialize_params, "kp_task", config.num_joints);
  config.initialize.kd_task =
    parse_required_vector_or_scalar(initialize_params, "kd_task", config.num_joints);

  return config;
}

}  // namespace aristo_controller::config
