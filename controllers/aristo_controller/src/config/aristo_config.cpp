#include "aristo_controller/config/aristo_config.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "mppi_core/config/mppi_config.hpp"
#include "mppi_core/config/rollout_config.hpp"
#include "mppi_core/task/task_config.hpp"

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

Eigen::VectorXd parse_optional_vector_or_scalar(
  const YAML::Node & node,
  const std::string & key,
  const int size)
{
  if (!node || !node[key]) {
    return Eigen::VectorXd{};
  }
  return parse_vector_or_scalar(node, key, size, 0.0);
}

Eigen::Vector3d parse_optional_vector3_or_scalar(
  const YAML::Node & node,
  const std::string & key,
  const Eigen::Vector3d & fallback)
{
  if (!node || !node[key]) {
    return fallback;
  }
  const auto vector = parse_vector_or_scalar(node, key, 3, 0.0);
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
  config.lifecycle.duration = optional_scalar<double>(state, "duration", 0.0);
  config.lifecycle.wait_time = optional_scalar<double>(state, "wait_time", 0.0);
  config.lifecycle.next_state_id =
    optional_scalar<plato_robot_system::StateId>(state, "next_state_id", -1);
  if (state["next_state"]) {
    const auto next_state_name = state["next_state"].as<std::string>();
    const auto next_state = state_by_name(states, next_state_name);
    const auto next_state_id =
      required_scalar<plato_robot_system::StateId>(next_state, "id");
    if (state["next_state_id"] && config.lifecycle.next_state_id != next_state_id) {
      throw std::runtime_error(
        name + ".next_state and next_state_id refer to different states");
    }
    config.lifecycle.next_state_id = next_state_id;
  }
  config.lifecycle.stay_here = optional_scalar<bool>(state, "stay_here", true);
  if (config.lifecycle.duration < 0.0) {
    throw std::runtime_error(name + ".duration must be non-negative");
  }
  if (config.lifecycle.wait_time < 0.0) {
    throw std::runtime_error(name + ".wait_time must be non-negative");
  }
  if (!config.lifecycle.stay_here && config.lifecycle.next_state_id < 0) {
    throw std::runtime_error(name + ".next_state_id must be set when stay_here is false");
  }
  return config;
}

std::string state_name(const YAML::Node & state)
{
  return required_scalar<std::string>(state, "name");
}

JointPositionConfig parse_joint_position_config(
  const YAML::Node & states,
  const std::string & name,
  const int num_joints)
{
  const auto state = state_by_name(states, name);
  const auto params = required_node(state, "params");

  JointPositionConfig config;
  static_cast<StateConfig &>(config) = parse_state_config(states, name);
  config.duration_sec = optional_scalar<double>(state, "duration", 2.0);
  if (config.duration_sec <= 0.0) {
    throw std::runtime_error(name + ".duration must be positive");
  }
  config.lifecycle.duration = config.duration_sec;
  config.target_jpos =
    parse_vector_or_scalar(params, "target_jpos", num_joints, 0.0);
  config.kp_task =
    parse_required_vector_or_scalar(params, "kp_task", num_joints);
  config.kd_task =
    parse_required_vector_or_scalar(params, "kd_task", num_joints);
  return config;
}

void validate_distinct_state_names(const YAML::Node & states)
{
  if (!states || !states.IsSequence()) {
    throw std::runtime_error("state_machine.states must be a YAML sequence");
  }

  std::vector<std::string> names;
  for (const auto & state : states) {
    names.push_back(state_name(state));
  }
  std::sort(names.begin(), names.end());
  if (std::adjacent_find(names.begin(), names.end()) != names.end()) {
    throw std::runtime_error("state_machine state names must be distinct");
  }
}

plato_robot_system::task::GraspTaskConfig parse_grasp_task_config(
  const YAML::Node & params,
  const int num_joints)
{
  plato_robot_system::task::GraspTaskConfig config;

  config.q_ready = parse_optional_vector_or_scalar(params, "q_ready", num_joints);
  if (config.q_ready.size() == 0) {
    config.q_ready = parse_optional_vector_or_scalar(params, "target_jpos", num_joints);
  }
  config.force_enter_debounce_ticks =
    optional_scalar<int>(
      params, "force_enter_debounce_ticks", config.force_enter_debounce_ticks);
  config.force_exit_contact_lost_ticks =
    optional_scalar<int>(
      params, "force_exit_contact_lost_ticks", config.force_exit_contact_lost_ticks);
  config.force_exit_u_threshold =
    optional_scalar<double>(
      params, "force_exit_u_threshold", config.force_exit_u_threshold);
  config.min_contact_force_n =
    optional_scalar<double>(
      params, "min_contact_force_n", config.min_contact_force_n);
  config.use_tactile_presence_for_contact =
    optional_scalar<bool>(
      params, "use_tactile_presence_for_contact",
      config.use_tactile_presence_for_contact);
  config.force_feedback_enabled =
    optional_scalar<bool>(
      params, "force_feedback_enabled", config.force_feedback_enabled);
  config.lpf_alpha =
    optional_scalar<double>(
      params, "lpf_alpha", config.lpf_alpha);
  config.kp_tactile_u_fb =
    optional_scalar<double>(params, "kp_tactile_fb", config.kp_tactile_u_fb);
  config.kd_tactile_u_fb =
    optional_scalar<double>(params, "kd_tactile_fb", config.kd_tactile_u_fb);
  config.kp_tactile_u_fb =
    optional_scalar<double>(params, "kp_tactile_u_fb", config.kp_tactile_u_fb);
  config.kd_tactile_u_fb =
    optional_scalar<double>(params, "kd_tactile_u_fb", config.kd_tactile_u_fb);
  config.kp_tactile_phi_fb =
    optional_scalar<double>(params, "kp_tactile_phi_fb", config.kp_tactile_phi_fb);
  config.kd_tactile_phi_fb =
    optional_scalar<double>(params, "kd_tactile_phi_fb", config.kd_tactile_phi_fb);
  config.parallel_tip_radius_m =
    optional_scalar<double>(
      params, "parallel_tip_radius_m", config.parallel_tip_radius_m);
  config.parallel_lateral_offset_m =
    optional_scalar<double>(
      params, "parallel_lateral_offset_m", config.parallel_lateral_offset_m);
  config.parallel_qmin_rad =
    optional_scalar<double>(
      params, "parallel_qmin_rad", config.parallel_qmin_rad);
  config.parallel_qmax_rad =
    optional_scalar<double>(
      params, "parallel_qmax_rad", config.parallel_qmax_rad);
  config.parallel_q5_min_rad =
    optional_scalar<double>(
      params, "parallel_q5_min_rad", config.parallel_q5_min_rad);
  config.parallel_midpoint_u =
    optional_scalar<double>(
      params, "parallel_midpoint_u", config.parallel_midpoint_u);
  config.parallel_max_flexion_rad =
    optional_scalar<double>(
      params, "parallel_max_flexion_rad", config.parallel_max_flexion_rad);

  return config;
}

aristo_controller::state_machines::GraspTeleopStateConfig parse_grasp_teleop_state_config(
  const YAML::Node & params,
  const int num_joints)
{
  aristo_controller::state_machines::GraspTeleopStateConfig config;

  config.default_u =
    optional_scalar<double>(params, "default_u", config.default_u);
  config.default_phi =
    optional_scalar<double>(params, "default_phi", config.default_phi);
  config.default_desired_force_n =
    optional_scalar<double>(
      params, "default_desired_force_n", config.default_desired_force_n);
  config.shared_grasp_control =
    optional_scalar<bool>(
      params,
      "handoff_to_mppi_on_force_ready",
      config.shared_grasp_control);
  config.shared_grasp_control =
    optional_scalar<bool>(
      params,
      "handoff_on_force_ready",
      config.shared_grasp_control);
  config.shared_grasp_control =
    optional_scalar<bool>(
      params,
      "shared_grasp_control",
      config.shared_grasp_control);
  const auto grasp_task = required_node(params, "grasp_task");
  config.grasp_task = parse_grasp_task_config(grasp_task, num_joints);
  config.grasp_task.force_feedback_enabled = false;
  config.shared_control_min_contact_sensors =
    optional_scalar<int>(
      params,
      "shared_control_min_contact_sensors",
      config.shared_control_min_contact_sensors);
  config.shared_control_enter_debounce_ticks =
    optional_scalar<int>(
      params,
      "shared_control_enter_debounce_ticks",
      config.grasp_task.force_enter_debounce_ticks);
  config.shared_control_requires_u_below_threshold =
    optional_scalar<bool>(
      params,
      "shared_control_requires_u_below_threshold",
      config.shared_control_requires_u_below_threshold);
  config.shared_control_requires_enough_contact =
    optional_scalar<bool>(
      params,
      "shared_control_requires_enough_contact",
      config.shared_control_requires_enough_contact);

  return config;
}

aristo_controller::state_machines::GraspForceStateConfig parse_grasp_force_state_config(
  const YAML::Node & params,
  const int num_joints)
{
  aristo_controller::state_machines::GraspForceStateConfig config;

  config.default_u =
    optional_scalar<double>(params, "default_u", config.default_u);
  config.default_phi =
    optional_scalar<double>(params, "default_phi", config.default_phi);
  config.default_desired_force_n =
    optional_scalar<double>(
      params, "default_desired_force_n", config.default_desired_force_n);
  const auto grasp_task = required_node(params, "grasp_task");
  config.grasp_task = parse_grasp_task_config(grasp_task, num_joints);

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

aristo_controller::state_machines::MPPIGraspStateConfig parse_mppi_grasp_state_config(
  const YAML::Node & params,
  const int num_joints)
{
  if (params && !params.IsMap()) {
    throw std::runtime_error("mppi_grasp.params must be a map when provided");
  }

  aristo_controller::state_machines::MPPIGraspStateConfig config;
  const auto mppi_params = params ? params["mppi"] : YAML::Node();
  const auto rollout_params = params ? params["rollout"] : YAML::Node();
  const auto task_params = params ? params["task"] : YAML::Node();
  const auto safety_params = params ? params["safety"] : YAML::Node();
  const auto tactile_params = params ? params["tactile"] : YAML::Node();
  config.mppi = mppi_core::ParseMPPIConfig(mppi_params, num_joints, config.mppi);
  config.rollout = mppi_core::ParseGraspStateRolloutConfig(rollout_params, config.rollout);
  config.tactile_transition =
    mppi_core::ParseTactileTransitionConfig(rollout_params, config.tactile_transition);
  config.task = mppi_core::ParseTaskConfig(task_params, config.task);
  config.tactile_transition =
    mppi_core::ApplyTaskToleranceToTransitionConfig(config.task, config.tactile_transition);
  config.safety.max_reference_tracking_error_rad =
    optional_scalar<double>(
      safety_params,
      "max_reference_tracking_error_rad",
      config.safety.max_reference_tracking_error_rad);
  config.safety.max_qdot_cmd_rad_s =
    optional_scalar<double>(
      safety_params,
      "max_qdot_cmd_rad_s",
      config.safety.max_qdot_cmd_rad_s);
  config.safety.max_tau_cmd_nm =
    optional_scalar<double>(
      safety_params,
      "max_tau_cmd_nm",
      config.safety.max_tau_cmd_nm);
  config.safety.max_tau_rate_nm_s =
    optional_scalar<double>(
      safety_params,
      "max_tau_rate_nm_s",
      config.safety.max_tau_rate_nm_s);
  config.safety.clamp_q_cmd_to_model_limits =
    optional_scalar<bool>(
      safety_params,
      "clamp_q_cmd_to_model_limits",
      config.safety.clamp_q_cmd_to_model_limits);
  config.tactile.thumb_normal_axis_sign =
    optional_scalar<double>(
      tactile_params,
      "thumb_normal_axis_sign",
      config.tactile.thumb_normal_axis_sign);
  config.tactile.index_normal_axis_sign =
    optional_scalar<double>(
      tactile_params,
      "index_normal_axis_sign",
      config.tactile.index_normal_axis_sign);
  return config;
}

aristo_controller::state_machines::MPPIMotionGraspStateConfig
parse_mppi_motion_grasp_state_config(
  const YAML::Node & params,
  const int num_joints)
{
  if (params && !params.IsMap()) {
    throw std::runtime_error("mppi_motion_grasp.params must be a map when provided");
  }

  aristo_controller::state_machines::MPPIMotionGraspStateConfig config;
  config.enabled = optional_scalar<bool>(params, "enabled", config.enabled);

  const auto command_params = params ? params["command"] : YAML::Node();
  config.default_u =
    optional_scalar<double>(command_params, "u_open", config.default_u);
  config.default_phi =
    optional_scalar<double>(command_params, "phi", config.default_phi);
  config.default_desired_force_n =
    optional_scalar<double>(
      command_params,
      "desired_force_n",
      config.default_desired_force_n);

  const auto initiation_params = params ? params["initiation"] : YAML::Node();
  config.initiation.contact_enter_debounce_ticks =
    optional_scalar<int>(
      initiation_params,
      "contact_enter_debounce_ticks",
      config.initiation.contact_enter_debounce_ticks);
  config.initiation.contact_exit_debounce_ticks =
    optional_scalar<int>(
      initiation_params,
      "contact_exit_debounce_ticks",
      config.initiation.contact_exit_debounce_ticks);
  config.initiation.min_contact_force_n =
    optional_scalar<double>(
      initiation_params,
      "min_contact_force_n",
      config.initiation.min_contact_force_n);
  config.initiation.use_tactile_presence_for_contact =
    optional_scalar<bool>(
      initiation_params,
      "use_tactile_presence_for_contact",
      config.initiation.use_tactile_presence_for_contact);
  config.initiation.contacted_finger_hold_weight =
    optional_scalar<double>(
      initiation_params,
      "contacted_finger_hold_weight",
      config.initiation.contacted_finger_hold_weight);
  config.initiation.moving_finger_target_weight =
    optional_scalar<double>(
      initiation_params,
      "moving_finger_target_weight",
      config.initiation.moving_finger_target_weight);
  config.initiation.posture_weight =
    optional_scalar<double>(
      initiation_params,
      "posture_weight",
      config.initiation.posture_weight);
  config.initiation.max_reference_tracking_error_rad =
    optional_scalar<double>(
      initiation_params,
      "max_reference_tracking_error_rad",
      config.initiation.max_reference_tracking_error_rad);

  const auto safety_params = params ? params["safety"] : YAML::Node();
  config.safety.max_velocity_rad_s =
    optional_scalar<double>(
      safety_params,
      "max_velocity_rad_s",
      config.safety.max_velocity_rad_s);
  config.safety.max_torque_nm =
    optional_scalar<double>(
      safety_params,
      "max_torque_nm",
      config.safety.max_torque_nm);
  config.safety.max_torque_rate_nm_per_s =
    optional_scalar<double>(
      safety_params,
      "max_torque_rate_nm_per_s",
      config.safety.max_torque_rate_nm_per_s);

  const auto mppi_params = params ? params["mppi"] : YAML::Node();
  config.mppi = mppi_core::ParseMPPIConfig(mppi_params, num_joints, config.mppi);

  const auto cost_params = params ? params["cost"] : YAML::Node();
  config.cost.q_target_weight =
    optional_scalar<double>(
      cost_params, "q_target_weight", config.cost.q_target_weight);
  config.cost.qdot_target_weight =
    optional_scalar<double>(
      cost_params, "qdot_target_weight", config.cost.qdot_target_weight);
  config.cost.qddot_weight =
    optional_scalar<double>(
      cost_params, "qddot_weight", config.cost.qddot_weight);
  config.cost.tau_weight =
    optional_scalar<double>(
      cost_params, "tau_weight", config.cost.tau_weight);
  config.cost.joint_limit_weight =
    optional_scalar<double>(
      cost_params, "joint_limit_weight", config.cost.joint_limit_weight);
  config.cost.joint_limit_margin_rad =
    optional_scalar<double>(
      cost_params, "joint_limit_margin_rad", config.cost.joint_limit_margin_rad);
  config.cost.enable_line_of_action_cost =
    optional_scalar<bool>(
      cost_params,
      "enable_line_of_action_cost",
      config.cost.enable_line_of_action_cost);
  config.cost.line_of_action_weight =
    optional_scalar<double>(
      cost_params, "line_of_action_weight", config.cost.line_of_action_weight);
  config.cost.frame_a_name =
    optional_scalar<std::string>(
      cost_params, "frame_a_name", config.cost.frame_a_name);
  config.cost.frame_b_name =
    optional_scalar<std::string>(
      cost_params, "frame_b_name", config.cost.frame_b_name);
  config.cost.close_axis_base =
    parse_optional_vector3_or_scalar(
      cost_params,
      "close_axis_base",
      config.cost.close_axis_base);

  const auto grasp_task = params ? params["grasp_task"] : YAML::Node();
  if (grasp_task) {
    config.grasp_task = parse_grasp_task_config(grasp_task, num_joints);
  }
  config.grasp_task.force_feedback_enabled = false;
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
  validate_distinct_state_names(states);
  config.idle = parse_state_config(states, "idle");
  static_cast<StateConfig &>(config.mppi_grasp) = parse_state_config(states, "mppi_grasp");
  const auto mppi_grasp = state_by_name(states, "mppi_grasp");
  config.mppi_grasp.state =
    parse_mppi_grasp_state_config(mppi_grasp["params"], config.num_joints);
  static_cast<StateConfig &>(config.mppi_motion_grasp) =
    parse_state_config(states, "mppi_motion_grasp");
  const auto mppi_motion_grasp = state_by_name(states, "mppi_motion_grasp");
  config.mppi_motion_grasp.state =
    parse_mppi_motion_grasp_state_config(mppi_motion_grasp["params"], config.num_joints);
  config.initialize = parse_joint_position_config(states, "initialize", config.num_joints);
  config.poke = parse_joint_position_config(states, "poke", config.num_joints);
  config.grasp_ready =
    parse_joint_position_config(states, "grasp_ready", config.num_joints);

  const auto joint_teleop = state_by_name(states, "joint_teleop");
  config.joint_teleop.id =
    required_scalar<plato_robot_system::StateId>(joint_teleop, "id");
  config.joint_teleop.state =
    parse_joint_teleop_state_config(required_node(joint_teleop, "params"), config.num_joints);

  const auto grasp_teleop = state_by_name(states, "grasp_teleop");
  static_cast<StateConfig &>(config.grasp_teleop) = parse_state_config(states, "grasp_teleop");
  config.grasp_teleop.state =
    parse_grasp_teleop_state_config(grasp_teleop["params"], config.num_joints);
  if (config.grasp_teleop.state.grasp_task.q_ready.size() == 0) {
    config.grasp_teleop.state.grasp_task.q_ready = config.grasp_ready.target_jpos;
  }

  const auto grasp_force = state_by_name(states, "grasp_force");
  static_cast<StateConfig &>(config.grasp_force) = parse_state_config(states, "grasp_force");
  config.grasp_force.state =
    parse_grasp_force_state_config(grasp_force["params"], config.num_joints);
  if (config.grasp_force.state.grasp_task.q_ready.size() == 0) {
    config.grasp_force.state.grasp_task.q_ready = config.grasp_ready.target_jpos;
  }
  if (config.mppi_motion_grasp.state.grasp_task.q_ready.size() == 0) {
    config.mppi_motion_grasp.state.grasp_task.q_ready = config.grasp_ready.target_jpos;
  }

  validate_distinct_state_ids(
    {config.idle.id, config.initialize.id, config.poke.id, config.grasp_ready.id,
      config.joint_teleop.id, config.grasp_teleop.id, config.grasp_force.id,
      config.mppi_grasp.id, config.mppi_motion_grasp.id});

  return config;
}

}  // namespace aristo_controller::config
