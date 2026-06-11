#include "plato_ros_controller/plato_ros_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "aristo_controller/state_machines/grasp_force.hpp"
#include "aristo_controller/state_machines/grasp_ready.hpp"
#include "aristo_controller/state_machines/grasp_teleop.hpp"
#include "aristo_controller/state_machines/idle.hpp"
#include "aristo_controller/state_machines/initialize.hpp"
#include "aristo_controller/state_machines/joint_teleop.hpp"
#include "aristo_controller/state_machines/mppi_grasp.hpp"
#include "aristo_controller/state_machines/poke.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "plato_robot_system/sensor/nari_touch_adapter.hpp"
#include "rclcpp/qos.hpp"

namespace plato_ros_controller
{
namespace
{

// Aristo ros2_control effort-like interfaces use mNm, while RobotSystem and
// RobotCommand use SI units internally.
constexpr double kMNmToNm = 1.0e-3;
constexpr double kNmToMNm = 1.0e3;

void set_command_interface_value(
  hardware_interface::LoanedCommandInterface & command_interface,
  const double value)
{
  (void)command_interface.set_value(value);
}

double read_state_interface_value(const hardware_interface::LoanedStateInterface & state_interface)
{
  return state_interface.get_value();
}

std::vector<std::string> default_tactile_topics()
{
  return {};
}

std::string resolve_package_url(const std::string & path)
{
  const std::string prefix = "package://";
  if (path.rfind(prefix, 0) != 0) {
    return path;
  }

  const auto package_and_path = path.substr(prefix.size());
  const auto slash = package_and_path.find('/');
  if (slash == std::string::npos || slash == 0 || slash + 1 >= package_and_path.size()) {
    throw std::runtime_error("Invalid package URL: " + path);
  }

  const auto package_name = package_and_path.substr(0, slash);
  const auto relative_path = package_and_path.substr(slash + 1);
  return ament_index_cpp::get_package_share_directory(package_name) + "/" + relative_path;
}

std::string load_robot_model_from_config(
  const aristo_controller::config::RobotModelConfig & config,
  plato_robot_system::RobotSystem & robot)
{
  const auto resolved_urdf_path = resolve_package_url(config.urdf_path);
  if (config.is_floating_base) {
    robot.LoadUrdf(resolved_urdf_path, pinocchio::JointModelFreeFlyer());
  } else {
    robot.LoadUrdf(resolved_urdf_path);
  }
  return resolved_urdf_path;
}

}  // namespace

PlatoRosController::PlatoRosController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn PlatoRosController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<std::vector<std::string>>("tactile_topics", default_tactile_topics());
  auto_declare<std::string>("joint_teleop_command_topic", "~/joint_teleop");
  auto_declare<std::string>(
    "grasp_teleop_command_topic",
    "/plato2/parallel_grasp_controller/commands");
  auto_declare<std::string>(
    "grasp_force_reference_topic",
    "/grasp_force_reference/target_normal_force_n");
  auto_declare<std::string>(
    "grasp_force_reference_valid_topic",
    "/grasp_force_reference/reference_valid");
  auto_declare<std::string>("control_config_yaml_path", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PlatoRosController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  tactile_topics_ = get_node()->get_parameter("tactile_topics").as_string_array();
  joint_teleop_command_topic_ =
    get_node()->get_parameter("joint_teleop_command_topic").as_string();
  grasp_teleop_command_topic_ =
    get_node()->get_parameter("grasp_teleop_command_topic").as_string();
  grasp_force_reference_topic_ =
    get_node()->get_parameter("grasp_force_reference_topic").as_string();
  grasp_force_reference_valid_topic_ =
    get_node()->get_parameter("grasp_force_reference_valid_topic").as_string();
  control_config_yaml_path_ = get_node()->get_parameter("control_config_yaml_path").as_string();
  joint_teleop_state_ = nullptr;
  grasp_teleop_state_ = nullptr;
  grasp_force_state_ = nullptr;
  grasp_force_reference_n_.store(0.0);
  grasp_force_reference_valid_.store(false);

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!tactile_topics_.empty() && tactile_topics_.size() != kTactileFrameNames.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'tactile_topics' must be empty or contain exactly %zu topics for the hardcoded tactile frames",
      kTactileFrameNames.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t num_joints = joint_names_.size();
  positions_.assign(num_joints, 0.0);
  velocities_.assign(num_joints, 0.0);
  efforts_.assign(num_joints, 0.0);

  robot_ = std::make_shared<plato_robot_system::RobotSystem>();
  control_architecture_.SetRobot(robot_);
  if (!control_config_yaml_path_.empty()) {
    if (!configure_from_control_config(control_architecture_, *robot_)) {
      return controller_interface::CallbackReturn::ERROR;
    }
  } else {
    control_architecture_.Configure(static_cast<int>(num_joints), static_cast<int>(num_joints));
    configure_identity_joint_mapping(num_joints);
    if (configure_control_architecture(control_architecture_) !=
      controller_interface::CallbackReturn::SUCCESS)
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    register_robot_states(control_architecture_, *robot_);
    configure_initial_state(control_architecture_);
  }
  control_architecture_.Initialize();
  model_q_.setZero(control_architecture_.nq());
  model_qdot_.setZero(control_architecture_.nv());
  model_tau_.setZero(control_architecture_.nv());
  filtered_command_.Resize(control_architecture_.nq(), control_architecture_.nv());
  filtered_command_.valid = false;

  nari_touch_.clear();
  nari_touch_.reserve(tactile_topics_.size());
  tactile_stream_seen_.assign(tactile_topics_.size(), false);
  auto tactile_vector = std::make_shared<TactileSensorVector>();
  tactile_vector->resize(tactile_topics_.size());
  for (std::size_t i = 0; i < tactile_topics_.size(); ++i) {
    nari_touch_.emplace_back(static_cast<int>(i), tactile_frame_name(i));
    (*tactile_vector)[i] =
      plato_robot_system::sensor::ConvertNARITouchToTactileState(nari_touch_[i].sample());
  }
  rt_tactile_ptr_.writeFromNonRT(tactile_vector);
  rt_joint_teleop_command_ptr_.writeFromNonRT(std::shared_ptr<JointTeleopCommand>{});
  rt_grasp_teleop_command_ptr_.writeFromNonRT(std::shared_ptr<GraspTeleopCommand>{});

  tactile_subs_.clear();
  tactile_subs_.reserve(tactile_topics_.size());
  for (std::size_t i = 0; i < tactile_topics_.size(); ++i) {
    tactile_subs_.push_back(get_node()->create_subscription<TactileMsg>(
      tactile_topics_[i], rclcpp::SensorDataQoS(),
      [this, i](const TactileMsg::SharedPtr msg) { tactile_callback(i, msg); }));
  }
  if (!joint_teleop_command_topic_.empty()) {
    joint_teleop_command_sub_ = get_node()->create_subscription<JointTeleopMsg>(
      joint_teleop_command_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const JointTeleopMsg::SharedPtr msg) {
        joint_teleop_command_callback(msg);
      });
  }
  if (!grasp_teleop_command_topic_.empty()) {
    grasp_teleop_command_sub_ = get_node()->create_subscription<GraspTeleopMsg>(
      grasp_teleop_command_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const GraspTeleopMsg::SharedPtr msg) {
        grasp_teleop_command_callback(msg);
      });
  }
  if (!grasp_force_reference_topic_.empty()) {
    grasp_force_reference_sub_ = get_node()->create_subscription<GraspForceReferenceMsg>(
      grasp_force_reference_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const GraspForceReferenceMsg::SharedPtr msg) {
        grasp_force_reference_callback(msg);
      });
  }
  if (!grasp_force_reference_valid_topic_.empty()) {
    grasp_force_reference_valid_sub_ =
      get_node()->create_subscription<GraspForceReferenceValidMsg>(
        grasp_force_reference_valid_topic_,
        rclcpp::SystemDefaultsQoS(),
        [this](const GraspForceReferenceValidMsg::SharedPtr msg) {
          grasp_force_reference_valid_callback(msg);
        });
  }
  controller_state_pub_ =
    get_node()->create_publisher<plato_interfaces::msg::ImpedanceControllerState>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
  request_state_srv_ = get_node()->create_service<RequestStateSrv>(
    "~/request_state",
    [this](
      const std::shared_ptr<RequestStateSrv::Request> request,
      std::shared_ptr<RequestStateSrv::Response> response)
    {
      request_state_callback(request, response);
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured Plato ROS controller with %zu joints, %zu tactile topics, joint teleop topic '%s', grasp teleop topic '%s', and grasp force reference topics '%s'/'%s'",
    num_joints,
    tactile_topics_.size(),
    joint_teleop_command_topic_.c_str(),
    grasp_teleop_command_topic_.c_str(),
    grasp_force_reference_topic_.c_str(),
    grasp_force_reference_valid_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PlatoRosController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!assign_state_interfaces()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  read_state_interfaces();
  if (!assign_command_interfaces()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Activated Plato ROS controller");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PlatoRosController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    set_command_interface_value(position_command_interfaces_[i].get(), 0.0);
    set_command_interface_value(velocity_command_interfaces_[i].get(), 0.0);
    set_command_interface_value(effort_command_interfaces_[i].get(), 0.0);
    set_command_interface_value(stiffness_command_interfaces_[i].get(), 0.0);
    set_command_interface_value(damping_command_interfaces_[i].get(), 0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PlatoRosController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  read_state_interfaces();

  const auto tactile_sensors = current_tactile_sensors();
  const auto num_joints = static_cast<Eigen::Index>(joint_names_.size());

  if (robot_ && robot_->hasModel()) {
    model_q_ = robot_->state().q;
  } else {
    model_q_.setZero();
  }
  model_qdot_.setZero();
  model_tau_.setZero();
  for (Eigen::Index i = 0; i < num_joints; ++i) {
    const auto joint_index = static_cast<std::size_t>(i);
    model_q_[model_q_indices_[joint_index]] = positions_[joint_index];
    model_qdot_[model_v_indices_[joint_index]] = velocities_[joint_index];
    model_tau_[model_v_indices_[joint_index]] = efforts_[joint_index];
  }

  auto robot_state = plato_robot_system::MakeRobotState(
    model_q_, model_qdot_, model_tau_, tactile_sensors, time.seconds());
  if (!plato_robot_system::IsValid(robot_state)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Robot state contains non-finite values");
    write_zero_command();
    return controller_interface::return_type::ERROR;
  }

  apply_pending_state_request();
  sync_joint_teleop_input();
  sync_grasp_teleop_input();

  plato_robot_system::ControlUpdateResult control_result;
  try {
    control_result = control_architecture_.Update(robot_state, period.seconds());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Control architecture update failed: %s", e.what());
    if (!write_safe_hold_command(robot_state)) {
      write_zero_command();
    }
    return controller_interface::return_type::ERROR;
  }
  if (!control_result.ok) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Control architecture update failed: %s", control_result.reason.c_str());
    if (!write_safe_hold_command(robot_state)) {
      write_zero_command();
    }
    return controller_interface::return_type::ERROR;
  }
  const auto & command = control_architecture_.command();
  if (!command.IsUsable()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Control architecture produced an invalid command");
    if (!write_safe_hold_command(robot_state)) {
      write_zero_command();
    }
    return controller_interface::return_type::ERROR;
  }

  filtered_command_ = command;
  filter_command(&filtered_command_);
  if (!filtered_command_.IsUsable()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Controller command filter produced an invalid command");
    if (!write_safe_hold_command(robot_state)) {
      write_zero_command();
    }
    return controller_interface::return_type::ERROR;
  }
  publish_controller_state(time, filtered_command_);
  write_command(filtered_command_);

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration PlatoRosController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    config.names.push_back(joint + "/stiffness");
    config.names.push_back(joint + "/damping");
  }

  return config;
}

controller_interface::InterfaceConfiguration PlatoRosController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

controller_interface::CallbackReturn PlatoRosController::configure_control_architecture(
  plato_robot_system::ControlArchitecture & architecture)
{
  (void)architecture;
  return controller_interface::CallbackReturn::SUCCESS;
}

void PlatoRosController::register_robot_states(
  plato_robot_system::ControlArchitecture & architecture,
  plato_robot_system::RobotSystem & robot)
{
  (void)architecture;
  (void)robot;
}

void PlatoRosController::configure_initial_state(
  plato_robot_system::ControlArchitecture & architecture)
{
  (void)architecture;
}

void PlatoRosController::filter_command(plato_robot_system::RobotCommand * command) const
{
  if (!fixed_thumb_ || command == nullptr) {
    return;
  }

  constexpr Eigen::Index kNumFixedThumbJoints = 2;
  for (Eigen::Index joint_index = 0; joint_index < kNumFixedThumbJoints; ++joint_index) {
    const auto ros_joint_index = static_cast<std::size_t>(joint_index);
    if (ros_joint_index >= model_q_indices_.size() || ros_joint_index >= model_v_indices_.size()) {
      continue;
    }

    const Eigen::Index q_index = model_q_indices_[ros_joint_index];
    const Eigen::Index v_index = model_v_indices_[ros_joint_index];
    if (q_index < command->q_cmd.size()) {
      command->q_cmd[q_index] = 0.0;
    }
    if (v_index < command->qdot_cmd.size()) {
      command->qdot_cmd[v_index] = 0.0;
    }
    if (v_index < command->tau_cmd.size()) {
      command->tau_cmd[v_index] = 0.0;
    }
    if (v_index < command->kp.size()) {
      command->kp[v_index] = 0.0;
    }
    if (v_index < command->kd.size()) {
      command->kd[v_index] = 0.0;
    }
  }
}

void PlatoRosController::read_state_interfaces()
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    positions_[i] = read_state_interface_value(position_state_interfaces_[i].get());
    velocities_[i] = read_state_interface_value(velocity_state_interfaces_[i].get());
    efforts_[i] = read_state_interface_value(effort_state_interfaces_[i].get()) * kMNmToNm;
  }
}

bool PlatoRosController::assign_state_interfaces()
{
  const auto num_joints = joint_names_.size();
  position_state_interfaces_.clear();
  velocity_state_interfaces_.clear();
  effort_state_interfaces_.clear();

  position_state_interfaces_.reserve(num_joints);
  velocity_state_interfaces_.reserve(num_joints);
  effort_state_interfaces_.reserve(num_joints);

  for (const auto & joint_name : joint_names_) {
    const auto pos_name = joint_name + "/" + hardware_interface::HW_IF_POSITION;
    const auto vel_name = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto eff_name = joint_name + "/" + hardware_interface::HW_IF_EFFORT;

    for (auto & state_interface : state_interfaces_) {
      const auto & iface_name = state_interface.get_name();
      if (iface_name == pos_name) {
        position_state_interfaces_.emplace_back(state_interface);
      } else if (iface_name == vel_name) {
        velocity_state_interfaces_.emplace_back(state_interface);
      } else if (iface_name == eff_name) {
        effort_state_interfaces_.emplace_back(state_interface);
      }
    }
  }

  if (
    position_state_interfaces_.size() != num_joints ||
    velocity_state_interfaces_.size() != num_joints ||
    effort_state_interfaces_.size() != num_joints)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Not all state interfaces were found");
    return false;
  }
  return true;
}

bool PlatoRosController::assign_command_interfaces()
{
  const auto num_joints = joint_names_.size();
  position_command_interfaces_.clear();
  velocity_command_interfaces_.clear();
  effort_command_interfaces_.clear();
  stiffness_command_interfaces_.clear();
  damping_command_interfaces_.clear();

  position_command_interfaces_.reserve(num_joints);
  velocity_command_interfaces_.reserve(num_joints);
  effort_command_interfaces_.reserve(num_joints);
  stiffness_command_interfaces_.reserve(num_joints);
  damping_command_interfaces_.reserve(num_joints);

  for (const auto & joint_name : joint_names_) {
    const auto pos_name = joint_name + "/" + hardware_interface::HW_IF_POSITION;
    const auto vel_name = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto eff_name = joint_name + "/" + hardware_interface::HW_IF_EFFORT;
    const auto stiff_name = joint_name + "/stiffness";
    const auto damp_name = joint_name + "/damping";

    for (auto & command_interface : command_interfaces_) {
      const auto & iface_name = command_interface.get_name();
      if (iface_name == pos_name) {
        position_command_interfaces_.emplace_back(command_interface);
      } else if (iface_name == vel_name) {
        velocity_command_interfaces_.emplace_back(command_interface);
      } else if (iface_name == eff_name) {
        effort_command_interfaces_.emplace_back(command_interface);
      } else if (iface_name == stiff_name) {
        stiffness_command_interfaces_.emplace_back(command_interface);
      } else if (iface_name == damp_name) {
        damping_command_interfaces_.emplace_back(command_interface);
      }
    }
  }

  if (
    position_command_interfaces_.size() != num_joints ||
    velocity_command_interfaces_.size() != num_joints ||
    effort_command_interfaces_.size() != num_joints ||
    stiffness_command_interfaces_.size() != num_joints ||
    damping_command_interfaces_.size() != num_joints)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Not all command interfaces were found");
    return false;
  }
  return true;
}

PlatoRosController::TactileSensorVector PlatoRosController::current_tactile_sensors()
{
  const auto tactile_ptr = rt_tactile_ptr_.readFromRT();
  if (tactile_ptr && *tactile_ptr) {
    return **tactile_ptr;
  }
  return {};
}

void PlatoRosController::tactile_callback(
  const std::size_t index,
  const TactileMsg::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto sample = convert_tactile_msg(index, *msg);
  auto tactile_vector = std::make_shared<TactileSensorVector>();
  bool first_sample_for_stream = false;

  {
    std::lock_guard<std::mutex> lock(nari_touch_mutex_);
    if (index >= nari_touch_.size()) {
      return;
    }
    nari_touch_[index].Update(sample);
    if (index < tactile_stream_seen_.size() && !tactile_stream_seen_[index]) {
      tactile_stream_seen_[index] = true;
      first_sample_for_stream = true;
    }

    tactile_vector->reserve(nari_touch_.size());
    for (const auto & sensor : nari_touch_) {
      tactile_vector->push_back(
        plato_robot_system::sensor::ConvertNARITouchToTactileState(sensor.sample()));
    }
  }

  if (first_sample_for_stream) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Receiving tactile stream %zu from '%s' for frame '%s'",
      index,
      tactile_topics_[index].c_str(),
      tactile_frame_name(index));
  }

  rt_tactile_ptr_.writeFromNonRT(tactile_vector);
}

void PlatoRosController::joint_teleop_command_callback(
  const JointTeleopMsg::SharedPtr msg)
{
  const auto expected_size = joint_names_.size();
  if (!msg || msg->data.size() != expected_size) {
    const auto size = msg ? msg->data.size() : 0U;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *(get_node()->get_clock()),
      1000,
      "Ignoring joint teleop command with %zu values; expected %zu joint positions.",
      size,
      expected_size);
    return;
  }

  JointTeleopCommand command;
  command.target_jpos.resize(static_cast<Eigen::Index>(expected_size));
  for (std::size_t i = 0; i < expected_size; ++i) {
    command.target_jpos[static_cast<Eigen::Index>(i)] = msg->data[i];
  }

  if (!command.target_jpos.allFinite()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *(get_node()->get_clock()),
      1000,
      "Ignoring joint teleop command with non-finite values.");
    return;
  }

  rt_joint_teleop_command_ptr_.writeFromNonRT(
    std::make_shared<JointTeleopCommand>(command));
}

void PlatoRosController::sync_joint_teleop_input()
{
  if (joint_teleop_state_ == nullptr || !robot_) {
    return;
  }

  const auto command_ptr = rt_joint_teleop_command_ptr_.readFromRT();
  if (command_ptr == nullptr || !*command_ptr) {
    return;
  }

  try {
    const auto target_q =
      map_joint_positions_to_model_q((**command_ptr).target_jpos, *robot_);
    if (!joint_teleop_state_->SetTargetPosition(target_q)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *(get_node()->get_clock()),
        1000,
        "Joint teleop state rejected the latest target position command.");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *(get_node()->get_clock()),
      1000,
      "Failed to map joint teleop target into model coordinates: %s",
      e.what());
  }
}

void PlatoRosController::grasp_teleop_command_callback(
  const GraspTeleopMsg::SharedPtr msg)
{
  if (!msg || msg->data.empty() || msg->data.size() > 3U) {
    const auto size = msg ? msg->data.size() : 0U;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *(get_node()->get_clock()),
      1000,
      "Ignoring grasp teleop command with %zu values; expected [u], [u, phi], or [u, phi, f].",
      size);
    return;
  }

  GraspTeleopCommand command;
  command.u = msg->data[0];
  if (msg->data.size() > 1U) {
    command.phi = msg->data[1];
  }
  if (msg->data.size() == 3U) {
    command.desired_force_n = msg->data[2];
    command.has_desired_force = true;
  }

  if (
    !std::isfinite(command.u) ||
    !std::isfinite(command.phi) ||
    (command.has_desired_force &&
    (!std::isfinite(command.desired_force_n) || command.desired_force_n < 0.0)))
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *(get_node()->get_clock()),
      1000,
      "Ignoring grasp teleop command with non-finite or negative values.");
    return;
  }

  rt_grasp_teleop_command_ptr_.writeFromNonRT(
    std::make_shared<GraspTeleopCommand>(command));
}

void PlatoRosController::grasp_force_reference_callback(
  const GraspForceReferenceMsg::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!std::isfinite(msg->data) || msg->data < 0.0) {
    grasp_force_reference_n_.store(0.0);
    grasp_force_reference_valid_.store(false);
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *(get_node()->get_clock()),
      1000,
      "Ignoring invalid grasp force reference target.");
    return;
  }

  grasp_force_reference_n_.store(msg->data);
}

void PlatoRosController::grasp_force_reference_valid_callback(
  const GraspForceReferenceValidMsg::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  grasp_force_reference_valid_.store(msg->data);
}

void PlatoRosController::sync_grasp_teleop_input()
{
  if (grasp_teleop_state_ == nullptr && grasp_force_state_ == nullptr) {
    return;
  }

  const auto command_ptr = rt_grasp_teleop_command_ptr_.readFromRT();
  const bool has_command = command_ptr != nullptr && *command_ptr;

  aristo_controller::state_machines::GraspTeleopInput input;
  input.u = std::numeric_limits<double>::quiet_NaN();
  input.phi = std::numeric_limits<double>::quiet_NaN();

  bool has_explicit_force = false;
  if (has_command) {
    input.u = (**command_ptr).u;
    input.phi = (**command_ptr).phi;
    if ((**command_ptr).has_desired_force) {
      input.desired_force_n = (**command_ptr).desired_force_n;
      has_explicit_force = true;
    }
  }

  if (!has_explicit_force) {
    double default_desired_force_n = 0.0;
    if (grasp_teleop_state_ != nullptr) {
      default_desired_force_n = grasp_teleop_state_->default_desired_force_n();
    } else if (grasp_force_state_ != nullptr) {
      default_desired_force_n = grasp_force_state_->default_desired_force_n();
    }
    input.desired_force_n = grasp_force_reference_valid_.load() ?
      grasp_force_reference_n_.load() :
      default_desired_force_n;
  }

  if (grasp_teleop_state_ != nullptr) {
    grasp_teleop_state_->SetInput(input);
  }
  if (grasp_force_state_ != nullptr) {
    aristo_controller::state_machines::GraspForceInput force_input;
    force_input.u = input.u;
    force_input.phi = input.phi;
    force_input.desired_force_n = input.desired_force_n;
    grasp_force_state_->SetInput(force_input);
  }
}

plato_robot_system::sensor::NARITouchSample PlatoRosController::convert_tactile_msg(
  const std::size_t index,
  const TactileMsg & msg) const
{
  plato_robot_system::sensor::NARITouchSample sample;
  sample.valid = true;
  sample.stamp_sec = tactile_sample_time(msg).seconds();
  sample.sensor_index = static_cast<int>(index);
  sample.frame_name = tactile_frame_name(index);
  sample.contact_state = convert_contact_state(msg.contact_state);
  sample.force_n = Eigen::Vector3d{msg.force.x, msg.force.y, msg.force.z};
  sample.shear_displacement_m = Eigen::Vector2d{
    msg.shear_displacement.x,
    msg.shear_displacement.y};
  sample.rotational_shear_rad = msg.shear_displacement.theta;

  const auto count = std::min(sample.units.size(), msg.units.size());
  for (std::size_t i = 0; i < count; ++i) {
    sample.units[i].contact = msg.units[i].contact;
    sample.units[i].cop = Eigen::Vector2d{
      msg.units[i].cop.x * plato_robot_system::sensor::kNARITouchCopToMScale,
      msg.units[i].cop.y * plato_robot_system::sensor::kNARITouchCopToMScale};
    sample.units[i].normal_force_n = msg.units[i].normal_force;
  }

  return sample;
}

plato_robot_system::sensor::NARITouchContactState PlatoRosController::convert_contact_state(
  const int state) const
{
  switch (state) {
    case TactileMsg::ENOUGH_CONTACTS:
      return plato_robot_system::sensor::NARITouchContactState::kEnoughContacts;
    case TactileMsg::FEW_CONTACTS:
      return plato_robot_system::sensor::NARITouchContactState::kFewContacts;
    default:
      return plato_robot_system::sensor::NARITouchContactState::kNoContact;
  }
}

rclcpp::Time PlatoRosController::tactile_sample_time(const TactileMsg & msg) const
{
  if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0U) {
    return get_node()->now();
  }
  return rclcpp::Time(msg.header.stamp);
}

const char * PlatoRosController::tactile_frame_name(const std::size_t index) const
{
  return kTactileFrameNames[index];
}

void PlatoRosController::publish_controller_state(
  const rclcpp::Time & time,
  const plato_robot_system::RobotCommand & command)
{
  if (!controller_state_pub_) {
    return;
  }

  plato_interfaces::msg::ImpedanceControllerState msg;
  msg.header.stamp = time;
  msg.joint_names = joint_names_;
  msg.position_actual = positions_;
  msg.velocity_actual = velocities_;
  msg.effort_actual = efforts_;

  const auto num_joints = joint_names_.size();
  msg.position_desired.resize(num_joints);
  msg.velocity_desired.resize(num_joints);
  msg.position_error.resize(num_joints);
  msg.velocity_error.resize(num_joints);
  msg.stiffness.resize(num_joints);
  msg.damping.resize(num_joints);
  msg.effort_desired.resize(num_joints);
  msg.effort_actual.resize(num_joints);
  msg.effort_ff.resize(num_joints);
  msg.effort_fb.resize(num_joints);

  for (std::size_t i = 0; i < num_joints; ++i) {
    const Eigen::Index q_index = model_q_indices_[i];
    const Eigen::Index v_index = model_v_indices_[i];
    msg.position_desired[i] = command.q_cmd[q_index];
    msg.velocity_desired[i] = command.qdot_cmd[v_index];
    msg.position_error[i] = command.q_cmd[q_index] - positions_[i];
    msg.velocity_error[i] = command.qdot_cmd[v_index] - velocities_[i];
    msg.stiffness[i] = command.kp[v_index];
    msg.damping[i] = command.kd[v_index];
    msg.effort_desired[i] = command.tau_cmd[v_index];
    msg.effort_ff[i] = command.tau_cmd[v_index];
    msg.effort_fb[i] = 0.0;
  }
  controller_state_pub_->publish(msg);
}

void PlatoRosController::request_state_callback(
  std::shared_ptr<RequestStateSrv::Request> request,
  std::shared_ptr<RequestStateSrv::Response> response)
{
  const auto requested_state_id =
    static_cast<plato_robot_system::StateId>(request->state_id);
  if (requested_state_id < 0) {
    response->success = false;
    response->message = "state_id must be nonnegative";
    return;
  }

  const auto * fsm_handler = control_architecture_.fsmHandler();
  const auto & states = fsm_handler->states();
  const auto requested_state_it = states.find(requested_state_id);
  if (requested_state_it == states.end() || !requested_state_it->second) {
    response->success = false;
    response->message = "state_id " + std::to_string(requested_state_id) +
      " is not registered";
    return;
  }

  plato_robot_system::StateId effective_state_id = requested_state_id;
  if (requested_state_it->second->name() ==
    aristo_controller::state_machines::GraspTeleopState::kName)
  {
    const auto grasp_ready_id =
      fsm_handler->FindStateIdByName(
        aristo_controller::state_machines::GraspReadyState::kName);
    if (!grasp_ready_id) {
      response->success = false;
      response->message = "state '" +
        std::string(aristo_controller::state_machines::GraspReadyState::kName) +
        "' is not registered";
      return;
    }
    effective_state_id = *grasp_ready_id;
  }

  const auto effective_state_it = states.find(effective_state_id);
  if (effective_state_it == states.end() || !effective_state_it->second) {
    response->success = false;
    response->message = "redirect target state_id " + std::to_string(effective_state_id) +
      " is not registered";
    return;
  }

  pending_requested_state_id_.store(effective_state_id);
  response->success = true;
  response->message = "accepted request for state_id " +
    std::to_string(effective_state_id) + " (" + effective_state_it->second->name() + ")";
  if (effective_state_id != requested_state_id) {
    response->message += "; redirected from state_id " +
      std::to_string(requested_state_id) + " (" + requested_state_it->second->name() + ")";
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Accepted FSM state request: requested_id=%d requested_name='%s' effective_id=%d effective_name='%s'",
    requested_state_id,
    requested_state_it->second->name().c_str(),
    effective_state_id,
    effective_state_it->second->name().c_str());
}

void PlatoRosController::apply_pending_state_request()
{
  const auto requested_state_id = pending_requested_state_id_.exchange(-1);
  if (requested_state_id < 0) {
    return;
  }

  if (!control_architecture_.RequestState(requested_state_id)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Queued FSM state request id %d was rejected by ControlArchitecture",
      requested_state_id);
  }
}

bool PlatoRosController::configure_from_control_config(
  plato_robot_system::ControlArchitecture & architecture,
  plato_robot_system::RobotSystem & robot)
{
  try {
    const auto resolved_config_path = resolve_package_url(control_config_yaml_path_);
    const auto aristo_config =
      aristo_controller::config::load_aristo_config(resolved_config_path);

    if (aristo_config.num_joints != static_cast<int>(joint_names_.size())) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Control config expects %d joints, but the controller was given %zu joint names",
        aristo_config.num_joints,
        joint_names_.size());
      return false;
    }

    const auto resolved_urdf_path = load_robot_model_from_config(aristo_config.robot_model, robot);
    architecture.Configure(robot.nq(), robot.nv());
    if (!configure_model_joint_mapping(robot)) {
      return false;
    }

    fixed_thumb_ = aristo_config.robot_model.fixed_thumb;
    architecture.setTimingEnabled(aristo_config.debug_enabled);
    auto driver_gains = aristo_config.driver_gains;
    driver_gains.kp = map_joint_values_to_model_v(aristo_config.driver_gains.kp);
    driver_gains.kd = map_joint_values_to_model_v(aristo_config.driver_gains.kd);
    if (!driver_gains.HasValidDimensions(architecture.nv())) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Mapped driver gains have invalid dimensions for robot model nv=%d",
        architecture.nv());
      return false;
    }
    architecture.SetDriverPdGainsConfig(driver_gains);

    auto idle = std::make_unique<aristo_controller::state_machines::IdleState>(
      aristo_config.idle.id,
      architecture.nq(),
      architecture.nv());
    architecture.RegisterState(std::move(idle));

    auto joint_teleop_config = aristo_config.joint_teleop.state;
    joint_teleop_config.joint_task.kp_task =
      map_joint_values_to_model_v(aristo_config.joint_teleop.state.joint_task.kp_task);
    joint_teleop_config.joint_task.kd_task =
      map_joint_values_to_model_v(aristo_config.joint_teleop.state.joint_task.kd_task);

    auto joint_teleop = std::make_unique<aristo_controller::state_machines::JointTeleopState>(
      aristo_config.joint_teleop.id,
      &robot);
    if (!joint_teleop->ConfigureTask(joint_teleop_config)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure joint_teleop task");
      return false;
    }
    joint_teleop_state_ = joint_teleop.get();
    architecture.RegisterState(std::move(joint_teleop));

    auto grasp_teleop_config = aristo_config.grasp_teleop.state;
    if (grasp_teleop_config.grasp_task.q_ready.size() > 0) {
      grasp_teleop_config.grasp_task.q_ready =
        map_joint_positions_to_model_q(grasp_teleop_config.grasp_task.q_ready, robot);
    }

    auto grasp_teleop = std::make_unique<aristo_controller::state_machines::GraspTeleopState>(
      aristo_config.grasp_teleop.id,
      &robot);
    if (!grasp_teleop->ConfigureTask(grasp_teleop_config)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure grasp_teleop task");
      return false;
    }
    grasp_teleop->ConfigureLifecycle(aristo_config.grasp_teleop.lifecycle);
    grasp_teleop_state_ = grasp_teleop.get();
    architecture.RegisterState(std::move(grasp_teleop));

    auto grasp_force_config = aristo_config.grasp_force.state;
    if (grasp_force_config.grasp_task.q_ready.size() > 0) {
      grasp_force_config.grasp_task.q_ready =
        map_joint_positions_to_model_q(grasp_force_config.grasp_task.q_ready, robot);
    }

    auto grasp_force = std::make_unique<aristo_controller::state_machines::GraspForceState>(
      aristo_config.grasp_force.id,
      &robot);
    if (!grasp_force->ConfigureTask(grasp_force_config)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure grasp_force task");
      return false;
    }
    grasp_force->ConfigureLifecycle(aristo_config.grasp_force.lifecycle);
    grasp_force_state_ = grasp_force.get();
    architecture.RegisterState(std::move(grasp_force));

    auto mppi_grasp = std::make_unique<aristo_controller::state_machines::MPPIGraspState>(
      aristo_config.mppi_grasp.id,
      &robot);
    if (!mppi_grasp->ConfigureTask(aristo_config.mppi_grasp.state)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure mppi_grasp task");
      return false;
    }
    mppi_grasp->ConfigureLifecycle(aristo_config.mppi_grasp.lifecycle);
    architecture.RegisterState(std::move(mppi_grasp));

    auto initialize = std::make_unique<aristo_controller::state_machines::InitializeState>(
      aristo_config.initialize.id,
      &robot);
    initialize->SetTargetPosition(
      map_joint_positions_to_model_q(aristo_config.initialize.target_jpos, robot));
    initialize->SetDuration(aristo_config.initialize.duration_sec);
    initialize->SetTaskFeedbackGains(
      map_joint_values_to_model_v(aristo_config.initialize.kp_task),
      map_joint_values_to_model_v(aristo_config.initialize.kd_task));
    initialize->ConfigureLifecycle(aristo_config.initialize.lifecycle);
    architecture.RegisterState(std::move(initialize));

    auto poke = std::make_unique<aristo_controller::state_machines::PokeState>(
      aristo_config.poke.id,
      &robot);
    poke->SetTargetPosition(
      map_joint_positions_to_model_q(aristo_config.poke.target_jpos, robot));
    poke->SetDuration(aristo_config.poke.duration_sec);
    poke->SetTaskFeedbackGains(
      map_joint_values_to_model_v(aristo_config.poke.kp_task),
      map_joint_values_to_model_v(aristo_config.poke.kd_task));
    poke->ConfigureLifecycle(aristo_config.poke.lifecycle);
    architecture.RegisterState(std::move(poke));

    auto grasp_ready = std::make_unique<aristo_controller::state_machines::GraspReadyState>(
      aristo_config.grasp_ready.id,
      &robot);
    grasp_ready->SetTargetPosition(
      map_joint_positions_to_model_q(aristo_config.grasp_ready.target_jpos, robot));
    grasp_ready->SetDuration(aristo_config.grasp_ready.duration_sec);
    grasp_ready->SetTaskFeedbackGains(
      map_joint_values_to_model_v(aristo_config.grasp_ready.kp_task),
      map_joint_values_to_model_v(aristo_config.grasp_ready.kd_task));
    grasp_ready->ConfigureLifecycle(aristo_config.grasp_ready.lifecycle);
    architecture.RegisterState(std::move(grasp_ready));

    if (!architecture.SetStartState(aristo_config.initialize.id) ||
      !architecture.RequestState(aristo_config.initialize.id))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to select initialize state id %d from control config",
        aristo_config.initialize.id);
      return false;
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Loaded control config '%s', robot model '%s' (nq=%d, nv=%d), and selected state 'initialize' (id=%d, fixed_thumb=%s)",
      resolved_config_path.c_str(),
      resolved_urdf_path.c_str(),
      robot.nq(),
      robot.nv(),
      aristo_config.initialize.id,
      fixed_thumb_ ? "true" : "false");
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to load control config '%s': %s",
      control_config_yaml_path_.c_str(),
      e.what());
    return false;
  }
}

void PlatoRosController::configure_identity_joint_mapping(const std::size_t num_joints)
{
  model_q_indices_.resize(num_joints);
  model_v_indices_.resize(num_joints);
  for (std::size_t i = 0; i < num_joints; ++i) {
    const auto index = static_cast<Eigen::Index>(i);
    model_q_indices_[i] = index;
    model_v_indices_[i] = index;
  }
}

bool PlatoRosController::configure_model_joint_mapping(
  const plato_robot_system::RobotSystem & robot)
{
  if (!robot.hasModel()) {
    configure_identity_joint_mapping(joint_names_.size());
    return true;
  }

  model_q_indices_.clear();
  model_v_indices_.clear();
  model_q_indices_.reserve(joint_names_.size());
  model_v_indices_.reserve(joint_names_.size());

  const auto & model = robot.model();
  for (const auto & joint_name : joint_names_) {
    if (!model.existJointName(joint_name)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Robot model does not contain ros2_control joint '%s'",
        joint_name.c_str());
      return false;
    }

    const auto joint_id = model.getJointId(joint_name);
    if (joint_id >= static_cast<pinocchio::JointIndex>(model.njoints)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Robot model returned invalid joint id for '%s'",
        joint_name.c_str());
      return false;
    }

    const auto & joint = model.joints[joint_id];
    if (joint.nq() != 1 || joint.nv() != 1) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "ros2_control joint '%s' maps to nq=%d nv=%d; this adapter expects one-DoF controlled joints",
        joint_name.c_str(),
        joint.nq(),
        joint.nv());
      return false;
    }
    model_q_indices_.push_back(static_cast<Eigen::Index>(joint.idx_q()));
    model_v_indices_.push_back(static_cast<Eigen::Index>(joint.idx_v()));
  }
  return true;
}

plato_robot_system::RobotCommand PlatoRosController::make_safe_hold_command(
  const plato_robot_system::RobotState & state) const
{
  plato_robot_system::RobotCommand command;
  command.Resize(control_architecture_.nq(), control_architecture_.nv());
  if (!plato_robot_system::IsValid(state) ||
    state.q.size() != command.q_cmd.size() ||
    state.qdot.size() != command.qdot_cmd.size())
  {
    command.valid = false;
    return command;
  }

  command.q_cmd = state.q;
  command.qdot_cmd.setZero();
  command.tau_cmd.setZero();
  command.stamp_sec = state.time_s;
  command.valid = true;
  control_architecture_.FinalizeCommand(&command);
  return command;
}

bool PlatoRosController::write_safe_hold_command(const plato_robot_system::RobotState & state)
{
  auto safe_command = make_safe_hold_command(state);
  filter_command(&safe_command);
  if (!safe_command.IsUsable()) {
    return false;
  }

  write_command(safe_command);
  return true;
}

void PlatoRosController::write_zero_command()
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    set_command_interface_value(position_command_interfaces_[i].get(), 0.0);
    set_command_interface_value(velocity_command_interfaces_[i].get(), 0.0);
    set_command_interface_value(effort_command_interfaces_[i].get(), 0.0);
    set_command_interface_value(stiffness_command_interfaces_[i].get(), 0.0);
    set_command_interface_value(damping_command_interfaces_[i].get(), 0.0);
  }
}

Eigen::VectorXd PlatoRosController::map_joint_positions_to_model_q(
  const Eigen::VectorXd & joint_positions,
  const plato_robot_system::RobotSystem & robot) const
{
  if (joint_positions.size() != static_cast<Eigen::Index>(joint_names_.size())) {
    if (joint_positions.size() == control_architecture_.nq()) {
      return joint_positions;
    }
    throw std::runtime_error(
      "Joint position vector must match either robot nq or the configured joint count");
  }

  Eigen::VectorXd model_q = robot.hasModel() ?
    robot.state().q :
    Eigen::VectorXd::Zero(control_architecture_.nq());
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    model_q[model_q_indices_[i]] = joint_positions[static_cast<Eigen::Index>(i)];
  }
  return model_q;
}

Eigen::VectorXd PlatoRosController::map_joint_values_to_model_v(
  const Eigen::VectorXd & joint_values) const
{
  if (joint_values.size() != static_cast<Eigen::Index>(joint_names_.size())) {
    if (joint_values.size() == control_architecture_.nv()) {
      return joint_values;
    }
    throw std::runtime_error(
      "Joint tangent vector must match either robot nv or the configured joint count");
  }

  Eigen::VectorXd model_values = Eigen::VectorXd::Zero(control_architecture_.nv());
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    model_values[model_v_indices_[i]] = joint_values[static_cast<Eigen::Index>(i)];
  }
  return model_values;
}

void PlatoRosController::write_command(const plato_robot_system::RobotCommand & command)
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const Eigen::Index q_index = model_q_indices_[i];
    const Eigen::Index v_index = model_v_indices_[i];
    set_command_interface_value(position_command_interfaces_[i].get(), command.q_cmd[q_index]);
    set_command_interface_value(velocity_command_interfaces_[i].get(), command.qdot_cmd[v_index]);
    set_command_interface_value(
      effort_command_interfaces_[i].get(), command.tau_cmd[v_index] * kNmToMNm);
    set_command_interface_value(
      stiffness_command_interfaces_[i].get(), command.kp[v_index] * kNmToMNm);
    set_command_interface_value(
      damping_command_interfaces_[i].get(), command.kd[v_index] * kNmToMNm);
  }
}

}  // namespace plato_ros_controller

PLUGINLIB_EXPORT_CLASS(
  plato_ros_controller::PlatoRosController,
  controller_interface::ControllerInterface)
