#include "plato_ros_controller/plato_ros_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "aristo_controller/state_machines/initialize.hpp"
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
  auto_declare<bool>("compute_impedance_torque", false);
  auto_declare<std::string>("control_config_yaml_path", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PlatoRosController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  tactile_topics_ = get_node()->get_parameter("tactile_topics").as_string_array();
  control_config_yaml_path_ = get_node()->get_parameter("control_config_yaml_path").as_string();
  compute_impedance_torque_ = get_node()->get_parameter("compute_impedance_torque").as_bool();

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
  control_architecture_.config().compute_impedance_torque = compute_impedance_torque_;
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

  tactile_subs_.clear();
  tactile_subs_.reserve(tactile_topics_.size());
  for (std::size_t i = 0; i < tactile_topics_.size(); ++i) {
    tactile_subs_.push_back(get_node()->create_subscription<TactileMsg>(
      tactile_topics_[i], rclcpp::SensorDataQoS(),
      [this, i](const TactileMsg::SharedPtr msg) { tactile_callback(i, msg); }));
  }
  controller_state_pub_ =
    get_node()->create_publisher<plato_interfaces::msg::ImpedanceControllerState>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured Plato ROS controller with %zu joints and %zu tactile topics",
    num_joints,
    tactile_topics_.size());

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
    return controller_interface::return_type::ERROR;
  }

  try {
    control_architecture_.Update(robot_state, period.seconds());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Control architecture update failed: %s", e.what());
    return controller_interface::return_type::ERROR;
  }
  const auto & command = control_architecture_.command();
  if (!command.IsUsable()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Control architecture produced an invalid command");
    return controller_interface::return_type::ERROR;
  }

  filtered_command_ = command;
  filter_command(&filtered_command_);
  if (!filtered_command_.IsUsable()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "Controller command filter produced an invalid command");
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
  architecture.RequestMode(plato_robot_system::ControlMode::kHold);
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

    auto initialize = std::make_unique<aristo_controller::state_machines::InitializeState>(
      aristo_config.initialize.id,
      aristo_controller::state_machines::InitializeState::kName,
      &robot);
    initialize->SetTargetPosition(
      map_joint_positions_to_model_q(aristo_config.initialize.target_jpos, robot));
    initialize->SetDuration(aristo_config.initialize.duration_sec);
    initialize->SetTaskFeedbackGains(
      map_joint_values_to_model_v(aristo_config.initialize.kp_task),
      map_joint_values_to_model_v(aristo_config.initialize.kd_task));
    architecture.RegisterState(std::move(initialize));

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
