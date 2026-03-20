#include "aristo_hardware_interface/aristo_hand.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <iostream>
#include <vector>

namespace aristo_hand
{

namespace
{
using JointArrayf = Eigen::Array<float, static_cast<Eigen::Index>(Hand::kNumActuators), 1>;
using JointArrayd = Eigen::Array<double, static_cast<Eigen::Index>(Hand::kNumActuators), 1>;
using JointMapd = Eigen::Map<JointArrayd>;
using ConstJointMapd = Eigen::Map<const JointArrayd>;

void accumulate_poll_result(
  can_hardware_common::CanBusManager::PollResult & aggregate,
  const can_hardware_common::CanBusManager::PollResult & update)
{
  aggregate.processed_frames += update.processed_frames;
  aggregate.hit_frame_budget = aggregate.hit_frame_budget || update.hit_frame_budget;

  const auto is_nonfatal_status = [](TPCANStatus status) {
      return status == PCAN_ERROR_OK || status == PCAN_ERROR_QRCVEMPTY;
    };

  if (is_nonfatal_status(aggregate.read_status) && !is_nonfatal_status(update.read_status)) {
    aggregate.read_status = update.read_status;
  } else if (
    aggregate.read_status == PCAN_ERROR_QRCVEMPTY &&
    update.read_status == PCAN_ERROR_OK)
  {
    aggregate.read_status = PCAN_ERROR_OK;
  }
}
}  // namespace

Hand::Hand()
{
  actuators_.reserve(actuator_configs_.size());
  for (const auto & config : actuator_configs_) {
    actuators_.emplace_back(config);
  }

  ft_sensors_.reserve(ft_sensor_configs_.size());
  for (const auto & config : ft_sensor_configs_) {
    ft_sensors_.emplace_back(config);
  }

  initialize_rx_dispatch_table_();

  print_actuator_info_();
}

can_hardware_common::CanBusManager::PollResult Hand::poll_can_bus()
{
  auto poll_result = can_bus_manager_.poll_once({this, &Hand::dispatch_rx_frame_static_});
  if (poll_result.hit_frame_budget && poll_result.read_status == PCAN_ERROR_OK) {
    const auto extra_poll_result =
      can_bus_manager_.poll_once({this, &Hand::dispatch_rx_frame_static_});
    accumulate_poll_result(poll_result, extra_poll_result);
  }

  return poll_result;
}

void Hand::enable_all_actuators()
{
  for (auto & actuator : actuators_) {
    send_command_(actuator.enable_motor());
  }
}

void Hand::disable_all_actuators()
{
  for (auto & actuator : actuators_) {
    send_command_(actuator.disable_motor());
  }
}

TPCANStatus Hand::send_command_(const actuator::TxCommand & command)
{
  return can_bus_manager_.send_frame(command.frame, command.post_send_delay);
}

void Hand::update_ft_sensor_wrenches_(std::vector<geometry_msgs::msg::Wrench> & ft_sensor_states)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (ft_sensor_states.size() != ft_sensors_.size()) {
    ft_sensor_states.resize(ft_sensors_.size());
  }

  for (size_t i = 0; i < ft_sensors_.size(); ++i) {
    const auto & states = ft_sensors_[i].get_states();
    ft_sensor_states[i].force.x = states.force_filtered.x();
    ft_sensor_states[i].force.y = states.force_filtered.y();
    ft_sensor_states[i].force.z = states.force_filtered.z();
    ft_sensor_states[i].torque.x = states.torque_filtered.x();
    ft_sensor_states[i].torque.y = states.torque_filtered.y();
    ft_sensor_states[i].torque.z = states.torque_filtered.z();
  }
}

void Hand::print_hardware_info_(const char * actuator_total_label) const
{
  std::cout << "================== Actuators Info ===================" << std::endl;
  std::cout << "[INFO] " << actuator_total_label << ": " << actuators_.size() << std::endl;

  for (size_t i = 0; i < actuators_.size(); ++i) {
    std::cout << "[INFO] Actuator " << (i + 1)
              << " - TX: 0x" << std::hex << actuators_[i].get_tx_id()
              << ", RX: 0x" << actuators_[i].get_rx_id() << std::dec << std::endl;
  }

  std::cout << "=================== FT Sensors Info ==================" << std::endl;
  std::cout << "[INFO] Total FT Sensors: " << ft_sensors_.size() << std::endl;

  for (size_t i = 0; i < ft_sensors_.size(); ++i) {
    std::cout << "[INFO] FT Sensor " << (i + 1)
              << " - Force RX: 0x" << std::hex << ft_sensors_[i].get_force_rx_id()
              << ", Torque RX: 0x" << ft_sensors_[i].get_torque_rx_id() << std::dec << std::endl;
  }
}

void Hand::initialize_rx_dispatch_table_()
{
  size_t dispatch_index = 0;
  for (size_t actuator_index = 0; actuator_index < actuators_.size(); ++actuator_index) {
    rx_dispatch_table_[dispatch_index++] = {
      actuators_[actuator_index].get_rx_id(),
      DispatchTargetKind::kActuator,
      actuator_index};
  }

  for (size_t sensor_index = 0; sensor_index < ft_sensors_.size(); ++sensor_index) {
    rx_dispatch_table_[dispatch_index++] = {
      ft_sensors_[sensor_index].get_force_rx_id(),
      DispatchTargetKind::kForceSensor,
      sensor_index};
    rx_dispatch_table_[dispatch_index++] = {
      ft_sensors_[sensor_index].get_torque_rx_id(),
      DispatchTargetKind::kTorqueSensor,
      sensor_index};
  }

  std::sort(
    rx_dispatch_table_.begin(),
    rx_dispatch_table_.end(),
    [](const RxDispatchEntry & lhs, const RxDispatchEntry & rhs) {
      return lhs.rx_id < rhs.rx_id;
    });
}

void Hand::dispatch_rx_frame_static_(void * context, const TPCANMsg & frame)
{
  static_cast<Hand *>(context)->dispatch_rx_frame_(frame);
}

void Hand::dispatch_rx_frame_(const TPCANMsg & frame)
{
  if (frame.MSGTYPE != PCAN_MESSAGE_STANDARD) {
    return;
  }

  const auto entry_it = std::lower_bound(
    rx_dispatch_table_.begin(),
    rx_dispatch_table_.end(),
    frame.ID,
    [](const RxDispatchEntry & entry, uint32_t rx_id) {
      return entry.rx_id < rx_id;
    });
  if (entry_it == rx_dispatch_table_.end() || entry_it->rx_id != frame.ID) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  switch (entry_it->target_kind) {
    case DispatchTargetKind::kActuator:
      actuators_[entry_it->target_index].process_message(frame);
      break;
    case DispatchTargetKind::kForceSensor:
    case DispatchTargetKind::kTorqueSensor:
      ft_sensors_[entry_it->target_index].process_message(frame);
      break;
  }
}

void Hand::enable()
{
  enable_all_actuators();
}

void Hand::disable()
{
  disable_all_actuators();
}

void Hand::set_current_position_as_zero()
{
  for (auto & actuator : actuators_) {
    send_command_(actuator.set_current_position_as_zero());
  }
}

void Hand::set_default_can_limits()
{
  for (auto & actuator : actuators_) {
    send_command_(actuator.set_default_can_limits());
  }
}

TPCANStatus Hand::write_joint_commands()
{
  if (joint_commands_.size() != kNumActuators)
  {
    return PCAN_ERROR_OK;
  }

  std::vector<actuator::TxCommand> commands;
  commands.reserve(kNumActuators);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const ConstJointMapd joint_position_cmd_map(joint_commands_.position_data());
    const ConstJointMapd joint_velocity_cmd_map(joint_commands_.velocity_data());
    const ConstJointMapd joint_stiffness_cmd_map(joint_commands_.stiffness_data());
    const ConstJointMapd joint_damping_cmd_map(joint_commands_.damping_data());
    const ConstJointMapd joint_torque_cmd_map(joint_commands_.effort_data());

    JointArrayf joint_position_cmd = joint_position_cmd_map.cast<float>();
    const JointArrayf joint_velocity_cmd = joint_velocity_cmd_map.cast<float>();
    const JointArrayf joint_stiffness_cmd = joint_stiffness_cmd_map.cast<float>();
    const JointArrayf joint_damping_cmd = joint_damping_cmd_map.cast<float>();
    const JointArrayf joint_torque_cmd = joint_torque_cmd_map.cast<float>();

    // PIP joints are modeled relative to the MCP joint in URDF but actuated from the palm.
    joint_position_cmd(kThumbPipIndex) += actuators_[kThumbMcpIndex].get_feedback().position;
    joint_position_cmd(kIndexPipIndex) += actuators_[kIndexMcpIndex].get_feedback().position;
    joint_position_cmd(kMiddlePipIndex) += actuators_[kMiddleMcpIndex].get_feedback().position;

    for (size_t i = 0; i < kNumActuators; ++i) {
      const Eigen::Index joint_index = static_cast<Eigen::Index>(i);
      const can_hardware_common::ActuatorTarget impedance_target{
        joint_position_cmd(joint_index),
        joint_velocity_cmd(joint_index),
        joint_stiffness_cmd(joint_index),
        joint_damping_cmd(joint_index),
        joint_torque_cmd(joint_index)};

      if (const auto command = actuators_[i].set_joint_impedance(impedance_target))
      {
        commands.push_back(*command);
      }
    }

  }

  TPCANStatus first_error = PCAN_ERROR_OK;
  for (const auto & command : commands) {
    const TPCANStatus status = send_command_(command);
    if (first_error == PCAN_ERROR_OK && status != PCAN_ERROR_OK) {
      first_error = status;
    }
  }

  if (first_error == PCAN_ERROR_OK) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    joint_commands_.capture_previous();
  }

  return first_error;
}

void Hand::read_joint_states()
{
  if (joint_states_.size() != kNumActuators)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  JointArrayf joint_positions = JointArrayf::Zero();
  JointArrayf joint_velocities = JointArrayf::Zero();
  JointArrayf joint_efforts = JointArrayf::Zero();

  for (size_t i = 0; i < kNumActuators; ++i) {
    const auto & states = actuators_[i].get_feedback();
    const Eigen::Index joint_index = static_cast<Eigen::Index>(i);
    joint_positions(joint_index) = states.position;
    joint_velocities(joint_index) = states.velocity;
    joint_efforts(joint_index) = states.torque;
  }

  joint_positions(kThumbPipIndex) -= joint_positions(kThumbMcpIndex);
  joint_positions(kIndexPipIndex) -= joint_positions(kIndexMcpIndex);
  joint_positions(kMiddlePipIndex) -= joint_positions(kMiddleMcpIndex);

  JointMapd joint_position_map(joint_states_.position_data());
  JointMapd joint_velocity_map(joint_states_.velocity_data());
  JointMapd joint_effort_map(joint_states_.effort_data());

  joint_position_map = joint_positions.cast<double>();
  joint_velocity_map = joint_velocities.cast<double>();
  joint_effort_map = joint_efforts.cast<double>();
}

void Hand::update_ft_sensor_states(std::vector<geometry_msgs::msg::Wrench> & ft_sensor_states)
{
  update_ft_sensor_wrenches_(ft_sensor_states);
}

void Hand::print_actuator_info_() const
{
  print_hardware_info_();
}

}  // namespace aristo_hand
