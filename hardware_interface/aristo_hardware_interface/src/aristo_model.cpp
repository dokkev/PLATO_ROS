#include "aristo_hardware_interface/aristo_model.hpp"

#include <Eigen/Core>

namespace aristo_hand
{
namespace
{
constexpr std::size_t kThumbMcpIndex = 2;
constexpr std::size_t kThumbPipIndex = 3;
constexpr std::size_t kIndexMcpIndex = 4;
constexpr std::size_t kIndexPipIndex = 5;
constexpr std::size_t kMiddleMcpIndex = 6;
constexpr std::size_t kMiddlePipIndex = 7;

using JointArrayf = Eigen::Array<float, 8, 1>;
using JointArrayd = Eigen::Array<double, 8, 1>;
using JointMapd = Eigen::Map<JointArrayd>;
using ConstJointMapd = Eigen::Map<const JointArrayd>;
}  // namespace

void AristoModel::update_joint_states(
  const std::vector<aristo_actuator::Actuator> & actuators,
  can_hardware_common::RobotIO::ActuatorState & actuator_states,
  can_hardware_common::RobotIO::JointState & joint_states) const
{
  if (joint_states.size() != actuators.size() || actuator_states.size() != actuators.size()) {
    return;
  }

  JointArrayf joint_positions = JointArrayf::Zero();
  JointArrayf joint_velocities = JointArrayf::Zero();
  JointArrayf joint_efforts = JointArrayf::Zero();

  if (!can_hardware_common::RobotIO::copy_actuator_feedback_to_buffer(
      actuators,
      actuator_states,
      [](const auto & actuator) { return actuator.get_feedback(); }))
  {
    return;
  }

  for (std::size_t i = 0; i < actuators.size(); ++i) {
    const auto & states = actuators[i].get_feedback();
    const Eigen::Index joint_index = static_cast<Eigen::Index>(i);
    joint_positions(joint_index) = states.position;
    joint_velocities(joint_index) = states.velocity;
    joint_efforts(joint_index) = states.torque;
  }

  joint_positions(static_cast<Eigen::Index>(kThumbPipIndex)) -=
    joint_positions(static_cast<Eigen::Index>(kThumbMcpIndex));
  joint_positions(static_cast<Eigen::Index>(kIndexPipIndex)) -=
    joint_positions(static_cast<Eigen::Index>(kIndexMcpIndex));
  joint_positions(static_cast<Eigen::Index>(kMiddlePipIndex)) -=
    joint_positions(static_cast<Eigen::Index>(kMiddleMcpIndex));

  JointMapd joint_position_map(joint_states.position_data());
  JointMapd joint_velocity_map(joint_states.velocity_data());
  JointMapd joint_effort_map(joint_states.effort_data());

  joint_position_map = joint_positions.cast<double>();
  joint_velocity_map = joint_velocities.cast<double>();
  joint_effort_map = joint_efforts.cast<double>();
}

void AristoModel::build_impedance_targets(
  const std::vector<aristo_actuator::Actuator> & actuators,
  const can_hardware_common::RobotIO::JointCommand & joint_commands,
  can_hardware_common::RobotIO::ActuatorCommand & actuator_commands,
  std::vector<can_hardware_common::ActuatorTarget> & impedance_targets) const
{
  const ConstJointMapd joint_position_cmd_map(joint_commands.position_data());
  const ConstJointMapd joint_velocity_cmd_map(joint_commands.velocity_data());
  const ConstJointMapd joint_stiffness_cmd_map(joint_commands.stiffness_data());
  const ConstJointMapd joint_damping_cmd_map(joint_commands.damping_data());
  const ConstJointMapd joint_torque_cmd_map(joint_commands.effort_data());

  JointArrayf joint_position_cmd = joint_position_cmd_map.cast<float>();
  const JointArrayf joint_velocity_cmd = joint_velocity_cmd_map.cast<float>();
  const JointArrayf joint_stiffness_cmd = joint_stiffness_cmd_map.cast<float>();
  const JointArrayf joint_damping_cmd = joint_damping_cmd_map.cast<float>();
  const JointArrayf joint_torque_cmd = joint_torque_cmd_map.cast<float>();

  joint_position_cmd(static_cast<Eigen::Index>(kThumbPipIndex)) +=
    actuators[kThumbMcpIndex].get_feedback().position;
  joint_position_cmd(static_cast<Eigen::Index>(kIndexPipIndex)) +=
    actuators[kIndexMcpIndex].get_feedback().position;
  joint_position_cmd(static_cast<Eigen::Index>(kMiddlePipIndex)) +=
    actuators[kMiddleMcpIndex].get_feedback().position;

  impedance_targets.clear();
  impedance_targets.reserve(actuators.size());
  for (std::size_t i = 0; i < actuators.size(); ++i) {
    const Eigen::Index joint_index = static_cast<Eigen::Index>(i);
    actuator_commands.position_at(i) = joint_position_cmd(joint_index);
    actuator_commands.velocity_at(i) = joint_velocity_cmd(joint_index);
    actuator_commands.effort_at(i) = joint_torque_cmd(joint_index);
    actuator_commands.stiffness_at(i) = joint_stiffness_cmd(joint_index);
    actuator_commands.damping_at(i) = joint_damping_cmd(joint_index);

    impedance_targets.push_back(can_hardware_common::ActuatorTarget{
      joint_position_cmd(joint_index),
      joint_velocity_cmd(joint_index),
      joint_stiffness_cmd(joint_index),
      joint_damping_cmd(joint_index),
      joint_torque_cmd(joint_index)});
  }
}

bool AristoModel::actuators_ready(const std::vector<aristo_actuator::Actuator> & actuators) const
{
  return can_hardware_common::RobotIO::any_actuator_ready(
    actuators,
    [](const auto & actuator) { return actuator.has_feedback(); });
}

void AristoModel::copy_feedback_snapshot(
  const std::vector<aristo_actuator::Actuator> & actuators,
  can_hardware_common::core::StateSnapshot & snapshot) const
{
  can_hardware_common::RobotIO::copy_actuator_feedback_to_snapshot(
    actuators,
    snapshot,
    [](const auto & actuator) { return actuator.get_feedback(); });
}

}  // namespace aristo_hand
