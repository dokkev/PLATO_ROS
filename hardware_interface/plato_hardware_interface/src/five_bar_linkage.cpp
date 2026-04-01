#include "plato_hardware_interface/five_bar_linkage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace FiveBarLinkage
{

namespace
{

constexpr float kDefaultAmplification = 1.0f;
constexpr float kEpsilon = 1e-6f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kSteadywinTorqueScale = 8.0f;
constexpr size_t kThumbRollIndex = 0;
constexpr size_t kThumbYawIndex = 1;
constexpr size_t kThumbMcpIndex = 2;
constexpr size_t kThumbPipIndex = 3;
constexpr size_t kIndexMcpIndex = 4;
constexpr size_t kIndexPipIndex = 5;
constexpr size_t kMiddleMcpIndex = 6;
constexpr size_t kMiddlePipIndex = 7;
constexpr float kInfinity = std::numeric_limits<float>::infinity();

float clamp_joint_effort_command(float joint_effort, float joint_effort_limit)
{
  if (!std::isfinite(joint_effort_limit)) {
    return joint_effort;
  }
  return std::clamp(joint_effort, -joint_effort_limit, joint_effort_limit);
}

}  // namespace

Transmission::Transmission(FiveBarLinkageConfig config)
: Transmission(
    config,
    JointArray::Constant(kInfinity))
{
}

Transmission::Transmission(
  FiveBarLinkageConfig config,
  const JointArray & joint_effort_limits)
: config_(config),
  joint_effort_limits_(joint_effort_limits)
{
}

void Transmission::compute_ratios_(
  const can_hardware_common::RobotIO::ActuatorState & actuator_state,
  JointArray & position_ratios,
  JointArray & velocity_ratios,
  JointArray & torque_ratios) const
{
  position_ratios.setOnes();
  velocity_ratios.setOnes();
  torque_ratios.setOnes();

  if (actuator_state.size() != kNumActuators) {
    return;
  }

  const auto actuator_state_view = actuator_state.const_view();
  const JointArray actuator_positions = actuator_state_view.position.cast<float>();

  auto apply_kinematics =
    [this, &actuator_positions, &position_ratios, &velocity_ratios, &torque_ratios](
      size_t mcp_index,
      size_t pip_index,
      float sign)
    {
      const Eigen::Index mcp = static_cast<Eigen::Index>(mcp_index);
      const Eigen::Index pip = static_cast<Eigen::Index>(pip_index);
      const float mcp_position = actuator_positions(mcp);
      const float pip_position = actuator_positions(pip);

      if (!std::isfinite(mcp_position) || !std::isfinite(pip_position)) {
        return;
      }

      const auto kinematics = compute_kinematics(config_, sign * mcp_position, sign * pip_position);
      position_ratios(pip) = kinematics.position_amplification;
      velocity_ratios(pip) = 1.0f / kinematics.torque_amplification;
      torque_ratios(pip) = kinematics.torque_amplification;
    };

  apply_kinematics(kThumbMcpIndex, kThumbPipIndex, 1.0f);
  apply_kinematics(kIndexMcpIndex, kIndexPipIndex, -1.0f);
  apply_kinematics(kMiddleMcpIndex, kMiddlePipIndex, -1.0f);
}

void Transmission::actuator_to_joint(
  const can_hardware_common::RobotIO::ActuatorState & actuator_state,
  can_hardware_common::RobotIO::JointState & joint_state) const
{
  if (actuator_state.size() != kNumActuators || joint_state.size() != kNumJoints) {
    return;
  }

  JointArray position_ratios;
  JointArray velocity_ratios;
  JointArray torque_ratios;
  compute_ratios_(actuator_state, position_ratios, velocity_ratios, torque_ratios);

  const auto actuator_state_view = actuator_state.const_view();
  const JointArray actuator_positions = actuator_state_view.position.cast<float>();
  const JointArray actuator_velocities = actuator_state_view.velocity.cast<float>();
  const JointArray actuator_efforts = actuator_state_view.effort.cast<float>();

  auto joint_state_view = joint_state.view();
  joint_state_view.position = (actuator_positions * position_ratios).cast<double>();
  joint_state_view.velocity = (actuator_velocities * velocity_ratios).cast<double>();

  JointArray joint_effort = (actuator_efforts / kSteadywinTorqueScale) * torque_ratios;
  joint_effort.segment<2>(static_cast<Eigen::Index>(kThumbRollIndex)).setZero();
  joint_state_view.effort = joint_effort.cast<double>();
}

void Transmission::joint_to_actuator(
  const can_hardware_common::RobotIO::JointCommand & joint_command,
  const can_hardware_common::RobotIO::ActuatorState & actuator_state,
  const can_hardware_common::RobotIO::JointState & joint_state,
  can_hardware_common::RobotIO::ActuatorCommand & actuator_command) const
{
  (void)joint_state;

  if (
    joint_command.size() != kNumJoints ||
    actuator_state.size() != kNumActuators ||
    joint_state.size() != kNumJoints ||
    actuator_command.size() != kNumActuators)
  {
    return;
  }

  JointArray position_ratios;
  JointArray velocity_ratios;
  JointArray torque_ratios;
  compute_ratios_(actuator_state, position_ratios, velocity_ratios, torque_ratios);

  const auto joint_command_view = joint_command.const_view();

  JointArray actuator_positions = JointArray::Constant(std::numeric_limits<float>::quiet_NaN());
  JointArray actuator_velocities = JointArray::Constant(std::numeric_limits<float>::quiet_NaN());
  JointArray actuator_efforts = JointArray::Constant(std::numeric_limits<float>::quiet_NaN());
  JointArray actuator_stiffness = JointArray::Constant(std::numeric_limits<float>::quiet_NaN());
  JointArray actuator_damping = JointArray::Constant(std::numeric_limits<float>::quiet_NaN());

  for (size_t i = 0; i < kNumActuators; ++i) {
    const Eigen::Index index = static_cast<Eigen::Index>(i);
    const float position_ratio = position_ratios(index);
    const float velocity_ratio = velocity_ratios(index);
    const float torque_ratio = torque_ratios(index);

    if (std::isfinite(position_ratio) && std::abs(position_ratio) > kEpsilon) {
      actuator_positions(index) =
        static_cast<float>(joint_command_view.position(index)) / position_ratio;
    }

    if (std::isfinite(velocity_ratio) && std::abs(velocity_ratio) > kEpsilon) {
      actuator_velocities(index) =
        static_cast<float>(joint_command_view.velocity(index)) / velocity_ratio;
    }

    if (std::isfinite(torque_ratio) && std::abs(torque_ratio) > kEpsilon) {
      const float joint_effort =
        clamp_joint_effort_command(
        static_cast<float>(joint_command_view.effort(index)),
        joint_effort_limits_(index));
      actuator_efforts(index) =
        joint_effort / torque_ratio;
      actuator_stiffness(index) =
        static_cast<float>(joint_command_view.stiffness(index)) / torque_ratio;
      actuator_damping(index) =
        static_cast<float>(joint_command_view.damping(index)) / torque_ratio;
    }
  }

  for (size_t i = 0; i < kNumActuators; ++i) {
    const Eigen::Index index = static_cast<Eigen::Index>(i);
    actuator_command.position_at(i) = static_cast<double>(actuator_positions(index));
    actuator_command.velocity_at(i) = static_cast<double>(actuator_velocities(index));
    actuator_command.effort_at(i) = static_cast<double>(actuator_efforts(index));
    actuator_command.stiffness_at(i) = static_cast<double>(actuator_stiffness(index));
    actuator_command.damping_at(i) = static_cast<double>(actuator_damping(index));
  }
}

Kinematics compute_kinematics(
  const FiveBarLinkageConfig & config,
  float mcp_motor_angle,
  float pip_motor_angle)
{
  Kinematics kinematics;

  const float theta1 = pip_motor_angle + config.eef_linkage_angle;
  const float theta4 = mcp_motor_angle;

  const float s_theta1 = std::sin(theta1);
  const float c_theta1 = std::cos(theta1);
  const float s_theta4 = std::sin(theta4);
  const float c_theta4 = std::cos(theta4);

  const float A = 2.0f * config.L3 * config.L4 * s_theta4 - 2.0f * config.L3 * config.L1 * s_theta1;
  const float B = 2.0f * config.L3 * config.L5 - 2.0f * config.L1 * config.L3 * c_theta1 +
    2.0f * config.L3 * config.L4 * c_theta4;
  const float C = std::pow(config.L1, 2) - std::pow(config.L2, 2) + std::pow(config.L3, 2) +
    std::pow(config.L4, 2) + std::pow(config.L5, 2) -
    2.0f * config.L1 * config.L4 * s_theta1 * s_theta4 -
    2.0f * config.L1 * config.L5 * c_theta1 +
    2.0f * config.L4 * config.L5 * c_theta4 -
    2.0f * config.L1 * config.L4 * c_theta1 * c_theta4;

  const float discriminant = A * A + B * B - C * C;
  if (discriminant <= 0.0f) {
    return kinematics;
  }

  const float tan_x = A + std::sqrt(discriminant);
  const float tan_y = B - C;
  const float theta3 = kPi - 2.0f * std::atan2(tan_y, tan_x);
  const float s_theta3 = std::sin(theta3);
  const float theta2_argument =
    (config.L3 * s_theta3 + config.L4 * s_theta4 - config.L1 * s_theta1) / config.L2;
  if (theta2_argument < -1.0f || theta2_argument > 1.0f) {
    return kinematics;
  }

  const float theta2 = std::asin(theta2_argument);
  const float position_denominator = std::abs(pip_motor_angle);
  const float torque_denominator = config.L1 * std::sin(theta1 - theta2);

  if (position_denominator > kEpsilon) {
    kinematics.position_amplification =
      (theta3 - config.eef_linkage_angle - theta4) / pip_motor_angle;
  }

  if (std::abs(torque_denominator) > kEpsilon) {
    kinematics.torque_amplification =
      std::abs((-config.L3 * std::sin(theta2 - theta3)) / torque_denominator);
  }

  if (!std::isfinite(kinematics.position_amplification)) {
    kinematics.position_amplification = kDefaultAmplification;
  }
  if (!std::isfinite(kinematics.torque_amplification) || kinematics.torque_amplification <= kEpsilon) {
    kinematics.torque_amplification = kDefaultAmplification;
  }

  return kinematics;
}

}  // namespace FiveBarLinkage
