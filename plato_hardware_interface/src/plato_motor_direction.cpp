#include "plato_hardware_interface/plato_motor_direction.hpp"

namespace plato_motor_direction {

PlatoMotorDirection::PlatoMotorDirection()
    : motor_config_{{{MOTOR_0_DIRECTION, MOTOR_0_ANGLE_OFFSET, MOTOR_0_REDUCTION_RATIO},
                     {MOTOR_1_DIRECTION, MOTOR_1_ANGLE_OFFSET, MOTOR_1_REDUCTION_RATIO},
                     {MOTOR_2_DIRECTION, MOTOR_2_ANGLE_OFFSET, MOTOR_2_REDUCTION_RATIO},
                     {MOTOR_3_DIRECTION, MOTOR_3_ANGLE_OFFSET, MOTOR_3_REDUCTION_RATIO},
                     {MOTOR_4_DIRECTION, MOTOR_4_ANGLE_OFFSET, MOTOR_4_REDUCTION_RATIO},
                     {MOTOR_5_DIRECTION, MOTOR_5_ANGLE_OFFSET, MOTOR_5_REDUCTION_RATIO},
                     {MOTOR_6_DIRECTION, MOTOR_6_ANGLE_OFFSET, MOTOR_6_REDUCTION_RATIO},
                     {MOTOR_7_DIRECTION, MOTOR_7_ANGLE_OFFSET, MOTOR_7_REDUCTION_RATIO},
                     {MOTOR_8_DIRECTION, MOTOR_8_ANGLE_OFFSET, MOTOR_8_REDUCTION_RATIO}}} {}

void PlatoMotorDirection::convert_joint_to_motor_effort(
    const std::vector<double> &joint_effort_commands,
    std::vector<double> &motor_effort_commands) {
  // Ensure the output vector is correctly sized
  motor_effort_commands.resize(joint_effort_commands.size());

  for (std::size_t i = 0; i < joint_effort_commands.size(); ++i) {

    // Nan value to zero
    if (std::isnan(joint_effort_commands[i])) {
      motor_effort_commands[i] = 0.0;

      continue;
    }

    // Adjust the effort based on the motor's configuration
    motor_effort_commands[i] = motor_config_[i].is_cw
                                   ? joint_effort_commands[i]
                                   : -joint_effort_commands[i];
    motor_effort_commands[i] /=
        0.06; // Convert torque to current, where torque constant is 0.06 Nm/A
  }
}

void PlatoMotorDirection::convert_joint_to_motor_position(
    const std::vector<double> &joint_position_commands,
    std::vector<double> &motor_position_commands) {
  // Ensure the output vector is correctly sized
  motor_position_commands.resize(joint_position_commands.size());

  for (std::size_t i = 0; i < joint_position_commands.size(); ++i) {
    // Adjust the direction based on the motor's configuration
    motor_position_commands[i] = motor_config_[i].is_cw
                                     ? joint_position_commands[i]
                                     : -joint_position_commands[i];
    // Add the offset to the position
    motor_position_commands[i] += motor_config_[i].angle_offset;
  }
}

void PlatoMotorDirection::convert_motor_to_joint_position(
    const std::vector<double> &motor_position_states,
    std::vector<double> &joint_position_states) {
  // Ensure the output vector is correctly sized to match the input
  joint_position_states.resize(motor_position_states.size());

  for (std::size_t i = 0; i < motor_position_states.size(); ++i) {


    // apply reduction ration and subtract the angle offset to normalize the position
    double normalized_position =
        (motor_position_states[i] / motor_config_[i].reduction_ratio) - motor_config_[i].angle_offset;

    // Then, adjust the normalized position based on the motor's direction
    double adjusted_position =
        motor_config_[i].is_cw ? normalized_position : -normalized_position;


    // Update the joint position state with the adjusted position
    joint_position_states[i] = adjusted_position;
  }
}

} // namespace plato_motor_direction