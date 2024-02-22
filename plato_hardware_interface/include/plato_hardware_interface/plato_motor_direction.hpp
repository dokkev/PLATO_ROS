#ifndef PLATO_HARDWARE_INTERFACE__PLATO_MOTOR_DIRECTION_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO_MOTOR_DIRECTION_HPP_

#include "plato_hardware_interface/plato_common.hpp"

#include <vector>
#include <array>


namespace plato_motor_direction {


struct MotorConfig {
    bool is_cw;
    double angle_offset;
};

class PlatoMotorDirection{

public:
    PlatoMotorDirection();
    void set_motor_config(const std::array<MotorConfig, 9> &motor_config);

    void convert_joint_to_motor_effort(const std::vector<double>& joint_effort_commands, std::vector<double>& motor_effort_commands);

    void convert_motor_to_joint_position(const std::vector<double>&motor_position_states, std::vector<double>& joint_positon_states);


private:
    std::array<MotorConfig, 9> motor_config_;

};


} // namespace plato_motor_direction
#endif