#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_

#include "plato2_hardware_interface/pcan_interface.hpp"
#include "plato2_hardware_interface/can_protocol.hpp"

namespace actuator{

/// @brief Actuator commands variables : position, torque
struct Commands{
    float position;
    float torque;
};

/// @brief Actuator states variables : position, velocity, torque
struct States{
    float position;
    float velocity;
    float torque;
};

struct Gains{
    uint32_t kp_velocity;
    uint32_t ki_velocity;
    uint32_t kp_position;
    uint32_t ki_position;
};

struct Status{
    float voltage;
    float current;
    uint8_t temperature;
};

struct Config{
    const uint8_t can_id;
    const float position_offset;
    const char direction; // 1 for counter-clockwise, -1 for clockwise
};

class Actuator{
private:
    pcan_interface::PCANInterface& pcan_interface_;

    can_protocol::ControlMessage control_msg_;
    can_protocol::ResponseMessage response_msg_;
    can_protocol::GainMessage gain_msg_;
    can_protocol::StatusMessage status_msg_;

    Commands commands_;
    States states_;
    Gains gains_;
    Status status_;
    Config config_;  

public:
    Actuator(pcan_interface::PCANInterface& pcan_interface, const Config& config)

    set_joint_torque(const float joint_torque, const uint32_t duration);

    set_joint_position(const float joint_position, const uint32_t duration);





private:

    TPCANMsg init_message_();

    /// @brief Convert joint command Value to Motor Command Value considering motor direction and offset
    /// @param joint_value joint command value from the robot
    /// @param motor_value reference to store the motor command value to send to the motor
    inline void joint_to_motor_(const &joint_value, &motor_value){
        // motor_value = joint_value * direction + offset
    }

    /// @brief convert motor state value to joint state value considering motor direction and offset
    /// @param motor_value Motor State Value from the motor
    /// @param  
    inline void motor_to_joint_(const &motor_value, &joint_value){

    }

    void send_message(const TPCANMsg &msg);

    void receive_message();

};

} // namespace actuator







#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_