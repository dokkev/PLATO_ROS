#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_

#include "plato2_hardware_interface/pcan_interface.hpp"
#include "plato2_hardware_interface/can_protocol.hpp"

namespace actuator{

/// @brief Actuator commands variables : position, torque
struct Commands{
    float position;
    float velocity;
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
    uint32_t kd_position;

    // unordered map for gain parameters
    std::unordered_map<uint8_t, uint32_t*> param_map = {
        {ParamID::KP_SPEED, &kp_velocity},
        {ParamID::KI_SPEED, &ki_velocity},
        {ParamID::KP_POSITION, &kp_position},
        {ParamID::KI_POSITION, &ki_position},
        {ParamID::KD_POSITION, &kd_position}
    };
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

    const float torque_constant;
    const float gear_ratio;
};

class Actuator{
private:     
    pcan_interface::PCANInterface& pcan_interface_;

    Commands commands_;
    States states_;
    Gains gains_;
    Status status_;
    Config config_;

    can_protocol::MsgEncoder encoder_;
    can_protocol::MsgDecoder decoder_;

    // unordered map for gain parameters
    std::unordered_map<uint8_t, uint32_t*> gain_map_ = {
        {ParamID::KP_SPEED,    &gains_.kp_velocity},
        {ParamID::KI_SPEED,    &gains_.ki_velocity},
        {ParamID::KP_POSITION, &gains_.kp_position},
        {ParamID::KI_POSITION, &gains_.ki_position},
        {ParamID::KD_POSITION, &gains_.kd_position}
    };


public:
    Actuator(pcan_interface::PCANInterface& pcan_interface, Config& config);

    ~Actuator();
   

    States get_states() const { return states_; }

    Gains get_gains() const { return gains_; }

    void enable_motor();

    void disable_motor();

    void stop_control();
    
    void set_joint_torque(const float &joint_torque, const uint32_t &duration);

    void set_joint_velocity(const float &joint_velocity, const uint32_t &duration);

    void set_joint_position(const float &joint_position, const uint32_t &duration);

    void process_message(const TPCANMsg &msg);
        

private:
    /// @brief Cached message to send to the motor
    TPCANMsg onoff_msg_;
    TPCANMsg pos_msg_;
    TPCANMsg vel_msg_;
    TPCANMsg trq_msg_;

    /// @brief Initialize a message with with 0 data and the configured CAN ID
    /// @return TPCANMsg initialized message
    inline TPCANMsg init_message_(){
        TPCANMsg msg;
        msg.ID = config_.can_id;
        msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
        msg.LEN = 8;
        std::memset(msg.DATA, 0, 8);

        return msg;
    }

    /// @brief Convert joint command Value to Motor Command Value considering motor direction and offset
    /// @param joint_value joint command value from the robot
    /// @param motor_value reference to store the motor command value to send to the motor
    inline void joint_to_motor_(const float &joint_value, float &motor_value){
        motor_value = (joint_value * config_.direction) + config_.position_offset;
    }

    /// @brief convert motor state value to joint state value considering motor direction and offset
    /// @param motor_value Motor State Value from the motor
    /// @param joint_value reference to store the joint state value
    inline void motor_to_joint(const float &motor_value, float &joint_value){
        joint_value = (motor_value - config_.position_offset) * config_.direction;
    }


   
   

    
};

// some useful functions
inline bool almost_equal(float a, float b, float epsilon = 1e-5f) {
    return std::fabs(a - b) < epsilon;
}

} // namespace actuator







#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_