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
};

class Actuator{
private:
    pcan_interface::PCANInterface& pcan_interface_;

    can_protocol::MsgEncoder encoder_;
    can_protocol::MsgDecoder decoder_;

    Commands commands_;
    States states_;
    Gains gains_;
    Status status_;
    Config config_;  

public:
    Actuator(pcan_interface::PCANInterface& pcan_interface, const Config& config);

    void set_joint_torque(const float &joint_torque, const uint32_t &duration);

    void set_joint_position(const float &joint_position, const uint32_t &duration);

    void get_joint_torque(float &joint_torque);

    void get_joint_velocity(float &joint_velocity);

    void get_joint_position(float &joint_position);

    void set_joint_gains(const Gains &gains);

    void process_message(const TPCANMsg &msg){
        
        // Check the Command Byte of the Received Message and call the corresponding function
        // In switch statement, check the Message with higher priority first (msg such as motion control msgs which are updated every loop)
        // uint8_t command_byte = msg.DATA[0];
        switch (msg.DATA[0]){

            // Response Message from the Motion Control
            case CommandByte::POSITION_CONTROL:
            case CommandByte::SPEED_CONTROL:
            case CommandByte::TORQUE_CONTROL:
                // get the states if there is a valid response without any error
                decoder_.get_states(msg, status_.temperature, states_.position, states_.velocity, states_.torque);
                
                break;

            // Gain message reponse upon request to get the gains from the motor
           case CommandByte::RETRIVE_PARAMETER:
                // If the parameter exists in the map, retrieve the pointer to the corresponding gain variable, dereference it, and pass it to the get_gain function.
                // uint8_t param_id = msg.DATA[1];
                break;

            /////////////////////////// RESPONSE MESSAGES WITHOUT SIGNIFICANT DATA ///////////////////////////

            // Response Message without encoder data; noting to read besides the result
            case CommandByte::START_MOTOR:  
            case CommandByte::STOP_MOTOR: 
            case CommandByte::STOP_CONTROL:
                can_protocol::get_result(msg, msg.DATA[1]);
                break;

            // Gain message response upon setting the gains; noting to read besides the result
            case CommandByte::MODIFY_PARAMETER:
                can_protocol::get_result(msg, msg.DATA[2]);
                break;
        }
    }    


private:

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

} // namespace actuator







#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_