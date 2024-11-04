#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_

#include "plato2_hardware_interface/hardware_config/actuator_config.hpp"

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
    uint32_t kp_velocity = 0;
    uint32_t ki_velocity = 0;
    uint32_t kp_position = 0;
    uint32_t ki_position = 0;
    uint32_t kd_position = 0;

    bool b_kp_velocity_updated = false;
    bool b_ki_velocity_updated = false;
    bool b_kp_position_updated = false;
    bool b_ki_position_updated = false;
    bool b_kd_position_updated = false;
};

struct Status{
    float voltage;
    float current;
    uint8_t temperature;
};

struct Config{
    const uint8_t can_tx_id;
    const uint8_t can_rx_id;
    const float position_offset;
    const float command_offset;
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

    /// @brief Motor Enable Status
    bool b_motor_enabled_;


    // unordered map for gain parameters
    std::unordered_map<uint8_t, uint32_t*> gain_map_ = {
        {ParamID::KP_SPEED,    &gains_.kp_velocity},
        {ParamID::KI_SPEED,    &gains_.ki_velocity},
        {ParamID::KP_POSITION, &gains_.kp_position},
        {ParamID::KI_POSITION, &gains_.ki_position},
        {ParamID::KD_POSITION, &gains_.kd_position}
    };


public:
    /// @brief Constructor
    /// @param pcan_interface pcan_interface::PCANInterface object
    /// @param config Actuator configuration
    Actuator(pcan_interface::PCANInterface& pcan_interface, Config& config);

    /// @brief Destructor
    ~Actuator();
   
    /// @brief Get the current states of the motor
    /// @return  States struct containing the current position, velocity and torque
    States get_states() const { return states_; }

    /// @brief Get the current gains of the motor
    /// @return  Gains struct containing the current gains
    Gains get_gains() const { return gains_; }

    /// @brief enable the motor 
    void enable_motor();

    /// @brief disable the motor. When the motor is in motion, it will go to stop position. Since we can't change the stop position, don't use this function while the motor is moving
    void disable_motor();

    /// @brief stop current ongoing control command immediately, and if there is no ongoing control command it is just ignored.
    void stop_control();
    
    /// @brief Send a torque command to the motor after applying offsets and direction. It ignores the command if the command is the almost equal as the previous command 
    /// @param joint_torque  float torque command in Nm +CCW, -CW
    /// @param duration uint32_t execution time in ms 
    void set_joint_torque(const float &joint_torque, const uint32_t &duration);

    /// @brief Send a velocity command to the motor after applying offsets and direction . It ignores the command if the command is the almost equal as the previous command
    /// @param joint_velocity float velocity command in rad/s +CCW, -CW
    /// @param duration uint32_t execution time in ms
    void set_joint_velocity(const float &joint_velocity, const uint32_t &duration);

    /// @brief Send a position command to the motor after applying offsets and direction. It ignores the command if the command is the almost equal as the previous command
    /// @param joint_position float position command in rad +CCW, -CW
    /// @param duration uint32_t execution time in ms 
    void set_joint_position(const float &joint_position, const uint32_t &duration);

    /// @brief Send a modify runtime gain parameter command to the motor
    /// @param gains Gains struct containing the new gains 
    void set_gains(const Gains &gains);

    /// @brief Send a command message to modify default gains of the motor
    void set_default_gains(const Gains &gains);

    /// @brief set zero position of the output shaft of the motor
    void set_zero_position(const float &zero_position);

    /// @brief Send a command to retrieve the position of the motor
    void retrieve_position();

    /// @brief Send a command to retrieve the gains of the motor
    void retrieve_gains();    

    /// @brief Given the received message, identify the type of message and process it to store the data in the buffer
    /// @param msg 
    void process_message(const TPCANMsg &msg);

    /// @brief Get the CAN TX ID of the motor
    /// @return uint CAN TX ID
    uint get_tx_id() const { return config_.can_tx_id; }

    /// @brief Get the CAN RX ID of the motor
    /// @return uint CAN RX ID
    uint get_rx_id() const { return config_.can_rx_id; }

    /// @brief Get the motor position without offset
    /// @return float motor position
    float get_motor_position() { return motor_position_; }

    void calibrate_encoder();

    void calibrate_phase_order();

    /// @brief Get the motor enable status
    bool b_is_enabled() { return b_motor_enabled_; }





private:
    /// @brief Motor position without offset
    float motor_position_;

    /// @brief Cached message to send to the motor
    TPCANMsg onoff_msg_;
    TPCANMsg pos_msg_;
    TPCANMsg vel_msg_;
    TPCANMsg trq_msg_;
    TPCANMsg gain_msg_;
    TPCANMsg ind_msg_;
    TPCANMsg config_msg_;

    /// @brief Initialize a message with with 0 data and the configured CAN ID
    /// @return TPCANMsg initialized message
    inline TPCANMsg init_message_(){
        TPCANMsg msg;
        msg.ID = config_.can_tx_id;
        msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
        msg.LEN = 8;
        std::memset(msg.DATA, 0, 8);

        return msg;
    }

    /// @brief Convert joint command Value to Motor Command Value considering motor direction and offset
    /// @param joint_value joint command value from the robot
    /// @param motor_value reference to store the motor command value to send to the motor
    /// @param apply_offset boolean to apply the offset or not (only for position)
    inline void joint_to_motor_(const float &joint_value, float &motor_value, bool apply_offset = false) {
        if (apply_offset) {
            motor_value = (joint_value * config_.direction) + config_.position_offset - config_.command_offset;
            std::cout << "Joint Cmd: " << joint_value << std::endl;

        } else {
            motor_value = joint_value * config_.direction;
        }
    }

    /// @brief convert motor state value to joint state value considering motor direction and offset
    /// @param motor_value Motor State Value from the motor
    /// @param joint_value reference to store the joint state value
    /// @param apply_offset boolean to apply the offset or not (only for position)
    void motor_to_joint_(const float &motor_value, float &joint_value, bool apply_offset = false) {
        if (apply_offset) {
            joint_value = (motor_value - config_.position_offset)* config_.direction;


        } else {
            joint_value = motor_value * config_.direction;
        }
        
    }
    
};

} // namespace actuator







#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_