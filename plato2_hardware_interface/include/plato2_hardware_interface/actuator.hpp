#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_

#include "plato2_hardware_interface/hardware_config/actuator_config.hpp"

#include "plato2_hardware_interface/pcan_interface.hpp"
#include "plato2_hardware_interface/mit_can_protocol.hpp"

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


struct Status{
    float voltage;
    float current;
    uint8_t temperature;
};

struct Config{
    const uint8_t can_tx_id;
    const uint8_t can_rx_id;
    
    const float position_offset;
    const char direction; // 1 for counter-clockwise, -1 for clockwise

    const float torque_constant;
    const float gear_ratio;

    const float joint_limit_max;
    const float joint_limit_min;
};

class Actuator{
private:     
    pcan_interface::PCANInterface& pcan_interface_;

    Commands commands_;
    States states_;
    Status status_;
    Config config_;

    can_protocol::MsgEncoder encoder_;
    can_protocol::MsgDecoder decoder_;

    /// @brief Motor Enable Status
    bool b_motor_enabled_;



public:
    /// @brief Constructor
    /// @param pcan_interface pcan_interface::PCANInterface object
    /// @param config Actuator configuration
    Actuator(pcan_interface::PCANInterface& pcan_interface, const Config& config);

    /// @brief Destructor
    ~Actuator();
   
    /// @brief Get the current states of the motor
    /// @return  States struct containing the current position, velocity and torque
    States get_states() const { return states_; }


    /// @brief Get the cached joint commands of the actuator
    /// @return  Commands struct containing the cached joint commands
    Commands get_commands() const { return commands_; }

    /// @brief Get the status of the motor
    /// @return  Status struct containing the current voltage, current and temperature
    Status get_status() const { return status_; }

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
            motor_value = (joint_value * config_.direction) + config_.position_offset;
            // std::cout << "Joint Cmd: " << joint_value << std::endl;

        } else {
            motor_value = joint_value * config_.direction;
        }
    }

    /// @brief convert motor state value to joint state value considering motor direction and offset
    /// @param motor_value Motor State Value from the motor
    /// @param joint_value reference to store the joint state value
    /// @param apply_offset boolean to apply the offset or not (only for position)
    inline void motor_to_joint_(const float &motor_value, float &joint_value, bool apply_offset = false) {
        if (apply_offset) {
            joint_value = (motor_value - config_.position_offset) * config_.direction;


        } else {
            joint_value = motor_value * config_.direction;
        }
        
    }

    /// @brief Safety margin (in radians) to begin limiting torque near joint limits
    static constexpr float JOINT_LIMIT_SAFETY_MARGIN = 0.05f;
    
    /// @brief Precalculated limit thresholds (calculated once in constructor)
    float min_limit_threshold_;
    float max_limit_threshold_;
    
    /// @brief Fast approximation for quadratic falloff near limits
    /// @param norm_dist Normalized distance from limit (0-1)
    /// @return Quadratic scale factor
    inline float fast_quad_scale_(float norm_dist) const {
        // Clamp input between 0-1
        norm_dist = norm_dist < 0.0f ? 0.0f : (norm_dist > 1.0f ? 1.0f : norm_dist);

        return norm_dist * norm_dist;
    }

    /// @brief Limits torque commands to prevent pressing against hard stops
    /// @param joint_torque Original torque command
    /// @param current_position Current joint position
    /// @return Modified torque that won't push against limits
    inline float limit_torque_near_bounds_(const float &joint_torque, const float &current_position) const {
        // Check minimum limit - negative torque would push toward min limit
        if (current_position < min_limit_threshold_ && joint_torque < 0) {
            // Fast math: linear mapping + quadratic scale
            const float norm_dist = (current_position - config_.joint_limit_min) / JOINT_LIMIT_SAFETY_MARGIN;
            return joint_torque * fast_quad_scale_(norm_dist);
        } 
        
        // Check maximum limit - positive torque would push toward max limit
        if (current_position > max_limit_threshold_ && joint_torque > 0) {
            // Fast math: linear mapping + quadratic scale
            const float norm_dist = (config_.joint_limit_max - current_position) / JOINT_LIMIT_SAFETY_MARGIN;
            return joint_torque * fast_quad_scale_(norm_dist);
        }
        
        // Default: no limiting needed
        return joint_torque;
    }
};

} // namespace actuator







#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_