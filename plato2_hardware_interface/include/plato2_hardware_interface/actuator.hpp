#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_

#include "plato2_hardware_interface/hardware_config/actuator_config.hpp"
#include "plato2_hardware_interface/pcan_interface.hpp"
#include "plato2_hardware_interface/mit_can_protocol.hpp"

namespace actuator {

/// @brief Actuator command values
struct Commands {
    float position;
    float velocity;
    float torque;
    float stiffness;
    float damping;
};

/// @brief Actuator state values
struct States {
    float position;
    float velocity;
    float torque;
    
    bool in_oc_mode;
    bool has_fault;
};

/// @brief Actuator status values
struct Status {
    float voltage;
    float current;
    uint8_t temperature;
};

/// @brief Actuator configuration
struct Config {
    const uint8_t can_tx_id;
    const uint8_t can_rx_id;
    const float position_offset;
    const char direction;  // 1 for CCW, -1 for CW
    const float torque_constant;
    const float gear_ratio;
    const float joint_limit_max;
    const float joint_limit_min;
};

class Actuator {
public:
    /// @brief Constructor
    Actuator(pcan_interface::PCANInterface& pcan_interface, const Config& config);

    /// @brief Destructor
    ~Actuator();
   
    /// @brief Get current actuator states
    States get_states() const { return states_; }

    /// @brief Get cached actuator commands
    Commands get_commands() const { return commands_; }

    /// @brief Get actuator status
    Status get_status() const { return status_; }

    /// @brief Enable the motor
    void enable_motor();

    /// @brief Disable the motor
    void disable_motor();

    /// @brief Set current position as zero reference
    void set_current_position_as_zero();
    
    /// @brief Send torque command
    /// @param joint_torque Torque in Nm (+CCW, -CW)
    /// @param duration Execution time in ms
    void set_joint_torque(float joint_torque, uint32_t duration);

    /// @brief Send impedance control command
    /// @param joint_position Target position
    /// @param joint_velocity Target velocity
    /// @param joint_stiffness Stiffness gain (Kp)
    /// @param joint_damping Damping gain (Kd)
    /// @param joint_torque Feedforward torque
    void set_joint_impedance(float joint_position, 
                             float joint_velocity, 
                             float joint_stiffness, 
                             float joint_damping, 
                             float joint_torque);

    /// @brief Process received CAN message
    void process_message(const TPCANMsg &msg);

    /// @brief Get CAN TX ID
    uint get_tx_id() const { return config_.can_tx_id; }

    /// @brief Get CAN RX ID
    uint get_rx_id() const { return config_.can_rx_id; }

    /// @brief Get raw motor position (without offset)
    float get_motor_position() const { return motor_position_; }

    /// @brief Check if motor is enabled
    bool is_enabled() const { return b_motor_enabled_; }

private:
    /// @brief Process state messages (MIT control responses)
    void process_state_message(const TPCANMsg &msg);

    /// @brief Process limits configuration messages
    void process_limits_message(const TPCANMsg &msg);

    /// @brief Initialize a CAN message with configured ID
    inline TPCANMsg init_message_(uint8_t len) {
        TPCANMsg msg;
        std::memset(&msg, 0, sizeof(msg));
        msg.ID = config_.can_tx_id;
        msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
        msg.LEN = (len > 8) ? 8 : len;
        return msg;
    }

    /// @brief Convert joint value to motor value (direction only)
    inline void joint_to_motor_(float joint_value, float &motor_value) {
        motor_value = joint_value * config_.direction;
    }

    /// @brief Convert motor value to joint value (direction only)
    inline void motor_to_joint_(float motor_value, float &joint_value) {
        joint_value = motor_value * config_.direction;
    }

    // Hardware interface
    pcan_interface::PCANInterface& pcan_interface_;

    // State and command data
    Commands commands_;
    States states_;
    Status status_;
    Config config_;

    // MIT CAN protocol handlers
    mit_can_protocol::MsgEncoder encoder_;
    mit_can_protocol::MsgDecoder decoder_;

    // Cached CAN messages
    TPCANMsg onoff_msg_;
    TPCANMsg cmd_msg_;
    TPCANMsg config_msg_;
    TPCANMsg calibrate_msg_;

    // Runtime state
    float motor_position_;
    bool b_motor_enabled_;
};

} // namespace actuator

#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_HPP_