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

    const float position_limit_max;
    const float position_limit_min;
    const float velocity_limit;
    const float effort_limit;
    const float stiffness_limit;
    const float damping_limit;
};

/// @brief Factory functions to create actuator configurations
namespace ActuatorConfigFactory {
    
    inline Config create_thumb_roll_config() {
        return {
            MotorTxID::MOTOR1, MotorRxID::MOTOR1,
            MotorOffset::MOTOR1, MotorDirection::MOTOR1,
            GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO,
            JointPositionLimit::THUMB_ROLL_MAX, JointPositionLimit::THUMB_ROLL_MIN,
            JointVelocityLimit::THUMB_ROLL, JointEffortLimit::THUMB_ROLL,
            JointStiffnessLimit::THUMB_ROLL, JointDampingLimit::THUMB_ROLL
        };
    }
    
    inline Config create_thumb_yaw_config() {
        return {
            MotorTxID::MOTOR2, MotorRxID::MOTOR2,
            MotorOffset::MOTOR2, MotorDirection::MOTOR2,
            GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO,
            JointPositionLimit::THUMB_YAW_MAX, JointPositionLimit::THUMB_YAW_MIN,
            JointVelocityLimit::THUMB_YAW, JointEffortLimit::THUMB_YAW,
            JointStiffnessLimit::THUMB_YAW, JointDampingLimit::THUMB_YAW
        };
    }
    
    inline Config create_mcp_config(uint8_t motor_num) {
        const uint8_t tx_id = MotorTxID::MOTOR1 + motor_num - 1;
        const uint8_t rx_id = MotorRxID::MOTOR1 + motor_num - 1;
        const float* offsets = &MotorOffset::MOTOR1;
        const char* directions = &MotorDirection::MOTOR1;
        
        return {
            tx_id, rx_id,
            offsets[motor_num - 1], directions[motor_num - 1],
            GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO,
            JointPositionLimit::MCP_MAX, JointPositionLimit::MCP_MIN,
            JointVelocityLimit::MCP, JointEffortLimit::MCP,
            JointStiffnessLimit::MCP, JointDampingLimit::MCP
        };
    }
    
    inline Config create_pip_config(uint8_t motor_num) {
        const uint8_t tx_id = MotorTxID::MOTOR1 + motor_num - 1;
        const uint8_t rx_id = MotorRxID::MOTOR1 + motor_num - 1;
        const float* offsets = &MotorOffset::MOTOR1;
        const char* directions = &MotorDirection::MOTOR1;
        
        return {
            tx_id, rx_id,
            offsets[motor_num - 1], directions[motor_num - 1],
            GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO,
            JointPositionLimit::PIP_MAX, JointPositionLimit::PIP_MIN,
            JointVelocityLimit::PIP, JointEffortLimit::PIP,
            JointStiffnessLimit::PIP, JointDampingLimit::PIP
        };
    }
}

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
    

    /// @brief Send impedance control command
    /// @param q_des Target position [rad]
    /// @param qd_des Target velocity [rad/s]
    /// @param K_nom Nominal stiffness gain (Kp) [Nm/rad]
    /// @param B_nom Nominal damping gain (Kd) [Nms/rad]
    /// @param tau_ff Feedforward torque [Nm]
    void set_joint_impedance(float q_des, 
                             float qd_des, 
                             float K_nom, 
                             float B_nom, 
                             float tau_ff);

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

    /// @brief Convert joint value to motor value 
    inline float joint_to_motor_(float joint_value) const {
        return joint_value * config_.direction;
    }

    /// @brief Convert motor value to joint value
    inline float motor_to_joint_(float motor_value) const {
        return motor_value * config_.direction;
    }

    /// @brief Soft limit state machine
    enum class SoftLimitState {
        kOperational,
        kUpperLimit,    // Approaching upper limit
        kLowerLimit,    // Approaching lower limit
        kOverLimit      // Over limit - zero all commands
    };

    /// @brief Check joint limits and return false if exceeded
    /// @param joint_position Current joint position [rad]
    /// @return true if within limits, false otherwise
    bool check_joint_limits_(float joint_position) const;

    /// @brief Determine current soft limit state based on position and velocity
    void determine_current_state_();

    /// @brief Clamp all commands to their respective limits
    /// @param joint_position Joint position command [rad]
    /// @param joint_velocity Joint velocity command [rad/s]
    /// @param joint_stiffness Joint stiffness command [Nm/rad]
    /// @param joint_damping Joint damping command [Nms/rad]
    /// @param joint_torque Joint torque command [Nm]
    void clamp_commands_(float& joint_position, float& joint_velocity, 
                        float& joint_stiffness, float& joint_damping, 
                        float& joint_torque) const;

    // Hardware interface
    pcan_interface::PCANInterface& pcan_interface_;

    // Soft limit parameters
    static constexpr float safety_margin_ = 0.174f;      // Distance to start gradient torque [rad] (~10 deg)
    static constexpr float hysteresis_margin_ = 0.02f;   // Hysteresis to prevent state chattering [rad]

    // State and command data
    Commands commands_;
    States states_;
    Status status_;
    Config config_;
    SoftLimitState control_state_;

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