#ifndef PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_
#define PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <cmath> 
#include <iostream>

#include "PCANBasic.h"

#include "plato2_hardware_interface/hardware_config/can_ids.hpp"

namespace mit_can_protocol{


/// @brief Runtime motor motion control command message. ControlMessage takes TPCANMsg message and encode the control command following
/// the MIT Steadywin CAN Protocol for PCANInterface to send to the motor driver later
class MsgEncoder{

public:
    /// @brief Default constructor
    MsgEncoder(const float &gear_ratio, const float &torque_constant, const uint8_t &tx_id);

    // Build the 0xF0 frame (big-endian “hi, lo” per field).
    // DLC = 7 bytes: [0]=0xF0, [1..2]=Pos_Max, [3..4]=Vel_Max, [5..6]=T_Max
    /// @brief Set the limits for position, velocity, and torque
    /// @param pos_max_rad position in radians
    /// @param vel_max_rps velocity in radians per second
    /// @param t_max_nm torque in Newton-meters
    void set_limits(TPCANMsg& msg, const float pos_max_rad, const float vel_max_rps, const float t_max_nm);

    /// @brief Set the zero position ID to the message
    void set_zero_position(TPCANMsg &msg);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// COMMAND MESSAGE //////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the motor enable ID to the message
    void start_motor(TPCANMsg &msg);

    /// @brief Set the motor disable ID to the message. This is equivalent to the stop motor command in this protocol.
    void stop_motor(TPCANMsg &msg);

    /// @brief Set the stop control ID to the message. Current ongoing control command should be stopped immediately. 
    void stop_control(TPCANMsg &msg);

    /// @brief  Send command message to clear any fault and return to normal state
    /// @param msg TPCANMsg reference to store the command message
    void clear_fault(TPCANMsg &msg);

    // --- Motion commands (operation-control frame; no command byte) ---
    // Frame layout (StdID bit10=1):
    // [0..1]: target position (16b, signed mapped to 0..65535)
    // [2..3]: target velocity (12b: [2]=hi8, [3][7:4]=lo4)
    // [3..4]: Kp (12b: [3][3:0]=hi4, [4]=lo8), span 0..500
    // [5..6]: Kd (12b: [5]=hi8, [6][7:4]=lo4), span 0..5
    // [6..7]: target torque (12b: [6][3:0]=hi4, [7]=lo8)

    /// @brief Send the position, velocity, kp, kd and torque in one frame to the motor
    /// @param position_rad desired position in radians
    /// @param velocity_rps desired velocity in radians per second
    /// @param kp desired proportional gain
    /// @param kd desired velocity gain
    /// @param torque_nm desired torque in Newton-meters
    void set_impedance(TPCANMsg &msg, const float position_rad, 
                                        const float velocity_rps,
									    const float kp, 
                                        const float kd, 
                                        const float torque_nm);

private:
    /// @brief gear ratio of the motor initialized in the actuator constructor
    const float &gear_ratio_;

    /// @brief torque constant of the motor initialized in the actuator constructor
    const float &torque_constant_;

    /// @brief transmit ID for the motor
    const uint8_t &tx_id_;

};

class MsgDecoder{
public:
    /// @brief constructor
    MsgDecoder(const float &gear_ratio, const float &torque_constant);

    /// @brief retrieve the position, velocity, kp, kd and torque from the received message
    /// @param msg received TPCANMsg message
    /// @param position reference to store the decoded position value
    /// @param velocity reference to store the decoded velocity value
    /// @param kp reference to store the decoded proportional gain value
    /// @param kd reference to store the decoded derivative gain value
    /// @param torque reference to store the decoded torque value
    /// @param in_oc_mode reference to store the operation control mode status (true if in OC mode, false otherwise)
    /// @param has_fault reference to store the fault status
    void get_states(const TPCANMsg &msg, float &position, float &velocity, float &kp, float &kd, float &torque, bool &in_oc_mode, bool &has_fault) const;

    /// @brief retrieve the configured limits from the received message
    /// @param msg received TPCANMsg message
    /// @param pos_max_rad reference to store the decoded position limit value
    /// @param vel_max_rps reference to store the decoded velocity limit value
    /// @param tq_max_nm reference to store the decoded torque limit value
    void get_limits(const TPCANMsg &msg, float &pos_max_rad, float &vel_max_rps, float &tq_max_nm) const;

private:
    /// @brief gear ratio of the motor
    const float &gear_ratio_;

    /// @brief torque constant of the motor
    const float &torque_constant_; 
};

} // namespace mit_can_protocol

#endif // PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_