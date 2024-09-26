#ifndef PLATO_HARDWARE_INTERFACE__STEADYWIN_CAN_PROTOCOL_HPP_
#define PLATO_HARDWARE_INTERFACE__STEADYWIN_CAN_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <cmath> 
#include <iostream>

#include "PCANBasic.h"

#include "plato2_hardware_interface/steadywin_can_ids.hpp"

namespace can_protocol{


/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// COMMAND MESSAGE //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Runtime motor control message builder for SteadyWin motor driver
class CommandMessage{

public:
    /// @brief Default constructor
    CommandMessage() = default;

    /// @brief Set the motor enable ID to the message
    void start_motor(TPCANMsg &msg);

    /// @brief Set the motor disable ID to the message
    void stop_motor(TPCANMsg &msg);

    /// @brief Set the stop control ID to the message. Current ongoing control command should be stopped immediately
    void stop_control(TPCANMsg &msg);

    /// @brief Set the torque command ID, torque value and duration to the message
    /// @param torque desired torque value
    /// @param duration execution time in ms
    /// @param msg TPCANMsg reference to store the command message
    void set_torque(const float torque, const uint32_t duration, TPCANMsg &msg);

    /// @brief Set the speed command ID, speed value and duration to the message
    /// @param speed desired speed value
    /// @param duration execution time in ms
    /// @param msg TPCANMsg reference to store the command message
    void set_velocity(const float velocity, const uint32_t duration, TPCANMsg &msg);

    /// @brief Set the position command ID, position value and duration to the message
    /// @param position in rad
    /// @param duration execution time in ms
    /// @param msg TPCANMsg reference to store the command message
    void set_position(const float position, const uint32_t duration, TPCANMsg &msg);

private:

    /// @brief encode target command value with LSB byte order
    /// @param value target command value
    /// @param buffer buffer to store the encoded value
    void encode_command_float_(const float value, TPCANMsg &msg) const;

    /// @brief encode 24-bit unsigned integer indicating control execution time in unit of ms.
    /// @param duration control execution time in unit of ms
    /// @param buffer buffer to store the encoded value
    void encode_duration_int_(const uint32_t duration, TPCANMsg &msg) const;

    /// @brief initialize the message with standard CAN message type and 8 bytes of data with 0
    /// @param msg 
    void init_message_(TPCANMsg &msg) const;


};

/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// STATE MESSAGE ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


class StateMessage{
public:


    /// @brief  get the result of the control command from the received message
    /// @param msg 
    /// @param result 
    void get_result(const TPCANMsg& msg, uint8_t& result) const;

    /// @brief get current temperature of motor or driver board depending on which is available or which is higher if both are available.
    /// @param msg 
    /// @param temperature 
    void get_temperature(const TPCANMsg& msg, int &temperature) const;

    /// @brief get the torque of the motor from the received message
    /// @param msg received TPACNMsg message
    /// @param torque reference to store the decoded torque value
    void get_torque(const TPCANMsg& msg, float& torque) const;

    /// @brief get the velocity of the motor from the received message
    /// @param msg received TPACNMsg message
    /// @param velocity reference to store the decoded velocity value
    void get_velocity(const TPCANMsg& msg, float& velocity) const;

    /// @brief get the position of the motor from the received message
    /// @param msg received TPACNMsg message
    /// @param position reference to store the decoded position value
    void get_position(const TPCANMsg& msg, float& position) const;
};




} // namespace can_protocol

#endif // PLATO_HARDWARE_INTERFACE__STEADYWIN_CAN_PROTOCOL_HPP_