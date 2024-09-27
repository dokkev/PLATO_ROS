#ifndef PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_
#define PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <cmath> 
#include <iostream>

#include "PCANBasic.h"

#include "plato2_hardware_interface/can_ids.hpp"

namespace can_protocol{


/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// COMMAND MESSAGE //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Runtime motor motion control command message. ControlMessage takes TPCANMsg message and encode the control command following the Steadywin
/// CAN Protocol for PCANInterface to send to the motor driver later
class ControlMessage{

public:
    /// @brief Default constructor
    ControlMessage() = default;

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
    void set_torque(TPCANMsg &msg, const float torque, const uint32_t duration);

    /// @brief Set the speed command ID, speed value and duration to the message
    /// @param speed desired speed value
    /// @param duration execution time in ms
    /// @param msg TPCANMsg reference to store the command message
    void set_velocity(TPCANMsg &msg, const float velocity, const uint32_t duration);

    /// @brief Set the position command ID, position value and duration to the message
    /// @param position in rad
    /// @param duration execution time in ms
    /// @param msg TPCANMsg reference to store the command message
    void set_position(TPCANMsg &msg, const float position, const uint32_t duration);


    /// @brief  On any fault, motor would stop running and wait host command. If you want to continue
    /// running, this COMMAND should be sent to erase fault status and return to normal state. If fault
    /// is not acknowledged in runtime, motor driver will decline any COMMANDs from host.
    /// @param msg TPCANMsg reference to store the command message
    void acknowledge_fault(TPCANMsg &msg);

private:

    /// @brief encode target command value with LSB byte order
    /// @param value target command value
    /// @param msg buffer to store the encoded value
    inline void encode_command_float_(TPCANMsg &msg, const float value) const {
        // Copy the float value to the buffer using little-endian byte order
        // most CPU architectures are little-endian, but make sure that the byte order is correct
        std::memcpy(&msg.DATA[1], &value, sizeof(float));
    }

    /// @brief encode 24-bit unsigned integer indicating control execution time in unit of ms.
    /// @param duration control execution time in unit of ms
    /// @param msg buffer to store the encoded value
    inline void encode_duration_int_( TPCANMsg &msg, const uint32_t duration) const {
        // Copy the 24-bit unsigned integer value to the buffer using little-endian byte order
        msg.DATA[5] = duration & 0xFF;
        msg.DATA[6] = (duration >> 8) & 0xFF;
        msg.DATA[7] = (duration >> 16) & 0xFF;
    }
};

/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// STATE MESSAGE ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


class StateMessage{
public:
    /// @brief Default constructor
    StateMessage() = default;

    /// @brief get the result of the response message from the command message if successful return true otherwise false
    /// @param msg 
    /// @return 
    bool get_result(const TPCANMsg& msg) const;

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

private:
    /// @brief torque constant of the motor
    const float torque_constant_ = 0.41f;

    /// @brief gear ratio of the motor
    const float gear_ratio_ = 8.0f;

};

/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// GAIN MESSAGE /////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

class GainMessage{
public:
    /// @brief Default constructor
    GainMessage() = default;

    void set_kp_velocity(TPCANMsg &msg, const uint32_t kp_velocity);

    void set_ki_velocity(TPCANMsg &msg, const uint32_t  ki_velocity);

    void set_kp_position(TPCANMsg &msg, const uint32_t  kp_position);

    void set_ki_position(TPCANMsg &msg, const uint32_t  ki_position);

    void set_kd_position(TPCANMsg &msg, const uint32_t  kd_position);

    void get_kp_velocity(const TPCANMsg &msg, uint32_t  &kp_velocity);

    void get_ki_velocity(const TPCANMsg &msg, uint32_t  &ki_velocity);

    void get_kp_position(const TPCANMsg &msg, uint32_t  &kp_position);

    void get_ki_position(const TPCANMsg &msg, uint32_t  &ki_position);

    void get_kd_position(const TPCANMsg &msg, uint32_t  &kd_position);

private:

    /// @brief encode 32-bit unsigned integer value to the message buffer's BYTE4 to BYTE7
    /// @param msg reference  TPCANMsg to store the encoded value
    /// @param value desired 32-bit unsigned integer value of the gain parameter
    inline void encode_param_int_(TPCANMsg &msg, const uint32_t value) const {
        // Copy the 32-bit unsigned integer value to the buffer using little-endian byte order
        msg.DATA[4] = value & 0xFF;
        msg.DATA[5] = (value >> 8) & 0xFF;
        msg.DATA[6] = (value >> 16) & 0xFF;
        msg.DATA[7] = (value >> 24) & 0xFF;
    }

    /// @brief decode 32-bit unsigned integer value from the message buffer's BYTE4 to BYTE7
    /// @param msg recevied TPCANMsg message
    /// @param value reference to store the decoded 32-bit unsigned integer value of the gain parameter
    inline void decode_param_int_(const TPCANMsg &msg, uint32_t &value) const {
        value = msg.DATA[4] | (msg.DATA[5] << 8) | (msg.DATA[6] << 16) | (msg.DATA[7] << 24);
    }

};


/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// INDICAOTR MESSAGE ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////  
////////////////////////////////////// UTILITY FUNCTIONS ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


inline void init_message_(TPCANMsg &msg){
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    msg.LEN = 8;
    std::memset(msg.DATA, 0, 8);
}

// Global utility function to identify the result
inline bool identify_result(const uint8_t error_byte) {
    switch (error_byte){

        case ResultByte::SUCCESS:
            return true;

        case ResultByte::FAILURE:
            std::cout << "FAILURE" << std::endl;
            return false;
        case ResultByte::FAILURE_UNKNOWN_COMMAND:
            std::cout << "FAILURE: UNKNOWN COMMAND" << std::endl;
            return false;
        case ResultByte::FAILURE_UNKNOWN_ID:
            std::cout << "FAILURE: UNKNOWN ID" << std::endl;
            return false;
        case ResultByte::FAILURE_READ_ONLY_REGISTER:
            std::cout << "FAILURE: READ ONLY REGISTER" << std::endl;
            return false;
        case ResultByte::FAILURE_UNKNOWN_REGISTER:
            std::cout << "FAILURE: UNKNOWN REGISTER" << std::endl;
            return false;

        default:
            std::cout << "UNKNOWN ERROR" << std::endl;
            return false;
    }
}


} // namespace can_protocol

#endif // PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_