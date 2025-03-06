#ifndef PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_
#define PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <cmath> 
#include <iostream>

#include "PCANBasic.h"

#include "plato2_hardware_interface/hardware_config/can_ids.hpp"

namespace can_protocol{


/// @brief Runtime motor motion control command message. ControlMessage takes TPCANMsg message and encode the control command following the Steadywin
/// CAN Protocol for PCANInterface to send to the motor driver later
class MsgEncoder{

public:
    /// @brief Default constructor
    MsgEncoder(const float &gear_ratio, const float &torque_constant);


    /////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// CONFIG MESSAGE ///////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // BYTE0   | BYTE1    | BYTE2  | BYTE3 | BYTE4  | BYTE5 | BYTE6 | BYTE7 |
    // COMMAND | ConfType | ConfID | NULL  | DATA0  | DATA1 | DATA2 | DATA3 |

    /// @brief Set the zero position ID to the message
    void set_zero_position(TPCANMsg &msg, const float zero_position);

    /// @brief Set the default gain ID to the message
    void set_default_gain(TPCANMsg &msg, const uint32_t &gain_val, const uint8_t param_id);

    void calibrate_encoder(TPCANMsg &msg);

    void calibrate_phase_order(TPCANMsg &msg);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// COMMAND MESSAGE //////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the motor enable ID to the message
    static void start_motor(TPCANMsg &msg);

    /// @brief Set the motor disable ID to the message
    static void stop_motor(TPCANMsg &msg);

    /// @brief Set the stop control ID to the message. Current ongoing control command should be stopped immediately
    static void stop_control(TPCANMsg &msg);

    /// @brief Set the torque command ID, torque value and duration to the message
    /// @param torque desired torque value
    /// @param duration execution time in ms
    /// @param msg TPCANMsg reference to store the command message
    static void set_torque(TPCANMsg &msg, const float torque, const uint32_t duration);

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

    /////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// GAIN MESSAGE /////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the gain parameter ID and value to the message
    /// @param gain_val desired gain value
    void set_gain(TPCANMsg &msg, const uint32_t &gain_val, const uint8_t param_id);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////// INDICATOR MESSAGE //////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Send command message to retrieve the output shaft position of the motor
    /// @param msg TPCANMsg reference to store the command message
    void retrieve_position(TPCANMsg &msg);

    
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////// PARAM MESSAGE //////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Send command message to retrieve the gains of the motor
    /// @param msg 
    void retrieve_gain(TPCANMsg &msg, const uint8_t param_id);

private:
    /// @brief gear ratio of the motor initialized in the actuator constructor
    const float &gear_ratio_;

    /// @brief torque constant of the motor initialized in the actuator constructor
    const float &torque_constant_;


    // BYTE0   | BYTE1 | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
    // COMMAND | CMD0  | CMD1  | CMD2  | CMD3  | DUR0  | DUR1  | DUR2  |
    // Where Torque0~Torque3 are value of target torque with byte order of LSB and in unit of N.m. 
    // It is represented using IEEE format and can be converted into float value with correct byte order.
    // Duration0~Duration2 are 24-bit unsigned integer indicating torque control execution time in unit of ms.

    /// @brief encode target command value with LSB byte order
    /// @param value target command value
    /// @param msg buffer to store the encoded value
    /// @details BYTE0   | BYTE1 | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
    static inline void encode_command_float_(TPCANMsg &msg, const float value){
        // Copy the float value to the buffer using little-endian byte order
        // most CPU architectures are little-endian, but make sure that the byte order is correct
        std::memcpy(&msg.DATA[1], &value, sizeof(float));
    }

    /// @brief encode 24-bit unsigned integer indicating control execution time in unit of ms.
    /// @param duration control execution time in unit of ms
    /// @param msg buffer to store the encoded value
    static inline void encode_duration_int_( TPCANMsg &msg, const uint32_t duration){
        // Copy the 24-bit unsigned integer value to the buffer using little-endian byte order
        msg.DATA[5] = duration & 0xFF;
        msg.DATA[6] = (duration >> 8) & 0xFF;
        msg.DATA[7] = (duration >> 16) & 0xFF;
    }

    /// @brief encode 32-bit unsigned integer value to the message buffer's BYTE4 to BYTE7 for the gain parameter
    /// @param msg reference  TPCANMsg to store the encoded value
    /// @param value desired 32-bit unsigned integer value of the gain parameter
    static inline void encode_param_int_(TPCANMsg &msg, const uint32_t value) {
        // Copy the 32-bit unsigned integer value to the buffer using little-endian byte order
        msg.DATA[4] = value & 0xFF;
        msg.DATA[5] = (value >> 8) & 0xFF;
        msg.DATA[6] = (value >> 16) & 0xFF;
        msg.DATA[7] = (value >> 24) & 0xFF;
    }
};

/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// STATE MESSAGE ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


class MsgDecoder{
public:
    /// @brief constructor
    MsgDecoder(const float &gear_ratio, const float &torque_constant);

    /// @brief get the result of the response message from the command message if successful return true otherwise false
    /// @param msg 
    /// @param error_byte
    /// @return  true if the response message is successful
    static inline bool get_result(const uint8_t error_byte){
        switch (error_byte){

            case ResultByte::SUCCESS:
                return true;

            case ResultByte::FAILURE:
                std::cerr << "FAILURE" << std::endl;
                return false;
            case ResultByte::FAILURE_UNKNOWN_COMMAND:
                std::cerr << "FAILURE: UNKNOWN COMMAND" << std::endl;
                return false;
            case ResultByte::FAILURE_UNKNOWN_ID:
                std::cerr << "FAILURE: UNKNOWN ID" << std::endl;
                return false;
            case ResultByte::FAILURE_READ_ONLY_REGISTER:
                std::cerr << "FAILURE: READ ONLY REGISTER" << std::endl;
                return false;
            case ResultByte::FAILURE_UNKNOWN_REGISTER:
                std::cerr << "FAILURE: UNKNOWN REGISTER" << std::endl;
                return false;
            default:
                std::cerr << "UNKNOWN ERROR" << std::endl;
                return false;
            }
}


    /// @brief get the state of the motor from the received message
    /// @param msg received TPACNMsg message
    /// @param temperature reference to store the decoded temperature value
    /// @param position reference to store the decoded position value
    /// @param velocity reference to store the decoded velocity value
    /// @param torque reference to store the decoded torque value
    void get_states(const TPCANMsg &msg, uint8_t &temperature, float &position, float &velocity, float &torque) const;

    /// @brief get the gain value from the received message
    void get_gain(const TPCANMsg &msg, uint32_t &gain_val) const;

    /// @brief decode the float value from the message buffer's BYTE4 to BYTE7 for Indicator
    /// @param msg 
    /// @param position 
    void retrieve_position(const TPCANMsg &msg, float &position) const;

    /// @brief decode the 3D force value from the Aidin FT sensor
    /// @param msg received TPCANMsg message 
    /// @param force_x reference to store the decoded force_x value
    /// @param force_y reference to store the decoded force_y value 
    /// @param force_z  reference to store the decoded force_z value
    static void retrieve_force(const TPCANMsg &msg, float &force_x, float &force_y, float &force_z);

    /// @brief decode the 3D torque value from the Aidin FT sensor
    /// @param msg received TPCANMsg message 
    /// @param torque_x reference to store the decoded torque_x value 
    /// @param torque_y reference to store the decoded torque_y value 
    /// @param torque_z reference to store the decoded torque_z value 
    static void retrieve_torque(const TPCANMsg &msg, float &torque_x, float &torque_y, float &torque_z);






private:
    /// @brief gear ratio of the motor
    const float &gear_ratio_ ;

    /// @brief torque constant of the motor
    const float &torque_constant_; 

    /// @brief decode 32-bit unsigned integer value from the message buffer's BYTE4 to BYTE7
    /// @param msg recevied TPCANMsg message
    /// @param value reference to store the decoded 32-bit unsigned integer value of the gain parameter
    inline void decode_param_int_(const TPCANMsg &msg, uint32_t &value) const {
        value = msg.DATA[4] | (msg.DATA[5] << 8) | (msg.DATA[6] << 16) | (msg.DATA[7] << 24);
    }

    /// @brief decode the float value from the message buffer's BYTE4 to BYTE7 for Indicator 
    /// @param msg 
    /// @param value 
    inline void decode_ind_float_(const TPCANMsg &msg, float &value) const {
        std::memcpy(&value, &msg.DATA[4], sizeof(float));
    }
};


/////////////////////////////////////////////////////////////////////////////////////////////////////  
////////////////////////////////////// UTILITY FUNCTIONS ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////




} // namespace can_protocol

#endif // PLATO_HARDWARE_INTERFACE__CAN_PROTOCOL_HPP_