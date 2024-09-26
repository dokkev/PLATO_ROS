#include "plato2_hardware_interface/steadywin_can_ids.hpp"
#include "plato2_hardware_interface/steadywin_can_protocol.hpp"

#include <cstring> 
#include <iostream>

namespace can_protocol{


////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// COMMAND MESSAGE ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// BYTE0   | BYTE1 | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
// COMMAND | CMD0  | CMD1  | CMD2  | CMD3  | DUR0  | DUR1  | DUR2  |

// Where Torque0~Torque3 are value of target torque with byte order of LSB and in unit of N.m. 
// It is represented using IEEE format and can be converted into float value with correct byte order.
// Duration0~Duration2 are 24-bit unsigned integer indicating torque control execution time in unit of ms.



void CommandMessage::start_motor(TPCANMsg &msg) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::START_MOTOR;
}

void CommandMessage::stop_motor(TPCANMsg &msg) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::STOP_MOTOR;
}

void CommandMessage::stop_control(TPCANMsg &msg) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::STOP_CONTROL;
}

void CommandMessage::set_torque(const float torque, const uint32_t duration, TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::TORQUE_CONTROL;
    encode_command_float_(torque, msg);
    encode_duration_int_(duration, msg);
}

void CommandMessage::set_velocity(const float velocity, const uint32_t duration, TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::SPEED_CONTROL;
    encode_command_float_(velocity, msg);
    encode_duration_int_(duration, msg);
}

void CommandMessage::set_position(float position, uint32_t duration, TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::POSITION_CONTROL;
    encode_command_float_(position, msg);
    encode_duration_int_(duration, msg);
}

void CommandMessage::encode_command_float_(const float value, TPCANMsg &msg) const {

    // Copy the float value to the buffer using little-endian byte order
    // most CPU architectures are little-endian, but make sure that the byte order is correct
    std::memcpy(&msg.DATA[1], &value, sizeof(float));
}

void CommandMessage::encode_duration_int_(const uint32_t duration, TPCANMsg &msg) const {
    // Copy the 24-bit unsigned integer value to the buffer using little-endian byte order
    msg.DATA[5] = duration & 0xFF;
    msg.DATA[6] = (duration >> 8) & 0xFF;
    msg.DATA[7] = (duration >> 16) & 0xFF;
}

void CommandMessage::init_message_(TPCANMsg &msg) const {
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    msg.LEN = 8;
    std::memset(msg.DATA, 0, 8);
    
}


////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// STATE MESSAGE /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// BYTE0   | BYTE1  | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
// COMMAND | RESULT | TEMP  | POS0  | POS1  |  ST0  |  ST1  |  ST2  |

// Pos0~Pos1 are value of current position with byte order of LSB and in unit of RAD. The actual position value is float:
// pos_float = pos_int * 25 / 65536 - 12.5
//
// ST0~ST2 are the encoding result of current speed and torque. 12-bit speed is formed of ST0 as its higher 8 bits and ST1[7-4] as its lower 4 bits.
// The actual speed value is float in RAD/s:
// speed_float = speed_int * 130 / 4095 – 65
//
// 12-bit torque is formed of ST2 as its lower 8 bits and ST1[3-0] as its higher 4 bits. 
// The actual torque value is float in N.m:
// torque_float = torque_int * (450 * torque_constant * gear_ratio) / 4095 – 225 * torque_constant * gear_ratio




} // namespace can_protocol