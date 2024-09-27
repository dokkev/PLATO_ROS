#include <cstring> 
#include <iostream>

#include "plato2_hardware_interface/can_protocol.hpp"

namespace can_protocol{


////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// CONTROL MESSAGE ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// BYTE0   | BYTE1 | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
// COMMAND | CMD0  | CMD1  | CMD2  | CMD3  | DUR0  | DUR1  | DUR2  |

// Where Torque0~Torque3 are value of target torque with byte order of LSB and in unit of N.m. 
// It is represented using IEEE format and can be converted into float value with correct byte order.
// Duration0~Duration2 are 24-bit unsigned integer indicating torque control execution time in unit of ms.

void ControlMessage::start_motor(TPCANMsg &msg) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::START_MOTOR;
}

void ControlMessage::stop_motor(TPCANMsg &msg) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::STOP_MOTOR;
}

void ControlMessage::stop_control(TPCANMsg &msg) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::STOP_CONTROL;
}

void ControlMessage::set_torque(TPCANMsg &msg, const float torque, const uint32_t duration) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::TORQUE_CONTROL;
    encode_command_float_(msg, torque);
    encode_duration_int_(msg, duration);
}

void ControlMessage::set_velocity(TPCANMsg &msg, const float velocity, const uint32_t duration) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::SPEED_CONTROL;
    encode_command_float_(msg, velocity);
    encode_duration_int_(msg, duration);
}

void ControlMessage::set_position(TPCANMsg &msg, const float position, const uint32_t duration) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::POSITION_CONTROL;
    encode_command_float_(msg, position);
    encode_duration_int_(msg, duration);
}

void ControlMessage::acknowledge_fault(TPCANMsg &msg) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::ACKNOWLEDGE_FAULT;
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

bool StateMessage::get_result(const TPCANMsg& msg) const {
    return identify_result(msg.DATA[1]);
}

void StateMessage::get_temperature(const TPCANMsg& msg, int &temperature) const {
    temperature = msg.DATA[2];
}

void StateMessage::get_position(const TPCANMsg& msg, float &position) const {
    uint16_t pos_int = (msg.DATA[4] << 8) | msg.DATA[3];
    position = pos_int * 25.0f / 65536.0f - 12.5f;
}

void StateMessage::get_velocity(const TPCANMsg& msg, float &velocity) const {
    uint16_t velocity_int = (msg.DATA[5] << 4) | ((msg.DATA[6] & 0xF0) >> 4);
    velocity = velocity_int * 130.0f / 4095.0f - 65.0f;
}

void StateMessage::get_torque(const TPCANMsg& msg, float &torque) const {
    uint16_t torque_int = ((msg.DATA[6] & 0x0F) << 8) | msg.DATA[7];
    torque = torque_int * (450.0f * torque_constant_ * gear_ratio_) / 4095.0f - 225.0f * torque_constant_ * gear_ratio_;
}

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// GAIN MESSAGE //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// BYTE0   | BYTE1  | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
// COMMAND | ParaID | NULL  | NULL  | DATA0 | DATA1 | DATA2 | DATA0 |


void GainMessage::set_kp_velocity(TPCANMsg &msg, const uint32_t kp_velocity) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::MODIFY_PARAMETER;
    msg.DATA[1] = ParamID::KP_SPEED;
    encode_param_int_(msg, kp_velocity);
}
void GainMessage::set_ki_velocity(TPCANMsg &msg, const uint32_t ki_velocity) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::MODIFY_PARAMETER;
    msg.DATA[1] = ParamID::KI_SPEED;
    encode_param_int_(msg, ki_velocity);
}

void GainMessage::set_kp_position(TPCANMsg &msg, const uint32_t kp_position) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::MODIFY_PARAMETER;
    msg.DATA[1] = ParamID::KP_POSITION;
    encode_param_int_(msg, kp_position);
}

void GainMessage::set_ki_position(TPCANMsg &msg, const uint32_t ki_position) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::MODIFY_PARAMETER;
    msg.DATA[1] = ParamID::KI_POSITION;
    encode_param_int_(msg, ki_position);
}

void GainMessage::set_kd_position(TPCANMsg &msg, const uint32_t kd_position) {
    init_message_(msg);
    msg.DATA[0] = CommandByte::MODIFY_PARAMETER;
    msg.DATA[1] = ParamID::KD_POSITION;
    encode_param_int_(msg, kd_position);
}

void GainMessage::get_kp_velocity(const TPCANMsg &msg, uint32_t &kp_velocity) {

    decode_param_int_(msg, kp_velocity);
}

void GainMessage::get_ki_velocity(const TPCANMsg &msg, uint32_t &ki_velocity) {

    decode_param_int_(msg, ki_velocity);
}

void GainMessage::get_kp_position(const TPCANMsg &msg, uint32_t &kp_position) {
    
    decode_param_int_(msg, kp_position);
}

void GainMessage::get_ki_position(const TPCANMsg &msg, uint32_t &ki_position) {

    decode_param_int_(msg, ki_position);
}

void GainMessage::get_kd_position(const TPCANMsg &msg, uint32_t &kd_position) {

    decode_param_int_(msg, kd_position);
}

} // namespace can_protocol