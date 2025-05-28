#include <cstring> 
#include <iostream>

#include "plato2_hardware_interface/can_protocol.hpp"

namespace can_protocol{


////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// MsgEncoder ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

MsgEncoder::MsgEncoder(const float &gear_ratio, const float &torque_constant) 
        : gear_ratio_(gear_ratio), torque_constant_(torque_constant) {} 

////////////////////////////////////////////////////////////////////////////////////////
void MsgEncoder::set_zero_position(TPCANMsg &msg, const float zero_position) {
    msg.DATA[0] = CommandByte::MODIFY_CONFIGURATION;
    msg.DATA[1] = ConfigType::INT32;
    msg.DATA[2] = ConfigByte::ZERO_POSITION;
    int32_t pos_int = zero_position / (2*M_PI) * 65536;
    msg.DATA[4] = pos_int & 0xFF;
    msg.DATA[5] = (pos_int >> 8) & 0xFF;
    msg.DATA[6] = (pos_int >> 16) & 0xFF;
    msg.DATA[7] = (pos_int >> 24) & 0xFF;
}

void MsgEncoder::set_default_gain(TPCANMsg &msg, const uint32_t &gain_val, const uint8_t conf_id) {
    msg.DATA[0] = CommandByte::MODIFY_CONFIGURATION;
    msg.DATA[1] = ConfigType::INT32;
    msg.DATA[2] = conf_id;
    encode_param_int_(msg, gain_val);
}


void MsgEncoder::calibrate_encoder(TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::CALIBRATE;
    msg.DATA[1] = CalibrationID::ENCODER;
}

void MsgEncoder::calibrate_phase_order(TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::CALIBRATE;
    msg.DATA[1] = CalibrationID::PHASE_ORDER;
}

////////////////////////////////////////////////////////////////////////////////////////

void MsgEncoder::start_motor(TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::START_MOTOR;
}

////////////////////////////////////////////////////////////////////////////////////////

void MsgEncoder::stop_motor(TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::STOP_MOTOR;
}

////////////////////////////////////////////////////////////////////////////////////////

void MsgEncoder::stop_control(TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::STOP_CONTROL;
}

////////////////////////////////////////////////////////////////////////////////////////

void MsgEncoder::set_torque(TPCANMsg &msg, const float torque, const uint32_t duration) {
    msg.DATA[0] = CommandByte::TORQUE_CONTROL;
    encode_command_float_(msg, torque);
    encode_duration_int_(msg, duration);
}

////////////////////////////////////////////////////////////////////////////////////////

void MsgEncoder::set_velocity(TPCANMsg &msg, const float velocity, const uint32_t duration) {
    msg.DATA[0] = CommandByte::SPEED_CONTROL;
    encode_command_float_(msg, velocity);
    encode_duration_int_(msg, duration);
}

////////////////////////////////////////////////////////////////////////////////////////

void MsgEncoder::set_position(TPCANMsg &msg, const float position, const uint32_t duration) {
    msg.DATA[0] = CommandByte::POSITION_CONTROL;
    encode_command_float_(msg, position);
    encode_duration_int_(msg, duration);
}

////////////////////////////////////////////////////////////////////////////////////////

void MsgEncoder::acknowledge_fault(TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::ACKNOWLEDGE_FAULT;
}

////////////////////////////////////////////////////////////////////////////////////////

// BYTE0   | BYTE1  | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
// COMMAND | ParaID | NULL  | NULL  | DATA0 | DATA1 | DATA2 | DATA3 |
void MsgEncoder::set_gain(TPCANMsg &msg, const uint32_t &gain_val, const uint8_t param_id) {
    msg.DATA[0] = CommandByte::MODIFY_PARAMETER;
    msg.DATA[1] = param_id;
    encode_param_int_(msg, gain_val);
}

////////////////////////////////////////////////////////////////////////////////////////
// BYTE0   | BYTE1  | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
// COMMAND | IndID  | NULL  | NULL  | NULL  | NULL  | NULL  | NULL  |
void MsgEncoder::retrieve_position(TPCANMsg &msg) {
    msg.DATA[0] = CommandByte::RETRIVE_INDICATOR;
    msg.DATA[1] = IndicatorID::SHAFT_ANGLE;
}

////////////////////////////////////////////////////////////////////////////////////////
void MsgEncoder::retrieve_gain(TPCANMsg &msg, const uint8_t param_id) {
    msg.DATA[0] = CommandByte::RETRIVE_PARAMETER;
    msg.DATA[1] = param_id;
}


////////////////////////////////////////////////////////////////////////////////////////

MsgDecoder::MsgDecoder(const float &gear_ratio, const float &torque_constant) 
        : gear_ratio_(gear_ratio), torque_constant_(torque_constant) {} 

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
void MsgDecoder::get_states(const TPCANMsg &msg, uint8_t &temperature, float &position, float &velocity, float &torque) const {
    
    if (get_result(msg.DATA[1]) == false) {
        std::cerr << "Error in MsgDecoder::get_states" << std::endl;
        return;
    }
    // decode the temperature value
    temperature = msg.DATA[2];
    // decode the position value
    uint16_t pos_int = (msg.DATA[4] << 8) | msg.DATA[3];
    position = pos_int * 25.0f / 65535.0f - 12.5f;
    // decode the velocity value
    uint16_t velocity_int = (msg.DATA[5] << 4) | ((msg.DATA[6] & 0xF0) >> 4);
    velocity = velocity_int * 130.0f / 4095.0f - 65.0f;
    // decode the torque value
    uint16_t torque_int = ((msg.DATA[6] & 0x0F) << 8) | msg.DATA[7];
    torque = torque_int * (450.0f * torque_constant_ * gear_ratio_) / 4095.0f - 225.0f * torque_constant_ * gear_ratio_;
}

////////////////////////////////////////////////////////////////////////////////////////

// BYTE0   | BYTE1  | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
// COMMAND | ParaID | NULL  | NULL  | DATA0 | DATA1 | DATA2 | DATA3 |
void MsgDecoder::get_gain(const TPCANMsg &msg, uint32_t &gain_val) const {

    if (get_result(msg.DATA[2]) == false) {
        return;
    }
    decode_param_int_(msg, gain_val);
}

////////////////////////////////////////////////////////////////////////////////////////

// BYTE0   | BYTE1  | BYTE2 | BYTE3 | BYTE4 | BYTE5 | BYTE6 | BYTE7 |
// COMMAND | IndID  | RES   | NULL  | DATA0 | DATA1 | DATA2 | DATA3 |
void MsgDecoder::retrieve_position(const TPCANMsg &msg, float &position) const {
 if (get_result(msg.DATA[2]) == false) {
        return;
    }
   decode_ind_float_(msg, position);
}


////////////////////////////////////////////////////////////////////////////////////////

// BYTE0   | BYTE1   |   BYTE2 |   BYTE3 |   BYTE4 |   BYTE5 | BYTE6 | BYTE7 |
// FORCE[0]| FORCE[1]| FORCE[2]| FORCE[3]| FORCE[4]| FORCE[5]|   -   |   -   |
void MsgDecoder::retrieve_force(const TPCANMsg &msg,
                                float &fx, float &fy, float &fz)
{
    constexpr float F_SCALE = 0.001f;   // = 1/1000
    constexpr float F_BIAS  = -30.0f;
    const uint8_t* data = msg.DATA;

    // high-byte * 256 + low-byte
    fx = (uint16_t(data[0]) * 256u + uint16_t(data[1])) * F_SCALE + F_BIAS;
    fy = (uint16_t(data[2]) * 256u + uint16_t(data[3])) * F_SCALE + F_BIAS;
    fz = (uint16_t(data[4]) * 256u + uint16_t(data[5])) * F_SCALE + F_BIAS;
}

void MsgDecoder::retrieve_torque(const TPCANMsg &msg,
                                 float &tx, float &ty, float &tz)
{
    constexpr float T_SCALE = 1e-5f;   // = 1/100000
    constexpr float T_BIAS  = -0.3f;
    const uint8_t* data = msg.DATA;

    tx = (uint16_t(data[0]) * 256u + uint16_t(data[1])) * T_SCALE + T_BIAS;
    ty = (uint16_t(data[2]) * 256u + uint16_t(data[3])) * T_SCALE + T_BIAS;
    tz = (uint16_t(data[4]) * 256u + uint16_t(data[5])) * T_SCALE + T_BIAS;
}



} // namespace can_protocol