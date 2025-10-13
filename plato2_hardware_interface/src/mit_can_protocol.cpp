// can_protocol.cpp — MIT-type CAN protocol mapping with legacy API preserved
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "plato2_hardware_interface/mit_can_protocol.hpp"

namespace mit_can_protocol
{

// ===== Protocol constants (from documentation) =====
static constexpr uint8_t  CMD_CFG_LIMITS     = 0xF0;   // Pos_Max, Vel_Max, T_Max
static constexpr uint8_t  CMD_READ_STATES    = 0xF1;   // position/velocity/torque/status
static constexpr uint8_t  CMD_CLEAR_FAULT    = 0xAF;   // clear fault
static constexpr uint8_t  CMD_EXIT_OC_MODE   = 0xCF;   // exit operation control mode
static constexpr uint8_t  CMD_SET_ZERO       = 0xB1;   // set current position as zero

// Protocol-default maxima are defined as inline constexpr in the header

// StdID bit[10] must be 1 for operation-control command frames (no command byte)
static constexpr uint32_t STDID_OC_BIT   	 = 0x400;

// ===== Optimized scaling functions =====
// Compile-time constants for better performance
static constexpr float INV_0_1 = 10.0f;    // 1/0.1
static constexpr float INV_0_01 = 100.0f;  // 1/0.01
static constexpr float INV_65535 = 1.0f / 65535.0f;
static constexpr float INV_4095 = 1.0f / 4095.0f;
static constexpr uint16_t MAX_12BIT = 4095;
static constexpr uint16_t MAX_16BIT = 65535;

// Limits scaling (0xF0 cmd)
static constexpr uint16_t clamp_u16(float v) {
    return (v <= 0) ? 0 : (v >= MAX_16BIT) ? MAX_16BIT : uint16_t(v + 0.5f);
}

static constexpr uint16_t to_pos_max_u16(float rad) { return clamp_u16(rad * INV_0_1); }
static constexpr uint16_t to_vel_max_u16(float rps) { return clamp_u16(rps * INV_0_01); }
static constexpr uint16_t to_tmax_u16(float nm)     { return clamp_u16(nm * INV_0_01); }

// Signed mapping for states/OC frames  
static constexpr uint16_t map_signed_16(float x, float x_max) {
    float n = (x / (x_max + x_max) + 0.5f) * MAX_16BIT;
    return (n <= 0) ? 0 : (n >= MAX_16BIT) ? MAX_16BIT : uint16_t(n + 0.5f);
}

static constexpr uint16_t map_signed_12(float x, float x_max) {
    float n = (x / (x_max + x_max) + 0.5f) * MAX_12BIT;
    return (n <= 0) ? 0 : (n >= MAX_12BIT) ? MAX_12BIT : uint16_t(n + 0.5f);
}

static constexpr float unmap_signed_16(uint16_t u, float x_max) {
    return (float(u) * INV_65535 - 0.5f) * (x_max + x_max);
}

static constexpr float unmap_signed_12(uint16_t u, float x_max) {
    return (float(u) * INV_4095 - 0.5f) * (x_max + x_max);
}

// ======= MsgEncoder ===============================================================
MsgEncoder::MsgEncoder(const float &gear_ratio, const float &torque_constant, const uint8_t &tx_id)
: gear_ratio_(gear_ratio), torque_constant_(torque_constant), tx_id_(tx_id) {}

// void MsgEncoder::set_limits(TPCANMsg& msg,
//                             float pos_max_rad,
//                             float vel_max_rps,
//                             float tq_max_nm, bool set_pos, bool set_vel, bool set_tq)
// {
//     // Build a protocol-correct 0xF0 (CFG_LIMITS) frame with DLC=7.
//     // Zero the whole struct to ensure deterministic contents.
//     std::memset(&msg, 0, sizeof(msg));
//     msg.ID = tx_id_;
//     msg.LEN = 7;
//     msg.DATA[0] = CMD_CFG_LIMITS; // 0xF0

//     // Convert to protocol units using documented LSBs (helpers above)
//     uint16_t pos_u16 = set_pos ? to_pos_max_u16(pos_max_rad) : to_pos_max_u16(POS_MAX);
//     uint16_t vel_u16 = set_vel ? to_vel_max_u16(vel_max_rps) : to_vel_max_u16(VEL_MAX);
void MsgEncoder::set_limits(TPCANMsg& msg, float pos_max_rad, float vel_max_rps, float tq_max_nm, 
                           bool set_pos, bool set_vel, bool set_tq)
{
    msg.ID      = tx_id_;
    msg.LEN     = 7;
    msg.DATA[0] = CMD_CFG_LIMITS;
    
    // Convert to protocol units and pack little-endian
    uint16_t pos_u16 = to_pos_max_u16(pos_max_rad);  // 0.1 rad/LSB
    uint16_t vel_u16 = to_vel_max_u16(vel_max_rps);  // 0.01 rad/s/LSB 
    uint16_t tq_u16  = to_tmax_u16(tq_max_nm);       // 0.01 Nm/LSB
    
    // Pack little-endian (LSB first)
    msg.DATA[1] = uint8_t(pos_u16 & 0xFF);
    msg.DATA[2] = uint8_t(pos_u16 >> 8);
    msg.DATA[3] = uint8_t(vel_u16 & 0xFF);
    msg.DATA[4] = uint8_t(vel_u16 >> 8);
    msg.DATA[5] = uint8_t(tq_u16  & 0xFF);
    msg.DATA[6] = uint8_t(tq_u16  >> 8);
}

void MsgEncoder::set_zero_position(TPCANMsg &msg)
{
    // Documentation: send 0xB1 to set current position as origin.
    // Normal command frames use base StdID (no STDID_OC_BIT)
    msg.ID      = tx_id_;
    std::cout << "Setting zero position with TX ID: " << std::hex << int(tx_id_) << std::dec << std::endl;
    msg.LEN     = 1;
    msg.DATA[0] = CMD_SET_ZERO;
}

static inline void pack_oc_frame(TPCANMsg& msg,
                                 float pos_rad, bool pos_set,
                                 float vel_rps, bool vel_set,
                                 float kp,      bool kp_set,
                                 float kd,      bool kd_set,
                                 float tq_nm,   bool tq_set,
                                 float pos_max, float vel_max, float t_max, uint8_t tx_id)
{
    std::memset(&msg, 0, sizeof(msg));
    msg.ID  = (tx_id | STDID_OC_BIT);
    msg.LEN = 8;

    // Saturate to configured maxima
    if (pos_set){ if (pos_rad >  pos_max) pos_rad =  pos_max; if (pos_rad < -pos_max) pos_rad = -pos_max; }
    if (vel_set){ if (vel_rps >  vel_max) vel_rps =  vel_max; if (vel_rps < -vel_max) vel_rps = -vel_max; }
    if (tq_set) { if (tq_nm  >   t_max)  tq_nm  =   t_max;   if (tq_nm  <  -t_max)  tq_nm  =  -t_max;   }

    // Clamp and convert values efficiently
    const float pos_val = pos_set ? std::clamp(pos_rad, -pos_max, pos_max) : 0.0f;
    const float vel_val = vel_set ? std::clamp(vel_rps, -vel_max, vel_max) : 0.0f;
    const float tq_val = tq_set ? std::clamp(tq_nm, -t_max, t_max) : 0.0f;
    const float kp_val = kp_set ? std::clamp(kp, 0.0f, KP_MAX) : 0.0f;
    const float kd_val = kd_set ? std::clamp(kd, 0.0f, KD_MAX) : 0.0f;

    const uint16_t p16 = map_signed_16(pos_val, pos_max);
    const uint16_t v12 = map_signed_12(vel_val, vel_max);
    const uint16_t t12 = map_signed_12(tq_val, t_max);
    const uint16_t kp12 = uint16_t(kp_val * (MAX_12BIT / KP_MAX) + 0.5f);
    const uint16_t kd12 = uint16_t(kd_val * (MAX_12BIT / KD_MAX) + 0.5f);

    // Bytes:
    // pos
    msg.DATA[0] = uint8_t(p16 >> 8);
    msg.DATA[1] = uint8_t(p16 & 0xFF);

    // vel
    msg.DATA[2] = uint8_t(v12 >> 4);               // hi8
    msg.DATA[3] = uint8_t((v12 & 0x0F) << 4);      // lo4 in [7:4]

    // kp
    msg.DATA[3] |= uint8_t((kp12 >> 8) & 0x0F);    // hi4 in [3:0]
    msg.DATA[4]  = uint8_t(kp12 & 0xFF);           // lo8

    // kd
    msg.DATA[5]  = uint8_t(kd12 >> 4);             // hi8
    msg.DATA[6]  = uint8_t((kd12 & 0x0F) << 4);    // lo4 in [7:4]

    // torque
    msg.DATA[6] |= uint8_t((t12 >> 8) & 0x0F);     // hi4 in [3:0]
    msg.DATA[7]  = uint8_t(t12 & 0xFF);            // lo8

    // std::cout << "OC Frame Data: " << std::hex << int(msg.ID) << " ";
    // for (int i = 0; i < 8; ++i) {
    //     std::cout << std::hex << int(msg.DATA[i]) << " ";
    // }
    // std::cout << std::dec << std::endl;
}

void MsgEncoder::start_motor(TPCANMsg &msg)
{
    // Enter operation-control mode by issuing an OC frame (bit10=1) with zeros.
    // std::memset(&msg, 0, sizeof(msg));
    std::cout << "Starting motor with TX ID: " << std::hex << int(tx_id_) << std::dec << std::endl;
    pack_oc_frame(msg,
				  /*pos*/0.0f, true,
				  /*vel*/0.0f, true,
				  /*kp*/0.0f, true,
				  /*kd*/0.0f, true,
				  /*tq*/0.0f, true,
				  POS_MAX, VEL_MAX, T_MAX, tx_id_);

    
}

void MsgEncoder::stop_motor(TPCANMsg &msg)
{
    // Same as stop_control in this protocol (exit OC mode).
    stop_control(msg);
}

void MsgEncoder::stop_control(TPCANMsg &msg)
{
    // Exit operation-control mode: 0xCF
    // Normal command frames use base StdID (no STDID_OC_BIT)
    std::cout << "Stopping control with TX ID: " << std::hex << int(tx_id_) << std::dec << std::endl;
    msg.ID      = tx_id_;
    msg.LEN     = 1;
    msg.DATA[0] = CMD_EXIT_OC_MODE;
}

void MsgEncoder::clear_fault(TPCANMsg &msg)
{
    // Clear fault: 0xAF
    // Normal command frames use base StdID (no STDID_OC_BIT)
    msg.ID      = tx_id_;
    msg.LEN     = 1;
    msg.DATA[0] = CMD_CLEAR_FAULT;
}



void MsgEncoder::set_impedance(TPCANMsg &msg, const float position_rad, 
											  const float velocity_rps,
											  const float kp, 
											  const float kd, 
											  const float torque_nm)
{
    const float motor_position = position_rad;
    const float motor_velocity = velocity_rps;
    const float motor_torque = torque_nm / 8.0f;
    
    // std::cout << "Sending impedance to TX ID: " << std::hex << int(tx_id_) << std::dec << "  joint_pos=" << position_rad << " -> motor_pos=" << motor_position << " rad, joint_vel=" << velocity_rps << " -> motor_vel=" << motor_velocity << " rad/s, joint_tq=" << torque_nm << " -> motor_tq=" << motor_torque << " Nm\n";
	pack_oc_frame(msg,
				  /*pos*/motor_position, true,
				  /*vel*/motor_velocity, true,
				  /*kp*/kp, true,           // Pass actual kp parameter
				  /*kd*/kd, true,           // Pass actual kd parameter
				  /*tq*/motor_torque, true,
				  POS_MAX, VEL_MAX, T_MAX, tx_id_);
}



// ======= MsgDecoder ===============================================================
MsgDecoder::MsgDecoder(const float &gear_ratio, const float &torque_constant)
: gear_ratio_(gear_ratio), torque_constant_(torque_constant) {}

// Parse MIT CAN responses:
// 8-byte: Operation-Control (OC) response frame (same format as OC command, no cmd byte)
// 7-byte: 0xF1 state query response
void MsgDecoder::get_states(const TPCANMsg &msg, float &position, float &velocity, float &kp, float &kd, float &torque, bool &in_oc_mode, bool &has_fault) const
{
    uint16_t p16, v12, t12, kp12 = 0, kd12 = 0;
    
    if (msg.LEN == 8) {
        // 8-byte Operation-Control response frame (no command byte)
        // Same format as OC command frame sent to motor
        p16 = (uint16_t(msg.DATA[0]) << 8) | uint16_t(msg.DATA[1]);
        v12 = (uint16_t(msg.DATA[2]) << 4) | ((msg.DATA[3] & 0xF0) >> 4);
        kp12 = ((msg.DATA[3] & 0x0F) << 8) | msg.DATA[4];
        kd12 = (uint16_t(msg.DATA[5]) << 4) | ((msg.DATA[6] & 0xF0) >> 4);
        t12 = ((msg.DATA[6] & 0x0F) << 8) | msg.DATA[7];
        
        // OC mode is active since we received an OC response
        in_oc_mode = true;
        has_fault = false;  // No fault status in OC response
        
    } else if (msg.LEN >= 7 && msg.DATA[0] == CMD_READ_STATES) {
        // 7-byte 0xF1 state query response
        p16 = uint16_t(msg.DATA[1]) << 8 | uint16_t(msg.DATA[2]);
        v12 = (uint16_t(msg.DATA[3]) << 4) | ((msg.DATA[4] & 0xF0) >> 4);
        t12 = ((msg.DATA[4] & 0x0F) << 8) | msg.DATA[5];
        
        // Status bits from 0xF1 response
        in_oc_mode = (msg.DATA[6] & 0x01) != 0;
        has_fault  = (msg.DATA[6] & 0x02) != 0;
        
    } else {
        // Unexpected frame format
        std::cout << "MsgDecoder::get_states: unexpected frame RX ID: " << std::hex << int(msg.ID) << std::dec 
                  << " LEN: " << int(msg.LEN) << " DATA[0]: " << std::hex << int(msg.DATA[0]) << std::dec << std::endl;
        position = velocity = kp = kd = torque = 0.0f;
        in_oc_mode = has_fault = false;
        return;
    }
    
    // Unmap motor values and convert to joint-space
    // Motor -> Joint transformations (multiply by gear ratio for position/velocity, gear_ratio² for torque)
    const float motor_position = unmap_signed_16(p16, POS_MAX);
    const float motor_velocity = unmap_signed_12(v12, VEL_MAX);
    const float motor_torque = unmap_signed_12(t12, T_MAX);
    
    // Apply gear ratio conversions to get joint-space values
    position = motor_position;
    velocity = motor_velocity;
    torque   = motor_torque;

    // Decode Kp/Kd if available (only from 8-byte OC responses)
    if (msg.LEN == 8) {
        kp = float(kp12) * (KP_MAX / MAX_12BIT);
        kd = float(kd12) * (KD_MAX / MAX_12BIT);
    } else {
        kp = 0.0f;
        kd = 0.0f;
    }

    if (has_fault)
        std::cout << "Motor fault detected! RX ID: " << std::hex << int(msg.ID) << std::dec << std::endl;
    // std::cout << "Receiving states from RX ID: " << std::hex << int(msg.ID) << std::dec << "  pos=" << position << " rad, vel=" << velocity << " rad/s, tq=" << torque << " Nm, oc=" << in_oc_mode << ", fault=" << has_fault << "\n";
}

void MsgDecoder::get_limits(const TPCANMsg &msg, float &pos_max_rad, float &vel_max_rps, float &tq_max_nm) const
{
    if (msg.LEN < 7 || msg.DATA[0] != CMD_CFG_LIMITS)
    {
        std::cerr << "MsgDecoder::get_limits: unexpected frame\n";
        pos_max_rad = vel_max_rps = tq_max_nm = 0.0f;
        return;
    }

    // Unpack little-endian (LSB first)
    uint16_t pos_u16 = uint16_t(msg.DATA[1]) | (uint16_t(msg.DATA[2]) << 8);
    uint16_t vel_u16 = uint16_t(msg.DATA[3]) | (uint16_t(msg.DATA[4]) << 8);
    uint16_t tq_u16  = uint16_t(msg.DATA[5]) | (uint16_t(msg.DATA[6]) << 8);

    pos_max_rad = pos_u16 * 0.1f;   // Direct conversion
    vel_max_rps = vel_u16 * 0.01f;  // Direct conversion
    tq_max_nm   = tq_u16 * 0.01f;   // Direct conversion

}
} // namespace can_protocol
