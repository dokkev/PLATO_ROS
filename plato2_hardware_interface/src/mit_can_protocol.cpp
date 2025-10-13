// can_protocol.cpp — MIT-type CAN protocol mapping with legacy API preserved
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

static float 			  KP_MAX			 = 5.0F;   // default max KP (rad)
static float 			  KD_MAX			 = 0.1F;     // default max KD (rad/s) corresponds to 0 05
static float 			  POS_MAX			 = 95.50f;   // rad (doc default) // 4pi rads
static float 			  VEL_MAX			 = 45.00f;   // rad/s (doc default) // 42 rad/s
static float 			  T_MAX  			 = 18.00f;   // Nm (doc default) // .52*3 Nm

// StdID bit[10] must be 1 for operation-control command frames (no command byte)
static constexpr uint32_t STDID_OC_BIT   	 = 0x400;

// ===== Scaling per documentation =====
// Config limits (CMD 0xF0): Pos_Max=0.1 rad LSB; Vel_Max=0.01 rad/s LSB; T_Max=0.01 Nm LSB
static inline uint16_t to_pos_max_u16(float rad)   { float v = rad / 0.1f;   if (v < 0) v = 0; if (v > 65535) v = 65535; return uint16_t(lroundf(v)); }
static inline uint16_t to_vel_max_u16(float rps)   { float v = rps / 0.01f;  if (v < 0) v = 0; if (v > 65535) v = 65535; return uint16_t(lroundf(v)); }
static inline uint16_t to_tmax_u16   (float nm)    { float v = nm  / 0.01f;  if (v < 0) v = 0; if (v > 65535) v = 65535; return uint16_t(lroundf(v)); }

// States (0xF1) and OC frame mapping use normalized 16/12-bit fields spanning [-Max, +Max]
static inline uint16_t map_signed_16(float x, float x_max)
{
    // maps [-x_max, +x_max] -> [0..65535]
    float n = (x / (2.0f * x_max) + 0.5f) * 65535.0f;
    if (n < 0) n = 0; 
    if (n > 65535) n = 65535;
    return uint16_t(lroundf(n));
}
static inline uint16_t map_signed_12(float x, float x_max)
{
    // maps [-x_max, +x_max] -> [0..4095]
    float n = (x / (2.0f * x_max) + 0.5f) * 4095.0f;
    if (n < 0) n = 0; 
    if (n > 4095) n = 4095;
    return uint16_t(lroundf(n));
}
static inline float unmap_signed_16(uint16_t u, float x_max)
{
    // [0..65535] -> [-x_max, +x_max]
    return ( (float(u) / 65535.0f) - 0.5f ) * (2.0f * x_max);
}
static inline float unmap_signed_12(uint16_t u, float x_max)
{
    // [0..4095] -> [-x_max, +x_max]
    return ( (float(u) / 4095.0f) - 0.5f ) * (2.0f * x_max);
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
//     uint16_t tq_u16  = set_tq  ? to_tmax_u16(tq_max_nm)     : to_tmax_u16(T_MAX);

//     // Pack big-endian hi/lo for each 16-bit field as documented
//     msg.DATA[1] = static_cast<uint8_t>(pos_u16 >> 8);
//     msg.DATA[2] = static_cast<uint8_t>(pos_u16 & 0xFF);

//     msg.DATA[3] = static_cast<uint8_t>(vel_u16 >> 8);
//     msg.DATA[4] = static_cast<uint8_t>(vel_u16 & 0xFF);

//     msg.DATA[5] = static_cast<uint8_t>(tq_u16 >> 8);
//     msg.DATA[6] = static_cast<uint8_t>(tq_u16 & 0xFF);
// }

void MsgEncoder::set_limits(TPCANMsg& msg,
                            float pos_max_rad,
                            float vel_max_rps,
                            float tq_max_nm, bool set_pos, bool set_vel, bool set_tq)
{
    msg.ID      = tx_id_;
    msg.LEN     = 7;
    msg.DATA[0] = CMD_CFG_LIMITS;
    msg.DATA[1] = 0x03;
    msg.DATA[2] = 0xbb;
    msg.DATA[3] = 0x0f;
    msg.DATA[4] = 0xa0;
    msg.DATA[5] = 0x07;
    msg.DATA[6] = 0x08;
    
}

void MsgEncoder::set_zero_position(TPCANMsg &msg)
{
    // Documentation: send 0xB1 to set current position as origin.
    msg.ID      = (tx_id_ | STDID_OC_BIT);
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

    uint16_t p16 = map_signed_16(pos_set ? pos_rad : 0.0f, pos_max);
    uint16_t v12 = map_signed_12(vel_set ? vel_rps : 0.0f, vel_max);
    uint16_t t12 = map_signed_12(tq_set  ? tq_nm  : 0.0f, t_max);

    // Kp 0..500 -> 12-bit
    if (!kp_set) kp = 0.0f;
    if (kp < 0) kp = 0; 
    if (kp > KP_MAX) kp = KP_MAX;
    uint16_t kp12 = uint16_t(lroundf(kp / KP_MAX * 4095.0f));

    // Kd 0..5 -> 12-bit
    if (!kd_set) kd = 0.0f;
    if (kd < 0) kd = 0; 
    if (kd > KD_MAX) kd = KD_MAX;
    uint16_t kd12 = uint16_t(lroundf(kd / KD_MAX * 4095.0f));

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

    std::cout << "OC Frame Data: " << std::hex << int(msg.ID) << " ";
    for (int i = 0; i < 8; ++i) {
        std::cout << std::hex << int(msg.DATA[i]) << " ";
    }
    std::cout << std::dec << std::endl;
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
    std::cout << "Stopping control with TX ID: " << std::hex << int(tx_id_) << std::dec << std::endl;
    msg.ID      = (tx_id_ | STDID_OC_BIT);
    msg.LEN     = 1;
    msg.DATA[0] = CMD_EXIT_OC_MODE;
}

void MsgEncoder::clear_fault(TPCANMsg &msg)
{
    // Clear fault: 0xAF
    msg.ID      = (tx_id_ | STDID_OC_BIT);
    msg.LEN     = 1;
    msg.DATA[0] = CMD_CLEAR_FAULT;
}



void MsgEncoder::set_impedance(TPCANMsg &msg, const float position_rad, 
											  const float velocity_rps,
											  const float kp, 
											  const float kd, 
											  const float torque_nm)
{
    std::cout << "Sending impedance to TX ID: " << std::hex << int(tx_id_) << std::dec << "  pos=" << position_rad << " rad, vel=" << velocity_rps << " rad/s, kp=" << kp << ", kd=" << kd << ", tq=" << torque_nm << " Nm\n";
	pack_oc_frame(msg,
				  /*pos*/position_rad, true,
				  /*vel*/velocity_rps, true,
				  /*kp*/0, true,
				  /*kd*/0, true,
				  /*tq*/torque_nm, true,
				  POS_MAX, VEL_MAX, T_MAX, tx_id_);
}



// ======= MsgDecoder ===============================================================
MsgDecoder::MsgDecoder(const float &gear_ratio, const float &torque_constant)
: gear_ratio_(gear_ratio), torque_constant_(torque_constant) {}

// Parse 0xF1 response:
// [0]=0xF1
// [1..2]: mech pos 16b (hi,lo) -> (-Pos_Max..+Pos_Max)
// [3..4]: mech vel 12b ([3]=hi8, [4][7:4]=lo4)
// [4..5]: torque 12b ([4][3:0]=hi4, [5]=lo8)
// [6]: status bits (bit0: in OC mode, bit1: fault)
void MsgDecoder::get_states(const TPCANMsg &msg, float &position, float &velocity, float &kp, float &kd, float &torque, bool &in_oc_mode, bool &has_fault) const
{
    if (msg.LEN < 7 || msg.DATA[0] != CMD_READ_STATES)
    {
        std::cout << "MsgDecoder::get_states: unexpected frame" << "RX ID: " << std::hex << int(msg.ID) << std::dec << "  LEN: " << int(msg.LEN) << "  DATA: " << std::hex << int(msg.DATA[0]) << " " << int(msg.DATA[1]) << " " << int(msg.DATA[2]) << " " << int(msg.DATA[3]) << " " << int(msg.DATA[4]) << " " << int(msg.DATA[5]) << " " << int(msg.DATA[6]) << std::dec << std::endl;
        
        position = velocity = torque = 0.0f;
        return;
    }
    
    uint16_t p16 = uint16_t(msg.DATA[1]) << 8 | uint16_t(msg.DATA[2]);
    uint16_t v12 = (uint16_t(msg.DATA[3]) << 4) | ((msg.DATA[4] & 0xF0) >> 4);
    uint16_t t12 = ((msg.DATA[4] & 0x0F) << 8) | msg.DATA[5];
    
    position = unmap_signed_16(p16 * gear_ratio_, POS_MAX);
    velocity = unmap_signed_12(v12 * gear_ratio_, VEL_MAX);
    torque   = unmap_signed_12(t12 * torque_constant_ * gear_ratio_, T_MAX);

    // Optional: interpret status in msg.DATA[6] if needed
    in_oc_mode = (msg.DATA[6] & 0x01) != 0;
    has_fault  = (msg.DATA[6] & 0x02) != 0;

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

    uint16_t pos_u16 = (uint16_t(msg.DATA[1]) << 8) | uint16_t(msg.DATA[2]);
    uint16_t vel_u16 = (uint16_t(msg.DATA[3]) << 8) | uint16_t(msg.DATA[4]);
    uint16_t tq_u16  = (uint16_t(msg.DATA[5]) << 8) | uint16_t(msg.DATA[6]);

    pos_max_rad = float(pos_u16) * 0.1f;   // 0.1 rad / LSB
    vel_max_rps = float(vel_u16) * 0.01f;  // 0.01 rad/s / LSB
    tq_max_nm   = float(tq_u16) * 0.01f;   // 0.01 Nm / LSB

}
} // namespace can_protocol
