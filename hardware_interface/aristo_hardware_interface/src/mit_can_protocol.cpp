#include "aristo_hardware_interface/mit_can_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace mit_can_protocol
{

static constexpr uint8_t CMD_CFG_LIMITS = 0xF0;
static constexpr uint8_t CMD_READ_STATES = 0xF1;
static constexpr uint8_t CMD_MOTOR_PARAMS = 0xB0;
static constexpr uint8_t CMD_CLEAR_FAULT = 0xAF;
static constexpr uint8_t CMD_EXIT_OC_MODE = 0xCF;
static constexpr uint8_t CMD_SET_ZERO = 0xB1;
static constexpr uint32_t STDID_OC_BIT = 0x400;

static constexpr float INV_0_1 = 10.0f;
static constexpr float INV_0_01 = 100.0f;
static constexpr float INV_65535 = 1.0f / 65535.0f;
static constexpr float INV_4095 = 1.0f / 4095.0f;
static constexpr uint16_t MAX_12BIT = 4095;
static constexpr uint16_t MAX_16BIT = 65535;

static void reset_message(TPCANMsg & msg, uint32_t can_id, uint8_t len)
{
  std::memset(&msg, 0, sizeof(msg));
  msg.ID = can_id;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = len > 8 ? 8 : len;
}

static constexpr uint16_t clamp_u16(float v)
{
  return (v <= 0.0f) ? 0 : (v >= MAX_16BIT) ? MAX_16BIT : static_cast<uint16_t>(v + 0.5f);
}

static constexpr uint16_t to_pos_max_u16(float rad) { return clamp_u16(rad * INV_0_1); }
static constexpr uint16_t to_vel_max_u16(float rps) { return clamp_u16(rps * INV_0_01); }
static constexpr uint16_t to_tmax_u16(float nm) { return clamp_u16(nm * INV_0_01); }

static constexpr uint16_t map_signed_16(float x, float x_max)
{
  const float n = (x / (x_max + x_max) + 0.5f) * MAX_16BIT;
  return (n <= 0.0f) ? 0 : (n >= MAX_16BIT) ? MAX_16BIT : static_cast<uint16_t>(n + 0.5f);
}

static constexpr uint16_t map_signed_12(float x, float x_max)
{
  const float n = (x / (x_max + x_max) + 0.5f) * MAX_12BIT;
  return (n <= 0.0f) ? 0 : (n >= MAX_12BIT) ? MAX_12BIT : static_cast<uint16_t>(n + 0.5f);
}

static constexpr float unmap_signed_16(uint16_t u, float x_max)
{
  return (static_cast<float>(u) * INV_65535 - 0.5f) * (x_max + x_max);
}

static constexpr float unmap_signed_12(uint16_t u, float x_max)
{
  return (static_cast<float>(u) * INV_4095 - 0.5f) * (x_max + x_max);
}

static float finite_or_zero(float value)
{
  return std::isfinite(value) ? value : 0.0f;
}

static bool valid_positive_limit(float value)
{
  return std::isfinite(value) && value > 0.0f;
}

static MitLimits sanitize_limits(const MitLimits & limits)
{
  MitLimits sanitized;
  sanitized.pos_max_rad = valid_positive_limit(limits.pos_max_rad) ? limits.pos_max_rad : POS_MAX;
  sanitized.vel_max_rad_s = valid_positive_limit(limits.vel_max_rad_s) ? limits.vel_max_rad_s : VEL_MAX;
  sanitized.t_max_nm = valid_positive_limit(limits.t_max_nm) ? limits.t_max_nm : T_MAX;
  return sanitized;
}

static void pack_oc_frame(
  TPCANMsg & msg,
  float pos_rad,
  bool pos_set,
  float vel_rps,
  bool vel_set,
  float kp,
  bool kp_set,
  float kd,
  bool kd_set,
  float tq_nm,
  bool tq_set,
  const MitLimits & limits,
  uint8_t tx_id)
{
  reset_message(msg, tx_id | STDID_OC_BIT, 8);
  const MitLimits active_limits = sanitize_limits(limits);

  const float pos_val = pos_set ?
    std::clamp(finite_or_zero(pos_rad), -active_limits.pos_max_rad, active_limits.pos_max_rad) : 0.0f;
  const float vel_val = vel_set ?
    std::clamp(finite_or_zero(vel_rps), -active_limits.vel_max_rad_s, active_limits.vel_max_rad_s) : 0.0f;
  const float tq_val = tq_set ?
    std::clamp(finite_or_zero(tq_nm), -active_limits.t_max_nm, active_limits.t_max_nm) : 0.0f;
  const float kp_val = kp_set ? std::clamp(finite_or_zero(kp), 0.0f, KP_MAX) : 0.0f;
  const float kd_val = kd_set ? std::clamp(finite_or_zero(kd), 0.0f, KD_MAX) : 0.0f;

  const uint16_t p16 = map_signed_16(pos_val, active_limits.pos_max_rad);
  const uint16_t v12 = map_signed_12(vel_val, active_limits.vel_max_rad_s);
  const uint16_t t12 = map_signed_12(tq_val, active_limits.t_max_nm);
  const uint16_t kp12 = static_cast<uint16_t>(kp_val * (MAX_12BIT / KP_MAX) + 0.5f);
  const uint16_t kd12 = static_cast<uint16_t>(kd_val * (MAX_12BIT / KD_MAX) + 0.5f);

  msg.DATA[0] = static_cast<uint8_t>(p16 >> 8);
  msg.DATA[1] = static_cast<uint8_t>(p16 & 0xFF);
  msg.DATA[2] = static_cast<uint8_t>(v12 >> 4);
  msg.DATA[3] = static_cast<uint8_t>((v12 & 0x0F) << 4);
  msg.DATA[3] |= static_cast<uint8_t>((kp12 >> 8) & 0x0F);
  msg.DATA[4] = static_cast<uint8_t>(kp12 & 0xFF);
  msg.DATA[5] = static_cast<uint8_t>(kd12 >> 4);
  msg.DATA[6] = static_cast<uint8_t>((kd12 & 0x0F) << 4);
  msg.DATA[6] |= static_cast<uint8_t>((t12 >> 8) & 0x0F);
  msg.DATA[7] = static_cast<uint8_t>(t12 & 0xFF);
}

void MsgEncoder::set_can_limits(
  TPCANMsg & msg,
  float pos_max_rad,
  float vel_max_rps,
  float tq_max_nm,
  bool set_pos,
  bool set_vel,
  bool set_tq)
{
  reset_message(msg, tx_id_, 7);
  msg.DATA[0] = CMD_CFG_LIMITS;

  const uint16_t pos_u16 = to_pos_max_u16(set_pos ? pos_max_rad : POS_MAX);
  const uint16_t vel_u16 = to_vel_max_u16(set_vel ? vel_max_rps : VEL_MAX);
  const uint16_t tq_u16 = to_tmax_u16(set_tq ? tq_max_nm : T_MAX);

  msg.DATA[1] = static_cast<uint8_t>(pos_u16 & 0xFF);
  msg.DATA[2] = static_cast<uint8_t>(pos_u16 >> 8);
  msg.DATA[3] = static_cast<uint8_t>(vel_u16 & 0xFF);
  msg.DATA[4] = static_cast<uint8_t>(vel_u16 >> 8);
  msg.DATA[5] = static_cast<uint8_t>(tq_u16 & 0xFF);
  msg.DATA[6] = static_cast<uint8_t>(tq_u16 >> 8);
}

void MsgEncoder::read_can_limits(TPCANMsg & msg)
{
  reset_message(msg, tx_id_, 1);
  msg.DATA[0] = CMD_CFG_LIMITS;
}

void MsgEncoder::set_default_can_limits(TPCANMsg & msg)
{
  reset_message(msg, tx_id_, 7);
  msg.DATA[0] = CMD_CFG_LIMITS;

  constexpr uint16_t kPosMaxU16 = 955;
  constexpr uint16_t kVelMaxU16 = 4500;
  constexpr uint16_t kTorqueMaxU16 = 1800;

  msg.DATA[1] = static_cast<uint8_t>(kPosMaxU16 & 0xFF);
  msg.DATA[2] = static_cast<uint8_t>(kPosMaxU16 >> 8);
  msg.DATA[3] = static_cast<uint8_t>(kVelMaxU16 & 0xFF);
  msg.DATA[4] = static_cast<uint8_t>(kVelMaxU16 >> 8);
  msg.DATA[5] = static_cast<uint8_t>(kTorqueMaxU16 & 0xFF);
  msg.DATA[6] = static_cast<uint8_t>(kTorqueMaxU16 >> 8);
}

void MsgEncoder::read_motor_params(TPCANMsg & msg)
{
  reset_message(msg, tx_id_, 1);
  msg.DATA[0] = CMD_MOTOR_PARAMS;
}

void MsgEncoder::read_states(TPCANMsg & msg)
{
  reset_message(msg, tx_id_, 1);
  msg.DATA[0] = CMD_READ_STATES;
}

void MsgEncoder::set_zero_position(TPCANMsg & msg)
{
  reset_message(msg, tx_id_, 1);
  msg.DATA[0] = CMD_SET_ZERO;
}

void MsgEncoder::start_motor(TPCANMsg & msg)
{
  pack_oc_frame(
    msg,
    0.0f,
    true,
    0.0f,
    true,
    0.0f,
    true,
    0.0f,
    true,
    0.0f,
    true,
    active_limits_,
    tx_id_);
}

void MsgEncoder::stop_motor(TPCANMsg & msg)
{
  stop_control(msg);
}

void MsgEncoder::stop_control(TPCANMsg & msg)
{
  reset_message(msg, tx_id_, 1);
  msg.DATA[0] = CMD_EXIT_OC_MODE;
}

void MsgEncoder::clear_fault(TPCANMsg & msg)
{
  reset_message(msg, tx_id_, 1);
  msg.DATA[0] = CMD_CLEAR_FAULT;
}

void MsgEncoder::set_impedance(
  TPCANMsg & msg,
  const float position_rad,
  const float velocity_rps,
  const float kp,
  const float kd,
  const float torque_nm)
{
  pack_oc_frame(
    msg,
    position_rad,
    true,
    velocity_rps,
    true,
    kp,
    true,
    kd,
    true,
    torque_nm,
    true,
    active_limits_,
    tx_id_);
}

void MsgEncoder::set_active_limits(const MitLimits & limits)
{
  active_limits_ = sanitize_limits(limits);
}

bool MsgDecoder::get_states(
  const TPCANMsg & msg,
  float & position,
  float & velocity,
  float & kp,
  float & kd,
  float & torque,
  bool & in_oc_mode,
  bool & has_fault) const
{
  uint16_t p16 = 0;
  uint16_t v12 = 0;
  uint16_t t12 = 0;
  if (msg.LEN == 7 && msg.DATA[0] == CMD_READ_STATES) {
    p16 = (static_cast<uint16_t>(msg.DATA[1]) << 8) | static_cast<uint16_t>(msg.DATA[2]);
    v12 = (static_cast<uint16_t>(msg.DATA[3]) << 4) | ((msg.DATA[4] & 0xF0) >> 4);
    t12 = ((msg.DATA[4] & 0x0F) << 8) | msg.DATA[5];
    in_oc_mode = (msg.DATA[6] & 0x01) != 0;
    has_fault = (msg.DATA[6] & 0x02) != 0;
  } else {
    std::cout << "MsgDecoder::get_states unexpected frame RX ID: " << std::hex << int(msg.ID)
              << std::dec << " LEN: " << int(msg.LEN) << std::endl;
    position = velocity = kp = kd = torque = 0.0f;
    in_oc_mode = false;
    has_fault = false;
    return false;
  }

  const MitLimits active_limits = sanitize_limits(active_limits_);
  position = unmap_signed_16(p16, active_limits.pos_max_rad);
  velocity = unmap_signed_12(v12, active_limits.vel_max_rad_s);
  torque = unmap_signed_12(t12, active_limits.t_max_nm);
  kp = 0.0f;
  kd = 0.0f;
  return true;
}

void MsgDecoder::get_limits(
  const TPCANMsg & msg,
  float & pos_max_rad,
  float & vel_max_rps,
  float & tq_max_nm) const
{
  if (msg.LEN < 7 || msg.DATA[0] != CMD_CFG_LIMITS) {
    std::cerr << "MsgDecoder::get_limits: unexpected frame" << std::endl;
    pos_max_rad = vel_max_rps = tq_max_nm = 0.0f;
    return;
  }

  const uint16_t pos_u16 = static_cast<uint16_t>(msg.DATA[1]) | (static_cast<uint16_t>(msg.DATA[2]) << 8);
  const uint16_t vel_u16 = static_cast<uint16_t>(msg.DATA[3]) | (static_cast<uint16_t>(msg.DATA[4]) << 8);
  const uint16_t tq_u16 = static_cast<uint16_t>(msg.DATA[5]) | (static_cast<uint16_t>(msg.DATA[6]) << 8);

  pos_max_rad = pos_u16 * 0.1f;
  vel_max_rps = vel_u16 * 0.01f;
  tq_max_nm = tq_u16 * 0.01f;
}

std::optional<MitLimits> MsgDecoder::get_limits(const TPCANMsg & msg) const
{
  float pos_max_rad = 0.0f;
  float vel_max_rad_s = 0.0f;
  float tq_max_nm = 0.0f;
  get_limits(msg, pos_max_rad, vel_max_rad_s, tq_max_nm);
  if (!valid_positive_limit(pos_max_rad) ||
    !valid_positive_limit(vel_max_rad_s) ||
    !valid_positive_limit(tq_max_nm))
  {
    return std::nullopt;
  }

  return MitLimits{pos_max_rad, vel_max_rad_s, tq_max_nm};
}

bool MsgDecoder::get_motor_params(const TPCANMsg & msg, MotorParams & params) const
{
  if (msg.LEN != 7 || msg.DATA[0] != CMD_MOTOR_PARAMS) {
    return false;
  }

  MotorParams decoded;
  decoded.pole_pairs = msg.DATA[1];
  std::memcpy(&decoded.torque_constant_nm_per_a, &msg.DATA[2], sizeof(decoded.torque_constant_nm_per_a));
  decoded.gear_ratio = msg.DATA[6];

  if (!std::isfinite(decoded.torque_constant_nm_per_a) ||
    decoded.torque_constant_nm_per_a <= 0.0f ||
    decoded.torque_constant_nm_per_a > 100.0f)
  {
    return false;
  }

  params = decoded;
  return true;
}

}  // namespace mit_can_protocol
