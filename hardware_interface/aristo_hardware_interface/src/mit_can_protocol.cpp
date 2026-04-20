#include "aristo_hardware_interface/mit_can_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>
#include <optional>

#include "aristo_hardware_interface/actuator.hpp"

namespace mit_can_protocol
{

static constexpr uint8_t CMD_CFG_LIMITS = 0xF0;
static constexpr uint8_t CMD_READ_STATES = 0xF1;
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
  float pos_max,
  float vel_max,
  float t_max,
  uint8_t tx_id)
{
  std::memset(&msg, 0, sizeof(msg));
  msg.ID = tx_id | STDID_OC_BIT;
  msg.LEN = 8;

  const float pos_val = pos_set ? std::clamp(pos_rad, -pos_max, pos_max) : 0.0f;
  const float vel_val = vel_set ? std::clamp(vel_rps, -vel_max, vel_max) : 0.0f;
  const float tq_val = tq_set ? std::clamp(tq_nm, -t_max, t_max) : 0.0f;
  const float kp_val = kp_set ? std::clamp(kp, 0.0f, KP_MAX) : 0.0f;
  const float kd_val = kd_set ? std::clamp(kd, 0.0f, KD_MAX) : 0.0f;

  const uint16_t p16 = map_signed_16(pos_val, pos_max);
  const uint16_t v12 = map_signed_12(vel_val, vel_max);
  const uint16_t t12 = map_signed_12(tq_val, t_max);
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
  msg.ID = tx_id_;
  msg.LEN = 7;
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

void MsgEncoder::set_default_can_limits(TPCANMsg & msg)
{
  msg.ID = tx_id_;
  msg.LEN = 7;
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

void MsgEncoder::set_zero_position(TPCANMsg & msg)
{
  msg.ID = tx_id_;
  msg.LEN = 1;
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
    POS_MAX,
    VEL_MAX,
    T_MAX,
    tx_id_);
}

void MsgEncoder::stop_motor(TPCANMsg & msg)
{
  stop_control(msg);
}

void MsgEncoder::stop_control(TPCANMsg & msg)
{
  msg.ID = tx_id_;
  msg.LEN = 1;
  msg.DATA[0] = CMD_EXIT_OC_MODE;
}

void MsgEncoder::clear_fault(TPCANMsg & msg)
{
  msg.ID = tx_id_;
  msg.LEN = 1;
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
  const float motor_position = position_rad;
  const float motor_velocity = velocity_rps;
  const float motor_torque = torque_nm * 2.0f / gear_ratio_;

  pack_oc_frame(
    msg,
    motor_position,
    true,
    motor_velocity,
    true,
    kp,
    true,
    kd,
    true,
    motor_torque,
    true,
    POS_MAX,
    VEL_MAX,
    T_MAX,
    tx_id_);
}

void MsgDecoder::get_states(
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
  uint16_t kp12 = 0;
  uint16_t kd12 = 0;

  if (msg.LEN == 8) {
    p16 = (static_cast<uint16_t>(msg.DATA[0]) << 8) | static_cast<uint16_t>(msg.DATA[1]);
    v12 = (static_cast<uint16_t>(msg.DATA[2]) << 4) | ((msg.DATA[3] & 0xF0) >> 4);
    kp12 = ((msg.DATA[3] & 0x0F) << 8) | msg.DATA[4];
    kd12 = (static_cast<uint16_t>(msg.DATA[5]) << 4) | ((msg.DATA[6] & 0xF0) >> 4);
    t12 = ((msg.DATA[6] & 0x0F) << 8) | msg.DATA[7];
    in_oc_mode = true;
    has_fault = false;
  } else if (msg.LEN >= 7 && msg.DATA[0] == CMD_READ_STATES) {
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
    return;
  }

  position = unmap_signed_16(p16, POS_MAX);
  velocity = unmap_signed_12(v12, VEL_MAX);
  torque = unmap_signed_12(t12, T_MAX);

  if (msg.LEN == 8) {
    kp = static_cast<float>(kp12) * (KP_MAX / MAX_12BIT);
    kd = static_cast<float>(kd12) * (KD_MAX / MAX_12BIT);
  } else {
    kp = 0.0f;
    kd = 0.0f;
  }
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

MITProtocol::MITProtocol(const can_hardware_common::ActuatorCoreConfig & config)
: tx_id_(config.can_tx_id),
  encoder_(config.gear_ratio, config.can_tx_id),
  decoder_(),
  onoff_msg_(make_message_(config.can_tx_id, 8)),
  cmd_msg_(make_message_(config.can_tx_id, 8)),
  config_msg_(make_message_(config.can_tx_id, 7))
{
}

actuator::TxCommand MITProtocol::make_enable_motor_command()
{
  encoder_.start_motor(onoff_msg_);
  return actuator::TxCommand{onoff_msg_, 0};
}

actuator::TxCommand MITProtocol::make_disable_motor_command()
{
  encoder_.stop_motor(onoff_msg_);
  return actuator::TxCommand{onoff_msg_, 0};
}

actuator::TxCommand MITProtocol::make_stop_control_command()
{
  encoder_.stop_control(onoff_msg_);
  return actuator::TxCommand{onoff_msg_, 0};
}

actuator::TxCommand MITProtocol::make_torque_command(float motor_torque)
{
  encoder_.set_impedance(cmd_msg_, 0.0f, 0.0f, 0.0f, 0.0f, motor_torque);
  return actuator::TxCommand{cmd_msg_, 0};
}

std::optional<actuator::TxCommand> MITProtocol::make_impedance_command(
  const can_hardware_common::ActuatorTarget & motor_target)
{
  encoder_.set_impedance(
    cmd_msg_,
    motor_target.position,
    motor_target.velocity,
    motor_target.stiffness,
    motor_target.damping,
    motor_target.torque);
  return actuator::TxCommand{cmd_msg_};
}

actuator::TxCommand MITProtocol::make_zero_position_command()
{
  encoder_.set_zero_position(onoff_msg_);
  return actuator::TxCommand{onoff_msg_, 0};
}

actuator::TxCommand MITProtocol::make_default_can_limits_command()
{
  encoder_.set_default_can_limits(config_msg_);
  return actuator::TxCommand{config_msg_, 0};
}

void MITProtocol::process_message(const TPCANMsg & msg, aristo_actuator::Actuator & actuator)
{
  if (msg.LEN == 8 || (msg.LEN >= 7 && msg.DATA[0] == CMD_READ_STATES)) {
    float motor_position = 0.0f;
    float motor_velocity = 0.0f;
    float motor_torque = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    bool in_oc_mode = false;
    bool has_fault = false;

    decoder_.get_states(
      msg,
      motor_position,
      motor_velocity,
      kp,
      kd,
      motor_torque,
      in_oc_mode,
      has_fault);

    actuator.apply_motor_feedback(motor_position, motor_velocity, motor_torque, false, false);
    actuator.set_fault_flags(in_oc_mode, has_fault);
  }
}

TPCANMsg MITProtocol::make_message_(uint32_t can_id, uint8_t len)
{
  TPCANMsg msg;
  std::memset(&msg, 0, sizeof(msg));
  msg.ID = can_id;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = len > 8 ? 8 : len;
  return msg;
}

}  // namespace mit_can_protocol
