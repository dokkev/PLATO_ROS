#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "aristo_hardware_interface/actuator.hpp"
#include "aristo_hardware_interface/aristo_protocol.hpp"
#include "aristo_hardware_interface/can_protocol.hpp"
#include "aristo_hardware_interface/mit_can_protocol.hpp"

namespace
{

aristo_actuator::Config make_aristo_config(std::uint8_t tx_id, std::uint8_t rx_id)
{
  aristo_actuator::Config config;
  config.core.can_tx_id = tx_id;
  config.core.can_rx_id = rx_id;
  config.core.direction = 1;
  config.core.torque_constant = 0.1f;
  config.core.gear_ratio = 1.0f;
  config.limits.position_limit_min = -3.14f;
  config.limits.position_limit_max = 3.14f;
  config.limits.velocity_limit = 10.0f;
  config.limits.effort_limit = 5000.0f;
  config.limits.stiffness_limit = 5000.0f;
  config.limits.damping_limit = 2500.0f;
  return config;
}

std::vector<aristo_actuator::Actuator> make_aristo_actuators(std::size_t count)
{
  std::vector<aristo_actuator::Actuator> actuators;
  actuators.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    actuators.emplace_back(make_aristo_config(
      static_cast<std::uint8_t>(0x10 + i + 1),
      static_cast<std::uint8_t>(0x20 + i + 1)));
  }
  return actuators;
}

float decode_command_torque(const TPCANMsg & msg, float torque_max_nm = mit_can_protocol::T_MAX)
{
  constexpr float kInvMax12Bit = 1.0f / 4095.0f;
  const uint16_t t12 = ((static_cast<uint16_t>(msg.DATA[6]) & 0x0F) << 8) |
                       static_cast<uint16_t>(msg.DATA[7]);
  return (static_cast<float>(t12) * kInvMax12Bit - 0.5f) * (torque_max_nm + torque_max_nm);
}

float decode_command_position(const TPCANMsg & msg, float position_max_rad = mit_can_protocol::POS_MAX)
{
  constexpr float kInvMax16Bit = 1.0f / 65535.0f;
  const uint16_t p16 = (static_cast<uint16_t>(msg.DATA[0]) << 8) |
                       static_cast<uint16_t>(msg.DATA[1]);
  return (static_cast<float>(p16) * kInvMax16Bit - 0.5f) *
         (position_max_rad + position_max_rad);
}

float decode_command_velocity(const TPCANMsg & msg, float velocity_max_rad_s = mit_can_protocol::VEL_MAX)
{
  constexpr float kInvMax12Bit = 1.0f / 4095.0f;
  const uint16_t v12 = (static_cast<uint16_t>(msg.DATA[2]) << 4) |
                       ((static_cast<uint16_t>(msg.DATA[3]) & 0xF0) >> 4);
  return (static_cast<float>(v12) * kInvMax12Bit - 0.5f) *
         (velocity_max_rad_s + velocity_max_rad_s);
}

float decode_command_stiffness(const TPCANMsg & msg)
{
  constexpr float kInvMax12Bit = 1.0f / 4095.0f;
  const uint16_t kp12 = ((static_cast<uint16_t>(msg.DATA[3]) & 0x0F) << 8) |
                        static_cast<uint16_t>(msg.DATA[4]);
  return static_cast<float>(kp12) * kInvMax12Bit * mit_can_protocol::KP_MAX;
}

float decode_command_damping(const TPCANMsg & msg)
{
  constexpr float kInvMax12Bit = 1.0f / 4095.0f;
  const uint16_t kd12 = (static_cast<uint16_t>(msg.DATA[5]) << 4) |
                        ((static_cast<uint16_t>(msg.DATA[6]) & 0xF0) >> 4);
  return static_cast<float>(kd12) * kInvMax12Bit * mit_can_protocol::KD_MAX;
}

TPCANMsg make_state_response_with_torque(float torque_nm, float torque_max_nm = mit_can_protocol::T_MAX)
{
  constexpr float kMax12Bit = 4095.0f;
  TPCANMsg msg{};
  msg.ID = 0x0A;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = 7;
  msg.DATA[0] = 0xF1;
  msg.DATA[1] = 0x80;
  msg.DATA[2] = 0x00;
  msg.DATA[3] = 0x80;
  msg.DATA[6] = 0x01;

  const float normalized = std::clamp(
    (torque_nm / (torque_max_nm + torque_max_nm)) + 0.5f,
    0.0f,
    1.0f);
  const auto t12 = static_cast<std::uint16_t>(std::round(normalized * kMax12Bit));
  msg.DATA[4] = static_cast<std::uint8_t>((t12 >> 8) & 0x0F);
  msg.DATA[5] = static_cast<std::uint8_t>(t12 & 0xFF);
  return msg;
}

TPCANMsg make_state_response(
  float position_rad,
  float velocity_rad_s,
  float torque_nm = 0.0f,
  float position_max_rad = mit_can_protocol::POS_MAX,
  float velocity_max_rad_s = mit_can_protocol::VEL_MAX,
  float torque_max_nm = mit_can_protocol::T_MAX)
{
  constexpr float kMax16Bit = 65535.0f;
  constexpr float kMax12Bit = 4095.0f;
  TPCANMsg msg{};
  msg.ID = 0x0A;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = 7;
  msg.DATA[0] = 0xF1;
  msg.DATA[6] = 0x01;

  const auto encode_signed_16 = [](float value, float max_value) {
    const float normalized = std::clamp(
      (value / (max_value + max_value)) + 0.5f,
      0.0f,
      1.0f);
    return static_cast<std::uint16_t>(std::round(normalized * kMax16Bit));
  };
  const auto encode_signed_12 = [](float value, float max_value) {
    const float normalized = std::clamp(
      (value / (max_value + max_value)) + 0.5f,
      0.0f,
      1.0f);
    return static_cast<std::uint16_t>(std::round(normalized * kMax12Bit));
  };

  const auto p16 = encode_signed_16(position_rad, position_max_rad);
  const auto v12 = encode_signed_12(velocity_rad_s, velocity_max_rad_s);
  const auto t12 = encode_signed_12(torque_nm, torque_max_nm);

  msg.DATA[1] = static_cast<std::uint8_t>(p16 >> 8);
  msg.DATA[2] = static_cast<std::uint8_t>(p16 & 0xFF);
  msg.DATA[3] = static_cast<std::uint8_t>(v12 >> 4);
  msg.DATA[4] = static_cast<std::uint8_t>(((v12 & 0x0F) << 4) | ((t12 >> 8) & 0x0F));
  msg.DATA[5] = static_cast<std::uint8_t>(t12 & 0xFF);
  return msg;
}

TEST(AristoProtocolTest, EnableBuildsDirectFrames)
{
  auto actuators = make_aristo_actuators(4);
  aristo_hand::AristoProtocol protocol;
  std::vector<TPCANMsg> direct_frames;

  protocol.append_enable_frames(actuators, direct_frames);

  ASSERT_EQ(direct_frames.size(), actuators.size());
  EXPECT_EQ(direct_frames.front().ID, actuators.front().enable_motor().frame.ID);
}

TEST(AristoProtocolTest, DisableBuildsDirectFrames)
{
  auto actuators = make_aristo_actuators(3);
  aristo_hand::AristoProtocol protocol;
  std::vector<TPCANMsg> direct_frames;

  protocol.append_disable_frames(actuators, direct_frames);

  ASSERT_EQ(direct_frames.size(), actuators.size());
  EXPECT_EQ(direct_frames.front().ID, actuators.front().disable_motor().frame.ID);
}

TEST(AristoProtocolTest, ZeroBuildsDirectFrames)
{
  auto actuators = make_aristo_actuators(2);
  aristo_hand::AristoProtocol protocol;
  std::vector<TPCANMsg> direct_frames;

  protocol.append_zero_frames(actuators, direct_frames);

  ASSERT_EQ(direct_frames.size(), actuators.size());
  EXPECT_EQ(direct_frames.front().ID, actuators.front().set_current_position_as_zero().frame.ID);
}

TEST(AristoProtocolTest, DetectsDangerousAllZeroMitCommandFrame)
{
  TPCANMsg frame{};
  frame.ID = 0x40A;
  frame.MSGTYPE = PCAN_MESSAGE_STANDARD;
  frame.LEN = 8;

  EXPECT_TRUE(aristo_hand::is_dangerous_all_zero_mit_command(frame));

  frame.DATA[0] = 0x80;
  EXPECT_FALSE(aristo_hand::is_dangerous_all_zero_mit_command(frame));
}

TEST(MitCanProtocolTest, SingleByteCommandsClearUnusedPayloadBytes)
{
  mit_can_protocol::MsgEncoder encoder(0x0A);
  TPCANMsg msg{};
  std::memset(&msg, 0xAA, sizeof(msg));

  encoder.set_zero_position(msg);

  EXPECT_EQ(msg.ID, 0x0A);
  EXPECT_EQ(msg.MSGTYPE, PCAN_MESSAGE_STANDARD);
  EXPECT_EQ(msg.LEN, 1);
  EXPECT_EQ(msg.DATA[0], 0xB1);
  for (std::size_t i = 1; i < 8; ++i) {
    EXPECT_EQ(msg.DATA[i], 0) << "DATA[" << i << "] should be deterministic padding";
  }

  std::memset(&msg, 0xAA, sizeof(msg));
  encoder.stop_control(msg);

  EXPECT_EQ(msg.ID, 0x0A);
  EXPECT_EQ(msg.MSGTYPE, PCAN_MESSAGE_STANDARD);
  EXPECT_EQ(msg.LEN, 1);
  EXPECT_EQ(msg.DATA[0], 0xCF);
  for (std::size_t i = 1; i < 8; ++i) {
    EXPECT_EQ(msg.DATA[i], 0) << "DATA[" << i << "] should be deterministic padding";
  }
}

TEST(MitCanProtocolTest, ZeroImpedanceCommandHasStableMidpointEncoding)
{
  mit_can_protocol::MsgEncoder encoder(0x0A);
  TPCANMsg first{};
  TPCANMsg second{};
  std::memset(&first, 0xAA, sizeof(first));
  std::memset(&second, 0x55, sizeof(second));

  encoder.set_impedance(first, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  encoder.set_impedance(second, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  constexpr std::array<uint8_t, 8> kExpectedZeroPayload = {
    0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00};

  EXPECT_EQ(first.ID, 0x40A);
  EXPECT_EQ(first.MSGTYPE, PCAN_MESSAGE_STANDARD);
  EXPECT_EQ(first.LEN, 8);
  EXPECT_EQ(second.ID, first.ID);
  EXPECT_EQ(second.MSGTYPE, first.MSGTYPE);
  EXPECT_EQ(second.LEN, first.LEN);

  for (std::size_t i = 0; i < kExpectedZeroPayload.size(); ++i) {
    EXPECT_EQ(first.DATA[i], kExpectedZeroPayload[i]);
    EXPECT_EQ(second.DATA[i], kExpectedZeroPayload[i]);
  }
}

TEST(MitCanProtocolTest, ReadsMotorParamsRequest)
{
  mit_can_protocol::MsgEncoder encoder(0x0A);
  TPCANMsg msg{};

  encoder.read_motor_params(msg);

  EXPECT_EQ(msg.ID, 0x0A);
  EXPECT_EQ(msg.MSGTYPE, PCAN_MESSAGE_STANDARD);
  EXPECT_EQ(msg.LEN, 1);
  EXPECT_EQ(msg.DATA[0], 0xB0);
  for (std::size_t i = 1; i < 8; ++i) {
    EXPECT_EQ(msg.DATA[i], 0);
  }
}

TEST(MitCanProtocolTest, ReadsStateRequest)
{
  mit_can_protocol::MsgEncoder encoder(0x0A);
  TPCANMsg msg{};

  encoder.read_states(msg);

  EXPECT_EQ(msg.ID, 0x0A);
  EXPECT_EQ(msg.MSGTYPE, PCAN_MESSAGE_STANDARD);
  EXPECT_EQ(msg.LEN, 1);
  EXPECT_EQ(msg.DATA[0], 0xF1);
  for (std::size_t i = 1; i < 8; ++i) {
    EXPECT_EQ(msg.DATA[i], 0);
  }
}

TEST(MitCanProtocolTest, DecodesMotorParamsResponse)
{
  mit_can_protocol::MsgDecoder decoder;
  TPCANMsg msg{};
  msg.ID = 0x0A;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = 7;
  msg.DATA[0] = 0xB0;
  msg.DATA[1] = 7;
  const float torque_constant = 0.123f;
  std::memcpy(&msg.DATA[2], &torque_constant, sizeof(torque_constant));
  msg.DATA[6] = 9;

  mit_can_protocol::MotorParams params;
  ASSERT_TRUE(decoder.get_motor_params(msg, params));
  EXPECT_EQ(params.pole_pairs, 7);
  EXPECT_FLOAT_EQ(params.torque_constant_nm_per_a, torque_constant);
  EXPECT_EQ(params.gear_ratio, 9);
}

TEST(MitCanProtocolTest, UsesActiveTorqueLimitForCommandEncoding)
{
  mit_can_protocol::MsgEncoder encoder(0x0A);
  encoder.set_active_limits(mit_can_protocol::MitLimits{
    mit_can_protocol::POS_MAX,
    mit_can_protocol::VEL_MAX,
    9.0f});
  TPCANMsg negative{};
  TPCANMsg zero{};
  TPCANMsg positive{};

  encoder.set_impedance(negative, 0.0f, 0.0f, 0.0f, 0.0f, -9.0f);
  encoder.set_impedance(zero, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  encoder.set_impedance(positive, 0.0f, 0.0f, 0.0f, 0.0f, 9.0f);

  EXPECT_NEAR(decode_command_torque(negative, 9.0f), -9.0f, 0.01f);
  EXPECT_NEAR(decode_command_torque(zero, 9.0f), 0.0f, 0.01f);
  EXPECT_NEAR(decode_command_torque(positive, 9.0f), 9.0f, 0.01f);
}

TEST(MitCanProtocolTest, NonFiniteImpedanceCommandFallsBackToZeroEncoding)
{
  mit_can_protocol::MsgEncoder encoder(0x0A);
  TPCANMsg msg{};
  const float nan = std::numeric_limits<float>::quiet_NaN();

  encoder.set_impedance(msg, nan, nan, nan, nan, nan);

  constexpr std::array<uint8_t, 8> kExpectedZeroPayload = {
    0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00};
  EXPECT_EQ(msg.ID, 0x40A);
  EXPECT_EQ(msg.MSGTYPE, PCAN_MESSAGE_STANDARD);
  EXPECT_EQ(msg.LEN, 8);
  for (std::size_t i = 0; i < kExpectedZeroPayload.size(); ++i) {
    EXPECT_EQ(msg.DATA[i], kExpectedZeroPayload[i]);
  }
}

TEST(MitCanProtocolTest, DecoderRejectsEightByteCommandShapedFrames)
{
  mit_can_protocol::MsgEncoder encoder(0x0A);
  mit_can_protocol::MsgDecoder decoder;
  TPCANMsg msg{};
  encoder.set_impedance(msg, 1.0f, 2.0f, 3.0f, 0.4f, 5.0f);

  float position = 1.0f;
  float velocity = 1.0f;
  float kp = 1.0f;
  float kd = 1.0f;
  float torque = 1.0f;
  bool in_oc_mode = true;
  bool has_fault = true;
  EXPECT_FALSE(decoder.get_states(msg, position, velocity, kp, kd, torque, in_oc_mode, has_fault));

  EXPECT_FLOAT_EQ(position, 0.0f);
  EXPECT_FLOAT_EQ(velocity, 0.0f);
  EXPECT_FLOAT_EQ(kp, 0.0f);
  EXPECT_FLOAT_EQ(kd, 0.0f);
  EXPECT_FLOAT_EQ(torque, 0.0f);
  EXPECT_FALSE(in_oc_mode);
  EXPECT_FALSE(has_fault);
}

TEST(MitCanProtocolTest, DecoderAcceptsSevenByteStateResponse)
{
  mit_can_protocol::MsgDecoder decoder;
  TPCANMsg msg{};
  msg.ID = 0x0A;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = 7;
  msg.DATA[0] = 0xF1;
  msg.DATA[1] = 0x80;
  msg.DATA[2] = 0x00;
  msg.DATA[3] = 0x80;
  msg.DATA[4] = 0x00;
  msg.DATA[5] = 0x00;
  msg.DATA[6] = 0x01;

  float position = 1.0f;
  float velocity = 1.0f;
  float kp = 1.0f;
  float kd = 1.0f;
  float torque = 1.0f;
  bool in_oc_mode = false;
  bool has_fault = true;
  ASSERT_TRUE(decoder.get_states(msg, position, velocity, kp, kd, torque, in_oc_mode, has_fault));

  EXPECT_NEAR(position, 0.0f, 0.01f);
  EXPECT_NEAR(velocity, 0.0f, 0.02f);
  EXPECT_NEAR(torque, -18.0f, 0.01f);
  EXPECT_TRUE(in_oc_mode);
  EXPECT_FALSE(has_fault);
}

TEST(MitCanProtocolTest, DecoderAcceptsSevenByteZeroTorqueStateResponse)
{
  mit_can_protocol::MsgDecoder decoder;
  TPCANMsg msg{};
  msg.ID = 0x0A;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = 7;
  msg.DATA[0] = 0xF1;
  msg.DATA[1] = 0x80;
  msg.DATA[2] = 0x00;
  msg.DATA[3] = 0x80;
  msg.DATA[4] = 0x08;
  msg.DATA[5] = 0x00;
  msg.DATA[6] = 0x01;

  float position = 1.0f;
  float velocity = 1.0f;
  float kp = 1.0f;
  float kd = 1.0f;
  float torque = 1.0f;
  bool in_oc_mode = false;
  bool has_fault = true;
  ASSERT_TRUE(decoder.get_states(msg, position, velocity, kp, kd, torque, in_oc_mode, has_fault));

  EXPECT_NEAR(position, 0.0f, 0.01f);
  EXPECT_NEAR(velocity, 0.0f, 0.02f);
  EXPECT_NEAR(torque, 0.0f, 0.01f);
  EXPECT_TRUE(in_oc_mode);
  EXPECT_FALSE(has_fault);
}

TEST(MitCanProtocolTest, DecoderUsesActiveTorqueLimitForStateResponse)
{
  mit_can_protocol::MsgDecoder decoder;
  decoder.set_active_limits(mit_can_protocol::MitLimits{
    mit_can_protocol::POS_MAX,
    mit_can_protocol::VEL_MAX,
    9.0f});

  TPCANMsg msg{};
  msg.ID = 0x0A;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = 7;
  msg.DATA[0] = 0xF1;
  msg.DATA[1] = 0x80;
  msg.DATA[2] = 0x00;
  msg.DATA[3] = 0x80;
  msg.DATA[6] = 0x01;

  float position = 0.0f;
  float velocity = 0.0f;
  float kp = 0.0f;
  float kd = 0.0f;
  float torque = 0.0f;
  bool in_oc_mode = false;
  bool has_fault = false;

  msg.DATA[4] = 0x00;
  msg.DATA[5] = 0x00;
  ASSERT_TRUE(decoder.get_states(msg, position, velocity, kp, kd, torque, in_oc_mode, has_fault));
  EXPECT_NEAR(torque, -9.0f, 0.01f);

  msg.DATA[4] = 0x08;
  msg.DATA[5] = 0x00;
  ASSERT_TRUE(decoder.get_states(msg, position, velocity, kp, kd, torque, in_oc_mode, has_fault));
  EXPECT_NEAR(torque, 0.0f, 0.01f);

  msg.DATA[4] = 0x0F;
  msg.DATA[5] = 0xFF;
  ASSERT_TRUE(decoder.get_states(msg, position, velocity, kp, kd, torque, in_oc_mode, has_fault));
  EXPECT_NEAR(torque, 9.0f, 0.01f);
}

TEST(AristoActuatorTest, SmoothsTorqueForJointImpedanceCommands)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.core.gear_ratio = 2.0f;
  config.torque_cmd_smoothing = 0.5f;
  aristo_actuator::Actuator actuator(config);

  can_hardware_common::ActuatorTarget target{};
  target.position = 0.0f;
  target.velocity = 0.0f;
  target.stiffness = 0.0f;
  target.damping = 0.0f;
  target.torque = 0.0f;
  const auto first_command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(first_command.has_value());
  EXPECT_NEAR(decode_command_torque(first_command->frame), 0.0f, 0.01f);

  target.torque = 1000.0f;
  const auto second_command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(second_command.has_value());
  EXPECT_NEAR(
    decode_command_torque(second_command->frame),
    500.0f * aristo_actuator::cmdEffortScale,
    0.02f);

  const auto third_command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(third_command.has_value());
  EXPECT_NEAR(
    decode_command_torque(third_command->frame),
    750.0f * aristo_actuator::cmdEffortScale,
    0.02f);
}

TEST(AristoActuatorTest, ConvertsMillinewtonMeterCommandsToCompensatedProtocolEfforts)
{
  aristo_actuator::Actuator actuator(make_aristo_config(0x0A, 0x0A));

  can_hardware_common::ActuatorTarget target{};
  target.position = 0.0f;
  target.velocity = 0.0f;
  target.stiffness = 1000.0f;  // mNm/rad
  target.damping = 250.0f;     // mNm/(rad/s)
  target.torque = 1000.0f;     // mNm

  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(
    decode_command_stiffness(command->frame),
    target.stiffness * aristo_actuator::cmdEffortScale,
    0.1f);
  EXPECT_NEAR(
    decode_command_damping(command->frame),
    target.damping * aristo_actuator::cmdEffortScale,
    0.01f);
  EXPECT_NEAR(
    decode_command_torque(command->frame),
    target.torque * aristo_actuator::cmdEffortScale,
    0.02f);
}

TEST(AristoActuatorTest, ClampsMillinewtonMeterTorqueToProtocolTorqueLimit)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.torque_cmd_smoothing = 0.0f;
  aristo_actuator::Actuator actuator(config);

  can_hardware_common::ActuatorTarget target{};
  target.position = 0.0f;
  target.velocity = 0.0f;
  target.stiffness = 0.0f;
  target.damping = 0.0f;
  target.torque = 3000.0f;

  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(decode_command_torque(command->frame), mit_can_protocol::T_MAX, 0.02f);
}

TEST(AristoActuatorTest, NonFiniteImpedanceTargetDoesNotBuildCommandOrPoisonSmoothing)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.core.gear_ratio = 2.0f;
  config.torque_cmd_smoothing = 0.5f;
  aristo_actuator::Actuator actuator(config);

  can_hardware_common::ActuatorTarget target{};
  target.position = 0.0f;
  target.velocity = 0.0f;
  target.stiffness = 0.0f;
  target.damping = 0.0f;
  target.torque = 0.0f;
  ASSERT_TRUE(actuator.set_joint_impedance(target).has_value());

  target.torque = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(actuator.set_joint_impedance(target).has_value());

  target.torque = 1000.0f;
  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(
    decode_command_torque(command->frame),
    500.0f * aristo_actuator::cmdEffortScale,
    0.02f);
}

TEST(AristoActuatorTest, ZeroTorqueSmoothingDisablesSmoothing)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.core.gear_ratio = 2.0f;
  config.torque_cmd_smoothing = 0.0f;
  aristo_actuator::Actuator actuator(config);

  can_hardware_common::ActuatorTarget target{};
  target.position = 0.0f;
  target.velocity = 0.0f;
  target.stiffness = 0.0f;
  target.damping = 0.0f;
  target.torque = 0.0f;
  ASSERT_TRUE(actuator.set_joint_impedance(target).has_value());

  target.torque = 1000.0f;
  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(
    decode_command_torque(command->frame),
    1000.0f * aristo_actuator::cmdEffortScale,
    0.02f);
}

TEST(AristoActuatorTest, LowerSoftLimitBlocksOutwardPositionTarget)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.torque_cmd_smoothing = 0.0f;
  config.limits.position_limit_min = -1.0f;
  config.limits.position_limit_max = 1.0f;
  aristo_actuator::Actuator actuator(config);

  actuator.process_rx_frame(make_state_response(-0.98f, 0.0f));
  ASSERT_TRUE(actuator.has_feedback());
  const float q_meas = actuator.get_feedback().position;

  can_hardware_common::ActuatorTarget target{};
  target.position = q_meas - 0.1f;
  target.velocity = 0.0f;
  target.stiffness = 1000.0f;
  target.damping = 80.0f;
  target.torque = 0.0f;

  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(decode_command_position(command->frame), q_meas, 0.01f);
}

TEST(AristoActuatorTest, LowerSoftLimitAllowsInwardPositionTarget)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.torque_cmd_smoothing = 0.0f;
  config.limits.position_limit_min = -1.0f;
  config.limits.position_limit_max = 1.0f;
  aristo_actuator::Actuator actuator(config);

  actuator.process_rx_frame(make_state_response(-0.98f, 0.0f));
  ASSERT_TRUE(actuator.has_feedback());
  const float q_meas = actuator.get_feedback().position;

  can_hardware_common::ActuatorTarget target{};
  target.position = q_meas + 0.1f;
  target.velocity = 0.0f;
  target.stiffness = 1000.0f;
  target.damping = 80.0f;
  target.torque = 0.0f;

  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(decode_command_position(command->frame), target.position, 0.01f);
}

TEST(AristoActuatorTest, LowerSoftLimitBlocksNegativeVelocityAndTorque)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.torque_cmd_smoothing = 0.0f;
  config.limits.position_limit_min = -1.0f;
  config.limits.position_limit_max = 1.0f;
  aristo_actuator::Actuator actuator(config);

  actuator.process_rx_frame(make_state_response(-0.98f, 0.0f));
  ASSERT_TRUE(actuator.has_feedback());

  can_hardware_common::ActuatorTarget target{};
  target.position = actuator.get_feedback().position;
  target.velocity = -1.0f;
  target.stiffness = 1000.0f;
  target.damping = 80.0f;
  target.torque = -1000.0f;

  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(decode_command_velocity(command->frame), 0.0f, 0.02f);
  EXPECT_NEAR(decode_command_torque(command->frame), 0.0f, 0.02f);
}

TEST(AristoActuatorTest, OverLimitUsesDampingOnlySafetyCommand)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.torque_cmd_smoothing = 0.0f;
  config.limits.position_limit_min = -1.0f;
  config.limits.position_limit_max = 1.0f;
  aristo_actuator::Actuator actuator(config);

  actuator.process_rx_frame(make_state_response(-1.02f, -0.5f));
  ASSERT_TRUE(actuator.has_feedback());
  const float q_meas = actuator.get_feedback().position;

  can_hardware_common::ActuatorTarget target{};
  target.position = 0.0f;
  target.velocity = -1.0f;
  target.stiffness = 2000.0f;
  target.damping = 80.0f;
  target.torque = -1000.0f;

  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(decode_command_position(command->frame), q_meas, 0.01f);
  EXPECT_NEAR(decode_command_velocity(command->frame), 0.0f, 0.02f);
  EXPECT_NEAR(decode_command_torque(command->frame), 0.0f, 0.02f);
  EXPECT_NEAR(decode_command_stiffness(command->frame), 0.0f, 0.1f);
  EXPECT_NEAR(
    decode_command_damping(command->frame),
    5.0f * aristo_actuator::cmdEffortScale,
    0.01f);
}

TEST(AristoActuatorTest, ScalesDecodedProtocolTorqueForJointEffort)
{
  aristo_actuator::Actuator actuator(make_aristo_config(0x0A, 0x0A));
  const TPCANMsg state = make_state_response_with_torque(8.0f);

  actuator.process_rx_frame(state);

  EXPECT_TRUE(actuator.has_feedback());
  EXPECT_NEAR(
    actuator.get_feedback().torque,
    8.0f * aristo_actuator::fbEffortScale,
    1.0f);
}

TEST(AristoActuatorTest, SmoothsDecodedJointEffortFeedback)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.torque_meas_smoothing = 0.5f;
  aristo_actuator::Actuator actuator(config);

  actuator.process_rx_frame(make_state_response_with_torque(8.0f));
  ASSERT_TRUE(actuator.has_feedback());
  EXPECT_NEAR(actuator.get_feedback().torque, 1000.0f, 1.0f);

  actuator.process_rx_frame(make_state_response_with_torque(16.0f));
  EXPECT_NEAR(actuator.get_feedback().torque, 1500.0f, 1.0f);

  actuator.process_rx_frame(make_state_response_with_torque(16.0f));
  EXPECT_NEAR(actuator.get_feedback().torque, 1750.0f, 1.0f);
}

}  // namespace
