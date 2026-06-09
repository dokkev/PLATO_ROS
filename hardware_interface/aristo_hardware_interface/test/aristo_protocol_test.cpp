#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "aristo_hardware_interface/actuator.hpp"
#include "aristo_hardware_interface/aristo_protocol.hpp"
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
  config.limits.effort_limit = 5.0f;
  config.limits.stiffness_limit = 500.0f;
  config.limits.damping_limit = 5.0f;
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

float decode_command_torque(const TPCANMsg & msg)
{
  constexpr float kTorqueMaxNm = 18.0f;
  constexpr float kInvMax12Bit = 1.0f / 4095.0f;
  const uint16_t t12 = ((static_cast<uint16_t>(msg.DATA[6]) & 0x0F) << 8) |
                       static_cast<uint16_t>(msg.DATA[7]);
  return (static_cast<float>(t12) * kInvMax12Bit - 0.5f) * (kTorqueMaxNm + kTorqueMaxNm);
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

TEST(MitCanProtocolTest, SingleByteCommandsClearUnusedPayloadBytes)
{
  mit_can_protocol::MsgEncoder encoder(1.0f, 0x0A);
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
  mit_can_protocol::MsgEncoder encoder(1.0f, 0x0A);
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

TEST(MitCanProtocolTest, NonFiniteImpedanceCommandFallsBackToZeroEncoding)
{
  mit_can_protocol::MsgEncoder encoder(1.0f, 0x0A);
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
  mit_can_protocol::MsgEncoder encoder(1.0f, 0x0A);
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
  decoder.get_states(msg, position, velocity, kp, kd, torque, in_oc_mode, has_fault);

  EXPECT_FLOAT_EQ(position, 0.0f);
  EXPECT_FLOAT_EQ(velocity, 0.0f);
  EXPECT_FLOAT_EQ(kp, 0.0f);
  EXPECT_FLOAT_EQ(kd, 0.0f);
  EXPECT_FLOAT_EQ(torque, 0.0f);
  EXPECT_FALSE(in_oc_mode);
  EXPECT_FALSE(has_fault);
}

TEST(AristoActuatorTest, SmoothsTorqueForJointImpedanceCommands)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.core.gear_ratio = 2.0f;
  config.torque_smoothing = 0.5f;
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

  target.torque = 1.0f;
  const auto second_command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(second_command.has_value());
  EXPECT_NEAR(decode_command_torque(second_command->frame), 0.5f, 0.02f);

  const auto third_command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(third_command.has_value());
  EXPECT_NEAR(decode_command_torque(third_command->frame), 0.75f, 0.02f);
}

TEST(AristoActuatorTest, NonFiniteImpedanceTargetDoesNotBuildCommandOrPoisonSmoothing)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.core.gear_ratio = 2.0f;
  config.torque_smoothing = 0.5f;
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

  target.torque = 1.0f;
  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(decode_command_torque(command->frame), 0.5f, 0.02f);
}

TEST(AristoActuatorTest, ZeroTorqueSmoothingDisablesSmoothing)
{
  auto config = make_aristo_config(0x0A, 0x0A);
  config.core.gear_ratio = 2.0f;
  config.torque_smoothing = 0.0f;
  aristo_actuator::Actuator actuator(config);

  can_hardware_common::ActuatorTarget target{};
  target.position = 0.0f;
  target.velocity = 0.0f;
  target.stiffness = 0.0f;
  target.damping = 0.0f;
  target.torque = 0.0f;
  ASSERT_TRUE(actuator.set_joint_impedance(target).has_value());

  target.torque = 1.0f;
  const auto command = actuator.set_joint_impedance(target);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(decode_command_torque(command->frame), 1.0f, 0.02f);
}

}  // namespace
