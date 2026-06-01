#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
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

}  // namespace
