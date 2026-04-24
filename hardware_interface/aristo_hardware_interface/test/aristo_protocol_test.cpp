#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "aristo_hardware_interface/actuator.hpp"
#include "aristo_hardware_interface/aristo_protocol.hpp"

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

}  // namespace
