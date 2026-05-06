#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "plato_hardware_interface/actuator.hpp"
#include "plato_hardware_interface/can_protocol.hpp"
#include "plato_hardware_interface/plato_protocol.hpp"

namespace
{

plato_actuator::Config make_plato_config(std::uint8_t tx_id, std::uint8_t rx_id)
{
  plato_actuator::Config config;
  config.static_config.can_tx_id = tx_id;
  config.static_config.can_rx_id = rx_id;
  config.static_config.direction = 1;
  config.static_config.torque_constant = 0.1f;
  config.static_config.gear_ratio = 1.0f;
  config.static_config.servo_current_milliamps = 100;
  return config;
}

std::vector<plato_actuator::Actuator> make_plato_actuators(std::size_t count)
{
  std::vector<plato_actuator::Actuator> actuators;
  actuators.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    actuators.emplace_back(make_plato_config(
      static_cast<std::uint8_t>(0x10 + i + 1),
      static_cast<std::uint8_t>(0x20 + i + 1)));
  }
  return actuators;
}

TEST(PlatoProtocolTest, AppendEnableFramesUsesActuatorEnableCommands)
{
  auto actuators = make_plato_actuators(4);
  plato_hand::PlatoProtocol protocol;
  std::vector<TPCANMsg> frames;

  protocol.append_enable_frames(actuators, frames);

  ASSERT_EQ(frames.size(), actuators.size());
  EXPECT_EQ(frames[0].ID, actuators[0].get_tx_id());
  EXPECT_EQ(frames.front().DATA[0], CommandByte::START_MOTOR);
  EXPECT_EQ(frames.front().DATA[1], 1);
  EXPECT_EQ(frames[2].ID, actuators[2].enable_motor().frame.ID);
  EXPECT_EQ(frames[2].DATA[0], CommandByte::START_MOTOR);
}

TEST(PlatoProtocolTest, AppendDisableFramesUsesActuatorDisableCommands)
{
  auto actuators = make_plato_actuators(3);
  plato_hand::PlatoProtocol protocol;
  std::vector<TPCANMsg> frames;

  protocol.append_disable_frames(actuators, frames);

  ASSERT_EQ(frames.size(), actuators.size());
  EXPECT_EQ(frames[0].ID, actuators[0].get_tx_id());
  EXPECT_EQ(frames.front().DATA[0], CommandByte::STOP_MOTOR);
  EXPECT_EQ(frames.front().DATA[1], 1);
  EXPECT_EQ(frames[2].ID, actuators[2].disable_motor().frame.ID);
  EXPECT_EQ(frames[2].DATA[0], CommandByte::STOP_MOTOR);
}

}  // namespace
