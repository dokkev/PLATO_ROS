#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "can_hardware_common/core/lifecycle_plan.hpp"
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

TEST(PlatoProtocolTest, EnableBuildsScheduledLifecycleRequests)
{
  auto actuators = make_plato_actuators(4);
  plato_hand::PlatoProtocol protocol;

  const auto plan = protocol.build_lifecycle_plan(
    actuators, can_hardware_common::core::LifecycleOperation::kEnable);

  EXPECT_TRUE(plan.ready);
  EXPECT_EQ(plan.operation, can_hardware_common::core::LifecycleOperation::kEnable);
  EXPECT_EQ(plan.dispatch_policy, can_hardware_common::core::DispatchPolicy::kScheduledRequests);
  EXPECT_TRUE(plan.direct_frames.empty());
  ASSERT_EQ(plan.scheduled_requests.size(), actuators.size());
  EXPECT_EQ(plan.scheduled_requests.front().reply.expected_rx_id, actuators.front().get_rx_id());
  EXPECT_EQ(
    plan.scheduled_requests.front().reply.expected_opcode,
    CommandByte::START_MOTOR);
}

TEST(PlatoProtocolTest, DisableBuildsScheduledLifecycleRequests)
{
  auto actuators = make_plato_actuators(3);
  plato_hand::PlatoProtocol protocol;

  const auto plan = protocol.build_lifecycle_plan(
    actuators, can_hardware_common::core::LifecycleOperation::kDisable);

  EXPECT_TRUE(plan.ready);
  EXPECT_EQ(plan.operation, can_hardware_common::core::LifecycleOperation::kDisable);
  EXPECT_EQ(plan.dispatch_policy, can_hardware_common::core::DispatchPolicy::kScheduledRequests);
  EXPECT_TRUE(plan.direct_frames.empty());
  ASSERT_EQ(plan.scheduled_requests.size(), actuators.size());
  EXPECT_EQ(plan.scheduled_requests.front().reply.expected_rx_id, actuators.front().get_rx_id());
  EXPECT_EQ(
    plan.scheduled_requests.front().reply.expected_opcode,
    CommandByte::STOP_MOTOR);
}

TEST(PlatoProtocolTest, ZeroUsesCustomExecutionPolicy)
{
  auto actuators = make_plato_actuators(2);
  plato_hand::PlatoProtocol protocol;

  const auto plan = protocol.build_lifecycle_plan(
    actuators, can_hardware_common::core::LifecycleOperation::kZero);

  EXPECT_TRUE(plan.ready);
  EXPECT_EQ(plan.operation, can_hardware_common::core::LifecycleOperation::kZero);
  EXPECT_EQ(plan.dispatch_policy, can_hardware_common::core::DispatchPolicy::kCustomExecution);
  EXPECT_TRUE(plan.direct_frames.empty());
  EXPECT_TRUE(plan.scheduled_requests.empty());
}

}  // namespace
