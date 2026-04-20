#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>

#include "aristo_hardware_interface/actuator.hpp"
#include "aristo_hardware_interface/aristo_protocol.hpp"
#include "can_hardware_common/core/lifecycle_plan.hpp"

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
  config.limits.position_limit_min = -std::numeric_limits<float>::infinity();
  config.limits.position_limit_max = std::numeric_limits<float>::infinity();
  config.limits.effort_limit = std::numeric_limits<float>::infinity();
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

  const auto plan = protocol.build_lifecycle_plan(
    actuators, can_hardware_common::core::LifecycleOperation::kEnable);

  EXPECT_TRUE(plan.ready);
  EXPECT_EQ(plan.operation, can_hardware_common::core::LifecycleOperation::kEnable);
  EXPECT_EQ(plan.dispatch_policy, can_hardware_common::core::DispatchPolicy::kDirectFrames);
  EXPECT_TRUE(plan.scheduled_requests.empty());
  ASSERT_EQ(plan.direct_frames.size(), actuators.size());
  EXPECT_EQ(plan.direct_frames.front().ID, actuators.front().enable_motor().frame.ID);
}

TEST(AristoProtocolTest, DisableBuildsDirectFrames)
{
  auto actuators = make_aristo_actuators(3);
  aristo_hand::AristoProtocol protocol;

  const auto plan = protocol.build_lifecycle_plan(
    actuators, can_hardware_common::core::LifecycleOperation::kDisable);

  EXPECT_TRUE(plan.ready);
  EXPECT_EQ(plan.operation, can_hardware_common::core::LifecycleOperation::kDisable);
  EXPECT_EQ(plan.dispatch_policy, can_hardware_common::core::DispatchPolicy::kDirectFrames);
  EXPECT_TRUE(plan.scheduled_requests.empty());
  ASSERT_EQ(plan.direct_frames.size(), actuators.size());
  EXPECT_EQ(plan.direct_frames.front().ID, actuators.front().disable_motor().frame.ID);
}

TEST(AristoProtocolTest, ZeroBuildsDirectFrames)
{
  auto actuators = make_aristo_actuators(2);
  aristo_hand::AristoProtocol protocol;

  const auto plan = protocol.build_lifecycle_plan(
    actuators, can_hardware_common::core::LifecycleOperation::kZero);

  EXPECT_TRUE(plan.ready);
  EXPECT_EQ(plan.operation, can_hardware_common::core::LifecycleOperation::kZero);
  EXPECT_EQ(plan.dispatch_policy, can_hardware_common::core::DispatchPolicy::kDirectFrames);
  EXPECT_TRUE(plan.scheduled_requests.empty());
  ASSERT_EQ(plan.direct_frames.size(), actuators.size());
  EXPECT_EQ(plan.direct_frames.front().ID, actuators.front().set_current_position_as_zero().frame.ID);
}

}  // namespace
