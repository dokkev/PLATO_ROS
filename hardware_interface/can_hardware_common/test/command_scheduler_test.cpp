#include <gtest/gtest.h>

#include "can_hardware_common/command_scheduler.hpp"

namespace can_hardware_common
{
namespace
{

TEST(CommandSchedulerTest, LatestOnlyReplacesPendingRequestWithSameKey)
{
  CommandScheduler scheduler;
  CommandRequest first;
  first.key = 7;
  first.tx_frame.ID = 0x11;

  CommandRequest replacement = first;
  replacement.tx_frame.ID = 0x22;

  scheduler.submit(first);
  scheduler.submit(replacement);

  ASSERT_TRUE(scheduler.has_pending());
  const auto next = scheduler.take_next();
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->tx_frame.ID, 0x22U);
  EXPECT_FALSE(scheduler.has_pending());
}

TEST(CommandSchedulerTest, NonLatestRequestsPreserveOrder)
{
  CommandScheduler scheduler;
  CommandRequest first;
  first.key = 1;
  first.latest_only = false;
  first.tx_frame.ID = 0x11;

  CommandRequest second;
  second.key = 1;
  second.latest_only = false;
  second.tx_frame.ID = 0x22;

  scheduler.submit(first);
  scheduler.submit(second);

  const auto next_first = scheduler.take_next();
  ASSERT_TRUE(next_first.has_value());
  EXPECT_EQ(next_first->tx_frame.ID, 0x11U);

  const auto next_second = scheduler.take_next();
  ASSERT_TRUE(next_second.has_value());
  EXPECT_EQ(next_second->tx_frame.ID, 0x22U);
}

}  // namespace
}  // namespace can_hardware_common
