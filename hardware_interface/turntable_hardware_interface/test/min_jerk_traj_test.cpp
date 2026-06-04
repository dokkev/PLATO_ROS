#include <cmath>

#include <gtest/gtest.h>

#include "turntable_hardware_interface/min_jerk_traj.hpp"

namespace turntable_hardware_interface
{
namespace
{

constexpr double kTolerance = 1e-9;

TEST(MinJerkTrajTest, StartsAndEndsAtRequestedPositions)
{
  const MinJerkTraj traj(10.0, 110.0, 2.0);

  const auto start = traj.sample(0.0);
  EXPECT_NEAR(start.position, 10.0, kTolerance);
  EXPECT_NEAR(start.velocity, 0.0, kTolerance);
  EXPECT_NEAR(start.acceleration, 0.0, kTolerance);

  const auto end = traj.sample(2.0);
  EXPECT_NEAR(end.position, 110.0, kTolerance);
  EXPECT_NEAR(end.velocity, 0.0, kTolerance);
  EXPECT_NEAR(end.acceleration, 0.0, kTolerance);
}

TEST(MinJerkTrajTest, HasReasonableMidpoint)
{
  const MinJerkTraj traj(0.0, 100.0, 2.0);

  const auto midpoint = traj.sample(1.0);
  EXPECT_NEAR(midpoint.position, 50.0, kTolerance);
  EXPECT_GT(midpoint.velocity, 0.0);
  EXPECT_NEAR(midpoint.acceleration, 0.0, kTolerance);
}

TEST(MinJerkTrajTest, IsMonotonicForIncreasingTrajectory)
{
  const MinJerkTraj traj(-25.0, 75.0, 1.0);

  double previous_position = traj.position(0.0);
  for (int i = 1; i <= 100; ++i) {
    const double t = static_cast<double>(i) / 100.0;
    const double position = traj.position(t);
    EXPECT_GE(position + kTolerance, previous_position);
    previous_position = position;
  }
}

TEST(MinJerkTrajTest, IsMonotonicForDecreasingTrajectory)
{
  const MinJerkTraj traj(75.0, -25.0, 1.0);

  double previous_position = traj.position(0.0);
  for (int i = 1; i <= 100; ++i) {
    const double t = static_cast<double>(i) / 100.0;
    const double position = traj.position(t);
    EXPECT_LE(position - kTolerance, previous_position);
    previous_position = position;
  }
}

TEST(MinJerkTrajTest, ClampsOutsideTimeRange)
{
  const MinJerkTraj traj(0.0, 42.0, 1.0);

  EXPECT_NEAR(traj.position(-1.0), 0.0, kTolerance);
  EXPECT_NEAR(traj.velocity(-1.0), 0.0, kTolerance);
  EXPECT_NEAR(traj.acceleration(-1.0), 0.0, kTolerance);

  EXPECT_NEAR(traj.position(2.0), 42.0, kTolerance);
  EXPECT_NEAR(traj.velocity(2.0), 0.0, kTolerance);
  EXPECT_NEAR(traj.acceleration(2.0), 0.0, kTolerance);
}

TEST(MinJerkTrajTest, RejectsNonPositiveDuration)
{
  EXPECT_THROW(MinJerkTraj(0.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(MinJerkTraj(0.0, 1.0, -1.0), std::invalid_argument);

  MinJerkTraj traj;
  EXPECT_THROW(traj.reset(0.0, 1.0, 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace turntable_hardware_interface
