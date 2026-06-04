#include "turntable_hardware_interface/min_jerk_traj.hpp"

#include <algorithm>
#include <stdexcept>

namespace turntable_hardware_interface
{

MinJerkTraj::MinJerkTraj(
  double start_position,
  double target_position,
  double duration_sec)
{
  reset(start_position, target_position, duration_sec);
}

void MinJerkTraj::reset(
  double start_position,
  double target_position,
  double duration_sec)
{
  if (duration_sec <= 0.0) {
    throw std::invalid_argument("MinJerkTraj duration_sec must be positive.");
  }

  start_position_ = start_position;
  target_position_ = target_position;
  duration_sec_ = duration_sec;
}

MinJerkTraj::Sample MinJerkTraj::sample(double elapsed_sec) const
{
  const double t = clamp_time_(elapsed_sec);
  const double u = normalized_time_(t);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double delta = target_position_ - start_position_;

  const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
  const double blend_dot = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration_sec_;
  const double blend_ddot = (60.0 * u - 180.0 * u2 + 120.0 * u3) /
    (duration_sec_ * duration_sec_);

  return Sample{
    start_position_ + delta * blend,
    delta * blend_dot,
    delta * blend_ddot,
  };
}

double MinJerkTraj::position(double elapsed_sec) const
{
  return sample(elapsed_sec).position;
}

double MinJerkTraj::velocity(double elapsed_sec) const
{
  return sample(elapsed_sec).velocity;
}

double MinJerkTraj::acceleration(double elapsed_sec) const
{
  return sample(elapsed_sec).acceleration;
}

bool MinJerkTraj::is_done(double elapsed_sec) const
{
  return elapsed_sec >= duration_sec_;
}

double MinJerkTraj::clamp_time_(double elapsed_sec) const
{
  return std::clamp(elapsed_sec, 0.0, duration_sec_);
}

double MinJerkTraj::normalized_time_(double elapsed_sec) const
{
  return elapsed_sec / duration_sec_;
}

}  // namespace turntable_hardware_interface
