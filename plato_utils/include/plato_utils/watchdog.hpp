#pragma once

#include <chrono>

namespace plato_utils
{

/// Simple timeout watchdog.
/// Call kick() each time a valid event occurs (e.g. actuator response received).
/// Call is_expired() to check if the timeout has elapsed since the last kick.
/// Starts in expired state until the first kick().
class Watchdog
{
public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;

  explicit Watchdog(Duration timeout)
  : timeout_(timeout) {}

  void kick() { last_kick_ = Clock::now(); }

  bool is_expired() const
  {
    if (last_kick_ == Clock::time_point{}) {
      return true;  // Never kicked.
    }
    return (Clock::now() - last_kick_) > timeout_;
  }

  Duration time_since_kick() const
  {
    if (last_kick_ == Clock::time_point{}) {
      return Duration::max();
    }
    return Clock::now() - last_kick_;
  }

  void set_timeout(Duration timeout) { timeout_ = timeout; }
  Duration timeout() const { return timeout_; }

private:
  Duration timeout_;
  Clock::time_point last_kick_{};
};

}  // namespace plato_utils
