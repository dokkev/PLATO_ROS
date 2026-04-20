#ifndef CAN_HARDWARE_COMMON__TIME_TYPES_HPP_
#define CAN_HARDWARE_COMMON__TIME_TYPES_HPP_

#include <chrono>

namespace can_hardware_common::types
{

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
using Duration = SteadyClock::duration;

}  // namespace can_hardware_common::types

#endif  // CAN_HARDWARE_COMMON__TIME_TYPES_HPP_
