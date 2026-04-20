#ifndef CAN_HARDWARE_COMMON__COMMAND_SCHEDULER_HPP_
#define CAN_HARDWARE_COMMON__COMMAND_SCHEDULER_HPP_

#include <deque>
#include <mutex>
#include <optional>

#include "can_hardware_common/command_request.hpp"

namespace can_hardware_common
{

class CommandScheduler
{
public:
  void submit(const CommandRequest & request);
  std::optional<CommandRequest> take_next();
  bool has_pending() const;
  void clear();

private:
  std::optional<std::size_t> find_latest_index_(uint32_t key) const;

  mutable std::mutex mutex_;
  std::deque<CommandRequest> pending_requests_;
};

}  // namespace can_hardware_common

#endif  // CAN_HARDWARE_COMMON__COMMAND_SCHEDULER_HPP_
