#include "can_hardware_common/command_scheduler.hpp"

namespace can_hardware_common
{

void CommandScheduler::submit(const CommandRequest & request)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (request.latest_only && request.key != 0) {
    if (const auto index = find_latest_index_(request.key)) {
      pending_requests_.at(*index) = request;
      return;
    }
  }

  pending_requests_.push_back(request);
}

std::optional<CommandRequest> CommandScheduler::take_next()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_requests_.empty()) {
    return std::nullopt;
  }

  CommandRequest request = pending_requests_.front();
  pending_requests_.pop_front();
  return request;
}

bool CommandScheduler::has_pending() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !pending_requests_.empty();
}

void CommandScheduler::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_requests_.clear();
}

std::optional<std::size_t> CommandScheduler::find_latest_index_(uint32_t key) const
{
  for (std::size_t index = 0; index < pending_requests_.size(); ++index) {
    if (pending_requests_[index].key == key) {
      return index;
    }
  }

  return std::nullopt;
}

}  // namespace can_hardware_common
