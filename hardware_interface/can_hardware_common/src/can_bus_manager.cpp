#include "can_hardware_common/can_bus_manager.hpp"

#include <cstdio>
#include <exception>
#include <thread>

namespace can_hardware_common
{

namespace
{
struct LogThrottleState
{
  std::mutex mutex;
  std::chrono::steady_clock::time_point last_log_time{};
};

LogThrottleState & handler_exception_log_state()
{
  static LogThrottleState state;
  return state;
}

bool should_emit_log(
  LogThrottleState & throttle_state,
  std::chrono::steady_clock::duration throttle_period)
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(throttle_state.mutex);
  if (
    throttle_state.last_log_time.time_since_epoch().count() == 0 ||
    now - throttle_state.last_log_time >= throttle_period)
  {
    throttle_state.last_log_time = now;
    return true;
  }

  return false;
}
}  // namespace

CanBusManager::CanBusManager() = default;

CanBusManager::~CanBusManager() = default;

CanBusManager::PollResult CanBusManager::poll_once(std::size_t max_frames)
{
  return poll_once(PollFrameHandler{}, max_frames);
}

CanBusManager::PollResult CanBusManager::poll_once(
  PollFrameHandler frame_handler,
  std::size_t max_frames)
{
  PollResult result;
  TPCANMsg frame{};

  while (result.processed_frames < max_frames) {
    const TPCANStatus status = transport_.read(frame);
    if (status == PCAN_ERROR_OK) {
      if (frame_handler.callback != nullptr) {
        try {
          frame_handler.callback(frame_handler.context, frame);
        } catch (const std::exception & exception) {
          handler_exception_count_.fetch_add(1);
          if (should_emit_log(handler_exception_log_state(), std::chrono::milliseconds(1000))) {
            std::fprintf(
              stderr, "[can_hardware_common] RX callback for CAN ID 0x%X threw an exception: %s\n",
              frame.ID, exception.what());
          }
        } catch (...) {
          handler_exception_count_.fetch_add(1);
          if (should_emit_log(handler_exception_log_state(), std::chrono::milliseconds(1000))) {
            std::fprintf(
              stderr, "[can_hardware_common] RX callback for CAN ID 0x%X threw an unknown exception\n",
              frame.ID);
          }
        }
      }
      ++result.processed_frames;
      continue;
    }

    if (status == PCAN_ERROR_QRCVEMPTY) {
      result.read_status = status;
      last_rx_status_.store(status);
      return result;
    }

    result.read_status = status;
    last_rx_status_.store(status);
    return result;
  }

  result.read_status = PCAN_ERROR_OK;
  result.hit_frame_budget = max_frames > 0 && result.processed_frames == max_frames;
  last_rx_status_.store(result.read_status);
  return result;
}

TPCANStatus CanBusManager::send_frame(
  const TPCANMsg & frame,
  std::chrono::milliseconds post_send_delay)
{
  const TPCANStatus status = transport_.write(frame);
  if (status == PCAN_ERROR_OK && post_send_delay.count() > 0) {
    std::this_thread::sleep_for(post_send_delay);
  }

  return status;
}

}  // namespace can_hardware_common
