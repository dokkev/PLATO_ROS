#ifndef CAN_HARDWARE_COMMON__CAN_BUS_MANAGER_HPP_
#define CAN_HARDWARE_COMMON__CAN_BUS_MANAGER_HPP_

#include <PCANBasic.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include "can_hardware_common/pcan_interface.hpp"

namespace can_hardware_common
{

class CanBusManager
{
public:
  struct PollFrameHandler
  {
    using Callback = void (*)(void * context, const TPCANMsg & frame);

    void * context = nullptr;
    Callback callback = nullptr;
  };

  struct PollResult
  {
    std::size_t processed_frames = 0;
    // Raw status returned by the last transport read attempt. PCAN_ERROR_QRCVEMPTY means
    // the RX queue was empty, not that a bus fault occurred.
    TPCANStatus read_status = PCAN_ERROR_OK;
    // True when poll_once stopped because it exhausted max_frames before observing queue empty.
    bool hit_frame_budget = false;
  };

  CanBusManager();
  ~CanBusManager();

  CanBusManager(const CanBusManager &) = delete;
  CanBusManager & operator=(const CanBusManager &) = delete;
  CanBusManager(CanBusManager &&) = delete;
  CanBusManager & operator=(CanBusManager &&) = delete;

  // The bus manager is a foreground transport wrapper. It consumes raw frames from the PCAN RX
  // queue and optionally forwards each frame to the provided callback. Any frame routing policy,
  // including CAN ID and MSGTYPE filtering, belongs to the caller.
  PollResult poll_once(std::size_t max_frames = kDefaultMaxFramesPerPoll);
  PollResult poll_once(
    PollFrameHandler frame_handler,
    std::size_t max_frames = kDefaultMaxFramesPerPoll);
  TPCANStatus last_rx_status() const { return last_rx_status_.load(); }
  std::size_t handler_exception_count() const { return handler_exception_count_.load(); }

  TPCANStatus send_frame(
    const TPCANMsg & frame,
    std::chrono::milliseconds post_send_delay = std::chrono::milliseconds{0});

private:
  static constexpr std::size_t kDefaultMaxFramesPerPoll = 15U;

  pcan_interface::PCANInterface transport_;
  std::atomic<TPCANStatus> last_rx_status_{PCAN_ERROR_OK};
  std::atomic<std::size_t> handler_exception_count_{0};
};

}  // namespace can_hardware_common

#endif  // CAN_HARDWARE_COMMON__CAN_BUS_MANAGER_HPP_
