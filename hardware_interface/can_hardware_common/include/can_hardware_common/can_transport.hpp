#ifndef CAN_HARDWARE_COMMON__CAN_TRANSPORT_HPP_
#define CAN_HARDWARE_COMMON__CAN_TRANSPORT_HPP_

#include <PCANBasic.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "can_hardware_common/pcan_interface.hpp"

namespace can_hardware_common
{

/// Frame-level CAN transport: paced TX, RX pump with multi-observer, diagnostics.
/// No protocol awareness (no opcode matching, no retry logic).
class CanTransport
{
public:
  using RxObserver = std::function<void(const TPCANMsg & frame)>;

  struct RxResult
  {
    std::size_t processed_frames = 0;
    TPCANStatus status = PCAN_ERROR_QRCVEMPTY;
    bool is_bus_error() const
    {
      return status != PCAN_ERROR_OK && status != PCAN_ERROR_QRCVEMPTY;
    }
  };

  struct BusDiagnostics
  {
    TPCANStatus bus_status = PCAN_ERROR_OK;
    uint32_t channel_condition = 0;
    uint32_t receive_status = 0;
  };

  CanTransport() = default;
  ~CanTransport();

  CanTransport(const CanTransport &) = delete;
  CanTransport & operator=(const CanTransport &) = delete;
  CanTransport(CanTransport &&) = delete;
  CanTransport & operator=(CanTransport &&) = delete;

  void set_min_inter_frame_gap(std::chrono::microseconds gap);

  // ── RX ──

  RxResult process_rx();
  TPCANStatus read_frame(TPCANMsg & frame, std::chrono::microseconds timeout);

  void add_rx_observer(RxObserver observer);
  void clear_rx_observers();
  const std::vector<RxObserver> & rx_observers() const { return rx_observers_; }

  // ── TX ──

  /// Non-blocking paced send. Returns PCAN_ERROR_QXMTFULL if gap not ready.
  TPCANStatus send_if_ready(const TPCANMsg & frame);

  /// Earliest time send_if_ready() can succeed.
  std::chrono::steady_clock::time_point next_send_time() const;

  // ── Diagnostics ──

  BusDiagnostics get_diagnostics();
  static std::string bus_status_string(TPCANStatus status);

private:
  static constexpr std::size_t kMaxRxPerProcess = 30U;

  pcan_interface::PCANInterface channel_;
  std::vector<RxObserver> rx_observers_;

  mutable std::mutex tx_mutex_;
  std::chrono::microseconds min_inter_frame_gap_{std::chrono::microseconds(0)};
  std::chrono::steady_clock::time_point last_tx_time_{};
};

}  // namespace can_hardware_common

#endif  // CAN_HARDWARE_COMMON__CAN_TRANSPORT_HPP_
