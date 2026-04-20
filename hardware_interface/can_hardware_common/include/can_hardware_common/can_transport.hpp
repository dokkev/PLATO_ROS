#ifndef CAN_HARDWARE_COMMON__CAN_TRANSPORT_HPP_
#define CAN_HARDWARE_COMMON__CAN_TRANSPORT_HPP_

#include <PCANBasic.h>

#include <chrono>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "can_hardware_common/can_frame_types.hpp"
#include "can_hardware_common/pcan_interface.hpp"

namespace can_hardware_common
{

class CanTransport
{
public:
  using RxObserver = std::function<void(const RxFrame & rx_frame)>;
  using TxSimulator = std::function<std::vector<RxFrame>(const TxFrame & tx_frame)>;

  struct RxPollResult
  {
    std::size_t processed_frames = 0;
    TPCANStatus status = PCAN_ERROR_QRCVEMPTY;

    bool transport_ok() const
    {
      return status == PCAN_ERROR_OK || status == PCAN_ERROR_QRCVEMPTY;
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

  void set_tx_gap(std::chrono::microseconds tx_gap);
  void set_min_inter_frame_gap(std::chrono::microseconds tx_gap) { set_tx_gap(tx_gap); }

  RxPollResult poll_rx();
  RxPollResult process_rx() { return poll_rx(); }
  TPCANStatus read_frame(RxFrame & rx_frame, std::chrono::microseconds timeout);

  void add_rx_observer(RxObserver observer);
  void clear_rx_observers();

  void set_tx_simulator(TxSimulator simulator, bool bypass_hardware = true);
  void clear_tx_simulator();
  bool has_tx_simulator() const;

  TPCANStatus send_tx_frame(const TxFrame & tx_frame);
  TPCANStatus send_if_ready(const TxFrame & tx_frame) { return send_tx_frame(tx_frame); }
  bool can_send_tx_now(std::chrono::steady_clock::time_point now) const;
  std::chrono::steady_clock::time_point next_tx_time() const;
  std::chrono::steady_clock::time_point next_send_time() const { return next_tx_time(); }

  BusDiagnostics get_diagnostics();
  static std::string bus_status_string(TPCANStatus status);

private:
  static constexpr std::size_t kMaxRxPerPoll = 30U;

  pcan_interface::PCANInterface channel_;
  std::vector<RxObserver> rx_observers_;

  mutable std::mutex tx_mutex_;
  std::chrono::microseconds tx_gap_{0};
  std::chrono::steady_clock::time_point last_tx_time_{};

  mutable std::mutex simulator_mutex_;
  TxSimulator tx_simulator_{};
  bool simulator_bypass_hardware_ = false;
  std::deque<RxFrame> injected_rx_frames_;
};

}  // namespace can_hardware_common

#endif  // CAN_HARDWARE_COMMON__CAN_TRANSPORT_HPP_
