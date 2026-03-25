#include "can_hardware_common/can_transport.hpp"

#include <cstdio>

namespace can_hardware_common
{

CanTransport::~CanTransport() = default;

void CanTransport::set_min_inter_frame_gap(std::chrono::microseconds gap)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  min_inter_frame_gap_ = gap;
}

// ── RX ──

CanTransport::RxResult CanTransport::process_rx()
{
  RxResult result;
  TPCANMsg frame{};

  for (std::size_t i = 0; i < kMaxRxPerProcess; ++i) {
    const TPCANStatus status = channel_.read(frame);
    if (status == PCAN_ERROR_OK) {
      for (auto & observer : rx_observers_) {
        try {
          observer(frame);
        } catch (const std::exception & e) {
          std::fprintf(stderr, "[CanTransport] RX observer threw for CAN ID 0x%X: %s\n",
            frame.ID, e.what());
        } catch (...) {
          std::fprintf(stderr, "[CanTransport] RX observer threw unknown for CAN ID 0x%X\n",
            frame.ID);
        }
      }
      ++result.processed_frames;
      continue;
    }
    result.status = status;
    return result;
  }

  result.status = PCAN_ERROR_OK;
  return result;
}

TPCANStatus CanTransport::read_frame(TPCANMsg & frame, std::chrono::microseconds timeout)
{
  return channel_.read_with_timeout(frame, timeout);
}

void CanTransport::add_rx_observer(RxObserver observer)
{
  rx_observers_.push_back(std::move(observer));
}

void CanTransport::clear_rx_observers()
{
  rx_observers_.clear();
}

// ── TX ──

TPCANStatus CanTransport::send_if_ready(const TPCANMsg & frame)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);

  const auto now = std::chrono::steady_clock::now();
  if (last_tx_time_ != std::chrono::steady_clock::time_point{} &&
    min_inter_frame_gap_.count() > 0 &&
    now - last_tx_time_ < min_inter_frame_gap_)
  {
    return PCAN_ERROR_QXMTFULL;
  }

  const TPCANStatus status = channel_.write(frame);
  if (status == PCAN_ERROR_OK) {
    last_tx_time_ = now;
  }
  return status;
}

std::chrono::steady_clock::time_point CanTransport::next_send_time() const
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (last_tx_time_ == std::chrono::steady_clock::time_point{}) {
    return {};
  }
  return last_tx_time_ + min_inter_frame_gap_;
}

// ── Diagnostics ──

CanTransport::BusDiagnostics CanTransport::get_diagnostics()
{
  BusDiagnostics diag;
  diag.bus_status = channel_.get_bus_status();
  (void)channel_.get_value(
    PCAN_CHANNEL_CONDITION, &diag.channel_condition, sizeof(diag.channel_condition));
  (void)channel_.get_value(
    PCAN_RECEIVE_STATUS, &diag.receive_status, sizeof(diag.receive_status));
  return diag;
}

std::string CanTransport::bus_status_string(TPCANStatus status)
{
  if (status == PCAN_ERROR_OK) {
    return "OK";
  }
  std::string result;
  auto append = [&](const char * label) {
      if (!result.empty()) { result += " | "; }
      result += label;
    };
  if (status & PCAN_ERROR_BUSLIGHT) { append("BUS_LIGHT"); }
  if (status & PCAN_ERROR_BUSHEAVY) { append("BUS_HEAVY"); }
  if (status & PCAN_ERROR_BUSPASSIVE) { append("BUS_PASSIVE"); }
  if (status & PCAN_ERROR_BUSOFF) { append("BUS_OFF"); }
  if (status & PCAN_ERROR_XMTFULL) { append("TX_BUFFER_FULL"); }
  if (status & PCAN_ERROR_OVERRUN) { append("RX_OVERRUN"); }
  if (status & PCAN_ERROR_QOVERRUN) { append("RX_QUEUE_OVERRUN"); }
  if (status & PCAN_ERROR_QXMTFULL) { append("TX_QUEUE_FULL"); }
  if (result.empty()) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "0x%X", status);
    result = buf;
  }
  return result;
}

}  // namespace can_hardware_common
