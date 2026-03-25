#include "can_hardware_common/can_transport.hpp"

#include <algorithm>
#include <cstdio>
#include <exception>
#include <thread>

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
  const auto dispatch_rx = [this](const TPCANMsg & rx_frame) {
      for (auto & observer : rx_observers_) {
        try {
          observer(rx_frame);
        } catch (const std::exception & e) {
          std::fprintf(stderr, "[CanTransport] RX observer threw for CAN ID 0x%X: %s\n",
            rx_frame.ID, e.what());
        } catch (...) {
          std::fprintf(stderr, "[CanTransport] RX observer threw unknown for CAN ID 0x%X\n",
            rx_frame.ID);
        }
      }
    };

  for (std::size_t i = 0; i < kMaxRxPerProcess; ++i) {
    {
      std::lock_guard<std::mutex> lock(simulator_mutex_);
      if (!injected_rx_frames_.empty()) {
        frame = injected_rx_frames_.front();
        injected_rx_frames_.pop_front();
        dispatch_rx(frame);
        ++result.processed_frames;
        continue;
      }
      if (simulator_bypass_hardware_ && static_cast<bool>(tx_simulator_)) {
        result.status = PCAN_ERROR_QRCVEMPTY;
        return result;
      }
    }

    const TPCANStatus status = channel_.read(frame);
    if (status == PCAN_ERROR_OK) {
      dispatch_rx(frame);
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
  {
    std::lock_guard<std::mutex> lock(simulator_mutex_);
    if (!injected_rx_frames_.empty()) {
      frame = injected_rx_frames_.front();
      injected_rx_frames_.pop_front();
      return PCAN_ERROR_OK;
    }
    if (!(simulator_bypass_hardware_ && static_cast<bool>(tx_simulator_))) {
      return channel_.read_with_timeout(frame, timeout);
    }
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(simulator_mutex_);
      if (!injected_rx_frames_.empty()) {
        frame = injected_rx_frames_.front();
        injected_rx_frames_.pop_front();
        return PCAN_ERROR_OK;
      }
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      break;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
    std::this_thread::sleep_for(std::min(std::chrono::microseconds(100), remaining));
  }

  {
    std::lock_guard<std::mutex> lock(simulator_mutex_);
    if (!injected_rx_frames_.empty()) {
      frame = injected_rx_frames_.front();
      injected_rx_frames_.pop_front();
      return PCAN_ERROR_OK;
    }
  }

  return PCAN_ERROR_QRCVEMPTY;
}

void CanTransport::set_tx_simulator(TxSimulator simulator, bool bypass_hardware)
{
  std::lock_guard<std::mutex> lock(simulator_mutex_);
  tx_simulator_ = std::move(simulator);
  simulator_bypass_hardware_ = bypass_hardware;
  injected_rx_frames_.clear();
}

void CanTransport::clear_tx_simulator()
{
  std::lock_guard<std::mutex> lock(simulator_mutex_);
  tx_simulator_ = {};
  simulator_bypass_hardware_ = false;
  injected_rx_frames_.clear();
}

bool CanTransport::has_tx_simulator() const
{
  std::lock_guard<std::mutex> lock(simulator_mutex_);
  return static_cast<bool>(tx_simulator_);
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

  TxSimulator simulator_copy;
  bool bypass_hardware = false;
  {
    std::lock_guard<std::mutex> sim_lock(simulator_mutex_);
    simulator_copy = tx_simulator_;
    bypass_hardware = simulator_bypass_hardware_;
  }

  const TPCANStatus status = bypass_hardware ? PCAN_ERROR_OK : channel_.write(frame);
  if (status == PCAN_ERROR_OK) {
    if (simulator_copy) {
      try {
        auto simulated_rx = simulator_copy(frame);
        if (!simulated_rx.empty()) {
          std::lock_guard<std::mutex> sim_lock(simulator_mutex_);
          for (const auto & rx_frame : simulated_rx) {
            injected_rx_frames_.push_back(rx_frame);
          }
        }
      } catch (const std::exception & e) {
        std::fprintf(
          stderr,
          "[CanTransport] TX simulator threw for CAN ID 0x%X: %s\n",
          frame.ID,
          e.what());
      } catch (...) {
        std::fprintf(
          stderr,
          "[CanTransport] TX simulator threw unknown for CAN ID 0x%X\n",
          frame.ID);
      }
    }
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
  {
    std::lock_guard<std::mutex> lock(simulator_mutex_);
    if (simulator_bypass_hardware_ && static_cast<bool>(tx_simulator_)) {
      return {};
    }
  }

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

void CanTransport::add_rx_observer(RxObserver observer)
{
  rx_observers_.push_back(std::move(observer));
}

void CanTransport::clear_rx_observers()
{
  rx_observers_.clear();
}

}  // namespace can_hardware_common
