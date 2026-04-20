#include "can_hardware_common/can_transport.hpp"

#include <algorithm>
#include <cstdio>
#include <exception>
#include <thread>

namespace can_hardware_common
{

CanTransport::~CanTransport() = default;

void CanTransport::set_tx_gap(std::chrono::microseconds tx_gap)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  tx_gap_ = tx_gap;
}

CanTransport::RxPollResult CanTransport::poll_rx()
{
  RxPollResult result;
  RxFrame rx_frame{};
  const auto dispatch_rx = [this](const RxFrame & frame) {
      for (auto & observer : rx_observers_) {
        try {
          observer(frame);
        } catch (const std::exception & error) {
          std::fprintf(stderr, "[CanTransport] RX observer threw for CAN ID 0x%X: %s\n", frame.ID, error.what());
        } catch (...) {
          std::fprintf(stderr, "[CanTransport] RX observer threw unknown for CAN ID 0x%X\n", frame.ID);
        }
      }
    };

  for (std::size_t index = 0; index < kMaxRxPerPoll; ++index) {
    {
      std::lock_guard<std::mutex> lock(simulator_mutex_);
      if (!injected_rx_frames_.empty()) {
        rx_frame = injected_rx_frames_.front();
        injected_rx_frames_.pop_front();
        dispatch_rx(rx_frame);
        ++result.processed_frames;
        continue;
      }
      if (simulator_bypass_hardware_ && static_cast<bool>(tx_simulator_)) {
        result.status = PCAN_ERROR_QRCVEMPTY;
        return result;
      }
    }

    const TPCANStatus status = channel_.read(rx_frame);
    if (status == PCAN_ERROR_OK) {
      dispatch_rx(rx_frame);
      ++result.processed_frames;
      continue;
    }

    result.status = status;
    return result;
  }

  result.status = PCAN_ERROR_OK;
  return result;
}

TPCANStatus CanTransport::read_frame(RxFrame & rx_frame, std::chrono::microseconds timeout)
{
  {
    std::lock_guard<std::mutex> lock(simulator_mutex_);
    if (!injected_rx_frames_.empty()) {
      rx_frame = injected_rx_frames_.front();
      injected_rx_frames_.pop_front();
      return PCAN_ERROR_OK;
    }
    if (!(simulator_bypass_hardware_ && static_cast<bool>(tx_simulator_))) {
      return channel_.read_with_timeout(rx_frame, timeout);
    }
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(simulator_mutex_);
      if (!injected_rx_frames_.empty()) {
        rx_frame = injected_rx_frames_.front();
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
      rx_frame = injected_rx_frames_.front();
      injected_rx_frames_.pop_front();
      return PCAN_ERROR_OK;
    }
  }

  return PCAN_ERROR_QRCVEMPTY;
}

void CanTransport::add_rx_observer(RxObserver observer)
{
  rx_observers_.push_back(std::move(observer));
}

void CanTransport::clear_rx_observers()
{
  rx_observers_.clear();
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

TPCANStatus CanTransport::send_tx_frame(const TxFrame & tx_frame)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);

  const auto now = std::chrono::steady_clock::now();
  if (last_tx_time_ != std::chrono::steady_clock::time_point{} &&
      tx_gap_.count() > 0 &&
      now - last_tx_time_ < tx_gap_)
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

  const TPCANStatus status = bypass_hardware ? PCAN_ERROR_OK : channel_.write(tx_frame);
  if (status == PCAN_ERROR_OK) {
    if (simulator_copy) {
      try {
        auto simulated_rx_frames = simulator_copy(tx_frame);
        if (!simulated_rx_frames.empty()) {
          std::lock_guard<std::mutex> sim_lock(simulator_mutex_);
          for (const auto & rx_frame : simulated_rx_frames) {
            injected_rx_frames_.push_back(rx_frame);
          }
        }
      } catch (const std::exception & error) {
        std::fprintf(stderr, "[CanTransport] TX simulator threw for CAN ID 0x%X: %s\n", tx_frame.ID, error.what());
      } catch (...) {
        std::fprintf(stderr, "[CanTransport] TX simulator threw unknown for CAN ID 0x%X\n", tx_frame.ID);
      }
    }
    last_tx_time_ = now;
  }

  return status;
}

bool CanTransport::can_send_tx_now(std::chrono::steady_clock::time_point now) const
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (last_tx_time_ == std::chrono::steady_clock::time_point{}) {
    return true;
  }
  return (now - last_tx_time_) >= tx_gap_;
}

std::chrono::steady_clock::time_point CanTransport::next_tx_time() const
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (last_tx_time_ == std::chrono::steady_clock::time_point{}) {
    return {};
  }
  return last_tx_time_ + tx_gap_;
}

CanTransport::BusDiagnostics CanTransport::get_diagnostics()
{
  {
    std::lock_guard<std::mutex> lock(simulator_mutex_);
    if (simulator_bypass_hardware_ && static_cast<bool>(tx_simulator_)) {
      return {};
    }
  }

  BusDiagnostics diagnostics;
  diagnostics.bus_status = channel_.get_bus_status();
  (void)channel_.get_value(
    PCAN_CHANNEL_CONDITION,
    &diagnostics.channel_condition,
    sizeof(diagnostics.channel_condition));
  (void)channel_.get_value(
    PCAN_RECEIVE_STATUS,
    &diagnostics.receive_status,
    sizeof(diagnostics.receive_status));
  return diagnostics;
}

std::string CanTransport::bus_status_string(TPCANStatus status)
{
  if (status == PCAN_ERROR_OK) {
    return "OK";
  }

  std::string result;
  auto append = [&](const char * label) {
      if (!result.empty()) {
        result += " | ";
      }
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
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "0x%X", status);
    result = buffer;
  }
  return result;
}

}  // namespace can_hardware_common
