#ifndef CAN_HARDWARE_COMMON__COMMAND_SCHEDULER_HPP_
#define CAN_HARDWARE_COMMON__COMMAND_SCHEDULER_HPP_

#include <PCANBasic.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>

#include "can_hardware_common/actuator.hpp"
#include "can_hardware_common/can_transport.hpp"

namespace can_hardware_common
{

class CanCommandScheduler
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  enum class State
  {
    kIdle,
    kPendingSend,
    kWaitingReply,
    kConfirmed,
    kRejected,
    kTimedOut,
    kTransportError,
  };

  struct EntryStatus
  {
    State state = State::kIdle;
    std::size_t send_attempts = 0;
    TPCANStatus last_transport_status = PCAN_ERROR_OK;
    uint8_t last_result_byte = 0xFF;
    TimePoint last_send_time{};
    TimePoint retry_after{};
  };

  struct Entry
  {
    CommandRequest request{};
    EntryStatus status{};
  };

  struct LastRx
  {
    uint32_t id = 0;
    uint8_t opcode = 0;
    uint8_t result_byte = 0xFF;
    TimePoint time{};
  };

  struct ServiceResult
  {
    std::size_t processed_rx_frames = 0;
    bool sent_frame = false;
    bool has_pending = false;
    bool transport_error = false;
    TPCANStatus transport_status = PCAN_ERROR_OK;
  };

  struct TransactionResult
  {
    enum class Status
    {
      kConfirmed,
      kRejected,
      kTimeout,
      kTransportError,
      kInvalidArgument,
    };

    Status status = Status::kTimeout;
    TPCANStatus transport_status = PCAN_ERROR_OK;
    uint8_t result_byte = 0xFF;

    static const char * status_label(Status s)
    {
      switch (s) {
        case Status::kConfirmed: return "confirmed";
        case Status::kRejected: return "rejected";
        case Status::kTimeout: return "timeout";
        case Status::kTransportError: return "transport_error";
        case Status::kInvalidArgument: return "invalid_argument";
      }
      return "unknown";
    }
  };

  explicit CanCommandScheduler(CanTransport & transport);

  // ── Streaming ──

  void submit_or_replace(const CommandRequest & request);
  void cancel(uint32_t key);
  void clear();

  ServiceResult service_once();
  ServiceResult service_until(TimePoint budget_end);

  std::optional<EntryStatus> status(uint32_t key) const;
  bool has_pending() const;

  // ── Blocking transaction ──

  TransactionResult execute_blocking(
    const CommandRequest & request,
    std::chrono::microseconds timeout,
    std::size_t max_retries = 0);

  // ── RX observation ──

  void observe_rx(const TPCANMsg & frame, TimePoint rx_time = Clock::now());

private:
  CanTransport & transport_;
  std::unordered_map<uint32_t, Entry> entries_;
  std::unordered_map<uint32_t, LastRx> last_rx_by_key_;
  uint32_t next_round_robin_key_ = 0;
};

}  // namespace can_hardware_common

#endif  // CAN_HARDWARE_COMMON__COMMAND_SCHEDULER_HPP_
