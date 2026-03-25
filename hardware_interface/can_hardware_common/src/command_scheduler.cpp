#include "can_hardware_common/command_scheduler.hpp"

#include <algorithm>
#include <thread>

namespace can_hardware_common
{

namespace
{

bool is_pending_state(CanCommandScheduler::State s)
{
  return (
    s == CanCommandScheduler::State::kPendingSend ||
    s == CanCommandScheduler::State::kWaitingReply);
}

bool reply_matches_frame(
  const TPCANMsg & frame,
  const ReplySpec & reply,
  uint8_t & result_byte)
{
  if (!reply.enabled) {
    return false;
  }

  if (frame.ID != reply.expected_rx_id) {
    return false;
  }

  if (frame.MSGTYPE != PCAN_MESSAGE_STANDARD) {
    return false;
  }

  if (frame.LEN < 2) {
    return false;
  }

  if (frame.DATA[0] != reply.expected_opcode) {
    return false;
  }

  result_byte = frame.DATA[1];
  return true;
}

bool entry_is_due(
  const CanCommandScheduler::Entry & entry,
  CanCommandScheduler::TimePoint now)
{
  switch (entry.status.state) {
    case CanCommandScheduler::State::kPendingSend:
      return true;

    case CanCommandScheduler::State::kWaitingReply:
      return now >= entry.status.retry_after;

    default:
      return false;
  }
}

}  // namespace

CanCommandScheduler::CanCommandScheduler(CanTransport & transport)
: transport_(transport)
{
}

void CanCommandScheduler::submit_or_replace(const CommandRequest & request)
{
  const auto existing = entries_.find(request.key);
  if (
    existing != entries_.end() &&
    !request.retry.latest_only &&
    is_pending_state(existing->second.status.state))
  {
    return;
  }

  auto & entry = entries_[request.key];
  entry.request = request;

  // latest_only=true: replace pending/waiting state with the newest request.
  // latest_only=false: pending entries are preserved (handled by the early return above).
  entry.status.state = State::kPendingSend;
  entry.status.send_attempts = 0;
  entry.status.last_transport_status = PCAN_ERROR_OK;
  entry.status.last_result_byte = 0xFF;
  entry.status.last_send_time = TimePoint{};
  entry.status.retry_after = TimePoint{};
}

void CanCommandScheduler::cancel(uint32_t key)
{
  entries_.erase(key);
  last_rx_by_key_.erase(key);
}

void CanCommandScheduler::clear()
{
  entries_.clear();
  last_rx_by_key_.clear();
  next_round_robin_key_ = 0;
}

void CanCommandScheduler::observe_rx(const TPCANMsg & frame, TimePoint rx_time)
{
  for (auto & [key, entry] : entries_) {
    auto & status = entry.status;
    const auto & request = entry.request;

    if (status.state != State::kWaitingReply) {
      continue;
    }

    uint8_t result_byte = 0xFF;
    if (!reply_matches_frame(frame, request.reply, result_byte)) {
      continue;
    }

    last_rx_by_key_[key] = LastRx{
      frame.ID,
      frame.DATA[0],
      result_byte,
      rx_time};

    status.last_result_byte = result_byte;

    if (result_byte == request.reply.success_byte) {
      status.state = State::kConfirmed;
    } else {
      status.state = State::kRejected;
    }
  }
}

std::optional<CanCommandScheduler::EntryStatus>
CanCommandScheduler::status(uint32_t key) const
{
  const auto it = entries_.find(key);
  if (it == entries_.end()) {
    return std::nullopt;
  }
  return it->second.status;
}

bool CanCommandScheduler::has_pending() const
{
  for (const auto & [key, entry] : entries_) {
    (void)key;
    if (is_pending_state(entry.status.state)) {
      return true;
    }
  }
  return false;
}

CanCommandScheduler::ServiceResult CanCommandScheduler::service_once()
{
  ServiceResult result{};

  // 1) RX 먼저 처리
  const auto rx_result = transport_.process_rx();
  result.processed_rx_frames = rx_result.processed_frames;

  if (rx_result.is_bus_error()) {
    result.transport_error = true;
    result.transport_status = rx_result.status;
    result.has_pending = has_pending();
    return result;
  }

  // 2) 아직 송신 가능 시점이 아니면 여기서 종료
  const auto now = Clock::now();
  if (now < transport_.next_send_time()) {
    result.has_pending = has_pending();
    return result;
  }

  if (entries_.empty()) {
    result.has_pending = false;
    return result;
  }

  // 3) round-robin으로 due entry 하나 선택
  std::optional<uint32_t> chosen_key;

  uint32_t first_after_cursor = 0;
  bool found_after_cursor = false;

  uint32_t first_before_cursor = 0;
  bool found_before_cursor = false;

  for (const auto & [key, entry] : entries_) {
    if (!entry_is_due(entry, now)) {
      continue;
    }

    if (key >= next_round_robin_key_) {
      if (!found_after_cursor || key < first_after_cursor) {
        first_after_cursor = key;
        found_after_cursor = true;
      }
    } else {
      if (!found_before_cursor || key < first_before_cursor) {
        first_before_cursor = key;
        found_before_cursor = true;
      }
    }
  }

  if (found_after_cursor) {
    chosen_key = first_after_cursor;
  } else if (found_before_cursor) {
    chosen_key = first_before_cursor;
  }

  if (!chosen_key.has_value()) {
    result.has_pending = has_pending();
    return result;
  }

  // 4) 딱 한 frame만 dispatch
  auto & entry = entries_.at(*chosen_key);
  auto & status = entry.status;
  const auto & request = entry.request;

  if (request.retry.max_attempts > 0 && status.send_attempts >= request.retry.max_attempts) {
    status.state = State::kTimedOut;
    result.has_pending = has_pending();
    return result;
  }

  const TPCANStatus tx_status = transport_.send_if_ready(request.frame);

  if (tx_status == PCAN_ERROR_OK) {
    result.sent_frame = true;

    status.send_attempts += 1;
    status.last_transport_status = PCAN_ERROR_OK;
    status.last_send_time = now;
    status.retry_after = now + request.retry.holdoff;

    if (request.reply.enabled) {
      status.state = State::kWaitingReply;
    } else {
      status.state = State::kConfirmed;
    }

    next_round_robin_key_ = *chosen_key + 1;
  } else if (tx_status == PCAN_ERROR_QXMTFULL) {
    // Bus not ready yet: transport error 아님
    status.last_transport_status = tx_status;
  } else {
    // 실제 transport error
    status.last_transport_status = tx_status;
    status.state = State::kTransportError;
    result.transport_error = true;
    result.transport_status = tx_status;
  }

  result.has_pending = has_pending();
  return result;
}

CanCommandScheduler::ServiceResult
CanCommandScheduler::service_until(TimePoint budget_end)
{
  ServiceResult aggregate{};

  while (Clock::now() < budget_end) {
    const auto step = service_once();

    aggregate.processed_rx_frames += step.processed_rx_frames;
    aggregate.sent_frame = aggregate.sent_frame || step.sent_frame;
    aggregate.transport_error = aggregate.transport_error || step.transport_error;

    if (step.transport_error) {
      aggregate.transport_status = step.transport_status;
    }

    aggregate.has_pending = step.has_pending;

    if (step.transport_error) {
      break;
    }

    if (!step.has_pending) {
      break;
    }

    // No progress: check if anything can happen before budget ends.
    if (step.processed_rx_frames == 0 && !step.sent_frame) {
      auto earliest = transport_.next_send_time();
      for (const auto & [key, entry] : entries_) {
        (void)key;
        if (entry.status.state == State::kWaitingReply &&
          entry.status.retry_after < earliest)
        {
          earliest = entry.status.retry_after;
        }
      }
      if (earliest >= budget_end) {
        break;
      }
    }
  }

  return aggregate;
}

CanCommandScheduler::TransactionResult
CanCommandScheduler::execute_blocking(
  const CommandRequest & request,
  std::chrono::microseconds timeout,
  std::size_t max_retries)
{
  TransactionResult out{};

  if (!request.reply.enabled || timeout <= std::chrono::microseconds::zero()) {
    out.status = TransactionResult::Status::kInvalidArgument;
    return out;
  }

  std::size_t attempts = 0;
  while (true) {
    const auto send_deadline = Clock::now() + timeout;
    while (true) {
      const TPCANStatus tx_status = transport_.send_if_ready(request.frame);

      if (tx_status == PCAN_ERROR_OK) {
        break;
      }

      if (tx_status != PCAN_ERROR_QXMTFULL) {
        out.status = TransactionResult::Status::kTransportError;
        out.transport_status = tx_status;
        return out;
      }

      const auto now = Clock::now();
      if (now >= send_deadline) {
        out.status = TransactionResult::Status::kTimeout;
        return out;
      }

      const auto next_send_time = transport_.next_send_time();
      if (next_send_time > now) {
        std::this_thread::sleep_until(std::min(next_send_time, send_deadline));
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
    }

    const auto start = Clock::now();
    while (Clock::now() - start < timeout) {
      TPCANMsg frame{};
      const auto elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start);
      if (elapsed >= timeout) {
        break;
      }
      const auto remaining = timeout - elapsed;

      const TPCANStatus rx_status = transport_.read_frame(frame, remaining);

      if (rx_status == PCAN_ERROR_QRCVEMPTY) {
        // Keep waiting until timeout expires. Some PCAN backends can report an
        // empty queue before the full timeout elapses (e.g. event wakeups
        // without a readable frame yet).
        continue;
      }

      if (rx_status != PCAN_ERROR_OK) {
        out.status = TransactionResult::Status::kTransportError;
        out.transport_status = rx_status;
        return out;
      }

      // Fan out to observers so actuator state parsers see all frames.
      for (const auto & observer : transport_.rx_observers()) {
        observer(frame);
      }

      uint8_t result_byte = 0xFF;
      if (!reply_matches_frame(frame, request.reply, result_byte)) {
        continue;
      }

      out.result_byte = result_byte;

      if (result_byte == request.reply.success_byte) {
        out.status = TransactionResult::Status::kConfirmed;
      } else {
        out.status = TransactionResult::Status::kRejected;
      }

      return out;
    }

    if (attempts >= max_retries) {
      out.status = TransactionResult::Status::kTimeout;
      return out;
    }

    ++attempts;
  }
}

}  // namespace can_hardware_common
