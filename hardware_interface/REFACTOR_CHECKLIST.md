# CAN Hardware Interface Refactoring Checklist

## Architecture Principles

- [x] PCANInterface: raw driver wrapper only
- [x] CanBusManager → CanBus: transport/pacing/RX dispatch only
- [x] send_and_wait() moved out of transport into scheduler
- [x] Hand: joint↔actuator mapping + request generation only
- [x] Streaming write: latest-only semantics confirmed

## Semantics

- [x] QXMTFULL = not-ready, not error
- [x] Rejected reply = terminal state
- [x] Blocking (enable/disable/zeroing) vs streaming (torque/servo) separated

---

## 1. New Types

- [x] ReplySpec (enabled, expected_rx_id, expected_opcode, success_byte)
- [x] RetryPolicy (holdoff, max_attempts, latest_only)
- [x] CommandRequest (key, frame, ReplySpec, RetryPolicy)
- [x] TxCommand.post_send_delay removed (TxCommand is now frame+opcode only)

## 2. PCANInterface

- [x] Decision: keep name PCANInterface (renaming breaks ft_sensor without adding value)
- [x] No pacing, handler, retry, scheduler state in this layer
- [x] Constructor: hardcoded (PCAN_USBBUS1, 1M) — config injection deferred
- [x] read_with_timeout() semantics documented in header comment
- [x] Thread-safety: io_mutex_ protects all I/O, ppoll outside lock is safe

## 3. CanBusManager → CanBus

- [x] Renamed to CanBus (new file can_bus.hpp/cpp)
- [x] send_and_wait() removed from transport
- [x] Single RxHandler → multi-observer (add_rx_observer / clear_rx_observers)
- [x] process_rx() max frame cap retained (kMaxRxPerProcess = 30)
- [x] send_if_ready() + next_send_time() retained
- [x] write_paced_() removed (inlined into send_if_ready)
- [x] Diagnostics retained (get_diagnostics, bus_status_string)
- [x] read_frame(timeout) retained for blocking path

## 4. CanCommandScheduler

### API
- [x] submit_or_replace(CommandRequest)
- [x] cancel(key)
- [x] clear()
- [x] observe_rx(frame, time)
- [x] service_once() → max 1 frame
- [x] service_until(budget_end)
- [x] execute_blocking(request, timeout, max_retries)
- [x] status(key) → optional<EntryStatus>
- [x] has_pending()

### State Machine
- [x] kIdle → kPendingSend → kWaitingReply → kConfirmed/kRejected/kTransportError
- [x] Rejected = terminal, no retry
- [x] QXMTFULL = stays in current state

### Fairness
- [x] Round-robin cursor as member (persists across ticks)
- [x] Works with non-contiguous key space (unordered_map)

### RX
- [x] Scheduler is observer, not handler owner
- [x] observe_rx() only matches kWaitingReply entries
- [x] O(N) scan for first version

### Blocking Transaction
- [x] Replaces old send_and_wait()
- [x] Uses same reply_matches() helper
- [x] Pacing not-ready vs real transport error separated

## 5. Actuator API

- [x] Return CommandRequest (make_enable_request, make_disable_request, etc.)
- [x] Request includes: frame, expected_rx_id, expected_opcode, success_byte, key
- [x] Default retry holdoff in request (via RetryPolicy defaults)
- [x] Clamp/protocol encoding stays in Actuator
- [x] process_message() / state bookkeeping stays

## 6. Hand Simplification

### Members Removed
- [x] pending_commands_
- [x] last_rx_
- [x] next_send_index_

### Members Added
- [x] CanCommandScheduler

### write_joint_commands()
- [x] Phase 1: compute + submit requests to scheduler
- [x] Phase 2: scheduler.service_until(budget_end)
- [x] Phase 3: capture_*commands()

### enable/disable/zero
- [x] Use scheduler.execute_blocking()
- [x] No concurrent streaming during blocking commands (single-threaded ros2_control)

### on_rx_frame_()
- [x] Replaced by transport RX observers (two lambdas in constructor)
- [x] Observer 1: actuator state update under state_mutex_
- [x] Observer 2: scheduler.observe_rx()
- [x] last_rx_ removed

## 7. Synchronization

- [x] Hand state_mutex_ separate from scheduler (scheduler has no mutex)
- [x] observe_rx() and service() race-free (single-threaded ros2_control loop)
- [x] Lock ordering: state_mutex_ only held briefly, never during transport I/O
- [x] Blocking transaction: read_frame() does not invoke handlers (no deadlock)

## 8. Migration Steps

- [x] Step 1: New types (ReplySpec, RetryPolicy, CommandRequest)
- [x] Step 2: CanCommandScheduler with execute_blocking + streaming service
- [x] Step 3: Move streaming scheduler from Hand → CanCommandScheduler
- [x] Step 4: Hand slim down (no pending_, last_rx_, next_send_index_)
- [x] Step 5: CanBus created (no send_and_wait, multi-observer)
- [x] Step 6: PCANInterface review (kept as-is, already raw-only)
- [x] Step 7: Old can_bus_manager.hpp/cpp deleted
- [ ] Step 8: Update profiler node to new API (deferred — commented out of build)

## 9. Cleanup

- [x] Old send_and_wait() removed (file deleted)
- [x] TxCommand.post_send_delay removed
- [x] Old names purged (ack, slot, inflight, drain in core files)
- [x] Stale comments removed

## 10. Final Acceptance

- [x] PCANInterface knows raw driver only
- [x] Transport knows frame transport only
- [x] Reply/retry/session logic in scheduler only
- [x] Hand knows mapping/request only
- [x] No actuator starvation (round-robin cursor persists across ticks)
- [x] QXMTFULL doesn't flood logs (treated as not-ready, no log)
- [x] Reject/timeout/transport_error separately observable (State enum)
- [x] Builds clean (4 packages, no errors)
