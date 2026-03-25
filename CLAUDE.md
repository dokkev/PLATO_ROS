# CLAUDE.md

## Code Style

Follow the lab coding guideline in `hcrl_controller_code_guideline.md` at the repository root.

Key rules:

- Top-level lifecycle functions (`enable`, `disable`, `read`, `write`, `step`) must be **5–20 lines** of pure orchestration. No algorithmic or protocol detail.
- Separate orchestration from mechanism. If a function decides what happens next, it should not also contain low-level implementation.
- Use the **plan-build → validate → execute** pattern for command paths.
- Use domain-level names at the top (`refresh_state_snapshot_`, `build_control_plan_`), not implementation names (`poll_twice_then_accumulate_`).
- Group code by reader importance: lifecycle → high-level helpers → low-level helpers → logging.
- Logging should not dominate control flow — move formatting into helpers.
- Private types (`WritePlan`, `ControlPlan`) should express the architecture.
- Keep "desired," "computed," and "applied" state semantically distinct.

## Build

```bash
cd /home/dk/workspace/plato_ws
colcon build --packages-select can_hardware_common plato_hardware_interface
```

## Architecture

- `CanBusManager` owns all CAN transport logic (send, drain, ack-wait, diagnostics).
- `Hand` (plato_hand) is orchestration only — calls `send_frame`, `drain_rx`, `send_and_wait_ack`.
- `send_and_wait_ack()` is for startup/service operations (enable, disable, zeroing). Never use in the control loop.
- Control loop write path uses fire-and-forget `send_frame()`. Responses arrive via `drain_rx()` in the next `read()` cycle.
