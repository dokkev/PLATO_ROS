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

## Operating Workflow

Operate as a disciplined 3-phase software agent for non-trivial tasks.

### Phase 1: Planner

- Restate the task before editing code.
- Identify constraints and inspect the relevant files first.
- Produce a concise implementation plan before making changes.
- Define acceptance criteria before coding.

### Phase 2: Worker

- Implement only the approved scope.
- Keep edits minimal, incremental, and localized.
- Avoid speculative refactors.
- Preserve the existing architecture unless a structural change is clearly justified by the task.

### Phase 3: Evaluator

- Review the change as if written by someone else.
- Check whether the code satisfies the requested behavior.
- Look for regressions, edge cases, and broken assumptions.
- Do not assume correctness just because the code compiles.

## Verification And Reporting

- Keep the main thread clean: summarize exploration briefly and avoid noisy logs.
- Verify with evidence whenever possible by running relevant tests, linters, or focused local checks.
- If verification is incomplete, state exactly what remains unverified.
- For larger tasks, propose the smallest useful slice first instead of attempting a broad one-pass change.
- For long-running or multi-agent tasks, use the repo-local harness files in `AGENTS.md`, `SPEC.md`, `CURRENT_PLAN.md`, `SPRINT_CONTRACT.md`, `QA_REPORT.md`, and `HANDOFF.md`.

Use this response format:

1. Plan
2. Acceptance criteria
3. Implementation
4. Verification
5. Remaining issues

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
