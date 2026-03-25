# Lab Coding Guideline: Make Robot Control Code Readable at a Glance

## Purpose

Our code should make the **main control behavior obvious at first sight**.

A reader who is unfamiliar with a specific subsystem should still be able to open a source file and quickly understand:

- where the system is initialized
- where inputs are read
- where the state is updated
- where commands are generated
- where outputs are sent or applied
- what high-level sequence happens in each cycle
- which parts describe the overall flow and which parts implement low-level detail

This guideline applies broadly across robotics software, including:

- robot hardware interfaces
- state estimation modules
- inverse kinematics
- operational space control
- impedance control
- whole-body control
- trajectory tracking
- communication layers
- safety, homing, and calibration logic

The goal is not just "clean code."
The goal is:

> **When reading a file for the first time, the reader should understand what happens before understanding how it happens.**

---

## Core Principle

### Show the flow first, hide the mechanics second

Top-level functions should read like an execution story.

Examples of high-level actions:

- update measurements
- refresh state snapshot
- estimate system state
- build task targets
- solve IK
- solve dynamics or control law
- validate command
- apply command
- process safety or calibration logic

A reader should be able to skim the top-level lifecycle functions and understand the system without opening every helper.

---

## Rule 1 — Top-level control functions must be short and narrative

Functions such as:

- `initialize()`
- `reset()`
- `read()`
- `update()`
- `compute()`
- `write()`
- `step()`

should stay short and primarily express **orchestration**.

### Good

```cpp
bool RobotController::step()
{
  if (!update_measurements_()) {
    return false;
  }

  refresh_state_snapshot_();
  process_mode_transitions_();

  const ControlPlan plan = build_control_plan_();
  if (!plan.ready) {
    handle_precondition_failure_(plan);
    return true;
  }

  return execute_control_plan_(plan);
}
```

### Bad

```cpp
bool RobotController::step()
{
  // sensor reads
  // frame parsing
  // filtering
  // estimation
  // task construction
  // Jacobian math
  // control solve
  // saturation
  // transport retries
  // error logging
  // calibration logic
  // safety branching
  // command application
}
```

### Guideline

Top-level control functions should usually be around 5–20 lines.

They should answer:

- What is happening?
- In what order?
- Under what high-level condition?

They should **not** answer:

- How the Jacobian is assembled
- How the optimizer is configured
- How a transport ack is matched
- How torques are packed
- How a specific estimator computes one term

Those belong below.

---

## Rule 2 — Separate orchestration from mechanism

Each function should primarily do one of the following:

**Orchestration** — Describes the sequence of actions.

Examples:

- `step()`
- `update()`
- `compute_control()`
- `process_mode_transitions_()`

**Mechanism** — Implements one detailed action.

Examples:

- `solve_inverse_kinematics_()`
- `compute_task_jacobian_()`
- `filter_joint_velocity_()`
- `dispatch_rx_frame_()`
- `apply_saturation_()`

### Guideline

If a function is deciding what happens next, it should not also contain large amounts of algorithmic or protocol detail.

If a function is implementing algorithmic or protocol detail, it should not also decide the full control flow.

---

## Rule 3 — Build plans before executing actions

For control and command paths, prefer this structure:

1. Build plan
2. Validate preconditions
3. Execute
4. Commit resulting state

This is clearer than interleaving preparation, validation, solving, and application.

### Recommended pattern

```cpp
bool RobotController::step()
{
  const ControlPlan plan = build_control_plan_();
  if (!plan.ready) {
    handle_precondition_failure_(plan);
    return true;
  }

  return execute_control_plan_(plan);
}
```

### Why this is better

A new reader can immediately understand:

- first we determine what we want to do
- then we check whether we are allowed to do it
- then we execute it

This is much easier to reason about than a large function that mixes:

- state checks
- estimation
- target generation
- solver calls
- logging
- safety branches
- application logic

---

## Rule 4 — Use domain-level names at the top

Top-level helpers should express intent.

**Prefer:**

- `update_measurements_()`
- `refresh_state_snapshot_()`
- `estimate_robot_state_()`
- `build_control_plan_()`
- `solve_ik_()`
- `execute_control_plan_()`
- `process_safety_()`
- `process_calibration_()`

**Avoid exposing low-level wording too early:**

- `poll_twice_then_accumulate_()`
- `recompute_cached_block_matrices_()`
- `pack_and_retry_transport_frame_()`

### Principle

At the top of the file, use names that match how a human explains the behavior aloud.

Example:

> "We update measurements, refresh the state, build the control plan, and execute it."

That is better than:

> "We update caches, rebuild matrices, evaluate preconditions, and invoke transport-dependent application logic."

---

## Rule 5 — Group code by reader importance, not by implementation chronology

Arrange source files so the reader sees the most important story first.

### Recommended order inside a `.cpp`

1. Construction / initialization
2. Public lifecycle functions
3. High-level private helpers
4. Low-level algorithmic helpers
5. Low-level transport or numerical helpers
6. Logging / printing / debug utilities

### Example order

```cpp
RobotController::RobotController(...)
RobotController::initialize(...)
RobotController::reset(...)
RobotController::step(...)
RobotController::read(...)
RobotController::write(...)

update_measurements_(...)
refresh_state_snapshot_(...)
estimate_robot_state_(...)
build_control_plan_(...)
handle_precondition_failure_(...)
execute_control_plan_(...)
process_mode_transitions_(...)
process_safety_(...)

solve_inverse_kinematics_(...)
compute_operational_space_target_(...)
compute_impedance_command_(...)
apply_command_limits_(...)

dispatch_rx_frame_(...)
pack_command_(...)
unpack_feedback_(...)

print_debug_state_(...)
```

### Why

The first screen of the file should reveal the system behavior, not bury it.

---

## Rule 6 — Hide repeated loops behind meaningful helpers when the loop has semantic meaning

A `for` loop is not bad.
But when the loop represents a meaningful operation, name that operation.

### Example

```cpp
void RobotController::update_task_targets_()
{
  for_each_task_([&](auto & task) {
    task.update_reference(robot_state_);
  });
}
```

This is clearer than repeating the same loop pattern inside multiple top-level functions.

### Guideline

Wrap loops when the loop means something like:

- update all tasks
- evaluate all constraints
- apply all filters
- gather all measurements
- validate all commands
- append all active subtasks

Do **not** hide loops just for the sake of abstraction.

The helper should clarify meaning, not obscure control flow.

---

## Rule 7 — Separate "desired," "computed," and "applied"

These are not the same thing.

Examples:

- **Desired** task target
- **Computed** solver output
- **Applied** command after safety checks and saturation

Never collapse these concepts into one state unless that simplification is explicitly safe.

### Good

- `build_control_plan_()` prepares desired references
- `solve_control_()` computes a candidate command
- `apply_command_limits_()` modifies it to respect constraints
- `commit_applied_command_()` stores what was actually sent or used

### Why

This reduces ambiguity and makes logs, debugging, and safety analysis much easier.

---

## Rule 8 — Use "snapshot" terminology for state refresh logic

State refresh code is often confusing because partial updates, stale values, estimator outputs, and transport timing get mixed together.

Use naming that makes the model explicit:

- `refresh_state_snapshot_()`
- `commit_state_snapshot_()`
- `estimate_robot_state_()`
- `update_measurement_cache_()`

### Why

The word *snapshot* tells the reader that:

- we are assembling a coherent state view
- the state may come from multiple sources
- we care about consistency, not just assignment

This is much clearer than scattered direct mutation across the file.

---

## Rule 9 — Precondition failure should have its own path

Do not bury failure handling inside the middle of the main control logic.

### Better

```cpp
if (!plan.ready) {
  handle_precondition_failure_(plan);
  return true;
}
```

### Why

This makes the main path easy to read:

- normal case flows downward
- exceptional cases exit early
- the body stays compact

Examples of precondition failures:

- non-finite input state
- missing measurements
- invalid mode
- stale state estimate
- solver prerequisites not satisfied
- calibration not completed
- safety gate active

---

## Rule 10 — Logging should not dominate control flow

If log formatting makes the main function hard to read, move it into a helper.

### Better

```cpp
log_plan_failure_if_needed_(plan);
```

### Worse

```cpp
if (!plan.ready) {
  RCLCPP_WARN_THROTTLE(
    logger(),
    clock_,
    1000,
    "Control plan invalid because state was stale and task target was not finite...");
  return true;
}
```

### Guideline

Top-level code should say *what happened*.
Detailed logging format should live below.

---

## Rule 11 — Make private types express the architecture

If a concept exists in the control flow, it often deserves a type.

Examples:

- `ControlPlan`
- `SolverInput`
- `TaskSet`
- `ConstraintSet`
- `StateSnapshot`
- `CommandResult`

### Good

```cpp
struct ControlPlan
{
  bool ready{false};
  bool stale_state{false};
  bool invalid_target{false};
  TaskSet tasks;
  ConstraintSet constraints;
  Command command;
};
```

### Why

This makes the structure visible even before reading function bodies.

---

## Rule 12 — One screen should explain the class

A reader should be able to understand the class behavior from the first screen or two of the `.cpp`.

When opening a control file, a reader should quickly learn:

- how the controller is updated
- where the state is refreshed
- where targets are built
- where control is solved
- where safety or calibration is handled
- where outputs are applied

If the first screen is dominated by matrix math, protocol detail, or logging, the structure is too hidden.

---

## Rule 13 — Keep solver calls visible, but keep solver internals below

In robotics code, the solver is often the conceptual center.

So top-level code should clearly show when the solve happens:

- `solve_inverse_kinematics_()`
- `solve_operational_space_control_()`
- `solve_impedance_command_()`
- `solve_whole_body_control_()`

But the math details should remain below.

### Good

```cpp
ControlCommand RobotController::compute_command_()
{
  const SolverInput input = build_solver_input_();
  const SolverOutput output = solve_whole_body_control_(input);
  return postprocess_solver_output_(output);
}
```

### Why

This preserves the conceptual structure without exposing all internals at the top.

---

## Rule 14 — Keep mode logic explicit

Robotics software often becomes unreadable because state machines, calibration states, safety modes, and runtime modes are buried inside unrelated code.

Prefer explicit mode-processing helpers:

- `process_mode_transitions_()`
- `process_safety_()`
- `process_calibration_()`
- `process_fault_recovery_()`

Avoid mixing these directly into task building or command application logic unless unavoidable.

---

## Rule 15 — Prefer a pipeline mental model

Most robot control modules can be understood as a pipeline:

1. collect inputs
2. refresh state
3. estimate or derive secondary state
4. build objectives
5. solve or compute
6. validate and limit
7. apply or publish outputs

Structure the code to reflect that pipeline.

### Example

```cpp
bool RobotController::step()
{
  if (!update_measurements_()) {
    return false;
  }

  refresh_state_snapshot_();
  estimate_robot_state_();

  const ControlPlan plan = build_control_plan_();
  if (!plan.ready) {
    handle_precondition_failure_(plan);
    return true;
  }

  return execute_control_plan_(plan);
}
```

This is readable because the code matches how humans reason about the control cycle.

---

## Recommended Design Pattern for Generic Robot Control Code

### Top-level lifecycle

```cpp
void RobotController::initialize()
{
  reset_runtime_state_();
  initialize_subsystems_();
}

bool RobotController::step()
{
  if (!update_measurements_()) {
    return false;
  }

  refresh_state_snapshot_();
  process_mode_transitions_();

  const ControlPlan plan = build_control_plan_();
  if (!plan.ready) {
    handle_precondition_failure_(plan);
    return true;
  }

  return execute_control_plan_(plan);
}
```

### High-level helpers

```cpp
void RobotController::reset_runtime_state_();
bool RobotController::update_measurements_();
void RobotController::refresh_state_snapshot_();
void RobotController::process_mode_transitions_();
ControlPlan RobotController::build_control_plan_();
void RobotController::handle_precondition_failure_(const ControlPlan & plan);
bool RobotController::execute_control_plan_(const ControlPlan & plan);
```

### Low-level helpers

```cpp
StateSnapshot RobotController::estimate_robot_state_();
SolverInput RobotController::build_solver_input_();
SolverOutput RobotController::solve_control_(const SolverInput & input);
Command RobotController::postprocess_solver_output_(const SolverOutput & output);
void RobotController::apply_command_limits_(Command & command);
void RobotController::publish_command_(const Command & command);
```

---

## Example Across Different Control Levels

### IK-style structure

```cpp
bool IKController::step()
{
  refresh_state_snapshot_();
  const IKPlan plan = build_ik_plan_();
  if (!plan.ready) {
    handle_precondition_failure_(plan);
    return true;
  }
  return execute_ik_plan_(plan);
}
```

### OSC-style structure

```cpp
bool OSCController::step()
{
  refresh_state_snapshot_();
  estimate_task_space_state_();

  const OSCPlan plan = build_osc_plan_();
  if (!plan.ready) {
    handle_precondition_failure_(plan);
    return true;
  }

  return execute_osc_plan_(plan);
}
```

### Impedance-style structure

```cpp
bool ImpedanceController::step()
{
  refresh_state_snapshot_();
  const ImpedancePlan plan = build_impedance_plan_();
  if (!plan.ready) {
    handle_precondition_failure_(plan);
    return true;
  }

  return execute_impedance_plan_(plan);
}
```

The exact algorithm changes, but the readable structure stays the same.

---

## Anti-Patterns

### 1. Giant lifecycle functions

Bad signs:

- `step()` is 150+ lines
- read/update/solve/apply are mixed together
- multiple unrelated concerns are interleaved

### 2. Helper names that describe implementation, not intent

- **Bad:** `rebuild_qp_terms_and_cached_blocks_()`
- **Better:** `build_solver_input_()`

### 3. Logging buried inside core logic

If the function reads like log formatting with occasional control logic, it is too noisy.

### 4. Hidden state semantics

If code silently treats:

- desired state
- computed state
- applied state
- stale state
- partial state

as equivalent, it becomes hard to reason about correctness.

### 5. Premature abstraction without semantic value

Not every repeated pattern needs a helper.
Helpers should exist because they clarify behavior, not because repetition is automatically bad.

### 6. Deep helper call chains

If reading a function requires jumping through 3+ levels of helpers to understand what happens, the decomposition has gone too far. Each hop forces the reader to hold context, open another location, and mentally reconstruct the flow.

Bad signs:

- `A()` calls `B()` calls `C()` calls `D()`, where B and C are 2–5 line wrappers
- A helper exists only to add a cooldown check or a single `if` before calling the real function
- You need to open 4 files or scroll 4 times to trace one logical operation

Guidelines:

- **Maximum 2 hops** from a lifecycle function to the actual work. `write() → build_write_plan_() → append_torque_commands_()` is fine. `write() → A() → B() → C() → D()` is not.
- **Inline 2–3 line wrappers** that only add a guard or a log. A cooldown check + function call is one block of logic, not two functions.
- **Ask: does this helper justify a separate name?** If the answer is "it just adds an `if`", inline it.
- Code navigation should flow downward, not sideways. A reader should be able to understand the full path by scrolling down, not by jumping between scattered small functions.

### 7. Solver internals dominating top-level structure

The reader should see *that* a solve happens, not every matrix term before understanding the control flow.

---

## Review Checklist

When reviewing robot control code, ask:

### High-level readability

- Can I understand `initialize`, `update`, `compute`, or `step` without reading low-level helpers?
- Do top-level functions read like a sequence of meaningful actions?

### Structural separation

- Are orchestration and mechanism clearly separated?
- Are preparation, validation, execution, and commit separated?

### Naming quality

- Do helper names explain intent?
- Would a non-expert understand what happens from the helper names alone?

### State semantics

- Are desired, computed, and applied quantities distinguished where needed?
- Is snapshot logic explicit and coherent?

### File organization

- Does the file show the main story first?
- Are low-level details pushed below the main flow?

### Maintainability

- If a new member joins the lab, can they follow the control flow in a few minutes?
- Can a reviewer quickly find the right layer of abstraction?

---

## Practical Rule of Thumb

If someone opens a robot control file and asks:

> "What happens in one control cycle?"

the answer should be visible almost immediately from the top-level code.

If they must inspect matrix assembly, transport details, or solver internals before understanding the control flow, the structure is wrong.
