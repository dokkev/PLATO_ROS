# CAN Hand Protocol/Model Report

## Purpose

This report captures the hard boundary decisions that remain after introducing:

- `PlatoProtocol` / `PlatoModel`
- `AristoProtocol` / `AristoModel`

The immediate goal was to thin `plato_hand.cpp` and `aristo_hand.cpp` without breaking runtime behavior.
That part is working.
The remaining items are the parts that are still structurally ambiguous and worth deciding together before pushing the split further.

## What moved cleanly

### Plato
- lifecycle plan construction moved to `PlatoProtocol`
- stream frame construction moved to `PlatoProtocol`
- transmission-based joint<->actuator mapping moved to `PlatoModel`
- fallback-feedback joint-state update moved to `PlatoModel`

### Aristo
- lifecycle frame construction moved to `AristoProtocol`
- impedance frame construction moved to `AristoProtocol`
- joint-state assembly moved to `AristoModel`
- PIP relative-command mapping moved to `AristoModel`

## Hard parts still left

### 1. RX decode ownership is still split awkwardly

Current state:
- Plato actuator RX decode still lives in `can_protocol::SteadywinProtocol::process_message()`
- Aristo actuator RX decode still lives in `mit_can_protocol::MITProtocol::process_message()`
- the hand still owns transport observers and dispatch wiring
- Aristo still owns the RX dispatch table and FT-sensor observer routing

Why this is hard:
- frame-level decode is protocol work
- dispatch-table ownership feels protocol-ish
- observer registration and mutex discipline feel hand/core-ish
- actuator state mutation currently happens through actuator-owned protocol objects

Decision to make:
- Do we want the next step to create a hand-level RX adapter inside `*Protocol`?
- Or do we treat actuator-local decode as an acceptable lower-level protocol layer and leave observer wiring in `Hand`?

My recommendation:
- keep actuator-local decode for now
- move only RX dispatch-table construction/routing next
- avoid a second abstraction layer around every individual actuator protocol unless we need it

### 2. Lifecycle execution policy is not a pure protocol concern

Current state:
- `CanHandBase` owns lifecycle flow
- `*Protocol` now builds lifecycle plans
- `PlatoHand` still owns scheduled execution and thumb-servo tolerance policy
- `AristoHand` still owns direct-frame lifecycle execution

Why this is hard:
- protocol should know how to encode lifecycle frames/requests
- but execution semantics depend on scheduler/direct-TX policy, retries, timeout, and tolerated failure rules
- those rules are not just codec semantics

Decision to make:
- Should lifecycle execution policy stay in `Hand`?
- Should it move upward into the core as a shared execution strategy?
- Or do we want a separate execution-policy object later?

My recommendation:
- do not move this into protocol
- either keep it in `Hand` temporarily or move it into a future core execution-policy layer
- especially for Plato, thumb-servo tolerance is a runtime policy, not a frame codec rule

### 3. Plato zeroing crosses model/protocol/runtime boundaries

Current state:
- zeroing probe sends torque requests
- zero capture mutates actuator offsets
- persistence writes YAML
- joint-state refresh depends on model semantics

Why this is hard:
- probing looks like runtime/lifecycle orchestration
- torque request creation looks protocol-ish
- offset capture/update is model/calibration semantics
- YAML persistence is neither protocol nor model

Decision to make:
- Should zeroing remain owned by `PlatoHand` as an integration workflow?
- Or should we split it into a dedicated calibration component?

My recommendation:
- keep zeroing in `PlatoHand` for now
- if it grows again, extract a `PlatoCalibration` or `PlatoZeroingSession`
- do not force it into `Protocol` or `Model`

### 4. Plato simulator also crosses the boundary

Current state:
- simulator TX interception is transport-adjacent
- fake reply generation is protocol-shaped
- fake motor state evolution is model-shaped

Why this is hard:
- simulator is naturally hybrid
- over-splitting it too early would create more plumbing than value

My recommendation:
- keep simulator local to `PlatoHand` for now
- only split it if we want reusable simulator tests across hands

### 5. We still do not have common abstract `CanHandProtocol` / `CanHandModel`

Current state:
- we now have concrete `PlatoProtocol`, `PlatoModel`, `AristoProtocol`, `AristoModel`
- but no common abstract interface in `can_hardware_common`

Why this is hard:
- the current hand implementations do not need dynamic polymorphism here
- adding abstract interfaces right now may just add ceremony
- but the plan document names a shared `CanHandProtocol` / `CanHandModel`

Decision to make:
- Do we actually need common abstract base types now?
- Or is “same conceptual role, concrete per-hand class” enough?

My recommendation:
- defer common abstract interfaces until we need shared generic composition
- keep the conceptual split first
- avoid interface-first design if no code is consuming the abstraction yet

## Suggested next refactor slices

### Option A: safer next step
- move Aristo RX dispatch-table build/routing into `AristoProtocol`
- move Plato RX observer actuator-routing into `PlatoProtocol`
- leave lifecycle execution, zeroing, and simulator where they are

### Option B: more ambitious next step
- create a small core-owned execution-policy helper for lifecycle/direct-TX/scheduler execution
- reduce duplicated `execute_write_plan_()` / `execute_lifecycle_plan_()` patterns
- keep protocol/model classes as pure builders/mappers

## Recommendation

If we want the cleanest progress with the least risk, I recommend:

1. finish RX routing split next
2. do not split zeroing yet
3. do not add common abstract protocol/model interfaces yet
4. revisit lifecycle execution policy after RX routing is cleaner
