# CAN Hardware Common Reset Plan — Transport-Centered Utility Layer

## Goal

Rebuild `can_hardware_common` as a small CAN bus toolbox.

The common package should own only code that would still make sense for an arbitrary CAN device:
- transport
- shared frame/time types
- very small utilities that are not hand-specific

The common package should **not** own:
- lifecycle execution engines
- confirmation frameworks
- runtime state frameworks
- generic hand base classes
- robot semantics
- protocol/model attachment contracts

## Target Shape

```text
can_hardware_common/
  include/can_hardware_common/
    pcan_interface.hpp
    can_bus.hpp
    can_frame_types.hpp
    time_types.hpp
    actuator_types.hpp   # keep only if still genuinely shared
    utils/
      can_helper.hpp       # keep only if still used by hand packages
  src/
    pcan_interface.cpp
    can_bus.cpp
```

## Common Layer Rules

Use this rule for every file:

> If this code would not be reused by an arbitrary CAN device, it does not belong in `can_hardware_common`.

Examples:
- `CanBus` belongs in common.
- A lifecycle runner does not belong in common.
- A hand runtime state machine does not belong in common.
- Confirmation predicates do not belong in common.

## Reset Checklist

### Phase 0 — Freeze hardware packages

- [x] keep `aristo_hardware_interface` frozen with `COLCON_IGNORE`
- [x] add `COLCON_IGNORE` to `plato_hardware_interface`

### Phase 1 — Shrink `can_hardware_common`

- [x] keep transport and shared type code
- [x] remove `core/` from the public/common design
- [x] remove `CanHandBase`
- [x] remove lifecycle session execution from common
- [x] remove control/lifecycle result frameworks from common
- [x] delete duplicate legacy transport sources from `src/`
- [x] flatten public headers and sources to top-level paths

### Phase 2 — Keep common minimal

- [x] keep transport pacing inside `CanBus`
- [x] remove now-unnecessary top-level include shims by making them the real headers
- [ ] revisit whether `actuator_types.hpp` is still genuinely common
- [ ] revisit whether transport-level TX simulator belongs in common long-term

### Phase 3 — Validation

- [x] `colcon build --packages-select can_hardware_common --symlink-install`
- [x] `colcon test --packages-select can_hardware_common --event-handlers console_direct+`
- [x] no C++ smoke executable remains under `src/`

## Acceptance Criteria

The reset is correct only if all of the following are true:

- [x] `can_hardware_common` is a transport-centered utility layer
- [x] no lifecycle engine remains in common
- [x] no generic hand base class remains in common
- [x] common no longer defines a runtime-state framework for hands
- [x] common still provides working PCAN bus I/O and pacing behavior
- [ ] Aristo/Plato are reattached later with hand-local `write()/enable()/disable()` procedures

## Next Step

Do **not** grow a new common framework immediately.

The next real implementation step should be:
1. choose one hand package
2. rebuild its `write()/enable()/disable()` as short local procedure code
3. depend only on `CanBus`, `PCANInterface`, and shared types from `can_hardware_common`
