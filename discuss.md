# Discuss

## Latest Report

### `can_hardware_common` was reduced again

What changed:
- `plato_hardware_interface` stays frozen behind `COLCON_IGNORE`
- `aristo_hardware_interface` stays frozen behind its existing `COLCON_IGNORE`
- `can_hardware_common` was reduced to a transport-centered utility layer
- the common `core/` framework was removed
- `CanHandBase` was deleted
- lifecycle execution no longer lives in common
- duplicate legacy transport sources were deleted
- public headers and sources were flattened to top-level paths
- the C++ `pcan_transport_smoke_test` source/target was removed

What remains in common:
- `CanBus`
- `PCANInterface`
- shared frame/time/actuator types
- existing utility headers that are still referenced by frozen hand packages

Validation completed:
- `colcon build --packages-select can_hardware_common --symlink-install`
- `colcon test --packages-select can_hardware_common --event-handlers console_direct+`

## Current Hard Parts

### 1. `CanBus` is now transport-only

This is now intentionally narrow.

What it owns:
- PCAN-backed CAN frame send/receive
- TX pacing
- RX observer dispatch
- bounded RX collection helpers

Working recommendation:
- keep it small
- keep response semantics and loop policy in the hand layer

### 2. `actuator_types.hpp` may still be too semantic for common

`ActuatorTarget`, `ActuatorState`, and `ActuatorStatus` are more than pure bus primitives.
That may still be acceptable temporarily, but it is not obviously transport-generic.

Working recommendation:
- leave it in place while hand packages are frozen
- trim it later if reattachment shows these types are really hand-specific

### 3. Reattachment must stay hand-local

The reset only helps if Aristo/Plato do not immediately recreate a new mini framework on top.

Working recommendation:
- implement hand-local procedure code
- keep `write()`, `enable()`, and `disable()` short and direct
- use common only for bus I/O and shared low-level types
