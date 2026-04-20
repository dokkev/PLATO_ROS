# ADR-0001-can-hand-core-boundary.md

Date: 2026-04-17
Status: Accepted

## Context

- The repository supports multiple CAN-based hands with shared ROS controller semantics but different hardware protocol and transmission details.
- `can_hardware_common` already existed for transport-level reuse, but hand lifecycle orchestration and semantic runtime types were starting to duplicate across `plato_hardware_interface` and `aristo_hardware_interface`.
- Before continuing the refactor, the repository needed one explicit agreement on naming, ownership boundaries, and hot-path validation rules.

## Decision

- The shared semantic layer lives under `can_hardware_common/core/` for the first migration pass instead of creating a new `can_robot_core` package immediately.
- The common lifecycle vocabulary is:
  - `read()`
  - `write()`
  - `enable()`
  - `disable()`
  - `zero()`
- The common semantic types are:
  - `StateSnapshot`
  - `WritePlan`
  - `LifecyclePlan`
  - `CommandResult`
  - `CommandValidityReport`
- The architectural boundary is:
  - `can_hardware_common`: transport, scheduler, generic robot buffers, common lifecycle skeleton
  - `can_hardware_common/core`: shared hand runtime semantics and orchestration flow
  - robot packages: protocol, transmission, hardware mapping, calibration/zeroing policy, robot-specific state interpretation
  - ROS hardware interface files: thin ros2_control adapter only
  - bringup packages: launch and ROS wiring only
- Desired, computed, and applied command semantics remain distinct:
  - desired command: ROS/controller joint-space request currently stored in `RobotIO::joint_commands()`
  - computed command: per-cycle plan/build result represented by `WritePlan`
  - applied command: command history captured only after successful plan execution
- Structural validation moves to initialization time whenever possible:
  - config count
  - mapping size
  - CAN ID uniqueness
  - config completeness
  - buffer sizing
- Runtime hot paths should check only dynamic safety and availability:
  - finite values
  - stale RX
  - mode validity
  - command readiness

## Consequences

- Positive:
  - Top-level lifecycle flow now reads the same way for Plato and Aristo.
  - The ownership boundary between transport, protocol, and robot semantics is explicit.
  - Future robot-specific refactors can land under a stable naming and lifecycle contract.
  - The repository can defer a separate `can_robot_core` package until the boundary proves stable.
- Negative:
  - `can_hardware_common` now temporarily owns both low-level infrastructure and higher-level lifecycle semantics.
  - Compatibility wrappers such as `write_joint_commands()` still exist during migration and should be removed in a later cleanup pass.

## Alternatives Considered

- Alternative: add a new `can_robot_core` package immediately.
  Why rejected:
  - It would introduce packaging churn before the semantic boundary is fully exercised by both Plato and Aristo.

- Alternative: keep lifecycle logic fully robot-local and share only transport utilities.
  Why rejected:
  - The same orchestration pattern was already duplicating across both hand implementations.
