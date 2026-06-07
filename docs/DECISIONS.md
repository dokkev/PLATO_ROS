# Decisions

Newest decisions go first. Record only stable choices that future work should
preserve or intentionally replace.

---

## 2026-06-05 - MPPI Predicts Modular Tactile GraspState

Status: Accepted

### Context

`mppi_core` needs a rollout target that keeps robot reference state and all
tactile sensors visible. A single tactile field made it too easy to ignore one
sensor, while fixed first/second tactile fields made the rollout unnecessarily
specific to one hand configuration.

### Decision

Use `GraspState = RobotState + vector<TactileState>` as the top-level rollout
state. Keep `TactileState` sensor-specific and compact: sensor-level aggregates
plus a dense full hemisphere list. Carry one `TactileSensorContext` per tactile
state. Use `RobotSystem` as the robot model/data holder for Pinocchio
integrate/RNEA.

### Reason

The current hand setup may have two tactile sensors, but rollout, cost, adapter,
and tests should not bake that count into production state fields. A vector
keeps the current paired-sensor setup representable while leaving room for
additional tactile sensors.

### Consequences

Positive:

- Validity checks must consider every tactile sensor.
- Inactive hemispheres remain representable, so contact birth can be modeled
  later without changing `TactileState` shape.
- MPPI horizon rollout does not use measured-torque residual force projection.
  Residual/contact-force projection helpers remain separate utilities for later
  observation-time correction experiments.
- The `/grasp` include directory no longer acts as a catch-all for robot,
  tactile, contact, and rollout code.

Trade-offs:

- Runtime integration must construct and cache one `RobotSystem` for the robot
  model plus one contact kinematics context per tactile sensor frame.
- Old include compatibility headers and aliases are removed; callers must use
  the current responsibility directories directly.

### Related files

- `mppi_core/include/mppi_core/state/grasp_state.hpp`
- `mppi_core/include/mppi_core/rollout/grasp_state_rollout_model.hpp`
- `mppi_core/include/mppi_core/tactile/tactile_state.hpp`
- `mppi_core/docs/grasp_state_mppi.md`

---

## 2026-06-05 - MPPI Action Is Solver Joint Acceleration

Status: Accepted

### Context

The contact-local MPPI core previously sampled joint reference increments and
used a simple torque proxy during rollout. The intended runtime now sends hybrid
impedance commands to an embedded 1 kHz controller.

### Decision

Solve/sample `qddot_sol` in `mppi_core`, integrate ideal `qdot` and `q` during
rollout, compute `tau` with Pinocchio RNEA when model/data are available, and
fall back to deterministic zero torque otherwise. Keep embedded PD feedback out
of the rollout model. `qddot_sol` remains the action and is not stored in
`RobotState` or `RobotCommand`.

### Reason

This matches the command actually consumed by the low-level controller while
keeping MPPI prediction deterministic, ROS-free, and testable.

### Consequences

Positive:

- Clear separation between measured state, rollout reference state, and final
  command packet.
- Explicit acceleration units for action bounds and noise.
- No fake rollout torque stiffness/damping proxy in production behavior.

Trade-offs:

- ROS adapters must populate measured fields and command gains explicitly.
- Older delta-q callers need to migrate to `GraspStateRolloutModel` and
  `qddot_sol` actions directly.

### Related files

- `mppi_core/include/mppi_core/rollout/grasp_state_rollout_model.hpp`
- `mppi_core/include/mppi_core/robot/robot_system.hpp`
- `mppi_core/config/mppi.yaml`
- `mppi_core/MPPI.md`

---

## 2026-06-05 - Keep AGENTS.md As A Map

Status: Accepted

### Context

The repository had a short `AGENTS.md` plus several template harness docs.
The OpenAI harness engineering guidance favors a small entrypoint that points to
structured, versioned repository knowledge instead of one large instruction
blob.

### Decision

Keep `AGENTS.md` short and use it as a table of contents. Put architecture,
commands, decisions, plans, and runbooks under `docs/`.

### Reason

Agents need a fast entrypoint and durable sources of truth. Large instruction
files drift quickly and consume context that should be spent on the task and
nearby code.

### Consequences

Positive:

- Easier for humans and agents to find the right context.
- Lower chance of stale prose being treated as a hard rule.
- More room to keep detailed runbooks near their domain.

Trade-offs:

- Docs must stay linked and current.
- New project rules need a clear home instead of being appended to `AGENTS.md`.

### Related files

- `AGENTS.md`
- `docs/ARCHITECTURE.md`
- `docs/COMMANDS.md`
- `docs/PLANS.md`

---

## 2026-06-05 - Generic Coding Guidance Lives Outside The Repo

Status: Accepted

### Context

The old root-level lab controller guideline duplicated general robot-control
readability guidance now covered by the local `code-implementer` skill and its
robot-control readability reference.

### Decision

Remove the generic lab guideline from this repository. Keep repository docs
focused on PLATO-specific architecture, commands, decisions, and runbooks.

### Reason

Generic style guidance is easier to maintain once in a skill. Repo docs should
avoid repeating broad rules that are not project-specific.

### Consequences

Positive:

- Less duplicated guidance.
- Smaller repository knowledge surface.
- Future code work can use the shared implementation skill while this repo
  records PLATO-specific contracts.

Trade-offs:

- Agents without that skill need to infer general coding style from local code
  and the remaining harness.

### Related files

- `docs/ARCHITECTURE.md`

---

## 2026-06-05 - Controllers Stay Joint-Space

Status: Accepted

### Context

PLATO and Aristo share `joint_impedance_controller`, but their actuator
protocols and transmission behavior differ.

### Decision

Controllers should consume and produce joint-space data only. Robot-specific CAN
protocols, actuator IDs, zeroing, limits, and transmission math belong below the
ros2_control hardware interface boundary.

### Reason

The shared impedance controller remains reusable across PLATO and Aristo when
the hardware layer absorbs robot-specific details.

### Consequences

Positive:

- One controller can serve multiple hand implementations.
- Hardware-specific risks stay near hardware config and protocol code.
- Controller tests can focus on command validation and impedance semantics.

Trade-offs:

- Hardware plugins must maintain the full joint-space interface contract.
- Some robot-specific behavior must be selected through bringup YAML or hardware
  parameters rather than controller branches.

### Related files

- `controllers/joint_impedance_controller/`
- `hardware_interface/plato_hardware_interface/`
- `hardware_interface/aristo_hardware_interface/`
- `bringup/plato_bringup/config/`
- `bringup/aristo_bringup/config/`

---

## 2026-06-05 - Hardware Config Belongs With Hardware Packages

Status: Accepted

### Context

Controller YAML is ROS-facing runtime configuration, while actuator constants,
offsets, linkage geometry, and CAN behavior are package-owned hardware
configuration.

### Decision

Keep fixed hardware configuration under the relevant hardware package. Keep
controller manager and controller parameter YAML under bringup packages.

### Reason

Hardware packages own parsing, validation, and lifecycle behavior. Bringup owns
ROS wiring and launch-time selection.

### Consequences

Positive:

- Clearer ownership of fixed hardware constants.
- Fewer ROS parameters in the control loop.
- Easier to validate config loading near hardware code.

Trade-offs:

- Calibration files such as actuator offsets still need careful update flows.
- Launch files may need explicit override arguments for local hardware setups.

### Related files

- `hardware_interface/plato_hardware_interface/config/`
- `hardware_interface/aristo_hardware_interface/config/`
- `bringup/plato_bringup/config/`
- `bringup/aristo_bringup/config/`

---

## 2026-06-05 - MPPI Core Remains ROS-Free

Status: Accepted

### Context

`mppi_core` is reusable C++ logic for contact-local MPPI rollout, costs, tactile
state, and command representation.

### Decision

Keep `mppi_core` independent from ROS nodes and ROS message types. Put ROS
adapters, launch wiring, and topic contracts outside the package.

### Reason

The MPPI optimizer and rollout tests are easier to reason about when they are
deterministic C++ logic with explicit inputs and outputs.

### Consequences

Positive:

- Focused unit tests.
- Reusable planning core.
- Lower coupling to ROS runtime.

Trade-offs:

- Integrations must translate ROS data into `mppi_core` observations and
  commands.

### Related files

- `mppi_core/`
- `mppi_core/MPPI.md`

---

## 2026-06-05 - Preserve Existing Misspelled Firmware Path For Now

Status: Accepted

### Context

The firmware notes live under `docs/frimware/plato2/`. The directory name is
misspelled, but changing it would touch links, history, and user muscle memory.

### Decision

Do not rename the directory during harness cleanup. Mention the path as-is and
only migrate it in a dedicated rename task.

### Reason

The harness pass should improve docs without creating unrelated churn.

### Consequences

Positive:

- Small diff.
- No link or script breakage from a documentation cleanup.

Trade-offs:

- The typo remains visible until an explicit migration.

### Related files

- `docs/frimware/plato2/README.md`
