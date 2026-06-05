# Decisions

Newest decisions go first. Record only stable choices that future work should
preserve or intentionally replace.

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
- `mppi_core/README.md`

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
