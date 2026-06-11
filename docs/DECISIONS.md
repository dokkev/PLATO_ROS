# Decisions

Newest decisions go first. Record only stable choices that future work should
preserve or intentionally replace.

---

## 2026-06-11 - Jenga Object Support Is Scenario Evaluation

Status: Accepted

### Context

The object-aware grasp TODO was narrowed to a defensible MVP: known Jenga box
geometry, approximate initial pose, sampled pose perturbations, and geometric
hemisphere-object support evaluation.

### Decision

Use `VirtualObjectBelief` particles as sampled Jenga pose scenarios. For each
candidate hand action rollout, Pinocchio computes tactile hemisphere world
positions and `ObjectContactSupportEvaluator` queries primitive signed distance
against those sampled object poses. The resulting cost penalizes contact loss,
support deficit, edge risk, and excessive distance-derived force proxy.

This is not object dynamics, contact-force simulation, or true future contact
prediction.

### Consequences

- Rollout action selection can now use object-prior geometry without adding a
  rigid-body object simulator.
- The implementation is suitable for figure/debug visualization because it logs
  selected action names, object sample count, geometry query count, predicted
  support count, and edge/contact-loss diagnostics.
- Mesh/FCL/URDF collision parsing, RViz visualization, and hardware timing
  validation remain future work.

Related files:

- `mppi_core/object/jenga_block.urdf`
- `mppi_core/include/mppi_core/object/object_contact_support_evaluator.hpp`
- `mppi_core/src/object/object_contact_support_evaluator.cpp`
- `mppi_core/src/costs/robust_grasp_state_cost.cpp`
- `mppi_core/src/policy/robust_grasp_policy.cpp`

---

## 2026-06-11 - Object Belief Initialization Is Contact-Weighted Sampling

Status: Accepted

### Context

After adding an object belief slot to `GraspState`, the first object-aware step
is to initialize that belief when tactile contact appears. This should not yet
be a full pose optimizer or object dynamics model.

### Decision

Initialize `VirtualObjectBelief` from measured joint configuration, active
tactile contact points, tactile contact normals, and an object pose prior. The
initializer samples particles around the prior and assigns weights from:

- contact point to object primitive surface distance,
- tactile normal versus object surface normal alignment,
- particle pose distance from the prior.

Use local Pinocchio `Data` for contact extraction so initialization does not
mutate `RobotSystem` state. Keep mesh/URDF surface parsing out of this phase;
URDF or mesh handles can be scored only when primitive dimensions are provided.

### Consequences

- Touch-time object belief is now contact-consistent instead of display-only.
- The rollout still does not predict object pose or object contact dynamics.
- Future work can replace the sampler with an optimizer without changing the
  `VirtualObjectBelief` state contract.

### Related files

- `mppi_core/include/mppi_core/object/object_belief_initializer.hpp`
- `mppi_core/include/mppi_core/object/object_contact_prediction.hpp`
- `mppi_core/include/mppi_core/object/object_geometry_query.hpp`
- `mppi_core/src/object/object_belief_initializer.cpp`
- `mppi_core/src/object/object_contact_prediction.cpp`
- `mppi_core/src/object/object_geometry_query.cpp`
- `mppi_core/test/test_mppi_core.cpp`

---

## 2026-06-11 - MPPI GraspState Carries Virtual Object Belief

Status: Accepted

### Context

The grasp rollout was tactile-only: `GraspState` carried robot and tactile
state, disturbances only moved tactile observations, and the initial rollout
root could follow the current reference instead of measured feedback. That made
virtual object visualization tempting but would have hidden the missing object
state behind a display-only artifact.

### Decision

Add optional object prior and virtual object belief types to `mppi_core`.
`GraspObservation` may carry `ObjectPrior` and `VirtualObjectBelief`, and
`GraspState` now carries `VirtualObjectBelief`. Empty object fields remain valid
so the tactile-only rollout path still works. Rollout propagation preserves the
belief without inventing object motion yet. Initial rollout states use valid
measured `q_meas/qdot_meas` when available, while command generation can still
integrate from the current reference.

### Consequences

- Object-aware rollout work now has a real state slot for object particles,
  geometry handles, and observation priors.
- Tactile-only transition remains the fallback until object-contact transition
  and costs are implemented.
- Visualization should consume state/belief from rollout, not fabricate object
  support independently of the planner.

### Related files

- `mppi_core/include/mppi_core/object/object_prior.hpp`
- `mppi_core/include/mppi_core/object/virtual_object_state.hpp`
- `mppi_core/include/mppi_core/object/object_contact_belief.hpp`
- `mppi_core/include/mppi_core/state/grasp_state.hpp`
- `mppi_core/include/mppi_core/state/grasp_observation.hpp`
- `mppi_core/object/jenga_block.urdf`

---

## 2026-06-07 - MPPI Splits Controller, Rollout, And Task Config

Status: Accepted

### Context

The previous grasp YAML mixed controller/model rollout settings, task
objectives, tactile tolerances, cost weights, and experimental residual knobs.
That made tuning hard and made it unclear which values belonged to the
controller versus a task spec.

### Decision

Use:

- `mppi_core/config/mppi.yaml` for MPPI sampling, action bounds, and command
  gains.
- `mppi_core/config/rollout.yaml` for rollout/tactile model constants and
  disturbance defaults.
- `mppi_core/task/*.yaml` for task start gates, contact support objectives,
  shear/rotation tolerances, and task cost weights.

Keep task YAML files focused on the MVP costs: active tactile sensor loss, active
hemisphere support, shear, rotation, `qddot`, and rollout `RobotState::tau`.
Do not expose residual/contact-force rollout knobs in task YAML files.

### Reason

The first integration should tune physically interpretable task values without
making normal force or residual projection look like the primary horizon
dynamics. Public config headers stay under `include/mppi_core/config`, while
YAML parser implementations and parser helper logic live under `src/util`.

### Consequences

- Tactile birth/loss is rule-based, not a large weighted scoring model.
- Normal force remains a proxy/debug value and is not a default loss trigger.
- Contact support cost penalizes insufficient support but does not reward
  unlimited active hemispheres.
- Residual/contact-force rollout settings stay in experimental config or C++
  defaults until observation-time correction work is promoted.
- `grasp_config.hpp/cpp` and `config/grasp.yaml` are removed instead of
  remaining as a mixed-responsibility surface.

### Related files

- `mppi_core/config/rollout.yaml`
- `mppi_core/config/experimental_residual.yaml`
- `mppi_core/task/grasp_hold.yaml`
- `mppi_core/MPPI.md`
- `mppi_core/include/mppi_core/config/rollout_config.hpp`
- `mppi_core/include/mppi_core/task/task_config.hpp`
- `mppi_core/include/mppi_core/tactile/tactile_transition.hpp`
- `mppi_core/include/mppi_core/costs/grasp_stability_cost.hpp`

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

- `controllers/ros2_control/joint_impedance_controller/`
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
