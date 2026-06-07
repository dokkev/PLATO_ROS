# Plans

This file records current priorities, deferred work, and non-goals. Keep entries
short enough that an agent can quickly decide what matters.

## Current Focus

- Work on `mppi_core` first.
- Keep `mppi_core` ROS-free while making it ready to drive the PLATO grasp stack
  through an explicit adapter layer.
- Validate MPPI behavior with deterministic tests before connecting it to live
  ROS topics or hardware.

## Active Tasks

### MPPI Core Integration Readiness

Status: In progress

Goal:

- Make the contact-local MPPI core ready for the next integration step: measured
  joint state + tactile/contact observation in, hybrid impedance command out.

Non-goals:

- Add ROS message dependencies to `mppi_core`.
- Start real hardware validation before the adapter and tests are clear.
- Hide tactile/contact preconditions inside `MPPIOptimizer`; the caller should
  gate no-contact observations before calling `Update(...)`.

Steps:

1. Review the current `mppi_core` API surface: `GraspObservation`,
   `TactileState`, `MPPIOptimizer`, `GraspStateRolloutModel`,
   `GraspStabilityCost`, and `RobotCommand`.
2. Keep the MPPI action contract as `qddot_des` with units `rad/s^2` or
   `m/s^2`; the rollout predicts `GraspState = RobotState +
   vector<TactileState>`, where `RobotState` carries `q_des`, `qdot_des`,
   `qddot_des`, and `tau_ff`, and each `TactileState` carries predicted
   hemisphere/contact features plus sensor-level aggregate fields.
3. Define the adapter boundary outside `mppi_core`: ROS joint/tactile messages
   become `GraspObservation`; `RobotCommand` becomes `ImpedanceCommands`.
4. Check whether contact kinematics, torque-residual transition config,
   and NARI tactile conversion are available at the intended runtime call site.
5. Add or tighten deterministic tests for no-contact gating, invalid residual
   transition, acceleration integration, RNEA/zero fallback, config parsing,
   and command dimensions before wiring into ROS.
6. Only after tests pass, choose the package/node that owns the ROS adapter and
   launch wiring.

Validation:

- `colcon build --symlink-install --packages-up-to mppi_core`
- `colcon test --packages-select mppi_core`
- `colcon test-result --verbose`

Notes:

- `mppi_core` already documents its intended shape in `mppi_core/README.md`.
- Detailed package notes live in `mppi_core/docs/grasp_state_mppi.md` and
  `mppi_core/docs/file_structure.md`.
- Default tactile transition policy is `residual_required`; a kinematic fallback
  should remain an explicit ablation/debug path.
- `RobotCommand` is the final embedded packet, not the sampled MPPI action.

### Repository Harness Cleanup

Status: Done

Goal:

- Replace template harness docs with current package maps, commands, decisions,
  and runbooks.

Validation:

- `colcon list --names-only` confirmed package discovery.
- Harness docs were checked for placeholder text and stale removed-guideline
  references.
- No build was required for documentation-only edits.

## Backlog

- Replace placeholder `package.xml` descriptions and license fields where
  ownership is clear.
- Add or document a standard fake-hardware smoke test for PLATO if one exists or
  is introduced.
- Add deterministic tests for config parsers when touching actuator, linkage, or
  grasp-plan YAML parsing.
- Decide whether to migrate `docs/frimware/` to `docs/firmware/` and update all
  references in one dedicated change.
- Consider a lightweight docs freshness check for `AGENTS.md` and root `docs/`
  links.
- Consolidate older root command notes from `cli_commands.md` into a clearer
  operator runbook if they continue to be used.

## Not Now

- Broad package restructuring.
- Renaming launch arguments or topics without an explicit compatibility plan.
- Real hardware testing before `mppi_core` adapter behavior is covered by
  deterministic tests.
- Reformatting unrelated source files.

## Open Questions

- Which package should own the first ROS adapter from tactile/joint ROS topics
  to `mppi_core::GraspObservation`?
- Does the first integration target use NARI tactile messages directly, or a
  higher-level tactile/contact estimate already produced elsewhere?
- Which integration package should construct and cache the robot dynamics
  context and per-sensor Pinocchio contact kinematics contexts for runtime
  rollout?
- Are `tactile_sensing` and `sdr_grasp_msgs` always expected in the same
  workspace, or should setup docs point to an external source?
