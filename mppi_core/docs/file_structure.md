# File Structure

`mppi_core` is organized by responsibility:

```text
include/mppi_core/
  core/
  rollout/
  state/
  robot/
  tactile/
  contact/
  object/
  costs/
  config/
  task/
  util/
```

## core/

MPPI optimizer mechanics, action sequences, and sampling configuration.

## rollout/

Top-level rollout interfaces and `GraspStateRolloutModel`. This layer owns
`GraspState -> GraspState` stepping, contact-kinematic tactile transition, and
the `qddot_sol` action contract.

## state/

Rollout and measured input state contracts:

- `grasp_state.hpp`
- `grasp_observation.hpp`

## robot/

Robot rollout state, low-level command packet, and Pinocchio-backed robot
system helpers.

## tactile/

Compact tactile data structures, NARI conversion, and tactile transition
helpers. `TactileState` is sensor-specific and contains all hemispheres for one
sensor.

## contact/

Contact kinematics, hemisphere motion, force projection, and force correction
helpers. Contact kinematics uses one Pinocchio frame per tactile sensor and
derives hemisphere point Jacobians from local offsets.

## object/

Object prior, geometry handle, primitive surface queries, contact-consistent
object belief initialization, object-distance contact prediction, and Jenga
pose-scenario contact support evaluation.
Installable object assets live under the package-level `object/` directory.

## costs/

Cost terms that evaluate predicted `GraspState` rollouts.

## config/

Controller/model config parsers:

- `mppi_config.hpp/cpp` for sampling, horizon, action bounds, and command gains.
- `rollout_config.hpp/cpp` for rollout/tactile model constants and disturbance
  defaults.
Default files live under `config/`:

- `mppi.yaml`
- `rollout.yaml`
- `experimental_residual.yaml`

## task/

Task-spec types and task YAML files. This layer owns start conditions, contact
support objectives, shear/rotation tolerances, task cost weights, and later
task-specific disturbance or object priors.

Default files live under `task/`:

- `grasp_hold.yaml`
- `robust_grasp_hold.yaml`

## util/

Implementation helpers that do not own controller, rollout, or task semantics.
YAML parser helper code and task YAML loading live here so `mppi_core/task/`
can stay a data directory.

## Include Policy

Use the responsibility directories above directly. Old catch-all include paths
and compatibility aliases have been removed, so new code should include the
current owner header instead of relying on migration forwarders.
