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
  costs/
  config/
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

## costs/

Cost terms that evaluate predicted `GraspState` rollouts.

## config/

YAML parsers for MPPI, grasp cost, tactile transition, rollout, and optional
contact-force rollout configuration.

## Include Policy

Use the responsibility directories above directly. Old catch-all include paths
and compatibility aliases have been removed, so new code should include the
current owner header instead of relying on migration forwarders.
