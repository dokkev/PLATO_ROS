# Object Belief

Object belief represents a weighted set of plausible object poses used for
grasp and contact-support evaluation. It is not a general object-pose tracker
or a rigid-body object-dynamics simulation.

## Ownership

The algorithmic core lives primarily under:

- `mppi_core/include/mppi_core/object/`
- `mppi_core/src/object/`

The main components include `ObjectPrior`, `ObjectPriorEstimator`, belief
initialization, primitive geometry queries, object-contact prediction, and
object-contact support evaluation. Rollout integration is in
`mppi_core/src/rollout/object_prior_grasp_rollout.cpp`; object-support cost
evaluation is in `mppi_core/src/costs/`.

ROS-facing observation and prior integration lives in `plato_state_estimator/`.
`object_prior_estimator_node.cpp` reads joint-state and tactile topics, uses a
configured object geometry and pose prior, calls the ROS-free core estimator,
and publishes a representative pose, particle poses, status, and visualization
markers. These topics are estimates and visualization/diagnostic outputs; the
node does not publish a ROS message containing `VirtualObjectBelief` for MPPI.

The executable `object_state_estimator_node` is separate: it is a legacy
compatibility name for the reference grasp-force generator described in
[`plato_state_estimator/README.md`](../../plato_state_estimator/README.md). It
is not an object-pose or object-belief estimator.

## Data path

```text
measured joint state + tactile contacts + configured object pose prior
  -> object prior / contact-conditioned belief initialization
  -> weighted VirtualObjectBelief pose particles
  -> geometric hemisphere/object contact and support evaluation
  -> MPPI rollout cost
```

Initialization samples poses around the prior and scores them using measured
tactile contact geometry, including surface distance and normal alignment.
Configuration can also use thumb/index contact width and quasi-static cues.
Particles and weights encode plausible object-pose scenarios consistent with
the current prior and contact evidence.

`GraspObservation` carries an `ObjectPrior` and may carry an existing
`VirtualObjectBelief`. The optimizer resolves the initial belief into
`GraspState`; object-aware rollout and cost code then evaluates fingertip
hemisphere geometry against the pose particles. Where enabled, configured
object disturbances make simple scenario pose updates. These updates do not
integrate a rigid-body contact or dynamics model.

The ROS node and `mppi_core` use different boundary types: the node publishes
standard pose and diagnostic messages, while the MPPI core accepts C++ prior
and belief structures. An application adapter is needed to pass estimated
beliefs into MPPI. This repository does not include that adapter.

## Key implementation files

- `include/mppi_core/object/object_prior.hpp`
- `include/mppi_core/object/object_contact_belief.hpp`
- `include/mppi_core/object/object_belief_initializer.hpp`
- `include/mppi_core/object/object_prior_estimator.hpp`
- `include/mppi_core/object/object_geometry_query.hpp`
- `include/mppi_core/object/object_contact_prediction.hpp`
- `include/mppi_core/object/object_contact_support_evaluator.hpp`
- `src/object/object_belief_initializer.cpp`
- `src/object/object_prior_estimator.cpp`
- `src/object/object_geometry_query.cpp`
- `src/object/object_contact_prediction.cpp`
- `src/object/object_contact_support_evaluator.cpp`

For the broader state and command contract, see
[`grasp_state_mppi.md`](grasp_state_mppi.md) and [`MPPI.md`](../MPPI.md).
