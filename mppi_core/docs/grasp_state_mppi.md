# GraspState-Predictive MPPI

## State definitions

The MPPI prediction target is `GraspState`:

```text
GraspState = RobotState + vector<TactileState>
```

The rollout equation is:

```text
G_{k+1} = f_G(G_k, u_k)
u_k = qddot_sol,k
```

The rollout predicts an ideal/reference robot state and compact tactile/contact
state. It does not predict exact future contact force.

## RobotState

`RobotState` is the robot state container used inside `GraspState` and
`RobotSystem`. The owner decides whether the values are measured or rollout
values:

```cpp
struct RobotState {
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd tau;
};
```

`qddot_sol` is the MPPI action and is not stored in `RobotState`.

## TactileState

`TactileState` describes one tactile sensor. It carries sensor identity,
top-level contact gate, aggregate force, shear, rotational shear, slip scores,
confidence, and a dense list of hemispheres.

`contact_state` is the high-level sensor gate for MPPI start/stop. Hemisphere
contact topology is represented by `hemispheres[i].contact`.

## HemisphereState

`HemisphereState` is one MPPI-level tactile reasoning unit:

```text
hemisphere_index
contact
cop_sensor_m
normal_force_n
confidence
```

Each `TactileState` should contain all hemispheres for the sensor. Inactive
hemispheres remain present with `contact == false`; this makes future contact
birth representable without changing state shape.

## Modular GraspState

`GraspState` carries a dense vector of tactile sensor states:

```cpp
struct GraspState {
  bool valid;
  RobotState robot;
  std::vector<TactileState, Eigen::aligned_allocator<TactileState>>
      tactile_sensors;
};
```

Runtime code should not hard-code production fields for a fixed pair of tactile
sensors. The current PLATO setup may provide two sensors, but rollout, cost, and
adapter-facing code should treat them as an ordered per-sensor vector.

## Action definition

The MPPI action is the solver joint acceleration:

```text
u = qddot_sol
```

Do not treat `delta_q_ref` as the primary action. Use `qddot_sol`,
`GraspStateRolloutConfig`, and `GraspStateRolloutModel` directly.

## Ideal robot rollout

For each rollout step:

```text
qdot[k + 1] = qdot[k] + qddot_sol[k] * dt
q[k + 1]    = integrate(q[k], qdot[k + 1] * dt)
tau[k + 1]  = RNEA(q[k], qdot[k], qddot_sol[k])
```

`EvaluateRollout()` and `PredictRollout()` initialize the rollout root from
`GraspObservation::q_ref_current` and `qdot_ref_current`, with zero initial
rollout torque. The horizon therefore extends the accepted host reference
state, while measured `q_meas`/`qdot_meas` stay in the observation layer.

If Pinocchio model/data are unavailable or incompatible, `tau` falls back to
zero. The rollout does not add embedded PD feedback torque.

## Tactile transition

The current transition preserves `TactileState` shape and updates each sensor
through contact kinematics:

1. Compute one `HemisphereMotion` per hemisphere from robot state, next robot
   state, `TactileState`, and `TactileSensorContext`.
2. Apply deterministic contact survival/loss/birth rules.
3. Refresh contact topology, contact-point motion, shear/rotation features,
   confidence, and aggregate fields.

The transition is applied to each entry in `tactile_sensors` with a matching
entry in `RolloutContext::tactile_contexts`. The MPPI horizon rollout does not
perform measured-torque residual projection or exact future contact-force
prediction.

Rollout tactile model constants are loaded from `config/rollout.yaml`, while
task-specific start gates, support objectives, shear/rotation tolerances, and
cost weights are loaded from `task/*.yaml`. Birth/loss is rule-based for now:
inactive neighboring hemispheres can be born when approach velocity is high
enough and shear/rotation remain within task thresholds; active hemispheres can
be lost when unloading or excessive shear/rotation is detected. Low normal force
alone is not a default loss trigger.

The contact-force projection utilities remain separate from
`GraspStateRolloutModel` and are reserved for later observation-time correction
experiments.

`tau_meas` remains required in `GraspObservation` as robot feedback, but the
current rollout does not use it for residual projection. Rollout
`RobotState::tau` is the RNEA model torque proxy.

## Contact kinematics

Use one Pinocchio frame per tactile sensor. Per-hemisphere point Jacobians are
computed from the sensor-frame spatial Jacobian and the hemisphere local point
offset. Do not add one Pinocchio frame per hemisphere.

`TactileState` stores measured or predicted tactile values. Geometry/context
types store where tactile units are and which Pinocchio frame represents the
sensor.

`GraspObservation` carries measured tactile states in `tactile_meas`,
Pinocchio robot model/data through `robot_system`, and per-sensor contact
kinematics in `tactile_contexts`. `RolloutContext` mirrors those context
pointers during prediction.

## Command packet

`RobotCommand` is the final low-level packet:

```text
q_cmd
qdot_cmd
tau_cmd
kp
kd
stamp_sec
```

`qddot_sol` is integrated into `q_cmd` and `qdot_cmd`; it is not sent as part of
`RobotCommand`. The command builder computes:

```text
tau_ff_cmd = RNEA(q, qdot, qddot_sol)
tau_cmd    = tau_ff_cmd
```

The MPPI command builder does not tune or populate driver-local gains.
`RobotCommand.kp` and `RobotCommand.kd` are copied from controller
`driver_gains` during command finalization outside the planner.

The embedded driver may then apply:

```text
tau_driver = tau_cmd
           + kp * (q_cmd - q_meas)
           + kd * (qdot_cmd - qdot_meas)
```

Debug fields belong in a separate debug type, not in `RobotCommand`.

## Non-goals

- Full physics-based or probabilistic contact birth prediction.
- Object pose tracking.
- Rigid-body contact simulation.
- Controller modes or a new state machine.
- One URDF/Pinocchio frame per hemisphere.
- Exact future contact force prediction.

## Testing harness

Use focused `mppi_core` validation from the workspace root:

```bash
colcon build --symlink-install --packages-up-to mppi_core
colcon test --packages-select mppi_core --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/mppi_core
```

The deterministic tests cover tactile dense hemisphere state, NARI adapter
conversion, modular `GraspState` validity, clean command packet dimensions,
solver-acceleration rollout integration, tactile propagation, and
include-structure drift.
