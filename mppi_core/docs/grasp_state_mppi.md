# GraspState-Predictive MPPI

## State definitions

The MPPI prediction target is `GraspState`:

```text
GraspState = RobotState + vector<TactileState>
```

The rollout equation is:

```text
G_{k+1} = f_G(G_k, u_k)
u_k = qddot_des,k
```

The rollout predicts an ideal/reference robot state and compact tactile/contact
state. It does not predict exact future contact force.

## RobotState

`RobotState` is the ideal/reference robot rollout state:

```cpp
struct RobotState {
  Eigen::VectorXd q_des;
  Eigen::VectorXd qdot_des;
  Eigen::VectorXd qddot_des;
  Eigen::VectorXd tau_ff;
};
```

Measured robot values belong in `GraspObservation`, not in `RobotState`.

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

The MPPI action is desired joint acceleration:

```text
u = qddot_des
```

Do not treat `delta_q_ref` as the primary action. Use `qddot_des`,
`GraspStateRolloutConfig`, and `GraspStateRolloutModel` directly.

## Ideal robot rollout

For each rollout step:

```text
qdot_des[k + 1] = qdot_des[k] + qddot_des[k] * dt
q_des[k + 1]    = integrate(q_des[k], qdot_des[k + 1] * dt)
tau_ff[k + 1]   = RNEA(q_des[k + 1], qdot_des[k + 1], qddot_des[k])
```

If Pinocchio model/data are unavailable or incompatible, `tau_ff` falls back to
zero. The rollout does not add embedded PD feedback torque.

## Tactile transition

The current transition preserves the compact `TactileState` shape and updates
existing contact/sensor aggregate fields through either:

- measured-torque residual force projection, or
- an explicit kinematic fallback used for debug and ablation.

The transition is applied to each entry in `tactile_sensors` with a matching
entry in `RolloutContext::tactile_contexts`. To avoid projecting the same
measured torque residual into multiple contacts, residual force projection is
only used when exactly one tactile sensor has active hemisphere contact. If zero
or more than one sensor is active, the rollout skips residual projection and
uses the kinematic tactile transition until a coupled multi-sensor residual
solver exists.

The force projection output is clamped into a simple friction cone before it is
fed back into tactile force rollout: no positive normal force means zero
tangential force, otherwise tangential force is limited by `mu * normal_force`.
Full contact birth prediction is a non-goal for this refactor.

## Contact kinematics

Use one Pinocchio frame per tactile sensor. Per-hemisphere point Jacobians are
computed from the sensor-frame spatial Jacobian and the hemisphere local point
offset. Do not add one Pinocchio frame per hemisphere.

`TactileState` stores measured or predicted tactile values. Geometry/context
types store where tactile units are and which Pinocchio frame represents the
sensor.

`GraspObservation` carries measured tactile states in `tactile_meas`,
Pinocchio robot dynamics in `robot_dynamics`, and per-sensor contact kinematics
in `tactile_contexts`. `RolloutContext` mirrors those context pointers during
prediction.

## Command packet

`RobotCommand` is the final low-level packet:

```text
q_des
qdot_des
qddot_des
tau_ff
kp
kd
stamp_sec
```

The embedded driver applies:

```text
tau_cmd = tau_ff
        + kp * (q_des - q_meas)
        + kd * (qdot_des - qdot_meas)
```

Debug fields belong in a separate debug type, not in `RobotCommand`.

## Non-goals

- Full contact birth prediction.
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
qddot rollout integration, tactile propagation, and include-structure drift.
