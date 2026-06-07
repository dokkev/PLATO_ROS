# mppi_core

`mppi_core` is a ROS-free C++ package for contact-local MPPI grasp planning.
It samples desired joint acceleration, predicts a modular tactile-sensor
`GraspState` rollout, evaluates grasp costs, and outputs a low-level robot
command packet.

Main runtime pieces:

- `mppi_core::MPPIOptimizer`
- `mppi_core::GraspStateRolloutModel`
- `mppi_core::GraspState`
- `mppi_core::RobotState`
- `mppi_core::TactileState`
- `mppi_core::GraspObservation`
- `mppi_core::GraspStabilityCost`
- `mppi_core::RobotCommand`

Public headers are organized by responsibility under `state/`, `rollout/`,
`robot/`, `tactile/`, `contact/`, `costs/`, `config/`, and `core/`.

## Runtime Contract

The MPPI action is `qddot_des`. Its dimension is `nv`; units are `rad/s^2` for
revolute joints and `m/s^2` for prismatic joints.

The top-level rollout target is always `GraspState`:

```text
GraspState = RobotState + vector<TactileState>
```

Each robot rollout step assumes ideal reference tracking:

```text
qdot_des[k + 1] = qdot_des[k] + qddot_des[k] * dt
q_des[k + 1]    = integrate(q_des[k], qdot_des[k + 1] * dt)
tau_ff[k + 1]   = rnea(q_des[k + 1], qdot_des[k + 1], qddot_des[k])
```

If a valid Pinocchio model/data context is unavailable, `tau_ff` falls back to a
deterministic zero vector. The embedded low-level driver applies the feedback
law:

```text
tau_cmd = tau_ff
        + kp * (q_des - q_meas)
        + kd * (qdot_des - qdot_meas)
```

The embedded PD feedback torque is not part of MPPI prediction.

## State Separation

`GraspObservation` contains measured inputs:

- `q_meas`
- `qdot_meas`
- `tau_meas`
- `q_ref_current`
- `qdot_ref_current`
- `tactile_meas`
- `robot_dynamics`
- `tactile_contexts`

`RobotState` contains rollout/reference robot state:

- `q_des`
- `qdot_des`
- `qddot_des`
- `tau_ff`

`TactileState` describes one tactile sensor. It contains sensor-level aggregate
fields such as `total_force_n`, shear displacement, rotational shear, slip
scores, confidence, and a dense fixed list of `HemisphereState` entries.
Inactive hemispheres stay in the list with `contact == false` so future contact
birth can be represented as `false -> true`.

The rollout predicts future `GraspState`: ideal/reference robot state plus
predicted tactile/contact-mode state. It does not claim exact future contact
force prediction.

## Tactile Transition

The default tactile transition uses measured-torque residual projection. If the
residual projection is unavailable or invalid, the rollout step is invalid.
Kinematic tactile transition remains available only as an explicit debug or
ablation fallback through
`TactileRolloutPolicy::kResidualThenKinematicFallback`.

`MPPIOptimizer` is rollout-model agnostic. The caller should gate no-contact
observations before entering contact-local MPPI. If every sampled rollout is
invalid, the optimizer resets the nominal action sequence and returns a
zero-acceleration hold command.

## Example

```cpp
mppi_core::MPPIConfig mppi_config;
mppi_core::GraspStateRolloutConfig rollout_config;
rollout_config.tactile_rollout_policy =
    mppi_core::TactileRolloutPolicy::kResidualRequired;

auto model = std::make_shared<mppi_core::GraspStateRolloutModel>(
    joint_dim, rollout_config);
auto cost = std::make_shared<mppi_core::GraspStabilityCost>(
    mppi_core::GraspStabilityCostConfig{});

mppi_core::MPPIOptimizer optimizer;
optimizer.Initialize(mppi_config, model, cost);

mppi_core::GraspObservation obs;
obs.q_meas = q_meas;
obs.qdot_meas = qdot_meas;
obs.tau_meas = tau_meas;
obs.q_ref_current = q_ref_current;
obs.qdot_ref_current = qdot_ref_current;
obs.tactile_meas = tactile_sensors;
obs.robot_dynamics = &robot_dynamics;
obs.tactile_contexts = tactile_contexts;
obs.contact_force_projection_config = &projection_config;
obs.contact_force_rollout_config = &force_rollout_config;

const mppi_core::RobotCommand command = optimizer.Update(obs);
```

## Config

Sampling budget, acceleration action bounds, and command gains live in
`config/mppi.yaml`.

Contact-local costs and grasp-state transition settings live in
`config/grasp.yaml`. Hemisphere-based config names are preferred, for example:

```yaml
grasp:
  grasp_state_transition:
    rollout_policy: residual_required

  contact_force_rollout:
    min_active_hemisphere_count: 4

  hemisphere_contact:
    enabled: true
    target_active_hemisphere_count: 6
    weight: 2.0
```

More detail:

- `docs/grasp_state_mppi.md`
- `docs/file_structure.md`
