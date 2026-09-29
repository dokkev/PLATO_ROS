# `mppi_core`

`mppi_core` is a reusable C++ implementation of grasp-state MPPI and robust
grasp-policy components. It is intentionally ROS-free: it defines its own
observation, rollout, tactile, object-belief, cost, and command types, and has no
ROS nodes or ROS message conversion. A ROS application must adapt its messages
to `GraspObservation` and adapt the resulting `RobotCommand` to its controller.

## Main data and control path

- **Observation:** `GraspObservation` contains measured and reference robot
  state, a vector of tactile sensor states, per-sensor contact-kinematics
  contexts, and optional object prior/belief values.
- **Rollout state:** `GraspState` contains `RobotState`, the tactile sensor
  vector, and a `VirtualObjectBelief` value that may be empty when object-aware
  evaluation is not used.
- **Action:** MPPI samples sequences of `qddot_sol` joint accelerations. The
  rollout integrates ideal/reference `q` and `qdot`; Pinocchio RNEA supplies a
  torque value when the robot model is available, with a zero fallback.
- **Tactile/contact reasoning:** rollout updates hemisphere contact features
  using robot and sensor-frame kinematics. The robust grasp policy also
  evaluates sampled tactile/contact disturbances.
- **Object-belief integration:** pose particles are plausible object-pose
  scenarios. Geometric hemisphere/object queries score contact support across
  those scenarios; they are not rigid-body object trajectories.
- **Costs and command:** cost terms evaluate contact support, force/shear
  features, object support, and action effort. The selected first acceleration
  is converted into a `RobotCommand` packet; controller gains are supplied by
  the surrounding application.

The default contact-kinematic rollout is deterministic for a given input and
action sequence. Disturbance sampling is implemented in the continuous-qddot
and robust-policy paths. Neither predicts exact future contact force or
simulates full rigid-body contact.

## Configuration and tasks

- `config/mppi.yaml`: MPPI sampling, horizon, action bounds, and command options.
- `config/rollout.yaml`: rollout, tactile transition, and disturbance defaults.
- `config/experimental_residual.yaml`: experimental residual-correction values.
- `task/grasp_hold.yaml`: grasp-hold objectives and cost settings.
- `task/robust_grasp_hold.yaml`: robust-policy task and disturbance settings.

## Documentation

- [MPPI design and current implementation](MPPI.md)
- [GraspState, rollout, and command contract](docs/grasp_state_mppi.md)
- [Object-belief ownership and semantics](docs/object_belief.md)
- [Robust grasp policy](docs/robust_grasp_policy.md)
- [Package file structure](docs/file_structure.md)

## Focused build and test

Run workspace commands from `~/workspace/plato_ws`:

```bash
colcon build --symlink-install --packages-up-to mppi_core
colcon test --packages-select mppi_core --event-handlers console_direct+
colcon test-result --verbose
```
