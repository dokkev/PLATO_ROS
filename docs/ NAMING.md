# Controller Variable Naming Convention

This document defines naming conventions for robot controller variables.

The goal is to keep the control pipeline explicit:

```txt
task objective -> solver result -> host command -> driver/local control -> robot feedback -> accepted state
```

In an ideal system, several values may be numerically equal. In real robot software, they still belong to different layers and should not be collapsed.

---

## 1. Core Suffix Rule

Use suffixes to describe the control layer of a value.

```txt
_des      = desired by a task, planner, or controller objective
_sol      = computed by a solver
_cmd      = passed from the host to the driver interface
_meas     = reported by hardware, sensor, or driver
_est      = estimated or filtered value
_applied  = physically applied ground-truth-like value, rarely observable
```

Use unsuffixed `q`, `qdot`, `qddot`, and `tau` only for the canonical current robot state accepted by the host controller.

```txt
q     = current configuration used by model/control
qdot  = current velocity used by model/control
qddot = current acceleration used by model/control, if available
tau   = current effort/torque used by model/control
```

The unsuffixed state may come from measured, calibrated, filtered, or estimated values. The suffix-free name means the host controller has accepted it as the current state for this control tick.

---

## 2. Why Layers Are Separate

A task may desire one value, but a solver may compute another because of conflicts, priorities, constraints, or limits.

```txt
q_des != q_sol
```

A solver result may then be processed by the host before being sent to the driver.

```txt
q_sol != q_cmd
```

The robot may not exactly track the command because of delay, compliance, saturation, backlash, contact, or actuator limits.

```txt
q_cmd != q_meas
```

The intended control pipeline is:

```txt
x_des / q_des
  -> solver
  -> q_sol / qdot_sol / qddot_sol / tau_sol
  -> host command builder and limits
  -> q_cmd / qdot_cmd / tau_cmd
  -> embedded driver
  -> robot
  -> q_meas / qdot_meas / tau_meas
  -> host state preprocessing
  -> q / qdot / tau
```

---

## 3. Current State and Feedback Values

Use unsuffixed values for the canonical state used by the controller during the current control tick.

```cpp
struct RobotState
{
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd tau;
};
```

Use `*_meas` for values reported by hardware, sensors, or drivers.

```txt
q_meas
qdot_meas
tau_meas
current_meas
```

Example:

```cpp
const JointFeedback feedback = driver.read();

RobotState state;
state.q = apply_joint_calibration(feedback.q_meas);
state.qdot = velocity_filter.update(feedback.qdot_meas);
state.tau = torque_filter.update(feedback.tau_meas);
```

For many motors, reported torque is estimated from measured current:

```txt
tau_meas ≈ current_meas * Kt
```

For geared joints, depending on the driver convention:

```txt
tau_meas ≈ current_meas * Kt * gear_ratio
```

Use `*_est` for explicit estimates or filtered intermediate values.

```txt
qdot_est
tau_est
contact_force_est
```

---

## 4. Desired Values

Use `*_des` for values desired by a task, planner, or controller objective.

```txt
x_des
xdot_des
q_des
qdot_des
```

These values represent what the objective wants before arbitration or conflict resolution.

Good:

```cpp
posture_task.q_des = nominal_posture;
hand_task.x_des = target_hand_pose;
```

Bad:

```cpp
q_des = solve_wbc(problem);  // Bad: solver output is not a desired value.
```

---

## 5. Solver Results

Use `*_sol` for raw results from solvers such as IK, QP, WBC, MPC, inverse dynamics, or trajectory optimization.

```txt
q_sol
qdot_sol
qddot_sol
tau_sol
contact_force_sol
```

Meaning:

```txt
The solver computed this value.
```

It does not mean the value was sent to hardware, accepted by the driver, physically applied, or measured back.

Example:

```cpp
const WbcSolution solution = solve_wbc(problem, state);

const Eigen::VectorXd qddot_sol = solution.qddot_sol;
const Eigen::VectorXd contact_force_sol = solution.contact_force_sol;
```

---

## 6. From WBC Acceleration to Driver Command

In acceleration-based WBC, the solver commonly outputs:

```txt
qddot_sol
```

The host controller converts this solver result into torque, velocity, and position commands.

Typical flow:

```txt
qddot_sol
  -> inverse dynamics
  -> tau_ff_cmd

qddot_sol
  -> host-side integration and limits
  -> qdot_cmd, q_cmd

q_cmd, qdot_cmd, q, qdot
  -> optional host-side feedback
  -> tau_fb_cmd

tau_ff_cmd + tau_fb_cmd
  -> tau_cmd

tau_cmd, qdot_cmd, q_cmd, kp, kd
  -> motor driver
```

---

## 7. Feedforward Torque Command

Compute model-based feedforward torque from `qddot_sol` using inverse dynamics:

```txt
tau_ff_cmd = M(q) qddot_sol + h(q, qdot)
```

where:

```txt
h(q, qdot) = C(q, qdot) qdot + g(q)
```

In Pinocchio:

```cpp
tau_ff_cmd = pinocchio::rnea(model, data, q, qdot, qddot_sol);
```

Meaning:

```txt
tau_ff_cmd = pure model-based feedforward torque command
```

Do not include feedback inside `tau_ff_cmd`.

If contact force compensation is explicitly modeled, the controller may include a contact term depending on sign convention:

```txt
tau_ff_cmd = M(q) qddot_sol + h(q, qdot) - J_c(q)^T f_c
```

Only include this term when contact force compensation is intentionally part of the controller.

---

## 8. Host-Side Integration to Position and Velocity Commands

The host may integrate `qddot_sol` to produce driver position and velocity commands.

The integrated values should be clamped before being stored as `_cmd` values:

```cpp
qdot_cmd = clamp_velocity(qdot_cmd_prev + dt * qddot_sol);
q_cmd = clamp_position(q_cmd_prev + dt * qdot_cmd);
```

or, if integrating from the current accepted robot state:

```cpp
qdot_cmd = clamp_velocity(state.qdot + dt * qddot_sol);
q_cmd = clamp_position(state.q + dt * qdot_cmd);
```

Meaning:

```txt
q_cmd    = position command passed to the driver after host-side limits
qdot_cmd = velocity command passed to the driver after host-side limits
```

Do not keep separate names such as `q_cmd_raw` unless the unclamped value is explicitly needed for debugging or analysis.

The `_cmd` values should be the actual values passed to the driver interface.

---

## 9. Optional Host-Side Feedback Torque

The host may optionally compute a tracking feedback torque:

```txt
tau_fb_cmd = kp_fb * (q_cmd - q)
           + kd_fb * (qdot_cmd - qdot)
```

where:

```txt
kp_fb = host-side feedback proportional gain
kd_fb = host-side feedback derivative gain
```

Then:

```txt
tau_cmd = tau_ff_cmd + tau_fb_cmd
```

In some modes:

```txt
tau_fb_cmd = 0
```

This is appropriate when:

* the model-based torque is intended to be used alone
* the embedded driver handles all impedance tracking
* the user does not want host-side tracking feedback
* the model is assumed to be accurate enough for the task

Naming rule:

```txt
tau_ff_cmd = model-based feedforward torque command
tau_fb_cmd = optional host-side feedback torque command
tau_cmd    = final torque command passed to the driver
```

Good:

```cpp
tau_ff_cmd = compute_inverse_dynamics(q, qdot, qddot_sol);
tau_fb_cmd = compute_host_feedback(q_cmd, qdot_cmd, q, qdot);
tau_cmd = tau_ff_cmd + tau_fb_cmd;
```

Bad:

```cpp
tau_ff_cmd = tau_ff_cmd + tau_fb_cmd;  // Bad: no longer pure feedforward.
```

---

## 10. Driver-Local Impedance Gains

The motor driver command should contain only values passed to the embedded driver.

```cpp
struct JointCommand
{
  Eigen::VectorXd tau_cmd;
  Eigen::VectorXd qdot_cmd;
  Eigen::VectorXd q_cmd;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};
```

Meaning:

```txt
tau_cmd  = final torque command passed to the driver
qdot_cmd = velocity command passed to the driver
q_cmd    = position command passed to the driver
kp       = driver-local proportional gain
kd       = driver-local derivative gain
```

The gains `kp` and `kd` are used by the embedded driver. They are not host-side feedback gains.

Host-side feedback gains must be named separately:

```txt
kp_fb
kd_fb
```

These are used only to compute `tau_fb_cmd` on the host.

Good:

```cpp
JointCommand command;
command.tau_cmd = tau_cmd;
command.qdot_cmd = qdot_cmd;
command.q_cmd = q_cmd;
command.kp = driver_kp;
command.kd = driver_kd;
```

Bad:

```cpp
command.kp = kp_fb;  // Bad: host-side feedback gain sent as driver-local gain.
command.kd = kd_fb;  // Bad.
```

The embedded driver may internally combine the command as an impedance controller:

```txt
tau_driver_internal =
    tau_cmd
  + kp * (q_cmd - q_meas)
  + kd * (qdot_cmd - qdot_meas)
```

This internal value should not be called `tau_applied` unless it is explicitly reported and validated as a physical applied torque.

---

## 11. Measured vs Applied Values

Use `*_meas` for values reported by hardware, sensors, or drivers.

Use `*_applied` only for values that represent physically applied ground-truth-like quantities.

For torque:

```txt
tau_cmd     = torque command sent from host to driver
tau_meas    = torque reported or estimated by driver/robot
tau_applied = actual physical joint torque, rarely directly observable
```

In most systems:

```txt
tau_applied is not available
```

Do not assume:

```txt
tau_applied = tau_cmd
```

Do not assume:

```txt
tau_applied = tau_meas
```

unless the measurement source is explicitly calibrated and documented as a physical ground-truth-like torque measurement.

Good:

```cpp
feedback.tau_meas = driver_feedback.tau_meas;
state.tau = torque_filter.update(feedback.tau_meas);
```

Bad:

```cpp
state.tau_applied = driver_feedback.tau_meas;  // Bad: reported estimate is not ground truth.
```

Use `*_applied` only for values from calibrated physical measurement systems, external ground-truth sensors, or explicitly validated applied-value reports.

---

## 12. Complete WBC Command Example

```cpp
const WbcSolution solution = solve_wbc(problem, state);

// Solver result.
const Eigen::VectorXd qddot_sol = solution.qddot_sol;

// Pure model-based feedforward torque.
const Eigen::VectorXd tau_ff_cmd =
    pinocchio::rnea(model, data, state.q, state.qdot, qddot_sol);

// Integrate acceleration into host-side command references.
// Clamp before storing as _cmd values.
Eigen::VectorXd qdot_cmd =
    clamp_velocity(qdot_cmd_prev + dt * qddot_sol);

Eigen::VectorXd q_cmd =
    clamp_position(q_cmd_prev + dt * qdot_cmd);

// Optional host-side feedback torque.
Eigen::VectorXd tau_fb_cmd = Eigen::VectorXd::Zero(model.nv);

if (use_host_feedback) {
  tau_fb_cmd =
      kp_fb.cwiseProduct(q_cmd - state.q)
    + kd_fb.cwiseProduct(qdot_cmd - state.qdot);
}

// Final torque command passed to the driver.
Eigen::VectorXd tau_cmd = tau_ff_cmd + tau_fb_cmd;
tau_cmd = clamp_torque(tau_cmd);

// Build driver command.
JointCommand command;
command.tau_cmd = tau_cmd;
command.qdot_cmd = qdot_cmd;
command.q_cmd = q_cmd;
command.kp = driver_kp;
command.kd = driver_kd;

// Send to embedded driver.
if (driver.write(command)) {
  last_sent_command = command;
  qdot_cmd_prev = qdot_cmd;
  q_cmd_prev = q_cmd;
}
```

---

## 13. Rule of Thumb

General suffixes:

```txt
_des       = wanted
_sol       = solved
_cmd       = passed to driver
_meas      = reported back by robot, sensor, or driver
_est       = estimated or filtered
_applied   = physically applied ground-truth-like value
q/qdot/tau = accepted current state
```

Torque-specific names:

```txt
tau_ff_cmd = model-based feedforward torque command
tau_fb_cmd = host-side feedback torque command
tau_cmd    = final torque command passed to driver
tau_meas   = measured/reported torque, often current_meas * Kt
```

Gain-specific names:

```txt
kp, kd        = driver-local impedance gains
kp_fb, kd_fb  = host-side feedback gains
```

Anti-patterns:

```cpp
q_cmd = solve_ik(x_des);  // Bad: solver output skipped the _sol layer.
```

Good:

```cpp
q_sol = solve_ik(x_des);
q_cmd = build_command_from_solution(q_sol, state);
```

Bad:

```cpp
tau_ff_cmd = tau_ff_cmd + tau_fb_cmd;  // Bad: feedforward name reused for final torque.
```

Good:

```cpp
tau_cmd = tau_ff_cmd + tau_fb_cmd;
```

Bad:

```cpp
state.tau_applied = driver_feedback.tau_meas;  // Bad: measured estimate is not applied ground truth.
```

Good:

```cpp
state.tau = torque_filter.update(driver_feedback.tau_meas);
```
