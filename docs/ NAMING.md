# Controller Variable Naming Convention

This document defines naming conventions for robot controller variables.

The goal is to keep the control pipeline explicit:

```txt
task objective
  -> solver result
  -> host command construction
  -> driver/local control
  -> robot feedback
  -> accepted state
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

The unsuffixed state may come from measured, calibrated, filtered, or estimated values.

The suffix-free name means:

```txt
The host controller has accepted this value as the current state for this control tick.
```

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

Do not skip layers in variable names just because two values are currently equal.

For example, even if the solver output is directly sent to the driver, write:

```cpp
q_sol = solve_ik(x_des);
q_cmd = q_sol;
```

rather than:

```cpp
q_cmd = solve_ik(x_des);  // Avoid: hides the solver layer.
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
tau_meas ~= current_meas * Kt
```

For geared joints, depending on the driver convention:

```txt
tau_meas ~= current_meas * Kt * gear_ratio
```

Use `*_est` for explicit estimates or filtered intermediate values.

```txt
qdot_est
tau_est
contact_force_est
object_pose_est
```

Do not call a value `*_est` just because it passed through normal state preprocessing. Use `*_est` when estimation itself is semantically important.

---

## 4. Desired Values

Use `*_des` for values desired by a task, planner, or controller objective.

```txt
x_des
xdot_des
q_des
qdot_des
tau_des
```

These values represent what the objective wants before arbitration, conflict resolution, constraints, or command construction.

Good:

```cpp
posture_task.q_des = nominal_posture;
hand_task.x_des = target_hand_pose;
```

Bad:

```cpp
q_des = solve_wbc(problem);  // Bad: solver output is not a desired value.
```

For task-space control, desired values usually belong to a task objective:

```cpp
task.x_des = target_pose;
task.xdot_des = target_velocity;
```

For joint-space initialization or hold behavior, desired values may be simple constants:

```cpp
initialize_task.q_des = initial_posture;
```

---

## 5. Solver Results

Use `*_sol` for raw results from solvers such as IK, QP, WBC, MPC, MPPI, inverse dynamics optimization, or trajectory optimization.

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

For sampling-based controllers such as MPPI, use `_sol` for the selected rollout/control result after optimization or selection:

```cpp
const MppiSolution solution = mppi.solve(state);

qddot_sol = solution.qddot_sol;
tau_sol = solution.tau_sol;
```

Do not call solver outputs `_cmd` until they have passed through the host command builder and safety limits.

---

## 6. Host Command Construction

The host command builder converts solver/task outputs into values that are actually passed to the embedded driver.

The command layer uses the `_cmd` suffix.

```txt
q_cmd
qdot_cmd
tau_cmd
```

Meaning:

```txt
This value is passed from the host to the driver interface.
```

The host command builder may perform:

```txt
solver result integration
position/velocity/torque limits
fixed joint overrides
soft limits
safety fallback
unit conversion at hardware boundaries
driver gain attachment
```

The `_cmd` values should be the actual values passed to the driver interface after host-side command processing.

Do not keep separate names such as `q_cmd_raw` unless the unclamped value is explicitly needed for debugging or analysis.

---

## 7. From WBC Acceleration to Driver Command

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
  -> optional host-side task feedback
  -> tau_fb_cmd

tau_ff_cmd + tau_fb_cmd
  -> tau_cmd

q_cmd, qdot_cmd, tau_cmd
  -> RobotCommand

RobotCommand + driver_gains from YAML
  -> final RobotCommand.kp / RobotCommand.kd

RobotCommand
  -> embedded driver
```

The planner, solver, task, or MPPI policy should not tune driver-local gains unless a future controller mode explicitly supports that behavior.

In the current convention:

```txt
planner/state/task/MPPI:
  produces q_cmd, qdot_cmd, tau_cmd

command finalization:
  attaches driver gains into RobotCommand.kp and RobotCommand.kd
```

---

## 8. Feedforward Torque Command

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

Good:

```cpp
tau_ff_cmd = compute_inverse_dynamics(q, qdot, qddot_sol);
```

Bad:

```cpp
tau_ff_cmd = tau_ff_cmd + tau_fb_cmd;  // Bad: no longer pure feedforward.
```

If contact force compensation is explicitly modeled, the controller may include a contact term depending on sign convention:

```txt
tau_ff_cmd = M(q) qddot_sol + h(q, qdot) - J_c(q)^T f_c
```

Only include this term when contact force compensation is intentionally part of the model-based feedforward controller.

---

## 9. Host-Side Integration to Position and Velocity Commands

The host may integrate `qddot_sol` to produce driver position and velocity commands.

The integrated values should be clamped before being stored as `_cmd` values.

Example using the previous command state:

```cpp
qdot_cmd = clamp_velocity(qdot_cmd_prev + dt * qddot_sol);
q_cmd = clamp_position(q_cmd_prev + dt * qdot_cmd);
```

Example integrating from the current accepted robot state:

```cpp
qdot_cmd = clamp_velocity(state.qdot + dt * qddot_sol);
q_cmd = clamp_position(state.q + dt * qdot_cmd);
```

Meaning:

```txt
q_cmd    = position command passed to the driver after host-side limits
qdot_cmd = velocity command passed to the driver after host-side limits
```

Do not store an unclamped value as `_cmd`.

Good:

```cpp
const Eigen::VectorXd qdot_next =
    qdot_cmd_prev + dt * qddot_sol;

qdot_cmd = clamp_velocity(qdot_next);
```

Bad:

```cpp
qdot_cmd = qdot_cmd_prev + dt * qddot_sol;
qdot_cmd = clamp_velocity(qdot_cmd);  // Acceptable but less explicit.
```

The final `_cmd` value should always represent the command that is safe to send.

---

## 10. Optional Host-Side Task Feedback Torque

The host may optionally compute a tracking feedback torque.

Use:

```txt
kp_task
kd_task
```

for host-side task feedback gains.

These gains are used only on the host to compute `tau_fb_cmd`.

```txt
tau_fb_cmd = kp_task * (q_cmd - q)
           + kd_task * (qdot_cmd - qdot)
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

```txt
the model-based torque is intended to be used alone
the embedded driver handles all impedance tracking
the user does not want host-side tracking feedback
the model is assumed to be accurate enough for the task
the controller is in a passive or damping-only mode
```

Naming rule:

```txt
kp_task     = host-side task proportional gain
kd_task     = host-side task derivative gain
tau_ff_cmd  = model-based feedforward torque command
tau_fb_cmd  = optional host-side task feedback torque command
tau_cmd     = final torque command passed to the driver
```

Good:

```cpp
tau_ff_cmd = compute_inverse_dynamics(q, qdot, qddot_sol);

tau_fb_cmd =
    kp_task.cwiseProduct(q_cmd - q)
  + kd_task.cwiseProduct(qdot_cmd - qdot);

tau_cmd = tau_ff_cmd + tau_fb_cmd;
```

Bad: copying task feedback gains into `RobotCommand.kp` or
`RobotCommand.kd`. Those fields are reserved for driver-local command gains.

Avoid using ambiguous names such as `kp_fb`, `kd_fb`, `kp`, or `kd` for host-side task feedback gains in state/task code.

Preferred:

```txt
kp_task
kd_task
```

Allowed in equations or discussion:

```txt
feedback gain
host-side feedback gain
task-space feedback gain
```

But in code, prefer `kp_task` and `kd_task`.

---

## 11. Driver-Local Impedance Gains

The embedded driver may run its own local impedance controller.

Driver-local gains are sent as part of the final command packet.

In code, inside `RobotCommand`, use:

```cpp
struct RobotCommand
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
kp       = driver-local proportional gain inside RobotCommand
kd       = driver-local derivative gain inside RobotCommand
```

Do not introduce redundant gain field names inside `RobotCommand`.

Preferred:

```cpp
RobotCommand cmd;
cmd.kp = driver_gains.kp;
cmd.kd = driver_gains.kd;
```

The driver-local gains are copied from controller configuration during command finalization.

Example YAML:

```yaml
driver_gains:
  # Driver-local impedance gains copied into RobotCommand.kp / RobotCommand.kd.
  # These are sent to the embedded driver.
  # They are separate from host-side task gains.
  kp: [0.0, 0.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
  kd: [0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

The planner, state machine, task controller, WBC, or MPPI solver should not modify `RobotCommand.kp` or `RobotCommand.kd`.

Instead:

```txt
planner/task/solver:
  computes q_cmd, qdot_cmd, tau_cmd

command finalization:
  copies driver_gains.kp into RobotCommand.kp
  copies driver_gains.kd into RobotCommand.kd
```

Good:

```cpp
JointCommandCore core;
core.q_cmd = q_cmd;
core.qdot_cmd = qdot_cmd;
core.tau_cmd = tau_cmd;

RobotCommand cmd = build_robot_command(core, driver_gains);
```

Good:

```cpp
RobotCommand cmd;
cmd.q_cmd = q_cmd;
cmd.qdot_cmd = qdot_cmd;
cmd.tau_cmd = tau_cmd;
cmd.kp = config.driver_gains.kp;
cmd.kd = config.driver_gains.kd;
```

Bad: copying host-side task gains into `cmd.kp` or `cmd.kd`.

The embedded driver may internally combine the command as an impedance controller:

```txt
tau_driver_internal =
    tau_cmd
  + cmd.kp * (q_cmd - q_meas)
  + cmd.kd * (qdot_cmd - qdot_meas)
```

This internal value should not be called `tau_applied` unless it is explicitly reported and validated as a physical applied torque.

---

## 12. RobotCommand Semantics

`RobotCommand` is the final host-to-driver command representation.

It should contain only values that are passed to the embedded driver interface.

```cpp
struct RobotCommand
{
  Eigen::VectorXd q_cmd;
  Eigen::VectorXd qdot_cmd;
  Eigen::VectorXd tau_cmd;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};
```

Meaning:

```txt
q_cmd    = position command passed to the driver
qdot_cmd = velocity command passed to the driver
tau_cmd  = final torque command passed to the driver
kp       = driver-local proportional gain passed to the driver
kd       = driver-local derivative gain passed to the driver
```

`tau_cmd` may already include host-side feedback:

```txt
tau_cmd = tau_ff_cmd + tau_fb_cmd
```

where:

```txt
tau_ff_cmd = pure model-based feedforward torque command
tau_fb_cmd = optional host-side task feedback torque command
```

`RobotCommand.kp` and `RobotCommand.kd` are not host-side task gains.

They are driver-local gains attached at command finalization time.

Recommended command construction:

```cpp
RobotCommand cmd;
cmd.q_cmd = q_cmd;
cmd.qdot_cmd = qdot_cmd;
cmd.tau_cmd = tau_cmd;

// Driver-local gains. These are not produced by the planner.
cmd.kp = config.driver_gains.kp;
cmd.kd = config.driver_gains.kd;
```

Do not introduce an intermediate `ImpedanceSetpoint` unless there is a concrete need for a separate abstraction.

In the preferred architecture:

```txt
State / Task / Planner / MPPI
  -> q_cmd, qdot_cmd, tau_cmd

Command finalization
  -> attach driver-local kp/kd
  -> apply fixed joint overrides
  -> apply final safety checks
  -> RobotCommand

ROS/hardware interface
  -> write RobotCommand to embedded driver
```

---

## 13. Units and Hardware Boundaries

Model-based computation should use consistent physical units internally.

Recommended internal convention:

```txt
q       rad
qdot    rad/s
qddot   rad/s^2
tau     Nm
force   N
length  m
```

Pinocchio, inverse dynamics, MPPI rollout, WBC, and model-based control should use SI units unless explicitly documented otherwise.

Hardware or driver layers may use a different command convention.

For example, a driver command interface may expect torque and gains in:

```txt
tau_cmd  mNm
kp       mNm/rad
kd       mNm/(rad/s)
```

If so, the conversion boundary must be explicit.

Good:

```cpp
const Eigen::VectorXd tau_cmd_Nm = tau_ff_cmd_Nm + tau_fb_cmd_Nm;
cmd.tau_cmd = tau_cmd_Nm;
cmd.kp = config.driver_gains.kp;
cmd.kd = config.driver_gains.kd;
write_command(convert_to_driver_units(cmd));
```

Bad:

```cpp
tau_ff_cmd = pinocchio::rnea(model, data, q, qdot, qddot_sol);
cmd.tau_cmd = tau_ff_cmd;  // Bad if cmd expects mNm.
```

If `RobotCommand` uses driver command units, document this directly in the command struct and YAML.

Example:

```cpp
// RobotCommand uses the Aristo driver command convention.
// tau_cmd: mNm
// kp:      mNm/rad
// kd:      mNm/(rad/s)
struct RobotCommand
{
  Eigen::VectorXd q_cmd;
  Eigen::VectorXd qdot_cmd;
  Eigen::VectorXd tau_cmd;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};
```

Do not silently mix SI model units with driver command units.

---

## 14. Measured vs Applied Values

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

Examples where `*_applied` may be appropriate:

```txt
externally measured joint torque from a calibrated torque sensor
ground-truth force from a calibrated force plate
experiment-only measurement from a validated external instrument
```

---

## 15. Tactile and Contact Values

Use clear suffixes for tactile and contact-related values as well.

Examples:

```txt
contact_force_meas
contact_force_est
contact_force_sol
contact_force_cmd
cop_meas
cop_est
```

Meaning:

```txt
contact_force_meas = reported directly by a tactile or force sensor
contact_force_est  = estimated or filtered contact force
contact_force_sol  = solver-computed contact force
contact_force_cmd  = commanded or desired contact force passed into a control layer
```

For tactile rollout or prediction:

```txt
contact_state
contact_state_pred
normal_force_pred
slip_velocity_pred
```

Use `_pred` when the value is predicted by a rollout or model but is not itself a solver decision variable.

Example:

```cpp
const TactileState tactile_meas = tactile_driver.read();
const TactileState tactile_est = tactile_filter.update(tactile_meas);
const TactileState tactile_pred = rollout.predict_tactile(state, action);
```

The canonical accepted tactile state used by the controller may be unsuffixed inside `RobotState`:

```cpp
struct RobotState
{
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd tau;
  TactileState tactile;
};
```

---

## 16. Complete WBC Command Example

```cpp
const WbcSolution solution = solve_wbc(problem, state);

// Solver result.
const Eigen::VectorXd qddot_sol = solution.qddot_sol;

// Pure model-based feedforward torque.
const Eigen::VectorXd tau_ff_cmd_Nm =
    pinocchio::rnea(model, data, state.q, state.qdot, qddot_sol);

// Integrate acceleration into host-side command references.
// Clamp before storing as _cmd values.
Eigen::VectorXd qdot_cmd =
    clamp_velocity(qdot_cmd_prev + dt * qddot_sol);

Eigen::VectorXd q_cmd =
    clamp_position(q_cmd_prev + dt * qdot_cmd);

// Optional host-side task feedback torque.
Eigen::VectorXd tau_fb_cmd_Nm = Eigen::VectorXd::Zero(model.nv);

if (use_host_task_feedback) {
  tau_fb_cmd_Nm =
      kp_task.cwiseProduct(q_cmd - state.q)
    + kd_task.cwiseProduct(qdot_cmd - state.qdot);
}

// Final torque command passed to the driver.
Eigen::VectorXd tau_cmd_Nm = tau_ff_cmd_Nm + tau_fb_cmd_Nm;
tau_cmd_Nm = clamp_torque(tau_cmd_Nm);

// Convert to driver command units if necessary.
const Eigen::VectorXd tau_cmd =
    convert_torque_to_driver_units(tau_cmd_Nm);

// Build final driver command.
RobotCommand cmd;
cmd.q_cmd = q_cmd;
cmd.qdot_cmd = qdot_cmd;
cmd.tau_cmd = tau_cmd;

// Driver-local gains are copied from configuration.
// Planner/WBC/MPPI does not tune these.
cmd.kp = config.driver_gains.kp;
cmd.kd = config.driver_gains.kd;

// Send to embedded driver.
if (driver.write(cmd)) {
  last_sent_command = cmd;
  qdot_cmd_prev = qdot_cmd;
  q_cmd_prev = q_cmd;
}
```

---

## 17. Complete MPPI Command Example

MPPI may produce a selected acceleration, velocity, torque, or action depending on the controller design.

Example where MPPI produces `qddot_sol`:

```cpp
const MppiSolution solution = mppi.solve(state, rollout_model);

// Solver result from MPPI.
const Eigen::VectorXd qddot_sol = solution.qddot_sol;

// Convert MPPI acceleration result into command references.
Eigen::VectorXd qdot_cmd =
    clamp_velocity(qdot_cmd_prev + dt * qddot_sol);

Eigen::VectorXd q_cmd =
    clamp_position(q_cmd_prev + dt * qdot_cmd);

// Model-based feedforward torque.
const Eigen::VectorXd tau_ff_cmd_Nm =
    robot.inverse_dynamics(state.q, state.qdot, qddot_sol);

// Optional host-side task feedback.
Eigen::VectorXd tau_fb_cmd_Nm = Eigen::VectorXd::Zero(robot.nv());

if (use_host_task_feedback) {
  tau_fb_cmd_Nm =
      kp_task.cwiseProduct(q_cmd - state.q)
    + kd_task.cwiseProduct(qdot_cmd - state.qdot);
}

Eigen::VectorXd tau_cmd_Nm = tau_ff_cmd_Nm + tau_fb_cmd_Nm;
tau_cmd_Nm = clamp_torque(tau_cmd_Nm);

// Build final command.
RobotCommand cmd;
cmd.q_cmd = q_cmd;
cmd.qdot_cmd = qdot_cmd;
cmd.tau_cmd = convert_torque_to_driver_units(tau_cmd_Nm);

// Driver-local gains are attached here, not optimized by MPPI.
cmd.kp = config.driver_gains.kp;
cmd.kd = config.driver_gains.kd;
```

Example where MPPI directly produces torque:

```cpp
const MppiSolution solution = mppi.solve(state, rollout_model);

// Solver-selected torque.
const Eigen::VectorXd tau_sol_Nm = solution.tau_sol;

// After host-side limits, this becomes the torque command.
const Eigen::VectorXd tau_cmd_Nm = clamp_torque(tau_sol_Nm);

RobotCommand cmd;
cmd.q_cmd = hold_or_integrate_position_reference(state);
cmd.qdot_cmd = hold_or_integrate_velocity_reference(state);
cmd.tau_cmd = convert_torque_to_driver_units(tau_cmd_Nm);

// Driver-local gains are still copied from configuration.
cmd.kp = config.driver_gains.kp;
cmd.kd = config.driver_gains.kd;
```

---

## 18. Recommended YAML Naming

Use explicit names for task-side gains.

Good:

```yaml
state_machine:
  states:
    - id: 0
      name: initialize
      params:
        target_jpos: 0.0
        kp_task: 0.0
        kd_task: 0.0

    - id: 1
      name: hold
      params:
        kp_task: 0.0
        kd_task: 0.0
```

Use a separate block for driver-local gains.

Good:

```yaml
driver_gains:
  # Driver-local impedance gains copied into RobotCommand.kp / RobotCommand.kd.
  # These are sent to the embedded driver and are separate from host-side task gains.
  kp: [0.0, 0.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
  kd: [0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

Avoid:

```yaml
state_machine:
  states:
    - id: 0
      name: initialize
      params:
        kp: 0.0  # Ambiguous: task gain or driver gain?
        kd: 0.0
```

Acceptable only when the surrounding schema makes the meaning unambiguous:

```yaml
driver_gains:
  kp: [...]
  kd: [...]
```

Because the parent key already states that these are driver-local gains.

---

## 19. Rule of Thumb

General suffixes:

```txt
_des       = wanted by task/objective
_sol       = solved by optimizer/solver
_cmd       = passed from host to driver
_meas      = reported by robot, sensor, or driver
_est       = estimated or filtered
_applied   = physically applied ground-truth-like value
q/qdot/tau = accepted current state
```

Torque-specific names:

```txt
tau_ff_cmd = pure model-based feedforward torque command
tau_fb_cmd = host-side task feedback torque command
tau_cmd    = final torque command passed to driver
tau_meas   = measured/reported torque, often current_meas * Kt
```

Gain-specific names:

```txt
kp_task / kd_task = host-side task feedback gains
cmd.kp / cmd.kd   = driver-local gains inside RobotCommand
```

Architecture rule:

```txt
Planner / task / state / MPPI does not tune driver-local kp/kd.

Driver-local kp/kd are attached during command finalization from configuration.
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

Bad: copying host-side task gains into `cmd.kp` or `cmd.kd`.

Good:

```cpp
cmd.kp = config.driver_gains.kp;
cmd.kd = config.driver_gains.kd;
```

Bad:

```cpp
state.tau_applied = driver_feedback.tau_meas;  // Bad: measured estimate is not applied ground truth.
```

Good:

```cpp
state.tau = torque_filter.update(driver_feedback.tau_meas);
```

---

## 20. Minimal Naming Checklist

Before introducing a new variable, ask:

```txt
Is this a desired objective?
  -> use _des

Was this computed by a solver?
  -> use _sol

Will this be sent to the driver?
  -> use _cmd

Was this reported by hardware/sensor/driver?
  -> use _meas

Was this estimated or filtered as an intermediate quantity?
  -> use _est

Is this physically applied ground-truth-like data?
  -> use _applied

Is this the accepted current state for this tick?
  -> use q, qdot, tau without suffix
```

For gains, ask:

```txt
Is this used on the host to compute tau_fb_cmd?
  -> use kp_task / kd_task

Is this inside RobotCommand and sent to the embedded driver?
  -> use cmd.kp / cmd.kd
```

For torque, ask:

```txt
Is this pure model-based inverse dynamics?
  -> tau_ff_cmd

Is this host-side feedback torque?
  -> tau_fb_cmd

Is this the final torque passed to the driver?
  -> tau_cmd
```
