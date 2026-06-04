# Controller Variable Naming Convention

This document defines controller variable naming conventions for robot control
code. The goal is to keep the control pipeline explicit: what a task wants,
what a solver computed, what the host passed to hardware, what the robot
reported back, and what the host accepted as the current state.

In ideal tracking, several values may be numerically equal. In real robot
software, they still belong to different layers and should not be collapsed.

## 1. Core Rule

Use suffixes to describe the control layer of a value.

```txt
_des     = what a task, planner, or controller objective wants
_sol     = what a solver computed
_cmd     = what the host passed to the embedded driver
_applied = what the embedded controller or lower-level driver actually used
_meas    = what the robot, sensor, or driver reported back
_est     = estimated or filtered intermediate value
q        = what the host controller accepts as the current robot state
```

Use unsuffixed `q`, `qdot`, `qddot`, and `tau` only for the canonical current
robot state used by kinematics, dynamics, estimation, and feedback control.

## 2. Why These Layers Are Separate

These values may be equal in an ideal system, but they do not represent the same
concept.

For example, in whole-body control, the controller may receive multiple task
objectives:

```cpp
right_hand_task.x_des;
com_task.x_des;
posture_task.q_des;
```

Even if one objective is a joint-space desired value, such as
`posture_task.q_des`, the final solver output may not equal that value. The
solver may need to satisfy task conflicts, contact constraints, torque limits,
balance constraints, joint limits, or higher-priority objectives.

Therefore:

```txt
q_des != q_sol
```

The solver result is then converted into a host-to-driver command. This
conversion may include actuator limits, hardware limits, safety clamps, command
smoothing, interpolation, or mode-dependent behavior.

Therefore:

```txt
q_sol != q_cmd
```

After the command is sent, the embedded controller or driver may still apply
additional processing. The measured robot state may differ from the command due
to tracking error, delay, compliance, backlash, saturation, or contact
interaction.

Therefore:

```txt
q_cmd != q_meas
```

The ideal chain is:

```txt
x_des -> q_sol -> q_cmd -> q_applied -> robot -> q_meas -> q
```

In an ideal system with no conflict, no delay, no saturation, perfect actuation,
and perfect sensing:

```txt
q_sol = q_cmd = q_applied = q_meas = q
```

Real robot software should still keep these names separate because each value
belongs to a different control layer.

## 3. Current State Values

Use unsuffixed `q`, `qdot`, `qddot`, and `tau` only for the canonical current
robot state accepted by the host controller for the current control tick.

```cpp
struct RobotState
{
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd tau;
};
```

Meaning:

```txt
q     = current robot configuration used by model/control
qdot  = current generalized velocity used by model/control
qddot = current generalized acceleration used by model/control, if available
tau   = current joint torque or effort used by model/control
```

These values may come from raw encoder feedback, calibrated measurements,
filtered velocity, sensor fusion, or estimator output. The unsuffixed name does
not describe how the value was obtained. It means the value has been accepted as
the current robot state.

Example:

```cpp
const JointFeedback feedback = read_joint_feedback_();

RobotState state;
state.q = apply_joint_calibration_(feedback.q_meas);
state.qdot = velocity_filter_.update(feedback.qdot_meas);
state.tau = feedback.tau_meas;

pinocchio::forwardKinematics(model, data, state.q, state.qdot);
```

Use `*_meas` for values reported by hardware or a lower-level driver:

```cpp
q_meas
qdot_meas
tau_meas
```

`*_meas` means the value came from the hardware or driver interface. It does not
necessarily mean the value is raw. For example, `qdot_meas` may already be
filtered by the embedded driver.

Use `*_est` or more specific names for explicit host-side preprocessing:

```cpp
q_calibrated
qdot_filtered
tau_est
```

## 4. Desired Values

Use `*_des` for values desired by a task, planner, or controller objective.

```cpp
x_des
xdot_des
q_des
qdot_des
```

These values describe what a task wants before arbitration or conflict
resolution.

Good:

```cpp
posture_task.q_des = nominal_posture;
hand_task.x_des = target_hand_pose;
```

Bad:

```cpp
q_des = solve_wbc(problem);  // Bad: this is a solver result.
```

Do not use `*_des` for solver outputs, host-to-driver commands, or measured
state.

## 5. Solver Results

Use `*_sol` for raw results from solvers such as IK, QP, WBC, MPC, inverse
dynamics, or trajectory optimization.

```cpp
q_sol
qdot_sol
qddot_sol
tau_sol
contact_force_sol
```

These values mean:

```txt
The solver computed this value.
```

They do not necessarily mean:

- the host sent it to the driver
- the embedded controller accepted it
- the robot physically applied it
- the measured robot state reached it

Example:

```cpp
const WbcSolution solution = solve_wbc_(problem, state);

solution.qddot_sol;
solution.tau_sol;
solution.contact_force_sol;
```

## 6. Host-to-Driver Commands

Use `*_cmd` for values that the host-side controller passes to a lower-level
driver or embedded controller.

```cpp
q_cmd
qdot_cmd
tau_ff_cmd
kp_cmd
kd_cmd
```

Meaning:

```txt
*_cmd = value passed from the host to the driver interface
```

`*_cmd` does not mean the actuator physically applied the value. It only means
the host provided that value to the driver interface.

Use explicit suffixes inside command structs:

```cpp
struct JointCommand
{
  Eigen::VectorXd q_cmd;
  Eigen::VectorXd qdot_cmd;
  Eigen::VectorXd tau_ff_cmd;
  Eigen::VectorXd kp_cmd;
  Eigen::VectorXd kd_cmd;
};
```

Typical host-side flow:

```cpp
const WbcSolution solution = solve_wbc_(problem, state);

JointCommand command = build_joint_command_(solution, state);
JointCommand limited_command = apply_host_limits_(command);

if (driver_.write(limited_command)) {
  last_sent_command_ = limited_command;
}
```

Use `last_sent_command_` for the last command successfully written to the
transport or driver interface.

## 7. Applied Values Are Rarely Observable

Use `*_applied` only for the value that the embedded controller or lower-level
driver actually used as its internal command after embedded-side processing.

For example:

```txt
q_cmd -> embedded processing -> q_applied
```

`q_cmd` is the value passed from the host to the embedded controller.
`q_applied` is the value actually used inside the embedded controller after
internal processing.

Embedded-side processing may include:

- joint limit clamping
- velocity or acceleration limits
- current or torque limits
- watchdog cutoffs
- mode gates
- command filtering
- interpolation
- thermal protection
- fault handling

Therefore, host-side code should not assume:

```txt
q_applied = q_cmd
```

Bad:

```cpp
if (driver.write(command)) {
  last_applied_command_ = command;  // Incorrect unless embedded confirms it.
}
```

Good:

```cpp
if (driver.write(command)) {
  last_sent_command_ = command;
}
```

Use `applied_command` or `q_applied` only if the embedded controller explicitly
reports or echoes the value it actually used internally.

If the host cannot observe the embedded-side applied value, use:

```cpp
last_sent_command_
```

not:

```cpp
last_applied_command_
```

In short:

```txt
cmd     = host passed it to the driver
applied = embedded actually used it internally
meas    = robot or sensor reported it back
q       = host accepted it as current state
```

## 8. Rule of Thumb

```txt
des     = wanted
sol     = solved
cmd     = passed to driver
applied = embedded actually used
meas    = reported back
est     = estimated or filtered
q       = accepted current state
```

Anti-patterns:

```cpp
q_cmd = solve_ik(x_des);  // Bad: solver output skipped the _sol layer.
```

```cpp
q_sol = solve_ik(x_des);
q_cmd = build_command_from_solution_(q_sol, state);
```

```cpp
last_applied_command_ = command;  // Bad unless embedded confirms it.
```

```cpp
last_sent_command_ = command;
```

Ideal systems may make these values numerically equal. Real robot software
should still name them as different layers.
