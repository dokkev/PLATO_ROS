# MPPI Direction: Disturbance-Sampled GraspState Control

## 1. Core Idea

This package implements a **disturbance-sampled GraspState-predictive MPPI controller** for tactile contact manipulation.

The controller is **not** designed to predict exact future contact force.

Instead, it samples possible future grasp outcomes under tactile/contact disturbances and chooses actions that keep the grasp stable.

The central question is not:

```txt
What exact force will occur in the future?
```

The central question is:

```txt
If this joint acceleration action is applied,
and the object/contact state is disturbed in plausible ways,
will the GraspState remain stable?
```

More specifically:

```txt
Will currently active tactile hemispheres remain active?
Will inactive hemispheres become useful new contacts?
Will contact support increase, decrease, or shift?
Will shear, rotation, or contact-loss risk increase?
Will the robot action help recover contact support under disturbance?
```

The key prediction target is therefore:

```txt
future GraspState under sampled contact disturbances
```

not exact future contact force.

The key geometric signal is:

```txt
per-hemisphere contact kinematics / contact Jacobian
```

because this tells us whether an action moves tactile hemispheres toward contact, away from contact, or tangentially across the contact region.

---

## 2. What the Rollout Predicts

The top-level rollout state is `GraspState`.

```txt
GraspState = RobotState + vector<TactileState>
```

Mathematically:

```txt
G_k = (R_k, T^0_k, T^1_k, ..., T^{N-1}_k)
```

where:

```txt
R_k   = RobotState
T^s_k = TactileState for tactile sensor s
```

The rollout predicts:

```txt
G_{k+1} = f_G(G_k, u_k, w_k)
```

where:

```txt
u_k = qddot_sol,k
w_k = sampled tactile/contact disturbance
```

So the rollout is not a single deterministic future.

For each candidate action sequence, MPPI should eventually evaluate multiple possible disturbance futures:

```txt
action sample i:
  u_0^i, u_1^i, ..., u_{H-1}^i

disturbance sample j:
  w_0^j, w_1^j, ..., w_{H-1}^j

rollout:
  G_{k+1}^{i,j} = f_G(G_k^{i,j}, u_k^i, w_k^j)
```

The objective is to choose an action sequence that performs well across plausible disturbed futures.

---

## 3. State Definition

Each `TactileState` represents one tactile sensor.

```txt
TactileState
  ├── sensor-level aggregate signals
  │     ├── contact_state
  │     ├── total_force_n
  │     ├── shear_displacement_m
  │     ├── rotational_shear_rad
  │     └── slip / confidence scores
  │
  └── vector<HemisphereState>
        ├── hemisphere_index
        ├── contact
        ├── cop_sensor_m
        ├── normal_force_n
        └── confidence
```

A `HemisphereState` is the MPPI-level tactile reasoning unit.

Raw tactile nodes are considered a lower-level sensor implementation detail and are not directly used by the MPPI rollout.

The active hemisphere set for tactile sensor `s` is:

```txt
A^s_k = { i | hemisphere_i.contact == true }
```

The tactile topology transition is conceptually:

```txt
A^s_{k+1} = (A^s_k - lost hemispheres) ∪ newly activated hemispheres
```

This means the rollout reasons about:

```txt
survival:
  an active hemisphere remains in contact

loss:
  an active hemisphere loses contact

birth / discovery:
  an inactive hemisphere becomes active
```

---

## 4. RobotState Semantics

`RobotState` is intentionally MPPI-agnostic.

```cpp
struct RobotState {
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd tau;
};
```

The meaning of `tau` depends on context.

For real robot feedback:

```txt
tau = accepted current torque
```

This may include:

```txt
motor-current-based torque measurement
embedded feedback effects
host feedback effects
contact reaction
friction
filtering
unmodeled dynamics
```

For MPPI rollout:

```txt
tau = model torque predicted by RNEA
```

In rollout, `tau` is used as a model effort / feasibility proxy.

Do not interpret rollout `RobotState::tau` as:

```txt
tau_cmd
tau_ff_cmd
tau_fb_cmd
tau_meas
tau_applied
```

Those names belong to command, measurement, or applied-torque layers.

---

## 5. MPPI Action

The MPPI action is solver joint acceleration:

```txt
u_k = qddot_sol,k
```

The sampled action sequence is:

```txt
U = (qddot_sol,0, qddot_sol,1, ..., qddot_sol,H-1)
```

`qddot_sol` is treated as the solver output selected by MPPI.

The command builder later converts the first selected `qddot_sol` into driver commands.

The rollout itself does not build the final driver packet.

---

## 6. Robot Rollout

The robot rollout assumes ideal tracking of the sampled acceleration command.

```txt
qdot_{k+1} = qdot_k + qddot_sol,k * dt

q_{k+1} = integrate(q_k, qdot_{k+1} * dt)

tau_k = RNEA(q_k, qdot_k, qddot_sol,k)
```

`EvaluateRollout()` and `PredictRollout()` initialize the root rollout
`RobotState` from:

```txt
q_ref_current
qdot_ref_current
tau = 0
```

This means the MPPI horizon extends the accepted host reference trajectory.
Measured `q_meas`, `qdot_meas`, and `tau_meas` remain observation-layer values;
they are not mixed into the rollout root `RobotState`.

In rollout:

```txt
tau_k = model torque / effort proxy
```

The rollout does not simulate embedded driver impedance control.

The embedded driver may later apply:

```txt
tau_driver_internal =
    tau_cmd
  + kp * (q_cmd - q_meas)
  + kd * (qdot_cmd - qdot_meas)
```

but this is not part of the MPPI rollout dynamics.

---

## 7. Tactile Rollout

For each tactile sensor:

```txt
T^s_{k+1} = f_T(T^s_k, R_k, R_{k+1}, qddot_sol,k, w^s_k)
```

where `w^s_k` is the sampled disturbance for tactile sensor `s`.

The tactile rollout predicts how contact support changes under both:

```txt
1. robot motion caused by qddot_sol
2. sampled contact/object/tactile disturbance
```

The rollout should evaluate:

```txt
active hemisphere survival
active hemisphere loss
inactive hemisphere birth
contact support shift
shear / rotational slip risk
confidence decay
normal force proxy change
```

The current implementation uses a deterministic contact-kinematic transition.
It does not use measured-torque residual projection inside MPPI horizon
dynamics.

The next step is to add explicit disturbance sampling.

---

## 8. Contact Jacobian as the Core Signal

The central geometric question is:

```txt
Does this action move inactive hemispheres toward contact,
and does it keep active hemispheres from unloading?
```

For each hemisphere `i`, Pinocchio provides a contact point Jacobian:

```txt
v_i = J_i(q) qdot
```

The normal component is:

```txt
v_i^n = n_i(q)^T J_i(q) qdot
```

The rollout convention is:

```txt
normal_velocity > 0 : closing / contact approach
normal_velocity < 0 : separating / unloading
```

`normal_axis_sign` exists to keep this convention stable when a tactile sensor
frame has the opposite local normal direction.

A candidate action changes joint velocity:

```txt
qdot_{k+1} = qdot_k + qddot_sol,k * dt
```

Therefore the rollout can evaluate whether a hemisphere is moving toward or away from the local contact direction.

This is more important than predicting exact future force.

The MPPI cost should prefer actions that:

```txt
increase useful active hemisphere support
maintain existing contact
avoid unloading active hemispheres
move inactive neighboring hemispheres toward plausible contact
avoid excessive shear or rotational slip
avoid torque/action limits
```

---

## 9. Pinocchio Contact Kinematics

We use **one Pinocchio frame per tactile sensor**, not one frame per hemisphere.

Hemisphere geometry is stored separately from `TactileState`.

```txt
TactileState:
  what is measured or predicted

TactileSensorGeometry / TactileSensorContext:
  where hemispheres are in the sensor frame

Pinocchio:
  how those points move with q and qdot
```

A hemisphere contact point is computed from the tactile sensor frame and local hemisphere point.

```txt
p_i^W(q) = p_S^W(q) + R_S^W(q) r_i^S
```

The point Jacobian is derived from the sensor frame spatial Jacobian.

```txt
J_i^c(q) = J_S^v(q) - [R_S^W r_i^S]_x J_S^ω(q)
```

The normal Jacobian is:

```txt
J_i^n(q) = n_i^W(q)^T J_i^c(q)
```

This allows MPPI to connect candidate joint acceleration to local contact motion.

---

## 10. Current Force vs Future Force

Current contact force is provided by tactile sensing.

The current `TactileState` may contain:

```txt
total_force_n
normal_force_n per hemisphere
contact_state
shear_displacement_m
rotational_shear_rad
```

These measured tactile values are part of the current GraspState.

Future contact force is not predicted exactly.

Instead, future rollout uses:

```txt
RobotState::tau = RNEA model torque
per-hemisphere contact Jacobian
tactile transition heuristics
sampled disturbance
```

to estimate whether contact support is likely to be maintained, lost, or expanded.

Torque may be used as:

```txt
effort cost
torque limit cost
preload / contact-maintenance proxy
```

but not as ground-truth future contact force.

---

## 11. Disturbance-Sampled GraspState MPPI

The controller should be understood as a disturbance-sampled grasp controller.

The disturbed rollout equation is:

```txt
G_{k+1}^{i,j} = f_G(G_k^{i,j}, u_k^i, w_k^j)
```

where:

```txt
i = action sample index
j = disturbance sample index
u_k = qddot_sol,k
w_k = sampled tactile/contact disturbance
```

Possible tactile/contact disturbances include:

```txt
contact hemisphere loss
new hemisphere contact birth
contact patch shift in sensor frame
shear displacement perturbation
rotational slip perturbation
normal force fluctuation
object pose drift
contact confidence decay
```

The cost estimates robustness over sampled future outcomes:

```txt
J(U) = E_W [ terminal_cost(G_H) + Σ stage_cost(G_k, u_k) ]
```

MPPI approximates this expectation through sampled rollouts.

Practical robust cost aggregations include:

```txt
mean cost:
  J_i = mean_j(J_{i,j})

mean + risk:
  J_i = mean_j(J_{i,j}) + risk_weight * std_j(J_{i,j})

CVaR / worst-fraction:
  J_i = mean of worst p% disturbance rollout costs
```

The initial implementation may use deterministic rollout.
The next robustness implementation should add disturbance samples and aggregate cost across those disturbance samples.

---

## 12. Residual Force Projection

Residual torque projection is currently **not the primary rollout mechanism**.

A residual is defined only from real observed robot state:

```txt
tau_residual = tau_observed - RNEA(q_observed, qdot_observed, qddot_base)
```

It is a noisy evidence signal, not ground-truth contact force.

It may contain:

```txt
contact reaction
embedded feedback torque
host feedback torque
friction
model error
sensor noise
unmodeled dynamics
```

Therefore residual should be used later as **observation-time GraspState correction**, not as the main horizon dynamics.

Current policy:

```txt
Residual projection is disabled in the main rollout.
The rollout should work without residual force projection.
```

Future policy:

```txt
Use residual to correct the current GraspState belief before rollout.
Do not inject the same measured residual repeatedly across the rollout horizon.
```

If residual is later projected into contact forces, multi-sensor contact must be handled jointly:

```txt
tau_residual ≈ Σ_s Σ_i J_{s,i}^T f_{s,i}
```

Do not project the same residual independently into each tactile sensor.

---

## 13. Cost Direction

The cost should evaluate GraspState robustness and quality.

Useful cost terms include:

```txt
robot/action costs:
  qddot magnitude
  qdot limit
  tau effort / torque limit
  action smoothness

tactile costs:
  contact loss
  insufficient active hemispheres
  insufficient active tactile sensors
  excessive shear displacement
  excessive rotational shear
  low tactile confidence
  unstable contact transition

multi-sensor costs:
  too few active tactile sensors
  unbalanced contact support
  asymmetric contact loss
```

The cost should not depend on exact future force prediction.

For disturbance-sampled rollout, the final cost for an action sequence should reflect not only average performance, but also robustness under worse disturbance outcomes.

---

## 14. Command Layer Boundary

The rollout does not directly define the final driver command.

The MPPI output is the first selected acceleration:

```txt
qddot_sol,0
```

A separate command builder converts this into driver command values.

```txt
qddot_sol
  -> integrate and clamp
  -> q_cmd, qdot_cmd

qddot_sol
  -> inverse dynamics
  -> tau_ff_cmd

optional host-side feedback:
  tau_fb_cmd = kp_fb * (q_cmd - q)
             + kd_fb * (qdot_cmd - qdot)

final driver torque:
  tau_cmd = tau_ff_cmd + tau_fb_cmd
```

The final driver packet contains:

```txt
q_cmd
qdot_cmd
tau_cmd
kp
kd
```

Here:

```txt
kp, kd       = driver-local impedance gains
kp_fb, kd_fb = host-side feedback gains
```

Do not mix these two gain layers.

---

## 15. Config Surface

Sampling budget, acceleration action bounds, and command gains live in:

```txt
mppi_core/config/mppi.yaml
```

Contact-local cost and tactile transition parameters live in:

```txt
mppi_core/config/grasp.yaml
```

The current tactile transition section is:

```yaml
grasp:
  tactile_transition:
    enable_birth: true
    enable_loss: true
    birth_score_threshold: 0.5
    loss_score_threshold: 0.5
    birth_neighbor_weight: 0.25
    birth_tangent_approach_weight: 1.0
    birth_normal_approach_weight: 0.5
    birth_shear_penalty_weight: 0.5
    loss_unloading_weight: 1.0
    loss_shear_weight: 0.5
    loss_low_force_weight: 0.5
    born_normal_force_n: 0.05
    born_confidence: 0.5
    inactive_confidence: 0.0
    contact_confidence_decay: 1.0
    aggregate_shear_decay: 1.0
    aggregate_rotation_decay: 1.0
    enough_contact_hemisphere_count: 2
```

These values are parsed into `TactileTransitionConfig`.

---

## 16. Current Implementation Priorities

The current priority is to stabilize the architecture:

```txt
1. GraspState = RobotState + vector<TactileState>
2. TactileState = sensor aggregate + vector<HemisphereState>
3. MPPI action = qddot_sol
4. Robot rollout = ideal q/qdot integration + RNEA tau
5. Contact kinematics = per-hemisphere motion/Jacobian from sensor frame
6. Tactile rollout = hemisphere contact topology transition
7. Residual projection = disabled in rollout
8. Command builder = separate from rollout
```

After this architecture is stable, add:

```txt
1. disturbance sampling in tactile transition
2. robust cost aggregation across disturbance samples
3. optional observation-time residual correction
4. optional object-prior-based contact birth
```

Do not add object pose tracking or multi-sensor residual force solving until the above architecture is stable.

---

## 17. Non-Goals

This controller is not:

```txt
a full rigid-body contact simulator
an exact future contact force predictor
an object pose tracker
a residual-force estimator as the main controller
a driver impedance simulator
```

It is:

```txt
a disturbance-sampled GraspState-predictive MPPI controller
that samples possible future tactile/contact outcomes
and chooses actions that preserve or improve grasp stability.
```
