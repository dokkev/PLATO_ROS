# Jenga Continuous qddot MPPI TODO

This TODO tracks the Jenga object-prior grasp controller. The current main path is now continuous qddot MPPI:

> Given a known Jenga block geometry and approximate initial pose, sample continuous joint-acceleration sequences and object pose/disturbance perturbations.
> Roll out each qddot sequence over a short horizon, use Pinocchio to compute hemisphere positions, evaluate geometric contact/support against sampled Jenga poses, compute MPPI soft weights, and apply only the first weighted qddot command.

This is **not** a true object dynamics simulator.
This is a **robust contact-support scenario evaluator**.
The old discrete action selector is allowed only as a fallback/baseline/debug comparison.

---

## 0. Core Philosophy

### 0.1 Do not overclaim prediction

* [x] Do not claim that the controller predicts the true future object motion.
* [x] Do not claim that Pinocchio predicts true contact force.
* [x] Do not claim that future contact sequence is known.
* [x] Treat rollout results as **scenario evaluation**, not ground-truth prediction.

Preferred wording:

```text
We evaluate candidate hand actions under sampled Jenga block pose perturbations
using geometric hemisphere-object contact queries.
```

Avoid:

```text
We predict exact future contact force.
We predict exact object motion.
Pinocchio simulates contact.
```

---

### 0.2 MVP goal

The first demo target is:

```text
2-finger Jenga grasp hold under object pose/disturbance perturbations.
```

Success means:

* [x] Continuous qddot sequence sampling is the default main path.
* [x] MPPI uses soft weighted averaging, not best-sample selection.
* [x] Stable contact with no disturbance and zero noise produces near-zero qddot.
* [ ] Low preload produces closing/squeezing qddot.
* [ ] One weak side biases the weighted qddot toward weak-side squeeze.
* [ ] Contact near edge biases the weighted qddot toward align/centering.
* [ ] Controller does not endlessly squeeze.
* [ ] Controller does not twitch when contact is stable.

---

## 1. Use Known Jenga Prior

### 1.1 Object model

* [x] Add/use Jenga block as a primitive box object.
* [x] Use box dimensions from task YAML.
* [x] Use approximate initial pose from task YAML.
* [x] Treat initial pose as a perturbation center, not exact ground truth.

Example config:

```yaml
task:
  object:
    name: jenga_block
    geometry_type: box
    primitive_size_m: [0.075, 0.025, 0.015]  # example only
    initial_pose_world:
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
    perturbation:
      xyz_std_m: [0.005, 0.005, 0.005]
      rpy_std_rad: [0.05, 0.05, 0.1]
```

---

### 1.2 Do not require full mesh/FCL for MVP

* [x] Use primitive point-to-box signed distance first.
* [x] Defer mesh/FCL distance query.
* [x] Defer full URDF collision handling.
* [x] Defer exact contact force simulation.

Required signed distance convention:

```text
d > 0 : point is outside object / gap
d = 0 : point is on object surface
d < 0 : point is inside object / penetration
```

---

## 2. Pinocchio Role

### 2.1 Pinocchio is only hand-side kinematics/dynamics

Pinocchio should provide:

* [x] hand/finger frame poses
* [x] hemisphere world positions
* [ ] hemisphere world velocities if needed
* [ ] optional hand RBD rollout

Pinocchio should **not** be treated as:

* [x] a contact simulator
* [x] a contact force predictor
* [x] an object dynamics simulator

Preferred wording:

```text
Pinocchio computes the hand and hemisphere kinematics.
Geometric signed-distance queries evaluate contact feasibility against sampled Jenga poses.
```

---

### 2.2 Hemisphere position query

For each candidate action and rollout step:

* [x] Roll out hand state `q_next`.
* [x] Run Pinocchio FK once per rollout step.
* [x] Compute all relevant hemisphere center positions in world frame.
* [x] Do not recompute full FK per hemisphere.

Expected flow:

```cpp
pinocchio::forwardKinematics(model, data, q, qdot);
pinocchio::updateFramePlacements(model, data);
pinocchio::computeJointJacobians(model, data, q);
```

---

## 3. Object Perturbation Sampling

### 3.1 Replace heavy object belief with perturbation samples for MVP

Instead of full contact-consistent particle filtering, start with:

```text
T_obj_sample = T_obj_initial + sampled pose perturbation
```

Checklist:

* [x] Sample translation perturbations around initial pose.
* [x] Sample rotation perturbations around initial pose.
* [x] Sample optional object velocity perturbations.
* [x] Keep sample count small at first.

Recommended MVP:

```yaml
robust_grasp:
  rollout:
    num_rollouts: 128
    horizon_steps: 3
  cost:
    object_support:
      object_pose_samples: 16
```

Then increase:

```yaml
robust_grasp:
  rollout:
    num_rollouts: 256
    horizon_steps: 3
  cost:
    object_support:
      object_pose_samples: 32
```

---

### 3.2 Perturbation dimensions

Sample:

* [x] `x`, `y`, `z` pose noise
* [x] roll/pitch/yaw pose noise
* [x] optional tangential slip velocity
* [x] optional pull-away velocity
* [x] optional yaw/roll rotation disturbance

Do not start with full 6D object dynamics.

---

### 3.3 Optional tactile contact consistency

For MVP, object samples can be centered around initial pose.

Later improvement:

* [x] Weight object perturbation samples by tactile contact consistency.
* [x] Prefer samples where measured tactile contact points lie near object surface.
* [x] Prefer samples where tactile compression normal and object surface normal are roughly opposite.

But this is **not required for first grasp hold demo**.

---

## 4. Geometry Contact Evaluation

### 4.1 Per-hemisphere signed distance

For each sampled object pose and each hemisphere:

```cpp
p_hemi_world = ComputeHemisphereWorldPosition(q_next, hemi);
d = SignedDistanceToJengaBox(T_obj_sample, p_hemi_world);
gap = d - hemisphere_radius_m;
```

Checklist:

* [x] Compute signed distance for active hemispheres.
* [x] Compute signed distance for inactive but nearby hemispheres.
* [x] Convert center signed distance to hemisphere surface gap.
* [x] Penalize excessive penetration separately from near-contact support.
* [x] For MVP, computing all 3x4 hemispheres per fingertip is acceptable.
* [x] Add contact margin.

Contact condition:

```cpp
predicted_contact = gap < contact_margin_m;
```

---

### 4.2 Contact birth

Replace neighbor-heuristic contact birth with geometry-based contact birth.

Required:

* [x] A new hemisphere can become contact if its signed distance falls below threshold.
* [x] Contact birth does not require an active neighbor.
* [x] Neighbor relationship may only be used for support smoothing or visualization.

Good explanation:

```text
A new hemisphere is predicted to be in contact when the sampled Jenga block surface
is within the contact margin of that hemisphere center.
```

---

### 4.3 Contact loss

A hemisphere contact is predicted lost when:

```cpp
d > contact_loss_margin_m;
```

Checklist:

* [x] Contact loss margin is configurable.
* [x] Contact birth margin and loss margin can include hysteresis.
* [x] Loss of all contact has high cost.
* [x] Loss of one finger contact has meaningful cost.

---

## 5. Contact Support Metrics

### 5.1 Predicted contact mask

For each fingertip:

* [ ] Build a 3x4 predicted contact mask.
* [ ] Build a measured contact mask from tactile sensor.
* [ ] Log both masks.

Example:

```text
measured:
. X . .
. X . .
. . . .

predicted:
. X X .
. X X .
. . . .
```

---

### 5.2 Predicted support count

* [x] Compute predicted active hemisphere count.
* [x] Penalize support deficit.
* [x] Do not blindly maximize active count.

Cost:

```cpp
support_deficit =
    std::max(0, target_contact_count - predicted_contact_count);
```

---

### 5.3 Predicted contact centroid

Compute force- or mask-weighted centroid on the 3x4 grid.

MVP:

```cpp
centroid = average(position_2d of predicted_contact hemispheres);
```

Later:

```cpp
centroid = weighted_average(position_2d, predicted_force);
```

Checklist:

* [ ] Compute measured centroid.
* [x] Compute predicted centroid.
* [ ] Log both.
* [x] Use centroid for edge-risk cost.

---

### 5.4 Edge margin

For each fingertip:

* [x] Compute distance from predicted contact centroid to tactile grid boundary.
* [x] Penalize small edge margin.
* [ ] Penalize contact moving toward edge if history is available.

Cost:

```cpp
edge_cost =
    std::max(0.0, target_edge_margin_m - predicted_edge_margin_m);
```

---

### 5.5 Support area/spread

MVP:

* [x] Approximate support area from predicted active count.
* [ ] Or approximate area from bounding box of predicted active hemispheres.

Later:

* [ ] Use covariance/spread of predicted contact positions.
* [ ] Use support polygon/convex hull if needed.

---

## 6. Contact Force Approximation

### 6.1 MVP force model

Use signed distance as contact support proxy first.

MVP force approximation:

```cpp
f_pred = k_contact * std::max(0.0, contact_margin_m - d);
```

Checklist:

* [x] Predicted force increases as object gets closer/penetrates.
* [x] Predicted force is clamped to safe max.
* [x] Excessive predicted force is penalized.
* [x] Measured tactile force is still used for current preload cost.

---

### 6.2 Do not overfocus on exact force prediction

For first demo, prioritize:

* [x] contact support count
* [x] contact loss
* [x] edge margin
* [x] measured preload
* [x] force imbalance

Exact predicted contact force is secondary.

---

## 7. Candidate Actions

### 7.1 MVP action set

Use a small action library first:

* [x] `hold`
* [x] `squeeze_symmetric`
* [x] `release_slight`
* [x] `thumb_squeeze_only`
* [x] `index_squeeze_only`
* [x] `align_contact_line_positive`
* [x] `align_contact_line_negative`

Optional combined actions:

* [x] `squeeze + align_positive`
* [x] `squeeze + align_negative`

---

### 7.2 Actions are evaluated, not hard-coded

Do not implement:

```cpp
if (thumb_force_low) {
  return thumb_squeeze;
}
```

Instead:

```text
Generate candidate actions.
Evaluate each candidate under sampled Jenga perturbations.
Select the lowest-cost action.
```

Checklist:

* [x] `hold` is always available.
* [x] Candidate action names are logged.
* [x] Best action and second-best action are logged.
* [x] Hold cost is logged.

---

## 8. Scenario Evaluation Rollout

### 8.1 MVP horizon

Start with a short horizon.

```text
candidate action -> q_next -> hemisphere positions -> object signed distances -> support cost
```

Then test horizon 2-3.

Do not start with long horizon.

Current continuous MPPI default:

```yaml
robust_grasp:
  rollout:
    num_rollouts: 128
    horizon_steps: 3
```

Later:

```yaml
robust_grasp:
  rollout:
    num_rollouts: 256
    horizon_steps: 3
```

---

### 8.2 Evaluation loop

Expected pseudo-code:

```cpp
for (const auto& action : candidate_actions) {
  double action_cost = 0.0;

  for (const auto& object_sample : sampled_jenga_poses) {
    RobotState next_robot = RolloutHand(state.robot, action, dt);

    ContactSupportField support;

    for (const auto& sensor : tactile_sensors) {
      for (const auto& hemi : sensor.hemispheres) {
        Eigen::Vector3d p_hemi_world =
            ComputeHemisphereWorldPosition(next_robot.q, sensor, hemi);

        double d =
            SignedDistanceToJengaBox(object_sample.pose_world, p_hemi_world);

        bool predicted_contact =
            d < contact_margin_m;

        double f_pred =
            k_contact * std::max(0.0, contact_margin_m - d);

        support.Add(sensor, hemi, predicted_contact, f_pred, d);
      }
    }

    action_cost += ContactSupportCost(support, measured_tactile_state);
  }

  action_cost /= sampled_jenga_poses.size();
}
```

Optional robust score:

```cpp
score = CVaR(action_cost_samples);
```

---

## 8.3 Continuous qddot MPPI Main Path

The main controller must be honestly described as MPPI only when it uses continuous sequence sampling and soft weighting.

Checklist:

* [x] `control_mode: continuous_qddot_mppi` is the default for robust grasp.
* [x] `control_mode: discrete_action_selector` remains available as a baseline.
* [x] Sample continuous qddot sequences around a persistent nominal sequence.
* [x] Configurable `num_rollouts`, `horizon_steps`, `temperature`, qddot noise std, and noise clip.
* [x] Enforce qddot bounds through rollout action lower/upper bounds.
* [x] Enforce qdot bounds during rollout and command generation.
* [x] Clamp q to finite Pinocchio model position limits when available.
* [x] Roll out horizon length >= 2 for continuous MPPI.
* [x] Accumulate full sequence cost over the horizon.
* [x] Compute numerically stable MPPI weights with beta/min-cost offset.
* [x] Update the full sequence by weighted average, not best sample.
* [x] Apply only the first weighted qddot command.
* [x] Shift the nominal sequence forward and zero the tail each tick.
* [x] Log best sample separately from weighted command.
* [x] Log effective sample size.
* [ ] Benchmark runtime for 128x3 and 256x3 settings.
* [ ] Add regression where horizon 2-3 rejects an aggressive squeeze that penetrates later.

---

## 9. Cost Function

### 9.1 Required MVP cost terms

Use:

* [x] contact loss cost
* [x] support deficit cost
* [x] edge risk cost
* [x] measured preload low/high cost
* [x] force imbalance cost
* [x] action magnitude cost
* [x] action change/rate cost

Do not start with complicated slip dynamics.

---

### 9.2 Contact loss

* [x] Predicted no-contact state has very high cost.
* [x] Losing one finger contact has high cost.
* [x] Losing support while object perturbation is small should be strongly penalized.

---

### 9.3 Support deficit

Cost:

```cpp
support_cost =
    w_support * std::max(0, target_contact_count - predicted_contact_count);
```

Or support area:

```cpp
support_cost =
    w_support * std::max(0.0, target_support_area - predicted_support_area);
```

---

### 9.4 Edge risk

Cost:

```cpp
edge_cost =
    w_edge * std::max(0.0, target_edge_margin - predicted_edge_margin);
```

---

### 9.5 Preload

Use measured tactile force for current preload:

```cpp
preload_low_cost =
    w_preload_low * max(0, target_force - measured_force);

preload_high_cost =
    w_preload_high * max(0, measured_force - max_force);
```

Later, combine with predicted force proxy.

---

### 9.6 Force imbalance

For thumb-index grasp:

```cpp
balance_cost =
    w_balance * abs(F_thumb - F_index);
```

---

### 9.7 Action cost and hold bias

* [x] Penalize large action magnitude.
* [x] Penalize action switching.
* [x] Prefer hold when cost difference is small.

Recommended tie-break:

```cpp
if (best_cost > hold_cost - hold_margin) {
  selected_action = hold;
}
```

---

## 10. PID / Reflex Baseline

### 10.1 Build a strong baseline first

Implement or preserve:

* [ ] force PID / preload regulator
* [ ] weak-side squeeze reflex
* [ ] edge centering reflex
* [ ] preload high limit
* [ ] hold fallback

This baseline is required to justify MPPI.

---

### 10.2 MPPI should beat baseline only on tradeoffs

Do not claim MPPI is better at simple force tracking.

MPPI should help when:

* [ ] squeeze increases force but worsens edge margin
* [ ] weak-side squeeze worsens contact-line alignment
* [ ] align improves support but has weaker force recovery
* [ ] multiple sampled Jenga perturbations disagree
* [ ] current metric is ambiguous but short rollout separates actions

---

## 11. Logging Requirements

Must log:

* [x] selected action name
* [x] best action cost
* [x] hold cost
* [x] second-best cost
* [ ] per-cost breakdown:

  * contact loss
  * support
  * edge
  * preload
  * balance
  * action
* [x] measured active hemisphere count
* [x] predicted active hemisphere count
* [ ] measured contact centroid
* [ ] predicted contact centroid
* [x] sampled object perturbation summary
* [x] geometry query count
* [ ] solve time per tick

---

## 12. Visualization Requirements

Add debug visualization:

* [ ] measured 3x4 force grid
* [ ] predicted 3x4 contact grid
* [ ] predicted distance field over 3x4 grid
* [ ] contact centroid
* [ ] edge margin
* [ ] selected action
* [ ] hold cost vs best cost
* [ ] sampled Jenga pose cloud or range

---

## 13. Sanity Tests

### 13.1 Signed distance

* [x] Point outside Jenga box gives positive distance.
* [x] Point on surface gives near-zero distance.
* [x] Point inside box gives negative distance.
* [x] Normal points outward from object.

---

### 13.2 Contact birth

* [x] A hemisphere becomes predicted contact when its distance falls below margin.
* [x] Contact birth does not require active neighbor.
* [x] Contact disappears when distance grows beyond loss margin.

---

### 13.3 Hold test

* [x] With stable measured contact, no object perturbation, and zero noise, continuous MPPI returns near-zero qddot.
* [ ] With nonzero noise, stable measured contact converges nominal sequence toward near-zero qddot.
* [ ] If qddot does not settle near zero, inspect:

  * action cost
  * control/rate cost
  * MPPI temperature
  * effective sample size
  * contact margin
  * preload cost
  * gap convention

---

### 13.4 Squeeze test

* [ ] With low measured preload, `squeeze_symmetric` improves cost.
* [ ] With already high preload, squeeze does not win.
* [ ] With excessive predicted contact/penetration, squeeze is penalized.

---

### 13.5 Weak-side test

* [ ] If thumb force is low, thumb squeeze candidate improves cost.
* [ ] If index force is low, index squeeze candidate improves cost.
* [ ] This must happen through rollout cost, not direct if-else selection.

---

### 13.6 Edge test

* [ ] If predicted contact centroid is near edge, edge cost increases.
* [ ] If align action improves edge margin, align cost decreases.
* [ ] If contact is centered and stable, align should not beat hold.

---

### 13.7 Runtime test

* [ ] Measure runtime for:

  * 128 qddot samples
  * horizon 3
  * object samples per rollout 1
* [ ] Measure runtime for:

  * 256 qddot samples
  * horizon 3
  * object samples per rollout 1
* [x] Log geometry query count.
* [x] Log solve time.
* [ ] Confirm target control frequency or document reduced config.

---

## 14. Implementation Phases

### Phase A — Jenga geometric contact evaluator

* [x] Add Jenga box primitive config.
* [x] Add point-to-box signed distance query.
* [x] Compute hemisphere world positions from Pinocchio.
* [x] Evaluate predicted contact mask for sampled Jenga poses.
* [ ] Visualize predicted 3x4 contact grid.
* [ ] No MPPI yet.

---

### Phase B — Candidate action evaluation baseline

* [x] Add candidate actions:

  * hold
  * squeeze
  * release
  * thumb squeeze
  * index squeeze
  * align positive
  * align negative
* [x] For each action, compute next hand state.
* [x] Evaluate predicted support against sampled Jenga poses.
* [x] Select lowest-cost action.
* [x] Keep as `control_mode: discrete_action_selector` baseline only.

---

### Phase C — Robust perturbation scoring

* [x] Sample multiple Jenga pose perturbations.
* [x] Use MPPI soft weighting for continuous qddot samples.
* [ ] Add optional CVaR/worst-tail diagnostic for continuous MPPI.
* [ ] Check that weighted command is robust to pose uncertainty.
* [ ] Tune object pose perturbation size.

---

### Phase D — Tactile integration

* [x] Use measured tactile force for preload cost.
* [x] Use measured active contact mask for current support.
* [x] Use measured centroid for edge state.
* [ ] Compare predicted contact mask with next tactile frame.
* [ ] Log one-step prediction/evaluation error.

---
* [x] RViz visualization for Jenga prior, measured tactile contacts/contact-force arrows, and selected one-step rollout motion.
* [ ] RViz prediction contact patch for the 3x4 tactile grid and full horizon rollout trace.
---
## 15. Final Acceptance Criteria

The MVP is successful if:

* [x] Continuous qddot sequence sampling is the main controller path.
* [x] Output qddot is a soft weighted average, not the best sample.
* [x] Only the first weighted qddot is applied each tick.
* [x] Stable contact + no perturbation + zero noise returns near-zero qddot.
* [ ] Stable contact + nonzero noise converges to near-zero qddot without twitching.
* [ ] Low preload produces closing qddot.
* [ ] One weak side biases qddot toward weak-side squeeze.
* [ ] Edge contact biases qddot toward align/centering action.
* [ ] Controller does not endlessly squeeze.
* [ ] Predicted contact mask changes consistently with geometric Jenga perturbations.
* [ ] Runtime fits the target loop for MVP settings.
* [x] MPPI is framed as robust scenario evaluation, not exact contact prediction.

---

## 16. Top 5 Critical Checks

If time is limited, check only these:

* [x] Pinocchio computes hemisphere world positions for candidate actions.
* [x] Jenga box signed-distance query works.
* [x] Hemisphere-object gap uses hemisphere radius and penetration cost.
* [ ] Sampled Jenga pose perturbations generate different predicted contact masks.
* [x] Stable no-disturbance zero-noise contact returns near-zero qddot.
* [x] Continuous MPPI uses soft weighted qddot averaging, not hard-coded if-else.
