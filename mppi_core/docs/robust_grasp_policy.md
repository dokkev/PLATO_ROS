# Robust Grasp Policy

`RobustGraspPolicy` is a disturbance-aware grasp-state scenario evaluator. It
does not estimate the true object pose and does not simulate a rigid object
body.

The prediction state remains:

```text
GraspState = RobotState + vector<TactileState> + optional VirtualObjectBelief
```

The policy differs from `MPPIOptimizer` in two important ways:

1. Robot action sampling happens in a low-dimensional grasp corrective action
   space, then the current robot+tactile state maps that action through a
   basis into `qddot`.
2. Randomness is applied to tactile/contact disturbances:
   tangent drift, rotational shear, normal-force rate, friction scale, CoP
   drift, and dropout.
3. If a valid Jenga/object belief is available, object particles are treated as
   sampled pose scenarios. The cost uses primitive signed-distance queries from
   rollout hemisphere centers to the sampled object poses.

The corrective action coefficients are:

```text
[squeeze, release, align_lateral, force_balance, thumb_bias, index_bias]
```

`GraspActionLibrary` computes the state-dependent basis directions from contact
normal Jacobians and active contact centroids. Debug probe candidates may be
enabled for sign checks, but the primary search is sampled coefficient actions,
not hard-coded joint-space presets.

Disturbance samples include one common hidden grasp disturbance plus smaller
sensor-local noise. Object belief particles provide a separate scenario set for
geometric contact support evaluation without claiming true object-pose
prediction.

Each candidate action sequence is rolled out against many pre-sampled
`GraspDisturbanceSequence`s. The selected action minimizes:

```text
mean disturbance cost + risk_weight * worst-tail/CVaR cost
```

Hold/zero corrective action is candidate 0 and is treated as the default
optimal behavior. Nonzero actions must beat hold by
`min_required_score_improvement`, unless hold is already above the configured
risk margin. This makes the policy least-invasive: a secure grasp should hold
still, while fragile grasps can still squeeze, align, balance, or release when
disturbance rollouts predict meaningful risk.

Most grasp-quality terms use hinge/deadband costs rather than exact tracking
costs. Inside the safe force interval, force-balance deadband, alignment
deadband, and low-shear region, the best action should be zero because action
effort and action-rate costs make unnecessary motion worse.

Object support terms are also hinge costs. They penalize predicted loss of
measured contacts, insufficient predicted support count, small predicted edge
margin on the tactile grid, and excessive signed-distance force proxy. The
proxy force is only a scenario score derived from distance to the primitive
object surface.

The disturbed transition updates:

- tactile shear and shear velocity
- rotational shear
- hemisphere normal force
- contact loss from unloading, dropout, shear/rotation limits, or low force
- neighbor-based contact birth

The object support evaluator separately allows geometry-based contact birth for
inactive hemispheres when their signed distance to a sampled Jenga pose falls
inside the contact margin. This does not mutate tactile state; it contributes
to rollout cost and diagnostics.

The policy should only run after measured tactile contact exists on both
fingers. It is a grasp stabilizer, not a first-contact finder.

Main entry points:

- `mppi_core/policy/robust_grasp_policy.hpp`
- `mppi_core/policy/grasp_action_library.hpp`
- `mppi_core/disturbance/grasp_disturbance_sampler.hpp`
- `mppi_core/rollout/disturbed_grasp_rollout.hpp`
- `mppi_core/tactile/disturbed_tactile_transition.hpp`
- `mppi_core/costs/robust_grasp_state_cost.hpp`

Example config:

- `mppi_core/task/robust_grasp_hold.yaml`
