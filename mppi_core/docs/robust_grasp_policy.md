# Robust Grasp Policy

`RobustGraspPolicy` is a disturbance-aware blind grasp-state predictor. It does
not estimate object pose and does not simulate a rigid object body.

The prediction state remains:

```text
GraspState = RobotState + vector<TactileState>
```

The policy differs from `MPPIOptimizer` in two important ways:

1. Robot action sampling happens in a low-dimensional grasp corrective action
   space, then the current robot+tactile state maps that action through a
   basis into `qddot`.
2. Randomness is applied to tactile/contact disturbances:
   tangent drift, rotational shear, normal-force rate, friction scale, CoP
   drift, and dropout.

The corrective action coefficients are:

```text
[squeeze, release, align_lateral, force_balance, thumb_bias, index_bias]
```

`GraspActionLibrary` computes the state-dependent basis directions from contact
normal Jacobians and active contact centroids. Debug probe candidates may be
enabled for sign checks, but the primary search is sampled coefficient actions,
not hard-coded joint-space presets.

Disturbance samples include one common hidden grasp disturbance plus smaller
sensor-local noise. This keeps the two fingertip contacts tied to one hidden
object/contact disturbance without requiring object-pose estimation.

Each candidate action sequence is rolled out against many pre-sampled
`GraspDisturbanceSequence`s. The selected action minimizes:

```text
mean disturbance cost + risk_weight * worst-tail/CVaR cost
```

The disturbed transition updates:

- tactile shear and shear velocity
- rotational shear
- hemisphere normal force
- contact loss from unloading, dropout, shear/rotation limits, or low force
- neighbor-based contact birth

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
