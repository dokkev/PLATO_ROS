# PLATO State Estimator

This package contains small sensing helpers for PLATO/Aristo experiments.

## Reference Grasp Force Generator

The executable `object_state_estimator_node` currently runs a
`ReferenceGraspForceGeneratorNode`. The executable name is kept for temporary
launch compatibility, but the node is not a full object-state estimator.

It uses two tactile sensors to publish:

- a scalar target normal grasp force,
- whether that force reference is valid,
- measured normal-force diagnostics,
- shear displacement diagnostics,
- a slip diagnostic state.

Task-level decisions such as closing, releasing, mode transitions, fallback
motion, MPPI, or whole-body control belong in the controller/state-machine
layer. This node only reports a safe reference and diagnostics.

### Subscribed Topics

- `/tactile_0/tactile_states` (`sdr_grasp_msgs/msg/Tactile`)
- `/tactile_1/tactile_states` (`sdr_grasp_msgs/msg/Tactile`)

### Published Topics

- `/grasp_force_reference/target_normal_force_n` (`std_msgs/msg/Float64`)
- `/grasp_force_reference/reference_valid` (`std_msgs/msg/Bool`)
- `/grasp_force_reference/measured_normal_force_min_n` (`std_msgs/msg/Float64`)
- `/grasp_force_reference/measured_normal_force_avg_n` (`std_msgs/msg/Float64`)
- `/grasp_force_reference/shear_translation_mm` (`std_msgs/msg/Float64`)
- `/grasp_force_reference/shear_rotation_rad` (`std_msgs/msg/Float64`)
- `/grasp_force_reference/slip_state` (`std_msgs/msg/String`)

Deprecated compatibility topics:

- `/object_state/minimal_force` (`std_msgs/msg/Float32`)
  Publishes `target_normal_force_n` only when `reference_valid=true`.
- `/object_state/measured_force` (`std_msgs/msg/Float64`)
  Publishes the conservative measured normal-force minimum.

### Safety Semantics

`reference_valid` is the field a downstream force controller should trust.

- No contact: `reference_valid=false`, `slip_state=NO_CONTACT`, target force is
  `0.0` only as informational output.
- One-sided or weak contact: `reference_valid=false`,
  `slip_state=PARTIAL_CONTACT`. This must not be interpreted as a command to
  track zero force.
- Valid two-sided contact: target force is positive, clamped, and based on shear
  magnitude and shear growth rate.

Measured normal force is reported as:

- `measured_normal_force_min_n = min(f0z, f1z)` when both sensors have contact,
- `measured_normal_force_avg_n = 0.5 * (f0z + f1z)` when both sensors have contact,
- for one-sided contact, min is `0.0` and avg treats the missing side as zero.

### Force Reference Law

The generator sign-corrects raw tactile shear into a common reference frame,
averages both sensors, then computes:

```txt
u_trans_mm = sqrt(ux_avg^2 + uy_avg^2)
u_rot_rad  = abs(utheta_avg)

force_increment_n =
    k_trans_n_per_mm   * max(0, u_trans_mm - trans_deadband_mm)
  + d_trans_n_per_mm_s * positive_trans_rate_mm_s
  + k_rot_n_per_rad    * max(0, u_rot_rad - rot_deadband_rad)
  + d_rot_n_per_rad_s  * positive_rot_rate_rad_s

target_normal_force_n =
    clamp(base_force_n + force_increment_n,
          min_contact_force_n,
          max_force_limit_n)
```

This is shear-displacement feedback. It does not compute analytical Narita
minimal force because tangential force and moment inputs are not available here.

### Parameters

See `config/object_state_estimator.yaml`.

Important parameters:

- `base_force_n`
- `min_contact_force_n`
- `max_force_limit_n`
- `trans_deadband_mm`
- `rot_deadband_rad`
- `translational_slip_threshold_mm`
- `rotational_slip_threshold_rad`
- `k_trans_n_per_mm`
- `d_trans_n_per_mm_s`
- `k_rot_n_per_rad`
- `d_rot_n_per_rad_s`
- `tactile0_shear_x_sign`, `tactile0_shear_y_sign`, `tactile0_shear_theta_sign`
- `tactile1_shear_x_sign`, `tactile1_shear_y_sign`, `tactile1_shear_theta_sign`

### Launch

```bash
ros2 launch plato_state_estimator object_state_estimator.launch.py
```

With a custom config:

```bash
ros2 launch plato_state_estimator object_state_estimator.launch.py \
  config_file:=/path/to/object_state_estimator.yaml
```

### Validation

Build this package from the workspace root:

```bash
colcon build --packages-select plato_state_estimator
```

Run the core unit test:

```bash
./build/plato_state_estimator/reference_grasp_force_generator_test --gtest_color=no
```
