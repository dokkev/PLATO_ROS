# PLATO2 State Estimator

State estimation package for the PLATO2 robot system, including object state estimation and slip detection.

## Nodes

### object_state_estimator

Estimates the slip state of a grasped object using dual tactile sensors and calculates minimal grasping force to prevent slip.

**Theory:**
Based on "Theoretical Derivation and Realization of Adaptive Grasping Based on Rotational Incipient Slip Detection" by T. Narita et al., ICRA 2020.

#### Subscribed Topics

- `/tactile_0/tactile_states` ([sdr_grasp_msgs/Tactile](../../nari_touch/sdr_grasp_msgs/msg/Tactile.msg))
  Tactile sensor data from first gripper finger

- `/tactile_1/tactile_states` ([sdr_grasp_msgs/Tactile](../../nari_touch/sdr_grasp_msgs/msg/Tactile.msg))
  Tactile sensor data from second gripper finger

#### Published Topics

- `/object_state/minimal_force` (std_msgs/Float32)
  Minimal normal force [N] required to prevent object slip

- `/object_state/estimated_wrench` (geometry_msgs/Wrench)
  Estimated wrench on the object including normal and tangential forces

#### Parameters

##### Material Properties
- `E_star` (double, default: 1.0e6 Pa)
  Effective Young's modulus

- `G_star` (double, default: 0.4e6 Pa)
  Transverse elastic modulus

- `C_n` (double, default: 1.0)
  Contact surface constant

- `lambda_n` (double, default: 1.0)
  Scaling factor for 3D contact

- `n` (int, default: 2)
  Order of contact surface (2 for quadric/hemisphere)

##### Slip Detection
- `translational_slip_threshold` (double, default: 0.5 mm)
  Threshold for detecting translational slip

- `rotational_slip_threshold` (double, default: 0.05 rad)
  Threshold for detecting rotational slip

##### Control
- `update_rate` (double, default: 100.0 Hz)
  Update frequency

- `min_contact_force` (double, default: 0.5 N)
  Minimum force to maintain contact

- `max_force_limit` (double, default: 30.0 N)
  **SAFETY LIMIT** - Maximum allowed force

##### PID Gains
- `pid_tx_p`, `pid_tx_i`, `pid_tx_d` (double, default: [1.8, 0.0, 4.5])
  PID gains for translational control

- `pid_theta_p`, `pid_theta_i`, `pid_theta_d` (double, default: [30.0, 0.0, 90.0])
  PID gains for rotational control

## Implementation Details

### State Machine

The estimator implements a state machine to track object slip:

1. **NO_CONTACT**: No sensors detect contact
2. **PARTIAL_CONTACT**: Only one sensor detects contact (object not fully grasped)
3. **STABLE_GRASP**: Both sensors in contact, no slip detected
4. **TRANSLATIONAL_SLIP**: Translational slip detected
5. **ROTATIONAL_SLIP**: Rotational slip detected
6. **COMBINED_SLIP**: Both translational and rotational slip

### Force Calculation

The minimal force is calculated based on equations (12) and (14) from the Narita paper:

**Translational force (Eq. 12):**
```
u_x = F_x / (G* * ((n+1)/(2n) * F_N/(E*C_n*λ_n))^(1/(n+1)))
```

**Rotational force (Eq. 14):**
```
u_θ = 3T_θ / (2G* * ((n+1)/(2n) * F_N/(E*C_n*λ_n))^(2/(n+1)))
```

Where:
- `u_x, u_y`: Translational shear displacements
- `u_θ`: Rotational shear displacement
- `F_N`: Normal (grasp) force
- `T_θ`: Applied moment

The controller uses PID control to drive shear displacements to zero, which corresponds to preventing slip.

## Usage

### Launch

```bash
ros2 launch plato2_state_estimator object_state_estimator.launch.py
```

### With custom configuration

```bash
ros2 launch plato2_state_estimator object_state_estimator.launch.py \
    config_file:=/path/to/custom_config.yaml
```

### Monitor minimal force

```bash
ros2 topic echo /object_state/minimal_force
```

## Dependencies

- `rclcpp`
- `geometry_msgs`
- `std_msgs`
- `plato_utils` (PID controller)
- `sdr_grasp_msgs` (from nari_touch package)

## Integration with Grasp Controller

This node publishes the minimal force needed to maintain a stable grasp. The parallel grasp controller should subscribe to `/object_state/minimal_force` and use it as the target force during dynamic manipulation.

## Safety Considerations

- **Always set `max_force_limit` appropriately for your gripper and objects**
- Test with non-fragile objects first
- Monitor the `/object_state/estimated_wrench` topic for unexpected forces
- Tune PID gains carefully to avoid oscillations

## Tuning Guide

1. **Start with default PID gains** from the paper
2. **Adjust slip thresholds** based on your sensor sensitivity:
   - If false positives (detecting slip when stable): increase thresholds
   - If missing slips: decrease thresholds
3. **Tune PID gains** if force oscillates or responds slowly:
   - Increase P for faster response
   - Increase D to reduce oscillations
   - Add small I term only if steady-state error exists
4. **Validate material properties** (E*, G*) if calculated forces seem incorrect

## References

[1] T. Narita, S. Nagakari, W. Conus, T. Tsuboi and K. Nagasaka, "Theoretical Derivation and Realization of Adaptive Grasping Based on Rotational Incipient Slip Detection," 2020 IEEE International Conference on Robotics and Automation (ICRA), 2020, pp. 531-537.
