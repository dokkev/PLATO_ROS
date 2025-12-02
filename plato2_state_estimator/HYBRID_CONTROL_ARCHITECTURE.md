# Hybrid Feedforward-Feedback Control Architecture

## Overview

The Object State Estimator uses a **hybrid control strategy** combining:
1. **Feedforward Control**: Physics-based force calculation from contact mechanics equations
2. **Feedback Control**: PID correction to compensate for modeling errors

This approach provides both theoretical accuracy and practical robustness.

---

## Control Architecture

```
                    ┌─────────────────────────────────────┐
                    │   Tactile Sensor Measurements       │
                    │   - Shear: ux, uy, u_theta          │
                    │   - Forces: Fx, Fy, Fz              │
                    └──────────────┬──────────────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │                             │
         ┌──────────▼──────────┐      ┌──────────▼──────────┐
         │  FEEDFORWARD PATH   │      │   FEEDBACK PATH     │
         │  (Physics Model)    │      │   (PID Control)     │
         └──────────┬──────────┘      └──────────┬──────────┘
                    │                             │
         ┌──────────▼──────────┐      ┌──────────▼──────────┐
         │ Equation (12) & (14)│      │ PID(ux, uy, u_θ)   │
         │ F_N = f(F_t, u, ...)│      │ Goal: u → 0         │
         └──────────┬──────────┘      └──────────┬──────────┘
                    │                             │
                    │   F_feedforward             │   F_feedback
                    └──────────────┬──────────────┘
                                   │
                         ┌─────────▼─────────┐
                         │  F_total = FF + FB │
                         │  Apply Limits      │
                         └─────────┬──────────┘
                                   │
                         ┌─────────▼──────────┐
                         │  Minimal Grasp     │
                         │  Force Output      │
                         └────────────────────┘
```

---

## 1. Feedforward Control (Physics-Based)

### Purpose
Provides theoretical estimate of required force based on contact mechanics.

### Implementation

#### Translational Component
Based on **Equation (12)** from Narita et al., ICRA 2020:

```
u_x = F_x / (G* · K_t · F_N^{1/(n+1)})
```

Rearranged to solve for F_N:
```
F_N_trans = (F_tangential / (G* · u_magnitude))^{n+1} / A
```

Where:
- `A = (n+1)/(2n) · 1/(E* · C_n · λ_n^n)` - Contact model constant
- `F_tangential = √(Fx² + Fy²)` - Tangential force magnitude
- `u_magnitude = √(ux² + uy²)` - Translational shear displacement
- `n = 2` - Contact surface order (quadric)

#### Rotational Component
Based on **Equation (14)**:

```
u_theta = 3·T_theta / (2·G* · K_r · F_N^{2/(n+1)})
```

Rearranged:
```
F_N_rot = (3·T_theta / (2·G* · u_theta))^{(n+1)/2} / B
```

Where `B = A` (same contact constant).

#### Combined Feedforward
```cpp
F_feedforward = F_N_trans + F_N_rot
```

### Advantages
- ✅ Based on first-principles physics
- ✅ Model-based prediction
- ✅ Accounts for material properties (E*, G*)
- ✅ No tuning needed for ideal conditions

### Limitations
- ⚠️ Assumes perfect contact model
- ⚠️ Sensitive to parameter accuracy
- ⚠️ Doesn't adapt to disturbances
- ⚠️ Open-loop (no error correction)

---

## 2. Feedback Control (PID)

### Purpose
Corrects for modeling errors, uncertainties, and disturbances.

### Implementation

The PID acts on **shear displacement error**:
- **Setpoint**: Zero displacement (stable grasp)
- **Error**: Measured displacement (ux, uy, u_theta)
- **Goal**: Drive displacement to zero

```cpp
// Translational PID (x and y independently)
feedback_x = PID_x(-ux, dt)
feedback_y = PID_y(-uy, dt)
feedback_trans = √(feedback_x² + feedback_y²)

// Rotational PID
feedback_rot = |PID_theta(-u_theta, dt)|

// Combined feedback (use max to avoid over-correction)
F_feedback = max(feedback_trans, feedback_rot)
```

### PID Gains (Default)

| Component | P | I | D | Purpose |
|-----------|---|---|---|---------|
| **Translational** | 1.8 | 0.0 | 4.5 | Corrects linear slip |
| **Rotational** | 30.0 | 0.0 | 90.0 | Corrects angular slip |

**Tuning Guidelines:**
- **P (Proportional)**: Response to current error
  - Higher P → Faster response, may cause oscillation
  - Lower P → Slower, smoother response
- **I (Integral)**: Eliminates steady-state error
  - Use sparingly (default 0) to avoid windup
  - Enable only if persistent offset observed
- **D (Derivative)**: Dampens oscillations
  - Higher D → More damping, smoother
  - Lower D → Less damping, faster

### Advantages
- ✅ Corrects modeling errors
- ✅ Adapts to disturbances
- ✅ Closed-loop stability
- ✅ Rejects steady-state errors (with I term)

### Limitations
- ⚠️ Requires tuning
- ⚠️ May introduce lag
- ⚠️ Can cause overshoot if gains too high

---

## 3. Combined Control

### Total Force Calculation

```cpp
F_total = F_feedforward + F_feedback

// Apply safety limits
F_total = clamp(F_total, min_contact_force, max_force_limit)
```

### Why This Works

| Scenario | Feedforward | Feedback | Result |
|----------|-------------|----------|--------|
| **Ideal conditions** | Accurate | ~0 | Physics model dominates |
| **Model mismatch** | Under/overestimate | Compensates | PID corrects error |
| **Disturbances** | Unchanged | Reacts | PID rejects disturbance |
| **Unknown dynamics** | Approximation | Learns | Combined adaptation |

### Key Benefits

1. **Fast Response**: Feedforward provides immediate action
2. **Accuracy**: Physics model guides nominal behavior
3. **Robustness**: Feedback handles uncertainties
4. **Stability**: PID ensures convergence to zero slip

---

## 4. Implementation Details

### Code Flow

**File**: `object_state_estimator.cpp`

```cpp
double calculateMinimalForce(tactile0, tactile1, dt) {
    // 1. Average sensor measurements
    ux_avg, uy_avg, utheta_avg = average(tactile0, tactile1)
    F_tangential = √(Fx² + Fy²)

    // 2. FEEDFORWARD: Physics-based calculation
    F_N_trans = calculateTranslationalForce(F_tangential, ux, uy)
    F_N_rot = calculateRotationalForce(T_theta, u_theta)
    F_feedforward = F_N_trans + F_N_rot

    // 3. FEEDBACK: PID correction
    F_feedback = calculatePIDFeedback(ux, uy, u_theta, dt)

    // 4. COMBINE
    F_total = F_feedforward + F_feedback

    // 5. SAFETY LIMITS
    return clamp(F_total, min_force, max_force)
}
```

### PID Reset Logic

```cpp
// Reset PIDs when contact is lost (prevents windup)
if (!hasBothContacts(tactile0, tactile1)) {
    pid_translational_x_->reset()
    pid_translational_y_->reset()
    pid_rotational_->reset()
    return min_contact_force
}
```

---

## 5. Tuning Guide

### Step 1: Verify Feedforward Parameters

Ensure physics parameters are accurate:
- `E_star`: Effective Young's modulus [Pa]
- `G_star`: Transverse elastic modulus [Pa]
- `C_n`, `lambda_n`: Contact surface constants
- `n`: Contact order (typically 2)

**Test**: Set all PID gains to 0, verify reasonable force estimates.

### Step 2: Tune Proportional Gains

Start with feedforward only, gradually increase P:
1. Set I=0, D=0
2. Increase P until oscillations appear
3. Reduce P by 50%

### Step 3: Add Derivative

If response is too oscillatory:
1. Set D = P × 2.5 (starting point)
2. Adjust to smooth response

### Step 4: Add Integral (Optional)

Only if steady-state error persists:
1. Set I = P / 10 (conservative)
2. Monitor for windup (saturation)

### Example Tuning Sequence

```yaml
# Start (feedforward only)
pid_tx_p: 0.0
pid_tx_d: 0.0

# Add proportional
pid_tx_p: 1.0   # Increase until oscillation
pid_tx_p: 1.8   # Reduce to 50% of oscillation point

# Add derivative (dampen)
pid_tx_d: 4.5   # D = P × 2.5

# Result: Fast, smooth response
```

---

## 6. Performance Characteristics

### Comparison: Feedforward vs Feedback vs Hybrid

| Metric | FF Only | FB Only | Hybrid |
|--------|---------|---------|--------|
| **Response Time** | Fast | Slow | Fast |
| **Steady-State Error** | High | Low | Low |
| **Disturbance Rejection** | Poor | Good | Excellent |
| **Model Sensitivity** | High | Low | Low |
| **Tuning Effort** | None | High | Medium |
| **Stability** | Conditional | Good | Excellent |

### Expected Behavior

| Slip State | Force Output | Control Action |
|------------|--------------|----------------|
| **No Contact** | `min_contact_force` | PIDs reset |
| **Stable Grasp** | `F_ff + F_fb ≈ min` | Minimal correction |
| **Incipient Slip** | `F_ff + F_fb` increases | PID drives u → 0 |
| **Active Slip** | High force | FF + FB both active |
| **Recovery** | Decreases as u → 0 | PID settles |

---

## 7. Safety Features

### Force Limiting
```cpp
F_minimal = min(F_total, max_force_limit)  // Upper bound (30 N default)
F_minimal = max(F_total, min_contact_force)  // Lower bound (0.5 N)
```

### PID Anti-Windup
Built into `PIDController` class:
- Output clamping
- Integral reset on saturation
- Rate limiting

### Noise Filtering
```cpp
constexpr double SLIP_NOISE_THRESHOLD = 0.25e-3  // 0.25 mm
if (u_magnitude <= SLIP_NOISE_THRESHOLD) {
    return 0.0  // Ignore noise-level displacements
}
```

---

## 8. Configuration Parameters

**File**: `config/object_state_estimator.yaml`

```yaml
object_state_estimator:
  ros__parameters:
    # Material properties (feedforward)
    E_star: 1.0e6        # Pa
    G_star: 0.4e6        # Pa
    C_n: 1.0
    lambda_n: 1.0
    n: 2

    # Slip thresholds
    translational_slip_threshold: 0.5  # mm
    rotational_slip_threshold: 0.05    # rad

    # Safety limits
    min_contact_force: 0.5    # N
    max_force_limit: 30.0     # N (CRITICAL: hardware safety)

    # PID gains (feedback)
    pid_tx_p: 1.8
    pid_tx_i: 0.0
    pid_tx_d: 4.5
    pid_theta_p: 30.0
    pid_theta_i: 0.0
    pid_theta_d: 90.0
```

---

## 9. Debugging Tips

### Check Feedforward Output
```bash
# Disable feedback temporarily
ros2 param set /object_state_estimator pid_tx_p 0.0
ros2 param set /object_state_estimator pid_theta_p 0.0

# Verify physics calculation makes sense
ros2 topic echo /object_state/minimal_force
```

### Check Feedback Output
Set very high PID gains to see feedback effect:
```bash
ros2 param set /object_state_estimator pid_tx_p 10.0
```

### Monitor Slip State
```bash
# Watch for slip detection
ros2 topic echo /object_state/estimated_wrench
```

Look for state transitions in logs:
```
[INFO] Slip state changed to: TRANSLATIONAL_SLIP
[INFO] Slip state changed to: STABLE_GRASP
```

---

## Summary

The hybrid control architecture combines:
- **Feedforward**: Model-based prediction (fast, accurate when model is good)
- **Feedback**: Error-based correction (robust, adapts to uncertainties)

This provides optimal performance: fast response from feedforward, robustness from feedback.

**Key Equation**:
```
F_minimal = F_feedforward(physics equations) + F_feedback(PID) + limits
```

The system is production-ready with sensible defaults, but can be tuned for specific grippers, objects, and tasks.
