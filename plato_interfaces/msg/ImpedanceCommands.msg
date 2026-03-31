
# Stiffness
float64[] stiffness

# Damping
float64[] damping

# Array of desired positions for all joints
float64[] position

# Array of desired velocities for all joints
float64[] velocity

# Array of feedforward torques for all joints
float64[] effort_ff

# Optional per-joint servo current commands in milliamps.
# Leave empty to use hardware defaults.
float64[] servo_current_milliamps
