
## Launch PLATO

The SocketCAN shortcut selects `socketcan` and `can0`:

```bash
ros2 launch plato_bringup plato_socketcan.launch.py
```

It starts the PLATO hardware interface, robot state publisher, joint state
broadcaster, joint impedance controller, and RViz.

```bash
ros2 launch plato_bringup plato_socketcan.launch.py zeroing:=true
```

Only use `zeroing:=true` when the hand is physically in its defined zero pose.
The regular launch file can also select SocketCAN explicitly and lets you
choose another interface name:

```bash
ros2 launch plato_bringup plato_hardware.launch.py \
  can_backend:=socketcan socketcan_interface:=can0
```

## Send A Joint Impedance Command

The controller subscribes to:

```text
/plato2/joint_impedance_controller/commands
plato_interfaces/msg/ImpedanceCommands
```

Each array must contain eight values, ordered `joint1` through `joint8`, as in
`bringup/plato_bringup/config/ros2_controllers.yaml`:

| Field | Meaning |
| --- | --- |
| `position` | Desired joint positions in radians |
| `velocity` | Desired joint velocities in radians per second |
| `stiffness` | Joint-space stiffness in Nm/rad |
| `damping` | Joint-space damping in Nm/(rad/s) |
| `effort_ff` | Feedforward joint effort in Nm |

Read the joint names and current positions first:

```bash
ros2 topic echo --once /plato2/joint_states
```

Publish once with all five arrays. Replace `P1`–`P8`, `K1`–`K8`, and `D1`–`D8`
with numeric values chosen for the current pose and controller tuning before
running the command:

```bash
ros2 topic pub --once \
  /plato2/joint_impedance_controller/commands \
  plato_interfaces/msg/ImpedanceCommands \
  '{
    position: [P1, P2, P3, P4, P5, P6, P7, P8],
    velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    stiffness: [K1, K2, K3, K4, K5, K6, K7, K8],
    damping: [D1, D2, D3, D4, D5, D6, D7, D8],
    effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }'
```

The all-zero message below is only a CLI syntax example: it targets zero
positions with zero stiffness, damping, and feedforward effort, so it does not
provide an impedance hold. Use current positions and known tuned gains for a
real command.

For example
```
ros2 topic pub --once \
  /plato2/joint_impedance_controller/commands \
  plato_interfaces/msg/ImpedanceCommands \
  '{
    position: [0, 0, 0, 0, 0, 0, 0, 0],
    velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    stiffness: [0.5, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    damping: [0, 0, 0.1, 0.1, 0.1, 0.1, 0.15, 0.15],
    effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }'
```

The PLATO config enables controller-side impedance torque computation:
`effort = effort_ff + stiffness * (position - measured_position) + damping *
(velocity - measured_velocity)`. A one-shot publish leaves the controller
using the last accepted command until another command arrives; it is not a
short-duration pulse.

To confirm the command topic and message type:

```bash
ros2 topic info /plato2/joint_impedance_controller/commands
```
