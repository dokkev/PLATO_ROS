# Parallel Grasp Controller - Command Examples

## Command Topic
**Topic**: `/plato2/parallel_grasp_controller/commands`
**Message Type**: `std_msgs/msg/Float64MultiArray`
**Format**: `[u, phi, f]`

### Parameters:
- **u** (required): Grasp distance [0.0, 1.0]
  - `0.0` = Fully closed (fingertips meet)
  - `0.5` = Neutral/mid-position
  - `1.0` = Fully open

- **phi** (optional, default=0.0): Contact angle [0.0, 1.0]
  - `0.0` = Parallel grasp (fingers mirrored)
  - `1.0` = Maximum flexion (~45° inward bend)

- **f** (optional, default=0.0): Force (reserved for future use)

---

## Command Examples

### 1. Parallel Grasp (Default Behavior)
Closes the gripper with fingers parallel to each other:

```bash
ros2 topic pub --once /plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0]"
```

### 2. Open Gripper (Parallel)
Opens the gripper to maximum with parallel fingers:

```bash
ros2 topic pub --once /plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 0.0, 0.0]"
```

### 3. Half-Open (Parallel)
Neutral mid-position with parallel fingers:

```bash
ros2 topic pub --once /plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.0, 0.0]"
```

### 4. Closed with Maximum Flexion
Closes gripper with fingers flexed inward for wrapping grasp:

```bash
ros2 topic pub --once /plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 1.0, 0.0]"
```

### 5. Half-Closed with Medium Flexion
Partially closed with moderate finger flexion:

```bash
ros2 topic pub --once /plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray "data: [0.3, 0.5, 0.0]"
```

### 6. Open with Slight Flexion
Open gripper with slightly flexed fingers:

```bash
ros2 topic pub --once /plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray "data: [0.8, 0.2, 0.0]"
```

---

## Shortened Format (Defaults Apply)

You can send fewer than 3 values - missing values default to safe settings:

### Just grasp distance (phi=0.0, f=0.0):
```bash
ros2 topic pub --once /plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"
```

### Grasp distance + contact angle (f=0.0):
```bash
ros2 topic pub --once /plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray "data: [0.3, 0.7]"
```

---

## Python Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class GripperCommander(Node):
    def __init__(self):
        super().__init__('gripper_commander')
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/plato2/parallel_grasp_controller/commands',
            10
        )

    def send_command(self, u, phi=0.0, f=0.0):
        msg = Float64MultiArray()
        msg.data = [u, phi, f]
        self.pub.publish(msg)
        self.get_logger().info(f'Sent: u={u}, phi={phi}, f={f}')

def main():
    rclpy.init()
    commander = GripperCommander()

    # Example: Close with parallel grasp
    commander.send_command(u=0.0, phi=0.0)

    rclpy.spin_once(commander, timeout_sec=0.1)

    # Example: Half-closed with flexion
    commander.send_command(u=0.3, phi=0.5)

    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Common Grasp Strategies

### Precision Grasp (Parallel Fingers)
For small, precise objects:
```bash
data: [0.2, 0.0, 0.0]
```

### Power Grasp (Flexed Fingers)
For large objects requiring wrapping:
```bash
data: [0.1, 0.8, 0.0]
```

### Open for Approach
Before grasping:
```bash
data: [0.9, 0.0, 0.0]
```

---

## Configuration Parameters

Maximum flexion angle can be adjusted in the header file:
```cpp
// parallel_grasp_controller.hpp
static constexpr double max_flexion_angle = 0.785;  // π/4 rad (~45°)
```

---

## Save Current Joint Position To YAML

`parallel_grasp_node` exposes a `save_joint_position` service using
`plato_interfaces/srv/SaveJointPosition`.

Save with an explicit name:
```bash
ros2 service call /parallel_grasp_node/save_joint_position plato_interfaces/srv/SaveJointPosition "{name: pinch_ready}"
```

If `name` is empty, a timestamp-based name is generated automatically.
