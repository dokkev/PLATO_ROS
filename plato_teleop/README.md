# PLATO Teleop Package

This package provides teleoperation interfaces for the PLATO robot system, supporting both C++ and Python ROS2 nodes.

## Overview

The `plato_teleop` package includes SpaceMouse-based control nodes that were refactored from the deprecated `spaceMouse_navigator` package with improved naming conventions and modular architecture.

## Package Structure

```
plato_teleop/
├── plato_teleop/              # Python package
│   ├── __init__.py
│   ├── spacemouse_hardware.py      # Hardware abstraction module
│   ├── spacemouse_twist.py         # Twist command publisher
│   └── spacemouse_state_machine.py # Gripper state machine
├── launch/                    # Launch files
│   ├── spacemouse_twist.launch.py
│   └── spacemouse_state_machine.launch.py
├── CMakeLists.txt
├── package.xml
├── setup.py
└── setup.cfg
```

## Nodes

### 1. spacemouse_twist

**Executable:** `spacemouse_twist`
**Description:** Reads SpaceMouse hardware directly and publishes TwistStamped commands for robot teleoperation.

**Published Topics:**
- `/optimo/servo/twist_cmd` (geometry_msgs/TwistStamped) - Twist commands for robot control

**Parameters:**
- `translation_scale` (double, default: 0.3) - Scaling factor for translation
- `rotation_scale` (double, default: 0.3) - Scaling factor for rotation
- `translation_threshold` (double, default: 0.3) - Threshold for translation magnitude
- `rotation_threshold` (double, default: 0.3) - Threshold for rotation magnitude
- `publish_rate` (double, default: 900.0) - Publishing rate in Hz
- `twist_topic` (string, default: "/optimo/servo/twist_cmd") - Twist command topic
- `frame_id` (string, default: "world") - Frame ID for twist messages

**Features:**
- Direct SpaceMouse hardware access via `spacemouse_hardware` module
- Intelligent motion filtering (translation OR rotation, not both)
- Configurable scaling and thresholds
- High-frequency publishing for responsive control

**Launch:**
```bash
ros2 launch plato_teleop spacemouse_twist.launch.py
```

### 2. spacemouse_state_machine

**Executable:** `spacemouse_state_machine`
**Description:** Controls gripper states based on SpaceMouse button presses. Reads SpaceMouse hardware directly.

**Subscribed Topics:**
- `/plato2/joint_states` (sensor_msgs/JointState) - Current joint positions

**Published Topics:**
- `/plato2/joint_impedance_trajectory_controller/commands` (std_msgs/Float64MultiArray) - Joint position commands

**Features:**
- Direct SpaceMouse hardware access
- Predefined gripper states (open, close, poke, zero)
- Smooth interpolation between states
- Button press detection

**Launch:**
```bash
ros2 launch plato_teleop spacemouse_state_machine.launch.py
```

## Hardware Module

### spacemouse_hardware.py

This module provides a clean hardware abstraction layer for the SpaceMouse device using `pyspacemouse`.

**Features:**
- Clean OOP interface
- Button transition detection (press/release)
- Error handling and device management
- Testable and mockable design

**Usage:**
```python
from plato_teleop.spacemouse_hardware import SpaceMouseHardware

spacemouse = SpaceMouseHardware()
if spacemouse.open():
    state = spacemouse.read()
    # Use state.x, state.y, state.z, etc.
    spacemouse.close()
```

## Migration from spaceMouse_navigator

This package replaces the deprecated `spaceMouse_navigator` package with improved naming conventions:

**Old Package** → **New Package**
- `spaceMouse_navigator` → `plato_teleop`
- `spaceMouse_publisher` → ❌ Removed (functionality integrated into nodes)
- `controller_spaceMouse.cpp` → `spacemouse_twist.py`
- `spaceMouse_state_machine` → `spacemouse_state_machine` (updated)

**Key Improvements:**
1. **Conventional naming:** All snake_case instead of camelCase
2. **Modular design:** Hardware abstraction separated from ROS nodes
3. **Direct hardware access:** Each node reads SpaceMouse independently
4. **Python implementation:** Easier to maintain than C++
5. **Unified package:** Both C++ and Python nodes in one package
6. **Better launch files:** Parameterized launch files for each node
7. **Eliminated middleware:** No Joy message layer - direct hardware access

## Dependencies

- ROS2 (Humble or later)
- Python 3
- pyspacemouse
- rclpy
- geometry_msgs
- sensor_msgs
- std_msgs

## Building

```bash
cd ~/workspaces/plato_ws
colcon build --packages-select plato_teleop
source install/setup.bash
```

## Usage Examples

### Basic SpaceMouse Control
```bash
# Launch twist publisher for robot control
ros2 launch plato_teleop spacemouse_twist.launch.py
```

### State Machine Control
```bash
# Launch state machine for gripper control
ros2 launch plato_teleop spacemouse_state_machine.launch.py
```

### Running Both Simultaneously

**Note:** Both `spacemouse_twist` and `spacemouse_state_machine` access the SpaceMouse hardware directly. You can run them simultaneously - each node will independently read the device:

```bash
# Terminal 1: Robot teleoperation
ros2 launch plato_teleop spacemouse_twist.launch.py

# Terminal 2: Gripper control
ros2 launch plato_teleop spacemouse_state_machine.launch.py
```

The `spacemouse_twist` node handles motion (translation/rotation), while `spacemouse_state_machine` handles button presses for gripper control.

## Architecture

```
SpaceMouse Device
      ↓
spacemouse_hardware.py (module)
      ↓
      ├─→ spacemouse_twist.py → /optimo/servo/twist_cmd → Robot Control
      │
      └─→ spacemouse_state_machine.py → joint commands → Gripper Control
```

Each node directly accesses the SpaceMouse hardware through the shared `spacemouse_hardware` module, eliminating the need for intermediate Joy message publishers.

## License

Apache-2.0
