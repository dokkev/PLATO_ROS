# FT Sensor Collision Detection Test Node

## Overview
This ROS2 test node demonstrates collision detection using the FT (Force-Torque) sensor 2. It moves joint 5 at a configurable speed and automatically stops when the FT sensor detects a collision based on force and torque thresholds.

## Features
- **Automatic Collision Detection**: Monitors FT sensor 2 (index finger) for collision events
- **Configurable Movement**: Control joint 5 velocity and movement range via parameters
- **Interactive Control**: Keyboard commands for real-time control
- **Safety Limits**: Joint position limits and threshold-based collision detection
- **Real-time Feedback**: Status updates and collision alerts

## Prerequisites
- ROS2 workspace with PLATO2 packages built
- FT sensor 2 properly configured and broadcasting on `/plato2/ft_sensor_broadcaster_2/wrench`
- Impedance controller running and accepting commands on `/plato2/joint_impedance_controller/commands`

## Building
```bash
cd /home/optimo/CODE/plato_ws
colcon build --packages-select aristo_hardware_interface
source install/setup.bash
```

## Running the Node

### Method 1: Using Helper Script (Recommended)
```bash
./src/PLATO_ROS/hardware_interface/aristo_hardware_interface/script/run_ft_collision_test.sh
```

### Method 1b: Run full stack (bringup + monitor + collision)
```bash
ros2 launch aristo_bringup aristo_hardware.launch.py
ros2 run aristo_hardware_interface ft_sensor_monitor_node
ros2 run aristo_hardware_interface ft_collision_test_node --ros-args \
  -p joint5_velocity:=0.05 -p force_threshold:=0.6 -p torque_threshold:=0.02 -p auto_start:=false
```
This starts Aristo hardware bringup, a live FT monitor, and the collision test node.

With custom parameters:
```bash
ros2 run aristo_hardware_interface ft_collision_test_node --ros-args \
    -p joint5_velocity:=0.15 \
    -p force_threshold:=3.0 \
    -p torque_threshold:=0.15 \
    -p auto_start:=true
```

### Method 2: Direct Execution
```bash
ros2 run aristo_hardware_interface ft_collision_test_node
```

With parameters:
```bash
ros2 run aristo_hardware_interface ft_collision_test_node \
    --ros-args \
    -p joint5_velocity:=0.05 \
    -p force_threshold:=0.6 \
    -p torque_threshold:=0.02 \
    -p joint5_min:=-1.0 \
    -p joint5_max:=1.0 \
    -p control_rate:=100.0 \
    -p auto_start:=false
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joint5_velocity` | double | 0.05 | Joint 5 velocity in rad/s (positive or negative) |
| `force_threshold` | double | 0.6 | Force magnitude threshold for collision detection (N) |
| `torque_threshold` | double | 0.02 | Torque magnitude threshold for collision detection (Nm) |
| `joint5_min` | double | -1.0 | Minimum joint 5 position (rad) |
| `joint5_max` | double | 1.0 | Maximum joint 5 position (rad) |
| `control_rate` | double | 100.0 | Control loop update rate (Hz) |
| `auto_start` | bool | false | Automatically start movement on launch |

## Keyboard Commands

| Key | Action |
|-----|--------|
| `s` | Start/Resume movement |
| `p` | Pause movement |
| `r` | Reset collision flag |
| `+` | Increase velocity by 0.05 rad/s |
| `-` | Decrease velocity by 0.05 rad/s |
| `q` | Quit the node |

## Workflow Example

### 1. Start the Hardware and Controllers
```bash
# Terminal 1: Launch Aristo hardware
ros2 launch aristo_bringup aristo_hardware.launch.py

# Terminal 2: Set controller gains (if needed)
ros2 run joint_impedance_controller impedance_trajectory_controller_node
```

### 2. Run the Collision Test
```bash
# Terminal 3: Run the collision test node
ros2 run aristo_hardware_interface ft_collision_test_node
```

### 3. Operate
1. The node starts in paused mode (unless `auto_start:=true`)
2. Press `s` to start joint 5 movement
3. The joint will move at the configured velocity
4. When the FT sensor detects a collision (force or torque exceeds threshold), the movement stops automatically
5. Press `r` to reset the collision flag
6. Press `s` to resume movement
7. Use `+`/`-` to adjust velocity in real-time
8. Press `q` to quit

## How It Works

### Collision Detection Logic
The node subscribes to FT sensor 2 (`/plato2/ft_sensor_broadcaster_2/wrench`) and calculates:

```
force_magnitude = sqrt(fx² + fy² + fz²)
torque_magnitude = sqrt(tx² + ty² + tz²)
```

If either magnitude exceeds its threshold while moving:
- Movement stops immediately
- Collision flag is set
- Warning message is displayed with current readings and joint position

### Movement Control
The node publishes `ImpedanceCommands` messages to control joint 5:
- Position is updated at the configured `control_rate`
- Only joint 5 (index 4 in the array) is controlled
- Other joints remain at position 0.0 with moderate stiffness/damping
- Movement automatically reverses at joint limits

## Monitoring

### Check FT Sensor Data
```bash
ros2 topic echo /plato2/ft_sensor_broadcaster_2/wrench
```

### Check Commands Being Sent
```bash
ros2 topic echo /plato2/joint_impedance_controller/commands
```

### View Node Parameters
```bash
ros2 param list /ft_collision_test_node
ros2 param get /ft_collision_test_node force_threshold
```

## Troubleshooting

### Node doesn't receive FT sensor data
- Verify FT sensor broadcaster is running:
  ```bash
  ros2 node list | grep ft_sensor
  ```
- Check topic exists:
  ```bash
  ros2 topic list | grep wrench
  ```

### Joint doesn't move
- Verify impedance controller is loaded and running:
  ```bash
  ros2 control list_controllers
  ```
- Check if collision flag is set (press `r` to reset)

### Collision detected too easily
- Increase `force_threshold` and/or `torque_threshold`
- Check FT sensor calibration (tare)

### Collision not detected
- Decrease `force_threshold` and/or `torque_threshold`
- Verify FT sensor is publishing valid data
- Check the sensor orientation and expected force direction

## Safety Notes
⚠️ **Important Safety Considerations:**
- Always monitor the robot during testing
- Start with low velocities (e.g., 0.05 rad/s)
- Set conservative force/torque thresholds
- Have emergency stop ready
- Ensure workspace is clear of obstacles
- Test collision detection with soft objects first

## Advanced Usage

### Testing Different Thresholds
Create different configuration files for different test scenarios:
```yaml
# config/ft_collision_test_sensitive.yaml
ft_collision_test_node:
  ros__parameters:
    force_threshold: 1.0
    torque_threshold: 0.05
    joint5_velocity: 0.05
```

### Logging Collision Data
The node logs collision events with:
- Force magnitude at collision
- Torque magnitude at collision
- Joint 5 position at collision

Check logs:
```bash
ros2 run rqt_console rqt_console
```

## Future Enhancements
- [ ] Record collision data to file
- [ ] Visualize force/torque in real-time (rqt_plot)
- [ ] Multi-joint coordinated movement
- [ ] Adaptive threshold adjustment
- [ ] Contact force estimation

## Related Files
- **Source**: `src/tools/ft_collision_test_node.cpp`
- **FT Sensor Class**: `include/aristo_hardware_interface/ft_sensor_can.hpp`
- **FT Sensor Implementation**: `src/ft_sensor_can.cpp`

## References
- FT Sensor Test Tool: `src/tools/ft_sensor_can_test.cpp`
- Impedance Controller: `controllers/joint_impedance_controller`
- FT Sensor CAN Interface: `ft_sensor_can.hpp/cpp`
