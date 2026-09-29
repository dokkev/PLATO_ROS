# Turntable Quick Launch Guide

Run these from the workspace root:

```bash
cd ~/workspace/aristo_ws
source install/setup.bash
```

## Launch

Launch the Dynamixel turntable wrapper:

```bash
ros2 launch turntable_hardware_interface turntable_dynamixel.launch.py
```

## Topics

State topics:

```text
/turntable_dynamixel_node/position
/turntable_dynamixel_node/velocity
```

Both state topics publish `std_msgs/msg/Float64`. Position is in radians, and
velocity is in radians per second.

Absolute desired position topic:

```text
/turntable_dynamixel_node/desired_position
```

Publish an absolute target in radians:

```bash
ros2 topic pub --once /turntable_dynamixel_node/desired_position std_msgs/msg/Float64 "{data: 1.57}"
```

## Relative Turn Services

```bash
ros2 service call /turntable_dynamixel_node/turn_90_cw std_srvs/srv/Trigger
ros2 service call /turntable_dynamixel_node/turn_90_ccw std_srvs/srv/Trigger
ros2 service call /turntable_dynamixel_node/turn_180_cw std_srvs/srv/Trigger
ros2 service call /turntable_dynamixel_node/turn_180_ccw std_srvs/srv/Trigger
```

CW is negative rotation, and CCW is positive rotation. New desired-position
commands or service calls are ignored while a turntable trajectory is already
active.

## Diagnostics

Scan for Dynamixel servos:

```bash
ros2 run turntable_hardware_interface turntable_dynamixel_scan
```
