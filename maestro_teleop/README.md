# maestro_teleop

ROS 2 teleoperation utilities for Maestro input devices.

This package is scaffolded as a C++ `ament_cmake` package.

## Fingertip Distance Relay

`fingertip_dist_to_parallel_grasp` subscribes to:

```text
/plato2/fingertip_dist_cmd std_msgs/msg/Float64MultiArray
```

Expected input layout:

```text
[t-i_dis, t-m_dis, T_IP, I_PIP, M_PIP]
```

It publishes:

```text
/plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray
[t-i_dis, I_PIP]
```

## HY Float8 To Joint Teleop

`float8array_to_impedance_trajectory` subscribes to:

```text
/plato2/joint_impedance_controller/commands_float8array_HY std_msgs/msg/Float64MultiArray
```

It republishes the 8-value position command to:

```text
/plato2/aristo_controller/joint_teleop std_msgs/msg/Float64MultiArray
```

The `plato_ros_controller` joint teleop state consumes this target.
