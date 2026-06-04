# Aristo Quick Launch Guide

Run these from the workspace root:

```bash
cd ~/workspace/aristo_ws
source install/setup.bash
```

## 1. Start Hardware

Launch the Aristo hardware, robot description, controller manager, joint state
broadcaster, joint impedance controller, TF helpers, and RViz:

```bash
ros2 launch aristo_bringup aristo_hardware.launch.py
```

Zero fingers during hardware activation:

```bash
ros2 launch aristo_bringup aristo_hardware.launch.py zeroing:=true gui:=false
```

Use `zeroing:=true` only when the fingers are physically in the zero pose. No
recompile is needed for zeroing.


## 2. Choose One Control Stack

### Parallel Grasp

Terminal 2:

```bash
ros2 launch parallel_grasp_controller parallel_grasp_with_position_controller.launch.py
```

Terminal 3, if using Maestro fingertip distance input:

```bash
ros2 launch maestro_teleop grasp_teleop.launch.py
```

Input topic:

```text
/plato2/fingertip_dist_cmd
```

Direct grasp command topic:

```text
/plato2/parallel_grasp_controller/commands
```

### Joint Teleop / HY Float8 Input

Terminal 2:

```bash
ros2 launch joint_impedance_controller joint_impedance_trajectory_controller.launch.py
```

Terminal 3:

```bash
ros2 launch maestro_teleop joint_teleop.launch.py
```

Input topic:

```text
/plato2/joint_impedance_controller/commands_float8array_HY
```

## Notes

- Parallel grasp launches Naritouch and the object state estimator. Joint teleop
  and the joint trajectory launch do not launch tactile sensing.
- Do not use the old `grasp_node` or `/naritouch_grasp_demo/start_grasp`
  commands from the old branch.
- To verify hardware:

```bash
ros2 control list_controllers -c /plato2/controller_manager
ros2 topic echo /plato2/joint_states
```
