# Commands

Run `colcon` commands from the workspace root:

```bash
cd ~/workspace/plato_ws
```

This repository is the `src/PLATO_ROS` subtree of that workspace.

## Environment

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/plato_ws/install/setup.bash
```

Use the local Python environment expected by ROS 2 Humble. Install dependencies
with rosdep when the machine has network/package-manager access:

```bash
cd ~/workspace/plato_ws
rosdep install --from-paths src --ignore-src -r -y
```

Hardware work may also require SocketCAN/PCAN setup from
`docs/motor/usbcan_setup.md` and `installation.md`.

## Package Discovery

```bash
cd ~/workspace/plato_ws/src/PLATO_ROS
colcon list --names-only
```

## Build

Build the whole workspace with symlink install:

```bash
cd ~/workspace/plato_ws
colcon build --symlink-install
source install/setup.bash
```

Build only packages touched by a change:

```bash
cd ~/workspace/plato_ws
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```

Build a package and the packages that depend on it:

```bash
cd ~/workspace/plato_ws
colcon build --symlink-install --packages-up-to <package_name>
source install/setup.bash
```

Common focused builds:

```bash
cd ~/workspace/plato_ws
colcon build --symlink-install --packages-select plato_interfaces
colcon build --symlink-install --packages-up-to joint_impedance_controller
colcon build --symlink-install --packages-up-to plato_hardware_interface
colcon build --symlink-install --packages-up-to aristo_hardware_interface
colcon build --symlink-install --packages-up-to plato_grasp_controller
colcon build --symlink-install --packages-up-to mppi_core
```

## Test

Run all tests for a package:

```bash
cd ~/workspace/plato_ws
colcon test --packages-select <package_name>
colcon test-result --verbose
```

Focused test packages currently visible in CMake:

```bash
cd ~/workspace/plato_ws
colcon test --packages-select joint_impedance_controller
colcon test --packages-select plato_grasp_controller
colcon test --packages-select plato_hardware_interface
colcon test --packages-select aristo_hardware_interface
colcon test --packages-select turntable_hardware_interface
colcon test --packages-select mppi_core
colcon test-result --verbose
```

Useful package/test mapping:

| Package | Tests |
| --- | --- |
| `joint_impedance_controller` | `joint_impedance_trajectory_controller_test`, `impedance_handler_test`, launch/mock hardware test |
| `plato_grasp_controller` | `plato_grasp_controller_core_test`, `plato_grasp_controller_pipeline_test` |
| `plato_hardware_interface` | `plato_protocol_test` |
| `aristo_hardware_interface` | `aristo_protocol_test` |
| `turntable_hardware_interface` | `min_jerk_traj_test` |
| `mppi_core` | `test_mppi_core` |

## Lint Or Static Checks

Many packages declare `ament_lint_auto` and `ament_lint_common`; use package
tests first because lint availability depends on the local ROS environment.

```bash
cd ~/workspace/plato_ws
colcon test --packages-select <package_name> --event-handlers console_direct+
```

For quick documentation hygiene:

```bash
cd ~/workspace/plato_ws/src/PLATO_ROS
rg -n "[T]ODO|FIX[M]E|PLACE[H]OLDER" AGENTS.md docs --glob '*.md'
```

## Run

PLATO bringup:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch plato_bringup plato_hardware.launch.py rviz:=true zeroing:=false
```

Aristo bringup:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch aristo_bringup aristo_hardware.launch.py gui:=true fake_hardware:=false zeroing:=false
```

Aristo fake hardware:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch aristo_bringup aristo_hardware.launch.py gui:=false fake_hardware:=true
```

Joint impedance trajectory helper:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch joint_impedance_controller joint_impedance_trajectory_controller.launch.py hand_namespace:=plato2
```

PLATO grasp task stack:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch plato_grasp_controller grasp_task_stack.launch.py use_trajectory:=false
```

PLATO grasp keyboard:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch plato_teleop grasp_task_keyboard.launch.py
```

SpaceMouse teleop:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch plato_teleop spacemouse_twist.launch.py
ros2 launch plato_teleop spacemouse_state_machine.launch.py
```

Reference grasp force generator:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch plato_state_estimator object_state_estimator.launch.py
```

Turntable:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch turntable_hardware_interface turntable_dynamixel.launch.py
```

## Dry Run, Fake Mode, Or Safe Validation

Use these before real hardware when possible:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 launch aristo_bringup aristo_hardware.launch.py fake_hardware:=true gui:=false
```

Generate robot descriptions without starting hardware:

```bash
cd ~/workspace/plato_ws
source install/setup.bash
ros2 run xacro xacro src/PLATO_ROS/plato_description/urdf/plato.urdf.xacro zeroing:=false
ros2 run xacro xacro src/PLATO_ROS/plato_description/urdf/aristo.urdf.xacro fake_hardware:=true zeroing:=false
```

Check CAN interface state before hardware bringup:

```bash
ip link show can0
candump can0
```

Send only deliberate, bounded commands. Examples live in:

- `cli_commands.md`
- `controllers/parallel_grasp_controller/COMMAND_EXAMPLES.md`

## Debugging Commands

```bash
ros2 topic list
ros2 node list
ros2 control list_controllers -c /plato2/controller_manager
ros2 control list_hardware_interfaces -c /plato2/controller_manager
ros2 topic echo /plato2/joint_states
ros2 topic echo /plato2/joint_impedance_controller/controller_state
```


## Validation Policy

- Use the narrowest relevant check first.
- Do not claim a test, build, launch, simulation, dry run, CAN check, or hardware
  run was performed unless it was actually performed.
- If a command cannot be run, clearly state what was not verified.
- For risky changes, prefer fake hardware, log inspection, protocol unit tests,
  config parsing tests, xacro generation, or limited-scope hardware validation
  before full operation.

## Known Issues

- Hardware validation depends on local CAN devices, motor power, bus wiring, and
  operator safety setup.
- Some package manifests still have placeholder metadata; this does not
  necessarily block builds but should be cleaned when package ownership is clear.
- `docs/frimware/` is an existing misspelled path.
- `tactile_sensing` and `sdr_grasp_msgs` may come from sibling workspaces or
  external packages depending on the machine.
