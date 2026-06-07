# Architecture

## Purpose

`PLATO_ROS` is a ROS 2 Humble workspace source tree for the PLATO/Aristo robot
hand stack. It contains message contracts, ros2_control hardware plugins,
joint-space controllers, bringup launch files, teleoperation helpers, object
state estimation, and a ROS-free MPPI core for contact-local grasp planning.

The repository is used for:

- Bringing up PLATO2 and Aristo hands through ros2_control.
- Translating joint-space controller commands into robot-specific CAN actuator
  commands.
- Running grasp, posture, impedance, teleoperation, and state-estimation nodes.
- Keeping hardware, controller, and planning contracts visible to humans and
  coding agents.

## Repository Shape

| Path | Role |
| --- | --- |
| `plato_interfaces/` | Shared ROS messages and services, including impedance commands and saved-position services. |
| `hardware_interface/can_hardware_common/` | Shared CAN transport, actuator buffers, frame execution, and generic robot data helpers. |
| `hardware_interface/plato_hardware_interface/` | PLATO ros2_control hardware plugin, GIM/Dynamixel CAN handling, five-bar linkage, actuator/linkage config loaders, and PLATO launch helpers. |
| `hardware_interface/aristo_hardware_interface/` | Aristo ros2_control hardware plugin, Aristo CAN protocol/model, actuator config, and hardware test tools. |
| `hardware_interface/turntable_hardware_interface/` | Dynamixel-based turntable support and minimum-jerk trajectory utility. |
| `controllers/joint_impedance_controller/` | Shared joint-space impedance controller plugin plus trajectory/impedance preset helper node. |
| `controllers/plato_grasp_controller/` | PLATO grasp task planner/node, saved joint-position storage, grasp plans, and pipeline tests. |
| `controllers/parallel_grasp_controller/` | Parallel grasp command node and command examples. |
| `controllers/adaptive_nail_controller/` | Force/torque feedback adaptive grasp node. |
| `controllers/posture_controller/` | Keyboard-driven posture selection node. |
| `bringup/plato_bringup/` | PLATO controller-manager YAML, robot-state publisher wiring, controller spawners, and RViz bringup. |
| `bringup/aristo_bringup/` | Aristo controller-manager YAML, robot-state publisher wiring, TF helpers, optional fake hardware, and RViz bringup. |
| `plato_description/` | URDF/Xacro, ros2_control Xacro, meshes, and RViz config for PLATO/Aristo. |
| `plato_utils/` | Shared C++ utilities: PID, interpolation, watchdog, YAML helpers, joint state ordering, and joint-position storage. |
| `plato_teleop/` | Python teleop nodes for SpaceMouse, grasp-task keyboard commands, and conversion helpers. |
| `maestro_teleop/` | C++ teleoperation conversion utilities for Maestro input streams. |
| `plato_state_estimator/` | Object/contact state estimator using tactile messages and PID force logic. |
| `mppi_core/` | ROS-free C++ MPPI optimizer, contact/tactile rollout, config parsing, and tests. |
| `plato_foxglove/` | Foxglove visualization scripts and configs. |
| `docs/` | Agent-readable repository harness, hardware runbooks, firmware notes, and images. |

## Runtime Flow

Typical hardware bringup flow:

```text
Launch file in bringup/*
-> xacro robot_description with hardware parameters
-> controller_manager ros2_control_node
-> hardware_interface plugin exports joint state/command interfaces
-> joint_state_broadcaster and joint_impedance_controller
-> command topics or controller nodes publish joint-space targets
-> hardware hand object maps joint commands to actuator commands
-> CAN frames are sent and actuator feedback refreshes joint state
```

Typical PLATO grasp stack flow:

```text
joint_states + motion/task topics
-> plato_grasp_controller_node
-> saved joint positions + grasp_plans.yaml + impedance presets
-> ImpedanceCommands goal or direct command
-> impedance_trajectory_controller_node, optional
-> joint_impedance_controller/commands
-> ros2_control hardware write path
```

MPPI flow:

```text
q_meas/qdot_meas/tau_meas + tactile_meas sensor vector
-> mppi_core qddot_des sampling, GraspState rollout, and stability cost
-> RobotCommand hybrid impedance packet with q_des/qdot_des/tau_ff/kp/kd
-> downstream controller or integration layer
```

## Core Boundaries

- `plato_interfaces` owns ROS message and service contracts. Changing a message
  affects every publisher, subscriber, test, and saved command example.
- Controllers should remain joint-space and hardware-agnostic. They must not
  know CAN IDs, actuator byte layout, five-bar details, or robot-specific
  transmission math.
- Hardware interface packages own robot-specific protocol, config parsing,
  actuator limits, zeroing, transmission, and CAN behavior.
- Bringup packages own ROS-facing launch and controller YAML. Fixed actuator
  and linkage constants belong in the hardware package config, not in bringup.
- `can_hardware_common` should stay generic. Do not add PLATO, Aristo, or
  turntable constants to the shared layer.
- `mppi_core` is intentionally ROS-free. Keep ROS message conversion and launch
  wiring outside it.
- `docs/ NAMING.md` is intentionally excluded from this harness pass; do not
  edit it unless a task explicitly targets naming.

## Key Interfaces

| Interface | Producer | Consumer | Notes |
| --- | --- | --- | --- |
| `plato_interfaces/msg/ImpedanceCommands` | Teleop, grasp controller, trajectory controller | `joint_impedance_controller` and helper nodes | Arrays must match the active joint count; PLATO/Aristo hand configs currently use 8 joints. |
| ros2_control state interfaces: `position`, `velocity`, `effort` | Hardware plugins | Controllers and broadcasters | Exported per joint by PLATO and Aristo hardware plugins. |
| ros2_control command interfaces: `position`, `velocity`, `effort`, `stiffness`, `damping` | Controllers | Hardware plugins | This is the shared controller/hardware contract. |
| `~/commands` on `joint_impedance_controller` | Command publishers | Controller plugin | Resolved under the controller namespace, commonly `/plato2/joint_impedance_controller/commands`. |
| `~/controller_state` on `joint_impedance_controller` | Controller plugin | Debugging/visualization | Publishes desired/actual/error vectors and effort terms. |
| `plato_interfaces/srv/SaveJointPosition` | Grasp/parallel grasp nodes | Teleop or CLI users | Saves named joint positions to YAML. |
| `sensor_msgs/msg/JointState` | Hardware and broadcaster path | Grasp, teleop, trajectory helper nodes | Joint ordering matters; use `plato_utils/joint_state_ordering` helpers where relevant. |
| SocketCAN/PCAN `can0` | CAN adapter and hardware interfaces | PLATO/Aristo/turntable low-level paths | Requires hardware setup and safe bus state before real hardware runs. |

## Safety-Critical Paths

These paths can affect physical hardware or experiments:

- `hardware_interface/**/src/**`
- `hardware_interface/**/config/**`
- `hardware_interface/**/launch/**`
- `plato_description/urdf/**`
- `bringup/**/config/**`
- `bringup/**/launch/**`
- `controllers/**/src/**`
- `controllers/**/config/**`
- `plato_interfaces/msg/**`
- `plato_interfaces/srv/**`
- `docs/frimware/plato2/**`

Safe validation path for risky changes:

1. Run the narrowest unit test for parser, protocol, math, or controller logic.
2. Build only the affected packages and direct dependencies.
3. Use fake hardware or launch argument dry runs where available.
4. For CAN or zeroing changes, inspect bus setup and start with low-risk
   commands, disabled actuators, or explicit operator supervision.
5. State clearly whether real hardware was not exercised.

## Known Limitations

- Several package manifests still contain placeholder descriptions/licenses.
- `docs/frimware/` is misspelled in the existing tree; keep the path stable
  unless doing an explicit migration.
- Hardware validation depends on local devices, CAN adapter setup, and ROS 2
  Humble environment state.
- Some runbooks are hardware-operator notes rather than mechanically tested
  scripts.
- Aristo and PLATO share the joint impedance controller contract, but their
  lower actuator semantics differ.

## Related Documents

- `docs/COMMANDS.md`
- `docs/DECISIONS.md`
- `docs/PLANS.md`
- `docs/motor/usbcan_setup.md`
- `docs/frimware/plato2/README.md`
- `installation.md`
- `cli_commands.md`
- `mppi_core/README.md`
- `mppi_core/docs/grasp_state_mppi.md`
- `mppi_core/docs/file_structure.md`
- `controllers/parallel_grasp_controller/COMMAND_EXAMPLES.md`
- `hardware_interface/plato_hardware_interface/plan.md`
